#include "NHAL_rf.h"
#include "vThread_RF.h"
#include "rt_wrapper.h"


//initial values for HW creation (u=0, 20MHz, 2048 FFT size, 1 slot = 1 subframe) 
//-> there is a bug on gNB side that only 30704000 samples are transmitted in a frame. After this bug is fixed, 30704 should be changed to 30720
#define NHAL_INITV_VRFSAMPLEMODE	VRF_SAMPLEMODE_3072
#define NHAL_INITV_SAMPLESPERFRAME	307040
#define NHAL_INITV_SAMPLESPERSLOT	30704	

#define NHAL_INITV_RXFREQ			3510000000.0
#define NHAL_INITV_TXFREQ			3510000000.0

#define NHAL_INITV_RXBW				20000000.0
#define NHAL_INITV_TXBW				20000000.0


#define NHAL_WATCHDOG_RFONOFF		100
#define NHAL_WATCHDOG_RFINIT		5000


static pthread_t nhal_vThread_rf;
static pthread_attr_t nhal_vthreadattr_rf;
static ix_halRfic_t *nhal_rfic_ix;


long long nhal_wdog_rfonoff;

typedef enum {
	nhal_rfNull=0,
	nhal_rfOff,
	nhal_rfOn_going,
	nhal_rfOn,
	nhal_rfOff_going
} nhal_rfStatus_e;

struct {
	nhal_rfStatus_e status;
	nhal_sampleRate_e sr;
	uint32_t samples_per_frame;
	uint32_t samples_per_slot;
	double rxFreq;
	double txFreq;
	double rx_bw;
	double tx_bw;
	double rx_gain; 
	double rx_gain_offset;
	double tx_gain;
	nhal_syncState_e syncState;
	int rx_offset;
	int tx_sample_advance;
	
} nhal_rfInstance;


void NHAL_initAbstract(void)
{
	nhal_rfInstance.sr = 0;
	nhal_rfInstance.samples_per_frame = 0;
	nhal_rfInstance.samples_per_slot = 0;
	nhal_rfInstance.rxFreq = 0.0;
	nhal_rfInstance.txFreq = 0.0;
	nhal_rfInstance.rx_bw = 0.0;
	nhal_rfInstance.tx_bw = 0.0;
	nhal_rfInstance.rx_gain = 0.0; 
	nhal_rfInstance.rx_gain_offset = 0.0;
	nhal_rfInstance.tx_gain = 0.0;

	nhal_rfInstance.syncState = nhal_syncState_outsync;
	nhal_rfInstance.rx_offset = 0;
	nhal_rfInstance.tx_sample_advance = 0;
}










int NHAL_RFcmd_init(void* UE, int mmapped_dma, int clock_source, char* configFilename, char* sdr_addrs)
{
	int i;

	nhal_rfInstance.status = nhal_rfNull;

	
	nhal_rfic_ix = (ix_halRfic_t*)malloc(sizeof(ix_halRfic_t));
	if (nhal_rfic_ix == NULL)
	{
		LOG_E(PHY, "[ERROR] Failed to make an instance for virtual H/W RF\n");
		return -1;
	}
	
	pthread_attr_init (&nhal_vthreadattr_rf);
	pthread_attr_setstacksize(&nhal_vthreadattr_rf, 8192);//5*PTHREAD_STACK_MIN);

	//RFIC vHW initial setting ---------------------------------------------------
	//set as default value
	nhal_rfic_ix->woReg.freq_offset = 0;
	nhal_rfic_ix->woReg.onoff = 0;

	nhal_rfic_ix->woReg.samplingMode = NHAL_INITV_VRFSAMPLEMODE;
	nhal_rfic_ix->woReg.samples_per_frame = NHAL_INITV_SAMPLESPERFRAME;
	nhal_rfic_ix->woReg.samples_per_slot = NHAL_INITV_SAMPLESPERSLOT;

	for (i=0;i<4;i++)
	{
		if (i == 0)
		{
			nhal_rfic_ix->woReg.rx_freq[i] = NHAL_INITV_RXFREQ;
			nhal_rfic_ix->woReg.tx_freq[i] = NHAL_INITV_TXFREQ;
		}
		else
		{
			nhal_rfic_ix->woReg.rx_freq[i] = 0;
			nhal_rfic_ix->woReg.tx_freq[i] = 0;
		}
	}
	nhal_rfic_ix->woReg.rx_bw = NHAL_INITV_RXBW;
	nhal_rfic_ix->woReg.tx_bw = NHAL_INITV_TXBW;
	for (i=0;i<4;i++)
	{
		nhal_rfic_ix->woReg.rx_gain[i] = 0;
		nhal_rfic_ix->woReg.rx_gain_offset[i] = 0;
		nhal_rfic_ix->woReg.tx_gain[i] = 0;
	}
	nhal_rfic_ix->woReg.rxMode = VRF_RXMODE_SRCH;
	nhal_rfic_ix->woReg.txMode = VRF_TXMODE_INSYNC;
	nhal_rfic_ix->woReg.rx_offset = 0;
	nhal_rfic_ix->woReg.tx_sample_advance = 0;

	nhal_rfic_ix->rwReg.gainChanged = 0;
	nhal_rfic_ix->rwReg.sync_offset = 0;


	pthread_mutex_init(&(nhal_rfic_ix->mutex_rfHal), NULL);
	pthread_cond_init(&(nhal_rfic_ix->cond_rfHal), NULL);
	
	nhal_rfic_ix->UE = UE; //temporary code : should be deleted after synch code resturcturing
	vrf_initHwParams(mmapped_dma, clock_source, configFilename, sdr_addrs);

	LOG_I(PHY,"Intializing RF virtual thread...\n");
	
	AssertFatal(0 == pthread_create(&nhal_vThread_rf,
	                            &nhal_vthreadattr_rf,
	                            vRFIC_mainThread,
	                            (void*)nhal_rfic_ix), "");

	NHAL_initAbstract();
	nhal_wdog_rfonoff = rdtsc_oai();
	//wait for status
	while (nhal_rfic_ix->roReg.hwStatus != rfst_off)
	{
		usleep(500000);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rfonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_RFINIT, "virtual RF on off timeout!! (hwStatus : %i)\n", nhal_rfic_ix->roReg.hwStatus);
	}


	
	nhal_rfInstance.status = nhal_rfOff;


	return 0;
}


nhal_sampleRate_e NHAL_convSampleRate(double rate)
{
	if ((int)rate == 30720000)
	{
		return nhal_sampleRate_3072;
	}
	if ((int)rate == 15360000)
	{
		return nhal_sampleRate_1536;
	}
	if ((int)rate == 7680000)
	{
		return nhal_sampleRate_768;
	}
	if ((int)rate == 1920000)
	{
		return nhal_sampleRate_192;
	}

	LOG_E(PHY, "[WARNING] input rate cannot be recognized (%lf), returning 30.72e6\n", rate);
	
	return nhal_sampleRate_3072;
		
}




int NHAL_RFcmd_on(nhal_sampleRate_e sr, 
						uint32_t samples_per_frame, uint32_t samples_per_slot, 
						double rxFreq, double txFreq,
						double rx_bw, double tx_bw,
						double rx_gain, double rx_gain_offset, double tx_gain,
						nhal_syncState_e syncState
						)
{
	//validity check (state)
	if (nhal_rfInstance.status == nhal_rfNull ||
		nhal_rfInstance.status == nhal_rfOn ||
		nhal_rfInstance.status == nhal_rfOn_going)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept on command : RF is already on! (%i)\n", nhal_rfInstance.status);
		return -1;
	}

	nhal_rfInstance.status = nhal_rfOn_going;	
	LOG_I(PHY, "[HALcmd] RF on (sampling rate : %i, samples per frame/slot : %i, %i, freq : (%lf / %lf), BW : (%lf, %lf), gain : %lf %lf %lf, mode:%i\n",
		sr, samples_per_frame, samples_per_slot, rxFreq, txFreq, rx_bw, tx_bw, rx_gain, rx_gain_offset, tx_gain, syncState);


	//1. wait for HW status to be off ---------------------------------------------
	nhal_wdog_rfonoff = rdtsc_oai();
	//wait for status
	while (nhal_rfic_ix->roReg.hwStatus != rfst_off)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rfonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_RFONOFF, "virtual RF initialization timeout!! (hwStatus : %i)\n", nhal_rfic_ix->roReg.hwStatus);
	}

	//2. RFIC vHW configuration ---------------------------------------------------
	switch(sr)
	{
		case nhal_sampleRate_192:
			nhal_rfic_ix->woReg.samplingMode = VRF_SAMPLEMODE_192;
			break;
		case nhal_sampleRate_768:
			nhal_rfic_ix->woReg.samplingMode = VRF_SAMPLEMODE_768;
			break;
		case nhal_sampleRate_1536:
			nhal_rfic_ix->woReg.samplingMode = VRF_SAMPLEMODE_1536;
			break;
		case nhal_sampleRate_3072:
			nhal_rfic_ix->woReg.samplingMode = VRF_SAMPLEMODE_3072;
			break;
		default:
			LOG_E(PHY, "[ERROR] configuration fail : inavlid sample rate (%i)\n", sr);
			return -1;
			break;
	}
	
	nhal_rfic_ix->woReg.samples_per_frame = samples_per_frame;
	nhal_rfic_ix->woReg.samples_per_slot = samples_per_slot;

	nhal_rfic_ix->woReg.rx_freq[0] = rxFreq;
	nhal_rfic_ix->woReg.tx_freq[0] = txFreq;	

	nhal_rfic_ix->woReg.rx_bw = rx_bw;
	nhal_rfic_ix->woReg.tx_bw = tx_bw;
	
	nhal_rfic_ix->woReg.rx_gain[0] = rx_gain;
	nhal_rfic_ix->woReg.rx_gain_offset[0] = rx_gain_offset;
	nhal_rfic_ix->woReg.tx_gain[0] = tx_gain;

	if (syncState == nhal_syncState_insync)
		nhal_rfic_ix->woReg.rxMode = VRF_RXMODE_SYNC;
	else
		nhal_rfic_ix->woReg.rxMode = VRF_RXMODE_SRCH;
	nhal_rfic_ix->woReg.txMode = VRF_TXMODE_INSYNC;
	nhal_rfic_ix->woReg.rx_offset = 0;
	nhal_rfic_ix->woReg.freq_offset = 0;	
	nhal_rfic_ix->rwReg.gainChanged = 0;
	nhal_rfic_ix->rwReg.sync_offset = 0;

	
	nhal_rfic_ix->woReg.onoff = 1;
	pthread_cond_signal(&(nhal_rfic_ix->cond_rfHal));

	nhal_wdog_rfonoff = rdtsc_oai();	
	while (nhal_rfic_ix->roReg.hwStatus != rfst_on)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rfonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_RFONOFF, "virtual RF on/off timeout!!\n");
	}


	nhal_rfInstance.sr = sr;
	nhal_rfInstance.samples_per_frame = samples_per_frame;
	nhal_rfInstance.samples_per_slot = samples_per_slot;
	nhal_rfInstance.rxFreq = rxFreq;
	nhal_rfInstance.txFreq = txFreq;
	nhal_rfInstance.rx_bw = rx_bw;
	nhal_rfInstance.tx_bw = tx_bw;
	nhal_rfInstance.rx_gain = rx_gain; 
	nhal_rfInstance.rx_gain_offset = rx_gain_offset;
	nhal_rfInstance.tx_gain = tx_gain;
	nhal_rfInstance.syncState = nhal_syncState_outsync;
	nhal_rfInstance.rx_offset = 0;
	nhal_rfInstance.tx_sample_advance = 0;

	nhal_rfInstance.status = nhal_rfOn;
	
	return 0;
	
}


int NHAL_RFcmd_off(void)
{
	//validity check (state)
	if (nhal_rfInstance.status == nhal_rfNull ||
		nhal_rfInstance.status == nhal_rfOff ||
		nhal_rfInstance.status == nhal_rfOff_going)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept on command : RF is already on! (%i)\n", nhal_rfInstance.status);
		return -1;
	}

	nhal_rfInstance.status = nhal_rfOff_going;
	LOG_I(PHY, "[HALcmd] RF off\n");

	//1. wait for HW status to be on ---------------------------------------------
	nhal_wdog_rfonoff = rdtsc_oai();
	//wait for status
	while (nhal_rfic_ix->roReg.hwStatus != rfst_on)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rfonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_RFONOFF, "virtual RF on/off timeout!!\n");
	}
	

	//2. set the register to be off!
	nhal_rfic_ix->woReg.onoff = 0;

	//3. wait for HW status to be off ---------------------------------------------
	nhal_wdog_rfonoff = rdtsc_oai();
	//wait for status
	while (nhal_rfic_ix->roReg.hwStatus != rfst_off)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_rfonoff)/cpuf/1000000.0) < NHAL_WATCHDOG_RFONOFF, "virtual RF on/off timeout!!\n");
	}

	NHAL_initAbstract();
	nhal_rfInstance.status = nhal_rfOff;
	
	return 0;
}

int NHAL_RFcmd_cfgRxMode(nhal_syncState_e state)
{
	//validity check (state)
	if (nhal_rfInstance.status != nhal_rfOff &&
		nhal_rfInstance.status != nhal_rfOn)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept cfgRxMode command : RF is invalid state! (%i)\n", nhal_rfInstance.status);
		return -1;
	}

	if (state == nhal_syncState_insync)
		nhal_rfic_ix->woReg.rxMode = VRF_RXMODE_SYNC;
	else if (state == nhal_syncState_outsync)
		nhal_rfic_ix->woReg.rxMode = VRF_RXMODE_SRCH;
	else
		return -1;
	LOG_I(PHY, "RF RX mode set to %i\n", nhal_rfic_ix->woReg.rxMode);

	nhal_rfInstance.syncState = state;

	return 0;
}

int NHAL_RFcmd_cfgRxSync(int sync_offset)
{
	//validity check (state)
	if (nhal_rfInstance.status != nhal_rfOn)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept cfgRxSync command : RF is invalid state!(%i)\n", nhal_rfInstance.status);
		return -1;
	}


	nhal_rfic_ix->rwReg.sync_offset = sync_offset;
	LOG_I(PHY, "RF RX sync set to %i\n", nhal_rfic_ix->rwReg.sync_offset);

	return 0;
}

int NHAL_RFcmd_cfgRxDrift(int rx_offset)
{
	//validity check (state)
	if (nhal_rfInstance.status != nhal_rfOn)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept cfgRxSync command : RF is invalid state!(%i)\n", nhal_rfInstance.status);
		return -1;
	}

	
	nhal_rfic_ix->woReg.rx_offset = rx_offset;
	//LOG_I(PHY, "RF RX drift set to %i\n", nhal_rfic_ix->woReg.rx_offset);
	
	nhal_rfInstance.rx_offset = rx_offset;

	return 0;
}


int NHAL_RFcmd_clrRxDrift(void)
{
	//validity check (state)
	if (nhal_rfInstance.status != nhal_rfOn)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept cfgRxSync command : RF is invalid state!(%i)\n", nhal_rfInstance.status);
		return -1;
	}

	nhal_rfic_ix->woReg.rx_offset = 0;
	nhal_rfInstance.rx_offset = 0;

	return 0;
}


int NHAL_RFcmd_cfgTxAdvance(int tx_sample_advance)
{
	//validity check (state)
	if (nhal_rfInstance.status != nhal_rfOn)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept cfgRxSync command : RF is invalid state!(%i)\n", nhal_rfInstance.status);
		return -1;
	}

	
	nhal_rfic_ix->woReg.tx_sample_advance = tx_sample_advance;

	nhal_rfInstance.tx_sample_advance = tx_sample_advance;
	//LOG_I(PHY, "RF TX sample advance set to %i\n", nhal_rfic_ix->woReg.tx_sample_advance);

	return 0;
}


int NHAL_RFcmd_cfgFreqOffset(int freq_offset, nhal_freqOff_e option)
{
	//validity check (state)
	if (nhal_rfInstance.status != nhal_rfOn)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept cfgRxSync command : RF is invalid state!(%i)\n", nhal_rfInstance.status);
		return -1;
	}

	if (option == nhal_freqOff_abs)
	{
		nhal_rfic_ix->woReg.freq_offset = freq_offset;
	}
	else if (option == nhal_freqOff_delta)
	{
		//nhal_rfic_ix->woReg.freq_offset += freq_offset;
		LOG_E(PHY, "[NHAL] Failed to config! we currently don't support frequency offset register in delta mode\n");
		return -1;
	}
	//LOG_I(PHY, "RF frequency offset set to %i\n", nhal_rfic_ix->woReg.freq_offset);
	
	return 0;
}


int NHAL_RFcmd_readDigitalFreqOffset(void)
{
	return nhal_rfic_ix->roReg.digital_freq_offset;
}





//this function is under construction (to be implemented when TPC is needed)
int NHAL_RFcmd_cfgTxPwr(double delta)
{
	//validity check (state)
	if (nhal_rfInstance.status != nhal_rfOn)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept cfgRxSync command : RF is invalid state!(%i)\n", nhal_rfInstance.status);
		return -1;
	}

	//TBD
	#if 0	
	nhal_rfic_ix->woReg.tx_gain[0] += delta;
	nhal_rfic_ix.rwReg.gainChanged = 1;
	#endif
	
	return 0;
}
