
#define _GNU_SOURCE
#include "vThread_RF.h"
#include "rt_wrapper.h"
#include "nr-uesoftmodem.h"
#include "../../ARCH/COMMON/common_lib.h"
#include "vThreadIx_rfSrch.h"

#include <sched.h>
#include <string.h>

#define VRF_FREQOFFSET_STEPSIZE		100


/* virtual H/W for RF 
- Assuming that there is only one RF in the UE
- virtual H/W has one real RF device and has HAL code for controling this real HW
*/


/*     ------- RF HW related ------          */

//1. SDR RF interface
static openair0_device vrf_rfdevice; 	//rf device object
static openair0_config_t vrf_rfDevcfg[MAX_CARDS];
static uint8_t vrf_rfHwOn; 				//RF device on/off status
static double vrf_rxcFreq[4];			//current RX center frequency
static double vrf_txcFreq[4];			//current TX center frequency

//2. TX/RX chain vHW interface
static int32_t ***vrf_rxdata;
static int32_t ***vrf_txdata;




/* initial configuration parameters for SDR RF------ */
static unsigned int vrf_mmapped_dma;
static clock_source_t vrf_clksrc = internal;
static char vrf_config_file[1024];
static char* vrf_usrpArg = NULL;



/*     ------- HAL - HW interface related ------          */
const halRfic_woReg_t* vrf_HALix_in; 	//write only registers
halRfic_roReg_t* vrf_HALix_out;			//read only registers
halRfic_rwReg_t* vrf_HALix_buf;			//read/write registers
pthread_mutex_t* vrf_muPtr_rfHal;		//MUTEX for HAL interface
pthread_cond_t* vrf_csPtr_rfHal;		//condition signal for off -> on

static struct {
	int freq_offset; //delta
	int tx_sample_advance;
	int sync_offset;
	int rx_offset;
	uint8_t rxMode;
	uint8_t txMode;
} vrf_rtCfg; //RFIC HW abstraction


/*  ------- other variables used in this vHW  ---------          */
//frame parameters
static uint32_t vrf_samples_in_subframe;
static uint32_t vrf_samples_in_slot;
static uint16_t vrf_slots_per_subframe;
static int 		vrf_nb_slot_frame;



/* ---------- HW - HW interfaces -----------       */

/* 1. RF - SRCH */
static vrfSrch_rfReg_t			vrfsrch_rfReg;
vrfSrch_srchReg_t*				vrfsrch_srchRegPtr=NULL;
vrfSrch_sharedReg_t				vrfsrch_sharedReg;


/* -------- 1. Initializing function codes ----------- */


//interface configuration between RF and SRCH
vrfSrch_rfReg_t* vrf_configRfReg(void)
{
	return &vrfsrch_rfReg;
}


vrfSrch_sharedReg_t* vrf_configSrchSharedReg(void)
{
	return &vrfsrch_sharedReg;
}



//initiation function for virtual RF H/W called by API
void vrf_initHwParams(unsigned int Vmmapped_dmam, int VclkSrc, char* inConfigFile, char* inUsrpArg)
{
	vrf_mmapped_dma = Vmmapped_dmam;

	vrf_clksrc = (clock_source_t)VclkSrc;

	if (inConfigFile != NULL)
		memcpy(vrf_config_file, inConfigFile, 1024*sizeof(char));
	if (inUsrpArg != NULL)
	{
		vrf_usrpArg = malloc((strlen(inUsrpArg)+1)*sizeof(char));
		strcpy(vrf_usrpArg, inUsrpArg);
	}
	
}

//initializing the abstraction variable for USRP
//called within this vHW
static void vrf_init_rfDevCfg(void) {

    int card;
    int i;

    for (card=0; card<MAX_CARDS; card++)
	{
        vrf_rfDevcfg[card].mmapped_dma = vrf_mmapped_dma;
        vrf_rfDevcfg[card].configFilename = NULL;
  		vrf_rfDevcfg[card].duplex_mode = duplex_mode_FDD;
		vrf_rfDevcfg[card].Mod_id = 0;
	
		vrf_rfDevcfg[card].clock_source = vrf_clksrc;
		vrf_rfDevcfg[card].digital_freq_offset = 0;
	
		vrf_rfDevcfg[card].tx_num_channels = min(2,HW_NB_TXANT);
		vrf_rfDevcfg[card].rx_num_channels = min(2,HW_NB_RXANT);
	
		for (i=0; i<4; i++)
		{
    		vrf_rfDevcfg[card].tx_freq[i]=0.0;
    		vrf_rfDevcfg[card].rx_freq[i]=0.0;	  
	  		vrf_rfDevcfg[card].autocal[i] = 1;
	  		vrf_rfDevcfg[card].tx_gain[i] = 0.0;
	  		vrf_rfDevcfg[card].rx_gain[i] = 0.0;
		}
		
		vrf_rfDevcfg[card].configFilename = vrf_config_file;
		if (vrf_usrpArg)
		{
			vrf_rfDevcfg[card].sdr_addrs = vrf_usrpArg;
		}
    }
}

//configuration function for off-on procedures
static void vrf_configInit(void)
{
	//check other configurations
	int i;	

	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
	switch(vrf_HALix_in->samplingMode)
	{
		case VRF_SAMPLEMODE_192:
			vrf_rfDevcfg[0].sample_rate = 1.92e6;
			
			break;
		case VRF_SAMPLEMODE_768:
			vrf_rfDevcfg[0].sample_rate = 7.68e6;
			break;
		case VRF_SAMPLEMODE_1536:
			vrf_rfDevcfg[0].sample_rate = 15.36e6;
			break;
		case VRF_SAMPLEMODE_3072:
			vrf_rfDevcfg[0].sample_rate = 30.72e6;
			break;
		default:
			printf("[HW RF] WARNING : unknown sample rate %i (default 30.72e6 is set)\n", vrf_HALix_in->samplingMode);
			vrf_rfDevcfg[0].sample_rate = 30.72e6;
			break;
	}

	//other frame parameter definition
	vrf_samples_in_subframe = vrf_HALix_in->samples_per_frame/10;
	vrf_samples_in_slot = vrf_HALix_in->samples_per_slot;
	vrf_slots_per_subframe = vrf_samples_in_subframe/vrf_samples_in_slot;
	vrf_nb_slot_frame = HW_NB_SUBF_IN_FRAME * vrf_slots_per_subframe;
	
	for (i=0;i<HW_NB_TXANT;i++)
	{
		vrf_rfDevcfg[0].tx_freq[i] = vrf_HALix_in->tx_freq[i];
		vrf_txcFreq[i] = vrf_HALix_in->tx_freq[i];
	}
	for (i=0;i<HW_NB_RXANT;i++)
	{
		vrf_rfDevcfg[0].rx_freq[i] = vrf_HALix_in->rx_freq[i];
		vrf_rxcFreq[i] = vrf_HALix_in->rx_freq[i];
	}

	vrf_rfDevcfg[0].rx_bw = vrf_HALix_in->rx_bw;
	vrf_rfDevcfg[0].tx_bw = vrf_HALix_in->tx_bw;

	for (i=0;i<HW_NB_RXANT;i++)
	{
		vrf_rfDevcfg[0].rx_gain[i] = vrf_HALix_in->rx_gain[i];
		vrf_rfDevcfg[0].rx_gain_offset[i] = vrf_HALix_in->rx_gain_offset[i];
	}
	for (i=0;i<HW_NB_TXANT;i++)
	{
		vrf_rfDevcfg[0].tx_gain[i] = vrf_HALix_in->tx_gain[i];
	}
	vrf_HALix_buf->gainChanged = 0;

	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
	
	vrf_rtCfg.freq_offset = 0;
	vrf_rtCfg.sync_offset = 0;
	vrf_rtCfg.rx_offset = 0;
	vrf_rfDevcfg[0].digital_freq_offset = 0;
	vrf_HALix_out->digital_freq_offset = 0;
}









/* -------- 2. real-time processing codes ----------- */

//calculating rx offset drift decision based on rx_offset register
//inner function
int vrf_computeSamplesShift(void)
{
	
  	// compute TO compensation that should be applied for this frame
  	//HAL - HW compensation cmd first, then RF-SRCH compensation cmd later
	if (vrf_rtCfg.rx_offset > 0)
	{
		return -1;
	}
	else if (vrf_rtCfg.rx_offset < 0)
	{
		return 1;
	}
	else
	{
		//command from srcher
		AssertFatal( 0 == pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
		int tComandSrch = vrfsrch_sharedReg.timeOffset;
		AssertFatal( 0 == pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");
			
		if (tComandSrch > 0)
		{
			return -1;
		}
		else if (tComandSrch < 0)
		{
			return 1;
		}
	}
  
  	return 0;
}

//calculating the frequency offset input (into USRP)
//inner function 
static int vrf_calc_freqOffsetIn(int freq_offset)
{
	int truncFreq;

	truncFreq = (freq_offset/VRF_FREQOFFSET_STEPSIZE)*VRF_FREQOFFSET_STEPSIZE;
	return truncFreq;
}


//main function for processing the real-time register command of this virtual RF
//inner function
static uint8_t vrf_configRealTime(void)
{
	uint8_t flag_cfg = 0;
	int i;

	//reading register area and get values if changed
	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");
	// -- Frequency offset (AFC)
	if (vrf_rtCfg.freq_offset != vrf_HALix_in->freq_offset)
	{
		vrf_rtCfg.freq_offset = vrf_HALix_in->freq_offset;
		flag_cfg |= 0x01;
	}
	// -- TX timing
	if (vrf_rtCfg.tx_sample_advance!= vrf_HALix_in->tx_sample_advance)
	{
		vrf_rtCfg.tx_sample_advance = vrf_HALix_in->tx_sample_advance;
	}
	// -- RX offsets
	//jumping offset
	if (vrf_HALix_buf->sync_offset != 0)
	{
		vrf_rtCfg.sync_offset = vrf_HALix_buf->sync_offset;
		vrf_HALix_buf->sync_offset = 0;
	}
	//real time drift offset
	if (vrf_HALix_in->rx_offset != vrf_rtCfg.rx_offset)
	{
		vrf_rtCfg.rx_offset = vrf_HALix_in->rx_offset;
	}
	// -- Gains
	if (vrf_HALix_buf->gainChanged == 1)
	{
		for (i=0;i<HW_NB_RXANT;i++)
		{
			vrf_rfDevcfg[0].rx_gain[i] = vrf_HALix_in->rx_gain[i];
			vrf_rfDevcfg[0].rx_gain_offset[i] = vrf_HALix_in->rx_gain_offset[i];
		}
		for (i=0;i<HW_NB_TXANT;i++)
		{
			vrf_rfDevcfg[0].tx_gain[i] = vrf_HALix_in->tx_gain[i];
		}
		vrf_HALix_buf->gainChanged = 0;
		flag_cfg |= 0x02;
	}
	vrf_rtCfg.rxMode = vrf_HALix_in->rxMode;
	vrf_rtCfg.txMode = vrf_HALix_in->txMode;
  	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading RT config");


  	/* direct USRP setting */
	//frequency offset (center freq + drift offset) config on SDR RF
	if (vrf_rtCfg.rxMode == VRF_RXMODE_SRCH && (flag_cfg & 0x01) )
	{
		int inFreq = vrf_calc_freqOffsetIn(vrf_rtCfg.freq_offset);

		if (vrf_rfDevcfg[0].digital_freq_offset != inFreq)
		{
			vrf_rfDevcfg[0].digital_freq_offset = inFreq;
			vrf_rfdevice.adjust_rx_freq_func(&vrf_rfdevice, vrf_rfDevcfg);

			vrf_HALix_out->digital_freq_offset = vrf_rfDevcfg[0].digital_freq_offset; //output the Doffset to the HAL layer
		}
	}
	
	//gain setting
	if (flag_cfg & 0x02)
	{
		vrf_rfdevice.trx_set_gains_func(&vrf_rfdevice, vrf_rfDevcfg);
	}
	
	return flag_cfg;
}

//reading on/off register in real-time
//inner function
static uint8_t vrf_readOnOff(void)
{
	uint8_t onOff;
	
	AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading onOff");
	onOff = vrf_HALix_in->onoff;
	AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "error on RF HAL MUTEX while reading onOff");

	return onOff;
}








/* --------- 3. USRP trx function (all inner functions) -------- */
//throw away samples of one frame
void vrf_trashFrame(openair0_timestamp *timestamp)
{
	void *dummy_rx[HW_NB_RXANT];

	for (int i=0; i<HW_NB_RXANT; i++)
		dummy_rx[i]=malloc16(vrf_samples_in_subframe*4);

	for (int sf=0; sf<HW_NB_SUBF_IN_FRAME; sf++)
	{
		vrf_rfdevice.trx_read_func(&vrf_rfdevice,
		                           timestamp,
		                           dummy_rx,
		                           vrf_samples_in_subframe,
		                           HW_NB_RXANT);
	}

	for (int i=0; i<HW_NB_RXANT; i++)
		free(dummy_rx[i]);
	
}


//throw away samples till the next frame boundary
void vrf_syncInFrame(openair0_timestamp *timestamp, int sync_offset)
{
	//int sync_offset = vrf_rtCfg.sync_offset;
	
	printf("Resynchronizing RX by %d samples (samples in slot:%i)\n", sync_offset, vrf_samples_in_slot);

	//if (vrf_rtCfg.sync_offset < 0)
	if (sync_offset < 0)
	{
		sync_offset = sync_offset + vrf_samples_in_subframe * HW_NB_SUBF_IN_FRAME;
	}
	for ( int size = sync_offset ; size > 0 ; size -= vrf_samples_in_slot )
	{
  		int unitTransfer = size > vrf_samples_in_slot ? vrf_samples_in_slot : size ;
  		AssertFatal(unitTransfer == vrf_rfdevice.trx_read_func(&vrf_rfdevice,
						                                         timestamp,
						                                         (void **)vrf_rxdata[0],
						                                         unitTransfer,
						                                         HW_NB_RXANT),
					"");
	}
}

//read one slot samples
void vrf_readFrame(openair0_timestamp *timestamp)
{
	void *rxp[HW_NB_RXANT];

	for(int x=0; x<HW_NB_SUBF_IN_FRAME; x++)
	{
		for (int i=0; i<HW_NB_RXANT; i++)
	  		rxp[i] = ((void *)&vrf_rxdata[0][i][0]) + 4*x*vrf_samples_in_subframe;

 		AssertFatal( vrf_samples_in_subframe == vrf_rfdevice.trx_read_func(&vrf_rfdevice,
						                                        timestamp,
						                                        rxp,
						                                        vrf_samples_in_subframe,
						                                        HW_NB_RXANT), 
					"");

	}
}

int vrf_checkSrchStatus(int processId)
{
	int processStatus;
	AssertFatal ( 0== pthread_mutex_lock(&vrfsrch_srchRegPtr->roReg.regMutex), ""); //SYNC register mutex
	uint8_t srcherStatus = vrfsrch_srchRegPtr->roReg.hwStatus;
	AssertFatal ( 0== pthread_mutex_unlock(&vrfsrch_srchRegPtr->roReg.regMutex), ""); //SYNC register mutex
	
	if (srcherStatus != hwIx_on)
		return -1;

	AssertFatal( 0 == pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
	processStatus = (vrfsrch_sharedReg.processBitmap & (1<<processId));
	AssertFatal( 0 == pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");

	return (processStatus);
}


void vrf_irqSrch(int processId)
{
	AssertFatal( 0 == pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
	vrfsrch_sharedReg.processBitmap |= (1<<processId);
	AssertFatal( 0 == pthread_cond_signal(&vrfsrch_srchRegPtr->woReg.irq_srchInst), "");
	AssertFatal( 0 == pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");
}




void *vRFIC_mainThread(void *arg)
{
	PHY_VARS_NR_UE *UE;
	uint8_t onOffCfg = 0;
	openair0_timestamp timestamp;
	void *rxp[HW_NB_RXANT], *txp[HW_NB_TXANT];
	int i;
	char threadname[128];
	int th_id;
	const uint16_t table_sf_slot[2][20] = { {0,1,2,3,4,5,6,7,8,9,0,0,0,0,0,0,0,0,0,0},
											{0,0,1,1,2,2,3,3,4,4,5,5,6,6,7,7,8,8,9,9}
											};
	static uint8_t thread_idx = 0;
	int rx_offset_diff;



	vrf_rxdata = (int32_t***) vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int32_t**) );
	for (int i=0;i<VRF_NB_SAMPLEBANK;i++)
	{
		vrf_rxdata[i] = (int32_t**) vhw_malloc16_clear( HW_NB_RXANT * sizeof(int32_t*) );
		for (int j=0;j<HW_NB_RXANT;j++)
		{
			vrf_rxdata[i][j] = (int32_t*) vhw_malloc16_clear( VRF_NB_MAXSAMPLES * sizeof(int32_t) );
		}
	}

	vrf_txdata = (int32_t***) vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int32_t**) );
	for (int i=0;i<VRF_NB_SAMPLEBANK;i++)
	{
		vrf_txdata[i] = (int32_t**) vhw_malloc16_clear( HW_NB_RXANT * sizeof(int32_t*) );
		for (int j=0;j<HW_NB_RXANT;j++)
		{
			vrf_txdata[i][j] = (int32_t*) vhw_malloc16_clear( VRF_NB_MAXSAMPLES * sizeof(int32_t) );
		}
	}

	

	// -------------- HAL interface configuration -------------------
	vrf_HALix_in = &( ((ix_halRfic_t*)arg)->woReg  );
	vrf_HALix_out = &( ((ix_halRfic_t*)arg)->roReg );
	vrf_HALix_buf = &( ((ix_halRfic_t*)arg)->rwReg );
	vrf_muPtr_rfHal = (pthread_mutex_t*)&(((ix_halRfic_t*)arg)->mutex_rfHal);
	vrf_csPtr_rfHal = (pthread_cond_t*)&(((ix_halRfic_t*)arg)->cond_rfHal);

	vrf_HALix_out->hwStatus = rfst_null;
	UE = (PHY_VARS_NR_UE*)((ix_halRfic_t*)arg)->UE; //temporary code : will be erased if all the restructuring process is done
	vrf_HALix_out->ptr_rxdata = (const int32_t***) vrf_rxdata;
	UE->common_vars.rxdata = (int32_t**)vrf_rxdata[0];			//temporary code : should be blocked after synch. code resturucturing is done

	vrf_HALix_out->ptr_txdata = (const int32_t***) vrf_txdata;
	UE->common_vars.txdata = (int32_t**)vrf_txdata[0];			//temporary code : should be blocked after synch. code resturucturing is done


	//HW interface configuration
	// vs. Srcher
	vrfsrch_rfReg.roReg.rxData = (int***) vrf_rxdata;
	vrfsrch_rfReg.roReg.hfslot_nb = vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int16_t));
	vrfsrch_rfReg.roReg.hfframe_nb = vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int));
	vrfsrch_rfReg.roReg.slot_offset = vhw_malloc16_clear( VRF_NB_SAMPLEBANK * sizeof(int));
	vrfsrch_rfReg.roReg.hwStatus = hwIx_null;
	pthread_mutex_init(&(vrfsrch_rfReg.roReg.regMutex), NULL);

	vrfsrch_sharedReg.timeOffset = 0;
	vrfsrch_sharedReg.freqOffset = 0;
	vrfsrch_sharedReg.processBitmap= 0;
	pthread_mutex_init(&(vrfsrch_sharedReg.sharedMutex), NULL);

	AssertFatal ( NULL != (vrfsrch_srchRegPtr = vsrch_configSrchReg()), "[vHW][ERROR] error in configuration RF-SRCH register : pointer is NULL!\n");

	
	// --------------- thread setting -------------------
	//CPU affinity setting (deleted if it is not needed)
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);
	if ( threads.main != -1 )
		CPU_SET(threads.main, &cpuset);
	sprintf(threadname, "RF virtual HW thread on UE side");
	vhw_initvThread(100000, 500000, HW_FIFO_PRIORITY, &cpuset, threadname);

	
	int slot_nr=-1;
	int frame_nr = -1;

	//SDR RF initialization
	vrf_init_rfDevCfg();
	vrf_configInit();
	AssertFatal(0== openair0_device_load(&vrf_rfdevice, &vrf_rfDevcfg[0]), "");	

	vrf_rfdevice.host_type = RAU_HOST;
	vrf_HALix_out->hwStatus = rfst_off;
	AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
	vrfsrch_rfReg.roReg.hwStatus = hwIx_off;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");

	printf("[vHW] RF virtual HW is initialized, waiting for on signal\n");
	
	//outer loop for off waiting
	while (!oai_exit)
	{
		//off state loop (stay until on signal comes)
		AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "");
		while (vrf_HALix_in->onoff == 0)
		      pthread_cond_wait( vrf_csPtr_rfHal, vrf_muPtr_rfHal );
		AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "");

		
		//check on/off register and process it
		//if register is 'on', then do static configuration and apply to the SDR RF device (init and start)
		if (vrf_rfHwOn == 0)
		{
			time_stats_t onTime2;
			onTime2.p_time = rdtsc_oai();
			
		  	vrf_configInit();			
			
			vrf_rfdevice.trx_configure_on(&vrf_rfdevice, vrf_rfDevcfg);
			AssertFatal(vrf_rfdevice.trx_start_func(&vrf_rfdevice) == 0, "Could not start the RF device\n");

			AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "");
			vrf_HALix_out->hwStatus = rfst_on;
			AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "");
			AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
			vrfsrch_rfReg.roReg.hwStatus = hwIx_on;
			AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");
			vrf_rfHwOn = 1;

			onTime2.p_time = rdtsc_oai() - onTime2.p_time;
			printf("[vHW] >>>>>>>>>>>>>>>>> RF virtual HW is on! (carrier DL freq : %lf) - on time : %4d us\n", 
				vrf_rfDevcfg[0].rx_freq[0], (int)(onTime2.p_time/cpuf/1000.0));
		}

		
		//inner loop for on operation
		while ( !oai_exit && 
				 vrf_rfHwOn == 1 && 
				 (onOffCfg = vrf_readOnOff()) == 1 )
		{
		
			//read the real-time configure register and apply in real-time
			uint8_t flag_rtcfg = vrf_configRealTime();

			if (vrf_rtCfg.sync_offset != 0)
			{
				vrf_syncInFrame(&timestamp, vrf_rtCfg.sync_offset);
				vrf_rtCfg.sync_offset = 0;

				//force to initialize the timing compensation command from srcher (HAL cmd first!)
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
				vrfsrch_sharedReg.timeOffset = 0;
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");				
			}
			else if (vrf_rtCfg.rxMode == VRF_RXMODE_SRCH)
			{
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
				int timeOffset = vrfsrch_sharedReg.timeOffset;
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");	
				if (timeOffset != 0)
				{
					LOG_E(PHY, "[vRF] timing compensation command from srcher : %i\n", timeOffset);
					vrf_syncInFrame(&timestamp, timeOffset);
					AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
					vrfsrch_sharedReg.timeOffset = 0;
					AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");
				}
			}

			if (vrf_rtCfg.rxMode == VRF_RXMODE_SYNC)
			{
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
				int freq_offset_cmd = vrfsrch_sharedReg.freqOffset;
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");	

				if (freq_offset_cmd != 0)
				{
					int inFreq = vrf_calc_freqOffsetIn(vrf_rfDevcfg[0].digital_freq_offset+freq_offset_cmd);

					if (vrf_rfDevcfg[0].digital_freq_offset != inFreq)
					{
						//LOG_E(PHY, "[vRF] FO in online mode : %i\n", inFreq);
						
						vrf_rfDevcfg[0].digital_freq_offset = inFreq;
						vrf_rfdevice.adjust_rx_freq_func(&vrf_rfdevice, vrf_rfDevcfg);

						AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedReg.sharedMutex)), "");
						vrfsrch_sharedReg.freqOffset = 0; //output the Doffset to the HAL layer
						AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedReg.sharedMutex)), "");	
					}
				}
			}



			//mode 1 : not synchronized (maybe initial cell search case)
			//if (is_synchronized == 0)
			if (vrf_rtCfg.rxMode == VRF_RXMODE_SRCH)
			{
			    if (vrf_checkSrchStatus(0) == 0)// && instance_cnt_synch == 0)
				{  	// we can invoke the synch
			      	// grab 10 ms of signal and wakeup synch thread
					vrf_readFrame(&timestamp);
					vrf_irqSrch(0);
			    } 
				else
				{
		      		// grab 10 ms of signal into dummy buffer to wait result of sync detection
					vrf_trashFrame(&timestamp);
		    	}

		  		continue;
			}













			//mode 2-2 : synchronized, and not the first RX stream
			// 2-2-1 : multiple thread management for channel processings a(TXRX seems to be processed by 2 threads)
			thread_idx++;
			thread_idx %= HW_RX_NB_TH;
			slot_nr++;
			slot_nr %= vrf_nb_slot_frame;
			UE_nr_rxtx_proc_t *proc = &UE->proc.proc_rxtx[thread_idx];

			// update thread index for received subframe
			UE->current_thread_id[slot_nr] = thread_idx; //allocating a slot process toa specific thread thread_idx

			// 2-2-2 : timing calculation (subframe, frame, slot numbers, ...)
			//current timing setting
			proc->nr_tti_rx = slot_nr; //slot num indication
			proc->subframe_rx = table_sf_slot[UE->frame_parms.numerology_index][slot_nr]; //subframe index indication
			proc->frame_tx = proc->frame_rx; //frame num for TX
			proc->nr_tti_tx= slot_nr + HW_GAP_TXRX;

			//exception case handling (when TX timing is out of the frame boundary
			if (proc->nr_tti_tx > vrf_nb_slot_frame)
			{
				proc->frame_tx = (proc->frame_tx + 1)%HW_MAX_FRAMENUM;
				proc->nr_tti_tx %= vrf_nb_slot_frame;
			}

			//frame numbering (increment of 1 after subframe num goes back to 0)
			if(slot_nr == 0)
			{
				UE->proc.proc_rxtx[0].frame_rx++;

			  	for (th_id=1; th_id < RX_NB_TH; th_id++)
				{
			    	UE->proc.proc_rxtx[th_id].frame_rx = UE->proc.proc_rxtx[0].frame_rx;
			  	}
				frame_nr = UE->proc.proc_rxtx[0].frame_rx;
			}


			// 2-2-3 : RX buffering and TX symbol determination
			//pointer setting
			//shifted structure (first CP+symbol in the first subframe is received initially
			//for other slot, get second symbol ~ the first symbol at the next slot
			//last slot, get one less symbol, and then get one long CP symbol
	  		for (i=0; i<HW_NB_RXANT; i++)
	    		rxp[i] = (void *)&vrf_rxdata[0][i][slot_nr*vrf_samples_in_slot];
	  		for (i=0; i<HW_NB_TXANT; i++)
	    		//txp[i] = (void *)&UE->common_vars.txdata[i][((slot_nr+2)%HW_NB_SUBF_IN_FRAME)*vrf_samples_in_slot];
	    		txp[i] = (void *)&vrf_txdata[0][i][((slot_nr+2)%HW_NB_SUBF_IN_FRAME)*vrf_samples_in_slot];


			//read/write size determination (based on slot number)
	  		int readBlockSize, writeBlockSize;
		    readBlockSize = vrf_samples_in_slot;
			writeBlockSize = vrf_samples_in_slot;
			if (slot_nr == (vrf_nb_slot_frame - 1) ) //??
			{
			    rx_offset_diff = vrf_computeSamplesShift();
			    readBlockSize -= rx_offset_diff;
			    writeBlockSize -= rx_offset_diff;
	  		}
			// 2-2-4 : RX/TX buffering
	  		AssertFatal(readBlockSize ==
	              		vrf_rfdevice.trx_read_func(&vrf_rfdevice,
			                                         &timestamp,
			                                         rxp,
			                                         readBlockSize,
			                                         HW_NB_RXANT),"");
			if (vrf_rtCfg.txMode == VRF_TXMODE_INSYNC)
			{
	  			AssertFatal( writeBlockSize ==
	               		 vrf_rfdevice.trx_write_func(&vrf_rfdevice,
									                   timestamp+
									                   2*vrf_samples_in_slot-
									                   //(UE->frame_parms.ofdm_symbol_size - UE->frame_parms.nb_prefix_samples0) -
									                   vrf_rfDevcfg[0].tx_sample_advance + vrf_rtCfg.tx_sample_advance,
									                   txp,
									                   writeBlockSize,
									                   HW_NB_TXANT,
									                   1),"");
			}

			// 2-2-5 : passing to the TXRX process
			// operate on thread sf mod 2

			AssertFatal(pthread_mutex_lock(&proc->mutex_rxtx) ==0,"");

			//setting tx timing
			proc->subframe_tx=proc->nr_tti_rx;
			proc->timestamp_tx = timestamp +
			                   (HW_GAP_TXRX*vrf_samples_in_slot);
			proc->instance_cnt_rxtx++;


			//monitoring the TXRX process activity
			if (proc->instance_cnt_rxtx != 0)
			{
			    AssertFatal( proc->instance_cnt_rxtx <= 4, "[SCHED][UE %d] !!! UE instance_cnt_rxtx > 2 (IC %d) (Proc %d)!!",
											                 UE->Mod_id, proc->instance_cnt_rxtx,
											                 UE->current_thread_id[slot_nr]);
	  		}
			AssertFatal (pthread_cond_signal(&proc->cond_rxtx) ==0,"");
			AssertFatal (pthread_mutex_unlock(&proc->mutex_rxtx) ==0,"");

			//interrupt srcher for online synch
			if (vrf_checkSrchStatus(0) == 0)
			{  	
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
				vrfsrch_rfReg.roReg.hfslot_nb[0] = slot_nr%(vrf_nb_slot_frame/2);					//half frame slot index
				vrfsrch_rfReg.roReg.hfframe_nb[0] = (frame_nr<<1) + slot_nr/(vrf_nb_slot_frame/2);	//half frame count (SFN *2 + hf)
				vrfsrch_rfReg.roReg.slot_offset[0] = slot_nr*vrf_samples_in_slot;
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");
				vrf_irqSrch(0);
			}
		} //HW on loop

		if (vrf_rfHwOn == 1)
		{
			AssertFatal(vrf_rfdevice.trx_stop_func(&vrf_rfdevice) == 0, "Could not stop the RF device\n");
			AssertFatal ( 0== pthread_mutex_lock(vrf_muPtr_rfHal), "");
			vrf_HALix_out->hwStatus = rfst_off;
			AssertFatal ( 0== pthread_mutex_unlock(vrf_muPtr_rfHal), "");
			AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfReg.roReg.regMutex)), "");
			vrfsrch_rfReg.roReg.hwStatus = hwIx_off;
			AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfReg.roReg.regMutex)), "");
			vrf_rfHwOn = 0;

			printf("[vHW] RF virtual HW is off!\n");
		}
	}// while !oai_exit

	if (vrf_rfdevice.trx_end_func)
		  vrf_rfdevice.trx_end_func(&vrf_rfdevice);

	return NULL;
}
