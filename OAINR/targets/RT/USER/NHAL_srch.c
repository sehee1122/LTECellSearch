#include "NHAL_srch.h"
#include "vThread_srch.h"
#include "rt_wrapper.h"


#define NHAL_WATCHDOG_SRCHINIT		3000
#define NHAL_WATCHDOG_SRCHONOFF		200

static pthread_t nhal_vThread_srch;
static pthread_attr_t nhal_vthreadattr_srch;
static ix_halSrch_t *nhal_srch_ix[VSRCH_MAXNB_SRCHBNK];


static long long nhal_wdog_onoff[VSRCH_MAXNB_SRCHBNK];
static srchReport_t* nhal_srchRes_load[VSRCH_MAXNB_SRCHBNK];

typedef enum {
	nhal_Null=0,
	nhal_Off,
	nhal_On_going,
	nhal_On,
	nhal_Idle,
	nhal_Off_going
} nhal_hwStatus_e;

static struct {
	nhal_hwStatus_e status;

	//non real-time configurations
	//halSrch_srchMode_e mode;		//full / online
	nhal_srchmode_e mode;
	uint32_t narfcn;				//center frequency
	uint8_t numerology;				//u value
	uint16_t fftSize;				//bandwidth (in terms of FFT Size)
	uint16_t duration; 				//in ms, if 0 then search till something is found
	uint32_t srchSubframe;			//number of subframes to be searched (0 : no constraint)
	uint32_t ssb_Foffset;			//subcarrier index that the SSB starts
	int high_speed_flag;			//whether it is high speed or not
	uint16_t ssb_periodicity;		//unit ; half-frame

	
	uint8_t option_decimation;		//0 : none, 1 : enable decimation (not yet implemented)
	uint8_t option_pbchCheck;		//0 :  none, 1 : sync. detection includes pbch CRC check
	int16_t target_nid;				//target NID to track (-1 : search for any)
	int16_t threshold;				//threashold for seaching (0 : hw-guided threshold)

	//real-time configurations
	uint32_t search_window;			//0 : full search (unit : sample)
	int32_t search_offset;			///search starting point by sample

	uint8_t issb;					//tracking SSB number (online mode only)
	uint8_t nhf;					//tracking SSB number (online mode only)

	//read only
	srchHwStatus_e hwStatus;
	uint8_t srch_cnt;
	srchReport_t* srch_result;

	
} nhal_srchInstance[VSRCH_MAXNB_SRCHBNK];


static void NHAL_initAbstract(uint8_t bank)
{
	srchReport_t* ptr = nhal_srchInstance[bank].srch_result;
	srchReport_t* nextPtr;
	
	nhal_srchInstance[bank].mode = srchMode_full;
	nhal_srchInstance[bank].narfcn = 640000;
	nhal_srchInstance[bank].numerology = 0;
	nhal_srchInstance[bank].fftSize = 2048;
	nhal_srchInstance[bank].duration = 0;
	nhal_srchInstance[bank].srchSubframe = 0;
	nhal_srchInstance[bank].ssb_Foffset = 0;
	nhal_srchInstance[bank].high_speed_flag = 0;
	nhal_srchInstance[bank].ssb_periodicity = 0;
	nhal_srchInstance[bank].option_decimation = 0;
	nhal_srchInstance[bank].option_pbchCheck = 1;
	nhal_srchInstance[bank].target_nid = -1;
	nhal_srchInstance[bank].threshold = 0;

	nhal_srchInstance[bank].search_window = 0;
	nhal_srchInstance[bank].search_offset = 0;
	nhal_srchInstance[bank].issb = 0;
	nhal_srchInstance[bank].nhf = 0;

	nhal_srchInstance[bank].hwStatus = srchst_null;
	nhal_srchInstance[bank].srch_cnt = 0;

	while (ptr != NULL)
	{
		nextPtr = ptr->next;
		free(ptr);
		ptr = nextPtr;
	}
	nhal_srchInstance[bank].srch_result = NULL;
	nhal_srchRes_load[bank] = NULL;
}


int NHAL_srchcmd_init(void)
{
	for (int i=0;i<VSRCH_MAXNB_SRCHBNK;i++)
	{
		nhal_srch_ix[i] = (ix_halSrch_t*)malloc(sizeof(ix_halSrch_t));
		if (nhal_srch_ix[i] == NULL)
		{
			LOG_E(PHY, "[ERROR] Failed to make an instance for virtual H/W SRCH interface\n");
			return -1;
		}
		
		pthread_attr_init (&nhal_vthreadattr_srch);
		pthread_attr_setstacksize(&nhal_vthreadattr_srch, 8192);//5*PTHREAD_STACK_MIN);

		//RFIC vHW initial setting ---------------------------------------------------
		//set as default value

		nhal_srch_ix[i]->woReg.mode = srchMode_full;
		nhal_srch_ix[i]->woReg.narfcn = 640000; 		 //3.6GHz
		nhal_srch_ix[i]->woReg.numerology = 0;
		nhal_srch_ix[i]->woReg.fftSize = 2048;
		nhal_srch_ix[i]->woReg.duration = 0;

		nhal_srch_ix[i]->woReg.srchSubframe = 0;
		nhal_srch_ix[i]->woReg.ssb_Foffset = 0;
		nhal_srch_ix[i]->woReg.ssb_periodicity = 0;
		
		nhal_srch_ix[i]->woReg.high_speed_flag = 0;
		nhal_srch_ix[i]->woReg.ssb_periodicity = 0;
		
		nhal_srch_ix[i]->woReg.option_decimation = 0;
		nhal_srch_ix[i]->woReg.option_pbchCheck = 1;
		
		nhal_srch_ix[i]->woReg.target_nid = -1;
		nhal_srch_ix[i]->woReg.threshold = 0;
		nhal_srch_ix[i]->woReg.search_window = 0;
		nhal_srch_ix[i]->woReg.search_offset = 0;
		nhal_srch_ix[i]->woReg.issb = 0;
		nhal_srch_ix[i]->woReg.nhf = 0;
		

		nhal_srch_ix[i]->rwReg.onoff = 0;

		nhal_srch_ix[i]->roReg.hwStatus = srchst_null;
		nhal_srch_ix[i]->roReg.srch_cnt = 0;
		nhal_srch_ix[i]->roReg.srch_result = NULL;
		

		pthread_mutex_init(&(nhal_srch_ix[i]->mutex_srchHal), NULL);
		pthread_cond_init(&(nhal_srch_ix[i]->cond_srchIrq), NULL);
		pthread_cond_init(&(nhal_srch_ix[i]->cond_srchHal), NULL);
		
		LOG_I(PHY,"Intializing SRCH virtual thread... \n");
		
		AssertFatal(0 == pthread_create(&nhal_vThread_srch,
									&nhal_vthreadattr_srch,
									vSRCH_mainThread,
									(void*)nhal_srch_ix[i]), "");

		NHAL_initAbstract(i);
		
		nhal_wdog_onoff[i] = rdtsc_oai();
		//wait for status
		while (nhal_srch_ix[i]->roReg.hwStatus != srchst_off)
		{
			usleep(500000);
			AssertFatal( (int)((rdtsc_oai() - nhal_wdog_onoff[i])/cpuf/1000000.0) < NHAL_WATCHDOG_SRCHINIT, "virtual SRCH on off timeout!! (hwStatus : %i)\n", nhal_srch_ix[0]->roReg.hwStatus);
		}


		nhal_srchInstance[i].status = nhal_Off;

	}

	return 0;
}



int NHAL_srchcmd_on(uint8_t bank,
						   int narfcn,
						   nhal_srchmode_e mode,
						   uint8_t numerology,
						   uint16_t fftSize,
						   uint16_t duration,
						   uint32_t srchSubframe,
						   uint32_t ssb_Foffset,
						   uint16_t ssb_periodicity,
						   int high_speed_flag,
						   uint8_t option_decimation,
						   uint8_t option_pbchCheck,
						   int16_t target_nid,
						   int16_t threshold,
						   uint32_t search_window,
						   int search_offset,
						   uint8_t issb,
						   uint8_t nhf
						   )
{
	//validity check (state)
	if (bank >= VSRCH_MAXNB_SRCHBNK)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept on command : invalid bank number (%i)\n", bank);
		return -1;
	}
	
	if (nhal_srchInstance[bank].status == nhal_Null ||
		nhal_srchInstance[bank].status == nhal_On ||
		nhal_srchInstance[bank].status == nhal_On_going)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept on command : SRCH is already on! (%i)\n", nhal_srchInstance[bank].status);
		return -1;
	}
	

	nhal_srchInstance[bank].status = nhal_On_going;	
	LOG_I(PHY, "[HALcmd] Searcher[%i] on (narfcn : %i, srchmode : %i, u : %i, fftSize : %i, duration : %i, subframe search : %i, SSB subcarrier offset: %i, SSB period : %i, highSpeedFlag:%i, option(decimation/pbch check) : %i / %i, target NID : %i, threshold : %i, search window : %i, offset : %i\n",
		bank, narfcn, mode, numerology, fftSize, duration, srchSubframe, ssb_Foffset, ssb_periodicity, high_speed_flag, option_decimation, option_pbchCheck, target_nid, threshold, search_window, search_offset);


	//1. wait for HW status to be off ---------------------------------------------
	nhal_wdog_onoff[bank] = rdtsc_oai();
	//wait for status
	while (nhal_srch_ix[bank]->roReg.hwStatus != srchst_off)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_onoff[bank])/cpuf/1000000.0) < NHAL_WATCHDOG_SRCHONOFF, "virtual SRCH initialization timeout!! (hwStatus : %i)\n", nhal_srch_ix[bank]->roReg.hwStatus);
	}



	//register setting for full search
	nhal_srch_ix[bank]->woReg.narfcn = narfcn;
	if (mode == nhal_srchmode_full)
	{
		nhal_srch_ix[bank]->woReg.mode = srchMode_full;
	}
	else if (mode == nhal_srchmode_online)
	{
		nhal_srch_ix[bank]->woReg.mode = srchMode_online;
	}
	else
	{
		LOG_E(PHY, "[WARNING] configuration mode is weird! (%i) force to set full search mode\n", mode);
		nhal_srch_ix[bank]->woReg.mode = srchMode_full;
	}
	
	nhal_srch_ix[bank]->woReg.numerology = numerology;	//set default as u=0 (later this should be changed according to channel configuration or synch. raster rule)
	nhal_srch_ix[bank]->woReg.fftSize = fftSize; 	//FFT size
	nhal_srch_ix[bank]->woReg.duration = duration;		//use default cell search time
	nhal_srch_ix[bank]->woReg.srchSubframe = srchSubframe;	//use default cell search time
	nhal_srch_ix[bank]->woReg.ssb_periodicity = ssb_periodicity;
	nhal_srch_ix[bank]->woReg.ssb_Foffset = ssb_Foffset;
	
	nhal_srch_ix[bank]->woReg.high_speed_flag = high_speed_flag;
	nhal_srch_ix[bank]->woReg.option_decimation = option_decimation;
	nhal_srch_ix[bank]->woReg.option_pbchCheck = option_pbchCheck;
	
	nhal_srch_ix[bank]->woReg.target_nid = target_nid; //full search
	nhal_srch_ix[bank]->woReg.threshold = threshold; //use default H/W threshold
	nhal_srch_ix[bank]->woReg.search_window = search_window; //full search (no limitation on search window)_
	nhal_srch_ix[bank]->woReg.search_offset = search_offset; //full search (no limitation on search window)
	nhal_srch_ix[bank]->woReg.issb = issb;
	nhal_srch_ix[bank]->woReg.nhf = nhf;
	

	//make the hardware on
	nhal_srch_ix[bank]->rwReg.onoff = 1;
	pthread_cond_signal(&(nhal_srch_ix[bank]->cond_srchHal));
	

	nhal_wdog_onoff[bank] = rdtsc_oai();	
	while (nhal_srch_ix[bank]->roReg.hwStatus != srchst_on)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_onoff[bank])/cpuf/1000000.0) < NHAL_WATCHDOG_SRCHONOFF, "virtual SRCH on/off timeout!!\n");
	}

	nhal_srchInstance[bank].narfcn = narfcn;
	nhal_srchInstance[bank].mode = mode;
	nhal_srchInstance[bank].numerology = numerology;
	nhal_srchInstance[bank].fftSize = fftSize;
	nhal_srchInstance[bank].duration = duration;	
	nhal_srchInstance[bank].srchSubframe = srchSubframe;
	nhal_srchInstance[bank].ssb_periodicity = ssb_periodicity;
	nhal_srchInstance[bank].ssb_Foffset = ssb_Foffset;
	nhal_srchInstance[bank].high_speed_flag = high_speed_flag;
	nhal_srchInstance[bank].option_decimation = option_decimation;
	nhal_srchInstance[bank].option_pbchCheck = option_pbchCheck;
	nhal_srchInstance[bank].target_nid = target_nid;
	nhal_srchInstance[bank].threshold = threshold;
	nhal_srchInstance[bank].search_window = search_window;
	nhal_srchInstance[bank].search_offset = search_offset;
	nhal_srchInstance[bank].issb = issb;
	nhal_srchInstance[bank].nhf = nhf;
		
	nhal_srchInstance[bank].status = nhal_On;

	return 0;
}





int NHAL_srchcmd_off(uint8_t bank)
{
	//validity check (state)
	if (bank >= VSRCH_MAXNB_SRCHBNK)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept off command : invalid bank number (%i)\n", bank);
		return -1;
	}
	if (nhal_srchInstance[bank].status == nhal_Null ||
		nhal_srchInstance[bank].status == nhal_Off ||
		nhal_srchInstance[bank].status == nhal_Off_going)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept on command : SRCH [%i]is already on! (%i)\n", bank, nhal_srchInstance[bank].status);
		return -1;
	}

	nhal_srchInstance[bank].status = nhal_Off_going;
	LOG_I(PHY, "[HALcmd] SRCH [%i] off\n", bank);

	//1. wait for HW status to be on ---------------------------------------------
	if (nhal_srchInstance[bank].status == nhal_On_going)
	{
		nhal_wdog_onoff[bank] = rdtsc_oai();
		//wait for status
		while (nhal_srch_ix[bank]->roReg.hwStatus != srchst_on)
		{
			usleep(10);
			AssertFatal( (int)((rdtsc_oai() - nhal_wdog_onoff[bank])/cpuf/1000000.0) < NHAL_WATCHDOG_SRCHONOFF, "virtual SRCH %i on/off timeout!!\n", bank);
		}
	}
	

	//2. set the register to be off!
	nhal_srch_ix[bank]->rwReg.onoff = 0;

	//3. wait for HW status to be off ---------------------------------------------
	nhal_wdog_onoff[bank] = rdtsc_oai();
	//wait for status
	while (nhal_srch_ix[bank]->roReg.hwStatus != srchst_off)
	{
		usleep(10);
		AssertFatal( (int)((rdtsc_oai() - nhal_wdog_onoff[bank])/cpuf/1000000.0) < NHAL_WATCHDOG_SRCHONOFF, "virtual SRCH %i on/off timeout!!\n", bank);
	}

	NHAL_initAbstract(bank);
	nhal_srchInstance[bank].status = nhal_Off;
	
	return 0;
}


srchReport_t* NHAL_getSrchRes(uint8_t bank, int index)
{
	int cnt=0;
	srchReport_t* aPtr = nhal_srchInstance[bank].srch_result;

	while (aPtr != NULL && cnt != index)
	{
		aPtr = aPtr->next;
		cnt++;
	}

	return aPtr;
}




int NHAL_cpySrchRes(uint8_t bank)
{
	int cnt=0;
	srchReport_t* aPtr;
	srchReport_t* aPrev;
	srchReport_t* vPtr = nhal_srch_ix[bank]->roReg.srch_result;

	while (vPtr != NULL)
	{
		aPtr = (srchReport_t*)malloc(sizeof(srchReport_t));
		memcpy(aPtr, vPtr, sizeof(srchReport_t));
		if (cnt == 0)
		{
			nhal_srchInstance[bank].srch_result = aPtr;
		}
		else
		{
			aPrev->next = aPtr;
		}
		aPtr->next = NULL;
		
		vPtr = vPtr->next;
		aPrev = aPtr;

		cnt++;
	}

	return cnt;
}

//only for full search mode
int NHAL_srchcmd_waitIRQ(uint8_t bank)
{
	//validity check (state)
	if (bank >= VSRCH_MAXNB_SRCHBNK)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept waitIRQ command : invalid bank number (%i)\n", bank);
		return -1;
	}
	if (nhal_srchInstance[bank].status != nhal_On)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept waitIRQ command : SRCH [%i] is not on! (%i)\n", bank, nhal_srchInstance[bank].status);
		return -1;
	}
	if (nhal_srchInstance[bank].mode != nhal_srchmode_full)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept waitIRQ command : SRCH [%i] is not full search mode! (%i)\n", bank, nhal_srchInstance[bank].mode);
		return -1;
	}

	//wait for searcher action ends (this is because we only have one searcher bank
	AssertFatal ( 0== pthread_mutex_lock(&(nhal_srch_ix[bank]->mutex_srchHal)), "");
	while (nhal_srch_ix[bank]->roReg.hwStatus == srchst_on)
		pthread_cond_wait( &(nhal_srch_ix[bank]->cond_srchIrq), &(nhal_srch_ix[bank]->mutex_srchHal)); // the thread waits here most of the time
	AssertFatal ( 0== pthread_mutex_unlock(&(nhal_srch_ix[bank]->mutex_srchHal)), "");

	nhal_srchInstance[bank].status = nhal_Idle;
	
	AssertFatal ( 0== pthread_mutex_lock(&(nhal_srch_ix[bank]->mutex_srchHal)), "");
	if ( (nhal_srchInstance[bank].srch_cnt = NHAL_cpySrchRes(bank)) != nhal_srch_ix[bank]->roReg.srch_cnt)
		LOG_E(PHY, "[WARNING] result from vSrch is wrong ! (cnt : %i, report cnt : %i)\n", nhal_srch_ix[bank]->roReg.srch_cnt, nhal_srchInstance[bank].srch_cnt);
	AssertFatal ( 0== pthread_mutex_unlock(&(nhal_srch_ix[bank]->mutex_srchHal)), "");

	return (int)nhal_srchInstance[bank].srch_cnt;	
}




int NHAL_srchcmd_loadSrchRes(uint8_t bank, uint8_t srchIndex)
{
	if (bank >= VSRCH_MAXNB_SRCHBNK)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept loadSrchRes command : invalid bank number (%i)\n", bank);
		return -1;
	}
	if (nhal_srchInstance[bank].status != nhal_Idle)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept loadSrchRes command : SRCH [%i] is not idle state! (%i)\n", bank, nhal_srchInstance[bank].status);
		return -1;
	}
	
	if ((nhal_srchRes_load[bank] = NHAL_getSrchRes(bank, srchIndex)) == NULL)
	{
		LOG_E(PHY, "[ERROR] HAL cmd failed loadSrchRes command : no %i'th report exist\n", srchIndex);
		return -1;
	}

	return 0;
}



int NHAL_srchcmd_getFrameFreqOffset(uint8_t bank, uint8_t srchIndex, int* out_frameOffset, int* out_freqOffset)
{
	//validity check (state)
	if (bank >= VSRCH_MAXNB_SRCHBNK)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getOffset command : invalid bank number (%i)\n", bank);
		return -1;
	}
	if (out_frameOffset == NULL || out_freqOffset == NULL)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getOffset command : output buffer is NULL! (%i, %i)\n", out_frameOffset, out_freqOffset);
		return -1;
	}
	if (nhal_srchInstance[bank].status != nhal_Idle)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getOffset command : SRCH [%i] is not idle state! (%i)\n", bank, nhal_srchInstance[bank].status);
		return -1;
	}
	if (nhal_srchRes_load[bank] == NULL)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getOffset command : SRCH [%i] is not in loaded state. call the load func. first\n", bank);
		return -1;
	}

	*out_frameOffset = nhal_srchRes_load[bank]->frame_offset;
	*out_freqOffset = nhal_srchRes_load[bank]->freq_offset;


	return 0;
}


int NHAL_srchcmd_getSsbProfile(uint8_t bank, uint8_t srchIndex, uint16_t* out_nid, uint8_t* out_ssbIndex, uint8_t* out_Lmax, uint8_t* out_nhf)
{
	//validity check (state)
	if (bank >= VSRCH_MAXNB_SRCHBNK)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getSsbProfile command : invalid bank number (%i)\n", bank);
		return -1;
	}
	if (out_nid == NULL || out_ssbIndex == NULL || out_Lmax == NULL || out_nhf == NULL)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getSsbProfile command : output buffer is NULL! (%i, %i, %i, %i)\n", out_nid, out_ssbIndex, out_Lmax, out_nhf);
		return -1;
	}
	if (nhal_srchInstance[bank].status != nhal_Idle)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getSsbProfile command : SRCH [%i] is not idle state! (%i)\n", bank, nhal_srchInstance[bank].status);
		return -1;
	}
	if (nhal_srchRes_load[bank] == NULL)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getSsbProfile command : SRCH [%i] is not in loaded state. call the load func. first\n", bank);
		return -1;
	}


	*out_nid = nhal_srchRes_load[bank]->nid;
	*out_ssbIndex = nhal_srchRes_load[bank]->ssb_index;
	*out_Lmax = nhal_srchRes_load[bank]->Lmax;
	*out_nhf = nhal_srchRes_load[bank]->nhf;
	
	return 0;
}

int NHAL_srchcmd_getPbchDec(uint8_t bank, uint8_t srchIndex, uint8_t* out_extraByte, uint8_t* out_mib)
{
	//validity check (state)
	if (bank >= VSRCH_MAXNB_SRCHBNK)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getpbchDec command : invalid bank number (%i)\n", bank);
		return -1;
	}
	if (out_extraByte == NULL || out_mib == NULL)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getpbchDec command : output buffer is NULL! (%i, %i)\n", out_extraByte, out_mib);
		return -1;
	}
	if (nhal_srchInstance[bank].status != nhal_Idle)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getpbchDec command : SRCH [%i] is not idle state! (%i)\n", bank, nhal_srchInstance[bank].status);
		return -1;
	}
	if (nhal_srchRes_load[bank] == NULL)
	{
		LOG_E(PHY, "[ERROR] HAL cmd cannot accept getpbchDec command : SRCH [%i] is not in loaded state. call the load func. first\n", bank);
		return -1;
	}


	*out_extraByte = nhal_srchRes_load[bank]->extraByte;
	memcpy(out_mib, nhal_srchRes_load[bank]->pbchRes, 3*sizeof(uint8_t));
	
	return 0;
}
