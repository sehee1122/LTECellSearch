#define _GNU_SOURCE
#include "vThread_RF.h"
#include "vThread_srch.h"
#include "nr-uesoftmodem.h"
#include "vThread_HWCommon.h"
#include "../../ARCH/COMMON/common_lib.h"
#include "vThreadIx_rfSrch.h"
#include "vSrch_ssbCommon.h"
#include "vSrch_pssDet.h"
#include "vSrch_sssDet.h"
#include "vSrch_pbchDet.h"
#include "vSrch_ssbFft.h"



#include <sched.h>
#include <string.h>


#define VSRCH_DEFAULT_ONLINE_WINDOW		12000


#define VSRCH_SRCHERR_SUCCESS			0
#define VSRCH_SRCHERR_PSSFAIL			1
#define VSRCH_SRCHERR_SSSFAIL			2
#define VSRCH_SRCHERR_PBCHFAIL			3
#define VSRCH_SRCHERR_MAX				4








/* --------------- HAL configuration status type --------------- */
typedef struct { //initial configurations
	//mandatory
	halSrch_srchMode_e mode;		//full / partial / online
	uint32_t narfcn;				//center frequency
	uint8_t numerology;				//u value
	uint16_t fftSize;				//bandwidth (in terms of FFT Size)	
	//unsigned int ssbOffset;			//offset of the SSB block

	//options
	uint16_t duration; 				//in ms, if 0 then search till something is found
	uint32_t srchSubframe;			//number of subframes to be searched
	int16_t target_nid;				//target NID to track (-1 : search for any)
	int16_t threshold;		//threashold for seaching (0 : hw-guided threshold)

	uint8_t slotsPerSubframe;		//number of slots in a subframe (depends on the numerology)
	uint16_t nbCp;					//number of samples for cyclic prefix
	uint32_t samplesPerSubframe;	//number of samples in a subframe
	uint32_t nbSamplesSsb;			//number of samples for a SSB (4 symbols * (FFT size + CP)
	uint32_t ssb_Foffset;			//number of the subcarrier index that the SSB starts
	int high_speed_flag;			//whether the UE is in high-speed or not

	uint32_t srchCnt;				//number of subframes to be searched
	long long srchStartTime;		//start of the search time

	int ssb_slot;					//number of slot that the srcher is interested in (online mode only)
	uint8_t issb;					//SSB index (beam number) : online mode only
	uint8_t nhf;

} vsrch_cfgInfo_t;

typedef struct { //real-time configurations
	uint8_t enable_decimation;		//0 : none, 1 : enable decimation (not yet implemented)
	uint8_t enable_pbchDec;			//0 :  none, 1 : sync. detection includes pbch CRC check

	//real-time configurations
	uint32_t search_window;			//0 : full search (unit : sample)
	int32_t search_offset;			///search starting point by sample
	uint16_t ssb_periodicity;		//ssb periodicity
} vsrch_rtCfg_t;





//Buffers
static uint8_t vsrch_nbBank=0; //number of srcher threads available
//static int vsrch_ssbBuf[VSRCH_MAXNB_SRCHBNK][HW_NB_RXANT][VSRCH_NB_SSBSYMB*(HW_MAX_FFTSIZE+HW_MAX_NBCP)] __attribute__((aligned(32)));
//static int vsrch_ssbFBuf[VSRCH_MAXNB_SRCHBNK][HW_NB_RXANT][VSRCH_NB_SSBSYMB*HW_MAX_FFTSIZE] __attribute__((aligned(32)));

static int** vsrch_ssbBuf[VSRCH_MAXNB_SRCHBNK];
static int** vsrch_ssbFBuf[VSRCH_MAXNB_SRCHBNK];


static int vsrch_mesTime=0;

/*     ------- HAL - HW interface related ------          */
//registers
const halSrch_woReg_t 		*vsrch_HALix_in[VSRCH_MAXNB_SRCHBNK]; 		//write only registers
halSrch_roReg_t 			*vsrch_HALix_out[VSRCH_MAXNB_SRCHBNK];		//read only registers
halSrch_rwReg_t 			*vsrch_HALix_buf[VSRCH_MAXNB_SRCHBNK];		//read/write registers

pthread_mutex_t				*vsrch_muPtr_srchHal[VSRCH_MAXNB_SRCHBNK];	//MUTEX for HAL interface
pthread_cond_t				*vsrch_csPtr_srchHal[VSRCH_MAXNB_SRCHBNK];	//condition signal for off -> on
pthread_cond_t				*vsrch_csPtr_srchIrq[VSRCH_MAXNB_SRCHBNK];  //interrupt signal for search end (full search mode only)


//static vsrch_rtCfg_t 		vsrch_rtCfg[VSRCH_MAXNB_SRCHBNK]; //SRCH HW abstraction



/*     ------- HW - HW interface related ------          */
//RF - SRCH interface -- only one is defined, because there is assumed to be only one RF
static vrfSrch_srchReg_t		vrfsrch_srchReg;		//srch register handled by RF
vrfSrch_rfReg_t* 				vrfsrch_rfRegPtr;		//rf registers seen by srcher banks
vrfSrch_sharedReg_t*			vrfsrch_sharedRegPtr;


char vsrch_print_srchError[VSRCH_SRCHERR_MAX][100] = {
	"Normal operation",
	"PSS detection fail",
	"SSS detection fail",
	"PBCH detection fail",
};


void vsrch_initSrchReport(uint8_t bankId)
{
	srchReport_t* ptr = vsrch_HALix_out[bankId]->srch_result;
	srchReport_t* nextPtr;

	while (ptr != NULL)
	{
		nextPtr = ptr->next;
		free(ptr);
		ptr = nextPtr;
	}

	vsrch_HALix_out[bankId]->srch_result = NULL;
	vsrch_HALix_out[bankId]->srch_cnt = 0;
}



void vsrch_makeSrchReport(uint8_t bankId, int16_t cellId, int rx_offset, int freq_offset, uint8_t ssb_index, uint8_t nhf, uint8_t* decoded_output, uint8_t Lmax, uint8_t extra_byte)
{
	srchReport_t* ptr = (srchReport_t*)malloc(sizeof(srchReport_t));
	

	AssertFatal ( ptr != NULL, "error in making searcher report : memory allocation failed!\n");

	ptr->nid = (uint16_t) cellId;
	ptr->frame_offset = rx_offset;
	ptr->freq_offset = freq_offset;
	ptr->ssb_index = ssb_index;
	ptr->nhf = nhf;
	ptr->pbchCrc = 0;
	memcpy(ptr->pbchRes, decoded_output, 3*sizeof(uint8_t));
	ptr->Lmax = Lmax;
	ptr->extraByte = extra_byte;
	
	ptr->next = NULL;	

	srchReport_t* listPtr = vsrch_HALix_out[bankId]->srch_result;
	if (listPtr == NULL)
	{
		vsrch_HALix_out[bankId]->srch_result = ptr;
	}
	else
	{
		while(listPtr->next != NULL)
		{
			listPtr = listPtr->next;
		}
		listPtr->next = ptr;
	}
	vsrch_HALix_out[bankId]->srch_cnt++;
	
}


//interface configuration between RF and SRCH
vrfSrch_srchReg_t* vsrch_configSrchReg(void)
{
	return &vrfsrch_srchReg;
}





uint8_t vsrch_calcLmax(uint32_t narfcn)
{
	if (narfcn < HW_MAX_NARFCN_3GHZ)
		return 4;
	else if (narfcn < HW_MAX_NARFCN_6GHZ)
		return 8;


	return 64;
}






int8_t vsrch_configInit(uint8_t bankId, vsrch_cfgInfo_t* cfgInfo, vsrch_rtCfg_t* rtCfgInfo)
{
	int search_offset;

	cfgInfo->mode = vsrch_HALix_in[bankId]->mode;
	cfgInfo->narfcn = vsrch_HALix_in[bankId]->narfcn;
	cfgInfo->numerology = vsrch_HALix_in[bankId]->numerology;
	cfgInfo->fftSize = vsrch_HALix_in[bankId]->fftSize;

	cfgInfo->duration = vsrch_HALix_in[bankId]->duration;
	cfgInfo->srchSubframe = vsrch_HALix_in[bankId]->srchSubframe;
	cfgInfo->target_nid = vsrch_HALix_in[bankId]->target_nid;
	cfgInfo->threshold = vsrch_HALix_in[bankId]->threshold;
	rtCfgInfo->ssb_periodicity = vsrch_HALix_in[bankId]->ssb_periodicity;

	rtCfgInfo->enable_decimation = vsrch_HALix_in[bankId]->option_decimation;
	rtCfgInfo->enable_pbchDec = vsrch_HALix_in[bankId]->option_pbchCheck;

	rtCfgInfo->search_window = vsrch_HALix_in[bankId]->search_window;
	search_offset = vsrch_HALix_in[bankId]->search_offset;
	cfgInfo->ssb_Foffset = vsrch_HALix_in[bankId]->ssb_Foffset;
	cfgInfo->high_speed_flag = vsrch_HALix_in[bankId]->high_speed_flag;
	cfgInfo->issb = vsrch_HALix_in[bankId]->issb;
	cfgInfo->nhf = vsrch_HALix_in[bankId]->nhf;


	cfgInfo->slotsPerSubframe = vhw_calcSlotPerSubframe(cfgInfo->numerology);
	cfgInfo->nbCp = vhw_calcNbPrefixSample(cfgInfo->fftSize); 
	cfgInfo->samplesPerSubframe = vhw_calcSamplesPerSubframe (cfgInfo->fftSize, cfgInfo->numerology);
	cfgInfo->nbSamplesSsb = VSRCH_NB_SSBSYMB * (cfgInfo->fftSize + cfgInfo->nbCp);

	if (cfgInfo->mode == srchMode_full)
	{
		if (cfgInfo->duration == 0 && cfgInfo->srchCnt == 0)
		{
			cfgInfo->srchCnt = VSRCH_DEFAULT_FULLSRCH_CNT;
		}
		else
		{
			cfgInfo->srchCnt = cfgInfo->srchSubframe;
		}

		if (cfgInfo->duration != 0)
		{
			cfgInfo->srchStartTime = rdtsc_oai();
		}
	}

	if (cfgInfo->mode == srchMode_online)
	{
		cfgInfo->ssb_slot = vhw_calcSsbSlotNb(cfgInfo->narfcn, cfgInfo->numerology, cfgInfo->issb); //this should be fixed according to issb

		int symbOffset = vhw_calcSsbSymbNb(cfgInfo->narfcn, cfgInfo->numerology, cfgInfo->issb)%HW_NB_SYM_IN_SLOT;
		if (symbOffset > 0)
		{
			rtCfgInfo->search_offset =	(cfgInfo->samplesPerSubframe/cfgInfo->slotsPerSubframe) 					//number of symbols in a slot
										- (HW_NB_SYM_IN_SLOT - symbOffset)*(cfgInfo->fftSize + cfgInfo->nbCp)  //minus SSB symbol offset
										- rtCfgInfo->search_window/2											//minus half of the search window
										+ search_offset;
		}
		else
		{
			rtCfgInfo->search_offset = search_offset;
		}
	}
	else
	{
		cfgInfo->ssb_slot = 0;
		rtCfgInfo->search_offset = search_offset;
	}

	//defence code : in case that search offset gets minus
	if (rtCfgInfo->search_offset < 0)
	{
		LOG_E(PHY, "[WARNING] vSrch realtime config : search offset is negative (%i). force to make the offset into 0\n", rtCfgInfo->search_offset);
		rtCfgInfo->search_offset = 0;
	}

	LOG_E(PHY, "[vSrch] configured and will start : mode %i, arfcn %i numerology %i fftSize %i duration %i subf %i window %i search offset %i Foffset %i highSpeedFlag %i, issb:%i, nhf:%i\n",
		cfgInfo->mode,
		cfgInfo->narfcn,
		cfgInfo->numerology,
		cfgInfo->fftSize,
		cfgInfo->duration,
		cfgInfo->srchSubframe,
		rtCfgInfo->search_window,
		rtCfgInfo->search_offset,
		cfgInfo->ssb_Foffset,
		cfgInfo->high_speed_flag,
		cfgInfo->issb,
		cfgInfo->nhf);

	vsrch_mesTime = 1;
	
	return 0;
}





static void vsrch_configRealTime(uint8_t bankId, vsrch_cfgInfo_t* cfgInfo, vsrch_rtCfg_t* rtCfgInfo)
{
	int search_offset;
	AssertFatal ( 0== pthread_mutex_lock(vsrch_muPtr_srchHal[bankId]), "error on SRCH HAL MUTEX while reading onOff");
	rtCfgInfo->enable_decimation = vsrch_HALix_in[bankId]->option_decimation;
	rtCfgInfo->enable_pbchDec = vsrch_HALix_in[bankId]->option_pbchCheck;
	rtCfgInfo->search_window = vsrch_HALix_in[bankId]->search_window;
	search_offset = vsrch_HALix_in[bankId]->search_offset;
	AssertFatal ( 0== pthread_mutex_unlock(vsrch_muPtr_srchHal[bankId]), "error on SRCH HAL MUTEX while reading onOff");

	if (cfgInfo->mode == srchMode_online)
	{
		int symbOffset = vhw_calcSsbSymbNb(cfgInfo->narfcn, cfgInfo->numerology, cfgInfo->issb)%HW_NB_SYM_IN_SLOT;
		if (symbOffset > 0)
		{
			rtCfgInfo->search_offset =  (cfgInfo->samplesPerSubframe/cfgInfo->slotsPerSubframe) 				//number of symbols in a slot
										- (HW_NB_SYM_IN_SLOT - symbOffset)*(cfgInfo->fftSize + cfgInfo->nbCp)  //minus 
										- rtCfgInfo->search_window/2											//minus half of the search window
										+ search_offset;
		}
		else
		{
			rtCfgInfo->search_offset = search_offset;
		}
	}
	else
	{
		rtCfgInfo->search_offset = search_offset;
	}

	//defence code : in case that search offset gets minus
	if (rtCfgInfo->search_offset < 0)
	{
		LOG_E(PHY, "[WARNING] vSrch realtime config : search offset is negative (%i). force to make the offset into 0\n", rtCfgInfo->search_offset);
		rtCfgInfo->search_offset = 0;
	}
}


static void vsrch_forceOff(uint8_t bankId)
{
	AssertFatal ( 0== pthread_mutex_lock(vsrch_muPtr_srchHal[bankId]), "error on SRCH HAL MUTEX while reading onOff");
	vsrch_HALix_buf[bankId]->onoff = 0;
	AssertFatal ( 0== pthread_mutex_unlock(vsrch_muPtr_srchHal[bankId]), "error on SRCH HAL MUTEX while reading onOff");
}



static uint8_t vsrch_readOnOff(uint8_t bankId)
{
	uint8_t onOff;
	
	AssertFatal ( 0== pthread_mutex_lock(vsrch_muPtr_srchHal[bankId]), "error on SRCH HAL MUTEX while reading onOff");
	onOff = vsrch_HALix_buf[bankId]->onoff;
	AssertFatal ( 0== pthread_mutex_unlock(vsrch_muPtr_srchHal[bankId]), "error on SRCH HAL MUTEX while reading onOff");

	return onOff;
}




void *vSRCH_mainThread(void *arg)
{
	static int __thread UE_thread_synch_retval;
	char threadname[128];
	uint8_t max_Nid2;
	int16_t cellId;
	double f_off_rad;
	int f_off;
	uint8_t rfProcNum;
	
	uint8_t bankId = vsrch_nbBank++;
	uint8_t onOffCfg;
	vsrch_cfgInfo_t cfgInfo;
	vsrch_rtCfg_t rtCfgInfo;

	long long syncTimer=0;
	int32_t pos_pssDet, pos_slotBound; //position offset detected by pss
	int rx_offset;
	int ssb_offset;
	unsigned int length;//, startOffset;
	uint8_t *pbchRes;
	uint8_t Lmax;
	uint8_t ssb_index, nhf;
	uint8_t extra_byte;
	int slot_offset;
	uint8_t erCause;
	
	//register initialization
	vsrch_HALix_in[bankId] 	= &( ((ix_halSrch_t*)arg)->woReg  );
	vsrch_HALix_out[bankId] 	= &( ((ix_halSrch_t*)arg)->roReg );
	vsrch_HALix_buf[bankId] 	= &( ((ix_halSrch_t*)arg)->rwReg );
	vsrch_muPtr_srchHal[bankId] = (pthread_mutex_t*)&(((ix_halSrch_t*)arg)->mutex_srchHal);
	vsrch_csPtr_srchHal[bankId] = (pthread_cond_t*)&(((ix_halSrch_t*)arg)->cond_srchHal);
	vsrch_csPtr_srchIrq[bankId] = (pthread_cond_t*)&(((ix_halSrch_t*)arg)->cond_srchIrq);
	vsrch_HALix_out[bankId]->hwStatus = srchst_null;

	//----------- Buffer initialization
	//main Srcher buffer for specific bank	
	vsrch_ssbBuf[bankId] = (int**) vhw_malloc16_clear( HW_NB_RXANT * sizeof(int*) );
	vsrch_ssbFBuf[bankId] = (int**) vhw_malloc16_clear( HW_NB_RXANT * sizeof(int*) );
	for (int i=0;i<HW_NB_RXANT;i++)
	{
		vsrch_ssbBuf[bankId][i] = (int*) vhw_malloc16_clear( VSRCH_NB_SSBSYMB*(HW_MAX_FFTSIZE+HW_MAX_NBCP) * sizeof(int) );
		vsrch_ssbFBuf[bankId][i] = (int*) vhw_malloc16_clear( VSRCH_NB_SSBSYMB*HW_MAX_FFTSIZE * sizeof(int) );
	}
	//sub block buffers
	vsrch_initSssBuf();
	vsrch_initPbchDetBuf();

	int** ssbBuf = vsrch_ssbBuf[bankId];
	int** ssbFBuf = vsrch_ssbFBuf[bankId];

		


	//thread initialization
	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);

	if ( threads.sync != -1 )
		CPU_SET(threads.sync, &cpuset);

	// this thread priority must be lower that the main acquisition thread
	sprintf(threadname, "srcher %d", bankId);
	vhw_initvThread(100000, 500000, HW_FIFO_PRIORITY-1, &cpuset, threadname);


	//HW interface initialization
	// vs. RF
	//pthread_mutex_init(&(vrfsrch_srchReg.rwReg.regMutex), NULL); //RF - SRCH interface mutex : intiialzied by latter block
	pthread_mutex_init(&(vrfsrch_srchReg.roReg.regMutex), NULL); //RF - SRCH interface mutex : intiialzied by latter block
	pthread_cond_init(&(vrfsrch_srchReg.woReg.irq_srchInst), NULL);

	vrfsrch_srchReg.roReg.hwStatus = hwIx_null;
	vrfsrch_sharedRegPtr = vrf_configSrchSharedReg();
		
	AssertFatal ( NULL != (vrfsrch_rfRegPtr = vrf_configRfReg()), "[vHW][ERROR] error in configuration RF-SRCH register : pointer is NULL!\n");

	
	//const halSrch_woReg_t* vth_woReg = vsrch_HALix_in[bankId];
	halSrch_roReg_t* vth_roReg = vsrch_HALix_out[bankId];
	halSrch_rwReg_t* vth_rwReg = vsrch_HALix_buf[bankId];
	pthread_mutex_t* vth_halMutex = vsrch_muPtr_srchHal[bankId];
	pthread_cond_t* vth_halCond = vsrch_csPtr_srchHal[bankId];


	vth_roReg->hwStatus = srchst_off;
	vth_roReg->srch_cnt = 0;
	vth_roReg->srch_result = NULL;
	AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_srchReg.roReg.regMutex)), "");
	vrfsrch_srchReg.roReg.hwStatus = hwIx_off;
	AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_srchReg.roReg.regMutex)), "");

	printf("[vHW] SRCH virtual HW (Bank %i) is initialized, waiting for on signal \n", bankId);
	
	while (!oai_exit)
	{

		//off state loop (stay until on signal comes)
		AssertFatal ( 0 == pthread_mutex_lock(vth_halMutex), "");
		while (vth_rwReg->onoff == 0)
			  pthread_cond_wait( vth_halCond, vth_halMutex );
		AssertFatal ( 0 == pthread_mutex_unlock(vth_halMutex), "");

		
		//check on/off register and process it
		//if register is 'on', then do static configuration (init and start)
		if (vsrch_configInit(bankId, &cfgInfo, &rtCfgInfo) == 0)
		{
			//initialize PSS/SSS hardwares
			vsrch_initPssDet(cfgInfo.ssb_Foffset, cfgInfo.fftSize);
			vsrch_initSssDet();
			Lmax = vsrch_calcLmax(cfgInfo.narfcn);
			vsrch_initSrchReport(bankId);
			
			AssertFatal ( 0== pthread_mutex_lock(vth_halMutex), "");
			vth_roReg->hwStatus = srchst_on;
			AssertFatal ( 0== pthread_mutex_unlock(vth_halMutex), "");
			AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_srchReg.roReg.regMutex)), "");
			vrfsrch_srchReg.roReg.hwStatus = hwIx_on;
			AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_srchReg.roReg.regMutex)), "");
			
			printf("[vHW] >>>>>>>>>>>>>>>>> SRCH virtual HW (%i) is on!\n", bankId);
		}
		else
		{
			printf("[vHW] >>>>>>>>>>>>>>>>> failed to start SRCH virtual HW (%i)!\n", bankId);
			vsrch_forceOff(bankId);
			continue;
		}


		

		
		//inner loop for on operation
		while ( !oai_exit && 
				vth_roReg->hwStatus == srchst_on &&
				(onOffCfg = vsrch_readOnOff(bankId)) == 1 )
		{
			//interrupt from RF
			AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedRegPtr->sharedMutex)), "");
			while ( (vrfsrch_sharedRegPtr->processBitmap & 0x01) == 0)
				pthread_cond_wait( &(vrfsrch_srchReg.woReg.irq_srchInst), &(vrfsrch_sharedRegPtr->sharedMutex) ); // the thread waits here most of the time
			AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedRegPtr->sharedMutex)), "");

			if (cfgInfo.mode == srchMode_online)
			{	
				//skip if there is no PSS
				AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_rfRegPtr->roReg.regMutex)), "");
				int16_t hfslot_nb = vrfsrch_rfRegPtr->roReg.hfslot_nb[0];
				int hfframe_nb = vrfsrch_rfRegPtr->roReg.hfframe_nb[0];
				slot_offset = vrfsrch_rfRegPtr->roReg.slot_offset[0];
				AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_rfRegPtr->roReg.regMutex)), "");

				//DMRS imp : 1. skip if SSB does not exist in this half frame
				
				
				
				LOG_D(PHY, "[vSrch] online mode operation %i, %i\n", hfslot_nb, hfframe_nb);
			}
			else
			{
				slot_offset = 0;
			}

			//read the real-time configure register and apply in real-time
			vsrch_configRealTime(bankId, &cfgInfo, &rtCfgInfo);
			rfProcNum = 0;




			//	 Initial synchronisation
			erCause = VSRCH_SRCHERR_SUCCESS;

			if (vsrch_mesTime == 1)
				vhw_initTime(&syncTimer);

			//initialize srch result variables
			pbchRes = NULL;
			cellId = -1;

			//1. PSS detection
			//full search case (length is the full)
			if (rtCfgInfo.search_window > 0)
			{
				length = rtCfgInfo.search_window;
			}
			else
			{
				if (cfgInfo.mode == srchMode_full)
				{
					length = HW_NB_SUBF_IN_FRAME*cfgInfo.samplesPerSubframe;
				}
				else
				{
					length = VSRCH_DEFAULT_ONLINE_WINDOW;
				}
			}
			
			pos_pssDet = vsrch_detectPss((const int**)vrfsrch_rfRegPtr->roReg.rxData[rfProcNum], length, cfgInfo.fftSize, slot_offset + rtCfgInfo.search_offset , &max_Nid2, &f_off_rad, vsrch_mesTime);

			if (pos_pssDet > 0)
			{
				f_off = f_off_rad * vhw_subcarrier_spacing[cfgInfo.numerology];

				if (pos_pssDet >= cfgInfo.nbCp)
				{
					ssb_offset = pos_pssDet - cfgInfo.nbCp;
				}
				else
				{
					ssb_offset = pos_pssDet + (cfgInfo.samplesPerSubframe * HW_NB_SUBF_IN_FRAME) - cfgInfo.nbCp;
				}

				if (vsrch_mesTime == 1)
				{
					vhw_timeStamp(&syncTimer, "[PSS time]");

					LOG_E(PHY,"[vSRCH] Estimated PSS position %d, Nid2 %d\n",  pos_pssDet, max_Nid2);
					LOG_I(PHY,"  -> ssb_offset %d sync_pos_slot %d, freq_offset:%i \n", ssb_offset, pos_slotBound, f_off);
				}

				/* check that SSS/PBCH block is continuous inside the received buffer */
				if (ssb_offset < HW_NB_SUBF_IN_FRAME*cfgInfo.samplesPerSubframe - cfgInfo.nbSamplesSsb )
				{
					// digital compensation of FFO for SSB symbols
					if (cfgInfo.mode == srchMode_full)
					{  
						double s_time = 1/(1.0e3*cfgInfo.samplesPerSubframe);  // sampling time
						double off_angle = -2*VHW_PI*s_time*f_off;  // offset rotation angle compensation per sample
						double im, re;
			
						int start = ssb_offset;  // start for offset correction is at ssb_offset (pss time position)
						int end = start + cfgInfo.nbSamplesSsb;  // loop over samples in 4 symbols (ssb size), including prefix  
						int cnt=0;
						for(int n=start; n<end; n++)
						{
							for (int ar=0; ar<HW_NB_RXANT; ar++)
							{
								re = ((double)(((short *)vrfsrch_rfRegPtr->roReg.rxData[rfProcNum][ar]))[2*n]);
								im = ((double)(((short *)vrfsrch_rfRegPtr->roReg.rxData[rfProcNum][ar]))[2*n+1]);
								((short *)ssbBuf[ar])[cnt++] = (short)(round(re*cos(n*off_angle) - im*sin(n*off_angle))); 
								((short *)ssbBuf[ar])[cnt++] = (short)(round(re*sin(n*off_angle) + im*cos(n*off_angle)));
							}
						}
					}
					
					if (vsrch_mesTime == 1)
						vhw_timeStamp(&syncTimer, "[AFC]");
			
					vsrch_ssbFft(ssbBuf, cfgInfo.fftSize, cfgInfo.nbCp, 
								 cfgInfo.samplesPerSubframe/cfgInfo.slotsPerSubframe, cfgInfo.samplesPerSubframe*HW_NB_SUBF_IN_FRAME, 
								 ssbFBuf, VSRCH_BITMAP_PSSSSS);
					
					if (vsrch_mesTime == 1)
						vhw_timeStamp(&syncTimer, "[SSBFFTx2 time]");
					
					cellId = vsrch_detectSss(ssbBuf, ssbFBuf, 
							  cfgInfo.fftSize, max_Nid2, cfgInfo.ssb_Foffset, vsrch_mesTime);

					if (vsrch_mesTime == 1)
						vhw_timeStamp(&syncTimer, "[SSS time]");

					if (cellId >= 0)
					{
						vsrch_ssbFft(ssbBuf, cfgInfo.fftSize, cfgInfo.nbCp, 
									 cfgInfo.samplesPerSubframe/cfgInfo.slotsPerSubframe, cfgInfo.samplesPerSubframe*HW_NB_SUBF_IN_FRAME, 
									 ssbFBuf, VSRCH_BITMAP_PBCH);
						
						if (cfgInfo.mode == srchMode_full)
						{
							ssb_index = VSRCH_ISSB_NOTSET;
						}
						else
						{
							ssb_index = cfgInfo.issb;
							nhf = cfgInfo.nhf;
						}
						pbchRes = vsrch_detectPbch(cellId, Lmax, cfgInfo.fftSize, cfgInfo.ssb_Foffset, cfgInfo.high_speed_flag, ssbFBuf, &ssb_index, &nhf, &extra_byte, vsrch_mesTime);
						if (pbchRes == NULL)
							erCause = VSRCH_SRCHERR_PBCHFAIL;
						
						if (vsrch_mesTime == 1)
							vhw_timeStamp(&syncTimer, "[PBCH time]");
					}
					else
					{
						erCause = VSRCH_SRCHERR_SSSFAIL;
					}
					
				}
				else //compensate timing and retry synch.
				{
					
					AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedRegPtr->sharedMutex)), "");
					vrfsrch_sharedRegPtr->timeOffset = VSRCH_NB_SSBSYMB*(cfgInfo.fftSize + cfgInfo.nbCp); //push SSB symbols and retry
					AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedRegPtr->sharedMutex)), "");
				}

			}
			else
			{
				erCause = VSRCH_SRCHERR_PSSFAIL;
			}

			//wrap up the srch result
			{
				bool terminateSrch = false;

				if (pbchRes != NULL) //if PBCH CRC is OK -> apply freq offset/time offset or make report
				{
					//DMRS imp : 2. calculate frame boundary (save in frame_offset)
					uint8_t nbSymbBackward;
					pos_slotBound = (cfgInfo.samplesPerSubframe/cfgInfo.slotsPerSubframe) - nbSymbBackward*(cfgInfo.fftSize + cfgInfo.nbCp);
					rx_offset = ssb_offset - pos_slotBound;

					//LOG_E(PHY, "rx offset :%i, ssb offset:%i, pos_slotBound:%i, symBackward:%i\n", rx_offset, ssb_offset, pos_slotBound, nbSymbBackward);
					
					if (cfgInfo.mode == srchMode_full)
					{
						int freq_offset_cfg = f_off;
						//DMRS imp : 2. calculate frame boundary (save in frame_offset)
						int frame_offset;


						
						
						//making results
						vsrch_makeSrchReport(bankId, cellId, frame_offset, freq_offset_cfg, ssb_index, nhf, pbchRes, Lmax, extra_byte);
						
						terminateSrch = true;
						
					}
					else if (cfgInfo.mode == srchMode_online)
					{				
						//online compensation
						AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedRegPtr->sharedMutex)), "");
						vrfsrch_sharedRegPtr->timeOffset = rx_offset - slot_offset; 
						vrfsrch_sharedRegPtr->freqOffset = f_off;
						AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedRegPtr->sharedMutex)), "");

					}
				}
				else if (cfgInfo.mode == srchMode_online)
				{
					LOG_E(PHY, "[vSRCH] Online srch fail! (cause : %s)\n", vsrch_print_srchError[erCause]);
				}

				
				//check end condition for the full search mode
				if (terminateSrch == false && cfgInfo.mode == srchMode_full)
				{
					if (cfgInfo.srchCnt > 0)
					{
						cfgInfo.srchCnt -= HW_NB_SUBF_IN_FRAME;
						if (cfgInfo.srchCnt < 0)
						{
							//termination
							terminateSrch = true;
						}
					}
					
					if (cfgInfo.duration > 0)
					{
						int cur_duration = (int)((rdtsc_oai() - cfgInfo.srchStartTime)/cpuf/1000000.0);
						if (cur_duration > cfgInfo.duration)
						{
							terminateSrch = true;
						}
					}
				}

					//termination is done here for full search mode
				if (terminateSrch == true)
				{
					//make it off
					vsrch_forceOff(bankId);
					
					//then interrupt
					AssertFatal( 0 == pthread_mutex_lock(vsrch_muPtr_srchHal[bankId]), "");
					AssertFatal( 0 == pthread_cond_signal(vsrch_csPtr_srchIrq[bankId]), "");
					AssertFatal( 0 == pthread_mutex_unlock(vsrch_muPtr_srchHal[bankId]), "");
				}
			}
			
			AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_sharedRegPtr->sharedMutex)), "");
			vrfsrch_sharedRegPtr->processBitmap = vrfsrch_sharedRegPtr->processBitmap & 0xFE; //only bank 0 is used currently, this should be fixed further if we want to use multi-banks
			AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_sharedRegPtr->sharedMutex)), "");
			if (cfgInfo.mode == srchMode_online)
				vsrch_mesTime = 0;

		}


		AssertFatal ( 0== pthread_mutex_lock(vth_halMutex), "");
		vth_roReg->hwStatus = srchst_off;
		AssertFatal ( 0== pthread_mutex_unlock(vth_halMutex), "");
		AssertFatal ( 0== pthread_mutex_lock(&(vrfsrch_srchReg.roReg.regMutex)), "");
		vrfsrch_srchReg.roReg.hwStatus = hwIx_off;
		AssertFatal ( 0== pthread_mutex_unlock(&(vrfsrch_srchReg.roReg.regMutex)), "");
		LOG_E(PHY, "[vSRCH] virtual H/W off now\n");
	}  // while !oai_exit

	return &UE_thread_synch_retval;
}




