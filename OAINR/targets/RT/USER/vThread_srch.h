#include "vThread_HWCommon.h"
#include "PHY/defs_nr_UE.h"

#define VSRCH_MAXNB_SRCHBNK				4
#define VSRCH_DEFAULT_FULLSRCH_CNT		100

typedef enum {
	srchst_null=0,
	srchst_off=1,
	srchst_on=2
} srchHwStatus_e;



typedef enum srchMode {
	srchMode_full=0,
	srchMode_online
} halSrch_srchMode_e;


typedef struct srchReport {
	uint16_t nid;
	int frame_offset;
	int freq_offset;
	uint8_t ssb_index;
	uint8_t nhf;
	uint8_t pbchCrc;
	uint8_t pbchRes[3];
	uint8_t Lmax;
	uint8_t extraByte;
	void* next;
} srchReport_t;


//configuration registers for virtual searcher
typedef struct halSrch_woReg {
	
	//non real-time configurations
	halSrch_srchMode_e mode;		//full / online
	uint32_t narfcn;				//center frequency
	uint8_t numerology;				//u value
	uint16_t fftSize;				//bandwidth (in terms of FFT Size)
	uint16_t duration; 				//in ms, if 0 then search till something is found
	uint32_t srchSubframe;			//number of subframes to be searched (0 : no constraint)
	uint32_t ssb_Foffset;			//subcarrier index that the SSB starts
	int high_speed_flag;			//whether it is high speed or not
	//unsigned int ssbOffset;			//offset of the ssb that we look for
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
		
} halSrch_woReg_t;



//status registers for virtual searcher
typedef struct halSrch_roReg {
	srchHwStatus_e hwStatus;
	uint8_t srch_cnt;
	srchReport_t* srch_result;
	
} halSrch_roReg_t;




//read/write registers for virtual searcher
typedef struct halSrch_rwReg {
	uint8_t onoff;					//

} halSrch_rwReg_t;






typedef struct {
	
	halSrch_roReg_t roReg;
	halSrch_woReg_t woReg;
	halSrch_rwReg_t rwReg;
	
	pthread_mutex_t mutex_srchHal;
	pthread_cond_t  cond_srchHal;
	
	pthread_cond_t  cond_srchIrq;

} ix_halSrch_t;

void *vSRCH_mainThread(void *arg);

