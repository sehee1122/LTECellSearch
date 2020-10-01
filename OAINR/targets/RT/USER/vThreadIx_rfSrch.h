#include "vThread_HWCommon.h"
#include "PHY/defs_nr_UE.h"


//registers in RF side
typedef struct { //read only
	hwIxStatus_e				hwStatus;
	int***						rxData;		//rf IQ data [bank][antenna][time]
	int16_t*					hfslot_nb;		//timing of the rf data [bank] (sfn*10 + subframe)
	int* 						hfframe_nb;		//timing of the rf data [bank] (sfn*10 + subframe)
	int*						slot_offset;
	pthread_mutex_t 			regMutex;
} vrfSrch_rfRoReg_t;



typedef struct {
	vrfSrch_rfRoReg_t roReg;
	//vrfSrch_rfRwReg_t rwReg;
} vrfSrch_rfReg_t;





//register in srcher side
#if 0
typedef struct { //read and write
	//wo
	uint8_t 					processBitmap; //number of srch banks that is processing
	pthread_mutex_t 			regMutex;
} vrfSrch_srchRwReg_t;
#endif

typedef struct { //write only
	pthread_cond_t 				irq_srchInst;
} vrfSrch_srchWoReg_t;

typedef struct { //read only
	hwIxStatus_e				hwStatus; //
	pthread_mutex_t 			regMutex;
} vrfSrch_srchRoReg_t;



typedef struct {
	//vrfSrch_srchRwReg_t rwReg;
	vrfSrch_srchWoReg_t woReg;
	vrfSrch_srchRoReg_t roReg;
	
} vrfSrch_srchReg_t;


typedef struct {
	uint8_t 					processBitmap;  //data bank that is ready and processed or not
	int							freqOffset;		//current frequency offset
	int							timeOffset;		//current timing offset status		
	pthread_mutex_t 			sharedMutex;
} vrfSrch_sharedReg_t;


vrfSrch_srchReg_t* vsrch_configSrchReg(void);
vrfSrch_rfReg_t* vrf_configRfReg(void);
vrfSrch_sharedReg_t* vrf_configSrchSharedReg(void);


