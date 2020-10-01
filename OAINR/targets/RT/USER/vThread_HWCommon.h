#include <stdint.h>
#include <pthread.h>
#include "rt_wrapper.h"
#include "assertions.h"
#include <stdbool.h>
#include "openair1/PHY/TOOLS/tools_defs.h"



#define HW_NB_TXANT					1
#define HW_NB_RXANT					1
#define HW_NB_SUBF_IN_FRAME			10
#define HW_NB_SYM_IN_SLOT			14

#define HW_NB_RE_IN_RB				12
#define HW_NB_DMRSRE_IN_RB			3
#define HW_OFFSET_DMRSRE_IN_RB		{0, 4, 8}
#define HW_MAX_NARFCN_3GHZ			599999
#define HW_MAX_NARFCN_6GHZ			1199999
#define HW_MAX_NARFCN_24GHZ			2016666



#define HW_GAP_TXRX					4 							//number of subframe between TX and RX -> this should be further enhanced when adding a feature of flexible TX_RX gap
#define HW_MAX_FRAMENUM 			0x400
#define HW_MAX_SAMPLERATE			30720000
#define HW_MAX_FFTSIZE				4096
#define HW_MAX_NBCP					352


#define HW_NB_HALFFRAME				2
#define	HW_LMAX						64



#define VHW_MAX_NUMEROLOGY			5

#define VHW_PI 						3.14159265358979323846



//#define HW_NB_MAXOFDMSYM			2048
//#define HW_NB_MAXOFDMCP			166





#define HW_RX_NB_TH					2 							//number of processes for channel encoding/decoding
#define HW_FIFO_PRIORITY   			40

#define HW_IQSIZE					2

#define HW_NB_SLOT_IN_SUBF 			{1, 2, 4, 8, 16} 			//index : mu(numerology)
#define HW_NB_SLOT_IN_FRAME 		{10, 20, 40, 80, 160}		//index : mu(numerology)


#ifndef __VHW_COMMON_ENUM_STRUCT
#define __VHW_COMMON_ENUM_STRUCT

typedef enum {
	hwIx_null=0,
	hwIx_off=1,
	hwIx_on=2
} hwIxStatus_e;

#endif

#if 0
typedef struct {
	uint8_t numerology;
	uint16_t ofdm_symbol_size;
	uint32_t samples_per_subframe;
	uint8_t nb_cp;
	uint8_t nb_cp0;	
} vhw_frameFormat_t;
#endif

extern uint32_t vhw_subcarrier_spacing[VHW_MAX_NUMEROLOGY];
//const extern vhw_frameFormat_t* vhw_frame_parms;


#if 0
//HW - HW interface definition
typedef const uint16_t* inPort_u16t;
typedef uint16_t* outPort_u16t;


typedef const uint32_t* inPort_u32t;
typedef uint32_t* outPort_u32t;

typedef const int16_t* inPort_16t;
typedef int16_t* outPort_16t;


typedef const int32_t* inPort_32t;
typedef int32_t* outPort_32t;
#endif


void vhw_initvThread(int sched_runtime, int sched_deadline, int sched_fifo, cpu_set_t *cpuset, char *name);
void vhw_initTime(long long *sysTime);
void vhw_timeStamp(long long *sysTime, char* inName);
unsigned char vhw_log2Approx(unsigned int x);
uint8_t vhw_calcSlotPerSubframe(uint8_t in_numerology);
uint16_t vhw_calcNbPrefixSample (uint16_t fftSize);
uint32_t vhw_calcSamplesPerSubframe (uint16_t fftSize, uint8_t in_numerology);

int vhw_calcPbchChLevel(int **dl_ch_estimates_ext, uint16_t nb_rb, uint32_t symbol);
uint32_t vhw_generateGoldGeneric(uint32_t *x1, uint32_t *x2, uint8_t reset);
int64_t vhw_abs64(int64_t x);
void* vhw_malloc16_clear( size_t size );
void vhw_printData(int16_t* data, int size, char* string);
int vhw_calcSsbSlotNb(int narfcn, uint8_t numerology, uint8_t issb);
int vhw_calcSsbSymbNb(int narfcn, uint8_t numerology, uint8_t issb);


