#include "vThread_HWCommon.h"
#include "PHY/defs_nr_UE.h"


#define VRF_NB_MAXSAMPLES		HW_MAX_SAMPLERATE/100 //number of samples (maximal) after RF layer -> assume it is 10ms samples
#define VRF_NB_SAMPLEBANK		4



#define VRF_SAMPLEMODE_192		0
#define VRF_SAMPLEMODE_768		1
#define VRF_SAMPLEMODE_1536		2
#define VRF_SAMPLEMODE_3072		3

#define VRF_RXMODE_SRCH			0
#define VRF_RXMODE_SYNC			1

#define VRF_TXMODE_INSYNC		0
#define VRF_TXMODE_OUTSYNC		1




typedef enum {
	rfst_null=0,
	rfst_off=1,
	rfst_on=2
} rfHwStatus_e;


//register definition
//1. read only registers
typedef struct {
	rfHwStatus_e hwStatus; //on/off indication
	int digital_freq_offset;
	const int32_t*** ptr_rxdata;
	const int32_t*** ptr_txdata;

} halRfic_roReg_t;

//2. write only
typedef struct {

	uint8_t onoff;

	/*registers which cannot be checked in real-time (only checked at the on/off timing) */
	uint8_t samplingMode;
	uint32_t samples_per_slot;
	uint32_t samples_per_frame;
	
  	double rx_freq[4];//! \brief Center frequency in Hz for RX. index: [0..rx_num_channels[
  	double tx_freq[4]; //! \brief Center frequency in Hz for TX. index: [0..rx_num_channels[ !!! see lte-ue.c:427 FIXME iterates over rx_num_channels
  	double rx_bw; //! RX bandwidth in Hz
  	double tx_bw;

	/* registers which can be checked in real-time */
	int tx_sample_advance;
	int freq_offset;
	
  	double rx_gain[4];//! \brief Gain for RX in dB. index: [0..rx_num_channels]
  	double rx_gain_offset[4];//! \brief Gain offset (for calibration) in dB index: [0..rx_num_channels]
  	double tx_gain[4]; //! gain for TX in dB
	int rx_offset;

	uint8_t rxMode; //RF mode for RX processing (SRCH mode : searching state (get sample by frame), SYNC mode : in synch state (get sample by slot))
	uint8_t txMode; //RF mode for TX processing (insync : normally transmitting power, outsync : temporarily blocking TX)
} halRfic_woReg_t;

typedef struct {
	/* registers which can be checked in real-time */
	int sync_offset; //give RX timing offset at RF level
	uint8_t gainChanged; //indication of gain configuration change 
} halRfic_rwReg_t;




typedef struct {
	

	halRfic_roReg_t roReg;
	halRfic_woReg_t woReg;
	halRfic_rwReg_t rwReg;
	
	pthread_mutex_t mutex_rfHal;
	pthread_cond_t cond_rfHal;
	
	PHY_VARS_NR_UE *UE;

	
} ix_halRfic_t;





void *vRFIC_mainThread(void* arg);
void vrf_initHwParams(unsigned int Vmmapped_dmam, int VclkSrc, char* inConfigFile, char* inUsrpArg);

