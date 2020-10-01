#include <stdint.h>


typedef enum {
	nhal_sampleRate_192=0,
	nhal_sampleRate_768,		
	nhal_sampleRate_1536,
	nhal_sampleRate_3072
} nhal_sampleRate_e;


typedef enum {
	nhal_syncState_insync =0,
	nhal_syncState_outsync
} nhal_syncState_e;

typedef enum {
	nhal_freqOff_abs=0,
	nhal_freqOff_delta
} nhal_freqOff_e;

void NHAL_initAbstract(void);

int NHAL_RFcmd_init(void* UE, int mmapped_dma, int clock_source, char* configFilename, char* sdr_addrs);


int NHAL_RFcmd_on(nhal_sampleRate_e sr, 
						uint32_t samples_per_frame, uint32_t samples_per_slot, 
						double rxFreq, double txFreq,
						double rx_bw, double tx_bw,
						double rx_gain, double rx_gain_offset, double tx_gain,
						nhal_syncState_e syncState
						);

int NHAL_RFcmd_off(void);

int NHAL_RFcmd_cfgRxMode(nhal_syncState_e state);

int NHAL_RFcmd_cfgRxSync(int sync_offset);

int NHAL_RFcmd_cfgRxDrift(int rx_offset);

int NHAL_RFcmd_cfgTxAdvance(int tx_sample_advance);

nhal_sampleRate_e NHAL_convSampleRate(double rate);

int NHAL_RFcmd_cfgFreqOffset(int freq_offset, nhal_freqOff_e option);

int NHAL_RFcmd_readDigitalFreqOffset(void);
int NHAL_RFcmd_clrRxDrift(void);



