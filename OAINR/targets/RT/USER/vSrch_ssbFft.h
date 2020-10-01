#include "vThread_HWCommon.h"


int vsrch_ssbFft(int** rxData, uint16_t fftSize, uint16_t nbCp, uint32_t samplePerSlot, unsigned int samplePerFrame, int** ssbFData, uint8_t symbolBitmap);

void *vsrch_getIdft(int ofdm_symbol_size);

