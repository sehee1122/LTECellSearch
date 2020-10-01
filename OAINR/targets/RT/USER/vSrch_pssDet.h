//#include "vSrch_ssbCommon.h"
#include "vThread_HWCommon.h"

void vsrch_initPssDet(unsigned int ssb_offset, int fftSize);
int32_t vsrch_detectPss (const int **rxdata, unsigned int length, uint16_t fftSize, unsigned int start, uint8_t *maxNid2, double *f_off, int option_print);

int vsrch_ChCompByPss(int32_t** rxPss, int32_t** comCh, uint8_t nid2);

