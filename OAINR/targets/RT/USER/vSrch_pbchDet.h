#define VSRCH_FLAG_PBCHDET_GOLD			0x01
#define VSRCH_FLAG_PBCHDET_PILOT		0x02
#define VSRCH_FLAG_PBCHDET_CHEST		0x04
#define VSRCH_FLAG_PBCHDET_SSBDEXT		0x08
#define VSRCH_FLAG_PBCHDET_CHESTEXT		0x10
#define VSRCH_FLAG_PBCHDET_SSBDCOMP		0x20
#define VSRCH_FLAG_PBCHDET_ERX			0x40
#define VSRCH_FLAG_PBCHDET_A			0x80
#define VSRCH_FLAG_PBCHDET_APRIME		0x100
#define VSRCH_FLAG_PBCHDET_AINT			0x200

#define VSRCH_FLAG_PBCHDET_DECOUT		0x400
#define VSRCH_FLAG_PBCHDET_BDDMRS		0x800



#define VSRCH_FLAG_MAXNB				11
#define VSRCH_ISSB_NOTSET				255



void vsrch_genGoldPbch(uint16_t cellId, uint8_t Lmax);
uint8_t* vsrch_detectPbch(int16_t cellId, uint8_t Lmax, uint16_t fftSize, unsigned int ssb_Foffset, int high_speed_flag, int** ssbFData, uint8_t *issb, uint8_t *nhf, uint8_t *extra_byte, int option_print);
void vsrch_initPbchDetBuf(void);

