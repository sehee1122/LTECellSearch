#include "vSrch_ssbCommon.h"
#include "vSrch_ssbFft.h"




void *vsrch_getIdft(int ofdm_symbol_size)
{
  void (*idft)(int16_t *,int16_t *, int);

  switch (ofdm_symbol_size) {
    case 128:
      idft = idft128;
      break;

    case 256:
      idft = idft256;
      break;

    case 512:
      idft = idft512;
      break;

    case 1024:
      idft = idft1024;
      break;

    case 1536:
      idft = idft1536;
      break;

    case 2048:
      idft = idft2048;
      break;

    case 4096:
      idft = idft4096;
      break;

    case 8192:
      idft = idft8192;
      break;

    default:
      printf("function get_idft : unsupported ofdm symbol size \n");
      assert(0);
      break;
 }
 return idft;
}



int vsrch_ssbSlotFft(int** rxData,
							uint16_t fftSize,
							uint16_t nbCp,
							uint32_t samplePerSlot,
							unsigned int frame_length_samples,
							unsigned char symbol,
							unsigned char Ns,							
							int** ssbFData)
{
	unsigned char aa;
	unsigned int slot_offset;
	unsigned int rx_offset;

	if (Ns<0 || Ns>=20)
	{
		printf("slot_fep: Ns must be between 0 and 19\n");
		return(-1);
	}

	
	void (*dft)(int16_t *,int16_t *, int);
	int tmp_dft_in[8192] __attribute__ ((aligned (32)));  // This is for misalignment issues for 6 and 15 PRBs

	switch (fftSize)
	{
		case 128:
			dft = dft128;
			break;
			
		case 256:
			dft = dft256;
			break;

		case 512:
			dft = dft512;
			break;

		case 1024:
			dft = dft1024;
			break;

		case 1536:
			dft = dft1536;
			break;

		case 2048:
			dft = dft2048;
			break;

		case 4096:
			dft = dft4096;
			break;

		case 8192:
			dft = dft8192;
			break;

		default:
			dft = dft512;
			break;
	}
	
	slot_offset = samplePerSlot * Ns;

	for (aa=0; aa<HW_NB_RXANT; aa++)
	{
		memset(&ssbFData[aa][fftSize*symbol], 0, fftSize*sizeof(int));

		rx_offset = slot_offset + nbCp;// - SOFFSET;
		rx_offset += (fftSize+nbCp)*symbol;

		if (rx_offset > (frame_length_samples - fftSize))
		{
			memcpy(tmp_dft_in, &rxData[aa][rx_offset], (frame_length_samples - rx_offset) );
			memcpy((short *)&tmp_dft_in[frame_length_samples - rx_offset], (short *)&rxData[aa][0], (fftSize - (frame_length_samples - rx_offset))*sizeof(int));
		}
		else
		{
			memcpy((void *)tmp_dft_in, (void *)&rxData[aa][rx_offset % frame_length_samples], fftSize*sizeof(int));
		}
				
		dft((int16_t *)tmp_dft_in, (int16_t *)&ssbFData[aa][fftSize*symbol], 1);
	}

	return(0);
}



int vsrch_ssbFft(int** rxData, uint16_t fftSize, uint16_t nbCp, 
					  uint32_t samplePerSlot,
					  unsigned int samplePerFrame, int** ssbFData, uint8_t symbolBitmap)
{
	for (int i=0;i<VSRCH_NB_SSBSYMB;i++)
	{
		if ((symbolBitmap & (0x01 << i)) > 0)
		{
			vsrch_ssbSlotFft(rxData, fftSize, nbCp, samplePerSlot, samplePerFrame, i, 0, ssbFData);
		}
	}

	return 0;
}


