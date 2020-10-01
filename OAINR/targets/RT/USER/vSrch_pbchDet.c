#include "vThread_HWCommon.h"
#include "vSrch_ssbCommon.h"
#include "vSrch_pbchDet.h"
#include "vSrch_filt16a_32.h"
#include "limits.h"


#include "PHY/CODING/nrPolar_tools/nr_polar_defs.h"


#define VSRCH_LEN_PBCH_DMRS_DWORD 		10 // ceil(2(QPSK)*NR_PBCH_DMRS_LENGTH/32)
#define VSRCH_PBCH_CHEST_ALPHA			24576
#define VSRCH_MAXNB_DMRSSYM_IN_OSYM		60
#define VSRCH_NB_PBCH_ABYTE				32
#define VSRCH_NB_PBCH_PLDBIT			32

#define VSRCH_NB_MIB_BYTE				3
#define VSRCH_NB_PBCH_EBIT				864
#define VSRCH_POLAR_PBCH_AGGLEVEL		0
#define VSRCH_POLAR_PBCH_MSG_TYPE		0
#define VSRCH_POLAR_PBCH_AGGLEVEL_PRIME	0


#define VSRCH_DEC_UNSCRMASK_L64			0x100006D
#define VSRCH_DEC_UNSCRMASK				0x1000041

#define VSRCH_MOD_TABLE_QPSK_OFFSET		3

#define VSRCH_XTRABITPOS_NHF		4
#define VSRCH_XTRABITMASK_NHF		0x10



#define cmax(a,b)  ((a>b) ? (a) : (b))
#define max(a,b) cmax(a,b)



/* Buffers ------------------------------------------ */

//constants
short vsrch_dmrs_mod_table[14]  = {0,0,23170,-23170,-23170,23170,23170,-23170,23170,23170,-23170,-23170,-23170,23170}; //fixed array
uint8_t vsrch_pbch_deinterleaving_pattern[32] = {28,0,31,30,7,29,25,27,5,8,24,9,10,11,12,13,1,4,3,14,15,16,17,2,26,18,19,20,21,22,6,23};



int vsrch_flag_pbchDetBufSet = 0; //buffer set status

//signals
uint32_t vsrch_goldPbch[HW_NB_HALFFRAME][HW_LMAX][VSRCH_LEN_PBCH_DMRS_DWORD] __attribute__((aligned(16))); //gold seq buffer
int vsrch_pbchPilot[VSRCH_MAXNB_DMRSSYM_IN_OSYM] __attribute__((aligned(16))); //generated DMRS seq
int vsrch_pbchPilotFull[VSRCH_NB_DMRSSYMS] __attribute__((aligned(16))); //generated DMRS seq
int vsrch_dmrsFullShift=0;


//channel estimation
int** vsrch_ChEst = NULL;

int vsrch_ssb_Fdata_ext[HW_NB_RXANT][HW_NB_RE_IN_RB*VSRCH_NB_RB_IN_SSB*VSRCH_NB_SSBSYMB] __attribute__((aligned(16)));
int** vsrch_ssb_chEst_ext = NULL;
int vsrch_ssb_Fdata_comp[HW_NB_RXANT][HW_NB_RE_IN_RB*VSRCH_NB_RB_IN_SSB*VSRCH_NB_SSBSYMB] __attribute__((aligned(16)));

int vsrch_pbchDmrsSym[HW_NB_RXANT][VSRCH_NB_DMRSSYMS];

//Decoder
int16_t __attribute__((aligned(16))) vsrch_pbch_e_rx[VSRCH_NB_PBCH_EBIT]; //(int16_t*)malloc16_clear( 1920 );
uint8_t vsrch_pbch_a[VSRCH_NB_PBCH_ABYTE];
static uint8_t vsrch_pbchDecoded[VSRCH_NB_MIB_BYTE];

uint32_t vsrch_pbch_a_prime;
uint32_t vsrch_pbch_a_interleaved;

t_nrPolar_params* vsrch_polarList = NULL;



/* Buffers ------------------------------------------ */

void vsrch_initPbchDetBuf(void)
{
	if (vsrch_ChEst == NULL)
	{
		vsrch_ChEst = (int**)vhw_malloc16_clear(HW_NB_RXANT*sizeof(int*));
		for (int i=0;i<HW_NB_RXANT;i++)
		{
			vsrch_ChEst[i] = (int*)vhw_malloc16_clear(VSRCH_NB_SSBSYMB*HW_MAX_FFTSIZE*sizeof(int));
		}
	}

	if (vsrch_ssb_chEst_ext == NULL)
	{
		vsrch_ssb_chEst_ext = (int**)vhw_malloc16_clear(HW_NB_RXANT*sizeof(int*));
		for (int i=0;i<HW_NB_RXANT;i++)
		{
			vsrch_ssb_chEst_ext[i] = (int*)vhw_malloc16_clear(HW_NB_RE_IN_RB*VSRCH_NB_RB_IN_SSB*VSRCH_NB_SSBSYMB*sizeof(int));
		}
	}
}




void vsrch_clearPbchDetBuf(int bufFlag)
{
	int flag_case;
	int setFlag = 1;
	for (int i=0;i<VSRCH_FLAG_MAXNB;i++)
	{
		flag_case = (0x01<<i);
		if ( (bufFlag & flag_case) && 
			 (vsrch_flag_pbchDetBufSet & flag_case) )
		{
			switch(flag_case)
			{
				case VSRCH_FLAG_PBCHDET_GOLD:
					for (int k1 = 0;k1 < HW_NB_HALFFRAME;k1++)
						for (int k2 = 0;k2 < HW_LMAX;k2++)
							memset(&vsrch_goldPbch[k1][k2][0], 0,  VSRCH_LEN_PBCH_DMRS_DWORD * sizeof(int32_t));
					break;
					
				case VSRCH_FLAG_PBCHDET_PILOT:
					memset(&vsrch_pbchPilot[0], 0,  VSRCH_MAXNB_DMRSSYM_IN_OSYM * sizeof(int));
					break;
					
				case VSRCH_FLAG_PBCHDET_CHEST:
					for (int k1 = 0;k1 < HW_NB_RXANT;k1++)
						memset(&vsrch_ChEst[k1][0], 0, VSRCH_NB_SSBSYMB*HW_MAX_FFTSIZE*sizeof(int));
					break;
					
				case VSRCH_FLAG_PBCHDET_SSBDEXT:
					for (int k1 = 0;k1 < HW_NB_RXANT;k1++)
						memset(&vsrch_ssb_Fdata_ext[k1][0], 0, HW_NB_RE_IN_RB*VSRCH_NB_RB_IN_SSB*VSRCH_NB_SSBSYMB*sizeof(int));
					break;
					
				case VSRCH_FLAG_PBCHDET_CHESTEXT:
					for (int k1 = 0;k1 < HW_NB_RXANT;k1++)
						memset(&vsrch_ssb_chEst_ext[k1][0], 0, HW_NB_RE_IN_RB*VSRCH_NB_RB_IN_SSB*VSRCH_NB_SSBSYMB*sizeof(int));
					break;
					
				case VSRCH_FLAG_PBCHDET_SSBDCOMP:
					for (int k1 = 0;k1 < HW_NB_RXANT;k1++)
						memset(&vsrch_ssb_Fdata_comp[k1][0], 0, HW_NB_RE_IN_RB*VSRCH_NB_RB_IN_SSB*VSRCH_NB_SSBSYMB*sizeof(int));
					break;


				case VSRCH_FLAG_PBCHDET_ERX:
					memset(&vsrch_pbch_e_rx[0], 0, VSRCH_NB_PBCH_EBIT*sizeof(int16_t)); //(int16_t*)malloc16_clear( 1920 );
					break;

				case VSRCH_FLAG_PBCHDET_A:
					memset(&vsrch_pbch_a[0], 0, VSRCH_NB_PBCH_ABYTE*sizeof(uint8_t));
					break;
					
				case VSRCH_FLAG_PBCHDET_APRIME:
					vsrch_pbch_a_prime = 0;
					break;
					
				case VSRCH_FLAG_PBCHDET_AINT:
					vsrch_pbch_a_interleaved = 0;
					break;

				case VSRCH_FLAG_PBCHDET_DECOUT:
					memset(&vsrch_pbchDecoded[0], 0, VSRCH_NB_MIB_BYTE*sizeof(uint8_t));
					break;
				case VSRCH_FLAG_PBCHDET_BDDMRS:
					for (int k1 = 0;k1 < HW_NB_RXANT;k1++)
						memset(&vsrch_pbchDmrsSym[k1][0], 0, VSRCH_NB_DMRSSYMS*sizeof(int));
					break;
				default:
					setFlag = 0;
					break;
			}
			
			if (setFlag == 1)
			{
				vsrch_flag_pbchDetBufSet |= ~(flag_case);
			}
		}
	}
	
}



void vsrch_genGoldPbch(uint16_t cellId, uint8_t Lmax)
{

	unsigned int n, x1, x2;
	unsigned int Nid, i_ssb, i_ssb2;
	unsigned char l, n_hf, N_hf;

	Nid = cellId;

	N_hf = (Lmax == 4)? 2:1;

	for (n_hf = 0; n_hf < N_hf; n_hf++)
	{
		for (l = 0; l < Lmax ; l++)
		{
			i_ssb = l & (Lmax-1);
			i_ssb2 = (i_ssb<<2) + n_hf;

			x1 = 1 + (1<<31);
			x2 = (1<<11) * (i_ssb2 + 1) * ((Nid>>2) + 1) + (1<<6) * (i_ssb2 + 1) + (Nid&3);
			x2 = x2 ^ ((x2 ^ (x2>>1) ^ (x2>>2) ^ (x2>>3))<<31);

			// skip first 50 double words (1600 bits)
			for (n = 1; n < 50; n++)
			{
				x1 = (x1>>1) ^ (x1>>4);
				x1 = x1 ^ (x1<<31) ^ (x1<<28);
				x2 = (x2>>1) ^ (x2>>2) ^ (x2>>3) ^ (x2>>4);
				x2 = x2 ^ (x2<<31) ^ (x2<<30) ^ (x2<<29) ^ (x2<<28);
			}

			for (n=0; n<VSRCH_LEN_PBCH_DMRS_DWORD; n++)
			{
				x1 = (x1>>1) ^ (x1>>4);
				x1 = x1 ^ (x1<<31) ^ (x1<<28);
				x2 = (x2>>1) ^ (x2>>2) ^ (x2>>3) ^ (x2>>4);
				x2 = x2 ^ (x2<<31) ^ (x2<<30) ^ (x2<<29) ^ (x2<<28);
				vsrch_goldPbch[n_hf][l][n] = x1 ^ x2;
			}
		}
	}

	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_GOLD;
}




int vsrch_genPbchDmrsFullSeq(unsigned int *nr_gold_pbch)
{
	int m;
	uint8_t idx=0;
	
	int16_t *output = (int16_t*)&vsrch_pbchPilotFull[0];
	memset(output, 0, VSRCH_NB_DMRSSYMS*sizeof(int));
	
	/// QPSK modulation
	for (m=0; m<VSRCH_NB_DMRSSYMS; m++)
	{
		idx = ((((nr_gold_pbch[(m<<1)>>5])>>((m<<1)&0x1f))&1)<<1) ^ (((nr_gold_pbch[((m<<1)+1)>>5])>>(((m<<1)+1)&0x1f))&1);
		output[(m)<<1] = vsrch_dmrs_mod_table[(VSRCH_MOD_TABLE_QPSK_OFFSET + idx)<<1];
		output[((m)<<1)+1] = -vsrch_dmrs_mod_table[((VSRCH_MOD_TABLE_QPSK_OFFSET + idx)<<1) + 1]; //conjugate
	}

	int maxval=0;
	for (int i=0;i<HW_IQSIZE*VSRCH_NB_DMRSSYMS;i++)
	{
		maxval = max(maxval,output[i]);
		maxval = max(maxval,-output[i]);
	}
		
	vsrch_dmrsFullShift = vhw_log2Approx(maxval);


	return(0);
}




int vsrch_genPbchSymDmrsSeq(int symbol,unsigned int *nr_gold_pbch)
{
	int m,m0,m1;
	uint8_t idx=0;
	
	AssertFatal(symbol>=0 && symbol <3,"illegal symbol %d\n",symbol);


	int *output = &vsrch_pbchPilot[0];
	memset(output, 0, VSRCH_MAXNB_DMRSSYM_IN_OSYM*sizeof(int));

	if (symbol == 0)
	{
		m0=0;
		m1=60;
	}
	else if (symbol == 1)
	{
		m0=60;
		m1=84;
	}
	else
	{
		m0=84;
		m1=144;
	}
	
	/// QPSK modulation
	for (m=m0; m<m1; m++)
	{
		idx = ((((nr_gold_pbch[(m<<1)>>5])>>((m<<1)&0x1f))&1)<<1) ^ (((nr_gold_pbch[((m<<1)+1)>>5])>>(((m<<1)+1)&0x1f))&1);
		((int16_t*)output)[(m-m0)<<1] = vsrch_dmrs_mod_table[(VSRCH_MOD_TABLE_QPSK_OFFSET + idx)<<1];
		((int16_t*)output)[((m-m0)<<1)+1] = vsrch_dmrs_mod_table[((VSRCH_MOD_TABLE_QPSK_OFFSET + idx)<<1) + 1];
	}

	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_PILOT;


	return(0);
}














static int vsrch_intcmp(const void *p1,const void *p2)
{
  	return(*(int16_t *)p1 > *(int16_t *)p2);
}

static void vsrch_polarInit(void)
{
	t_nrPolar_params *currentPtr = vsrch_polarList;
	uint16_t aggregation_prime = VSRCH_POLAR_PBCH_AGGLEVEL_PRIME;

	//Parse the list. If the node is already created, return without initialization.
	while (currentPtr != NULL)
	{
		return;
	}

	//Else, initialize and add node to the end of the linked list.
	t_nrPolar_params *newPolarInitNode = malloc(sizeof(t_nrPolar_params));

	if (newPolarInitNode != NULL)
	{
		newPolarInitNode->idx = (VSRCH_POLAR_PBCH_MSG_TYPE * VSRCH_NB_PBCH_PLDBIT * aggregation_prime);
		newPolarInitNode->nextPtr = NULL;
	
		//PBCH
		newPolarInitNode->n_max = NR_POLAR_PBCH_N_MAX;
		newPolarInitNode->i_il = NR_POLAR_PBCH_I_IL;
		newPolarInitNode->i_seg = NR_POLAR_PBCH_I_SEG;
		newPolarInitNode->n_pc = NR_POLAR_PBCH_N_PC;
		newPolarInitNode->n_pc_wm = NR_POLAR_PBCH_N_PC_WM;
		newPolarInitNode->i_bil = NR_POLAR_PBCH_I_BIL;
		newPolarInitNode->crcParityBits = NR_POLAR_PBCH_CRC_PARITY_BITS;
		newPolarInitNode->payloadBits = NR_POLAR_PBCH_PAYLOAD_BITS;
		newPolarInitNode->encoderLength = NR_POLAR_PBCH_E;
		newPolarInitNode->crcCorrectionBits = NR_POLAR_PBCH_CRC_ERROR_CORRECTION_BITS;
		newPolarInitNode->crc_generator_matrix = crc24c_generator_matrix(newPolarInitNode->payloadBits);//G_P
		

		newPolarInitNode->K = newPolarInitNode->payloadBits + newPolarInitNode->crcParityBits; // Number of bits to encode.
		newPolarInitNode->N = nr_polar_output_length(newPolarInitNode->K, newPolarInitNode->encoderLength, newPolarInitNode->n_max);
		newPolarInitNode->n = log2(newPolarInitNode->N);
		newPolarInitNode->G_N = nr_polar_kronecker_power_matrices(newPolarInitNode->n);
		//polar_encoder vectors:
		newPolarInitNode->nr_polar_crc = malloc(sizeof(uint8_t) * newPolarInitNode->crcParityBits);
		newPolarInitNode->nr_polar_aPrime = malloc(sizeof(uint8_t) * ((ceil((newPolarInitNode->payloadBits)/32.0)*4)+3));
		newPolarInitNode->nr_polar_APrime = malloc(sizeof(uint8_t) * newPolarInitNode->K);
		newPolarInitNode->nr_polar_D = malloc(sizeof(uint8_t) * newPolarInitNode->N);
		newPolarInitNode->nr_polar_E = malloc(sizeof(uint8_t) * newPolarInitNode->encoderLength);
		//Polar Coding vectors
		newPolarInitNode->nr_polar_U = malloc(sizeof(uint8_t) * newPolarInitNode->N); //Decoder: nr_polar_uHat
		newPolarInitNode->nr_polar_CPrime = malloc(sizeof(uint8_t) * newPolarInitNode->K); //Decoder: nr_polar_cHat
		newPolarInitNode->nr_polar_B = malloc(sizeof(uint8_t) * newPolarInitNode->K); //Decoder: nr_polar_bHat
		newPolarInitNode->nr_polar_A = malloc(sizeof(uint8_t) * newPolarInitNode->payloadBits); //Decoder: nr_polar_aHat
		newPolarInitNode->Q_0_Nminus1 = nr_polar_sequence_pattern(newPolarInitNode->n);
		newPolarInitNode->interleaving_pattern = malloc(sizeof(uint16_t) * newPolarInitNode->K);
		nr_polar_interleaving_pattern(newPolarInitNode->K,
		                              newPolarInitNode->i_il,
		                              newPolarInitNode->interleaving_pattern);
		newPolarInitNode->deinterleaving_pattern = malloc(sizeof(uint16_t) * newPolarInitNode->K);

		for (int i=0; i<newPolarInitNode->K; i++)
			newPolarInitNode->deinterleaving_pattern[newPolarInitNode->interleaving_pattern[i]] = i;

		newPolarInitNode->rate_matching_pattern = malloc(sizeof(uint16_t) * newPolarInitNode->encoderLength);
		uint16_t *J = malloc(sizeof(uint16_t) * newPolarInitNode->N);
		nr_polar_rate_matching_pattern(newPolarInitNode->rate_matching_pattern,
		                               J,
		                               nr_polar_subblock_interleaver_pattern,
		                               newPolarInitNode->K,
		                               newPolarInitNode->N,
		                               newPolarInitNode->encoderLength);
		newPolarInitNode->information_bit_pattern = malloc(sizeof(uint8_t) * newPolarInitNode->N);
		newPolarInitNode->Q_I_N = malloc(sizeof(int16_t) * (newPolarInitNode->K + newPolarInitNode->n_pc));
		newPolarInitNode->Q_F_N = malloc( sizeof(int16_t) * (newPolarInitNode->N + 1)); // Last element shows the final array index assigned a value.
		newPolarInitNode->Q_PC_N = malloc( sizeof(int16_t) * (newPolarInitNode->n_pc));

		for (int i = 0; i <= newPolarInitNode->N; i++)
	  		newPolarInitNode->Q_F_N[i] = -1; // Empty array.

		nr_polar_info_bit_pattern(newPolarInitNode->information_bit_pattern,
		                          newPolarInitNode->Q_I_N,
		                          newPolarInitNode->Q_F_N,
		                          J,
		                          newPolarInitNode->Q_0_Nminus1,
		                          newPolarInitNode->K,
		                          newPolarInitNode->N,
		                          newPolarInitNode->encoderLength,
		                          newPolarInitNode->n_pc);
		
		// sort the Q_I_N array in ascending order (first K positions)
		qsort((void *)newPolarInitNode->Q_I_N,newPolarInitNode->K,sizeof(int16_t),vsrch_intcmp);
		newPolarInitNode->channel_interleaver_pattern = malloc(sizeof(uint16_t) * newPolarInitNode->encoderLength);
		nr_polar_channel_interleaver_pattern(newPolarInitNode->channel_interleaver_pattern,
		                                     newPolarInitNode->i_bil,
		                                     newPolarInitNode->encoderLength);
		free(J);
		build_decoder_tree(newPolarInitNode);
		build_polar_tables(newPolarInitNode);
		init_polar_deinterleaver_table(newPolarInitNode);
	}
	else 
	{
		AssertFatal(1 == 0, "[nr_polar_init] New t_nrPolar_params * could not be created");
	}

	newPolarInitNode->nextPtr = vsrch_polarList;
	vsrch_polarList = newPolarInitNode;
	
	return;
}





t_nrPolar_params *vsrch_setPolarParams (void)
{
	vsrch_polarInit();
	
	if (vsrch_polarList != NULL)
	{
		return vsrch_polarList;
	}
	else
	{
		AssertFatal(false,"Polar Init tables internal failure\n");
	}
	
	return NULL;
}



uint8_t vsrch_blindDetectIssb(uint16_t cellId, uint8_t Lmax, unsigned int ssb_Foffset, uint16_t fftSize, int **ssbFData, uint8_t *nhf_max)
{
	uint8_t issb_max=0;

	LOG_I(PHY, "DMRS blind detection result : nhf:%i, issb:%i\n", *nhf_max, issb_max);

	return issb_max;
}







int vsrch_estimatePbchChannel (uint16_t cellId,
										  unsigned int ssb_Foffset,
										  uint16_t fftSize,
									       unsigned char symbol,
									       uint32_t high_speed_flag,
									       int **ssbFData,
									       uint8_t ssb_index,
									       uint8_t n_hf)
{
	unsigned char aarx;
	unsigned short v;
	unsigned int pilot_cnt;
	int16_t ch[2],*pil,*rxF,*dl_ch,*fl,*fm,*fr;
	int ch_offset,symbol_offset;
	int dmrss;

	//uint8_t ssb_index=0, n_hf=0;

	
	AssertFatal((symbol > 0 && symbol < 4), "symbol %d is illegal for PBCH DM-RS\n", symbol); //2, 6 should be fixed
	v = cellId%4;

	if (ssb_Foffset >= fftSize) 
		ssb_Foffset -= fftSize;

	dmrss = symbol-1;

	if (high_speed_flag == 0) // use second channel estimate position for temporary storage
		ch_offset     = fftSize;
	else
		ch_offset     = fftSize*symbol;

	symbol_offset = fftSize*symbol + v;

	switch (v)
	{
		case 0:
			fl = vsrch_filt16a_l0;
			fm = vsrch_filt16a_m0;
			fr = vsrch_filt16a_r0;
			break;

		case 1:
			fl = vsrch_filt16a_l1;
			fm = vsrch_filt16a_m1;
			fr = vsrch_filt16a_r1;
			break;

		case 2:
			fl = vsrch_filt16a_l2;
			fm = vsrch_filt16a_m2;
			fr = vsrch_filt16a_r2;
			break;

		case 3:
			fl = vsrch_filt16a_l3;
			fm = vsrch_filt16a_m3;
			fr = vsrch_filt16a_r3;
			break;

		default:
			LOG_E(PHY, "pbch_channel_estimation: v=%d -> ERROR\n",v);
			return(-1);
			break;
	}



	// generate pilot
	vsrch_genPbchSymDmrsSeq(dmrss,vsrch_goldPbch[n_hf][ssb_index]);

	//Convolution starts
	int re_offset = ssb_Foffset;
	for (aarx=0; aarx<HW_NB_RXANT; aarx++)
	{
		pil   = (int16_t *)&vsrch_pbchPilot[0];
		rxF   = (int16_t *)&ssbFData[aarx][(symbol_offset+re_offset)];
		dl_ch = (int16_t *)&vsrch_ChEst[aarx][ch_offset];

		memset(dl_ch,0,sizeof(int)*fftSize);
		if (high_speed_flag==0) // multiply previous channel estimate by ch_est_alpha
	  		multadd_complex_vector_real_scalar(dl_ch-(fftSize<<1),
			                                     VSRCH_PBCH_CHEST_ALPHA, dl_ch-(fftSize<<1),
			                                     1,fftSize);

		// Treat first 2 pilots specially (left edge)
		ch[0] = (int16_t)(((int32_t)pil[0]*rxF[0] - (int32_t)pil[1]*rxF[1])>>15);
		ch[1] = (int16_t)(((int32_t)pil[0]*rxF[1] + (int32_t)pil[1]*rxF[0])>>15);
		multadd_real_vector_complex_scalar(fl,
									       ch,
									       dl_ch,
									       16);
		pil+=2;
		re_offset = (re_offset+4)&(fftSize-1);
		rxF   = (int16_t *)&ssbFData[aarx][(symbol_offset+re_offset)];

		//for (int i= 0; i<8; i++)
		//printf("dl_ch addr %p %d\n", dl_ch+i, *(dl_ch+i));

		ch[0] = (int16_t)(((int32_t)pil[0]*rxF[0] - (int32_t)pil[1]*rxF[1])>>15);
		ch[1] = (int16_t)(((int32_t)pil[0]*rxF[1] + (int32_t)pil[1]*rxF[0])>>15);
		multadd_real_vector_complex_scalar(fm,
									       ch,
									       dl_ch,
									       16);

		pil+=2;
		re_offset = (re_offset+4)&(fftSize-1);
		rxF   = (int16_t *)&ssbFData[aarx][(symbol_offset+re_offset)];

		ch[0] = (int16_t)(((int32_t)pil[0]*rxF[0] - (int32_t)pil[1]*rxF[1])>>15);
		ch[1] = (int16_t)(((int32_t)pil[0]*rxF[1] + (int32_t)pil[1]*rxF[0])>>15);
		multadd_real_vector_complex_scalar(fr,
					       ch,
					       dl_ch,
					       16);

		
		pil+=2;
		re_offset = (re_offset+4)&(fftSize-1);
		rxF   = (int16_t *)&ssbFData[aarx][(symbol_offset+re_offset)];
		dl_ch+=24;
		for (pilot_cnt=3; pilot_cnt<(3*20); pilot_cnt+=3)
		{
			// in 2nd symbol, skip middle  REs (48 with DMRS,  144 for SSS, and another 48 with DMRS) 
			if (dmrss == 1 && pilot_cnt == 12)
			{
				pilot_cnt=48;
				re_offset = (re_offset+144)&(fftSize-1);
				rxF   = (int16_t *)&ssbFData[aarx][(symbol_offset+re_offset)];
				dl_ch += 288;
		  	}
			ch[0] = (int16_t)(((int32_t)pil[0]*rxF[0] - (int32_t)pil[1]*rxF[1])>>15);
			ch[1] = (int16_t)(((int32_t)pil[0]*rxF[1] + (int32_t)pil[1]*rxF[0])>>15);
			multadd_real_vector_complex_scalar(fl,
												 ch,
												 dl_ch,
												 16);

		  pil+=2;
		  re_offset = (re_offset+4)&(fftSize-1);
		  rxF   = (int16_t *)&ssbFData[aarx][(symbol_offset+re_offset)];
		    

		  ch[0] = (int16_t)(((int32_t)pil[0]*rxF[0] - (int32_t)pil[1]*rxF[1])>>15);
		  ch[1] = (int16_t)(((int32_t)pil[0]*rxF[1] + (int32_t)pil[1]*rxF[0])>>15);
		  multadd_real_vector_complex_scalar(fm,
											 ch,
											 dl_ch,
											 16);

		  pil+=2;
		  re_offset = (re_offset+4)&(fftSize-1);
		  rxF   = (int16_t *)&ssbFData[aarx][(symbol_offset+re_offset)];
		    

		  ch[0] = (int16_t)(((int32_t)pil[0]*rxF[0] - (int32_t)pil[1]*rxF[1])>>15);
		  ch[1] = (int16_t)(((int32_t)pil[0]*rxF[1] + (int32_t)pil[1]*rxF[0])>>15);
		  multadd_real_vector_complex_scalar(fr,
											 ch,
											 dl_ch,
											 16);

		  pil+=2;
		  re_offset = (re_offset+4)&(fftSize-1);
		  rxF   = (int16_t *)&ssbFData[aarx][(symbol_offset+re_offset)];
		  dl_ch+=24;
		}
	}

	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_CHEST;	

	return(0);
}












uint16_t vsrch_extractPbch(int **rxdataF,
		                         int **dl_ch_estimates,
		                         uint8_t symbol,
		                         uint16_t cellId,
		                         uint32_t high_speed_flag,
		                         uint32_t ssb_Foffset,
		                         uint16_t fftSize)
{
	uint8_t nushiftmod4 = cellId%4;
	uint16_t rb;
	uint8_t i,j,aarx;
	int32_t *dl_ch0,*dl_ch0_ext,*rxF,*rxF_ext;
	uint8_t dmrs_offset[3] = HW_OFFSET_DMRSRE_IN_RB;
	unsigned int  rx_offset = ssb_Foffset;

	if (rx_offset >= fftSize)
		rx_offset -= fftSize;

	for (aarx=0; aarx<HW_NB_RXANT; aarx++)
	{
		rxF        = &rxdataF[aarx][symbol*fftSize];
		rxF_ext    = &vsrch_ssb_Fdata_ext[aarx][symbol*VSRCH_NB_RB_IN_SSB*HW_NB_RE_IN_RB];

		//1. symbol aggregation
		for (rb=0; rb<VSRCH_NB_RB_IN_SSB; rb++)
		{
	  		j=0;

	  		if (symbol==1 || symbol==3)
			{
	    		for (i=0; i<HW_NB_RE_IN_RB; i++)
				{
					if ((i!=nushiftmod4 + dmrs_offset[0]) && (i!=(nushiftmod4+dmrs_offset[1])) && (i!=(nushiftmod4+dmrs_offset[2])))
					{
						rxF_ext[j]=rxF[rx_offset];
						j++;
					}

	      			rx_offset=(rx_offset+1)&(fftSize-1);
	    		}

	    		rxF_ext += (HW_NB_RE_IN_RB - HW_NB_DMRSRE_IN_RB);
	
			}
			else
			{ //symbol 2
			    if ( (rb < VSRCH_OFFSET_SSSSTARTRB) || (rb > VSRCH_OFFSET_SSSENDRB) )
				{
					for (i=0; i<HW_NB_RE_IN_RB; i++)
					{
				        if ((i!=nushiftmod4 + dmrs_offset[0]) && (i!=(nushiftmod4+dmrs_offset[1])) && (i!=(nushiftmod4+dmrs_offset[2])))
						{
							rxF_ext[j]=rxF[rx_offset];
							j++;
				        }

			        	rx_offset=(rx_offset+1)&(fftSize-1);
			    	}

			      	rxF_ext += (HW_NB_RE_IN_RB - HW_NB_DMRSRE_IN_RB);
			    }
				else
					rx_offset = (rx_offset+HW_NB_RE_IN_RB)&(fftSize-1);
			}
		}

		if (high_speed_flag == 1)
	  		dl_ch0     = &dl_ch_estimates[aarx][(symbol*fftSize)];
		else
	  		dl_ch0     = &dl_ch_estimates[aarx][0];

		dl_ch0_ext = &vsrch_ssb_chEst_ext[aarx][symbol*VSRCH_NB_RB_IN_SSB*HW_NB_RE_IN_RB];

		for (rb=0; rb<VSRCH_NB_RB_IN_SSB; rb++)
		{
	  		j=0;

	  		if (symbol==1 || symbol==3)
			{
	    		for (i=0; i<HW_NB_RE_IN_RB; i++)
				{
	      			if ((i!=nushiftmod4 + dmrs_offset[0]) && (i!=(nushiftmod4+dmrs_offset[1])) && (i!=(nushiftmod4+dmrs_offset[2])))
					{
				        dl_ch0_ext[j]=dl_ch0[i];
				        j++;
	      			}
	    		}

			    dl_ch0 += HW_NB_RE_IN_RB;
			    dl_ch0_ext += (HW_NB_RE_IN_RB - HW_NB_DMRSRE_IN_RB);
	  		}
			else
			{
	    		if ( (rb < VSRCH_OFFSET_SSSSTARTRB) || (rb >VSRCH_OFFSET_SSSENDRB) )
				{
	      			for (i=0; i<HW_NB_RE_IN_RB; i++)
					{
	        			if ((i!=nushiftmod4 + dmrs_offset[0]) && (i!=(nushiftmod4+dmrs_offset[1])) && (i!=(nushiftmod4+dmrs_offset[2])))
						{
							dl_ch0_ext[j]=dl_ch0[i];
							j++;
	        			}
	      			}

	      			dl_ch0_ext += (HW_NB_RE_IN_RB - HW_NB_DMRSRE_IN_RB);
	    		}

	    		dl_ch0 += HW_NB_RE_IN_RB;
	  		}
		}
	}

	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_SSBDEXT;
	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_CHESTEXT;

	return(0);
}









void vsrch_compPbchChannel(uint32_t symbol, uint8_t output_shift)
{
	const uint16_t nb_re = symbol == 2 ? 72 : 180;
	
	AssertFatal((symbol > 0 && symbol < 4),
	          "symbol %d is illegal for PBCH DM-RS\n",
	          symbol);

	for (int aarx=0; aarx<HW_NB_RXANT; aarx++)
	{
		vect128 *dl_ch128          = (vect128 *)&vsrch_ssb_chEst_ext[aarx][symbol*VSRCH_NB_RB_IN_SSB*HW_NB_RE_IN_RB];
		vect128 *rxdataF128        = (vect128 *)&vsrch_ssb_Fdata_ext[aarx][symbol*VSRCH_NB_RB_IN_SSB*HW_NB_RE_IN_RB];
		vect128 *rxdataF_comp128   = (vect128 *)&vsrch_ssb_Fdata_comp[aarx][symbol*VSRCH_NB_RB_IN_SSB*HW_NB_RE_IN_RB];

		for (int re=0; re < nb_re; re+= HW_NB_RE_IN_RB)
		{
			*rxdataF_comp128++ = mulByConjugate128(rxdataF128++, dl_ch128++, output_shift);
			*rxdataF_comp128++ = mulByConjugate128(rxdataF128++, dl_ch128++, output_shift);
			*rxdataF_comp128++ = mulByConjugate128(rxdataF128++, dl_ch128++, output_shift);
		}
	}

	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_SSBDCOMP;
}














/* decoder ------------------------------------------ */

void vsrch_pbchUnscrambling(uint16_t Nid,
			                          uint8_t nushift,
			                          uint16_t M,
			                          uint16_t length,
			                          uint8_t bitwise,
			                          uint32_t unscrambling_mask)
{
	uint8_t reset, offset;
	uint32_t x1, x2, s=0;
	uint8_t k=0;
	reset = 1;
	x2 = Nid; //this is c_init

	// The Gold sequence is shifted by nushift* M, so we skip (nushift*M /32) double words
	for (int i=0; i<(uint16_t)ceil(((float)nushift*M)/32); i++)
	{
		s = vhw_generateGoldGeneric(&x1, &x2, reset);
		reset = 0;
	}

	// Scrambling is now done with offset (nushift*M)%32
	offset = (nushift*M)&0x1f;

	for (int i=0; i<length; i++)
	{
	
		if (bitwise)
		{
			if (((k+offset)&0x1f)==0 && (!((unscrambling_mask>>i)&1)))
			{
				s = vhw_generateGoldGeneric(&x1, &x2, reset);
				reset = 0;
			}

			(vsrch_pbch_a_interleaved) ^= ((unscrambling_mask>>i)&1)? ((vsrch_pbch_a_prime>>i)&1)<<i : (((vsrch_pbch_a_prime>>i)&1) ^ ((s>>((k+offset)&0x1f))&1))<<i;
			k += (!((unscrambling_mask>>i)&1));
		}
		else
		{
			if (((i+offset)&0x1f)==0)
			{
				s = vhw_generateGoldGeneric(&x1, &x2, reset);
				reset = 0;
			}

			if (((s>>((i+offset)&0x1f))&1)==1)
				vsrch_pbch_e_rx[i] = -vsrch_pbch_e_rx[i];

		}
	}
}








void vsrch_quantizePbch(int16_t *pbch_llr8, int16_t *pbch_llr, uint16_t len) {
	uint16_t i;

	for (i=0; i<len; i++)
	{
		if (pbch_llr[i]>31)
			pbch_llr8[i]=32;
		else if (pbch_llr[i]<-31)
			pbch_llr8[i]=-32;
		else
			pbch_llr8[i] = (char)(pbch_llr[i]);
	}
}





int vSrch_decodePbch(int **ssb_Fdata,
							int **ssb_chEst,
							uint8_t Lmax,
							uint16_t fftSize,
							uint32_t ssb_Foffset,
							uint16_t nid,
			                uint32_t high_speed_flag,
			                uint8_t issb,
			                uint8_t* nhf_mib,
			                uint8_t* xtra_byte)
{
	int log2_maxh=0;
	int symbol;
	uint8_t nushift;
	uint16_t M;
	uint32_t decoderState=0;
	int16_t* pbch_e_rx = &vsrch_pbch_e_rx[0];
	
	vsrch_clearPbchDetBuf(VSRCH_FLAG_PBCHDET_SSBDEXT |
						  VSRCH_FLAG_PBCHDET_CHESTEXT);
	*xtra_byte = 0;

	// symbol refers to symbol within SSB. symbol_offset is the offset of the SSB wrt start of slot
	for (symbol=1; symbol<4; symbol++)
	{
		nushift = nid%4;
 		vsrch_extractPbch(ssb_Fdata, ssb_chEst,
 						  symbol, nushift, high_speed_flag, ssb_Foffset, fftSize);
	
		if (symbol == 1)
		{
			log2_maxh = 3+(log2_approx(vhw_calcPbchChLevel((int**)vsrch_ssb_chEst_ext, VSRCH_NB_RB_IN_SSB, symbol))/2);
	  		
		}
		vsrch_compPbchChannel(symbol, log2_maxh);		

		if (symbol==2)
		{
		  	vsrch_quantizePbch(pbch_e_rx,
		                   (short *)&(vsrch_ssb_Fdata_comp[0][symbol*240]),
		                   144);
		  	pbch_e_rx+=144;
		}
		else
		{
		  	vsrch_quantizePbch(pbch_e_rx,
		                   (short *)&(vsrch_ssb_Fdata_comp[0][symbol*240]),
		                   360);
		  	pbch_e_rx+=360;
		}
	}
	
	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_ERX;


	//decoder
	vsrch_clearPbchDetBuf(VSRCH_FLAG_PBCHDET_A | 
 						  VSRCH_FLAG_PBCHDET_APRIME	| 
 						  VSRCH_FLAG_PBCHDET_AINT |
 						  VSRCH_FLAG_PBCHDET_DECOUT);

	
	
	//un-scrambling 1
	M = VSRCH_NB_PBCH_EBIT;// NR_POLAR_PBCH_E;
	nushift = (Lmax==4)? issb&0x03 : issb&0x07;
	vsrch_pbchUnscrambling(nid,nushift,M,VSRCH_NB_PBCH_EBIT,0,0);

	
	
	
	//polar decoding de-rate matching
	const t_nrPolar_params *currentPtr = vsrch_setPolarParams();
	if( (decoderState = polar_decoder_int16(vsrch_pbch_e_rx, (uint64_t *)&vsrch_pbch_a_prime, currentPtr)) != 0 )
		return(decoderState);
	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_APRIME;

	// Decoder reversal
	uint32_t a_reversed=0;

	for (int i=0; i<VSRCH_NB_PBCH_PLDBIT; i++)
		a_reversed |= (((uint64_t)vsrch_pbch_a_prime>>i)&1)<<(31-i);

	vsrch_pbch_a_prime = a_reversed;
	
	//payload un-scrambling 2
	M = (Lmax == 64)? (VSRCH_NB_PBCH_PLDBIT - 6) : (VSRCH_NB_PBCH_PLDBIT - 3);
	nushift = ((vsrch_pbch_a_prime>>24)&1) ^ (((vsrch_pbch_a_prime>>6)&1)<<1);
	uint32_t unscrambling_mask = (Lmax==64)? VSRCH_DEC_UNSCRMASK_L64:VSRCH_DEC_UNSCRMASK;
	
	vsrch_pbchUnscrambling(nid,nushift,M,VSRCH_NB_PBCH_PLDBIT,1,unscrambling_mask);
	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_AINT;

	//payload deinterleaving
	//uint32_t in=0;
	uint32_t out=0;

	for (int i=0; i<32; i++)
	{
		out |= ((vsrch_pbch_a_interleaved>>i)&1)<<(vsrch_pbch_deinterleaving_pattern[i]);
	}

	uint32_t payload = 0;
	//uint8_t xtra_byte = 0;
	*xtra_byte = (out>>(VSRCH_NB_MIB_BYTE*8))&0xff;

	for (int i=0; i<VSRCH_NB_PBCH_PLDBIT; i++)
		payload |= ((out>>i)&1)<<(VSRCH_NB_PBCH_PLDBIT-i-1);

	for (int i=0; i<3; i++)
		vsrch_pbchDecoded[i] = (uint8_t)((payload>>((3-i)<<3))&0xff);

	vsrch_flag_pbchDetBufSet |= VSRCH_FLAG_PBCHDET_DECOUT;

	*nhf_mib = (*xtra_byte & VSRCH_XTRABITMASK_NHF)>>VSRCH_XTRABITPOS_NHF;

	
	return 0;
}







uint8_t* vsrch_detectPbch(int16_t cellId, 
								 uint8_t Lmax, 
								 uint16_t fftSize, 
								 unsigned int ssb_Foffset, 
								 int high_speed_flag, 
								 int** ssbFData, 
								 uint8_t *issb, 
								 uint8_t *nhf,
								 uint8_t *extra_byte,
								 int option_print)
{
	uint8_t* decoded_output = NULL;
	int ret;
	uint8_t nhf_mib;
	uint8_t nhf_dmrs;
	
	vsrch_clearPbchDetBuf(VSRCH_FLAG_PBCHDET_GOLD |
						  VSRCH_FLAG_PBCHDET_PILOT |
						  VSRCH_FLAG_PBCHDET_CHEST);

	vsrch_genGoldPbch(cellId, Lmax);

	*issb = vsrch_blindDetectIssb(cellId, Lmax, ssb_Foffset, fftSize, ssbFData, nhf);
	LOG_E(PHY, "ISSB est : %i, nhf : %i\n", *issb, *nhf);



	//run channel estimator and store the H information
	for (int i=0;i<3;i++)
	{
    	vsrch_estimatePbchChannel(cellId, ssb_Foffset, fftSize, i+1, high_speed_flag, ssbFData, *issb, *nhf);
	}

	if ( (ret = vSrch_decodePbch(ssbFData,
								  vsrch_ChEst,
								  Lmax,
								  fftSize,
								  ssb_Foffset,
								  cellId,
					              high_speed_flag,
					              *issb,
					              &nhf_mib,
					              extra_byte) ) == 0)
	{
		decoded_output = vsrch_pbchDecoded;
		if (option_print == 1)
			LOG_E(PHY,"PBCH dec: pbch decoded sucessfully\n");
	} 
	else
	{
		if (option_print == 1)
			LOG_E(PHY,"PBCH dec: pbch decoded failed!\n");
	}
	
	return(decoded_output);
}

