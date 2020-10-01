#include "vSrch_pssDet.h"
#include "vSrch_ssbFft.h"
#include "vSrch_ssbCommon.h"
#include "limits.h"


#define VSRCH_MAXNB_PSSCORR				307200
#define VSRCH_SCALING_PSS_NR			3

#define cmax(a,b)  ((a>b) ? (a) : (b))
#define max(a,b) cmax(a,b)


int vsrch_flag_pssDetBufSet = 0; //buffer set status


static int16_t vsrch_pssgen_time[VSRCH_NB_PSSSEQ][HW_IQSIZE*HW_MAX_FFTSIZE];
static int16_t vsrch_psspattern_comp[VSRCH_NB_PSSSEQ][HW_IQSIZE*VSRCH_LENGTH_PSS_NR];
static int16_t vsrch_psspattern[VSRCH_NB_PSSSEQ][VSRCH_LENGTH_PSS_NR];
static int vsrch_pssShift = 0;

static int16_t vsrch_ssproc_buf[HW_NB_RXANT*HW_MAX_FFTSIZE*HW_IQSIZE] __attribute__((aligned(32)));
static int16_t vsrch_ssfproc_buf[HW_MAX_FFTSIZE*HW_IQSIZE] __attribute__((aligned(32)));




void vsrch_calcPssShift(uint16_t fftSize)
{
	int maxval=0;
	
	for (int i=0;i<HW_IQSIZE*fftSize;i++)
	{
		maxval = max(maxval,vsrch_pssgen_time[0][i]);
		maxval = max(maxval,-vsrch_pssgen_time[0][i]);
		maxval = max(maxval,vsrch_pssgen_time[1][i]);
		maxval = max(maxval,-vsrch_pssgen_time[1][i]);
		maxval = max(maxval,vsrch_pssgen_time[2][i]);
		maxval = max(maxval,-vsrch_pssgen_time[2][i]);
	}
	
	vsrch_pssShift = vhw_log2Approx(maxval);
}



//void vsrch_generate_pss(NR_DL_FRAME_PARMS *fp, int N_ID_2)
void vsrch_generatePss(unsigned int ssb_offset, int fftSize, int N_ID_2)
{
	AssertFatal(N_ID_2>=0 && N_ID_2 <=2,"Illegal N_ID_2 %d\n",N_ID_2);
	
	int16_t d_pss[VSRCH_LENGTH_PSS_NR];
	int16_t x[VSRCH_LENGTH_PSS_NR];
	int16_t *primary_synchro_time = vsrch_pssgen_time[N_ID_2];
	unsigned int size = fftSize * HW_IQSIZE; /* i & q */
	
	int16_t *primary_synchro = vsrch_psspattern_comp[N_ID_2]; /* pss in complex with alternatively i then q */
	int16_t *primary_synchro2 = vsrch_psspattern[N_ID_2]; /* pss in complex with alternatively i then q */
	void (*idft)(int16_t *,int16_t *, int);
	const int x_initial[VSRCH_NB_INIT_PSS] = {0, 1, 1 , 0, 1, 1, 1};


	//generation of the PSS original patterns

	assert(N_ID_2 < VSRCH_NB_PSSSEQ);
	assert(size <= HW_MAX_FFTSIZE*HW_IQSIZE);
	assert(size <= HW_NB_RXANT*HW_MAX_FFTSIZE*HW_IQSIZE);

	bzero(vsrch_ssfproc_buf, size);
	bzero(vsrch_ssproc_buf, size);

	for (int i=0; i < VSRCH_NB_INIT_PSS; i++)
	{
		x[i] = x_initial[i];
	}

	for (int i=0; i < (VSRCH_LENGTH_PSS_NR - VSRCH_NB_INIT_PSS); i++)
	{
		x[i+VSRCH_NB_INIT_PSS] = (x[i + 4] + x[i])%(2);
	}

	for (int n=0; n < VSRCH_LENGTH_PSS_NR; n++)
	{
		int m = (n + 43*N_ID_2)%(VSRCH_LENGTH_PSS_NR);
		d_pss[n] = 1 - 2*x[m];
	}

	/* PSS is directly mapped to subcarrier without modulation 38.211 */
	for (int i=0; i < VSRCH_LENGTH_PSS_NR; i++)
	{
		primary_synchro[2*i] = (d_pss[i] * SHRT_MAX)>>VSRCH_SCALING_PSS_NR; /* Maximum value for type short int ie int16_t */
		primary_synchro[2*i+1] = 0;
		primary_synchro2[i] = d_pss[i];
	}

	//unsigned int  k = fp->first_carrier_offset + fp->ssb_start_subcarrier + 56; //and
	unsigned int  k = ssb_offset + VSRCH_SSB_PSS_OFFSET; //and
	if (k>= fftSize)
		k-=fftSize;

	for (int i=0; i < VSRCH_LENGTH_PSS_NR; i++)
	{
		vsrch_ssfproc_buf[2*k] = primary_synchro[2*i];
		vsrch_ssfproc_buf[2*k+1] = primary_synchro[2*i+1];

		k++;
		if (k == fftSize)
			k=0;
	}

	/* IFFT will give temporal signal of Pss */
	idft = vsrch_getIdft(fftSize);

	idft(vsrch_ssfproc_buf,          /* complex input */
	   	 vsrch_ssproc_buf,           /* complex output */
	     1);                 /* scaling factor */

	/* then get final pss in time */
	for (unsigned int i=0; i<fftSize; i++)
	{
		((int32_t *)primary_synchro_time)[i] = ((int32_t *)vsrch_ssproc_buf)[i];
	}


	vsrch_calcPssShift(fftSize);
}


void vsrch_initPssDet(unsigned int ssb_offset, int fftSize)
{
	for (int i=0;i<VSRCH_NB_PSSSEQ;i++)
	{
		vsrch_generatePss(ssb_offset, fftSize, i);
	}
}







int vsrch_ChCompByPss(int32_t** rxPss, int32_t** comCh, uint8_t nid2)
{
	int16_t *pssGen;
	int16_t *compCh16;
	int16_t *pss_orig,*sss_orig, tmp_re,tmp_im,tmp_re2,tmp_im2;
	uint8_t aarx,i;

	pssGen = vsrch_psspattern[nid2];

	compCh16 = (int16_t*)&comCh[0][0];

	for (aarx=0; aarx<HW_NB_RXANT; aarx++)
	{

		sss_orig = (int16_t*)&comCh[aarx][0];
		pss_orig = (int16_t*)&rxPss[aarx][0];

		int amp;
		int shift;
		for (i = 0; i < VSRCH_LENGTH_PSS_NR; i++)
		{
			// This is H*(PSS) = R* \cdot PSS
			tmp_re = pss_orig[i*2] * pssGen[i];
			tmp_im = -pss_orig[i*2+1] * pssGen[i];

			amp = (((int32_t)tmp_re)*tmp_re) + ((int32_t)tmp_im)*tmp_im;
			shift = log2_approx(amp)/2;
			
			// This is R(SSS) \cdot H*(PSS)
			tmp_re2 = (int16_t)(((tmp_re * (int32_t)sss_orig[i*2])>>shift)    - ((tmp_im * (int32_t)sss_orig[i*2+1]>>shift)));
			tmp_im2 = (int16_t)(((tmp_re * (int32_t)sss_orig[i*2+1])>>shift)  + ((tmp_im * (int32_t)sss_orig[i*2]>>shift)));

			// MRC on RX antennas
			if (aarx==0)
			{
				compCh16[i<<1]      = tmp_re2;
				compCh16[1+(i<<1)]  = tmp_im2;
			} 
			else
			{
				compCh16[i<<1]      += tmp_re2;
				compCh16[1+(i<<1)]  += tmp_im2;
			}
		}
	}

	return(0);
}


double vsrch_estFreqOffsetFromPss(unsigned int pss_source, short *rxdata, uint16_t fftSize)
{

	// fractional frequency offser computation according to Cross-correlation Synchronization Algorithm Using PSS
	// Shoujun Huang, Yongtao Su, Ying He and Shan Tang, "Joint time and frequency offset estimation in LTE downlink," 7th International Conference on Communications and Networking in China, 2012.
	int64_t result1,result2;
	// Computing cross-correlation at peak on half the symbol size for first half of data
	result1  = dot_product64((short*)vsrch_pssgen_time[pss_source], 
				  //(short*) &(rxdata[peak_position]), 
				  rxdata, 
				  (fftSize>>1), 
				  vsrch_pssShift);
	// Computing cross-correlation at peak on half the symbol size for data shifted by half symbol size 
	// as it is real and complex it is necessary to shift by a value equal to symbol size to obtain such shift
	result2  = dot_product64((short*)vsrch_pssgen_time[pss_source]+fftSize, 
				  rxdata+fftSize, 
				  (fftSize>>1), 
				  vsrch_pssShift);

	int64_t re1,re2,im1,im2;
	re1=((int*) &result1)[0];
	re2=((int*) &result2)[0];
	im1=((int*) &result1)[1];
	im2=((int*) &result2)[1];

	// estimation of fractional frequency offset: angle[(result1)'*(result2)]/pi
	return (atan2(re1*im2-re2*im1,re1*re2+im1*im2)/VHW_PI);
}



int32_t vsrch_detectPss (const int **rxdata, ///rx data in time domain
									unsigned int length,
									uint16_t fftSize,
									unsigned int start,
                       				uint8_t *maxNid2,
		       						double *f_off, 
		       						int option_print)
{
	unsigned int n, peak_position, pss_source;
	int64_t peak_value;
	int64_t result;
	//double ffo_est=0;
	int64_t psscorr_res=0, psscorr_avg[VSRCH_NB_PSSSEQ];


	const int* rxdata_ant = rxdata[0]; //currently, we are checking for ant port 0 only, later we need to implement pss detection for multiple ant.

	peak_value = 0;
	peak_position = 0;
	pss_source = 0;

	/* Search pss in the received buffer each 4 samples which ensures a memory alignment on 128 bits (32 bits x 4 ) */
	/* This is required by SIMD (single instruction Multiple Data) Extensions of Intel processors. */
	/* Correlation computation is based on a a dot product which is realized thank to SIMS extensions */
	for (int pss_index = 0; pss_index < VSRCH_NB_PSSSEQ; pss_index++)
	{
		psscorr_avg[pss_index] = 0;
	}


	for (n=start; n < start+length; n+=4)
	{ 
		for (int pss_index = 0; pss_index < VSRCH_NB_PSSSEQ; pss_index++)
		{
			psscorr_res = 0;
			if ( n < (start+length - fftSize) )
			{
				/* calculate dot product of primary_synchro_time_nr and rxdata[ar][n] (ar=0..nb_ant_rx) and store the sum in temp[n]; */
				/* perform correlation of rx data and pss sequence ie it is a dot product */
				result	= dot_product64((short*)vsrch_pssgen_time[pss_index], 
										(short*) &(rxdata_ant[n]), 
										  fftSize, 
										  vsrch_pssShift);
				psscorr_res += vhw_abs64(result);
				psscorr_avg[pss_index] += psscorr_res;
			}
			
			/* calculate the absolute value of sync_corr[n] */
			if (psscorr_res > peak_value)
			{
				peak_value = psscorr_res;
				peak_position = n;
				pss_source = pss_index;
			}
		}
	}

	*f_off = vsrch_estFreqOffsetFromPss(pss_source, (short*) &(rxdata_ant[peak_position]), fftSize);

	*maxNid2 = (uint8_t)pss_source;
	psscorr_avg[pss_source]/=(length/4);

	if (option_print == 1)
		LOG_I(PHY,"[vSrch] PSS detection : Sync source = %d, Peak found at pos %d, val = %llu (%d dB), av : %d dB, ffo %lf, start:%i, length:%i\n", 
			pss_source, peak_position, (unsigned long long)peak_value, dB_fixed64(peak_value), dB_fixed64(psscorr_avg[pss_source]), *f_off, start, length);

	if (peak_value < 5*psscorr_avg[pss_source])
		return(-1);

	return(peak_position);
}

