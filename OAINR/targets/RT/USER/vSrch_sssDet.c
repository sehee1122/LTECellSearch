#include "vSrch_ssbCommon.h"
#include "vSrch_pssDet.h"
#include "limits.h"


#define VSRCH_NB_PHASE_HYPO       		7
#define VSRCH_SCALING_METRIC        	15
#define VSRCH_SSS_METRIC_FLOOR   		30000
#define VSRCH_NB_INIT_SSS				7


static int16_t vsrch_d_sss[VSRCH_NB_PSSSEQ][VSRCH_NB_SSSSEQ][VSRCH_LENGTH_SSS_NR];


const int16_t vsrch_phase_re[VSRCH_NB_PHASE_HYPO] = {16383, 25101, 30791, 32767, 30791, 25101, 16383};
const int16_t vsrch_phase_im[VSRCH_NB_PHASE_HYPO] = {-28378, -21063, -11208, 0, 11207, 21062, 28377};

static int32_t** vsrch_pss_ext = NULL;
static int32_t** vsrch_sss_ext = NULL;




void vsrch_initSssBuf(void)
{
	if (vsrch_pss_ext == NULL)
	{
		vsrch_pss_ext = (int32_t**) vhw_malloc16_clear( HW_NB_RXANT * sizeof(int32_t*) );
		for (int i=0;i<HW_NB_RXANT;i++)
		{
			vsrch_pss_ext[i] = (int32_t*) vhw_malloc16_clear( VSRCH_LENGTH_PSS_NR * sizeof(int32_t) );
		}
	}

	if (vsrch_sss_ext == NULL)
	{
		vsrch_sss_ext = (int32_t**) vhw_malloc16_clear( HW_NB_RXANT * sizeof(int32_t*) );
		for (int i=0;i<HW_NB_RXANT;i++)
		{
			vsrch_sss_ext[i] = (int32_t*) vhw_malloc16_clear( VSRCH_LENGTH_SSS_NR * sizeof(int32_t) );
		}
	}
}



void vsrch_clearSssBuf(void)
{
	for (int i=0;i<HW_NB_RXANT;i++)
	{
		memset(vsrch_pss_ext[i], 0, VSRCH_LENGTH_PSS_NR * sizeof(int) );
		memset(vsrch_sss_ext[i], 0, VSRCH_LENGTH_SSS_NR * sizeof(int) );
	}
}



void vsrch_generateSss(void)
{
	int16_t x0[VSRCH_LENGTH_SSS_NR];
	int16_t x1[VSRCH_LENGTH_SSS_NR];
	int16_t dss_current;
	int m0, m1;

	const int x0_initial[VSRCH_NB_INIT_SSS] = { 1, 0, 0, 0, 0, 0, 0 };
	const int x1_initial[VSRCH_NB_INIT_SSS] = { 1, 0, 0, 0, 0, 0, 0 };

	for (int i=0; i < VSRCH_NB_INIT_SSS; i++)
	{
		x0[i] = x0_initial[i];
		x1[i] = x1_initial[i];
	}

	for (int i=0; i < (VSRCH_LENGTH_SSS_NR - VSRCH_NB_INIT_SSS); i++)
	{
		x0[i+7] = (x0[i + 4] + x0[i])%(2);
		x1[i+7] = (x1[i + 1] + x1[i])%(2);
	}

	for (int N_ID_2 = 0; N_ID_2 < VSRCH_NB_PSSSEQ; N_ID_2++)
	{
		for (int N_ID_1 = 0; N_ID_1 < VSRCH_NB_SSSSEQ; N_ID_1++)
		{
			m0 = 15*(N_ID_1/112) + (5*N_ID_2);
			m1 = N_ID_1%112;

	  		for (int n = 0; n < VSRCH_LENGTH_SSS_NR; n++)
			{
	    		dss_current = (1 - 2*x0[(n + m0)%(VSRCH_LENGTH_SSS_NR)])*(1 - 2*x1[(n + m1)%(VSRCH_LENGTH_SSS_NR)]);

		  		/* Modulation of SSS is a BPSK TS 36.211 chapter 5.1.2 BPSK */
	    		vsrch_d_sss[N_ID_2][N_ID_1][n]   = dss_current;// * amp;
	  		}
		}
	}

}


void vsrch_initSssDet(void)
{
	vsrch_generateSss();
	vsrch_clearSssBuf();
}



int vsrch_extractPssSssSym(int32_t **ssbFBuf,
									uint16_t fftSize,
									unsigned int ssb_Foffset,
		                            int32_t** pss_ext,
		                            int32_t** sss_ext) // add flag to indicate extracting only PSS, only SSS, or both
{
	uint8_t aarx;
	int32_t *pss_rxF,*pss_rxF_ext;
	int32_t *sss_rxF,*sss_rxF_ext;
	uint8_t pss_symbol, sss_symbol;
	int32_t **rxdataF;

	for (aarx=0; aarx<HW_NB_RXANT; aarx++)
	{
		pss_symbol = 0;
		sss_symbol = VSRCH_SSS_SYMOFFSET;

		rxdataF = ssbFBuf;

		pss_rxF  =  &rxdataF[aarx][pss_symbol*fftSize];
		sss_rxF  =  &rxdataF[aarx][sss_symbol*fftSize];

		pss_rxF_ext = &pss_ext[aarx][0];
		sss_rxF_ext = &sss_ext[aarx][0];

		unsigned int k = ssb_Foffset + VSRCH_SSB_PSS_OFFSET; //and
		if (k>= fftSize)
			k-= fftSize;

		for (int i=0; i < VSRCH_LENGTH_PSS_NR; i++)
		{
	    	pss_rxF_ext[i] = pss_rxF[k];
	    	sss_rxF_ext[i] = sss_rxF[k];

	  		k++;

	  		if (k == fftSize)
				k=0;
		  
		}
	}

	return(0);
}









int16_t vsrch_detectSss(int **ssbBuf, int **ssbFBuf, 
						  uint16_t fftSize, uint8_t Nid2, uint32_t ssb_Foffset, int option_print)
{
	uint8_t i;
	uint16_t Nid1;
	int16_t cellid = -1;
	uint8_t phase;
	int16_t *sss;
	int32_t metric_re;
	int16_t *d;
	int32_t tot_metric;
	uint8_t phase_max=0;

	/* slop_fep function works for lte and takes into account begining of frame with prefix for subframe 0 */
	/* for NR this is not the case but slot_fep is still used for computing FFT of samples */
	/* in order to achieve correct processing for NR prefix samples is forced to 0 and then restored after function call */
	/* symbol number are from beginning of SS/PBCH blocks as below:  */
	/*    Signal            PSS  PBCH  SSS  PBCH                     */
	/*    symbol number      0     1    2    3                       */
	/* time samples in buffer rxdata are used as input of FFT -> FFT results are stored in the frequency buffer rxdataF */

	vsrch_extractPssSssSym(ssbFBuf, fftSize, ssb_Foffset, vsrch_pss_ext, vsrch_sss_ext);


	// get conjugated channel estimate from PSS, H* = R* \cdot PSS
	// and do channel estimation and compensation based on PSS
	vsrch_ChCompByPss(&vsrch_pss_ext[0], &vsrch_sss_ext[0], Nid2);




	// now do the SSS detection based on the precomputed sequences in PHY/LTE_TRANSPORT/sss.h
	tot_metric = INT_MIN;
	sss = (int16_t*)&vsrch_sss_ext[0][0];

	/* for phase evaluation, one uses an array of possible phase shifts */
	/* then a correlation is done between received signal with a shift pĥase and the reference signal */
	/* Computation of signal with shift phase is based on below formula */
	/* cosinus cos(x + y) = cos(x)cos(y) - sin(x)sin(y) */
	/* sinus   sin(x + y) = sin(x)cos(y) + cos(x)sin(y) */

	for (Nid1 = 0 ; Nid1 < VSRCH_NB_SSSSEQ; Nid1++)
	{   // all possible Nid1 values
		for (phase=0; phase < VSRCH_NB_PHASE_HYPO; phase++)
		{	// phase offset between PSS and SSS
			metric_re = 0;
	  		d = (int16_t *)&vsrch_d_sss[Nid2][Nid1];

	 		// This is the inner product using one particular value of each unknown parameter
	  		for (i=0; i < VSRCH_LENGTH_SSS_NR; i++)
			{
	    		metric_re += d[i]*(((vsrch_phase_re[phase]*sss[2*i])>>VSRCH_SCALING_METRIC) - ((vsrch_phase_im[phase]*sss[2*i+1])>>VSRCH_SCALING_METRIC));   
	  		}

			// if the current metric is better than the last save it
			if (metric_re > tot_metric)
			{
				tot_metric = metric_re;
				cellid = Nid2+(3*Nid1);
				phase_max = phase;
			}
		}
	}

	if (tot_metric > VSRCH_SSS_METRIC_FLOOR)
	{	
		Nid2 = cellid%3;
		Nid1 = cellid/3;
		if (option_print == 1)
			LOG_I(PHY, "Nid2 %d Nid1 %d tot_metric %d, phase_max %d \n", Nid2, Nid1, tot_metric, phase_max);
	}

	return(cellid);
}

