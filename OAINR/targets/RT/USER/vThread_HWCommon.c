#define _GNU_SOURCE
#include <sched.h>
#include <stdlib.h>
#include "vThreadIx_rfSrch.h"

#define VHW_NARFCN_3GHZ		600000
#define VHW_MAX_LMAX_3GHZ	4
#define VHW_MAX_LMAX_6GHZ	8

uint32_t vhw_subcarrier_spacing[VHW_MAX_NUMEROLOGY] = {15e3, 30e3, 60e3, 120e3, 240e3};


//interfaces between virtual hardware


//common function for virtual hardware
void vhw_initvThread(int sched_runtime, int sched_deadline, int sched_fifo, cpu_set_t *cpuset, char *name) {
#ifdef DEADLINE_SCHEDULER

  if (sched_runtime!=0) {
    struct sched_attr attr= {0};
    attr.size = sizeof(attr);
    attr.sched_policy = SCHED_DEADLINE;
    attr.sched_runtime  = sched_runtime;
    attr.sched_deadline = sched_deadline;
    attr.sched_period   = 0;
    AssertFatal(sched_setattr(0, &attr, 0) == 0,
                "[SCHED] %s thread: sched_setattr failed %s \n", name, strerror(errno));
    LOG_I(HW,"[SCHED][eNB] %s deadline thread %lu started on CPU %d\n",
          name, (unsigned long)gettid(), sched_getcpu());
  }

#else

  if (CPU_COUNT(cpuset) > 0)
    AssertFatal( 0 == pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), cpuset), "");

  struct sched_param sp;
  sp.sched_priority = sched_fifo;
  AssertFatal(pthread_setschedparam(pthread_self(),SCHED_FIFO,&sp)==0,
              "Can't set thread priority, Are you root?\n");
  /* Check the actual affinity mask assigned to the thread */
  cpu_set_t *cset=CPU_ALLOC(CPU_SETSIZE);

  if (0 == pthread_getaffinity_np(pthread_self(), CPU_ALLOC_SIZE(CPU_SETSIZE), cset)) {
    char txt[512]= {0};

    for (int j = 0; j < CPU_SETSIZE; j++)
      if (CPU_ISSET(j, cset))
        sprintf(txt+strlen(txt), " %d ", j);

    printf("CPU Affinity of thread %s is %s\n", name, txt);
  }

  CPU_FREE(cset);
#endif
  // Lock memory from swapping. This is a process wide call (not constraint to this thread).
  mlockall(MCL_CURRENT | MCL_FUTURE);
  pthread_setname_np( pthread_self(), name );
  
  printf("started %s as PID: %ld\n",name, gettid());
}


void vhw_initTime(long long *sysTime)
{
	*sysTime = rdtsc_oai();
}


void vhw_timeStamp(long long *sysTime, char* inName)
{
	if (inName != NULL)
	{
		LOG_I(PHY, "%s : %d us\n", inName, (int)((rdtsc_oai() - *sysTime)/cpuf/1000.0));
		*sysTime = rdtsc_oai();
	}
}


unsigned char vhw_log2Approx(unsigned int x)
{

	int i;
	unsigned char l2;

	l2=0;

	for (i=0; i<31; i++)
	{
		if ((x&(1<<i)) != 0)
		{
	  		l2 = i+1;
		}
	}

	return(l2);
}


uint8_t vhw_calcSlotPerSubframe(uint8_t in_numerology)
{
	uint8_t slots_per_subframe;

	switch (in_numerology)
	{
		case 0:
			slots_per_subframe = 1;
			break;
		case 1:
			slots_per_subframe = 2;
			break;
		case 2:
			slots_per_subframe = 4;
			break;
		case 3:
			slots_per_subframe = 8;
			break;
		case 4:
			slots_per_subframe = 16;
			break;
			
		default :
			LOG_E(PHY, "[vHW] [ERROR] failed to calculate slot per subframe (numerology (%i) is invalid!)\n", in_numerology);
			slots_per_subframe = 255;
			break;
	}

	return slots_per_subframe;
}


uint16_t vhw_calcNbPrefixSample (uint16_t fftSize)
{

	uint16_t nb_cp;

	nb_cp = ((unsigned int)144*fftSize)/2048;
	
	return nb_cp;
}

uint16_t vhw_calcNbPrefixSample0 (uint16_t fftSize, uint8_t in_numerology)
{

	uint16_t nb_cp, nb_cp0;

	nb_cp = vhw_calcNbPrefixSample(fftSize);
	nb_cp0 = nb_cp + (16<<in_numerology);
	
	return nb_cp0;
}



uint32_t vhw_calcSamplesPerSubframe (uint16_t fftSize, uint8_t in_numerology)
{
	uint32_t samples_per_subframe;
	uint16_t nb_cp = vhw_calcNbPrefixSample(fftSize);
	uint16_t nb_cp0 = vhw_calcNbPrefixSample0(fftSize, in_numerology);
	uint8_t  slots_per_subframe = vhw_calcSlotPerSubframe(in_numerology);


	samples_per_subframe = (fftSize * HW_NB_SYM_IN_SLOT * slots_per_subframe) + 
							(nb_cp0 * slots_per_subframe) + 
							(nb_cp * slots_per_subframe * (HW_NB_SYM_IN_SLOT-1));

	return samples_per_subframe;
}








//compute average channel_level on each (TX,RX) antenna pair
int vhw_calcPbchChLevel(int **dl_ch_estimates_ext,
								uint16_t nb_rb,
                          		uint32_t symbol)
{
	int16_t rb;//, nb_rb=20;
	uint8_t aarx;
#if defined(__x86_64__) || defined(__i386__)
	__m128i avg128;
	__m128i *dl_ch128;
#elif defined(__arm__)
	int32x4_t avg128;
	int16x8_t *dl_ch128;
#endif
	int avg1=0,avg2=0;

	for (aarx=0; aarx<HW_NB_RXANT; aarx++)
	{
		//clear average level
#if defined(__x86_64__) || defined(__i386__)
		avg128 = _mm_setzero_si128();
		dl_ch128=(__m128i *)&dl_ch_estimates_ext[aarx][symbol*nb_rb*HW_NB_RE_IN_RB];
#elif defined(__arm__)
		avg128 = vdupq_n_s32(0);
		dl_ch128=(int16x8_t *)&dl_ch_estimates_ext[aarx][symbol*nb_rb*HW_NB_RE_IN_RB];
#endif

		for (rb=0; rb<nb_rb; rb++)
		{
#if defined(__x86_64__) || defined(__i386__)
			avg128 = _mm_add_epi32(avg128,_mm_madd_epi16(dl_ch128[0],dl_ch128[0]));
			avg128 = _mm_add_epi32(avg128,_mm_madd_epi16(dl_ch128[1],dl_ch128[1]));
			avg128 = _mm_add_epi32(avg128,_mm_madd_epi16(dl_ch128[2],dl_ch128[2]));
#elif defined(__arm__)
	  		// to be filled in
#endif
	  		dl_ch128+=3;
		}

		avg1 = (((int *)&avg128)[0] +
		        ((int *)&avg128)[1] +
		        ((int *)&avg128)[2] +
		        ((int *)&avg128)[3])/(nb_rb*12);

		if (avg1 > avg2)
			avg2 = avg1;
	}

#if defined(__x86_64__) || defined(__i386__)
	_mm_empty();
	_m_empty();
#endif

	return(avg2);
}






int64_t vhw_abs64(int64_t x)
{
  return (((int64_t)((int32_t*)&x)[0])*((int64_t)((int32_t*)&x)[0]) + ((int64_t)((int32_t*)&x)[1])*((int64_t)((int32_t*)&x)[1]));
}





uint32_t vhw_generateGoldGeneric(uint32_t *x1, uint32_t *x2, uint8_t reset)
{
	int32_t n;

	// 3GPP 3x.211
	// Nc = 1600
	// c(n)     = [x1(n+Nc) + x2(n+Nc)]mod2
	// x1(n+31) = [x1(n+3)                     + x1(n)]mod2
	// x2(n+31) = [x2(n+3) + x2(n+2) + x2(n+1) + x2(n)]mod2
	if (reset)
	{
		// Init value for x1: x1(0) = 1, x1(n) = 0, n=1,2,...,30
		// x1(31) = [x1(3) + x1(0)]mod2 = 1
		*x1 = 1 + (1<<31);
		// Init value for x2: cinit = sum_{i=0}^30 x2*2^i
		// x2(31) = [x2(3)    + x2(2)    + x2(1)    + x2(0)]mod2
		//        =  (*x2>>3) ^ (*x2>>2) + (*x2>>1) + *x2
		*x2 = *x2 ^ ((*x2 ^ (*x2>>1) ^ (*x2>>2) ^ (*x2>>3))<<31);

		// x1 and x2 contain bits n = 0,1,...,31

		// Nc = 1600 bits are skipped at the beginning
		// i.e., 1600 / 32 = 50 32bit words

		for (n = 1; n < 50; n++)
		{
			// Compute x1(0),...,x1(27)
			*x1 = (*x1>>1) ^ (*x1>>4);
			// Compute x1(28),..,x1(31) and xor
			*x1 = *x1 ^ (*x1<<31) ^ (*x1<<28);
			// Compute x2(0),...,x2(27)
			*x2 = (*x2>>1) ^ (*x2>>2) ^ (*x2>>3) ^ (*x2>>4);
			// Compute x2(28),..,x2(31) and xor
			*x2 = *x2 ^ (*x2<<31) ^ (*x2<<30) ^ (*x2<<29) ^ (*x2<<28);
		}
	}

	*x1 = (*x1>>1) ^ (*x1>>4);
	*x1 = *x1 ^ (*x1<<31) ^ (*x1<<28);
	*x2 = (*x2>>1) ^ (*x2>>2) ^ (*x2>>3) ^ (*x2>>4);
	*x2 = *x2 ^ (*x2<<31) ^ (*x2<<30) ^ (*x2<<29) ^ (*x2<<28);

	// c(n) = [x1(n+Nc) + x2(n+Nc)]mod2
	return(*x1^*x2);
}



void* vhw_malloc16_clear( size_t size )
{
#ifdef __AVX2__
  void* ptr = memalign(32, size+32);
#else
  void* ptr = memalign(16, size+16);
#endif
  DevAssert(ptr);
  memset( ptr, 0, size );
  return ptr;
}



void vhw_printData(int16_t* data, int size, char* string)
{
	LOG_E(PHY, "%s : ", string);
	for (int i=0;i<size;i++)
	{
		printf("%d ", data[i]);
	}
	printf("\n");
}


int vhw_ssbSymOffset[2][8] = { 
	{2, 8, 16, 22, 30, 36, 44, 50}, //case A
	{4, 8, 16, 20, 32, 36, 44, 48} //case B
	};

//currently case A, B is only supported.

int vhw_calcSsbSlotNb(int narfcn, uint8_t numerology, uint8_t issb)
{
	int symNb, slotNb;


	if (numerology >= 2)
	{
		LOG_E(PHY, "[vHW] WARNING : currently u=0,1 (case A, case B) is supported for calculating SSB slot number!\n");
		return 0;
	}
	if (narfcn < VHW_NARFCN_3GHZ && issb > VHW_MAX_LMAX_3GHZ/2)
	{
		LOG_E(PHY, "[vHW] WARNING : wrong issb (%i0 for the narfcn :%i\n", issb, narfcn);
		return 0;
	}

	symNb = vhw_ssbSymOffset[numerology][issb];
	slotNb = symNb/HW_NB_SYM_IN_SLOT;
	
	return (slotNb);
}


int vhw_calcSsbSymbNb(int narfcn, uint8_t numerology, uint8_t issb)
{
	int symNb;


	if (numerology >= 2)
	{
		LOG_E(PHY, "[vHW] WARNING : currently u=0,1 (case A, case B) is supported for calculating SSB slot number!\n");
		return 0;
	}
	if (narfcn < VHW_NARFCN_3GHZ && issb > VHW_MAX_LMAX_3GHZ/2)
	{
		LOG_E(PHY, "[vHW] WARNING : wrong issb (%i0 for the narfcn :%i\n", issb, narfcn);
		return 0;
	}

	symNb = vhw_ssbSymOffset[numerology][issb];
	
	return (symNb);
}


