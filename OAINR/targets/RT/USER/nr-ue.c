/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.0  (the "License"); you may not use this file
 * except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *      http://www.openairinterface.org/?page_id=698
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *-------------------------------------------------------------------------------
 * For more information about the OpenAirInterface (OAI) Software Alliance:
 *      contact@openairinterface.org
 */

/*! \file lte-ue.c
 * \brief threads and support functions for real-time LTE UE target
 * \author R. Knopp, F. Kaltenberger, Navid Nikaein
 * \date 2015
 * \version 0.1
 * \company Eurecom
 * \email: knopp@eurecom.fr,florian.kaltenberger@eurecom.fr, navid.nikaein@eurecom.fr
 * \note
 * \warning
 */
#include "nr-uesoftmodem.h"

#include "rt_wrapper.h"

#include "LAYER2/NR_MAC_UE/mac.h"
//#include "RRC/LTE/rrc_extern.h"
#include "PHY_INTERFACE/phy_interface_extern.h"

#undef MALLOC //there are two conflicting definitions, so we better make sure we don't use it at all
//#undef FRAME_LENGTH_COMPLEX_SAMPLES //there are two conflicting definitions, so we better make sure we don't use it at all

#include "fapi_nr_ue_l1.h"
#include "PHY/phy_extern_nr_ue.h"
#include "PHY/INIT/phy_init.h"
#include "PHY/MODULATION/modulation_UE.h"
#include "LAYER2/NR_MAC_UE/mac_proto.h"
#include "RRC/NR_UE/rrc_proto.h"

//#ifndef NO_RAT_NR
#include "SCHED_NR/phy_frame_config_nr.h"
//#endif
#include "SCHED_NR_UE/defs.h"

#include "PHY/NR_UE_TRANSPORT/nr_transport_proto_ue.h"

#include "common/utils/LOG/log.h"
#include "common/utils/LOG/vcd_signal_dumper.h"

#include "T.h"
#include "targets/RT/USER/NHAL_rf.h"
#include "targets/RT/USER/vThread_srch.h"
#include "PHY/NR_REFSIG/refsig_defs_ue.h"


#ifdef XFORMS
  #include "PHY/TOOLS/nr_phy_scope.h"

  extern char do_forms;
#endif


#define VSYNC_CHRASTER_STEPSIZE 		3000
#define VSYNC_CHRASTER_RANGE			10

#ifdef DNF_SYNCANDONSRCH
#define VSYNC_SPDUP_SRCHWINDOW			8192
#endif


uint8_t MIB_dout[3];


extern double cpuf;
//static  nfapi_nr_config_request_t config_t;
//static  nfapi_nr_config_request_t* config =&config_t;

/*
 *  NR SLOT PROCESSING SEQUENCE
 *
 *  Processing occurs with following steps for connected mode:
 *
 *  - Rx samples for a slot are received,
 *  - PDCCH processing (including DCI extraction for downlink and uplink),
 *  - PDSCH processing (including transport blocks decoding),
 *  - PUCCH/PUSCH (transmission of acknowledgements, CSI, ... or data).
 *
 *  Time between reception of the slot and related transmission depends on UE processing performance.
 *  It is defined by the value NR_UE_CAPABILITY_SLOT_RX_TO_TX.
 *
 *  In NR, network gives the duration between Rx slot and Tx slot in the DCI:
 *  - for reception of a PDSCH and its associated acknowledgment slot (with a PUCCH or a PUSCH),
 *  - for reception of an uplink grant and its associated PUSCH slot.
 *
 *  So duration between reception and it associated transmission depends on its transmission slot given in the DCI.
 *  NR_UE_CAPABILITY_SLOT_RX_TO_TX means the minimum duration but higher duration can be given by the network because UE can support it.
 *
*                                                                                                    Slot k
*                                                                                  -------+------------+--------
*                Frame                                                                    | Tx samples |
*                Subframe                                                                 |   buffer   |
*                Slot n                                                            -------+------------+--------
*       ------ +------------+--------                                                     |
*              | Rx samples |                                                             |
*              |   buffer   |                                                             |
*       -------+------------+--------                                                     |
*                           |                                                             |
*                           V                                                             |
*                           +------------+                                                |
*                           |   PDCCH    |                                                |
*                           | processing |                                                |
*                           +------------+                                                |
*                           |            |                                                |
*                           |            v                                                |
*                           |            +------------+                                   |
*                           |            |   PDSCH    |                                   |
*                           |            | processing | decoding result                   |
*                           |            +------------+    -> ACK/NACK of PDSCH           |
*                           |                         |                                   |
*                           |                         v                                   |
*                           |                         +-------------+------------+        |
*                           |                         | PUCCH/PUSCH | Tx samples |        |
*                           |                         |  processing | transfer   |        |
*                           |                         +-------------+------------+        |
*                           |                                                             |
*                           |/___________________________________________________________\|
*                            \  duration between reception and associated transmission   /
*
* Remark: processing is done slot by slot, it can be distribute on different threads which are executed in parallel.
* This is an architecture optimization in order to cope with real time constraints.
* By example, for LTE, subframe processing is spread over 4 different threads.
*
 */

#ifndef NO_RAT_NR
  #define DURATION_RX_TO_TX           (NR_UE_CAPABILITY_SLOT_RX_TO_TX)  /* for NR this will certainly depends to such UE capability which is not yet defined */
#else
  #define DURATION_RX_TO_TX           (4)   /* For LTE, this duration is fixed to 4 and it is linked to LTE standard for both modes FDD/TDD */
#endif

#define FRAME_PERIOD    100000000ULL
#define DAQ_PERIOD      66667ULL
#define FIFO_PRIORITY   40

typedef enum {
  pss=0,
  pbch=1,
  si=2
} sync_mode_t;

void init_UE_threads(PHY_VARS_NR_UE *UE);
void UE_threadPreInit(void *arg);
void init_UE(int nb_inst);

int32_t **rxdata;
int32_t **txdata;

#define KHz (1000UL)
#define MHz (1000*KHz)
#define SAIF_ENABLED

#ifdef SAIF_ENABLED
  uint64_t  g_ue_rx_thread_busy = 0;
#endif

typedef struct eutra_band_s {
  int16_t band;
  uint32_t ul_min;
  uint32_t ul_max;
  uint32_t dl_min;
  uint32_t dl_max;
  lte_frame_type_t frame_type;
} eutra_band_t;

typedef struct band_info_s {
  int nbands;
  eutra_band_t band_info[100];
} band_info_t;

band_info_t bands_to_scan;

static const eutra_band_t eutra_bands[] = {
  { 1, 1920    * MHz, 1980    * MHz, 2110    * MHz, 2170    * MHz, FDD},
  { 2, 1850    * MHz, 1910    * MHz, 1930    * MHz, 1990    * MHz, FDD},
  { 3, 1710    * MHz, 1785    * MHz, 1805    * MHz, 1880    * MHz, FDD},
  { 4, 1710    * MHz, 1755    * MHz, 2110    * MHz, 2155    * MHz, FDD},
  { 5,  824    * MHz,  849    * MHz,  869    * MHz,  894    * MHz, FDD},
  { 6,  830    * MHz,  840    * MHz,  875    * MHz,  885    * MHz, FDD},
  { 7, 2500    * MHz, 2570    * MHz, 2620    * MHz, 2690    * MHz, FDD},
  { 8,  880    * MHz,  915    * MHz,  925    * MHz,  960    * MHz, FDD},
  { 9, 1749900 * KHz, 1784900 * KHz, 1844900 * KHz, 1879900 * KHz, FDD},
  {10, 1710    * MHz, 1770    * MHz, 2110    * MHz, 2170    * MHz, FDD},
  {11, 1427900 * KHz, 1452900 * KHz, 1475900 * KHz, 1500900 * KHz, FDD},
  {12,  698    * MHz,  716    * MHz,  728    * MHz,  746    * MHz, FDD},
  {13,  777    * MHz,  787    * MHz,  746    * MHz,  756    * MHz, FDD},
  {14,  788    * MHz,  798    * MHz,  758    * MHz,  768    * MHz, FDD},
  {17,  704    * MHz,  716    * MHz,  734    * MHz,  746    * MHz, FDD},
  {20,  832    * MHz,  862    * MHz,  791    * MHz,  821    * MHz, FDD},
  {22, 3510    * MHz, 3590    * MHz, 3410    * MHz, 3490    * MHz, FDD},
  {33, 1900    * MHz, 1920    * MHz, 1900    * MHz, 1920    * MHz, TDD},
  {34, 2010    * MHz, 2025    * MHz, 2010    * MHz, 2025    * MHz, TDD},
  {35, 1850    * MHz, 1910    * MHz, 1850    * MHz, 1910    * MHz, TDD},
  {36, 1930    * MHz, 1990    * MHz, 1930    * MHz, 1990    * MHz, TDD},
  {37, 1910    * MHz, 1930    * MHz, 1910    * MHz, 1930    * MHz, TDD},
  {38, 2570    * MHz, 2620    * MHz, 2570    * MHz, 2630    * MHz, TDD},
  {39, 1880    * MHz, 1920    * MHz, 1880    * MHz, 1920    * MHz, TDD},
  {40, 2300    * MHz, 2400    * MHz, 2300    * MHz, 2400    * MHz, TDD},
  {41, 2496    * MHz, 2690    * MHz, 2496    * MHz, 2690    * MHz, TDD},
  {42, 3400    * MHz, 3600    * MHz, 3400    * MHz, 3600    * MHz, TDD},
  {43, 3600    * MHz, 3800    * MHz, 3600    * MHz, 3800    * MHz, TDD},
  {44, 703    * MHz, 803    * MHz, 703    * MHz, 803    * MHz, TDD},
};

PHY_VARS_NR_UE *init_nr_ue_vars(NR_DL_FRAME_PARMS *frame_parms,
                                uint8_t UE_id,
                                uint8_t abstraction_flag)

{
  PHY_VARS_NR_UE *ue;

  if (frame_parms!=(NR_DL_FRAME_PARMS *)NULL) { // if we want to give initial frame parms, allocate the PHY_VARS_UE structure and put them in
    ue = (PHY_VARS_NR_UE *)malloc(sizeof(PHY_VARS_NR_UE));
    memset(ue,0,sizeof(PHY_VARS_NR_UE));
    memcpy(&(ue->frame_parms), frame_parms, sizeof(NR_DL_FRAME_PARMS));
  } else ue = PHY_vars_UE_g[UE_id][0];

  ue->Mod_id      = UE_id;
  ue->mac_enabled = 1;
  // initialize all signal buffers
  init_nr_ue_signal(ue,1,abstraction_flag);
  // intialize transport
  init_nr_ue_transport(ue,abstraction_flag);
  return(ue);
}

void init_thread(int sched_runtime, int sched_deadline, int sched_fifo, cpu_set_t *cpuset, char *name) {
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
  // LTS: this sync stuff should be wrong
  printf("waiting for sync (%s)\n",name);
  pthread_mutex_lock(&sync_mutex);
  printf("Locked sync_mutex, waiting (%s)\n",name);

  while (sync_var<0)
    pthread_cond_wait(&sync_cond, &sync_mutex);

  pthread_mutex_unlock(&sync_mutex);
  printf("started %s as PID: %ld\n",name, gettid());
}


extern openair0_config_t rf_params;
void init_UE(int nb_inst)
{
	int inst = 0;
	NR_UE_MAC_INST_t *mac_inst;
	PHY_vars_UE_g[inst][0] = init_nr_ue_vars(NULL,inst,0);
	PHY_VARS_NR_UE *UE = PHY_vars_UE_g[inst][0];

	AssertFatal( 0 == NHAL_RFcmd_init((void*)UE, rf_params.mmapped_dma, rf_params.clock_source, rf_params.configFilename, rf_params.sdr_addrs), 
					"RF HW cannot be initialized properly\n");
	
	AssertFatal( 0 == NHAL_srchcmd_init(),
					"SRCH HW cannot be initialized properly\n");
	
	LOG_I(PHY,"Initializing memory for UE instance %d (%p)\n",inst,PHY_vars_UE_g[inst]);
		
	AssertFatal((UE->if_inst = nr_ue_if_module_init(inst)) != NULL, "can not initial IF module\n");
	nr_l3_init_ue();
	nr_l2_init_ue();
	mac_inst = get_mac_inst(0);
	mac_inst->if_module = UE->if_inst;
	UE->if_inst->scheduled_response = nr_ue_scheduled_response;
	UE->if_inst->phy_config_request = nr_ue_phy_config_request;

	LOG_I(PHY,"Intializing UE Threads for instance %d (%p,%p)...\n",inst,PHY_vars_UE_g[inst],PHY_vars_UE_g[inst][0]);
	UE_threadPreInit(UE);
	printf("UE threads created by %ld\n", gettid());
}




/*!
 * \brief This is the UE synchronize thread.
 * It performs band scanning and synchonization.
 * \param arg is a pointer to a \ref PHY_VARS_NR_UE structure.
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */

static void initialize_scanFreq(PHY_VARS_NR_UE *UE)
{	
	int current_band = 0;
	int CC_id = UE->CC_id;
	int i;

	if (UE->UE_scan == 0)
	{
    	int ind;

	    for ( ind=0; ind < sizeof(eutra_bands) / sizeof(eutra_bands[0]); ind++)
		{
			current_band = eutra_bands[ind].band;
			LOG_D(PHY, "Scanning band %d, dl_min %"PRIu32", ul_min %"PRIu32"\n", current_band, eutra_bands[ind].dl_min,eutra_bands[ind].ul_min);

			if ( eutra_bands[ind].dl_min <= downlink_frequency[0][0] && eutra_bands[ind].dl_max >= downlink_frequency[0][0] )
			{
				for (i=0; i<4; i++)
			  		uplink_frequency_offset[CC_id][i] = eutra_bands[ind].ul_min - eutra_bands[ind].dl_min;

				break;
			}
	    }

    	AssertFatal( ind < sizeof(eutra_bands) / sizeof(eutra_bands[0]), "Can't find EUTRA band for frequency");
    	LOG_I( PHY, "[SCHED][UE] Check absolute frequency DL %"PRIu32", UL %"PRIu32" (oai_exit %d)\n",
           	downlink_frequency[0][0], downlink_frequency[0][0]+uplink_frequency_offset[0][0],
           	oai_exit);
  	} 
	else
	{
	    current_band=0;

		for (i=0; i<rf_params.rx_num_channels; i++)
		{
			downlink_frequency[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[CC_id].dl_min;
			uplink_frequency_offset[UE->rf_map.card][UE->rf_map.chain+i] = bands_to_scan.band_info[CC_id].ul_min-bands_to_scan.band_info[CC_id].dl_min;
		}
	}
}


static void sendResUpper(PHY_VARS_NR_UE *UE, uint8_t res_eByte, uint8_t res_issb, uint8_t res_Lmax)
{
	UE->dl_indication.rx_ind = &UE->rx_ind; //	hang on rx_ind instance
	UE->rx_ind.rx_indication_body[0].pdu_type = FAPI_NR_RX_PDU_TYPE_MIB;
	UE->rx_ind.rx_indication_body[0].mib_pdu.pdu = &MIB_dout[0];
	UE->rx_ind.rx_indication_body[0].mib_pdu.additional_bits = res_eByte;
	UE->rx_ind.rx_indication_body[0].mib_pdu.ssb_index = res_issb;		   //  confirm with TCL
	UE->rx_ind.rx_indication_body[0].mib_pdu.ssb_length = res_Lmax; 			   //  confirm with TCL
	UE->rx_ind.rx_indication_body[0].mib_pdu.cell_id = UE->frame_parms.Nid_cell;  //  confirm with TCL
	UE->rx_ind.number_pdus = 1;
	
	if (UE->if_inst && UE->if_inst->dl_indication)
		UE->if_inst->dl_indication(&UE->dl_indication);

}


static void configUeFrameParam (NR_DL_FRAME_PARMS *fp)
{
	int n_ssb_crb=(fp->N_RB_DL-20);

	// First try TDD normal prefix, mu 1
	fp->Ncp=NORMAL;
	fp->frame_type=TDD;
	// FK: added N_RB_DL paramter here as this function shares code with the gNB where it is needed. We should rewrite this function for the UE. 
#ifdef FIX_ENABLENRUSRPB210
	nr_init_frame_parms_ue(fp,fp->numerology_index,NORMAL,fp->N_RB_DL,n_ssb_crb,0);
#else
	nr_init_frame_parms_ue(fp,NR_MU_1,NORMAL,fp->N_RB_DL,n_ssb_crb,0);
#endif
}


#define NPHY_FREQ_3GHZ				3000000000
#define NPHY_NARFCN_3GHZ			600000

#define NPHY_INTERFREQ_UNDER3GHZ	5000
#define NPHY_INTERFREQ_SUB6GHZ		15000



static uint32_t UE_calcNarfcn(uint32_t freq)
{	
	int narfcn;
	if (freq < NPHY_FREQ_3GHZ)
	{
		narfcn = (int)(freq / NPHY_INTERFREQ_UNDER3GHZ);
	}
	else
	{
		narfcn = NPHY_NARFCN_3GHZ + (int)((freq - NPHY_FREQ_3GHZ)/NPHY_INTERFREQ_SUB6GHZ);
	}

	return narfcn;
}










static void *UE_thread_synch(void *arg) {
	static int __thread UE_thread_synch_retval;
	int i;
	PHY_VARS_NR_UE *UE = (PHY_VARS_NR_UE *) arg;

	int CC_id = UE->CC_id;
	int freq_offset=0;
	char threadname[128];
	int ret;
	int sync_cnt=0;
	#ifdef DNF_SYNCANDONSRCH
	int ssbSlotOffset = 0;
	int ssbSymOffset = 0;
	#endif
	int narfcn = 0;

	cpu_set_t cpuset;
	CPU_ZERO(&cpuset);

	if ( threads.sync != -1 )
	CPU_SET(threads.sync, &cpuset);

	// this thread priority must be lower that the main acquisition thread
	sprintf(threadname, "sync UE %d", UE->Mod_id);
	init_thread(100000, 500000, FIFO_PRIORITY-1, &cpuset, threadname);

	
	long long estTime = rdtsc_oai();

	UE->is_synchronized = 0;
	initialize_scanFreq(UE);

	//turning the RF on	
	NHAL_RFcmd_on(NHAL_convSampleRate(rf_params.sample_rate), 
					UE->frame_parms.samples_per_frame, UE->frame_parms.samples_per_slot,
					downlink_frequency[CC_id][0], downlink_frequency[CC_id][0]+uplink_frequency_offset[CC_id][0],
					rf_params.rx_bw, rf_params.tx_bw, UE->rx_total_gain_dB, rf_params.rx_gain_offset[0], rf_params.tx_gain[0],
					nhal_syncState_outsync);	
	
  	while (oai_exit == 0) 
	{
		if (UE->is_synchronized == 0)
		{
			#ifdef DNF_SYNCANDONSRCH
			//config srcher H/W for full search first
			narfcn = UE_calcNarfcn(downlink_frequency[CC_id][0]);
			if (UE->UE_scan_carrier == 1)
			{
				NHAL_srchcmd_on(0, narfcn, srchMode_full, //bank, arfcn, mode
								UE->frame_parms.numerology_index, UE->frame_parms.ofdm_symbol_size, //frame structure (numerology, FFT size)
								0, 0, //search basic option :: duration, srchSubframe
								UE->frame_parms.first_carrier_offset + UE->frame_parms.ssb_start_subcarrier, 0, //SSB configuration (freq. offset, period)
								UE->high_speed_flag, //channel speed model
								0, 1, //option : decimation, pbch CRC check
								-1, 0, //PSS detection option : target NID, threshold
								0, ssbSlotOffset,//search window, search offset
								0, 0); //issb, half frame indicator
			}
			else
			{
				NHAL_srchcmd_on(0, narfcn, srchMode_full, 
								UE->frame_parms.numerology_index, UE->frame_parms.ofdm_symbol_size, //frame structure (numerology, FFT size)
								0, 0, //search basic option :: duration, srchSubframe
								UE->frame_parms.first_carrier_offset + UE->frame_parms.ssb_start_subcarrier, 0, //SSB configuration (freq. offset, period)
								UE->high_speed_flag, //channel speed model
								0, 1, //option : decimation, pbch CRC check
								UE->frame_parms.Nid_cell, 0, //PSS detection option : target NID, threshold
								VSYNC_SPDUP_SRCHWINDOW, ssbSlotOffset, //search window, search offset
								0, 0); //issb, half frame indicator
			}
			#else
			//1. narfcn calculation
			narfcn = UE_calcNarfcn(downlink_frequency[CC_id][0]);
			NHAL_srchcmd_on(0, narfcn, srchMode_full, 
							UE->frame_parms.numerology_index, UE->frame_parms.ofdm_symbol_size, //frame structure (numerology, FFT size)
							0, 0, //search basic option :: duration, srchSubframe
							UE->frame_parms.first_carrier_offset + UE->frame_parms.ssb_start_subcarrier, 0, //SSB configuration (freq. offset, period)
							UE->high_speed_flag, //channel speed model
							0, 1, //option : decimation, pbch CRC check
							-1, 0, //PSS detection option : target NID, threshold
							0, 0, //search window, search offset
							0, 0); //issb, half frame indicator
			#endif
				
			int srchCnt = NHAL_srchcmd_waitIRQ(0);
			LOG_I(PHY, "[UE thread Synch] Initial Synch trial has ended (srch cnt : %i)\n", srchCnt);
			
			if (srchCnt == 0)
			{
				ret = -3;
			}
			else
			{
				if (NHAL_srchcmd_loadSrchRes(0,0) == 0)
				{
					NHAL_srchcmd_getFrameFreqOffset(0,0, &(UE->rx_offset), &(UE->common_vars.freq_offset));
					ret = 0;
				}
				else
				{
					ret = -3;
				}
			}

			
			if (ret == 0)
			{
				uint8_t res_Lmax;
				uint8_t res_issb;
				uint8_t res_nhf;
				uint8_t res_eByte;
				int res_frame_offset = UE->rx_offset;

				NHAL_srchcmd_getSsbProfile(0, 0, &UE->frame_parms.Nid_cell, &res_issb, &res_Lmax, &res_nhf);
				NHAL_srchcmd_getPbchDec(0,0,&res_eByte, MIB_dout);
				

				
				
				#ifdef DNF_SYNCANDONSRCH
				//Fast search & window setting				
				ssbSlotOffset = res_nhf*UE->frame_parms.samples_per_slot*UE->frame_parms.slots_per_frame/2 + 
								UE->frame_parms.samples_per_slot*vhw_calcSsbSlotNb(narfcn, UE->frame_parms.numerology_index, res_issb); //further the full srching will search from this offset point
				ssbSymOffset = vhw_calcSsbSymbNb(narfcn, UE->frame_parms.numerology_index, res_issb)%UE->frame_parms.symbols_per_slot;
				if (ssbSymOffset > 0)
				{
					ssbSlotOffset += (UE->frame_parms.ofdm_symbol_size + UE->frame_parms.nb_prefix_samples0)
									 + (UE->frame_parms.ofdm_symbol_size + UE->frame_parms.nb_prefix_samples)*(ssbSymOffset-1)
									 - VSYNC_SPDUP_SRCHWINDOW/2;
					if (ssbSlotOffset < 0)
					{
						ssbSlotOffset = 0;
					}
				}
				#endif

				//synch configuration
				NHAL_RFcmd_cfgRxSync(res_frame_offset);
				UE->rx_offset=0;
				
				//set some parameters for the decoder part of the conventional code ------------------------------
				UE->frame_parms.nushift = (UE->frame_parms.Nid_cell)%4; //ue structure configuration
				UE->frame_parms.nb_antenna_ports_eNB = 1; //ue structure configuration
				UE->frame_parms.ssb_index = res_issb;
				UE->frame_parms.n_hf = res_nhf;
				for(int i=0; i<RX_NB_TH;i++)
				{
					  UE->proc.proc_rxtx[i].frame_tx = UE->proc.proc_rxtx[0].frame_rx;//ue structure configuration
				}
				configUeFrameParam(&(UE->frame_parms));
				nr_gold_pbch(UE);
				nr_gold_pdcch(UE,0, 2); //ue structure configuration		
				sendResUpper(UE, res_eByte, res_issb, res_Lmax);
				// -----------------------------------------------------------------------------------------------


				NHAL_srchcmd_off(0);	
				if (UE->UE_scan_carrier == 1)
				{
					UE->UE_scan_carrier = 0;
					#ifndef DNF_SYNCANDONSRCH
					NHAL_RFcmd_off();

					NHAL_RFcmd_on(NHAL_convSampleRate(rf_params.sample_rate), 
						UE->frame_parms.samples_per_frame, UE->frame_parms.samples_per_slot,
						downlink_frequency[CC_id][0]+freq_offset, downlink_frequency[CC_id][0]+uplink_frequency_offset[CC_id][0]+freq_offset,
						rf_params.rx_bw, rf_params.tx_bw, UE->rx_total_gain_dB, rf_params.rx_gain_offset[0], rf_params.tx_gain[0],
						nhal_syncState_outsync);	
					
					#endif
					
				} 
				else
				{
			    	UE->is_synchronized = 1; //RF register (sync mode RF (10ms + trash), TXRX mode RX (slot + slot number writing)
			    	#ifdef DNF_SYNCANDONSRCH
			    	ssbSlotOffset = 0;
					#endif
					UE->ssb_symbol_offset = vhw_calcSsbSymbNb(narfcn, UE->frame_parms.numerology_index, res_issb)%UE->frame_parms.symbols_per_slot; //further the full srching will search from this offset point;
					UE->ssb_slot_offset = res_nhf*UE->frame_parms.symbols_per_slot*UE->frame_parms.slots_per_frame/2 + vhw_calcSsbSlotNb(narfcn, UE->frame_parms.numerology_index, res_issb);
					
			    	LOG_E(PHY, "[SYNC time] : %d us\n", (int)((rdtsc_oai() - estTime)/cpuf/1000.0));
					
					NHAL_RFcmd_cfgRxMode(nhal_syncState_insync);

					
					narfcn = UE_calcNarfcn(downlink_frequency[CC_id][0]);
					NHAL_srchcmd_on(0, narfcn, srchMode_online, 
									UE->frame_parms.numerology_index, UE->frame_parms.ofdm_symbol_size,
									0, 0, //search basic option :: duration, srchSubframe
									UE->frame_parms.first_carrier_offset + UE->frame_parms.ssb_start_subcarrier, 2, //SSB configuration (freq. offset, period)
									UE->high_speed_flag,
									0, 1, //option : decimation, pbch CRC check
									UE->frame_parms.Nid_cell, 0, //PSS detection option : target NID, threshold
									VSYNC_SPDUP_SRCHWINDOW, 0, //search window, search offset
									res_issb, res_nhf); //issb, half frame indicator
			  	}
				
				freq_offset = 0;
			} 
			else
			{			
				// initial sync failed
				// calculate new offset and try again
				if (UE->UE_scan_carrier == 1)
				{
					if (ret == -3)
					{
						NHAL_RFcmd_cfgRxSync( UE->rx_offset);
						UE->rx_offset=0;
					}
					else if (ret == -2)
					{
						freq_offset += UE->common_vars.freq_offset;
						freq_offset -= (freq_offset%VSYNC_CHRASTER_STEPSIZE);
					}
					else
					{
						sync_cnt++;
						freq_offset = (sync_cnt+1)/2*VSYNC_CHRASTER_STEPSIZE;
						if (sync_cnt%2 == 1)
							freq_offset *= -1;
					}
					
			    	if (abs(freq_offset) > VSYNC_CHRASTER_STEPSIZE*VSYNC_CHRASTER_RANGE)
					{
			      		LOG_I( PHY, "[initial_sync] No cell synchronization found, abandoning\n" );
			      		FILE *fd;

			      		if ((fd = fopen("rxsig_frame0.dat","w"))!=NULL)
						{
			        		fwrite((void *)&( UE->common_vars.rxdata[0][0]),
														               sizeof(int32_t),
														               10*UE->frame_parms.samples_per_subframe,
														               fd);
			        		LOG_I(PHY,"Dummping Frame ... bye bye \n");
			        		fclose(fd);
			        		exit(0);
			      		}

			      		return &UE_thread_synch_retval; // not reached
			    	}
				}

			  	LOG_I(PHY, "[initial_sync] trying carrier off %d Hz, rxgain %d (DL %u, UL %u)\n",
									        freq_offset,
									        UE->rx_total_gain_dB,
									        downlink_frequency[0][0]+freq_offset,
									        downlink_frequency[0][0]+uplink_frequency_offset[0][0]+freq_offset );
				NHAL_RFcmd_cfgFreqOffset(freq_offset, nhal_freqOff_abs);
			}// initial_sync=0

			
		}
		else //synchronized case
		{
			usleep(1000);
		}

		VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME( VCD_SIGNAL_DUMPER_FUNCTIONS_UE_THREAD_SYNCH, 0 );
	}  // while !oai_exit

  	return &UE_thread_synch_retval;
}














void processSubframeRX( PHY_VARS_NR_UE *UE, UE_nr_rxtx_proc_t *proc) {
  // Process Rx data for one sub-frame
  if (slot_select_nr(&UE->frame_parms, proc->frame_tx, proc->nr_tti_tx) & NR_DOWNLINK_SLOT) {
    //clean previous FAPI MESSAGE
    UE->rx_ind.number_pdus = 0;
    UE->dci_ind.number_of_dcis = 0;
    //clean previous FAPI MESSAGE
    // call L2 for DL_CONFIG (DCI)
    UE->dcireq.module_id = UE->Mod_id;
    UE->dcireq.gNB_index = 0;
    UE->dcireq.cc_id     = 0;
    UE->dcireq.frame     = proc->frame_rx;
    UE->dcireq.slot      = proc->nr_tti_rx;
    nr_ue_dcireq(&UE->dcireq); //to be replaced with function pointer later
    NR_UE_MAC_INST_t *UE_mac = get_mac_inst(0);
    UE_mac->scheduled_response.dl_config = &UE->dcireq.dl_config_req;
    UE_mac->scheduled_response.slot = proc->nr_tti_rx;
    nr_ue_scheduled_response(&UE_mac->scheduled_response);
    //write_output("uerxdata_frame.m", "uerxdata_frame", UE->common_vars.rxdata[0], UE->frame_parms.samples_per_frame, 1, 1);
    //printf("Processing slot %d\n",proc->nr_tti_rx);
#ifdef UE_SLOT_PARALLELISATION
    phy_procedures_slot_parallelization_nrUE_RX( UE, proc, 0, 0, 1, UE->mode, no_relay, NULL );
#else
    phy_procedures_nrUE_RX( UE, proc, 0, 1, UE->mode);
    //            printf(">>> nr_ue_pdcch_procedures ended\n");
#endif
  }

  if (UE->mac_enabled==1) {
    //  trigger L2 to run ue_scheduler thru IF module
    //  [TODO] mapping right after NR initial sync
    if(UE->if_inst != NULL && UE->if_inst->ul_indication != NULL) {
      UE->ul_indication.module_id = 0;
      UE->ul_indication.gNB_index = 0;
      UE->ul_indication.cc_id = 0;
      UE->ul_indication.frame = proc->frame_rx;
      UE->ul_indication.slot = proc->nr_tti_rx;
      UE->if_inst->ul_indication(&UE->ul_indication);
    }
  }
}

/*!
 * \brief This is the UE thread for RX subframe n and TX subframe n+4.
 * This thread performs the phy_procedures_UE_RX() on every received slot.
 * then, if TX is enabled it performs TX for n+4.
 * \param arg is a pointer to a \ref PHY_VARS_NR_UE structure.
 * \returns a pointer to an int. The storage is not on the heap and must not be freed.
 */

static void *UE_thread_rxn_txnp4(void *arg) {
  struct nr_rxtx_thread_data *rtd = arg;
  UE_nr_rxtx_proc_t *proc = rtd->proc;
  PHY_VARS_NR_UE    *UE   = rtd->UE;
  //proc->counter_decoder = 0;
  proc->instance_cnt_rxtx=-1;
  proc->subframe_rx=proc->sub_frame_start;
  proc->dci_err_cnt=0;
  char threadname[256];
  sprintf(threadname,"UE_%d_proc_%d", UE->Mod_id, proc->sub_frame_start);
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);
  char timing_proc_name[256];
  sprintf(timing_proc_name,"Delay to process sub-frame proc %d",proc->sub_frame_start);

  if ( (proc->sub_frame_start+1)%RX_NB_TH == 0 && threads.one != -1 )
    CPU_SET(threads.one, &cpuset);

  if ( (proc->sub_frame_start+1)%RX_NB_TH == 1 && threads.two != -1 )
    CPU_SET(threads.two, &cpuset);

  if ( (proc->sub_frame_start+1)%RX_NB_TH == 2 && threads.three != -1 )
    CPU_SET(threads.three, &cpuset);

  //CPU_SET(threads.three, &cpuset);
  init_thread(900000,1000000, FIFO_PRIORITY-1, &cpuset,
              threadname);

  while (!oai_exit) {
    AssertFatal( 0 == pthread_mutex_lock(&proc->mutex_rxtx), "[SCHED][UE] error locking mutex for UE RXTX\n" );

    while (proc->instance_cnt_rxtx < 0) {
      // most of the time, the thread is waiting here
      pthread_cond_wait( &proc->cond_rxtx, &proc->mutex_rxtx );
    }

    AssertFatal ( 0== pthread_mutex_unlock(&proc->mutex_rxtx), "[SCHED][UE] error unlocking mutex for UE RXn_TXnp4\n" );
    processSubframeRX(UE, proc);
    //printf(">>> mac ended\n");
    // Prepare the future Tx data
    AssertFatal( 0 == pthread_mutex_lock(&proc->mutex_rxtx), "[SCHED][UE] error locking mutex for UE RXTX\n" );
    proc->instance_cnt_rxtx--;
#if BASIC_SIMULATOR

    if (pthread_cond_signal(&proc->cond_rxtx) != 0) abort();

#endif
    AssertFatal (0 == pthread_mutex_unlock(&proc->mutex_rxtx), "[SCHED][UE] error unlocking mutex for UE RXTX\n" );
  }

  // thread finished
  free(arg);
  return NULL;
}



//temporary function for code restructuring of vHW
void UE_threadPreInit(void *arg)
{
	PHY_VARS_NR_UE *UE = (PHY_VARS_NR_UE *) arg;

	for (int i=0; i<  RX_NB_TH_MAX; i++ )
		UE->proc.proc_rxtx[i].counter_decoder = 0;
	init_UE_threads(UE);
}

/*!
 * \brief Initialize the UE theads.
 * Creates the UE threads:
 * - UE_thread_rxtx0
 * - UE_thread_rxtx1
 * - UE_thread_synch
 * - UE_thread_fep_slot0
 * - UE_thread_fep_slot1
 * - UE_thread_dlsch_proc_slot0
 * - UE_thread_dlsch_proc_slot1
 * and the locking between them.
 */
void init_UE_threads(PHY_VARS_NR_UE *UE) {
  struct nr_rxtx_thread_data *rtd;
  pthread_mutex_init(&UE->proc.mutex_synch,NULL);
  pthread_cond_init(&UE->proc.cond_synch,NULL);
  UE->proc.instance_cnt_synch = -1;

  // the threads are not yet active, therefore access is allowed without locking
  for (int i=0; i<RX_NB_TH; i++) {
    rtd = calloc(1, sizeof(struct nr_rxtx_thread_data));

    if (rtd == NULL) abort();

    rtd->UE = UE;
    rtd->proc = &UE->proc.proc_rxtx[i];
    pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_rxtx,NULL);
    pthread_cond_init(&UE->proc.proc_rxtx[i].cond_rxtx,NULL);
    UE->proc.proc_rxtx[i].sub_frame_start=i;
    UE->proc.proc_rxtx[i].sub_frame_step=RX_NB_TH;
    printf("Init_UE_threads rtd %d proc %d nb_threads %d i %d\n",rtd->proc->sub_frame_start, UE->proc.proc_rxtx[i].sub_frame_start,RX_NB_TH, i);
    pthread_create(&UE->proc.proc_rxtx[i].pthread_rxtx, NULL, UE_thread_rxn_txnp4, rtd);
#ifdef UE_DLSCH_PARALLELISATION
    pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_dlsch_td,NULL);
    pthread_cond_init(&UE->proc.proc_rxtx[i].cond_dlsch_td,NULL);
    pthread_create(&UE->proc.proc_rxtx[i].pthread_dlsch_td,NULL,nr_dlsch_decoding_2thread0, rtd);
    //thread 2
    pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_dlsch_td1,NULL);
    pthread_cond_init(&UE->proc.proc_rxtx[i].cond_dlsch_td1,NULL);
    pthread_create(&UE->proc.proc_rxtx[i].pthread_dlsch_td1,NULL,nr_dlsch_decoding_2thread1, rtd);
#endif
#ifdef UE_SLOT_PARALLELISATION
    pthread_mutex_init(&UE->proc.proc_rxtx[i].mutex_slot1_dl_processing,NULL);
    pthread_cond_init(&UE->proc.proc_rxtx[i].cond_slot1_dl_processing,NULL);
    pthread_create(&UE->proc.proc_rxtx[i].pthread_slot1_dl_processing,NULL,UE_thread_slot1_dl_processing, rtd);
#endif
  }

  pthread_create(&UE->proc.pthread_synch,NULL,UE_thread_synch,(void *)UE);
}
