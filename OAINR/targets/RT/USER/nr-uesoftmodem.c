/*
 * Licensed to the OpenAirInterface (OAI) Software Alliance under one or more
 * contributor license agreements.  See the NOTICE file distributed with
 * this work for additional information regarding copyright ownership.
 * The OpenAirInterface Software Alliance licenses this file to You under
 * the OAI Public License, Version 1.1  (the "License"); you may not use this file
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


#define _GNU_SOURCE             /* See feature_test_macros(7) */
#include <sched.h>

#include "T.h"
#include "rt_wrapper.h"

#undef MALLOC //there are two conflicting definitions, so we better make sure we don't use it at all

#include "assertions.h"
#include "msc.h"

#include "PHY/types.h"
#include "PHY/defs_nr_UE.h"
#include "common/ran_context.h"
#include "common/config/config_userapi.h"
#include "common/utils/load_module_shlib.h"
#undef MALLOC //there are two conflicting definitions, so we better make sure we don't use it at all
//#undef FRAME_LENGTH_COMPLEX_SAMPLES //there are two conflicting definitions, so we better make sure we don't use it at all

#include "../../ARCH/COMMON/common_lib.h"
#include "../../ARCH/ETHERNET/USERSPACE/LIB/if_defs.h"

//#undef FRAME_LENGTH_COMPLEX_SAMPLES //there are two conflicting definitions, so we better make sure we don't use it at all

#include "PHY/phy_vars_nr_ue.h"
#include "PHY/LTE_TRANSPORT/transport_vars.h"
#include "SCHED/sched_common_vars.h"
#include "PHY/MODULATION/modulation_vars.h"
//#include "../../SIMU/USER/init_lte.h"

#include "LAYER2/MAC/mac.h"
#include "LAYER2/MAC/mac_vars.h"
#include "LAYER2/MAC/mac_proto.h"
#include "RRC/LTE/rrc_vars.h"
#include "PHY_INTERFACE/phy_interface_vars.h"

#ifdef SMBV
#include "PHY/TOOLS/smbv.h"
unsigned short config_frames[4] = {2,9,11,13};
#endif
#include "common/utils/LOG/log.h"
#include "common/utils/LOG/vcd_signal_dumper.h"

#include "UTIL/OPT/opt.h"
#include "enb_config.h"
//#include "PHY/TOOLS/time_meas.h"

#ifndef OPENAIR2
#include "UTIL/OTG/otg_vars.h"
#endif

#if defined(ENABLE_ITTI)
#include "intertask_interface.h"
#include "create_tasks.h"
#endif

#include "PHY/INIT/phy_init.h"
#include "system.h"
#include "stats.h"
#ifdef XFORMS
#include "PHY/TOOLS/nr_phy_scope.h"
//#include "stats.h"
// current status is that every UE has a DL scope for a SINGLE eNB (eNB_id=0)
// at eNB 0, an UL scope for every UE
FD_lte_phy_scope_ue  *form_ue[NUMBER_OF_UE_MAX];
//FD_lte_phy_scope_enb *form_enb[MAX_NUM_CCs][NUMBER_OF_UE_MAX];
//FD_stats_form                  *form_stats=NULL,*form_stats_l2=NULL;
char title[255];
//unsigned char                   scope_enb_num_ue = 2;
static pthread_t                forms_thread; //xforms
#endif //XFORMS
#include "nr-uesoftmodem.h"

RAN_CONTEXT_t RC;

pthread_cond_t sync_cond;
pthread_mutex_t sync_mutex;
int sync_var=-1; //!< protected by mutex \ref sync_mutex.

uint16_t runtime_phy_rx[29][6]; // SISO [MCS 0-28][RBs 0-5 : 6, 15, 25, 50, 75, 100]
uint16_t runtime_phy_tx[29][6]; // SISO [MCS 0-28][RBs 0-5 : 6, 15, 25, 50, 75, 100]

#if defined(ENABLE_ITTI)
volatile int             start_eNB = 0;
volatile int             start_UE = 0;
#endif
volatile int             oai_exit = 0;

static clock_source_t clock_source = internal;
static int wait_for_sync = 0;

unsigned int             mmapped_dma=0;
int                      single_thread_flag=1;

int                      threequarter_fs=0;

uint32_t                 downlink_frequency[MAX_NUM_CCs][4];
int32_t                  uplink_frequency_offset[MAX_NUM_CCs][4];

extern int16_t nr_dlsch_demod_shift;


//static char                    *conf_config_file_name = NULL;

#if defined(ENABLE_ITTI)
static char                    *itti_dump_file = NULL;
#endif

int UE_scan = 0;
int UE_scan_carrier = 0;
int UE_fo_compensation = 0;
runmode_t mode = normal_txrx;

FILE *input_fd=NULL;


#if MAX_NUM_CCs == 1
rx_gain_t                rx_gain_mode[MAX_NUM_CCs][4] = {{max_gain,max_gain,max_gain,max_gain}};
double tx_gain[MAX_NUM_CCs][4] = {{20,0,0,0}};
double rx_gain[MAX_NUM_CCs][4] = {{110,0,0,0}};
#else
rx_gain_t                rx_gain_mode[MAX_NUM_CCs][4] = {{max_gain,max_gain,max_gain,max_gain},{max_gain,max_gain,max_gain,max_gain}};
double tx_gain[MAX_NUM_CCs][4] = {{20,0,0,0},{20,0,0,0}};
double rx_gain[MAX_NUM_CCs][4] = {{110,0,0,0},{20,0,0,0}};
#endif

double rx_gain_off = 0.0;

double sample_rate=30.72e6;
double bw = 10.0e6;

static int  tx_max_power[MAX_NUM_CCs] = {0};

char   rf_config_file[1024];

int chain_offset=0;
int phy_test = 0;
uint8_t usim_test = 0;

uint8_t dci_Format = 0;
uint8_t agregation_Level =0xFF;

uint8_t nb_antenna_tx = 1;
uint8_t nb_antenna_rx = 1;

char ref[128] = "internal";
char channels[128] = "0";

char *parallel_config = NULL;
char *worker_config = NULL;

static THREAD_STRUCT thread_struct;
void set_parallel_conf(char *parallel_conf)
{
  if(strcmp(parallel_conf,"PARALLEL_SINGLE_THREAD")==0)           thread_struct.parallel_conf = PARALLEL_SINGLE_THREAD;
  else if(strcmp(parallel_conf,"PARALLEL_RU_L1_SPLIT")==0)        thread_struct.parallel_conf = PARALLEL_RU_L1_SPLIT;
  else if(strcmp(parallel_conf,"PARALLEL_RU_L1_TRX_SPLIT")==0)    thread_struct.parallel_conf = PARALLEL_RU_L1_TRX_SPLIT;
  printf("[CONFIG] parallel conf is set to %d\n",thread_struct.parallel_conf);
} 
void set_worker_conf(char *worker_conf)
{
  if(strcmp(worker_conf,"WORKER_DISABLE")==0)                     thread_struct.worker_conf = WORKER_DISABLE;
  else if(strcmp(worker_conf,"WORKER_ENABLE")==0)                 thread_struct.worker_conf = WORKER_ENABLE;
  printf("[CONFIG] worker conf is set to %d\n",thread_struct.worker_conf);
} 
PARALLEL_CONF_t get_thread_parallel_conf(void)
{
  return thread_struct.parallel_conf;
} 
WORKER_CONF_t get_thread_worker_conf(void)
{
  return thread_struct.worker_conf;
} 
int                      		rx_input_level_dBm;

//static int                      online_log_messages=0;

#ifdef XFORMS
extern int                      otg_enabled;
int                             do_forms=0;
#else
int                             otg_enabled;
#endif
//int                             number_of_cards =   1;

static NR_DL_FRAME_PARMS      *frame_parms[MAX_NUM_CCs];
static nfapi_nr_config_request_t *config[MAX_NUM_CCs];
int16_t   node_synch_ref[MAX_NUM_CCs];

uint32_t target_dl_mcs = 28; //maximum allowed mcs
uint32_t target_ul_mcs = 20;
uint32_t timing_advance = 0;
uint8_t exit_missed_slots=1;
uint64_t num_missed_slots=0; // counter for the number of missed slots

extern PHY_VARS_NR_UE* init_nr_ue_vars(NR_DL_FRAME_PARMS *frame_parms,
			  uint8_t UE_id,
			  uint8_t abstraction_flag);

int transmission_mode=1;
int numerology = 0;

/*
int16_t           glog_level         = LOG_INFO;
int16_t           glog_verbosity     = LOG_MED;
int16_t           hw_log_level       = LOG_INFO;
int16_t           hw_log_verbosity   = LOG_MED;
int16_t           phy_log_level      = LOG_INFO;
int16_t           phy_log_verbosity  = LOG_MED;
int16_t           mac_log_level      = LOG_INFO;
int16_t           mac_log_verbosity  = LOG_MED;
int16_t           rlc_log_level      = LOG_INFO;
int16_t           rlc_log_verbosity  = LOG_MED;
int16_t           pdcp_log_level     = LOG_INFO;
int16_t           pdcp_log_verbosity = LOG_MED;
int16_t           rrc_log_level      = LOG_INFO;
int16_t           rrc_log_verbosity  = LOG_MED;
int16_t           opt_log_level      = LOG_INFO;
int16_t           opt_log_verbosity  = LOG_MED;

# if defined(ENABLE_USE_MME)
int16_t           gtpu_log_level     = LOG_DEBUG;
int16_t           gtpu_log_verbosity = LOG_MED;
int16_t           udp_log_level      = LOG_DEBUG;
int16_t           udp_log_verbosity  = LOG_MED;
#endif
#if defined (ENABLE_SECURITY)
int16_t           osa_log_level      = LOG_INFO;
int16_t           osa_log_verbosity  = LOG_MED;
#endif
*/

char *rrh_UE_ip = "127.0.0.1";
int rrh_UE_port = 51000;

/* flag set by eNB conf file to specify if the radio head is local or remote (default option is local) */
//uint8_t local_remote_radio = BBU_LOCAL_RADIO_HEAD;
/* struct for ethernet specific parameters given in eNB conf file */
//eth_params_t *eth_params;

openair0_config_t rf_params;

double cpuf;

char uecap_xer[1024],uecap_xer_in=0;

int oaisim_flag=0;
int emulate_rf = 0;

threads_t threads = {-1,-1,-1,-1,-1,-1,-1,-1};
int threads_offset = 0;

char* usrp_args=NULL;
char* usrp_clksrc=NULL;

/* forward declarations */
void set_default_frame_parms(NR_DL_FRAME_PARMS *frame_parms[MAX_NUM_CCs]);


/* see file openair2/LAYER2/MAC/main.c for why abstraction_flag is needed
 * this is very hackish - find a proper solution
 */
uint8_t abstraction_flag=0;

/*---------------------BMC: timespec helpers -----------------------------*/

struct timespec min_diff_time = { .tv_sec = 0, .tv_nsec = 0 };
struct timespec max_diff_time = { .tv_sec = 0, .tv_nsec = 0 };

struct timespec clock_difftime(struct timespec start, struct timespec end) {
    struct timespec temp;
    if ((end.tv_nsec-start.tv_nsec)<0) {
        temp.tv_sec = end.tv_sec-start.tv_sec-1;
        temp.tv_nsec = 1000000000+end.tv_nsec-start.tv_nsec;
    } else {
        temp.tv_sec = end.tv_sec-start.tv_sec;
        temp.tv_nsec = end.tv_nsec-start.tv_nsec;
    }
    return temp;
}

void print_difftimes(void) {
#ifdef DEBUG
    printf("difftimes min = %lu ns ; max = %lu ns\n", min_diff_time.tv_nsec, max_diff_time.tv_nsec);
#else
    LOG_I(HW,"difftimes min = %lu ns ; max = %lu ns\n", min_diff_time.tv_nsec, max_diff_time.tv_nsec);
#endif
}

void update_difftimes(struct timespec start, struct timespec end) {
    struct timespec diff_time = { .tv_sec = 0, .tv_nsec = 0 };
    int             changed = 0;
    diff_time = clock_difftime(start, end);
    if ((min_diff_time.tv_nsec == 0) || (diff_time.tv_nsec < min_diff_time.tv_nsec)) {
        min_diff_time.tv_nsec = diff_time.tv_nsec;
        changed = 1;
    }
    if ((max_diff_time.tv_nsec == 0) || (diff_time.tv_nsec > max_diff_time.tv_nsec)) {
        max_diff_time.tv_nsec = diff_time.tv_nsec;
        changed = 1;
    }
#if 1
    if (changed) print_difftimes();
#endif
}

/*------------------------------------------------------------------------*/

unsigned int build_rflocal(int txi, int txq, int rxi, int rxq) {
    return (txi + (txq<<6) + (rxi<<12) + (rxq<<18));
}
unsigned int build_rfdc(int dcoff_i_rxfe, int dcoff_q_rxfe) {
    return (dcoff_i_rxfe + (dcoff_q_rxfe<<8));
}

#if !defined(ENABLE_ITTI)
void signal_handler(int sig) {
    void *array[10];
    size_t size;

    if (sig==SIGSEGV) {
        // get void*'s for all entries on the stack
        size = backtrace(array, 10);

        // print out all the frames to stderr
        fprintf(stderr, "Error: signal %d:\n", sig);
        backtrace_symbols_fd(array, size, 2);
        exit(-1);
    } else {
        printf("trying to exit gracefully...\n");
        oai_exit = 1;
    }
}
#endif
#define KNRM  "\x1B[0m"
#define KRED  "\x1B[31m"
#define KGRN  "\x1B[32m"
#define KBLU  "\x1B[34m"
#define RESET "\033[0m"


void exit_function(const char* file, const char* function, const int line, const char* s)
{

  int CC_id;

  if (s != NULL)
  {
    printf("%s:%d %s() Exiting OAI softmodem: %s\n",file,line, function, s);
  }

  oai_exit = 1;

#if defined(ENABLE_ITTI)
    sleep(1); //allow lte-softmodem threads to exit first
    itti_terminate_tasks (TASK_UNKNOWN);
#endif
}

#ifdef XFORMS


void reset_stats(FL_OBJECT *button, long arg) {
    //int i,j,k;
    /*PHY_VARS_eNB *phy_vars_eNB = PHY_vars_eNB_g[0][0];

    for (i=0; i<NUMBER_OF_UE_MAX; i++) {
        for (k=0; k<8; k++) { //harq_processes
            for (j=0; j<phy_vars_eNB->dlsch[i][0]->Mlimit; j++) {
                phy_vars_eNB->UE_stats[i].dlsch_NAK[k][j]=0;
                phy_vars_eNB->UE_stats[i].dlsch_ACK[k][j]=0;
                phy_vars_eNB->UE_stats[i].dlsch_trials[k][j]=0;
            }

            phy_vars_eNB->UE_stats[i].dlsch_l2_errors[k]=0;
            phy_vars_eNB->UE_stats[i].ulsch_errors[k]=0;
            phy_vars_eNB->UE_stats[i].ulsch_consecutive_errors=0;

            for (j=0; j<phy_vars_eNB->ulsch[i]->Mlimit; j++) {
                phy_vars_eNB->UE_stats[i].ulsch_decoding_attempts[k][j]=0;
                phy_vars_eNB->UE_stats[i].ulsch_decoding_attempts_last[k][j]=0;
                phy_vars_eNB->UE_stats[i].ulsch_round_errors[k][j]=0;
                phy_vars_eNB->UE_stats[i].ulsch_round_fer[k][j]=0;
            }
        }

        phy_vars_eNB->UE_stats[i].dlsch_sliding_cnt=0;
        phy_vars_eNB->UE_stats[i].dlsch_NAK_round0=0;
        phy_vars_eNB->UE_stats[i].dlsch_mcs_offset=0;
    }*/
}

static void *scope_thread(void *arg) {
# ifdef ENABLE_XFORMS_WRITE_STATS
    FILE *UE_stats, *eNB_stats;
# endif
    //int len = 0;
    struct sched_param sched_param;
    //int UE_id, CC_id;
    //int ue_cnt=0;

    sched_param.sched_priority = sched_get_priority_min(SCHED_FIFO)+1;
    sched_setscheduler(0, SCHED_FIFO,&sched_param);

    printf("Scope thread has priority %d\n",sched_param.sched_priority);

# ifdef ENABLE_XFORMS_WRITE_STATS

  UE_stats  = fopen("UE_stats.txt", "w");

#endif

  while (!oai_exit) {
      //len = dump_ue_stats (PHY_vars_UE_g[0][0], &PHY_vars_UE_g[0][0]->proc.proc_rxtx[0],stats_buffer, 0, mode,rx_input_level_dBm);
      //fl_set_object_label(form_stats->stats_text, stats_buffer);
      //fl_clear_browser(form_stats->stats_text);
      //fl_add_browser_line(form_stats->stats_text, stats_buffer);
      
      //if (PHY_vars_UE_g[0][0]->is_synchronized == 1)
	phy_scope_UE(form_ue[0],
		     PHY_vars_UE_g[0][0],
		     0,0,1);
      //else it is done in the synch thread
      
      //printf("doing forms\n");
      //usleep(100000); // 100 ms
      sleep(0.1);
    }

    //  printf("%s",stats_buffer);

# ifdef ENABLE_XFORMS_WRITE_STATS

        if (UE_stats) {
            rewind (UE_stats);
            fwrite (stats_buffer, 1, len, UE_stats);
            fclose (UE_stats);
        }

# endif

    pthread_exit((void*)arg);
}
#endif




#if defined(ENABLE_ITTI)
void *l2l1_task(void *arg) {
    MessageDef *message_p = NULL;
    int         result;

    itti_set_task_real_time(TASK_L2L1);
    itti_mark_task_ready(TASK_L2L1);


  do {
    // Wait for a message
    itti_receive_msg (TASK_L2L1, &message_p);

    switch (ITTI_MSG_ID(message_p)) {
    case TERMINATE_MESSAGE:
            oai_exit=1;
            itti_exit_task ();
            break;

        case ACTIVATE_MESSAGE:
            start_UE = 1;
            break;

        case DEACTIVATE_MESSAGE:
            start_UE = 0;
            break;

        case MESSAGE_TEST:
            LOG_I(EMU, "Received %s\n", ITTI_MSG_NAME(message_p));
            break;

        default:
            LOG_E(EMU, "Received unexpected message %s\n", ITTI_MSG_NAME(message_p));
            break;
        }

        result = itti_free (ITTI_MSG_ORIGIN_ID(message_p), message_p);
        AssertFatal (result == EXIT_SUCCESS, "Failed to free memory (%d)!\n", result);
    } while(!oai_exit);

    return NULL;
}
#endif

int16_t dlsch_demod_shift;

static void get_options(void) {
  int CC_id;
  int tddflag, nonbiotflag;
  char *loopfile=NULL;
  int dumpframe=0;
  uint32_t online_log_messages;
  uint32_t glog_level, glog_verbosity;
  uint32_t start_telnetsrv;

  paramdef_t cmdline_params[] =CMDLINE_PARAMS_DESC ;
  paramdef_t cmdline_logparams[] =CMDLINE_LOGPARAMS_DESC ;

  config_process_cmdline( cmdline_params,sizeof(cmdline_params)/sizeof(paramdef_t),NULL); 

  if (strlen(in_path) > 0) {
      opt_type = OPT_PCAP;
      opt_enabled=1;
      printf("Enabling OPT for PCAP  with the following file %s \n",in_path);
  }
  if (strlen(in_ip) > 0) {
      opt_enabled=1;
      opt_type = OPT_WIRESHARK;
      printf("Enabling OPT for wireshark for local interface");
  }

  config_process_cmdline( cmdline_logparams,sizeof(cmdline_logparams)/sizeof(paramdef_t),NULL);
  if(config_isparamset(cmdline_logparams,CMDLINE_ONLINELOG_IDX)) {
      set_glog_onlinelog(online_log_messages);
  }
  if(config_isparamset(cmdline_logparams,CMDLINE_GLOGLEVEL_IDX)) {
    set_glog(glog_level);
  }

  if (start_telnetsrv) {
    load_module_shlib("telnetsrv",NULL,0,NULL);
  }

  paramdef_t cmdline_uemodeparams[] =CMDLINE_UEMODEPARAMS_DESC;
  paramdef_t cmdline_ueparams[] =CMDLINE_UEPARAMS_DESC;


  config_process_cmdline( cmdline_uemodeparams,sizeof(cmdline_uemodeparams)/sizeof(paramdef_t),NULL);
  config_process_cmdline( cmdline_ueparams,sizeof(cmdline_ueparams)/sizeof(paramdef_t),NULL);
  if (loopfile != NULL) {
      printf("Input file for hardware emulation: %s\n",loopfile);
      mode=loop_through_memory;
      input_fd = fopen(loopfile,"r");
      AssertFatal(input_fd != NULL,"Please provide a valid input file\n");
  }

  if ( (cmdline_uemodeparams[CMDLINE_CALIBUERX_IDX].paramflags &  PARAMFLAG_PARAMSET) != 0) mode = rx_calib_ue;
  if ( (cmdline_uemodeparams[CMDLINE_CALIBUERXMED_IDX].paramflags &  PARAMFLAG_PARAMSET) != 0) mode = rx_calib_ue_med;
  if ( (cmdline_uemodeparams[CMDLINE_CALIBUERXBYP_IDX].paramflags &  PARAMFLAG_PARAMSET) != 0) mode = rx_calib_ue_byp;
  if (cmdline_uemodeparams[CMDLINE_DEBUGUEPRACH_IDX].uptr)
      if ( *(cmdline_uemodeparams[CMDLINE_DEBUGUEPRACH_IDX].uptr) > 0) mode = debug_prach;
  if (cmdline_uemodeparams[CMDLINE_NOL2CONNECT_IDX].uptr)
      if ( *(cmdline_uemodeparams[CMDLINE_NOL2CONNECT_IDX].uptr) > 0)  mode = no_L2_connect;
  if (cmdline_uemodeparams[CMDLINE_CALIBPRACHTX_IDX].uptr) 
      if ( *(cmdline_uemodeparams[CMDLINE_CALIBPRACHTX_IDX].uptr) > 0) mode = calib_prach_tx; 
  if (dumpframe  > 0)  mode = rx_dump_frame;
  
  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
    frame_parms[CC_id]->dl_CarrierFreq = downlink_frequency[0][0];
  }
  UE_scan=0;
   

  if (tddflag > 0) {
     for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) 
    	 frame_parms[CC_id]->frame_type = TDD;
  }

  /*if (frame_parms[0]->N_RB_DL !=0) {
      if ( frame_parms[0]->N_RB_DL < 6 ) {
    	 frame_parms[0]->N_RB_DL = 6;
    	 printf ( "%i: Invalid number of ressource blocks, adjusted to 6\n",frame_parms[0]->N_RB_DL);
      }
      if ( frame_parms[0]->N_RB_DL > 100 ) {
    	 frame_parms[0]->N_RB_DL = 100;
    	 printf ( "%i: Invalid number of ressource blocks, adjusted to 100\n",frame_parms[0]->N_RB_DL);
      }
      if ( frame_parms[0]->N_RB_DL > 50 && frame_parms[0]->N_RB_DL < 100 ) {
    	 frame_parms[0]->N_RB_DL = 50;
    	 printf ( "%i: Invalid number of ressource blocks, adjusted to 50\n",frame_parms[0]->N_RB_DL);
      }
      if ( frame_parms[0]->N_RB_DL > 25 && frame_parms[0]->N_RB_DL < 50 ) {
    	 frame_parms[0]->N_RB_DL = 25;
    	 printf ( "%i: Invalid number of ressource blocks, adjusted to 25\n",frame_parms[0]->N_RB_DL);
      }
      UE_scan = 0;
      frame_parms[0]->N_RB_UL=frame_parms[0]->N_RB_DL;
      for (CC_id=1; CC_id<MAX_NUM_CCs; CC_id++) {
    	  frame_parms[CC_id]->N_RB_DL=frame_parms[0]->N_RB_DL;
    	  frame_parms[CC_id]->N_RB_UL=frame_parms[0]->N_RB_UL;
      }
  }*/


  for (CC_id=1;CC_id<MAX_NUM_CCs;CC_id++) {
    	tx_max_power[CC_id]=tx_max_power[0];
    	rx_gain[0][CC_id] = rx_gain[0][0];
    	tx_gain[0][CC_id] = tx_gain[0][0];
  }

#if T_TRACER
  paramdef_t cmdline_ttraceparams[] =CMDLINE_TTRACEPARAMS_DESC ;
  config_process_cmdline( cmdline_ttraceparams,sizeof(cmdline_ttraceparams)/sizeof(paramdef_t),NULL);   
#endif

  if ( !(CONFIG_ISFLAGSET(CONFIG_ABORT))  && (!(CONFIG_ISFLAGSET(CONFIG_NOOOPT))) ) {
    // Here the configuration file is the XER encoded UE capabilities
    // Read it in and store in asn1c data structures
    sprintf(uecap_xer,"%stargets/PROJECTS/GENERIC-LTE-EPC/CONF/UE_config.xml",getenv("OPENAIR_HOME"));
    printf("%s\n",uecap_xer);
    uecap_xer_in=1;
  } /* UE with config file  */

}

#if T_TRACER
int T_nowait = 0;       /* by default we wait for the tracer */
int T_port = 2021;    /* default port to listen to to wait for the tracer */
int T_dont_fork = 0;  /* default is to fork, see 'T_init' to understand */
#endif

void set_default_frame_parms(NR_DL_FRAME_PARMS *frame_parms[MAX_NUM_CCs]) {

  int CC_id;

  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
        frame_parms[CC_id] = (NR_DL_FRAME_PARMS*) malloc(sizeof(NR_DL_FRAME_PARMS));
        /* Set some default values that may be overwritten while reading options */
        frame_parms[CC_id] = (NR_DL_FRAME_PARMS*) malloc(sizeof(NR_DL_FRAME_PARMS));
        config[CC_id] = (nfapi_nr_config_request_t*) malloc(sizeof(nfapi_nr_config_request_t));
        config[CC_id]->subframe_config.numerology_index_mu.value =1;
        config[CC_id]->subframe_config.duplex_mode.value = 1; //FDD
        config[CC_id]->subframe_config.dl_cyclic_prefix_type.value = 0; //NORMAL
        config[CC_id]->rf_config.dl_carrier_bandwidth.value = 106;
        config[CC_id]->rf_config.ul_carrier_bandwidth.value = 106;
        config[CC_id]->sch_config.physical_cell_id.value = 0;

        frame_parms[CC_id]->frame_type          = FDD;
        frame_parms[CC_id]->tdd_config          = 3;
        //frame_parms[CC_id]->tdd_config_S        = 0;
        frame_parms[CC_id]->N_RB_DL             = 106;
        frame_parms[CC_id]->N_RB_UL             = 106;
        frame_parms[CC_id]->Ncp                 = NORMAL;
        //frame_parms[CC_id]->Ncp_UL              = NORMAL;
        frame_parms[CC_id]->Nid_cell            = 0;
        //frame_parms[CC_id]->num_MBSFN_config    = 0;
        frame_parms[CC_id]->nb_antenna_ports_eNB  = 1;
        frame_parms[CC_id]->nb_antennas_tx      = 1;
        frame_parms[CC_id]->nb_antennas_rx      = 1;

        //frame_parms[CC_id]->nushift             = 0;

        ///frame_parms[CC_id]->phich_config_common.phich_resource = oneSixth;
        //frame_parms[CC_id]->phich_config_common.phich_duration = normal;

	// UL RS Config
        /*frame_parms[CC_id]->pusch_config_common.ul_ReferenceSignalsPUSCH.cyclicShift = 1;//n_DMRS1 set to 0
        frame_parms[CC_id]->pusch_config_common.ul_ReferenceSignalsPUSCH.groupHoppingEnabled = 1;
        frame_parms[CC_id]->pusch_config_common.ul_ReferenceSignalsPUSCH.sequenceHoppingEnabled = 0;
        frame_parms[CC_id]->pusch_config_common.ul_ReferenceSignalsPUSCH.groupAssignmentPUSCH = 0;

	frame_parms[CC_id]->pusch_config_common.n_SB = 1;
	frame_parms[CC_id]->pusch_config_common.hoppingMode = 0;
	frame_parms[CC_id]->pusch_config_common.pusch_HoppingOffset = 0;
	frame_parms[CC_id]->pusch_config_common.enable64QAM = 0;
		
        frame_parms[CC_id]->prach_config_common.rootSequenceIndex=22;
        frame_parms[CC_id]->prach_config_common.prach_ConfigInfo.zeroCorrelationZoneConfig=1;
        frame_parms[CC_id]->prach_config_common.prach_ConfigInfo.prach_ConfigIndex=0;
        frame_parms[CC_id]->prach_config_common.prach_ConfigInfo.highSpeedFlag=0;
        frame_parms[CC_id]->prach_config_common.prach_ConfigInfo.prach_FreqOffset=0;*/

        // NR: Init to legacy LTE 20Mhz params
        frame_parms[CC_id]->numerology_index	= 0;
        frame_parms[CC_id]->ttis_per_subframe	= 1;
        frame_parms[CC_id]->slots_per_tti		= 2;

    }

}

void init_rfparam(void);
void init_rfparam() {

    int i;

	rf_params.mmapped_dma=mmapped_dma;
	rf_params.configFilename = NULL;

    if(frame_parms[0]->N_RB_DL == 106)
	{
		if (numerology==0)
		{
            if (frame_parms[0]->threequarter_fs)
			{
                rf_params.sample_rate=23.04e6;
                rf_params.samples_per_frame = 230400;
                rf_params.tx_bw = 10e6;
                rf_params.rx_bw = 10e6;
            }
			else
			{
                rf_params.sample_rate=30.72e6;
                //rf_params.samples_per_frame = 307200 - 16*10; //16*10 is the temp code, this shouldbe modified after gNB bug is fixed
                rf_params.samples_per_frame = 307200;
#ifdef FIX_ENABLENRUSRPB210
                rf_params.tx_bw = 20e6;
                rf_params.rx_bw = 20e6;
#else
                rf_params.tx_bw = 10e6;
                rf_params.rx_bw = 10e6;
#endif
            }
	  	} 
		else if (numerology==1)
		{
		    rf_params.sample_rate=61.44e6;
		    rf_params.samples_per_frame = 307200;
		    rf_params.tx_bw = 20e6;
		    rf_params.rx_bw = 20e6;
	  	} 
		else if (numerology==2)
		{
		    rf_params.sample_rate=122.88e6;
		    rf_params.samples_per_frame = 307200;
		    rf_params.tx_bw = 40e6;
		    rf_params.rx_bw = 40e6;
	  	} 
	  	else
		{
	    	LOG_E(PHY,"Unsupported numerology!\n");
		    exit(-1);
	  	}
	} 
	else if(frame_parms[0]->N_RB_DL == 50)
	{
        rf_params.sample_rate=15.36e6;
        rf_params.samples_per_frame = 153600;
        rf_params.tx_bw = 5e6;
        rf_params.rx_bw = 5e6;
    }
	else if (frame_parms[0]->N_RB_DL == 25)
	{
        rf_params.sample_rate=7.68e6;
        rf_params.samples_per_frame = 76800;
        rf_params.tx_bw = 2.5e6;
        rf_params.rx_bw = 2.5e6;
    }
	else if (frame_parms[0]->N_RB_DL == 6)
	{
        rf_params.sample_rate=1.92e6;
        rf_params.samples_per_frame = 19200;
        rf_params.tx_bw = 1.5e6;
        rf_params.rx_bw = 1.5e6;
    }

	if (frame_parms[0]->frame_type==TDD)
		rf_params.duplex_mode = duplex_mode_TDD;
	else //FDD
	  	rf_params.duplex_mode = duplex_mode_FDD;
	
	printf("HW: Configuring nb_antennas_tx/rx %d/%d\n",
	       PHY_vars_UE_g[0][0]->frame_parms.nb_antennas_tx,
	       PHY_vars_UE_g[0][0]->frame_parms.nb_antennas_rx);
	rf_params.Mod_id = 0;
	
	rf_params.num_rb_dl = frame_parms[0]->N_RB_DL;
	
	rf_params.clock_source = clock_source;
	
	
	rf_params.tx_num_channels = min(2,PHY_vars_UE_g[0][0]->frame_parms.nb_antennas_tx);
	rf_params.rx_num_channels = min(2,PHY_vars_UE_g[0][0]->frame_parms.nb_antennas_rx);

	for (i=0; i<4; i++)
	{
	  	if (i < rf_params.tx_num_channels)
	    	rf_params.tx_freq[i] = downlink_frequency[0][i]+uplink_frequency_offset[0][i];
	  	else
	    	rf_params.tx_freq[i]=0.0;
	  
	  	if (i < rf_params.rx_num_channels)
	    	rf_params.rx_freq[i] = downlink_frequency[0][i];
	  	else
	    	rf_params.rx_freq[i]=0.0;
	  
	  	rf_params.autocal[i] = 1;
	  	rf_params.tx_gain[i] = tx_gain[0][i];
	  	rf_params.rx_gain[i] = PHY_vars_UE_g[0][0]->rx_total_gain_dB - rx_gain_off;
	  
	  
	  	rf_params.configFilename = rf_config_file;
	  	printf("channel %d, Setting tx_gain %f, rx_gain %f, tx_freq %f, rx_freq %f\n",
		 				i, rf_params.tx_gain[i],
		rf_params.rx_gain[i],
		rf_params.tx_freq[i],
		rf_params.rx_freq[i]);
	}
	
	if (usrp_args) rf_params.sdr_addrs = usrp_args;
	if (usrp_clksrc)
	{
		if (strcmp(usrp_clksrc, "internal") == 0)
		{
			rf_params.clock_source = internal;
			LOG_D(PHY, "USRP clock source set as internal\n");
		}
		else if (strcmp(usrp_clksrc, "external") == 0)
		{
			rf_params.clock_source = external;
			LOG_D(PHY, "USRP clock source set as external\n");
		}
		else if (strcmp(usrp_clksrc, "gpsdo") == 0)
		{
			rf_params.clock_source = gpsdo;
			LOG_D(PHY, "USRP clock source set as gpsdo\n");
		}
		else
		{
			rf_params.clock_source = internal;
			LOG_I(PHY, "USRP clock source unknown ('%s'). defaulting to internal\n", usrp_clksrc);	
		}
	}
	else
	{
	  rf_params.clock_source = internal;
	  LOG_I(PHY, "USRP clock source not specified. defaulting to internal\n");	
	}
}

int main( int argc, char **argv ) {
    int i;//j,k,aa,re;
#if defined (XFORMS)
    void *status;
#endif

    int CC_id;
    uint8_t  abstraction_flag=0;
    //uint8_t beta_ACK=0,beta_RI=0,beta_CQI=2;

#if defined (XFORMS)
    int ret;
#endif

    PHY_VARS_NR_UE *UE[MAX_NUM_CCs];

    start_background_system();
    if ( load_configmodule(argc,argv) == NULL) {
      exit_fun("[SOFTMODEM] Error, configuration module init failed\n");
    }
    CONFIG_SETRTFLAG(CONFIG_NOEXITONHELP);
 
#ifdef DEBUG_CONSOLE
    setvbuf(stdout, NULL, _IONBF, 0);
    setvbuf(stderr, NULL, _IONBF, 0);
#endif

    set_default_frame_parms(frame_parms);

    mode = normal_txrx;
    memset(&rf_params,0,sizeof(openair0_config_t));

    memset(tx_max_power,0,sizeof(int)*MAX_NUM_CCs);

    set_latency_target();

    // initialize logging
    logInit();


    // get options and fill parameters from configuration file
    get_options (); //Command-line options, enb_properties

#if T_TRACER
    T_Config_Init();
#endif

    //randominit (0);
    set_taus_seed (0);

    printf("configuring for UE\n");

    //if (ouput_vcd)
    //    VCD_SIGNAL_DUMPER_INIT("/tmp/openair_dump_UE.vcd");

    //if (opp_enabled ==1) {
    //    reset_opp_meas();
    //}
    cpuf=get_cpu_freq_GHz();

#if defined(ENABLE_ITTI)
    //log_set_instance_type (LOG_INSTANCE_UE);

    itti_init(TASK_MAX, THREAD_MAX, MESSAGES_ID_MAX, tasks_info, messages_info);

    // initialize mscgen log after ITTI
    MSC_INIT(MSC_E_UTRAN, THREAD_MAX+TASK_MAX);
#endif

    if (opt_type != OPT_NONE) {
        if (init_opt(in_path, in_ip) == -1)
            LOG_E(OPT,"failed to run OPT \n");
    }

#ifdef PDCP_USE_NETLINK
    netlink_init();
#if defined(PDCP_USE_NETLINK_QUEUES)
    pdcp_netlink_init();
#endif
#endif

#if !defined(ENABLE_ITTI)
    // to make a graceful exit when ctrl-c is pressed
    signal(SIGSEGV, signal_handler);
    signal(SIGINT, signal_handler);
#endif


  check_clock();

#ifndef PACKAGE_VERSION
#  define PACKAGE_VERSION "UNKNOWN-EXPERIMENTAL"
#endif

  LOG_I(HW, "Version: %s\n", PACKAGE_VERSION);
  
  // init the parameters
  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {

      frame_parms[CC_id]->nb_antennas_tx     = nb_antenna_tx;
      frame_parms[CC_id]->nb_antennas_rx     = nb_antenna_rx;
      frame_parms[CC_id]->nb_antenna_ports_eNB = 1; //initial value overwritten by initial sync later

      LOG_I(PHY,"Set nb_rx_antenna %d , nb_tx_antenna %d \n",frame_parms[CC_id]->nb_antennas_rx, frame_parms[CC_id]->nb_antennas_tx);
  
    //init_ul_hopping(frame_parms[CC_id]);
    //phy_init_nr_top(frame_parms[CC_id]);
  }

  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
        //init prach for openair1 test
        // prach_fmt = get_prach_fmt(frame_parms->prach_config_common.prach_ConfigInfo.prach_ConfigIndex, frame_parms->frame_type);
        // N_ZC = (prach_fmt <4)?839:139;
  }

  NB_UE_INST=1;
  NB_INST=1;
  PHY_vars_UE_g = malloc(sizeof(PHY_VARS_NR_UE**));
  PHY_vars_UE_g[0] = malloc(sizeof(PHY_VARS_NR_UE*)*MAX_NUM_CCs);
  
  for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
    printf("PHY_vars_UE_g[0][%d] = %p\n",CC_id,UE[CC_id]);
    
    printf("frame_parms %d\n",frame_parms[CC_id]->ofdm_symbol_size);

    nr_init_frame_parms_ue(frame_parms[CC_id],numerology,NORMAL,frame_parms[CC_id]->N_RB_DL,(frame_parms[CC_id]->N_RB_DL-20)>>1,0);
    PHY_vars_UE_g[0][CC_id] = init_nr_ue_vars(frame_parms[CC_id], 0,abstraction_flag);
    UE[CC_id] = PHY_vars_UE_g[0][CC_id];
    
    if (phy_test==1)
      UE[CC_id]->mac_enabled = 0;
    else
      UE[CC_id]->mac_enabled = 1;
    
    if (UE[CC_id]->mac_enabled == 0) {  //set default UL parameters for testing mode
      for (i=0; i<NUMBER_OF_CONNECTED_eNB_MAX; i++) {
	//UE[CC_id]->pusch_config_dedicated[i] = malloc(sizeof(PUSCH_CONFIG_DEDICATED));
	
	//UE[CC_id]->scheduling_request_config[i] = malloc(sizeof(SCHEDULING_REQUEST_CONFIG));
	
	/*UE[CC_id]->pusch_config_dedicated[i].betaOffset_ACK_Index = beta_ACK;
	  UE[CC_id]->pusch_config_dedicated[i].betaOffset_RI_Index  = beta_RI;
	  UE[CC_id]->pusch_config_dedicated[i].betaOffset_CQI_Index = beta_CQI;
	  
	  UE[CC_id]->scheduling_request_config[i].sr_PUCCH_ResourceIndex = 0;
	  UE[CC_id]->scheduling_request_config[i].sr_ConfigIndex = 7+(0%3);
	  UE[CC_id]->scheduling_request_config[i].dsr_TransMax = sr_n4;*/
      }
    }
    
    UE[CC_id]->UE_scan = UE_scan;
    UE[CC_id]->UE_scan_carrier = UE_scan_carrier;
    UE[CC_id]->UE_fo_compensation = UE_fo_compensation;
    UE[CC_id]->mode    = mode;
    printf("UE[%d]->mode = %d\n",CC_id,mode);
    
    for (uint8_t i=0; i<RX_NB_TH_MAX; i++) {
      //UE[CC_id]->pdcch_vars[i][0]->agregationLevel = agregation_Level;
      //UE[CC_id]->pdcch_vars[i][0]->dciFormat     = dci_Format;
    
    /*compute_prach_seq(&UE[CC_id]->frame_parms.prach_config_common,
      UE[CC_id]->frame_parms.frame_type,
      UE[CC_id]->X_u);*/
    
    if (UE[CC_id]->mac_enabled == 1)
	UE[CC_id]->pdcch_vars[i][0]->crnti = 0x1234;
    else
	UE[CC_id]->pdcch_vars[i][0]->crnti = 0x1235;
    }
    
    UE[CC_id]->rx_total_gain_dB =  (int)rx_gain[CC_id][0] + rx_gain_off;
    UE[CC_id]->tx_power_max_dBm = tx_max_power[CC_id];
    
    if (frame_parms[CC_id]->frame_type==FDD) {
      UE[CC_id]->N_TA_offset = 0;
    } else {
      if (frame_parms[CC_id]->N_RB_DL == 100)
	UE[CC_id]->N_TA_offset = 624;
      else if (frame_parms[CC_id]->N_RB_DL == 50)
	UE[CC_id]->N_TA_offset = 624/2;
      else if (frame_parms[CC_id]->N_RB_DL == 25)
	UE[CC_id]->N_TA_offset = 624/4;
    }
    
  }
  
  //  printf("tx_max_power = %d -> amp %d\n",tx_max_power[0],get_tx_amp(tx_max_poHwer,tx_max_power));
  
  
  fill_modeled_runtime_table(runtime_phy_rx,runtime_phy_tx);
  cpuf=get_cpu_freq_GHz();
  
  
  //dump_frame_parms(frame_parms[0]);
  
  init_rfparam();

  // default threads.main = 2
  // if there are enough processors, all others on subsequent CPUs
  if (threads_offset>0) {
    if (threads_offset+8<=get_nprocs()) {
      if (threads.main<0) threads.main=threads_offset+1;
      if (threads.sync<0) threads.sync=threads_offset+2;
      if (threads.one<0) threads.one=threads_offset+3;
      if (threads.two<0) threads.two=threads_offset+4;
      if (threads.three<0) threads.three=threads_offset+5;
      if (threads.slot1_proc_one<0) threads.slot1_proc_one=threads_offset+6;
      if (threads.slot1_proc_two<0) threads.slot1_proc_two=threads_offset+7;
      if (threads.slot1_proc_three<0) threads.slot1_proc_three=threads_offset+8;
    }
    else {
      LOG_E(HW,"Not enough CPUs available (nprocs (=%d) >= threadmain (=%d) + 10)\n",get_nprocs(),threads_offset);
      exit_fun("Error setting thread affinity\n");
    }
  }

#ifndef DEADLINE_SCHEDULER

    /* Currently we set affinity for UHD to CPU 0 for eNB/UE and only if number of CPUS >2 */

    cpu_set_t cpuset;
    int s;
    char cpu_affinity[1024];
    CPU_ZERO(&cpuset);
#ifdef CPU_AFFINITY
    int j;
    if (get_nprocs()>=2) {
      for (j = 2; j < get_nprocs(); j++)
	CPU_SET(j, &cpuset);

        s = pthread_setaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
        if (s != 0) {
            perror( "pthread_setaffinity_np");
            exit_fun("Error setting processor affinity");
        }
        LOG_I(HW, "Setting the affinity of main function to all CPUs, for device library to use CPU 0 only!\n");
    }
#endif

  /* Check the actual affinity mask assigned to the thread */
  s = pthread_getaffinity_np(pthread_self(), sizeof(cpu_set_t), &cpuset);
  if (s != 0) {
    perror( "pthread_getaffinity_np");
    exit_fun("Error getting processor affinity ");
  }
  memset(cpu_affinity, 0 , sizeof(cpu_affinity));
  for (int j = 0; j < CPU_SETSIZE; j++) {
    if (CPU_ISSET(j, &cpuset)) {
      char temp[1024];
      sprintf(temp, " CPU_%d ", j);
      strcat(cpu_affinity, temp);
    }
  }
  LOG_I(HW, "CPU Affinity of main() function is... %s\n", cpu_affinity);
#endif


    //openair0_cfg[0].log_level = glog_level;

    /*int eMBMS_active=0;
    if (node_function[0] <= NGFI_RAU_IF4p5) { // don't initialize L2 for RRU
        LOG_I(PHY,"Intializing L2\n");
        mac_xface = malloc(sizeof(MAC_xface));
        l2_init(frame_parms[0],eMBMS_active,(uecap_xer_in==1)?uecap_xer:NULL,
                0,// cba_group_active
                0); // HO flag
        mac_xface->macphy_exit = &exit_fun;
    } else if (node_function[0] == NGFI_RRU_IF4p5) { // Initialize PRACH in this case

    }*/



#if defined(ENABLE_ITTI)

        // don't create if node doesn't connect to RRC/S1/GTP
        /*if (create_tasks(1) < 0) {
            printf("cannot create ITTI tasks\n");
            exit(-1); // need a softer mode
        }*/

    printf("ITTI tasks created\n");
#endif

    /*if (phy_test==0) {
        if (UE_flag==1) {
            printf("Filling UE band info\n");
            fill_ue_band_info();
            mac_xface->dl_phy_sync_success (0, 0, 0, 1);
        } else if (node_function[0]>NGFI_RRU_IF4p5)
            mac_xface->mrbch_phy_sync_failure (0, 0, 0);
    }*/

	// init UE_PF_PO and mutex lock
	pthread_mutex_init(&ue_pf_po_mutex, NULL);
	memset (&UE_PF_PO[0][0], 0, sizeof(UE_PF_PO_t)*NUMBER_OF_UE_MAX*MAX_NUM_CCs);


    mlockall(MCL_CURRENT | MCL_FUTURE);

    pthread_cond_init(&sync_cond,NULL);
    pthread_mutex_init(&sync_mutex, NULL);

    init_UE(1);

#ifdef XFORMS
    int UE_id;
    int fl_argc=1;

  if (do_forms==1) {
    // fl_initialize messes with argv and argc, so pretend to not pass any options
    fl_initialize (&fl_argc, argv, NULL, 0, 0);
    // restore the original command line args
    // argv = fl_get_cmdline_args( &argc );
    //form_stats = create_form_stats_form();
    //fl_show_form (form_stats->stats_form, FL_PLACE_HOTSPOT, FL_FULLBORDER, "stats");
    UE_id = 0;
    form_ue[UE_id] = create_lte_phy_scope_ue();
    sprintf (title, "NR DL SCOPE UE");
    fl_show_form (form_ue[UE_id]->lte_phy_scope_ue, FL_PLACE_HOTSPOT, FL_FULLBORDER, title);

            /*
            if (openair_daq_vars.use_ia_receiver) {
                   fl_set_button(form_ue[UE_id]->button_0,1);
                   fl_set_object_label(form_ue[UE_id]->button_0, "IA Receiver ON");
            } else {
                   fl_set_button(form_ue[UE_id]->button_0,0);
                   fl_set_object_label(form_ue[UE_id]->button_0, "IA Receiver OFF");
            }*/
            fl_set_button(form_ue[UE_id]->button_0,0);
            fl_set_object_label(form_ue[UE_id]->button_0, "IA Receiver OFF");
        

        ret = pthread_create(&forms_thread, NULL, scope_thread, NULL);

        if (ret == 0)
            pthread_setname_np( forms_thread, "xforms" );

        printf("Scope thread created, ret=%d\n",ret);
    }

#endif

    rt_sleep_ns(10*100000000ULL);

    number_of_cards = 1;
    
    for(CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
      PHY_vars_UE_g[0][CC_id]->rf_map.card=0;
      PHY_vars_UE_g[0][CC_id]->rf_map.chain=CC_id+chain_offset;
    }


    for (CC_id=0; CC_id<MAX_NUM_CCs; CC_id++) {
      
#if defined(OAI_USRP) || defined(OAI_ADRV9371_ZC706)
      UE[CC_id]->hw_timing_advance = timing_advance;
#else
      UE[CC_id]->hw_timing_advance = 160;
#endif
    }

    sleep(3);

    printf("Sending sync to all threads\n");

    pthread_mutex_lock(&sync_mutex);
    sync_var=0;
    pthread_cond_broadcast(&sync_cond);
    pthread_mutex_unlock(&sync_mutex);

    // wait for end of program
    printf("TYPE <CTRL-C> TO TERMINATE\n");
    //getchar();

    /*
#if defined(ENABLE_ITTI)
    printf("Entering ITTI signals handler\n");
    itti_wait_tasks_end();
    oai_exit=1;
#else
    */

    while (oai_exit==0)
        rt_sleep_ns(100000000ULL);

    //#endif

  // stop threads
#ifdef XFORMS
  printf("waiting for XFORMS thread\n");

  if (do_forms==1) {
    pthread_join(forms_thread,&status);
    //fl_hide_form(form_stats->stats_form);
    //fl_free_form(form_stats->stats_form);
    fl_hide_form(form_ue[0]->lte_phy_scope_ue);
    fl_free_form(form_ue[0]->lte_phy_scope_ue);
  }

#endif

  printf("stopping MODEM threads\n");

  pthread_cond_destroy(&sync_cond);
  pthread_mutex_destroy(&sync_mutex);
  
  //if (ouput_vcd)
  //VCD_SIGNAL_DUMPER_CLOSE();

  if (opt_enabled == 1)
    terminate_opt();
  
  logClean();
  
  return 0;
}
