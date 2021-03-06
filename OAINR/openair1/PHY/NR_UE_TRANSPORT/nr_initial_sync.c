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

/*! \file PHY/LTE_TRANSPORT/initial_sync.c
* \brief Routines for initial UE synchronization procedure (PSS,SSS,PBCH and frame format detection)
* \author R. Knopp, F. Kaltenberger
* \date 2011
* \version 0.1
* \company Eurecom
* \email: knopp@eurecom.fr,kaltenberger@eurecom.fr
* \note
* \warning
*/
#include "PHY/types.h"
#include "PHY/defs_nr_UE.h"
#include "PHY/phy_extern_nr_ue.h"
#include "PHY/INIT/phy_init.h"
#include "PHY/MODULATION/modulation_UE.h"
#include "nr_transport_proto_ue.h"
//#include "SCHED/defs.h"
//#include "SCHED/extern.h"

#include "common_lib.h"
#include <math.h>

#include "PHY/NR_REFSIG/pss_nr.h"
#include "PHY/NR_REFSIG/sss_nr.h"
#include "PHY/NR_REFSIG/refsig_defs_ue.h"
#include "targets/RT/USER/vThread_HWCommon.h"

//static  nfapi_nr_config_request_t config_t;
//static  nfapi_nr_config_request_t* config =&config_t;
int cnt=0;

#define DEBUG_INITIAL_SYNCH

int nr_pbch_detection(PHY_VARS_NR_UE *ue, runmode_t mode)
{
  NR_DL_FRAME_PARMS *frame_parms=&ue->frame_parms;
  int ret =-1;


#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE%d] Initial sync: starting PBCH detection (rx_offset %d)\n",ue->Mod_id,
        ue->rx_offset);
#endif

  // save the nb_prefix_samples0 since we are not synchronized to subframes yet and the SSB has all symbols with nb_prefix_samples
  int nb_prefix_samples0 = frame_parms->nb_prefix_samples0;
  frame_parms->nb_prefix_samples0 = frame_parms->nb_prefix_samples;


  //symbol 1
  nr_slot_fep(ue,
	      1,
	      0,
	      ue->ssb_offset,
	      0,
	      1,
	      NR_PBCH_EST);
  
  //symbol 2
  nr_slot_fep(ue,
	      2,
	      0,
	      ue->ssb_offset,
	      0,
	      1,
	      NR_PBCH_EST);

  //symbol 3
  nr_slot_fep(ue,
	      3,
	      0,
	      ue->ssb_offset,
	      0,
	      1,
	      NR_PBCH_EST);

  //put back nb_prefix_samples0
  frame_parms->nb_prefix_samples0 = nb_prefix_samples0;
  
  
  ret = nr_rx_pbch(ue,
		   &ue->proc.proc_rxtx[0],
		   ue->pbch_vars[0],
		   frame_parms,
		   0,
		   SISO,
		   ue->high_speed_flag);
  
  
  if (ret==0) {
    
    frame_parms->nb_antenna_ports_eNB = 1; //pbch_tx_ant;
    
    // set initial transmission mode to 1 or 2 depending on number of detected TX antennas
    //frame_parms->mode1_flag = (pbch_tx_ant==1);
    // openair_daq_vars.dlsch_transmission_mode = (pbch_tx_ant>1) ? 2 : 1;


    // flip byte endian on 24-bits for MIB
    //    dummy = ue->pbch_vars[0]->decoded_output[0];
    //    ue->pbch_vars[0]->decoded_output[0] = ue->pbch_vars[0]->decoded_output[2];
    //    ue->pbch_vars[0]->decoded_output[2] = dummy;

    for(int i=0; i<RX_NB_TH;i++)
    {

        ue->proc.proc_rxtx[i].frame_tx = ue->proc.proc_rxtx[0].frame_rx;
    }
#ifdef DEBUG_INITIAL_SYNCH
    LOG_E(PHY,"Initial sync: pbch decoded sucessfully\n");
#endif
    return(0);
  } else {
    LOG_E(PHY,"Initial sync: pbch decoded failed!\n");

	return(-1);
  }

}

char duplex_string[2][4] = {"FDD","TDD"};
char prefix_string[2][9] = {"NORMAL","EXTENDED"};

int nr_initial_sync(PHY_VARS_NR_UE *ue, runmode_t mode)
{

  int32_t sync_pos, sync_pos_slot; // k_ssb, N_ssb_crb, sync_pos2,
  int32_t metric_tdd_ncp=0;
  uint8_t phase_tdd_ncp;
  double im, re;
  unsigned long long syncTimer;

  NR_DL_FRAME_PARMS *fp = &ue->frame_parms;
  int ret=-1;
  int rx_power=0; //aarx,
  //nfapi_nr_config_request_t* config;

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
  LOG_D(PHY,"nr_initial sync ue RB_DL %d\n", fp->N_RB_DL);

  /*
  write_output("rxdata0.m","rxd0",ue->common_vars.rxdata[0],10*fp->samples_per_subframe,1,1);
  exit(-1);
  */

  /*   Initial synchronisation
   *
  *                                 1 radio frame = 10 ms
  *     <--------------------------------------------------------------------------->
  *     -----------------------------------------------------------------------------
  *     |                                 Received UE data buffer                    |
  *     ----------------------------------------------------------------------------
  *                     --------------------------
  *     <-------------->| pss | pbch | sss | pbch |
  *                     --------------------------
  *          sync_pos            SS/PBCH block
  */

  cnt++;
  if (1){ // (cnt>100)
    cnt =0;


  syncTimer = rdtsc_oai();

  /* process pss search on received buffer */
  sync_pos = pss_synchro_nr(ue, NO_RATE_CHANGE);

  sync_pos_slot = (fp->samples_per_subframe/fp->slots_per_subframe) - 12*(fp->ofdm_symbol_size + fp->nb_prefix_samples); //12 should be modified

  if (sync_pos >= fp->nb_prefix_samples){
    ue->ssb_offset = sync_pos - fp->nb_prefix_samples;}
  else{
    ue->ssb_offset = sync_pos + (fp->samples_per_subframe * 10) - fp->nb_prefix_samples;}
    
  ue->rx_offset = ue->ssb_offset - sync_pos_slot;

  if (ue->rx_offset < 0)
  {
	LOG_E(PHY, "[WARNING] rx_offset is negative!!");
  }
  //write_output("rxdata1.m","rxd1",ue->common_vars.rxdata[0],10*fp->samples_per_subframe,1,1);

  LOG_I(PHY, "[PSS time] : %d us\n", (int)((rdtsc_oai() - syncTimer)/cpuf/1000.0));
  syncTimer = rdtsc_oai();

#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE%d] Initial sync : Estimated PSS position %d, Nid2 %d\n", ue->Mod_id, sync_pos,ue->common_vars.eNb_id);
  LOG_I(PHY,"sync_pos %d ssb_offset %d sync_pos_slot %d \n",sync_pos,ue->ssb_offset,sync_pos_slot);
#endif

  /* check that SSS/PBCH block is continuous inside the received buffer */
  if (ue->rx_offset < 0 ||
  	  ue->ssb_offset < (NR_NUMBER_OF_SUBFRAMES_PER_FRAME*fp->samples_per_subframe - (4 * (fp->ofdm_symbol_size + fp->nb_prefix_samples)))) // 4 should be changed
  {


  	// digital compensation of FFO for SSB symbols
	  if (ue->UE_fo_compensation){  
		double s_time = 1/(1.0e3*fp->samples_per_subframe);  // sampling time
		double off_angle = -2*M_PI*s_time*(ue->common_vars.freq_offset);  // offset rotation angle compensation per sample

		int start = ue->ssb_offset;  // start for offset correction is at ssb_offset (pss time position)
	  	int end = start + 4*(fp->ofdm_symbol_size + fp->nb_prefix_samples);  // loop over samples in 4 symbols (ssb size), including prefix  

		for(int n=start; n<end; n++){  	
		  for (int ar=0; ar<fp->nb_antennas_rx; ar++) {
			re = ((double)(((short *)ue->common_vars.rxdata[ar]))[2*n]);
			im = ((double)(((short *)ue->common_vars.rxdata[ar]))[2*n+1]);
			((short *)ue->common_vars.rxdata[ar])[2*(n)] = (short)(round(re*cos(n*off_angle) - im*sin(n*off_angle))); 
			((short *)ue->common_vars.rxdata[ar])[2*(n)+1] = (short)(round(re*sin(n*off_angle) + im*cos(n*off_angle)));
		  }
		}
	  }

	LOG_I(PHY, "[AFC] : %d us\n", (int)((rdtsc_oai() - syncTimer)/cpuf/1000.0));
  	syncTimer = rdtsc_oai();

  
#ifdef DEBUG_INITIAL_SYNCH
    LOG_I(PHY,"Calling sss detection (normal CP)\n");
#endif

    rx_sss_nr(ue,&metric_tdd_ncp,&phase_tdd_ncp);

    //FK: why do we need to do this again here?
    //nr_init_frame_parms_ue(fp,NR_MU_1,NORMAL,n_ssb_crb,0);

	LOG_I(PHY, "[SSS time] : %d us\n", (int)((rdtsc_oai() - syncTimer)/cpuf/1000.0));
  	syncTimer = rdtsc_oai();

    nr_gold_pbch(ue);
    ret = nr_pbch_detection(ue,mode);

	LOG_I(PHY, "[PBCH time] : %d us\n", (int)((rdtsc_oai() - syncTimer)/cpuf/1000.0));
    
    nr_gold_pdcch(ue,0, 2);
    /*
    int nb_prefix_samples0 = fp->nb_prefix_samples0;
    fp->nb_prefix_samples0 = fp->nb_prefix_samples;
	  
    nr_slot_fep(ue,0, 0, ue->ssb_offset, 0, 1, NR_PDCCH_EST);
    nr_slot_fep(ue,1, 0, ue->ssb_offset, 0, 1, NR_PDCCH_EST);
    fp->nb_prefix_samples0 = nb_prefix_samples0;	

    LOG_I(PHY,"[UE  %d] AUTOTEST Cell Sync : frame = %d, rx_offset %d, freq_offset %d \n",
              ue->Mod_id,
              ue->proc.proc_rxtx[0].frame_rx,
              ue->rx_offset,
              ue->common_vars.freq_offset );
    */

#ifdef DEBUG_INITIAL_SYNCH
    LOG_I(PHY,"TDD Normal prefix: CellId %d metric %d, phase %d, pbch %d\n",
          fp->Nid_cell,metric_tdd_ncp,phase_tdd_ncp,ret);
#endif
  }
  else {
    LOG_I(PHY,"PSS is detected far from the start point, restarting sync. (%i, %i)\n", sync_pos, sync_pos_slot);
	ret=-3;
  	}
  }
  else {
	  ret = -1;
  }

  //Consider this is a false detection if the offset is > 1000 Hz 
  //   Not to be used now that offest estimation is in place
  if( (abs(ue->common_vars.freq_offset) > 3000) && (ret == 0) ) //should be modified
  {
	  ret=-2;
#if DISABLE_LOG_X
	  printf("Ignore MIB with high freq offset [%d Hz] estimation \n",ue->common_vars.freq_offset);
#else
	  LOG_E(HW, "Ignore MIB with high freq offset [%d Hz] estimation \n",ue->common_vars.freq_offset);
#endif
  }

  if (ret==0) {  // PBCH found so indicate sync to higher layers and configure frame parameters

    //#ifdef DEBUG_INITIAL_SYNCH
#if DISABLE_LOG_X
    printf("[UE%d] In synch, rx_offset %d samples\n",ue->Mod_id, ue->rx_offset);
#else
    LOG_I(PHY, "[UE%d] In synch, rx_offset %d samples\n",ue->Mod_id, ue->rx_offset);
#endif
    //#endif

    if (ue->UE_scan_carrier == 0) {

    #if UE_AUTOTEST_TRACE
      LOG_I(PHY,"[UE  %d] AUTOTEST Cell Sync : frame = %d, rx_offset %d, freq_offset %d \n",
              ue->Mod_id,
              ue->proc.proc_rxtx[0].frame_rx,
              ue->rx_offset,
              ue->common_vars.freq_offset );
    #endif

// send sync status to higher layers later when timing offset converge to target timing

      ue->pbch_vars[0]->pdu_errors_conseq=0;

    }

#if DISABLE_LOG_X
    printf("[UE %d] Frame %d RRC Measurements => rssi %3.1f dBm (dig %3.1f dB, gain %d), N0 %d dBm,  rsrp %3.1f dBm/RE, rsrq %3.1f dB\n",ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  10*log10(ue->measurements.rssi)-ue->rx_total_gain_dB,
	  10*log10(ue->measurements.rssi),
	  ue->rx_total_gain_dB,
	  ue->measurements.n0_power_tot_dBm,
	  10*log10(ue->measurements.rsrp[0])-ue->rx_total_gain_dB,
	  (10*log10(ue->measurements.rsrq[0])));


    printf("[UE %d] Frame %d MIB Information => %s, %s, NidCell %d, N_RB_DL %d, PHICH DURATION %d, PHICH RESOURCE %s, TX_ANT %d\n",
	  ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  duplex_string[fp->frame_type],
	  prefix_string[fp->Ncp],
	  fp->Nid_cell,
	  fp->N_RB_DL,
	  fp->phich_config_common.phich_duration,
	  phich_string[fp->phich_config_common.phich_resource],
	  fp->nb_antenna_ports_eNB);
#else
    LOG_I(PHY, "[UE %d] Frame %d RRC Measurements => rssi %3.1f dBm (dig %3.1f dB, gain %d), N0 %d dBm,  rsrp %3.1f dBm/RE, rsrq %3.1f dB\n",ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  10*log10(ue->measurements.rssi)-ue->rx_total_gain_dB,
	  10*log10(ue->measurements.rssi),
	  ue->rx_total_gain_dB,
	  ue->measurements.n0_power_tot_dBm,
	  10*log10(ue->measurements.rsrp[0])-ue->rx_total_gain_dB,
	  (10*log10(ue->measurements.rsrq[0])));

#endif

#  if DISABLE_LOG_X
    printf("[UE %d] Frame %d Measured Carrier Frequency offset %d Hz\n",
	  ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  ue->common_vars.freq_offset);
#  else
    LOG_I(PHY, "[UE %d] Frame %d Measured Carrier Frequency offset %d Hz\n",
	  ue->Mod_id,
	  ue->proc.proc_rxtx[0].frame_rx,
	  ue->common_vars.freq_offset);
#  endif
  } else {
#ifdef DEBUG_INITIAL_SYNC
    LOG_I(PHY,"[UE%d] Initial sync : PBCH not ok\n",ue->Mod_id);
    LOG_I(PHY,"[UE%d] Initial sync : Estimated PSS position %d, Nid2 %d\n",ue->Mod_id,sync_pos,ue->common_vars.eNb_id);
    LOG_I(PHY,"[UE%d] Initial sync : Estimated Nid_cell %d, Frame_type %d\n",ue->Mod_id,
          frame_parms->Nid_cell,frame_parms->frame_type);
#endif

    ue->UE_mode[0] = NOT_SYNCHED;
    ue->pbch_vars[0]->pdu_errors_last=ue->pbch_vars[0]->pdu_errors;
    ue->pbch_vars[0]->pdu_errors++;
    ue->pbch_vars[0]->pdu_errors_conseq++;

  }

  // gain control
  if (ret!=0) { //we are not synched, so we cannot use rssi measurement (which is based on channel estimates)
    rx_power = 0;

    // do a measurement on the best guess of the PSS
    //for (aarx=0; aarx<frame_parms->nb_antennas_rx; aarx++)
    //  rx_power += signal_energy(&ue->common_vars.rxdata[aarx][sync_pos2],
	//			frame_parms->ofdm_symbol_size+frame_parms->nb_prefix_samples);

    /*
    // do a measurement on the full frame
    for (aarx=0; aarx<frame_parms->nb_antennas_rx; aarx++)
      rx_power += signal_energy(&ue->common_vars.rxdata[aarx][0],
				frame_parms->samples_per_subframe*10);
    */

    // we might add a low-pass filter here later
    ue->measurements.rx_power_avg[0] = rx_power/fp->nb_antennas_rx;

    ue->measurements.rx_power_avg_dB[0] = dB_fixed(ue->measurements.rx_power_avg[0]);

#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE%d] Initial sync : Estimated power: %d dB\n",ue->Mod_id,ue->measurements.rx_power_avg_dB[0] );
#endif

#ifndef OAI_USRP
#ifndef OAI_BLADERF
#ifndef OAI_LMSSDR
#ifndef OAI_ADRV9371_ZC706
  //phy_adjust_gain(ue,ue->measurements.rx_power_avg_dB[0],0);
#endif
#endif
#endif
#endif

  }
  else {

#ifndef OAI_USRP
#ifndef OAI_BLADERF
#ifndef OAI_LMSSDR
#ifndef OAI_ADRV9371_ZC706
  //phy_adjust_gain(ue,dB_fixed(ue->measurements.rssi),0);
#endif
#endif
#endif
#endif

  }

  //  exit_fun("debug exit");
  return ret;
}

