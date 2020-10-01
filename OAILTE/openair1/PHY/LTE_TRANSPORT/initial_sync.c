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
#include "PHY/defs.h"
#include "PHY/extern.h"
#include "SCHED/defs.h"
#include "SCHED/extern.h"
#include "defs.h"
#include "extern.h"
#include <math.h>

#include "common_lib.h"
#include "ltesync_Wvalues.h"

#define PHY_SYNC_SS_LENGTH			72
#define PHY_SYNC_MAXNB_OFDMSYMBOL	2048

extern openair0_config_t openair0_cfg[];






int pbch_detection(PHY_VARS_UE *ue, runmode_t mode)
{

  uint8_t l,pbch_decoded,frame_mod4,pbch_tx_ant,dummy;
  LTE_DL_FRAME_PARMS *frame_parms=&ue->frame_parms;
  char phich_resource[6];

#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE%d] Initial sync: starting PBCH detection (rx_offset %d)\n",ue->Mod_id,
        ue->rx_offset);
#endif

  for (l=0; l<frame_parms->symbols_per_tti/2; l++) {

    slot_fep(ue,
	     l,
	     0,
	     ue->rx_offset,
	     0,
	     1);
  }
  for (l=0; l<frame_parms->symbols_per_tti/2; l++) {

    slot_fep(ue,
	     l,
	     1,
	     ue->rx_offset,
	     0,
	     1);
  }
  slot_fep(ue,
	   0,
	   2,
	   ue->rx_offset,
	   0,
	   1);

  lte_ue_measurements(ue,
		      ue->rx_offset,
		      0,
                      0,
		      0,
                      0);


  if (ue->frame_parms.frame_type == TDD) {
    ue_rrc_measurements(ue,
			2,
			0);
  }
  else {
    ue_rrc_measurements(ue,
			0,
			0);
  }
#ifdef DEBUG_INITIAL_SYNCH
  LOG_I(PHY,"[UE %d] RX RSSI %d dBm, digital (%d, %d) dB, linear (%d, %d), avg rx power %d dB (%d lin), RX gain %d dB\n",
        ue->Mod_id,
        ue->measurements.rx_rssi_dBm[0] - ((ue->frame_parms.nb_antennas_rx==2) ? 3 : 0),
        ue->measurements.rx_power_dB[0][0],
        ue->measurements.rx_power_dB[0][1],
        ue->measurements.rx_power[0][0],
        ue->measurements.rx_power[0][1],
        ue->measurements.rx_power_avg_dB[0],
        ue->measurements.rx_power_avg[0],
        ue->rx_total_gain_dB);

  LOG_I(PHY,"[UE %d] N0 %d dBm digital (%d, %d) dB, linear (%d, %d), avg noise power %d dB (%d lin)\n",
        ue->Mod_id,
        ue->measurements.n0_power_tot_dBm,
        ue->measurements.n0_power_dB[0],
        ue->measurements.n0_power_dB[1],
        ue->measurements.n0_power[0],
        ue->measurements.n0_power[1],
        ue->measurements.n0_power_avg_dB,
        ue->measurements.n0_power_avg);
#endif

  pbch_decoded = 0;

  for (frame_mod4=0; frame_mod4<4; frame_mod4++) {
    pbch_tx_ant = rx_pbch(&ue->common_vars,
                          ue->pbch_vars[0],
                          frame_parms,
                          0,
                          SISO,
                          ue->high_speed_flag,
                          frame_mod4);

    if ((pbch_tx_ant>0) && (pbch_tx_ant<=2)) {
      pbch_decoded = 1;
      break;
    }

    pbch_tx_ant = rx_pbch(&ue->common_vars,
                          ue->pbch_vars[0],
                          frame_parms,
                          0,
                          ALAMOUTI,
                          ue->high_speed_flag,
                          frame_mod4);

    if ((pbch_tx_ant>0) && (pbch_tx_ant<=2)) {
      pbch_decoded = 1;
      break;
    }
  }


  if (pbch_decoded) {

    frame_parms->nb_antenna_ports_eNB = pbch_tx_ant;

    // set initial transmission mode to 1 or 2 depending on number of detected TX antennas
    frame_parms->mode1_flag = (pbch_tx_ant==1);
    // openair_daq_vars.dlsch_transmission_mode = (pbch_tx_ant>1) ? 2 : 1;


    // flip byte endian on 24-bits for MIB
    //    dummy = ue->pbch_vars[0]->decoded_output[0];
    //    ue->pbch_vars[0]->decoded_output[0] = ue->pbch_vars[0]->decoded_output[2];
    //    ue->pbch_vars[0]->decoded_output[2] = dummy;

    // now check for Bandwidth of Cell
    dummy = (ue->pbch_vars[0]->decoded_output[2]>>5)&7;

    switch (dummy) {

    case 0 :
      frame_parms->N_RB_DL = 6;
      break;

    case 1 :
      frame_parms->N_RB_DL = 15;
      break;

    case 2 :
      frame_parms->N_RB_DL = 25;
      break;

    case 3 :
      frame_parms->N_RB_DL = 50;
      break;

    case 4 :
      frame_parms->N_RB_DL = 75;
      break;

    case 5:
      frame_parms->N_RB_DL = 100;
      break;

    default:
      LOG_E(PHY,"[UE%d] Initial sync: PBCH decoding: Unknown N_RB_DL\n",ue->Mod_id);
      return -1;
      break;
    }


    // now check for PHICH parameters
    frame_parms->phich_config_common.phich_duration = (PHICH_DURATION_t)((ue->pbch_vars[0]->decoded_output[2]>>4)&1);
    dummy = (ue->pbch_vars[0]->decoded_output[2]>>2)&3;

    switch (dummy) {
    case 0:
      frame_parms->phich_config_common.phich_resource = oneSixth;
      sprintf(phich_resource,"1/6");
      break;

    case 1:
      frame_parms->phich_config_common.phich_resource = half;
      sprintf(phich_resource,"1/2");
      break;

    case 2:
      frame_parms->phich_config_common.phich_resource = one;
      sprintf(phich_resource,"1");
      break;

    case 3:
      frame_parms->phich_config_common.phich_resource = two;
      sprintf(phich_resource,"2");
      break;

    default:
      LOG_E(PHY,"[UE%d] Initial sync: Unknown PHICH_DURATION\n",ue->Mod_id);
      return -1;
      break;
    }

    for(int i=0; i<RX_NB_TH;i++)
    {
        ue->proc.proc_rxtx[i].frame_rx =   (((ue->pbch_vars[0]->decoded_output[2]&3)<<6) + (ue->pbch_vars[0]->decoded_output[1]>>2))<<2;
        ue->proc.proc_rxtx[i].frame_rx =   (((ue->pbch_vars[0]->decoded_output[2]&3)<<6) + (ue->pbch_vars[0]->decoded_output[1]>>2))<<2;

#ifndef USER_MODE
        // one frame delay
        ue->proc.proc_rxtx[i].frame_rx ++;
#endif
        ue->proc.proc_rxtx[i].frame_tx = ue->proc.proc_rxtx[0].frame_rx;
    }
#ifdef DEBUG_INITIAL_SYNCH
    LOG_I(PHY,"[UE%d] Initial sync: pbch decoded sucessfully mode1_flag %d, tx_ant %d, frame %d, N_RB_DL %d, phich_duration %d, phich_resource %s!\n",
          ue->Mod_id,
          frame_parms->mode1_flag,
          pbch_tx_ant,
          ue->proc.proc_rxtx[0].frame_rx,
          frame_parms->N_RB_DL,
          frame_parms->phich_config_common.phich_duration,
          phich_resource);  //frame_parms->phich_config_common.phich_resource);
#endif
    return(0);
  } else {
    return(-1);
  }

}

 

int tryPbchDecoding(PHY_VARS_UE *ue, runmode_t mode)
{
	LTE_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;
	
	frame_parms->nushift  = frame_parms->Nid_cell%6;
    lte_gold(frame_parms,ue->lte_gold_table[0],frame_parms->Nid_cell);

    return pbch_detection(ue,mode);
}


void calc_syncRssi(PHY_VARS_UE *ue, int32_t sync_pos)
{
	int aarx, rx_power = 0;
	LTE_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;

	// do a measurement on the best guess of the PSS
    for (aarx=0; aarx<frame_parms->nb_antennas_rx; aarx++)
    {
    	rx_power += signal_energy(&ue->common_vars.rxdata[aarx][sync_pos],
				frame_parms->ofdm_symbol_size+frame_parms->nb_prefix_samples);
    }
    ue->measurements.rx_power_avg[0] = rx_power/frame_parms->nb_antennas_rx;
    ue->measurements.rx_power_avg_dB[0] = dB_fixed(ue->measurements.rx_power_avg[0]);

  	LOG_D(PHY,"[UE%d] Initial sync : Estimated power: %d dB\n",ue->Mod_id,ue->measurements.rx_power_avg_dB[0] );
}


void postProcess_syncFailed(PHY_VARS_UE *ue, int32_t sync_pos)
{
	LTE_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;

	//log printing
	LOG_E(PHY,"[UE%d] Initial sync : PBCH not ok\n",ue->Mod_id);
    LOG_D(PHY,"[UE%d] Initial sync : Estimated PSS position %d, Nid2 %d\n",ue->Mod_id,sync_pos,ue->common_vars.eNb_id);
    LOG_D(PHY,"[UE%d] Initial sync : Estimated Nid_cell %d, Frame_type %d\n",ue->Mod_id,
          frame_parms->Nid_cell,frame_parms->frame_type);


	//UE state update for sync. failure
    ue->UE_mode[0] = NOT_SYNCHED;
    ue->pbch_vars[0]->pdu_errors_last=ue->pbch_vars[0]->pdu_errors;
    ue->pbch_vars[0]->pdu_errors++;
    ue->pbch_vars[0]->pdu_errors_conseq++;
}

char phich_string[13][4] = {"","1/6","","1/2","","","one","","","","","","two"};
char duplex_string[2][4] = {"FDD","TDD"};
char prefix_string[2][9] = {"NORMAL","EXTENDED"};

void postProcess_syncSuccess(PHY_VARS_UE *ue)
{

	LTE_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;
	
	LOG_I(PHY, "[UE%d] In synch, rx_offset %d samples\n",ue->Mod_id, ue->rx_offset);

	//frame structure definition
	if (ue->UE_scan_carrier == 0)
	{
  		generate_pcfich_reg_mapping(frame_parms);
  		generate_phich_reg_mapping(frame_parms);
    	ue->pbch_vars[0]->pdu_errors_conseq=0;
	}

 

	//event log printing
	LOG_I(PHY, "[UE %d] Frame %d RRC Measurements => rssi %3.1f dBm (dig %3.1f dB, gain %d), N0 %d dBm,  rsrp %3.1f dBm/RE, rsrq %3.1f dB\n",ue->Mod_id,
				ue->proc.proc_rxtx[0].frame_rx,
				10*log10(ue->measurements.rssi)-ue->rx_total_gain_dB,
				10*log10(ue->measurements.rssi),
				ue->rx_total_gain_dB,
				ue->measurements.n0_power_tot_dBm,
				10*log10(ue->measurements.rsrp[0])-ue->rx_total_gain_dB,
				(10*log10(ue->measurements.rsrq[0])));

	LOG_I(PHY, "[UE %d] Frame %d MIB Information => %s, %s, NidCell %d, N_RB_DL %d, PHICH DURATION %d, PHICH RESOURCE %s, TX_ANT %d\n",
				ue->Mod_id,
				ue->proc.proc_rxtx[0].frame_rx,
				duplex_string[ue->frame_parms.frame_type],
				prefix_string[ue->frame_parms.Ncp],
				ue->frame_parms.Nid_cell,
				ue->frame_parms.N_RB_DL,
				ue->frame_parms.phich_config_common.phich_duration,
				phich_string[ue->frame_parms.phich_config_common.phich_resource],
				ue->frame_parms.nb_antenna_ports_eNB);

	LOG_I(PHY, "[UE %d] Frame %d Measured Carrier Frequency %.0f Hz (offset %d Hz)\n",
				ue->Mod_id,
				ue->proc.proc_rxtx[0].frame_rx,
				openair0_cfg[0].rx_freq[0]-ue->common_vars.freq_offset,
				ue->common_vars.freq_offset);
}

 


static inline int abs32(int x)
{
  return (((int)((short*)&x)[0])*((int)((short*)&x)[0]) + ((int)((short*)&x)[1])*((int)((short*)&x)[1]));
}


int extract_pssSss(int32_t extResult[72], int32_t* fftResult, int fftSize)
{

	uint16_t rb,nb_rb=6;
	uint8_t i;
	int32_t *ss_rxF,*ss_rxF_ext;

	int ss_offset = fftSize-3*12;

    ss_rxF  =  &fftResult[ss_offset];
    ss_rxF_ext    = &extResult[0];

    for (rb=0; rb<nb_rb; rb++)
	{
		// skip DC carrier
		if (rb==3)
		{
			ss_rxF = &fftResult[1];
		}

		for (i=0; i<12; i++)
		{
			ss_rxF_ext[i] = ss_rxF[i];
		}

		ss_rxF+=12;
		ss_rxF_ext+=12;
    }

  return(0);
}

extern int pss_ch_est(PHY_VARS_UE *ue,
               int32_t pss_ext[4][72],
               int32_t sss_ext[4][72]);



#define SHIFT 17

static int16_t phase_re[7] = {16383, 25101, 30791, 32767, 30791, 25101, 16383};
static int16_t phase_im[7] = {-28378, -21063, -11208, 0, 11207, 21062, 28377};

int PSS_detection(int **rxdata, int length, uint16_t fftSize, int *eNB_id)
{
	// perform a time domain correlation using the oversampled sync sequence

	unsigned int n, s, peak_pos, peak_val, sync_source;
	int sync_out[3] = {0,0,0}, sync_out2[3] = {0,0,0};
	int tmp;
	//int length =   LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*frame_parms->samples_per_tti>>1;
	int half_length = length>>1;
	short cor_result[2];

	peak_val = 0;
	peak_pos = 0;
	sync_source = 0;

	for (n=0; n<half_length; n+=4)
	{
		for (s=0; s<3; s++)
		{
			sync_out[s]=0;
			sync_out2[s]=0;
		}

		if (n<(half_length-fftSize))
		{
			sync_out[0]  = dot_product((short*)primary_synch0_time, (short*) &(rxdata[0][n]), fftSize, SHIFT);
			sync_out2[0] = dot_product((short*)primary_synch0_time, (short*) &(rxdata[0][n+half_length]), fftSize, SHIFT);			

			sync_out[1] = dot_product((short*)primary_synch1_time, (short*) &(rxdata[0][n]), fftSize, SHIFT);
			sync_out2[1] = dot_product((short*)primary_synch1_time, (short*) &(rxdata[0][n+half_length]), fftSize, SHIFT);

		    sync_out[2] = dot_product((short*)primary_synch2_time, (short*) &(rxdata[0][n]), fftSize, SHIFT);
		    sync_out2[2] = dot_product((short*)primary_synch2_time, (short*) &(rxdata[0][n+half_length]), fftSize, SHIFT);

			for (s=0; s<3; s++)
			{
				tmp = (abs32(sync_out[s])>>1) + (abs32(sync_out2[s])>>1);

			  	if (tmp>peak_val)
				{
			    	peak_val = tmp;
				    peak_pos = n;
				    sync_source = s;
			  	}
			}
			
		}
	}

	*eNB_id = sync_source;

	LOG_I(PHY,"[UE] lte_sync_time: Sync source = %d, Peak found at pos %d, val = %d (%d dB)\n",sync_source,peak_pos,peak_val,dB_fixed(peak_val)/2);

	return(peak_pos);

}



void dumpCompData(int16_t* ptr, int n, char* str) 
{
	LOG_E(PHY, "dumping %s :\n", str);
	for(int i=0;i<n;i++)
	{
		printf("%i + (%i)*i ", ptr[2*i], ptr[2*i+1]);
	}
	printf("\n");
}


void dumpSimdData(__m128i *ptr, int option)
{
	 int16_t* ptr16 = (int16_t*)ptr;
	 int32_t* ptr32 = (int32_t*)ptr;
 
	 if (option == 0) //16bit
		{
			for(int j=0; j<8; j++)
				{ 
					printf("%i ",  ptr16[j]);
				}
		}
		
	 else if (option == 1) //32bit
		{
			 for(int j=0; j<4; j++)
				 { 
					printf("%i ",  ptr32[j]);
				 }
		}

	printf("\n");
}
 




#define LTESYNC_PSS_SOLUTION
//#define LTESYNC_FFT_SOLUTION
//#define LTESYNC_SSS_SOLUTION

//#define LTESYNC_DEBUG_PRINTON //enable this code if you want detailed debug prints


void printComplexNum(int16_t* ptr, char* inname)
{
	#ifdef LTESYNC_DEBUG_PRINTON
	LOG_E(PHY, "First 4 %s values : ", inname); 
	
	for (int i=0; i<4; i++)
	{
		printf("(%i, %i)", ptr[i*2], ptr[i*2+1]);
	}

	printf("\n");
	#endif
}



void print128_epi16(int16_t* ptr, char* inname)
{
	#ifdef LTESYNC_DEBUG_PRINTON
	LOG_E(PHY, "%s (16b): ", inname); 
	
	for (int i=0; i<8; i++)
	{
		printf("%i ", ptr[i]);
	}

	printf("\n");
	#endif
}

void print128_epi32(int32_t* ptr, char* inname)
{
	#ifdef LTESYNC_DEBUG_PRINTON
	LOG_E(PHY, "%s (32b): ", inname); 
	
	for (int i=0; i<4; i++)
	{
		printf("%i ", ptr[i]);
	}

	printf("\n");
	#endif
}


int16_t* FFT4(int16_t *x, int16_t *y);
int16_t* FFT16(int16_t *x, int16_t *y);
int16_t* FFT64(int16_t *x, int16_t *y);
int16_t* FFT256(int16_t *x, int16_t *y);
int16_t* FFT1024(int16_t *x, int16_t *y);




int initial_sync(PHY_VARS_UE *ue, runmode_t mode)
{
	int32_t syncPos_PSS; //synch timing detected by PSS (without considering CP)
	int32_t syncPos_symBnd; //synch timing : start timing of PSS related OFDM symbol + CP
	int32_t pssSlotOffset; //PSS - slot boundary offset

	int32_t rxdataF_PSS0[PHY_SYNC_MAXNB_OFDMSYMBOL], rxdataF_SSS0[PHY_SYNC_MAXNB_OFDMSYMBOL];
	int32_t rxdataF_PSS5[PHY_SYNC_MAXNB_OFDMSYMBOL], rxdataF_SSS5[PHY_SYNC_MAXNB_OFDMSYMBOL];

	int32_t pss0_ext[4][PHY_SYNC_SS_LENGTH], pss5_ext[4][PHY_SYNC_SS_LENGTH];
	int32_t sss0_ext[4][PHY_SYNC_SS_LENGTH], sss5_ext[4][PHY_SYNC_SS_LENGTH];

	
	uint8_t flip_fdd_ncp;

	LTE_DL_FRAME_PARMS *frame_parms = &ue->frame_parms;
	int ret=-1;
	int initial_N_DL_RB = frame_parms->N_RB_DL;

	pssSlotOffset = (frame_parms->samples_per_tti>>1) - frame_parms->ofdm_symbol_size - frame_parms->nb_prefix_samples;


	//1. PSS detection -----------------------------------------------------------
	//input : ue->common_vars.rxdata (sampled data (time domain signal))
	//        frame_parms : frame structure parameters
	//        primary_synch0_time (original PSS sequence values in time domain)
	//output : ue->common_vars.eNb_id (NID2)
	//         syncPos_PSS (timing offset that maximize PSS correlation metric)
	//reference : lte_sync_time.c, cdot_product.c

	#ifdef LTESYNC_PSS_SOLUTION
	//solution of the PSS code
	syncPos_PSS = PSS_detection(ue->common_vars.rxdata, LTE_NUMBER_OF_SUBFRAMES_PER_FRAME*frame_parms->samples_per_tti, frame_parms->ofdm_symbol_size, (int *)&(ue->common_vars.eNb_id) ); 
	#else
	// PSS can be coded here








	
	#endif
	
	LOG_E(PHY,"[UE%d] Initial sync : Estimated PSS position %d, Nid2 %d\n",ue->Mod_id,syncPos_PSS,ue->common_vars.eNb_id);









	
	//1.2 offset computation (PSS OFDM symbol start timing)
	//input : syncPos_PSS
	//output : syncPos_symBnd ( symbol boundary offset of the OFDM symbol + CP that includes PSS)
	//         ue->rx_offset ( slot boundary offset)



	if (syncPos_PSS >= frame_parms->nb_prefix_samples)
		syncPos_symBnd = syncPos_PSS - frame_parms->nb_prefix_samples;
	else
		syncPos_symBnd = syncPos_PSS + FRAME_LENGTH_COMPLEX_SAMPLES - frame_parms->nb_prefix_samples;

	if (syncPos_symBnd >= pssSlotOffset)
		ue->rx_offset = syncPos_symBnd - pssSlotOffset;
	else
		ue->rx_offset = FRAME_LENGTH_COMPLEX_SAMPLES + syncPos_symBnd - pssSlotOffset;

	


	//SSS detection -----------------------------------------------------------

	// PSS is hypothesized in last symbol of first slot in Frame
	if ((syncPos_symBnd - pssSlotOffset) <= 0 ||
		(syncPos_symBnd - pssSlotOffset) < (FRAME_LENGTH_COMPLEX_SAMPLES-frame_parms->samples_per_tti/2))
    {
    	//2. FFT
    	//input : ue->common_vars.rxdata, ue->rx_offset
		//output : rxdataF_PSS0 (FFT processed OFDM symbol of PSS)
		//         rxdataF_PSS5 (FFT processed OFDM symbol of the PSS after 10 slots)
		//         rxdataF_SSS0 (FFT processed OFDM symbol of the neighbor SSS)
		//         rxdataF_SSS5 (FFT processed OFDM symbol of the SSS after 10 slots)
		//reference : slot_fep.c
		#ifdef LTESYNC_FFT_SOLUTION
		// Do FFTs for SSS/PSS
		// SSS0
		slot_fep(ue,
	             (frame_parms->symbols_per_tti/2)-2, // second to last symbol of
	             0,                                  // slot 0
	             ue->rx_offset,
	             0,1);
    	// PSS0
    	slot_fep(ue,
	             (frame_parms->symbols_per_tti/2)-1, // last symbol of
	             0,                                  // slot 0
	             ue->rx_offset,
	             0,1);
		memcpy(rxdataF_PSS0, &(ue->common_vars.common_vars_rx_data_per_thread[ue->current_thread_id[0]].rxdataF[0][frame_parms->ofdm_symbol_size*6]), 
			PHY_SYNC_MAXNB_OFDMSYMBOL*sizeof(int32_t));
		memcpy(rxdataF_SSS0, &(ue->common_vars.common_vars_rx_data_per_thread[ue->current_thread_id[0]].rxdataF[0][frame_parms->ofdm_symbol_size*5]),
			PHY_SYNC_MAXNB_OFDMSYMBOL*sizeof(int32_t));
		
		// SSS5
		slot_fep(ue,
				 (frame_parms->symbols_per_tti/2)-2,
				 10,
				 ue->rx_offset,
				 0,1);
		// PSS5
		slot_fep(ue,
				 (frame_parms->symbols_per_tti/2)-1,
				 10,
				 ue->rx_offset,
				 0,1);		
		memcpy(rxdataF_PSS5, &(ue->common_vars.common_vars_rx_data_per_thread[ue->current_thread_id[5]].rxdataF[0][frame_parms->ofdm_symbol_size*6]),
			PHY_SYNC_MAXNB_OFDMSYMBOL*sizeof(int32_t));
		memcpy(rxdataF_SSS5, &(ue->common_vars.common_vars_rx_data_per_thread[ue->current_thread_id[5]].rxdataF[0][frame_parms->ofdm_symbol_size*5]),
			PHY_SYNC_MAXNB_OFDMSYMBOL*sizeof(int32_t));

		
		#else

		// test code area //
#if 0
		FFT4((int16_t*)&(ue->common_vars.rxdata[0][syncPos_PSS]), result) ;
		rxPtr = (int16_t*)&ue->common_vars.rxdata[0][syncPos_PSS];

		LOG_E(PHY, "input : %i+i%i, %i+i%i, %i+i%i, %i+i%i\n", 
		       (int16_t)rxPtr[0],
		       (int16_t)rxPtr[1],
		       (int16_t)rxPtr[2],
		       (int16_t)rxPtr[3],
		       (int16_t)rxPtr[4],
		       (int16_t)rxPtr[5],
		       (int16_t)rxPtr[6],
		       (int16_t)rxPtr[7]);
		LOG_E(PHY, "fft result : %i+i%i, %i+i%i, %i+i%i, %i+i%i\n", 
		       result[0],
		       result[1],
		       result[2],
		       result[3],
		       result[4],
		       result[5],
		       result[6],
		       result[7]);
#endif


#if 0

		int16_t* rxPtr;
		int16_t result_16[10000], result_64[10000], result_256[10000], result_1024[10000];

		FFT16((int16_t*)&(ue->common_vars.rxdata[0][syncPos_PSS]), (int16_t*)result_16);

#ifdef LTESYNC_DEBUG_PRINTON	  
		LOG_E(PHY, "FFT16\n");
		dumpCompData(&(ue->common_vars.rxdata[0][syncPos_PSS]), 16, "FFT 16 input");
		dumpCompData(result_16, 16, "FFT 16 result");
		printf("\n\n\n");
#endif		  
		  
		

		FFT64((int16_t*)&(ue->common_vars.rxdata[0][syncPos_PSS]), (int16_t*)result_64);

#ifdef LTESYNC_DEBUG_PRINTON
		LOG_E(PHY, "FFT64\n");
		dumpCompData(&(ue->common_vars.rxdata[0][syncPos_PSS]), 64, "FFT 64 input");
		dumpCompData(result_64, 64, "FFT 64 result");
		printf("\n\n\n");
#endif
		  

		FFT256((int16_t*)&(ue->common_vars.rxdata[0][syncPos_PSS]), (int16_t*)result_256);

#ifdef LTESYNC_DEBUG_PRINTON
		LOG_E(PHY, "FFT256\n");
		dumpCompData(&(ue->common_vars.rxdata[0][syncPos_PSS]), 256, "FFT 256 input");
		dumpCompData(result_256, 256, "FFT 256 result");
		printf("\n\n\n");
#endif
		  
		  
		FFT1024((int16_t*)&(ue->common_vars.rxdata[0][syncPos_PSS]), (int16_t*)result_1024);

#ifdef LTESYNC_DEBUG_PRINTON
		LOG_E(PHY, "FFT1024\n");
		dumpCompData(&(ue->common_vars.rxdata[0][syncPos_PSS]), 1024, "FFT 1024 input");
		dumpCompData(result_1024, 1024, "FFT 1024 result");
		printf("\n\n\n"); 
#endif
		   
#endif
		int syncPos_PSS0 = syncPos_PSS;
		int syncPos_PSS5 = syncPos_PSS + 5*(frame_parms->samples_per_tti);
		int syncPos_SSS0 = syncPos_PSS0 - (ue->frame_parms.ofdm_symbol_size + ue->frame_parms.nb_prefix_samples);
		int syncPos_SSS5 = syncPos_PSS5 - (ue->frame_parms.ofdm_symbol_size + ue->frame_parms.nb_prefix_samples);



		FFT1024((int16_t*)&(ue->common_vars.rxdata[0][syncPos_PSS0]), (int16_t*)rxdataF_PSS0);
		FFT1024((int16_t*)&(ue->common_vars.rxdata[0][syncPos_PSS5]), (int16_t*)rxdataF_PSS5);
		FFT1024((int16_t*)&(ue->common_vars.rxdata[0][syncPos_SSS0]), (int16_t*)rxdataF_SSS0);
		FFT1024((int16_t*)&(ue->common_vars.rxdata[0][syncPos_SSS5]), (int16_t*)rxdataF_SSS5);



		
		#endif


		// pss sss extract for subframe 0
		extract_pssSss(pss0_ext[0], rxdataF_PSS0, ue->frame_parms.ofdm_symbol_size);
		extract_pssSss(sss0_ext[0], rxdataF_SSS0, ue->frame_parms.ofdm_symbol_size);
		
		pss_ch_est(ue,
				   pss0_ext,
				   sss0_ext);

		
		// pss sss extract for subframe 5
		extract_pssSss(pss5_ext[0], rxdataF_PSS5, ue->frame_parms.ofdm_symbol_size);
		extract_pssSss(sss5_ext[0], rxdataF_SSS5, ue->frame_parms.ofdm_symbol_size);

		
		pss_ch_est(ue,
				   pss5_ext,
				   sss5_ext);



		
		
		//4. SSS detection
		//input : SSS0_ext, SSS5_ext, ue->common_vars.eNb_id (NID2)
		//        d0_sss, d5_sss (original SSS values)
		//output : ue->common_vars.eNb_id (NID1)
		//         flip_fdd_ncp (frame boundary indication)
		//reference : sss.c
		#ifdef LTESYNC_SSS_SOLUTION
		int metric, tot_metric=-99999;
		int flip, phase, Nid1;
		int16_t *d0,*d5;
		int Nid2 = ue->common_vars.eNb_id;
		
		int16_t *sss0 = (int16_t*)&sss0_ext[0][5];
		int16_t *sss5 = (int16_t*)&sss5_ext[0][5];

		
		

		flip_fdd_ncp = 0;
		for (flip=0; flip<2; flip++)
		{	  //  d0/d5 flip in RX frame
			for (phase=0; phase<7; phase++)
			{ // phase offset between PSS and SSS
				for (Nid1 = 0 ; Nid1 <= 167; Nid1++)
				{  	// 168 possible Nid1 values
					metric = 0;

					
					if (flip==0)
					{
						d0 = &d0_sss[62*(Nid2 + (Nid1*3))];
						d5 = &d5_sss[62*(Nid2 + (Nid1*3))];
					} 
					else
					{
					  	d5 = &d0_sss[62*(Nid2 + (Nid1*3))];
					  	d0 = &d5_sss[62*(Nid2 + (Nid1*3))];
					}


					// This is the inner product using one particular value of each unknown parameter
					for (int i=0; i<62; i++)
					{
					  	metric += (int16_t)(((d0[i]*((((phase_re[phase]*(int32_t)sss0[i<<1])>>19)-((phase_im[phase]*(int32_t)sss0[1+(i<<1)])>>19)))) +
										   (d5[i]*((((phase_re[phase]*(int32_t)sss5[i<<1])>>19)-((phase_im[phase]*(int32_t)sss5[1+(i<<1)])>>19))))));
					}

					// if the current metric is better than the last save it
					if (metric > tot_metric)
					{
						tot_metric = metric;
						ue->frame_parms.Nid_cell = Nid2+(3*Nid1);
						flip_fdd_ncp = flip;
					}
	  			}
			}
		}
		#else



		int metric,tot_metric = -9999999;
		int flip, phase, Nid1; // flip?? 
		int max_Nid1; // flip?? 
		int16_t *d0,*d5;
		int Nid2 = ue->common_vars.eNb_id; //Nid2 = estimatedNID2

		int i,m,n =0;
	
		short SSScase1[64];
		
		short SSScase2[64];

		short sss_real[62];
		
		int16_t *sss0 = (int16_t*)&sss0_ext[0][5];
		

		int *mmcumul1;
		int flag = 0;
		
		__m128i *case1,case3,case4, mmcumul_zero2,*case2, case2real, *case22real, *case3pre, *case4pre, premmtmp1, premmtmp2, mmtmp1, mmtmp2, mmcumul_re, mmcumul_zero, mmcumul, mmssscase;
		//__m64 *case2;

		long long timeSSS;


		timeSSS = rdtsc_oai();

		
		mmcumul_zero = _mm_setzero_si128();
		mmcumul_zero2 = _mm_setzero_si128();
			
		memset(SSScase2,0,64*sizeof(short));	
		memset(SSScase1,0,64*sizeof(short));
		
			
		

		flip_fdd_ncp = 0;
		for (flip=0; flip<2; flip++) 
		{	  //  d0/d5 flip in RX frame
			for (phase=0; phase<7; phase++) // 7�� �ݺ�
			{ // phase offset between PSS and SSS
				for (Nid1 = 0 ; Nid1 <= 167; Nid1++) // Nid1 = testNid
				{  	// 168 possible Nid1 values
					metric = 0;
					mmcumul_re = _mm_setzero_si128();
					mmcumul = _mm_setzero_si128();

					
					
					print128_epi16(sss0, "sss0");

					for (n=0; n<62; n++)
					{
					    m = 2*n;
						sss_real[n] = sss0[m]; 
					}
					
					memcpy(SSScase1, sss_real, 62*sizeof(short)); 
											
					
					// SSScase1은 16bit 짜리 원소 64개 끝에 두개는 00 이 0은 16bit
					


					if (flip==0) // seq = 1 ￀ￏ ﾶﾧ
					{
						//d0 = &d0_sss[62*(Nid2 + (Nid1*3))]; // Nid1 = testNid
						//d5 = &d5_sss[62*(Nid2 + (Nid1*3))];

						memcpy(SSScase2, &d0_sss[62*(Nid2 + (Nid1*3))], 62*sizeof(short)); // d0_sss ?ﾤ￬ﾗﾐ  00 ﾺ￬?S case2
						
						
					}   
					else // seq = 2 ￀ￏ ﾶﾧ
					{
					  	//d5 = &d5_sss[62*(Nid2 + (Nid1*3))];
					  	//d0 = &d5_sss[62*(Nid2 + (Nid1*3))];

						memcpy(SSScase2, &d5_sss[62*(Nid2 + (Nid1*3))], 62*sizeof(short));  // d5_sss ﾵￚ﾿ﾡ 00￀ￌ ﾺ￙￀ﾺ SSScase2
					  	
					  	
					}
					
					

					
					case1 = (__m128i*)&SSScase1[0];

					

					case2 = (__m128i*)&SSScase2[0]; 

					

					
					//case22real = &case2real[0];
										
					//(a_re - i a_im) * (b_re + i b_im) == SSScase2[i] * SSScase1[i]  
					for (i=0; i<8; i++)
					{
						mmtmp1 = _mm_madd_epi16(case1[0],case2[0]);  
						//mmtmp2 = _mm_srai_epi32(mmtmp1,16);  
						mmcumul_re = _mm_add_epi32(mmcumul_re,mmtmp1);
						case1++;
						case2++;
					}
					
					print128_epi32(&mmcumul_re, "mmcumul_re");
					
					 mmcumul = _mm_hadd_epi32(mmcumul_re,mmcumul_zero);
				 	 mmcumul = _mm_hadd_epi32(mmcumul,mmcumul_zero);


					 print128_epi32(&mmcumul, "mmcumul");
					 
					 
					 mmcumul1 = (int*)&mmcumul[0];

					 metric = mmcumul1[0];
					 
					 //metric = _mm_cvtsi64_si32(mmcumul1);
				
					 

					// This is the inner product using one particular value of each unknown parameter

					

					// if the current metric is better than the last save it
					if (metric > tot_metric) // tot_metric = max_metric
					{
						tot_metric = metric;
						ue->frame_parms.Nid_cell = Nid2+(3*Nid1); // estimatedNID ﾰﾪ 
						flip_fdd_ncp = flip; // max_seq == flip_fdd_ncp
						max_Nid1 = Nid1;							
					}
	  			}


			}
		}
		
		 printf("[SSS det] WHAT I NEED : %d, %d  (estimated time %d us)\n",ue->frame_parms.Nid_cell,max_Nid1, (int)((rdtsc_oai() - timeSSS)/cpuf/1000.0));
		
		 

		
		


		#endif

		//PBCH detection (cross check)		
		if (flip_fdd_ncp==1)
			ue->rx_offset += (FRAME_LENGTH_COMPLEX_SAMPLES>>1);
		ret = tryPbchDecoding(ue, mode);

	    LOG_E(PHY,"SS detection result: CellId %d metric %d, flip %d, pbch %d\n", ue->frame_parms.Nid_cell, tot_metric, flip_fdd_ncp,ret);
  	} 
	else
	{
    	LOG_E(PHY,"SSS error condition: sync_pos %d, sync_pos_slot %d\n", syncPos_symBnd, pssSlotOffset);
  	}

  	/* Consider this is a false detection if the offset is > 1000 Hz */
	if( (abs(ue->common_vars.freq_offset) > 200) && (ret == 0) )
	{
		ret=-2;
		ue->frame_parms.N_RB_DL = initial_N_DL_RB; //RB number should be also rolled back!!
		LOG_E(HW, "Ignore MIB with high freq offset [%d Hz] estimation \n",ue->common_vars.freq_offset);
	}


	//post process of synchronization
  	if (ret==0) //sync succeed
	{
		postProcess_syncSuccess(ue); // PBCH found so indicate sync to higher layers and configure frame parameters
  	} 
	else //sync failed
	{
  		postProcess_syncFailed(ue, syncPos_PSS);
		calc_syncRssi(ue, syncPos_symBnd);
  	}

  	return ret;
}





#ifndef LTESYNC_FFT_SOLUTION


int16_t* FFT4(int16_t *x, int16_t *y) 
{ 
	 int32_t* x32, *pcumulre, *pcumulim; 
	 __m128i x128, mmcumulre1, mmcumulim1, mmcumulre2, mmcumulim2, mmtmpre, mmtmpim, *result_FFT4;  
	 __m128i *Wptr; 


	 x32 = (int32_t*)x;
	 result_FFT4 = (__m128i*)y;
	 
	 mmcumulre1= _mm_setzero_si128();
	 mmcumulim1= _mm_setzero_si128();
	 
	 for(int i=0;i<4;i++)
	 { 
		  x128 = _mm_set_epi32(x32[i], x32[i], x32[i], x32[i]);
		  Wptr = (__m128i*)minusW4[i];
		  mmtmpre=_mm_madd_epi16(x128,Wptr[0]); 


		  Wptr = (__m128i*)shuffleW4[i];
		  mmtmpim=_mm_madd_epi16(x128,Wptr[0]); 

		  mmtmpre=_mm_srai_epi32(mmtmpre, 15);
		  mmtmpim=_mm_srai_epi32(mmtmpim, 15);

		  /*
		  printf("madd result  real (%i) : ", i);
		  dumpSimdData(&mmtmpre, 1);
		  printf("madd result  imag (%i) : ", i);
		  dumpSimdData(&mmtmpim, 1);
		  printf("\n");
		  */

		   
		  mmcumulre1 = _mm_add_epi32(mmcumulre1, mmtmpre);
		  mmcumulim1 = _mm_add_epi32(mmcumulim1, mmtmpim);

		  /*
		  printf("add result  real (%i) : ", i);
		  dumpSimdData(&mmcumulre1, 1);
		  printf("add result  imag (%i) : ", i);
		  dumpSimdData(&mmcumulim1, 1);
		  printf("\n");
		  */
  
 	 }

	 mmcumulre2=_mm_packs_epi32(mmcumulre1, mmcumulre1); 
	 mmcumulim2=_mm_packs_epi32(mmcumulim1, mmcumulim1);


	 /*
	 printf("pack result  real : ");
	 dumpSimdData(&mmcumulre2, 0);
	 printf("pack result  imag : ");
	 dumpSimdData(&mmcumulim2, 0);
	 printf("\n");
	 */

	 result_FFT4[0] =_mm_unpacklo_epi16(mmcumulre2, mmcumulim2);

	 /*
	 printf("pack result  final : ");
	 dumpSimdData(&result_FFT4[0], 0);
	 */
	 
	 
	 _mm_empty();
	 _m_empty();
	 
	 return((int16_t*) result_FFT4);


}


int16_t* bflyRadix4_16(int16_t *x, int16_t *y, int16_t *minus_W0, int16_t *minus_W1, int16_t *minus_W2, int16_t *minus_W3, 
						int16_t *shuffle_W0, int16_t *shuffle_W1, int16_t *shuffle_W2, int16_t *shuffle_W3)
{
	__m128i *pW16, *x128, *y128;
	__m128i mmtmpre0, mmtmpre1, mmtmpre2, mmtmpre3, mmtmpim0, mmtmpim1, mmtmpim2, mmtmpim3, mmre, mmim;

	
	y128 = (__m128i*)y;


	for(int i=0;i<4;i++)
	{
		
		x128 = (__m128i*)x;
		
		//1st term's (i)th output 4개
		pW16 = (__m128i*)&minus_W0[i*8]; 
		mmtmpre0 = _mm_madd_epi16(*x128, pW16[0]);	
				
		pW16= (__m128i*)&shuffle_W0[i*8];
		mmtmpim0 = _mm_madd_epi16(*x128, pW16[0]);
			
		mmtmpre0 = _mm_srai_epi32(mmtmpre0, 15);
		mmtmpim0 = _mm_srai_epi32(mmtmpim0, 15);

		x128++;
		
		
		//2nd term's (i)th output 4개
		pW16 = (__m128i*)&minus_W1[i*8];

		
		mmtmpre1= _mm_madd_epi16(*x128, pW16[0]);
		
		pW16 = (__m128i*)&shuffle_W1[i*8];

		
		mmtmpim1 = _mm_madd_epi16(*x128, pW16[0]);


		mmtmpre1 = _mm_srai_epi32(mmtmpre1, 15);
		mmtmpim1 = _mm_srai_epi32(mmtmpim1, 15);


		x128++;


		//3rd term's (i)th output 4개
		pW16 = (__m128i*)&minus_W2[i*8];
		mmtmpre2= _mm_madd_epi16(*x128, pW16[0]);

		
		pW16 = (__m128i*)&shuffle_W2[i*8];
		mmtmpim2 = _mm_madd_epi16(*x128, pW16[0]);


		mmtmpre2 = _mm_srai_epi32(mmtmpre2, 15);
		mmtmpim2 = _mm_srai_epi32(mmtmpim2, 15);

			
		x128++;

		
		//4th term's (i)th output 4개
		pW16 = (__m128i*)&minus_W3[i*8];
		mmtmpre3= _mm_madd_epi16(*x128, pW16[0]);
		
		pW16 = (__m128i*)&shuffle_W3[i*8];
		mmtmpim3 = _mm_madd_epi16(*x128, pW16[0]);
		

		mmtmpre3 = _mm_srai_epi32(mmtmpre3, 15);
		mmtmpim3 = _mm_srai_epi32(mmtmpim3, 15);

	
		//final result_ cumulation
		mmre = _mm_add_epi32(mmtmpre0, mmtmpre1);
		mmre = _mm_add_epi32(mmre, mmtmpre2);
		mmre = _mm_add_epi32(mmre, mmtmpre3); //real 4개가 나와야. 루프 돌 때마다 그 다음 real이 나와야. 

		mmim = _mm_add_epi32(mmtmpim0, mmtmpim1);
		mmim = _mm_add_epi32(mmim, mmtmpim2);
		mmim = _mm_add_epi32(mmim, mmtmpim3); //im 4개가 나와야. i가 증가할 때마다 그다음 4개 imaginary수가 나와야.
		
		
		//bit control
		mmre = _mm_packs_epi32(mmre, mmre);
		mmim = _mm_packs_epi32(mmim, mmim);	


		y128[0] = _mm_unpacklo_epi16(mmre, mmim);

		y128++;

	}

				 
	return(y);

}


	
int16_t* bflyRadix4_Nbit(int16_t *x, int16_t *y, int16_t *minus_W0, int16_t *minus_W1, int16_t *minus_W2, int16_t *minus_W3, 
						int16_t *shuffle_W0, int16_t *shuffle_W1, int16_t *shuffle_W2, int16_t *shuffle_W3, int N)
{
	__m128i *pW_N, *x128, *y128;
	__m128i __attribute__((aligned(16))) mmtmpre0[10000], __attribute__((aligned(16))) mmtmpre1[10000], __attribute__((aligned(16))) mmtmpre2[10000], __attribute__((aligned(16))) mmtmpre3[10000], 
		__attribute__((aligned(16))) mmtmpim0[10000], __attribute__((aligned(16))) mmtmpim1[10000], __attribute__((aligned(16))) mmtmpim2[10000], __attribute__((aligned(16))) mmtmpim3[10000], mmre, mmim;
	
	
	y128 = (__m128i*)y;


	for(int i=0; i<4; i++)
	{
		
		x128 = (__m128i*)x;

		//1st term's (i)th output 4개
		for(int j=0; j<(N/16); j++) //-----------------------add
		{
			pW_N = (__m128i*)&minus_W0[(N/2)*i+8*j]; 				//if i=0, minus_W0[0], minus_W0[8], minus_W0[16], minus_W0[24] 	--re	
			
			mmtmpre0[j] = _mm_madd_epi16(*x128, pW_N[0]);

			pW_N= (__m128i*)&shuffle_W0[(N/2)*i+8*j];             //if i=0, shuffle_W0[0], shuffle_W0[8], shuffle_W0[16], shuffle_W0[24] 	--im
			mmtmpim0[j] = _mm_madd_epi16(*x128, pW_N[0]);

			mmtmpre0[j] = _mm_srai_epi32(mmtmpre0[j], 15);
			mmtmpim0[j] = _mm_srai_epi32(mmtmpim0[j], 15);

			pW_N++;
			x128++;
		}

		
		
		//2nd term's (i)th output 4개 
		for(int j=0; j<(N/16); j++) //-----------------------add
		{
			pW_N = (__m128i*)&minus_W1[(N/2)*i+8*j]; 				//if i=0, minus_W0[0], minus_W0[8], minus_W0[16], minus_W0[24] 	--re	
			
			mmtmpre1[j] = _mm_madd_epi16(*x128, pW_N[0]);

			pW_N= (__m128i*)&shuffle_W1[(N/2)*i+8*j];             //if i=0, shuffle_W0[0], shuffle_W0[8], shuffle_W0[16], shuffle_W0[24] 	--im
			mmtmpim1[j] = _mm_madd_epi16(*x128, pW_N[0]);

			mmtmpre1[j] = _mm_srai_epi32(mmtmpre1[j], 15);
			mmtmpim1[j] = _mm_srai_epi32(mmtmpim1[j], 15);
 
			pW_N++;
			x128++;
		}



		//3rd term's (i)th output 4개
		for(int j=0; j<(N/16); j++) //-----------------------add
		{
			pW_N = (__m128i*)&minus_W2[(N/2)*i+8*j]; 				//if i=0, minus_W0[0], minus_W0[8], minus_W0[16], minus_W0[24] 	--re	
			
			mmtmpre2[j] = _mm_madd_epi16(*x128, pW_N[0]);

			pW_N= (__m128i*)&shuffle_W2[(N/2)*i+8*j];             //if i=0, shuffle_W0[0], shuffle_W0[8], shuffle_W0[16], shuffle_W0[24] 	--im
			mmtmpim2[j] = _mm_madd_epi16(*x128, pW_N[0]);

			mmtmpre2[j] = _mm_srai_epi32(mmtmpre2[j], 15);
			mmtmpim2[j] = _mm_srai_epi32(mmtmpim2[j], 15);

			pW_N++;
			x128++;
		}


		
		//4th term's (i)th output 4개
		for(int j=0; j<(N/16); j++) //-----------------------add
		{
			pW_N = (__m128i*)&minus_W3[(N/2)*i+8*j]; 				//if i=0, minus_W0[0], minus_W0[8], minus_W0[16], minus_W0[24] 	--re	
			
			mmtmpre3[j] = _mm_madd_epi16(*x128, pW_N[0]);

			pW_N= (__m128i*)&shuffle_W3[(N/2)*i+8*j];             //if i=0, shuffle_W0[0], shuffle_W0[8], shuffle_W0[16], shuffle_W0[24] 	--im
			mmtmpim3[j] = _mm_madd_epi16(*x128, pW_N[0]);

			mmtmpre3[j] = _mm_srai_epi32(mmtmpre3[j], 15);
			mmtmpim3[j] = _mm_srai_epi32(mmtmpim3[j], 15);

			pW_N++;
			x128++;
		}




		//final result_ cumulation
		for(int j=0; j<(N/16); j++)
		{

			mmre = _mm_add_epi32(mmtmpre0[j], mmtmpre1[j]);
			mmre = _mm_add_epi32(mmre, mmtmpre2[j]);
			mmre = _mm_add_epi32(mmre, mmtmpre3[j]); 

			mmim = _mm_add_epi32(mmtmpim0[j], mmtmpim1[j]);
			mmim = _mm_add_epi32(mmim, mmtmpim2[j]);
			mmim = _mm_add_epi32(mmim, mmtmpim3[j]); 
		
			
			//bit control
			mmre = _mm_packs_epi32(mmre, mmre);
			mmim = _mm_packs_epi32(mmim, mmim);	

			y128[0] = _mm_unpacklo_epi16(mmre, mmim);


			y128++;
		}
		
	}

				 
	return(y);
	
}




int16_t* FFT16(int16_t *x, int16_t *y) //ssss
{
	int16_t __attribute__((aligned(16))) y_4[10000]; //*outputy;
	int32_t *x32;
	__m128i x128, *y128_4;

	x32 = (int32_t*)x;
	y128_4 = (__m128i*)y_4;

	for(int i=0;i<4;i++)
	{
		
		x128 =_mm_set_epi32(x32[i+12], x32[i+8], x32[i+4], x32[i]); 
		FFT4((int16_t*)&x128, (int16_t*)y128_4);
		y128_4++;
	}
	

	bflyRadix4_16(y_4, y, minus_W16_0, minus_W16_1, minus_W16_2, minus_W16_3, shuffle_W16_0, shuffle_W16_1, shuffle_W16_2, shuffle_W16_3);


	_mm_empty();
	_m_empty();

	return (y);
}



int16_t* FFT64(int16_t *x, int16_t *y) 
	{
		int16_t __attribute__((aligned(16))) y_16[10000]; 
		int32_t *x32;
		int16_t __attribute__((aligned(16)))x128[10000];
		__m128i *px128, *y128_16;
	
		x32 = (int32_t*)x;
		y128_16 = (__m128i*)y_16;
		px128 = (__m128i*)x128;
		
		for(int i=0;i<4;i++)
			{
				for(int j=0; j<4; j++)
					{		
						//printf("FFT64 : (%i, %i)\n",i,j);
						//fflush(stdout);
						px128[0] =_mm_set_epi32(x32[i+16*j+12], x32[i+16*j+8], x32[i+16*j+4], x32[i+16*j]); //if i=0, x(0), x(4), x(8), x(12), ... , x(60) , N=64's 'input
						px128++;	
					}
		
				FFT16((int16_t*)(px128-4), (int16_t*)y128_16);
				y128_16 = y128_16+4;
		
			}

		bflyRadix4_Nbit(y_16, y,  minus_W64_0, minus_W64_1, minus_W64_2, minus_W64_3, shuffle_W64_0, shuffle_W64_1, shuffle_W64_2, shuffle_W64_3, 64);

		
		_mm_empty();
		_m_empty();

		return (y);
	
	}



int16_t* FFT256(int16_t *x, int16_t *y) //ssss
	{
		int16_t __attribute__((aligned(16))) y_64[10000]; //*outputy;
		int32_t *x32;
		int16_t __attribute__((aligned(16))) x128[10000];
		__m128i *px128, *y128_64;

		x32 = (int32_t*)x;
		px128 = (__m128i*)x128;
		y128_64 = (__m128i*)y_64;

		for(int i=0;i<4;i++)
			{
				for(int j=0; j<16; j++)
					{		
						px128[0] =_mm_set_epi32(x32[i+16*j+12], x32[i+16*j+8], x32[i+16*j+4], x32[i+16*j]); //if i=0, x(0), x(4), x(8), x(12), ... , x(60) , N=64's 'input
						px128++;
						
					}
				
					
				FFT64((int16_t*)(px128-16), (int16_t*)y128_64);
				
				y128_64 = y128_64+16;
				

			}

		bflyRadix4_Nbit(y_64, y,  minus_W256_0, minus_W256_1, minus_W256_2, minus_W256_3, shuffle_W256_0, shuffle_W256_1, shuffle_W256_2, shuffle_W256_3, 256);
		
		_mm_empty();
		_m_empty();

		return (y);
		


	
	}


int16_t* FFT1024(int16_t *x, int16_t *y) //ssss
	{
		int16_t __attribute__((aligned(16))) y_256[10000]; //*outputy;
		int32_t *x32;
		int16_t __attribute__((aligned(16))) x128[10000];
		__m128i *px128, *y128_256;

		x32 = (int32_t*)x;
		px128 = (__m128i*)x128;
		y128_256 = (__m128i*)y_256;

		for(int i=0;i<4;i++)
			{
				for(int j=0; j<64; j++)
					{		
						px128[0] =_mm_set_epi32(x32[i+16*j+12], x32[i+16*j+8], x32[i+16*j+4], x32[i+16*j]); //if i=0, x(0), x(4), x(8), x(12), ... , x(60) , N=64's 'input
						px128++;
						
					}
				
					
				FFT256((int16_t*)(px128-64), (int16_t*)y128_256);
				
				y128_256 = y128_256+64;

			}

		bflyRadix4_Nbit(y_256, y,  minus_W1024_0, minus_W1024_1, minus_W1024_2, minus_W1024_3, shuffle_W1024_0, shuffle_W1024_1, shuffle_W1024_2, shuffle_W1024_3, 1024);

		_mm_empty();
		_m_empty();


		return (y);


	
	}

	
 
#endif






