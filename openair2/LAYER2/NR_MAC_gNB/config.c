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

/*! \file config.c
 * \brief gNB configuration performed by RRC or as a consequence of RRC procedures
 * \author  Navid Nikaein and Raymond Knopp, WEI-TAI CHEN
 * \date 2010 - 2014, 2018
 * \version 0.1
 * \company Eurecom, NTUST
 * \email: navid.nikaein@eurecom.fr, kroempa@gmail.com
 * @ingroup _mac

 */

#include "COMMON/platform_types.h"
#include "common/platform_constants.h"
#include "common/ran_context.h"
#include "common/utils/nr/nr_common.h"
#include "common/utils/LOG/log.h"
#include "common/utils/LOG/vcd_signal_dumper.h"

#include "NR_BCCH-BCH-Message.h"
#include "NR_ServingCellConfigCommon.h"

#include "LAYER2/NR_MAC_gNB/mac_proto.h"
#include "SCHED_NR/phy_frame_config_nr.h"
#include "openair1/PHY/defs_gNB.h"

#include "NR_MIB.h"
#include "LAYER2/NR_MAC_COMMON/nr_mac_common.h"
#include "../../../../nfapi/oai_integration/vendor_ext.h"
/* Softmodem params */
#include "executables/softmodem-common.h"
#include <complex.h>

extern RAN_CONTEXT_t RC;
//extern int l2_init_gNB(void);
extern uint8_t nfapi_mode;

nfapi_nr_pm_list_t init_DL_MIMO_codebook(gNB_MAC_INST *gNB, rrc_pdsch_AntennaPorts_t antenna_ports)
{
  nfapi_nr_pm_list_t mat;
  int num_antenna_ports = antenna_ports.N1 * antenna_ports.N2 * antenna_ports.XP;
  if(num_antenna_ports < 2) {
    mat.num_pm_idx = 0;
    mat.pmi_pdu = NULL;
    return mat;
  }

  //NR Codebook Generation for codebook type1 SinglePanel
  int N1 = antenna_ports.N1;
  int N2 = antenna_ports.N2;
  //Uniform Planner Array: UPA
  //    X X X X ... X
  //    X X X X ... X
  // N2 . . . . ... .
  //    X X X X ... X
  //   |<-----N1---->|

  //Get the uniform planar array parameters
  // To be confirmed
  int O2 = N2 > 1 ? 4 : 1; //Vertical beam oversampling (1 or 4)
  int O1 = num_antenna_ports > 2 ? 4 : 1; //Horizontal beam oversampling (1 or 4)

  // num of allowed k1 and k2 according to 5.2.2.2.1-3 and -4 in 38.214
  int K1;
  if(N2 == N1 || N1 == 2)
    K1 = 2;
  else if (N2 == 1)
    K1 = 5;
  else
    K1 = 3;
  int K2 = N2 > 1 ? 2 : 1;

  int max_mimo_layers = (num_antenna_ports < NR_MAX_NB_LAYERS) ? num_antenna_ports : NR_MAX_NB_LAYERS;
  AssertFatal(max_mimo_layers <= 4, "Max number of layers supported is 4\n");

  gNB->precoding_matrix_size[0] = N1 * O1 * N2 * O2 * 4;
  mat.num_pm_idx = gNB->precoding_matrix_size[0];
  for (int i = 1; i < max_mimo_layers; i++) {
    gNB->precoding_matrix_size[i] = 2 * N1 * O1 * N2 * O2 * K1 * K2;
    mat.num_pm_idx += gNB->precoding_matrix_size[i];
  }

  nfapi_nr_pm_pdu_t *pmi_pdu = malloc16(mat.num_pm_idx * sizeof(*pmi_pdu));
  mat.pmi_pdu = pmi_pdu;

  // Generation of codebook Type1 with codebookMode 1 (num_antenna_ports < 16)
  if (num_antenna_ports < 16) {
    //Generate DFT vertical beams
    //ll: index of a vertical beams vector (represented by i1_1 in TS 38.214)
    const int max_l = N1 * O1 + (K1 - 1) * O1;
    double complex v[max_l][N1];
    for (int ll = 0; ll < max_l; ll++) { //i1_1
      for (int nn = 0; nn < N1; nn++) {
        v[ll][nn] = cexp(I * (2 * M_PI * nn * ll) / (N1 * O1));
        LOG_D(PHY,"v[%d][%d] = %f +j %f\n", ll, nn, creal(v[ll][nn]), cimag(v[ll][nn]));
      }
    }
    //Generate DFT Horizontal beams
    //mm: index of a Horizontal beams vector (represented by i1_2 in TS 38.214)
    const int max_m = N2 * O2 + (K2 - 1) * O2;
    double complex u[max_m][N2];
    for (int mm = 0; mm < max_m; mm++) { //i1_2
      for (int nn = 0; nn < N2; nn++) {
        u[mm][nn] = cexp(I * (2 * M_PI * nn * mm) / (N2 * O2));
        LOG_D(PHY,"u[%d][%d] = %f +j %f\n", mm, nn, creal(u[mm][nn]), cimag(u[mm][nn]));
      }
    }
    //Generate co-phasing angles
    //i_2: index of a co-phasing vector
    //i1_1, i1_2, and i_2 are reported from UEs
    double complex theta_n[4];
    for (int nn = 0; nn < 4; nn++) {
      theta_n[nn] = cexp(I * M_PI * nn / 2);
      LOG_D(PHY,"theta_n[%d] = %f +j %f\n", nn, creal(theta_n[nn]), cimag(theta_n[nn]));
    }
    //Kronecker product v_lm
    double complex v_lm[max_l][max_m][N2 * N1];
    //v_ll_mm_codebook denotes the elements of a precoding matrix W_i1,1_i_1,2
    for(int ll = 0; ll < max_l; ll++) { //i_1_1
      for (int mm = 0; mm < max_m; mm++) { //i_1_2
        for (int nn1 = 0; nn1 < N1; nn1++) {
          for (int nn2 = 0; nn2 < N2; nn2++) {
            v_lm[ll][mm][nn1 * N2 + nn2] = v[ll][nn1] * u[mm][nn2];
            LOG_D(PHY,"v_lm[%d][%d][%d] = %f +j %f\n",ll, mm, nn1 * N2 + nn2, creal(v_lm[ll][mm][nn1*N2+nn2]), cimag(v_lm[ll][mm][nn1*N2+nn2]));
          }
        }
      }
    }

    double complex res_code;

    //Table 5.2.2.2.1-5:
    int pmiq = 0;
    //Codebook for 1-layer CSI reporting using antenna ports 3000 to 2999+PCSI-RS
    for(int ll = 0; ll < N1 * O1; ll++) { //i_1_1
      for (int mm = 0; mm < N2 * O2; mm++) { //i_1_2
        for (int nn = 0; nn < 4; nn++) {
          pmiq = ll * N2 * O2 * 4 + mm * 4 + nn;
          pmi_pdu[pmiq].pm_idx = pmiq + 1; // index 0 is the identity matrix
          pmi_pdu[pmiq].numLayers = 1;
          pmi_pdu[pmiq].num_ant_ports = num_antenna_ports;
          LOG_D(PHY, "layer 1 Codebook pmiq = %d\n", pmiq);
          for (int len = 0; len < N1 * N2; len++) {
            nfapi_nr_pm_weights_t *weights = &pmi_pdu[pmiq].weights[0][len];
            res_code = sqrt( 1 /(double)num_antenna_ports) * v_lm[ll][mm][len];
            if (creal(res_code) > 0)
              weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) + 0.5);//convert to Q15
            else
              weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) - 0.5);//convert to Q15
            if (cimag(res_code) > 0)
              weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) + 0.5);//convert to Q15
            else
              weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) - 0.5);//convert to Q15
            LOG_D(PHY, "1 Layer Precoding Matrix[0][pmi %d][antPort %d]= %f+j %f -> Fixed Point %d+j %d \n",
                  pmiq, len, creal(res_code), cimag(res_code), weights->precoder_weight_Re, weights->precoder_weight_Im);
          }

          for(int len = N1 * N2; len < 2 * N1 * N2; len++) {
            nfapi_nr_pm_weights_t *weights = &pmi_pdu[pmiq].weights[0][len];
            res_code = sqrt(1 / (double)num_antenna_ports) * theta_n[nn] * v_lm[ll][mm][len-N1*N2];
            if (creal(res_code) > 0)
              weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) + 0.5);//convert to Q15
            else
              weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) - 0.5);//convert to Q15
            if (cimag(res_code) > 0)
              weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) + 0.5);//convert to Q15
            else
              weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) - 0.5);//convert to Q15
            LOG_D(PHY, "1 Layer Precoding Matrix[0][pmi %d][antPort %d]= %f+j %f -> Fixed Point %d+j %d \n",
                  pmiq, len, creal(res_code), cimag(res_code), weights->precoder_weight_Re, weights->precoder_weight_Im);
          }
        }
      }
    }
    int llc = 0;
    int mmc = 0;
    double complex phase_sign = 0;
    //Table 5.2.2.2.1-6:
    //Codebook for 2-layer CSI reporting using antenna ports 3000 to 2999+PCSI-RS
    //Compute the code book size for generating 2 layers out of Tx antenna ports

    //pmi=1,...,pmi_size, we construct
    for(int ll = 0; ll < N1 * O1; ll++) { //i_1_1
      for (int mm = 0; mm < N2 * O2; mm++) { // i_1_2
        for(int k1 = 0; k1 < K1; k1++) {
          for (int k2 = 0; k2 < K2; k2++) {
            for (int nn = 0; nn < 2; nn++) {  // i_2
              pmiq ++;
              pmi_pdu[pmiq].pm_idx = pmiq + 1;  // index 0 is the identity matrix
              pmi_pdu[pmiq].numLayers = 2;
              pmi_pdu[pmiq].num_ant_ports = num_antenna_ports;
              LOG_D(PHY, "layer 2 Codebook pmiq = %d\n", pmiq);
              for(int j_col = 0; j_col < 2; j_col++) {
                if (j_col == 0) {
                  llc = ll;
                  mmc = mm;
                  phase_sign = 1;
                }
                if (j_col == 1) {
                  llc = ll + k1 * O1;
                  mmc = mm + k2 * O2;
                  phase_sign = -1;
                }
                for (int i_rows = 0; i_rows < N1 * N2; i_rows++) {
                  nfapi_nr_pm_weights_t *weights = &pmi_pdu[pmiq].weights[j_col][i_rows];
                  res_code = sqrt(1 / (double)(2 * num_antenna_ports)) * v_lm[llc][mmc][i_rows];
                  if (creal(res_code) > 0)
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) - 0.5);//convert to Q15
                  if (cimag(res_code) > 0)
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) - 0.5);//convert to Q15
                  LOG_D(PHY, "2 Layer Precoding Matrix[1][pmi %d][antPort %d][layerIdx %d]= %f+j %f -> Fixed Point %d+j %d \n",
                        pmiq, i_rows, j_col, creal(res_code), cimag(res_code), weights->precoder_weight_Re, weights->precoder_weight_Im);
                }
                for (int i_rows = N1 * N2; i_rows < 2 * N1 * N2; i_rows++) {
                  nfapi_nr_pm_weights_t *weights = &pmi_pdu[pmiq].weights[j_col][i_rows];
                  res_code = sqrt(1 / (double)(2 * num_antenna_ports)) * (phase_sign) * theta_n[nn] * v_lm[llc][mmc][i_rows - N1 * N2];
                  if (creal(res_code) > 0)
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) - 0.5);//convert to Q15
                  if (cimag(res_code) > 0)
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) - 0.5);//convert to Q15
                  LOG_D(PHY, "2 Layer Precoding Matrix[1][pmi %d][antPort %d][layerIdx %d]= %f+j %f -> Fixed Point %d+j %d \n",
                        pmiq, i_rows, j_col, creal(res_code), cimag(res_code), weights->precoder_weight_Re, weights->precoder_weight_Im);
                }
              }
            }
          }
        }
      }
    }

    if(max_mimo_layers < 3)
      return mat;

    //Table 5.2.2.2.1-7:
    //Codebook for 3-layer CSI reporting using antenna ports 3000 to 2999+PCSI-RS

    //pmi=1,...,pmi_size are computed as follows
    for(int ll = 0; ll < N1 * O1; ll++) { //i_1_1
      for (int mm = 0; mm < N2 * O2; mm++) { // i_1_2
        for(int k1 = 0; k1 < K1; k1++) {
          for (int k2 = 0; k2 < K2; k2++) {
            for (int nn = 0; nn < 2; nn++) {  // i_2
              pmiq ++;
              pmi_pdu[pmiq].pm_idx = pmiq + 1;  // index 0 is the identity matrix
              pmi_pdu[pmiq].numLayers = 3;
              pmi_pdu[pmiq].num_ant_ports = num_antenna_ports;
              LOG_D(PHY, "layer 3 Codebook pmiq = %d\n",pmiq);
              for(int j_col = 0; j_col < 3; j_col++) {
                if (j_col == 0) {
                  llc = ll;
                  mmc = mm;
                  phase_sign = 1;
                }
                if (j_col==1) {
                  llc = ll + k1 * O1;
                  mmc = mm + k2 * O2;
                  phase_sign = 1;
                }
                if (j_col==2) {
                  llc = ll;
                  mmc = mm;
                  phase_sign = -1;
                }
                for (int i_rows = 0; i_rows < N1 * N2; i_rows++) {
                  nfapi_nr_pm_weights_t *weights = &pmi_pdu[pmiq].weights[j_col][i_rows];
                  res_code = sqrt(1 / (double)(3 * num_antenna_ports)) * v_lm[llc][mmc][i_rows];
                  if (creal(res_code) > 0)
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) - 0.5);//convert to Q15
                  if (cimag(res_code) > 0)
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) - 0.5);//convert to Q15
                  LOG_D(PHY, "3 Layer Precoding Matrix[1][pmi %d][antPort %d][layerIdx %d]= %f+j %f -> Fixed Point %d+j %d \n",
                        pmiq, i_rows, j_col, creal(res_code), cimag(res_code), weights->precoder_weight_Re, weights->precoder_weight_Im);
                }
                for (int i_rows = N1 * N2; i_rows < 2 * N1 * N2; i_rows++) {
                  nfapi_nr_pm_weights_t *weights = &pmi_pdu[pmiq].weights[j_col][i_rows];
                  res_code=sqrt(1 / (double)(3 * num_antenna_ports)) * (phase_sign) * theta_n[nn] * v_lm[llc][mmc][i_rows - N1 * N2];
                  if (creal(res_code) > 0)
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) - 0.5);//convert to Q15
                  if (cimag(res_code) > 0)
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) - 0.5);//convert to Q15
                  LOG_D(PHY, "3 Layer Precoding Matrix[1][pmi %d][antPort %d][layerIdx %d]= %f+j %f -> Fixed Point %d+j %d \n",
                        pmiq, i_rows, j_col, creal(res_code), cimag(res_code), weights->precoder_weight_Re, weights->precoder_weight_Im);
                }
              }
            }
          }
        }
      }
    }

    if(max_mimo_layers < 4)
      return mat;

    //Table 5.2.2.2.1-8:
    //Codebook for 4-layer CSI reporting using antenna ports 3000 to 2999+PCSI-RS

    for(int ll = 0; ll < N1 * O1; ll++) { //i_1_1
      for (int mm = 0; mm < N2 * O2; mm++) { // i_1_2
        for(int k1 = 0; k1 < K1; k1++) {
          for (int k2 = 0; k2 < K2; k2++) {
            for (int nn = 0; nn < 2; nn++) {  // i_2
              pmiq ++;
              pmi_pdu[pmiq].pm_idx = pmiq + 1;  // index 0 is the identity matrix
              pmi_pdu[pmiq].numLayers = 4;
              pmi_pdu[pmiq].num_ant_ports = num_antenna_ports;
              LOG_D(PHY, "layer 4 pmiq = %d\n", pmiq);
              for(int j_col = 0; j_col < 4; j_col++) {
                if (j_col == 0) {
                  llc = ll;
                  mmc = mm;
                  phase_sign = 1;
                }
                if (j_col == 1) {
                  llc = ll + k1 * O1;
                  mmc = mm + k2 * O2;
                  phase_sign = 1;
                }
                if (j_col == 2) {
                  llc = ll;
                  mmc = mm;
                  phase_sign = -1;
                }
                if (j_col == 3) {
                  llc = ll + k1 * O1;
                  mmc = mm + k2 * O2;
                  phase_sign = -1;
                }
                for (int i_rows = 0; i_rows < N1 * N2; i_rows++) {
                  nfapi_nr_pm_weights_t *weights = &pmi_pdu[pmiq].weights[j_col][i_rows];
                  res_code=sqrt(1 / (double)(4 * num_antenna_ports)) * v_lm[llc][mmc][i_rows];
                  if (creal(res_code) > 0)
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) - 0.5);//convert to Q15
                  if (cimag(res_code) > 0)
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) - 0.5);//convert to Q15
                  LOG_D(PHY, "4 Layer Precoding Matrix[1][pmi %d][antPort %d][layerIdx %d]= %f+j %f -> Fixed Point %d+j %d \n",
                        pmiq, i_rows, j_col, creal(res_code), cimag(res_code), weights->precoder_weight_Re, weights->precoder_weight_Im);
                }
                for (int i_rows = N1 * N2; i_rows < 2 * N1 * N2; i_rows++) {
                  nfapi_nr_pm_weights_t *weights = &pmi_pdu[pmiq].weights[j_col][i_rows];
                  res_code=sqrt(1 / (double)(4 * num_antenna_ports)) * (phase_sign) * theta_n[nn] * v_lm[llc][mmc][i_rows - N1 * N2];
                  if (creal(res_code) > 0)
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Re = (short) ((creal(res_code) * 32768) - 0.5);//convert to Q15
                  if (cimag(res_code) > 0)
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) + 0.5);//convert to Q15
                  else
                    weights->precoder_weight_Im = (short) ((cimag(res_code) * 32768) - 0.5);//convert to Q15
                  LOG_D(PHY, "4 Layer Precoding Matrix[1][pmi %d][antPort %d][layerIdx %d]= %f+j %f -> Fixed Point %d+j %d \n",
                        pmiq, i_rows, j_col, creal(res_code), cimag(res_code), weights->precoder_weight_Re, weights->precoder_weight_Im);
                }
              }
            }
          }
        }
      }
    }
    return mat;
  }
  else
    AssertFatal(false, "Max number of antenna ports supported is currently 16\n");
}


void process_rlcBearerConfig(struct NR_CellGroupConfig__rlc_BearerToAddModList *rlc_bearer2add_list,
                             struct NR_CellGroupConfig__rlc_BearerToReleaseList *rlc_bearer2release_list,
                             NR_UE_sched_ctrl_t *sched_ctrl) {

  if (rlc_bearer2release_list) {
    for (int i = 0; i < rlc_bearer2release_list->list.count; i++) {
      for (int idx = 0; idx < sched_ctrl->dl_lc_num; idx++) {
        if (sched_ctrl->dl_lc_ids[idx] == *rlc_bearer2release_list->list.array[i]) {
          const int remaining_lcs = sched_ctrl->dl_lc_num - idx - 1;
          memmove(&sched_ctrl->dl_lc_ids[idx], &sched_ctrl->dl_lc_ids[idx + 1], sizeof(sched_ctrl->dl_lc_ids[idx]) * remaining_lcs);
          sched_ctrl->dl_lc_num--;
          break;
        }
      }
    }
  }

  if (rlc_bearer2add_list) {
    // keep lcids
    for (int i = 0; i < rlc_bearer2add_list->list.count; i++) {
      const int lcid = rlc_bearer2add_list->list.array[i]->logicalChannelIdentity;
      bool found = false;
      for (int idx = 0; idx < sched_ctrl->dl_lc_num; idx++) {
        if (sched_ctrl->dl_lc_ids[idx] == lcid) {
          found = true;
          break;
        }
      }

      if (!found) {
        sched_ctrl->dl_lc_num++;
        sched_ctrl->dl_lc_ids[sched_ctrl->dl_lc_num - 1] = lcid;
        LOG_D(NR_MAC, "Adding LCID %d (%s %d)\n", lcid, lcid < 4 ? "SRB" : "DRB", lcid);
      }
    }
  }

  LOG_D(NR_MAC, "In %s: total num of active bearers %d) \n",
      __FUNCTION__,
      sched_ctrl->dl_lc_num);

}


void process_drx_Config(NR_UE_sched_ctrl_t *sched_ctrl,NR_SetupRelease_DRX_Config_t *drx_Config) {
 if (!drx_Config) return;
 AssertFatal(drx_Config->present != NR_SetupRelease_DRX_Config_PR_NOTHING, "Cannot have NR_SetupRelease_DRX_Config_PR_NOTHING\n");

 if (drx_Config->present == NR_SetupRelease_DRX_Config_PR_setup) {
   LOG_I(NR_MAC,"Adding DRX config\n");
 }
 else {
   LOG_I(NR_MAC,"Removing DRX config\n");
 }
}

void process_schedulingRequestConfig(NR_UE_sched_ctrl_t *sched_ctrl,NR_SchedulingRequestConfig_t *schedulingRequestConfig) {
 if (!schedulingRequestConfig) return;

   LOG_I(NR_MAC,"Adding SchedulingRequestconfig\n");
}

void process_bsrConfig(NR_UE_sched_ctrl_t *sched_ctrl,NR_BSR_Config_t *bsr_Config) {
  if (!bsr_Config) return;
  LOG_I(NR_MAC,"Adding BSR config\n");
}

void process_tag_Config(NR_UE_sched_ctrl_t *sched_ctrl,NR_TAG_Config_t *tag_Config) {
  if (!tag_Config) return;
  LOG_I(NR_MAC,"Adding TAG config\n");
}

void process_phr_Config(NR_UE_sched_ctrl_t *sched_ctrl,NR_SetupRelease_PHR_Config_t *phr_Config) {
   if (!phr_Config) return;
   AssertFatal(phr_Config->present != NR_SetupRelease_PHR_Config_PR_NOTHING, "Cannot have NR_SetupRelease_PHR_Config_PR_NOTHING\n");

   if (phr_Config->present == NR_SetupRelease_PHR_Config_PR_setup) {
     LOG_I(NR_MAC,"Adding PHR config\n");
   }
   else {
     LOG_I(NR_MAC,"Removing PHR config\n");
   }
}

void process_CellGroup(NR_CellGroupConfig_t *CellGroup, NR_UE_sched_ctrl_t *sched_ctrl) {

   AssertFatal(CellGroup, "CellGroup is null\n");
   NR_MAC_CellGroupConfig_t   *mac_CellGroupConfig = CellGroup->mac_CellGroupConfig;


   if (mac_CellGroupConfig) {
     process_drx_Config(sched_ctrl,mac_CellGroupConfig->drx_Config);
     process_schedulingRequestConfig(sched_ctrl,mac_CellGroupConfig->schedulingRequestConfig);
     process_bsrConfig(sched_ctrl,mac_CellGroupConfig->bsr_Config);
     process_tag_Config(sched_ctrl,mac_CellGroupConfig->tag_Config);
     process_phr_Config(sched_ctrl,mac_CellGroupConfig->phr_Config);
   }
   else {
     // apply defaults

   }

   process_rlcBearerConfig(CellGroup->rlc_BearerToAddModList,CellGroup->rlc_BearerToReleaseList,sched_ctrl);

}

void config_common(int Mod_idP, rrc_pdsch_AntennaPorts_t pdsch_AntennaPorts, int pusch_AntennaPorts, NR_ServingCellConfigCommon_t *scc)
{
  nfapi_nr_config_request_scf_t *cfg = &RC.nrmac[Mod_idP]->config[0];
  RC.nrmac[Mod_idP]->common_channels[0].ServingCellConfigCommon = scc;

  // Carrier configuration

  cfg->carrier_config.dl_bandwidth.value = config_bandwidth(scc->downlinkConfigCommon->frequencyInfoDL->scs_SpecificCarrierList.list.array[0]->subcarrierSpacing,
                                                            scc->downlinkConfigCommon->frequencyInfoDL->scs_SpecificCarrierList.list.array[0]->carrierBandwidth,
                                                            *scc->downlinkConfigCommon->frequencyInfoDL->frequencyBandList.list.array[0]);
  cfg->carrier_config.dl_bandwidth.tl.tag   = NFAPI_NR_CONFIG_DL_BANDWIDTH_TAG; //temporary
  cfg->num_tlv++;
  LOG_I(NR_MAC,"%s() dl_BandwidthP:%d\n", __FUNCTION__, cfg->carrier_config.dl_bandwidth.value);

  cfg->carrier_config.dl_frequency.value = from_nrarfcn(*scc->downlinkConfigCommon->frequencyInfoDL->frequencyBandList.list.array[0],
                                                        *scc->ssbSubcarrierSpacing,
                                                        scc->downlinkConfigCommon->frequencyInfoDL->absoluteFrequencyPointA)/1000; // freq in kHz
  cfg->carrier_config.dl_frequency.tl.tag = NFAPI_NR_CONFIG_DL_FREQUENCY_TAG;
  cfg->num_tlv++;

  for (int i=0; i<5; i++) {
    if (i==scc->downlinkConfigCommon->frequencyInfoDL->scs_SpecificCarrierList.list.array[0]->subcarrierSpacing) {
      cfg->carrier_config.dl_grid_size[i].value = scc->downlinkConfigCommon->frequencyInfoDL->scs_SpecificCarrierList.list.array[0]->carrierBandwidth;
      cfg->carrier_config.dl_k0[i].value = scc->downlinkConfigCommon->frequencyInfoDL->scs_SpecificCarrierList.list.array[0]->offsetToCarrier;
      cfg->carrier_config.dl_grid_size[i].tl.tag = NFAPI_NR_CONFIG_DL_GRID_SIZE_TAG;
      cfg->carrier_config.dl_k0[i].tl.tag = NFAPI_NR_CONFIG_DL_K0_TAG;
      cfg->num_tlv++;
      cfg->num_tlv++;
    }
    else {
      cfg->carrier_config.dl_grid_size[i].value = 0;
      cfg->carrier_config.dl_k0[i].value = 0;
    }
  }

  cfg->carrier_config.uplink_bandwidth.value = config_bandwidth(scc->uplinkConfigCommon->frequencyInfoUL->scs_SpecificCarrierList.list.array[0]->subcarrierSpacing,
                                                                scc->uplinkConfigCommon->frequencyInfoUL->scs_SpecificCarrierList.list.array[0]->carrierBandwidth,
                                                                *scc->uplinkConfigCommon->frequencyInfoUL->frequencyBandList->list.array[0]);
  cfg->carrier_config.uplink_bandwidth.tl.tag   = NFAPI_NR_CONFIG_UPLINK_BANDWIDTH_TAG; //temporary
  cfg->num_tlv++;
  LOG_I(NR_MAC,"%s() dl_BandwidthP:%d\n", __FUNCTION__, cfg->carrier_config.uplink_bandwidth.value);

  int UL_pointA;
  if (scc->uplinkConfigCommon->frequencyInfoUL->absoluteFrequencyPointA == NULL)
    UL_pointA = scc->downlinkConfigCommon->frequencyInfoDL->absoluteFrequencyPointA;
  else
    UL_pointA = *scc->uplinkConfigCommon->frequencyInfoUL->absoluteFrequencyPointA; 

  cfg->carrier_config.uplink_frequency.value = from_nrarfcn(*scc->uplinkConfigCommon->frequencyInfoUL->frequencyBandList->list.array[0],
                                                            *scc->ssbSubcarrierSpacing,
                                                            UL_pointA)/1000; // freq in kHz
  cfg->carrier_config.uplink_frequency.tl.tag = NFAPI_NR_CONFIG_UPLINK_FREQUENCY_TAG;
  cfg->num_tlv++;

  for (int i=0; i<5; i++) {
    if (i==scc->uplinkConfigCommon->frequencyInfoUL->scs_SpecificCarrierList.list.array[0]->subcarrierSpacing) {
      cfg->carrier_config.ul_grid_size[i].value = scc->uplinkConfigCommon->frequencyInfoUL->scs_SpecificCarrierList.list.array[0]->carrierBandwidth;
      cfg->carrier_config.ul_k0[i].value = scc->uplinkConfigCommon->frequencyInfoUL->scs_SpecificCarrierList.list.array[0]->offsetToCarrier;
      cfg->carrier_config.ul_grid_size[i].tl.tag = NFAPI_NR_CONFIG_UL_GRID_SIZE_TAG;
      cfg->carrier_config.ul_k0[i].tl.tag = NFAPI_NR_CONFIG_UL_K0_TAG;
      cfg->num_tlv++;
      cfg->num_tlv++;
    }
    else {
      cfg->carrier_config.ul_grid_size[i].value = 0;
      cfg->carrier_config.ul_k0[i].value = 0;
    }
  }

  uint32_t band = *scc->downlinkConfigCommon->frequencyInfoDL->frequencyBandList.list.array[0];
  frequency_range_t frequency_range = band<100?FR1:FR2;

  frame_type_t frame_type = get_frame_type(*scc->downlinkConfigCommon->frequencyInfoDL->frequencyBandList.list.array[0], *scc->ssbSubcarrierSpacing);
  RC.nrmac[Mod_idP]->common_channels[0].frame_type = frame_type;

  // Cell configuration
  cfg->cell_config.phy_cell_id.value = *scc->physCellId;
  cfg->cell_config.phy_cell_id.tl.tag = NFAPI_NR_CONFIG_PHY_CELL_ID_TAG;
  cfg->num_tlv++;

  cfg->cell_config.frame_duplex_type.value = frame_type;
  cfg->cell_config.frame_duplex_type.tl.tag = NFAPI_NR_CONFIG_FRAME_DUPLEX_TYPE_TAG;
  cfg->num_tlv++;


  // SSB configuration
  cfg->ssb_config.ss_pbch_power.value = scc->ss_PBCH_BlockPower;
  cfg->ssb_config.ss_pbch_power.tl.tag = NFAPI_NR_CONFIG_SS_PBCH_POWER_TAG;
  cfg->num_tlv++;

  cfg->ssb_config.bch_payload.value = 1;
  cfg->ssb_config.bch_payload.tl.tag = NFAPI_NR_CONFIG_BCH_PAYLOAD_TAG;
  cfg->num_tlv++;

  cfg->ssb_config.scs_common.value = *scc->ssbSubcarrierSpacing;
  cfg->ssb_config.scs_common.tl.tag = NFAPI_NR_CONFIG_SCS_COMMON_TAG;
  cfg->num_tlv++;

  // PRACH configuration

  uint8_t nb_preambles = 64;
  if(scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->totalNumberOfRA_Preambles != NULL)
     nb_preambles = *scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->totalNumberOfRA_Preambles;

  cfg->prach_config.prach_sequence_length.value = scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->prach_RootSequenceIndex.present-1;
  cfg->prach_config.prach_sequence_length.tl.tag = NFAPI_NR_CONFIG_PRACH_SEQUENCE_LENGTH_TAG;
  cfg->num_tlv++;  

  if (scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->msg1_SubcarrierSpacing)
    cfg->prach_config.prach_sub_c_spacing.value = *scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->msg1_SubcarrierSpacing;
  else 
    cfg->prach_config.prach_sub_c_spacing.value = scc->downlinkConfigCommon->frequencyInfoDL->scs_SpecificCarrierList.list.array[0]->subcarrierSpacing;
  cfg->prach_config.prach_sub_c_spacing.tl.tag = NFAPI_NR_CONFIG_PRACH_SUB_C_SPACING_TAG;
  cfg->num_tlv++;
  cfg->prach_config.restricted_set_config.value = scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->restrictedSetConfig;
  cfg->prach_config.restricted_set_config.tl.tag = NFAPI_NR_CONFIG_RESTRICTED_SET_CONFIG_TAG;
  cfg->num_tlv++;
  cfg->prach_config.prach_ConfigurationIndex.value = scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->rach_ConfigGeneric.prach_ConfigurationIndex;
  cfg->prach_config.prach_ConfigurationIndex.tl.tag = NFAPI_NR_CONFIG_PRACH_CONFIG_INDEX_TAG;
  cfg->num_tlv++;

  switch (scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->rach_ConfigGeneric.msg1_FDM) {
    case 0 :
      cfg->prach_config.num_prach_fd_occasions.value = 1;
      break;
    case 1 :
      cfg->prach_config.num_prach_fd_occasions.value = 2;
      break;
    case 2 :
      cfg->prach_config.num_prach_fd_occasions.value = 4;
      break;
    case 3 :
      cfg->prach_config.num_prach_fd_occasions.value = 8;
      break;
    default:
      AssertFatal(1==0,"msg1 FDM identifier %ld undefined (0,1,2,3) \n", scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->rach_ConfigGeneric.msg1_FDM);
  } 
  cfg->prach_config.num_prach_fd_occasions.tl.tag = NFAPI_NR_CONFIG_NUM_PRACH_FD_OCCASIONS_TAG;
  cfg->num_tlv++;

  cfg->prach_config.prach_ConfigurationIndex.value =  scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->rach_ConfigGeneric.prach_ConfigurationIndex;
  cfg->prach_config.prach_ConfigurationIndex.tl.tag = NFAPI_NR_CONFIG_PRACH_CONFIG_INDEX_TAG;
  cfg->num_tlv++;

  cfg->prach_config.num_prach_fd_occasions_list = (nfapi_nr_num_prach_fd_occasions_t *) malloc(cfg->prach_config.num_prach_fd_occasions.value*sizeof(nfapi_nr_num_prach_fd_occasions_t));
  for (int i=0; i<cfg->prach_config.num_prach_fd_occasions.value; i++) {
//    cfg->prach_config.num_prach_fd_occasions_list[i].num_prach_fd_occasions = i;
    if (cfg->prach_config.prach_sequence_length.value)
      cfg->prach_config.num_prach_fd_occasions_list[i].prach_root_sequence_index.value = scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->prach_RootSequenceIndex.choice.l139; 
    else
      cfg->prach_config.num_prach_fd_occasions_list[i].prach_root_sequence_index.value = scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->prach_RootSequenceIndex.choice.l839;
    cfg->prach_config.num_prach_fd_occasions_list[i].prach_root_sequence_index.tl.tag = NFAPI_NR_CONFIG_PRACH_ROOT_SEQUENCE_INDEX_TAG;
    cfg->num_tlv++;
    cfg->prach_config.num_prach_fd_occasions_list[i].k1.value = NRRIV2PRBOFFSET(scc->uplinkConfigCommon->initialUplinkBWP->genericParameters.locationAndBandwidth, MAX_BWP_SIZE) + scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->rach_ConfigGeneric.msg1_FrequencyStart + (get_N_RA_RB( cfg->prach_config.prach_sub_c_spacing.value, scc->uplinkConfigCommon->frequencyInfoUL->scs_SpecificCarrierList.list.array[0]->subcarrierSpacing ) * i);
    if (get_softmodem_params()->sa) {
      cfg->prach_config.num_prach_fd_occasions_list[i].k1.value = NRRIV2PRBOFFSET(scc->uplinkConfigCommon->initialUplinkBWP->genericParameters.locationAndBandwidth, MAX_BWP_SIZE) + scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->rach_ConfigGeneric.msg1_FrequencyStart + (get_N_RA_RB( cfg->prach_config.prach_sub_c_spacing.value, scc->uplinkConfigCommon->frequencyInfoUL->scs_SpecificCarrierList.list.array[0]->subcarrierSpacing ) * i);
    } else {
      cfg->prach_config.num_prach_fd_occasions_list[i].k1.value = scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->rach_ConfigGeneric.msg1_FrequencyStart + (get_N_RA_RB( cfg->prach_config.prach_sub_c_spacing.value, scc->uplinkConfigCommon->frequencyInfoUL->scs_SpecificCarrierList.list.array[0]->subcarrierSpacing ) * i);
    }
    cfg->prach_config.num_prach_fd_occasions_list[i].k1.tl.tag = NFAPI_NR_CONFIG_K1_TAG;
    cfg->num_tlv++;
    cfg->prach_config.num_prach_fd_occasions_list[i].prach_zero_corr_conf.value = scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->rach_ConfigGeneric.zeroCorrelationZoneConfig;
    cfg->prach_config.num_prach_fd_occasions_list[i].prach_zero_corr_conf.tl.tag = NFAPI_NR_CONFIG_PRACH_ZERO_CORR_CONF_TAG;
    cfg->num_tlv++;
    cfg->prach_config.num_prach_fd_occasions_list[i].num_root_sequences.value = compute_nr_root_seq(scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup,nb_preambles, frame_type, frequency_range);
    cfg->prach_config.num_prach_fd_occasions_list[i].num_root_sequences.tl.tag = NFAPI_NR_CONFIG_NUM_ROOT_SEQUENCES_TAG;
    cfg->num_tlv++;
    cfg->prach_config.num_prach_fd_occasions_list[i].num_unused_root_sequences.value = 1;
  }

  cfg->prach_config.ssb_per_rach.value = scc->uplinkConfigCommon->initialUplinkBWP->rach_ConfigCommon->choice.setup->ssb_perRACH_OccasionAndCB_PreamblesPerSSB->present-1;
  cfg->prach_config.ssb_per_rach.tl.tag = NFAPI_NR_CONFIG_SSB_PER_RACH_TAG;
  cfg->num_tlv++;

  // SSB Table Configuration
  uint32_t absolute_diff = (*scc->downlinkConfigCommon->frequencyInfoDL->absoluteFrequencySSB - scc->downlinkConfigCommon->frequencyInfoDL->absoluteFrequencyPointA);
  const int scaling_5khz = scc->downlinkConfigCommon->frequencyInfoDL->absoluteFrequencyPointA < 600000 ? 3 : 1;
  int sco = (absolute_diff/scaling_5khz) % 24;
  if(frequency_range == FR2)
    sco >>= 1; // this assumes 120kHz SCS for SSB and subCarrierSpacingCommon (only option supported by OAI for
  const int scs_scaling = frequency_range == FR2 ? 1 << (*scc->ssbSubcarrierSpacing - 2) : 1 << *scc->ssbSubcarrierSpacing;
  cfg->ssb_table.ssb_offset_point_a.value = absolute_diff/(12*scaling_5khz) - 10*scs_scaling; //absoluteFrequencySSB is the central frequency of SSB which is made by 20RBs in total
  cfg->ssb_table.ssb_offset_point_a.tl.tag = NFAPI_NR_CONFIG_SSB_OFFSET_POINT_A_TAG;
  cfg->num_tlv++;
  cfg->ssb_table.ssb_period.value = *scc->ssb_periodicityServingCell;
  cfg->ssb_table.ssb_period.tl.tag = NFAPI_NR_CONFIG_SSB_PERIOD_TAG;
  cfg->num_tlv++;
  cfg->ssb_table.ssb_subcarrier_offset.value = sco;
  cfg->ssb_table.ssb_subcarrier_offset.tl.tag = NFAPI_NR_CONFIG_SSB_SUBCARRIER_OFFSET_TAG;
  cfg->num_tlv++;

  RC.nrmac[Mod_idP]->ssb_SubcarrierOffset = cfg->ssb_table.ssb_subcarrier_offset.value;
  RC.nrmac[Mod_idP]->ssb_OffsetPointA = cfg->ssb_table.ssb_offset_point_a.value;

  switch (scc->ssb_PositionsInBurst->present) {
    case 1 :
      cfg->ssb_table.ssb_mask_list[0].ssb_mask.value = scc->ssb_PositionsInBurst->choice.shortBitmap.buf[0]<<24;
      cfg->ssb_table.ssb_mask_list[1].ssb_mask.value = 0;
      break;
    case 2 :
      cfg->ssb_table.ssb_mask_list[0].ssb_mask.value = ((uint32_t) scc->ssb_PositionsInBurst->choice.mediumBitmap.buf[0]) << 24;
      cfg->ssb_table.ssb_mask_list[1].ssb_mask.value = 0;
      break;
    case 3 :
      cfg->ssb_table.ssb_mask_list[0].ssb_mask.value = 0;
      cfg->ssb_table.ssb_mask_list[1].ssb_mask.value = 0;
      for (int i=0; i<4; i++) {
        cfg->ssb_table.ssb_mask_list[0].ssb_mask.value += (scc->ssb_PositionsInBurst->choice.longBitmap.buf[3-i]<<i*8);
        cfg->ssb_table.ssb_mask_list[1].ssb_mask.value += (scc->ssb_PositionsInBurst->choice.longBitmap.buf[7-i]<<i*8);
      }
      break;
    default:
      AssertFatal(1==0,"SSB bitmap size value %d undefined (allowed values 1,2,3) \n", scc->ssb_PositionsInBurst->present);
  }

  cfg->ssb_table.ssb_mask_list[0].ssb_mask.tl.tag = NFAPI_NR_CONFIG_SSB_MASK_TAG;
  cfg->ssb_table.ssb_mask_list[1].ssb_mask.tl.tag = NFAPI_NR_CONFIG_SSB_MASK_TAG;
  cfg->num_tlv+=2;

  // logical antenna ports
  int num_pdsch_antenna_ports = pdsch_AntennaPorts.N1 * pdsch_AntennaPorts.N2 * pdsch_AntennaPorts.XP;
  cfg->carrier_config.num_tx_ant.value = num_pdsch_antenna_ports;
  AssertFatal(num_pdsch_antenna_ports > 0 && num_pdsch_antenna_ports < 33, "pdsch_AntennaPorts in 1...32\n");
  cfg->carrier_config.num_tx_ant.tl.tag = NFAPI_NR_CONFIG_NUM_TX_ANT_TAG;

  int num_ssb=0;
  for (int i=0;i<32;i++) {
    cfg->ssb_table.ssb_beam_id_list[i].beam_id.tl.tag = NFAPI_NR_CONFIG_BEAM_ID_TAG;
    if ((cfg->ssb_table.ssb_mask_list[0].ssb_mask.value>>(31-i))&1) {
      cfg->ssb_table.ssb_beam_id_list[i].beam_id.value = num_ssb;
      num_ssb++;
    }
    cfg->num_tlv++;
  }
  for (int i=0;i<32;i++) {
    cfg->ssb_table.ssb_beam_id_list[32+i].beam_id.tl.tag = NFAPI_NR_CONFIG_BEAM_ID_TAG;
    if ((cfg->ssb_table.ssb_mask_list[1].ssb_mask.value>>(31-i))&1) {
      cfg->ssb_table.ssb_beam_id_list[32+i].beam_id.value = num_ssb;
      num_ssb++;
    }
    cfg->num_tlv++;
  } 

  cfg->carrier_config.num_rx_ant.value = pusch_AntennaPorts;
  AssertFatal(pusch_AntennaPorts > 0 && pusch_AntennaPorts < 13, "pusch_AntennaPorts in 1...12\n");
  cfg->carrier_config.num_rx_ant.tl.tag = NFAPI_NR_CONFIG_NUM_RX_ANT_TAG;
  LOG_I(NR_MAC,"Set RX antenna number to %d, Set TX antenna number to %d (num ssb %d: %x,%x)\n",
        cfg->carrier_config.num_tx_ant.value,cfg->carrier_config.num_rx_ant.value,num_ssb,cfg->ssb_table.ssb_mask_list[0].ssb_mask.value,cfg->ssb_table.ssb_mask_list[1].ssb_mask.value);
  AssertFatal(cfg->carrier_config.num_tx_ant.value > 0,"carrier_config.num_tx_ant.value %d !\n",cfg->carrier_config.num_tx_ant.value );
  cfg->num_tlv++;
  cfg->num_tlv++;

  // TDD Table Configuration
  if (cfg->cell_config.frame_duplex_type.value == TDD){
    cfg->tdd_table.tdd_period.tl.tag = NFAPI_NR_CONFIG_TDD_PERIOD_TAG;
    cfg->num_tlv++;
    if (scc->tdd_UL_DL_ConfigurationCommon->pattern1.ext1 == NULL) {
      cfg->tdd_table.tdd_period.value = scc->tdd_UL_DL_ConfigurationCommon->pattern1.dl_UL_TransmissionPeriodicity;
    } else {
      AssertFatal(scc->tdd_UL_DL_ConfigurationCommon->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530 != NULL,
                  "In %s: scc->tdd_UL_DL_ConfigurationCommon->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530 is null\n", __FUNCTION__);
      cfg->tdd_table.tdd_period.value = *scc->tdd_UL_DL_ConfigurationCommon->pattern1.ext1->dl_UL_TransmissionPeriodicity_v1530;
    }
    LOG_I(NR_MAC, "Setting TDD configuration period to %d\n", cfg->tdd_table.tdd_period.value);
    int periods_per_frame = set_tdd_config_nr(cfg,
                                              scc->uplinkConfigCommon->frequencyInfoUL->scs_SpecificCarrierList.list.array[0]->subcarrierSpacing,
                                              scc->tdd_UL_DL_ConfigurationCommon->pattern1.nrofDownlinkSlots,
                                              scc->tdd_UL_DL_ConfigurationCommon->pattern1.nrofDownlinkSymbols,
                                              scc->tdd_UL_DL_ConfigurationCommon->pattern1.nrofUplinkSlots,
                                              scc->tdd_UL_DL_ConfigurationCommon->pattern1.nrofUplinkSymbols);

    if (periods_per_frame < 0)
      LOG_E(NR_MAC,"TDD configuration can not be done\n");
    else {
      LOG_I(NR_MAC,"TDD has been properly configurated\n");
      RC.nrmac[Mod_idP]->tdd_beam_association = (int16_t *)malloc16(periods_per_frame*sizeof(int16_t));
    }
  }

  // precoding matrix configuration (to be improved)
  cfg->pmi_list = init_DL_MIMO_codebook(RC.nrmac[Mod_idP], pdsch_AntennaPorts);
}

int nr_mac_enable_ue_rrc_processing_timer(module_id_t Mod_idP, rnti_t rnti, NR_SubcarrierSpacing_t subcarrierSpacing, uint32_t rrc_reconfiguration_delay) {

  if (rrc_reconfiguration_delay == 0) {
    return -1;
  }

  NR_UE_info_t *UE_info = find_nr_UE(&RC.nrmac[Mod_idP]->UE_info,rnti);
  if (!UE_info) {
    LOG_W(NR_MAC, "Could not find UE for RNTI 0x%04x\n", rnti);
    return -1;
  }
  NR_UE_sched_ctrl_t *sched_ctrl = &UE_info->UE_sched_ctrl;
  const uint16_t sl_ahead = RC.nrmac[Mod_idP]->if_inst->sl_ahead;
  sched_ctrl->rrc_processing_timer = (rrc_reconfiguration_delay<<subcarrierSpacing) + sl_ahead;
  LOG_I(NR_MAC, "Activating RRC processing timer for UE %04x with %d ms\n", UE_info->rnti, rrc_reconfiguration_delay);

  return 0;
}

int rrc_mac_config_req_gNB(module_id_t Mod_idP,
                           rrc_pdsch_AntennaPorts_t pdsch_AntennaPorts,
                           int pusch_AntennaPorts,
                           int sib1_tda,
                           int minRXTXTIMEpdsch,
                           NR_ServingCellConfigCommon_t *scc,
                           NR_BCCH_BCH_Message_t *mib,
                           NR_BCCH_DL_SCH_Message_t *sib1,
                           int add_ue,
                           uint32_t rnti,
                           NR_CellGroupConfig_t *CellGroup) {

  if (scc != NULL ) {
    AssertFatal((scc->ssb_PositionsInBurst->present > 0) && (scc->ssb_PositionsInBurst->present < 4), "SSB Bitmap type %d is not valid\n",scc->ssb_PositionsInBurst->present);

    int n = nr_slots_per_frame[*scc->ssbSubcarrierSpacing];
    if (*scc->ssbSubcarrierSpacing == 0)
      n <<= 1; // to have enough room for feedback possibly beyond the frame we need a larger array at 15kHz SCS
    RC.nrmac[Mod_idP]->common_channels[0].vrb_map_UL = calloc(n * MAX_BWP_SIZE, sizeof(uint16_t));
    RC.nrmac[Mod_idP]->vrb_map_UL_size = n;
    AssertFatal(RC.nrmac[Mod_idP]->common_channels[0].vrb_map_UL,
                "could not allocate memory for RC.nrmac[]->common_channels[0].vrb_map_UL\n");

    LOG_I(NR_MAC,"Configuring common parameters from NR ServingCellConfig\n");

    RC.nrmac[Mod_idP]->xp_pdsch_antenna_ports = pdsch_AntennaPorts.XP;
    config_common(Mod_idP,
                  pdsch_AntennaPorts,
                  pusch_AntennaPorts,
		  scc);
    LOG_D(NR_MAC, "%s() %s:%d RC.nrmac[Mod_idP]->if_inst->NR_PHY_config_req:%p\n", __FUNCTION__, __FILE__, __LINE__, RC.nrmac[Mod_idP]->if_inst->NR_PHY_config_req);
  
    if (NFAPI_MODE == NFAPI_MODE_PNF || NFAPI_MODE == NFAPI_MODE_VNF) {
      // fake that the gNB is configured in nFAPI mode, which would normally be
      // done in a NR_PHY_config_req, but in this mode, there is no PHY
      RC.gNB[Mod_idP]->configured = 1;
    } else {
      NR_PHY_Config_t phycfg = {
        .Mod_id = Mod_idP,
        .CC_id  = 0,
        .cfg    = &RC.nrmac[Mod_idP]->config[0]
      };
      DevAssert(RC.nrmac[Mod_idP]->if_inst->NR_PHY_config_req);
      RC.nrmac[Mod_idP]->if_inst->NR_PHY_config_req(&phycfg);
    }

    RC.nrmac[Mod_idP]->minRXTXTIMEpdsch = minRXTXTIMEpdsch;
    find_SSB_and_RO_available(Mod_idP);

    const NR_TDD_UL_DL_Pattern_t *tdd = scc->tdd_UL_DL_ConfigurationCommon ? &scc->tdd_UL_DL_ConfigurationCommon->pattern1 : NULL;

    int nr_slots_period = n;
    int nr_dl_slots = n;
    int nr_ulstart_slot = 0;
    if (tdd) {
      nr_dl_slots = tdd->nrofDownlinkSlots + (tdd->nrofDownlinkSymbols != 0);
      nr_ulstart_slot = get_first_ul_slot(tdd->nrofDownlinkSlots, tdd->nrofDownlinkSymbols, tdd->nrofUplinkSymbols);
      nr_slots_period /= get_nb_periods_per_frame(tdd->dl_UL_TransmissionPeriodicity);
    }
    else
      // if TDD configuration is not present and the band is not FDD, it means it is a dynamic TDD configuration
      AssertFatal(RC.nrmac[Mod_idP]->common_channels[0].frame_type == FDD,"Dynamic TDD not handled yet\n");

    for (int slot = 0; slot < n; ++slot) {
      RC.nrmac[Mod_idP]->dlsch_slot_bitmap[slot / 64] |= (uint64_t)((slot % nr_slots_period) < nr_dl_slots) << (slot % 64);
      RC.nrmac[Mod_idP]->ulsch_slot_bitmap[slot / 64] |= (uint64_t)((slot % nr_slots_period) >= nr_ulstart_slot) << (slot % 64);

      LOG_I(NR_MAC, "In %s: slot %d DL %d UL %d\n",
            __FUNCTION__,
            slot,
            (RC.nrmac[Mod_idP]->dlsch_slot_bitmap[slot / 64] & ((uint64_t)1 << (slot % 64))) != 0,
            (RC.nrmac[Mod_idP]->ulsch_slot_bitmap[slot / 64] & ((uint64_t)1 << (slot % 64))) != 0);
    }

    if (get_softmodem_params()->phy_test) {
      RC.nrmac[Mod_idP]->pre_processor_dl = nr_preprocessor_phytest;
      RC.nrmac[Mod_idP]->pre_processor_ul = nr_ul_preprocessor_phytest;
    } else {
      RC.nrmac[Mod_idP]->pre_processor_dl = nr_init_fr1_dlsch_preprocessor(Mod_idP, 0);
      RC.nrmac[Mod_idP]->pre_processor_ul = nr_init_fr1_ulsch_preprocessor(Mod_idP, 0);
    }

    if (get_softmodem_params()->sa > 0) {
      NR_COMMON_channels_t *cc = &RC.nrmac[Mod_idP]->common_channels[0];
      RC.nrmac[Mod_idP]->sib1_tda = sib1_tda;
      for (int n=0;n<NR_NB_RA_PROC_MAX;n++ ) {
        cc->ra[n].cfra = false;
        cc->ra[n].msg3_dcch_dtch = false;
        cc->ra[n].rnti = 0;
        cc->ra[n].preambles.num_preambles = MAX_NUM_NR_PRACH_PREAMBLES;
        cc->ra[n].preambles.preamble_list = (uint8_t *) malloc(MAX_NUM_NR_PRACH_PREAMBLES*sizeof(uint8_t));
        for (int i = 0; i < MAX_NUM_NR_PRACH_PREAMBLES; i++)
          cc->ra[n].preambles.preamble_list[i] = i;
      }
    }
  }
 
  if (mib) RC.nrmac[Mod_idP]->common_channels[0].mib = mib;
  if (sib1) RC.nrmac[Mod_idP]->common_channels[0].sib1 = sib1;

  if (CellGroup) {

    if (add_ue == 1 && get_softmodem_params()->phy_test) {
      NR_UE_info_t* UE = add_new_nr_ue(RC.nrmac[Mod_idP], rnti, CellGroup);
      if (UE) {
        LOG_I(NR_MAC,"Added new UE %x with initial CellGroup\n", rnti);
      } else {
        LOG_E(NR_MAC,"Error adding UE %04x\n", rnti);
        return -1;
      }
      process_CellGroup(CellGroup,&UE->UE_sched_ctrl);
    } else if (add_ue == 1 && !get_softmodem_params()->phy_test) {
      const int CC_id = 0;
      NR_COMMON_channels_t *cc = &RC.nrmac[Mod_idP]->common_channels[CC_id];
      uint8_t ra_index = 0;
      /* checking for free RA process */
      for(; ra_index < NR_NB_RA_PROC_MAX; ra_index++) {
        if((cc->ra[ra_index].state == RA_IDLE) && (!cc->ra[ra_index].cfra)) break;
      }
      if (ra_index == NR_NB_RA_PROC_MAX) {
        LOG_E(NR_MAC, "%s() %s:%d RA processes are not available for CFRA RNTI :%x\n", __FUNCTION__, __FILE__, __LINE__, rnti);
        return -1;
      }	
      NR_RA_t *ra = &cc->ra[ra_index];
      ra->CellGroup = CellGroup;
      if (CellGroup->spCellConfig && CellGroup->spCellConfig->reconfigurationWithSync &&
	        CellGroup->spCellConfig->reconfigurationWithSync->rach_ConfigDedicated!=NULL) {
        if (CellGroup->spCellConfig->reconfigurationWithSync->rach_ConfigDedicated->choice.uplink->cfra != NULL) {
          ra->cfra = true;
          ra->rnti = rnti;
          struct NR_CFRA *cfra = CellGroup->spCellConfig->reconfigurationWithSync->rach_ConfigDedicated->choice.uplink->cfra;
          uint8_t num_preamble = cfra->resources.choice.ssb->ssb_ResourceList.list.count;
          ra->preambles.num_preambles = num_preamble;
          ra->preambles.preamble_list = (uint8_t *) malloc(num_preamble*sizeof(uint8_t));
          for(int i=0; i<cc->num_active_ssb; i++) {
            for(int j=0; j<num_preamble; j++) {
              if (cc->ssb_index[i] == cfra->resources.choice.ssb->ssb_ResourceList.list.array[j]->ssb) {
                // one dedicated preamble for each beam
                ra->preambles.preamble_list[i] =
                    cfra->resources.choice.ssb->ssb_ResourceList.list.array[j]->ra_PreambleIndex;
                break;
              }
            }
          }
        }
      } else {
        ra->cfra = false;
        ra->rnti = 0;
        ra->preambles.num_preambles = MAX_NUM_NR_PRACH_PREAMBLES;
        ra->preambles.preamble_list = (uint8_t *) malloc(MAX_NUM_NR_PRACH_PREAMBLES*sizeof(uint8_t));
        for (int i = 0; i < MAX_NUM_NR_PRACH_PREAMBLES; i++)
          ra->preambles.preamble_list[i] = i;
      }
      ra->msg3_dcch_dtch = false;
      LOG_I(NR_MAC,"Added new RA process for UE RNTI %04x with initial CellGroup\n", rnti);
    } else { // CellGroup has been updated
      NR_UE_info_t *UE = find_nr_UE(&RC.nrmac[Mod_idP]->UE_info, rnti);
      if (!UE) {
        LOG_E(NR_MAC, "Can't find UE %04x\n", rnti);
        return -1;
      }

      /* copy CellGroup by calling asn1c encode
         this is a temporary hack to avoid the gNB having
         a pointer to RRC CellGroup structure
         (otherwise it would be applied to early)
         TODO remove once we have a proper implementation */
      UE->enc_rval = uper_encode_to_buffer(&asn_DEF_NR_CellGroupConfig,
                                           NULL,
                                           (void *) CellGroup,
                                           UE->cg_buf,
                                           32768);

      if (UE->enc_rval.encoded == -1) {
        LOG_E(NR_MAC, "ASN1 message CellGroupConfig encoding failed (%s, %lu)!\n",
              UE->enc_rval.failed_type->name, UE->enc_rval.encoded);
        exit(1);
      }

      process_CellGroup(CellGroup,&UE->UE_sched_ctrl);
    }
  }
  VCD_SIGNAL_DUMPER_DUMP_FUNCTION_BY_NAME(VCD_SIGNAL_DUMPER_FUNCTIONS_RRC_MAC_CONFIG, VCD_FUNCTION_OUT);

  return 0;
}// END rrc_mac_config_req_gNB
