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

/*! \file PHY/NR_TRANSPORT/nr_ulsch_llr_computation.c
 * \brief Top-level routines for LLR computation of the PDSCH physical channel
 * \author Ahmed Hussein
 * \date 2019
 * \version 0.1
 * \company Fraunhofer IIS
 * \email: ahmed.hussein@iis.fraunhofer.de
 * \note
 * \warning
 */

#include "PHY/defs_nr_common.h"
#include "PHY/sse_intrin.h"
#include "PHY/impl_defs_top.h"



//----------------------------------------------------------------------------------------------
// QPSK
//----------------------------------------------------------------------------------------------
void nr_ulsch_qpsk_llr(int32_t *rxdataF_comp,
                      int16_t  *ulsch_llr,
                      uint32_t nb_re,
                      uint8_t  symbol)
{
  c16_t *rxF   = (c16_t *)rxdataF_comp;
  c16_t *llr32 = (c16_t *)ulsch_llr;

  if (!llr32) {
    LOG_E(PHY,"nr_ulsch_qpsk_llr: llr is null, symbol %d, llr32 = %p\n",symbol, llr32);
  }
  for (int i = 0; i < nb_re; i++) {
    //*llr32 = *rxF;
    llr32->r = rxF->r >> 3;
    llr32->i = rxF->i >> 3;
    rxF++;
    llr32++;
  }
}

//----------------------------------------------------------------------------------------------
// 16-QAM
//----------------------------------------------------------------------------------------------

void nr_ulsch_16qam_llr(int32_t *rxdataF_comp,
                        int32_t *ul_ch_mag,
                        int16_t  *ulsch_llr,
                        uint32_t nb_rb,
                        uint32_t nb_re,
                        uint8_t  symbol)
{

#if defined(__x86_64__) || defined(__i386__)
  __m256i *rxF = (__m256i*)rxdataF_comp;
  __m256i *ch_mag;
  __m256i llr256[2];
  register __m256i xmm0;
  uint32_t *llr32;
#elif defined(__arm__) || defined(__aarch64__)
  int16x8_t *rxF = (int16x8_t*)&rxdataF_comp;
  int16x8_t *ch_mag;
  int16x8_t xmm0;
  int16_t *llr16;
#endif


  int i;

  int off = ((nb_rb&1) == 1)? 4:0;

#if defined(__x86_64__) || defined(__i386__)
    llr32 = (uint32_t*)ulsch_llr;
#elif defined(__arm__) || defined(__aarch64__)
    llr16 = (int16_t*)ulsch_llr;
#endif

#if defined(__x86_64__) || defined(__i386__)
    ch_mag = (__m256i*)&ul_ch_mag[(symbol*(off+(nb_rb*12)))];
#elif defined(__arm__) || defined(__aarch64__)
  ch_mag = (int16x8_t*)&ul_ch_mag[(symbol*nb_rb*12)];
#endif
  unsigned char len_mod8 = nb_re&7;
  nb_re >>= 3;  // length in quad words (4 REs)
  nb_re += (len_mod8 == 0 ? 0 : 1);

  for (i=0; i<nb_re; i++) {
#if defined(__x86_64__) || defined(__i386)
    xmm0 = simde_mm256_abs_epi16(rxF[i]); // registers of even index in xmm0-> |y_R|, registers of odd index in xmm0-> |y_I|
    xmm0 = simde_mm256_subs_epi16(ch_mag[i],xmm0); // registers of even index in xmm0-> |y_R|-|h|^2, registers of odd index in xmm0-> |y_I|-|h|^2
 
    llr256[0] = simde_mm256_unpacklo_epi32(rxF[i],xmm0); // llr128[0] contains the llrs of the 1st,2nd,5th and 6th REs
    llr256[1] = simde_mm256_unpackhi_epi32(rxF[i],xmm0); // llr128[1] contains the llrs of the 3rd, 4th, 7th and 8th REs
    
    // 1st RE
    llr32[0] = simde_mm256_extract_epi32(llr256[0],0); // llr32[0] low 16 bits-> y_R        , high 16 bits-> y_I
    llr32[1] = simde_mm256_extract_epi32(llr256[0],1); // llr32[1] low 16 bits-> |h|-|y_R|^2, high 16 bits-> |h|-|y_I|^2

    // 2nd RE
    llr32[2] = simde_mm256_extract_epi32(llr256[0],2); // llr32[2] low 16 bits-> y_R        , high 16 bits-> y_I
    llr32[3] = simde_mm256_extract_epi32(llr256[0],3); // llr32[3] low 16 bits-> |h|-|y_R|^2, high 16 bits-> |h|-|y_I|^2

    // 3rd RE
    llr32[4] = simde_mm256_extract_epi32(llr256[1],0); // llr32[4] low 16 bits-> y_R        , high 16 bits-> y_I
    llr32[5] = simde_mm256_extract_epi32(llr256[1],1); // llr32[5] low 16 bits-> |h|-|y_R|^2, high 16 bits-> |h|-|y_I|^2

    // 4th RE
    llr32[6] = simde_mm256_extract_epi32(llr256[1],2); // llr32[6] low 16 bits-> y_R        , high 16 bits-> y_I
    llr32[7] = simde_mm256_extract_epi32(llr256[1],3); // llr32[7] low 16 bits-> |h|-|y_R|^2, high 16 bits-> |h|-|y_I|^2

    // 5th RE
    llr32[8] = simde_mm256_extract_epi32(llr256[0],4); // llr32[8] low 16 bits-> y_R        , high 16 bits-> y_I
    llr32[9] = simde_mm256_extract_epi32(llr256[0],5); // llr32[9] low 16 bits-> |h|-|y_R|^2, high 16 bits-> |h|-|y_I|^2

    // 6th RE
    llr32[10] = simde_mm256_extract_epi32(llr256[0],6); // llr32[10] low 16 bits-> y_R        , high 16 bits-> y_I
    llr32[11] = simde_mm256_extract_epi32(llr256[0],7); // llr32[11] low 16 bits-> |h|-|y_R|^2, high 16 bits-> |h|-|y_I|^2

    // 7th RE
    llr32[12] = simde_mm256_extract_epi32(llr256[1],4); // llr32[12] low 16 bits-> y_R        , high 16 bits-> y_I
    llr32[13] = simde_mm256_extract_epi32(llr256[1],5); // llr32[13] low 16 bits-> |h|-|y_R|^2, high 16 bits-> |h|-|y_I|^2

    // 8th RE
    llr32[14] = simde_mm256_extract_epi32(llr256[1],6); // llr32[14] low 16 bits-> y_R        , high 16 bits-> y_I
    llr32[15] = simde_mm256_extract_epi32(llr256[1],7); // llr32[15] low 16 bits-> |h|-|y_R|^2, high 16 bits-> |h|-|y_I|^2

    llr32+=16;
#elif defined(__arm__) || defined(__aarch64__)
    xmm0 = vabsq_s16(rxF[i]);
    xmm0 = vqsubq_s16((*(__m128i*)&ones[0]),xmm0);

    llr16[0]  = vgetq_lane_s16(rxF[i],0);
    llr16[1]  = vgetq_lane_s16(rxF[i],1);
    llr16[2]  = vgetq_lane_s16(xmm0,0);
    llr16[3]  = vgetq_lane_s16(xmm0,1);
    llr16[4]  = vgetq_lane_s16(rxF[i],2);
    llr16[5]  = vgetq_lane_s16(rxF[i],3);
    llr16[6]  = vgetq_lane_s16(xmm0,2);
    llr16[7]  = vgetq_lane_s16(xmm0,3);
    llr16[8]  = vgetq_lane_s16(rxF[i],4);
    llr16[9]  = vgetq_lane_s16(rxF[i],5);
    llr16[10] = vgetq_lane_s16(xmm0,4);
    llr16[11] = vgetq_lane_s16(xmm0,5);
    llr16[12] = vgetq_lane_s16(rxF[i],6);
    llr16[13] = vgetq_lane_s16(rxF[i],6);
    llr16[14] = vgetq_lane_s16(xmm0,7);
    llr16[15] = vgetq_lane_s16(xmm0,7);
    llr16+=16;
#endif

  }

#if defined(__x86_64__) || defined(__i386__)
  _mm_empty();
  _m_empty();
#endif
}

//----------------------------------------------------------------------------------------------
// 64-QAM
//----------------------------------------------------------------------------------------------

void nr_ulsch_64qam_llr(int32_t *rxdataF_comp,
                        int32_t *ul_ch_mag,
                        int32_t *ul_ch_magb,
                        int16_t  *ulsch_llr,
                        uint32_t nb_rb,
                        uint32_t nb_re,
                        uint8_t  symbol)
{
  int off = ((nb_rb&1) == 1)? 4:0;

#if defined(__x86_64__) || defined(__i386__)
  __m256i *rxF = (__m256i*)rxdataF_comp;
  __m256i *ch_mag,*ch_magb;
  register __m256i xmm0,xmm1,xmm2;
#elif defined(__arm__) || defined(__aarch64__)
  int16x8_t *rxF = (int16x8_t*)&rxdataF_comp;
  int16x8_t *ch_mag,*ch_magb; // [hna] This should be uncommented once channel estimation is implemented
  int16x8_t xmm0,xmm1,xmm2;
#endif

  int i;

#if defined(__x86_64__) || defined(__i386__)
  ch_mag = (__m256i*)&ul_ch_mag[(symbol*(off+(nb_rb*12)))];
  ch_magb = (__m256i*)&ul_ch_magb[(symbol*(off+(nb_rb*12)))];
#elif defined(__arm__) || defined(__aarch64__)
  ch_mag = (int16x8_t*)&ul_ch_mag[(symbol*nb_rb*12)];
  ch_magb = (int16x8_t*)&ul_ch_magb[(symbol*nb_rb*12)];
#endif

  int len_mod8 = nb_re&7;
  nb_re    = nb_re>>3;  // length in quad words (4 REs)
  nb_re   += ((len_mod8 == 0) ? 0 : 1);

  for (i=0; i<nb_re; i++) {
    xmm0 = rxF[i];
#if defined(__x86_64__) || defined(__i386__)
    xmm1 = simde_mm256_abs_epi16(xmm0);
    xmm1 = simde_mm256_subs_epi16(ch_mag[i],xmm1);
    xmm2 = simde_mm256_abs_epi16(xmm1);
    xmm2 = simde_mm256_subs_epi16(ch_magb[i],xmm2);
#elif defined(__arm__) || defined(__aarch64__)
    xmm1 = vabsq_s16(xmm0);
    xmm1 = vsubq_s16(ch_mag[i],xmm1);
    xmm2 = vabsq_s16(xmm1);
    xmm2 = vsubq_s16(ch_magb[i],xmm2);
#endif
    
    // ---------------------------------------
    // 1st RE
    // ---------------------------------------
#if defined(__x86_64__) || defined(__i386__)
    ulsch_llr[0] = simde_mm256_extract_epi16(xmm0,0);
    ulsch_llr[1] = simde_mm256_extract_epi16(xmm0,1);
    ulsch_llr[2] = simde_mm256_extract_epi16(xmm1,0);
    ulsch_llr[3] = simde_mm256_extract_epi16(xmm1,1);
    ulsch_llr[4] = simde_mm256_extract_epi16(xmm2,0);
    ulsch_llr[5] = simde_mm256_extract_epi16(xmm2,1);
#elif defined(__arm__) || defined(__aarch64__)
    ulsch_llr[0] = vgetq_lane_s16(xmm0,0);
    ulsch_llr[1] = vgetq_lane_s16(xmm0,1);
    ulsch_llr[2] = vgetq_lane_s16(xmm1,0);
    ulsch_llr[3] = vgetq_lane_s16(xmm1,1);
    ulsch_llr[4] = vgetq_lane_s16(xmm2,0);
    ulsch_llr[5] = vgetq_lane_s16(xmm2,1);
#endif
    // ---------------------------------------

    ulsch_llr+=6;
    
    // ---------------------------------------
    // 2nd RE
    // ---------------------------------------
#if defined(__x86_64__) || defined(__i386__)
    ulsch_llr[0] = simde_mm256_extract_epi16(xmm0,2);
    ulsch_llr[1] = simde_mm256_extract_epi16(xmm0,3);
    ulsch_llr[2] = simde_mm256_extract_epi16(xmm1,2);
    ulsch_llr[3] = simde_mm256_extract_epi16(xmm1,3);
    ulsch_llr[4] = simde_mm256_extract_epi16(xmm2,2);
    ulsch_llr[5] = simde_mm256_extract_epi16(xmm2,3);
#elif defined(__arm__) || defined(__aarch64__)
    ulsch_llr[2] = vgetq_lane_s16(xmm0,2);
    ulsch_llr[3] = vgetq_lane_s16(xmm0,3);
    ulsch_llr[2] = vgetq_lane_s16(xmm1,2);
    ulsch_llr[3] = vgetq_lane_s16(xmm1,3);
    ulsch_llr[4] = vgetq_lane_s16(xmm2,2);
    ulsch_llr[5] = vgetq_lane_s16(xmm2,3);
#endif
    // ---------------------------------------

    ulsch_llr+=6;
    
    // ---------------------------------------
    // 3rd RE
    // ---------------------------------------
#if defined(__x86_64__) || defined(__i386__)
    ulsch_llr[0] = simde_mm256_extract_epi16(xmm0,4);
    ulsch_llr[1] = simde_mm256_extract_epi16(xmm0,5);
    ulsch_llr[2] = simde_mm256_extract_epi16(xmm1,4);
    ulsch_llr[3] = simde_mm256_extract_epi16(xmm1,5);
    ulsch_llr[4] = simde_mm256_extract_epi16(xmm2,4);
    ulsch_llr[5] = simde_mm256_extract_epi16(xmm2,5);
#elif defined(__arm__) || defined(__aarch64__)
    ulsch_llr[0] = vgetq_lane_s16(xmm0,4);
    ulsch_llr[1] = vgetq_lane_s16(xmm0,5);
    ulsch_llr[2] = vgetq_lane_s16(xmm1,4);
    ulsch_llr[3] = vgetq_lane_s16(xmm1,5);
    ulsch_llr[4] = vgetq_lane_s16(xmm2,4);
    ulsch_llr[5] = vgetq_lane_s16(xmm2,5);
#endif
    // ---------------------------------------

    ulsch_llr+=6;
    
    // ---------------------------------------
    // 4th RE
    // ---------------------------------------
#if defined(__x86_64__) || defined(__i386__)
    ulsch_llr[0] = simde_mm256_extract_epi16(xmm0,6);
    ulsch_llr[1] = simde_mm256_extract_epi16(xmm0,7);
    ulsch_llr[2] = simde_mm256_extract_epi16(xmm1,6);
    ulsch_llr[3] = simde_mm256_extract_epi16(xmm1,7);
    ulsch_llr[4] = simde_mm256_extract_epi16(xmm2,6);
    ulsch_llr[5] = simde_mm256_extract_epi16(xmm2,7);
#elif defined(__arm__) || defined(__aarch64__)
    ulsch_llr[0] = vgetq_lane_s16(xmm0,6);
    ulsch_llr[1] = vgetq_lane_s16(xmm0,7);
    ulsch_llr[2] = vgetq_lane_s16(xmm1,6);
    ulsch_llr[3] = vgetq_lane_s16(xmm1,7);
    ulsch_llr[4] = vgetq_lane_s16(xmm2,6);
    ulsch_llr[5] = vgetq_lane_s16(xmm2,7);
#endif
    // ---------------------------------------

    ulsch_llr+=6;
    ulsch_llr[0] = simde_mm256_extract_epi16(xmm0,8);
    ulsch_llr[1] = simde_mm256_extract_epi16(xmm0,9);
    ulsch_llr[2] = simde_mm256_extract_epi16(xmm1,8);
    ulsch_llr[3] = simde_mm256_extract_epi16(xmm1,9);
    ulsch_llr[4] = simde_mm256_extract_epi16(xmm2,8);
    ulsch_llr[5] = simde_mm256_extract_epi16(xmm2,9);

    ulsch_llr[6] = simde_mm256_extract_epi16(xmm0,10);
    ulsch_llr[7] = simde_mm256_extract_epi16(xmm0,11);
    ulsch_llr[8] = simde_mm256_extract_epi16(xmm1,10);
    ulsch_llr[9] = simde_mm256_extract_epi16(xmm1,11);
    ulsch_llr[10] = simde_mm256_extract_epi16(xmm2,10);
    ulsch_llr[11] = simde_mm256_extract_epi16(xmm2,11);

    ulsch_llr[12] = simde_mm256_extract_epi16(xmm0,12);
    ulsch_llr[13] = simde_mm256_extract_epi16(xmm0,13);
    ulsch_llr[14] = simde_mm256_extract_epi16(xmm1,12);
    ulsch_llr[15] = simde_mm256_extract_epi16(xmm1,13);
    ulsch_llr[16] = simde_mm256_extract_epi16(xmm2,12);
    ulsch_llr[17] = simde_mm256_extract_epi16(xmm2,13);

    ulsch_llr[18] = simde_mm256_extract_epi16(xmm0,14);
    ulsch_llr[19] = simde_mm256_extract_epi16(xmm0,15);
    ulsch_llr[20] = simde_mm256_extract_epi16(xmm1,14);
    ulsch_llr[21] = simde_mm256_extract_epi16(xmm1,15);
    ulsch_llr[22] = simde_mm256_extract_epi16(xmm2,14);
    ulsch_llr[23] = simde_mm256_extract_epi16(xmm2,15);

    ulsch_llr+=24;
  }

#if defined(__x86_64__) || defined(__i386__)
  _mm_empty();
  _m_empty();
#endif
}


void nr_ulsch_compute_llr(int32_t *rxdataF_comp,
                          int32_t *ul_ch_mag,
                          int32_t *ul_ch_magb,
                          int16_t *ulsch_llr,
                          uint32_t nb_rb,
                          uint32_t nb_re,
                          uint8_t  symbol,
                          uint8_t  mod_order)
{
  switch(mod_order){
    case 2:
      nr_ulsch_qpsk_llr(rxdataF_comp,
                        ulsch_llr,
                        nb_re,
                        symbol);
      break;
    case 4:
      nr_ulsch_16qam_llr(rxdataF_comp,
                         ul_ch_mag,
                         ulsch_llr,
                         nb_rb,
                         nb_re,
                         symbol);
      break;
    case 6:
    nr_ulsch_64qam_llr(rxdataF_comp,
                       ul_ch_mag,
                       ul_ch_magb,
                       ulsch_llr,
                       nb_rb,
                       nb_re,
                       symbol);
      break;
    default:
      LOG_E(PHY,"nr_ulsch_compute_llr: invalid Qm value, symbol = %d, Qm = %d\n",symbol, mod_order);
      break;
  }
}


void nr_ulsch_qpsk_qpsk(short *stream0_in, short *stream1_in, short *stream0_out, short *rho01, int length)
{
/*
This function computes the LLRs of stream 0 (s_0) in presence of the interfering stream 1 (s_1) assuming that both symbols are QPSK. It can be used for both MU-MIMO interference-aware receiver or
for SU-MIMO receivers.

Parameters:
  stream0_in = Matched filter output y0' = (h0*g0)*y0
  stream1_in = Matched filter output y1' = (h0*g1)*y0
  stream0_out = LLRs
  rho01 = Correlation between the two effective channels \rho_{10} = (h1*g1)*(h0*g0)
  length = number of resource elements
*/

#if defined(__x86_64__) || defined(__i386)
  __m128i A __attribute__((aligned(16)));
  __m128i B __attribute__((aligned(16)));
  __m128i C __attribute__((aligned(16)));
  __m128i D __attribute__((aligned(16)));
  __m128i E __attribute__((aligned(16)));
  __m128i F __attribute__((aligned(16)));
  __m128i G __attribute__((aligned(16)));
  __m128i H __attribute__((aligned(16)));

#endif

#if defined(__x86_64__) || defined(__i386__)
  __m128i *rho01_128i = (__m128i *)rho01;
  __m128i *stream0_128i_in = (__m128i *)stream0_in;
  __m128i *stream1_128i_in = (__m128i *)stream1_in;
  __m128i *stream0_128i_out = (__m128i *)stream0_out;
  __m128i ONE_OVER_SQRT_8 = _mm_set1_epi16(23170); // round(2^16/sqrt(8))
#elif defined(__arm__) || defined(__aarch64__)
                                                                                          int16x8_t *rho01_128i = (int16x8_t *)rho01;
  int16x8_t *stream0_128i_in = (int16x8_t *)stream0_in;
  int16x8_t *stream1_128i_in = (int16x8_t *)stream1_in;
  int16x8_t *stream0_128i_out = (int16x8_t *)stream0_out;
  int16x8_t ONE_OVER_SQRT_8 = vdupq_n_s16(23170); // round(2^16/sqrt(8))
#endif

  int i;

  for (i = 0; i < length >> 2; i += 2) {
// in each iteration, we take 8 complex samples
#if defined(__x86_64__) || defined(__i386__)
    __m128i xmm0 = rho01_128i[i]; // 4 symbols
    __m128i xmm1 = rho01_128i[i + 1];

    // put (rho_r + rho_i)/2sqrt2 in rho_rpi
    // put (rho_r - rho_i)/2sqrt2 in rho_rmi

    xmm0 = _mm_shufflelo_epi16(xmm0, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm0 = _mm_shufflehi_epi16(xmm0, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm0 = _mm_shuffle_epi32(xmm0, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm1 = _mm_shufflelo_epi16(xmm1, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm1 = _mm_shufflehi_epi16(xmm1, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm1 = _mm_shuffle_epi32(xmm1, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    // xmm0 = [Re(0,1) Re(2,3) Im(0,1) Im(2,3)]
    // xmm1 = [Re(4,5) Re(6,7) Im(4,5) Im(6,7)]
    __m128i xmm2 = _mm_unpacklo_epi64(xmm0, xmm1); // Re(rho)
    __m128i xmm3 = _mm_unpackhi_epi64(xmm0, xmm1); // Im(rho)
    __m128i rho_rpi = _mm_adds_epi16(xmm2, xmm3); // rho = Re(rho) + Im(rho)
    __m128i rho_rmi = _mm_subs_epi16(xmm2, xmm3); // rho* = Re(rho) - Im(rho)

    // divide by sqrt(8), no shift needed ONE_OVER_SQRT_8 = Q1.16
    rho_rpi = _mm_mulhi_epi16(rho_rpi, ONE_OVER_SQRT_8);
    rho_rmi = _mm_mulhi_epi16(rho_rmi, ONE_OVER_SQRT_8);
#elif defined(__arm__) || defined(__aarch64__)

#endif
// Compute LLR for first bit of stream 0

// Compute real and imaginary parts of MF output for stream 0
#if defined(__x86_64__) || defined(__i386__)
    xmm0 = stream0_128i_in[i];
    xmm1 = stream0_128i_in[i + 1];

    xmm0 = _mm_shufflelo_epi16(xmm0, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm0 = _mm_shufflehi_epi16(xmm0, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm0 = _mm_shuffle_epi32(xmm0, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm1 = _mm_shufflelo_epi16(xmm1, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm1 = _mm_shufflehi_epi16(xmm1, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm1 = _mm_shuffle_epi32(xmm1, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    // xmm0 = [Re(0,1) Re(2,3) Im(0,1) Im(2,3)]
    // xmm1 = [Re(4,5) Re(6,7) Im(4,5) Im(6,7)]
    __m128i y0r = _mm_unpacklo_epi64(xmm0, xmm1); // = [y0r(1),y0r(2),y0r(3),y0r(4)]
    __m128i y0i = _mm_unpackhi_epi64(xmm0, xmm1);

    __m128i y0r_over2 = _mm_srai_epi16(y0r, 1); // divide by 2
    __m128i y0i_over2 = _mm_srai_epi16(y0i, 1); // divide by 2
#elif defined(__arm__) || defined(__aarch64__)

#endif
// Compute real and imaginary parts of MF output for stream 1
#if defined(__x86_64__) || defined(__i386__)
    xmm0 = stream1_128i_in[i];
    xmm1 = stream1_128i_in[i + 1];

    xmm0 = _mm_shufflelo_epi16(xmm0, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm0 = _mm_shufflehi_epi16(xmm0, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm0 = _mm_shuffle_epi32(xmm0, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm1 = _mm_shufflelo_epi16(xmm1, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm1 = _mm_shufflehi_epi16(xmm1, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    xmm1 = _mm_shuffle_epi32(xmm1, 0xd8); //_MM_SHUFFLE(0,2,1,3));
    // xmm0 = [Re(0,1) Re(2,3) Im(0,1) Im(2,3)]
    // xmm1 = [Re(4,5) Re(6,7) Im(4,5) Im(6,7)]
    __m128i y1r = _mm_unpacklo_epi64(xmm0, xmm1); //[y1r(1),y1r(2),y1r(3),y1r(4)]
    __m128i y1i = _mm_unpackhi_epi64(xmm0, xmm1); //[y1i(1),y1i(2),y1i(3),y1i(4)]

    __m128i y1r_over2 = _mm_srai_epi16(y1r, 1); // divide by 2
    __m128i y1i_over2 = _mm_srai_epi16(y1i, 1); // divide by 2

    // Compute the terms for the LLR of first bit

    xmm0 = _mm_setzero_si128(); // ZERO

    // 1 term for numerator of LLR
    xmm3 = _mm_subs_epi16(y1r_over2, rho_rpi);
    A = _mm_abs_epi16(xmm3); // A = |y1r/2 - rho/sqrt(8)|
    xmm2 = _mm_adds_epi16(A, y0i_over2); // = |y1r/2 - rho/sqrt(8)| + y0i/2
    xmm3 = _mm_subs_epi16(y1i_over2, rho_rmi);
    B = _mm_abs_epi16(xmm3); // B = |y1i/2 - rho*/sqrt(8)|
    __m128i logmax_num_re0 = _mm_adds_epi16(B, xmm2); // = |y1r/2 - rho/sqrt(8)|+|y1i/2 - rho*/sqrt(8)| + y0i/2

    // 2 term for numerator of LLR
    xmm3 = _mm_subs_epi16(y1r_over2, rho_rmi);
    C = _mm_abs_epi16(xmm3); // C = |y1r/2 - rho*/4|
    xmm2 = _mm_subs_epi16(C, y0i_over2); // = |y1r/2 - rho*/4| - y0i/2
    xmm3 = _mm_adds_epi16(y1i_over2, rho_rpi);
    D = _mm_abs_epi16(xmm3); // D = |y1i/2 + rho/4|
    xmm2 = _mm_adds_epi16(xmm2, D); // |y1r/2 - rho*/4| + |y1i/2 + rho/4| - y0i/2
    logmax_num_re0 = _mm_max_epi16(logmax_num_re0, xmm2); // max, numerator done

    // 1 term for denominator of LLR
    xmm3 = _mm_adds_epi16(y1r_over2, rho_rmi);
    E = _mm_abs_epi16(xmm3); // E = |y1r/2 + rho*/4|
    xmm2 = _mm_adds_epi16(E, y0i_over2); // = |y1r/2 + rho*/4| + y0i/2
    xmm3 = _mm_subs_epi16(y1i_over2, rho_rpi);
    F = _mm_abs_epi16(xmm3); // F = |y1i/2 - rho/4|
    __m128i logmax_den_re0 = _mm_adds_epi16(F, xmm2); // = |y1r/2 + rho*/4| + |y1i/2 - rho/4| + y0i/2

    // 2 term for denominator of LLR
    xmm3 = _mm_adds_epi16(y1r_over2, rho_rpi);
    G = _mm_abs_epi16(xmm3); // G = |y1r/2 + rho/4|
    xmm2 = _mm_subs_epi16(G, y0i_over2); // = |y1r/2 + rho/4| - y0i/2
    xmm3 = _mm_adds_epi16(y1i_over2, rho_rmi);
    H = _mm_abs_epi16(xmm3); // H = |y1i/2 + rho*/4|
    xmm2 = _mm_adds_epi16(xmm2, H); // = |y1r/2 + rho/4| + |y1i/2 + rho*/4| - y0i/2
    logmax_den_re0 = _mm_max_epi16(logmax_den_re0, xmm2); // max, denominator done

    // Compute the terms for the LLR of first bit

    // 1 term for nominator of LLR
    xmm2 = _mm_adds_epi16(A, y0r_over2);
    __m128i logmax_num_im0 = _mm_adds_epi16(B, xmm2); // = |y1r/2 - rho/4| + |y1i/2 - rho*/4| + y0r/2

    // 2 term for nominator of LLR
    xmm2 = _mm_subs_epi16(E, y0r_over2);
    xmm2 = _mm_adds_epi16(xmm2, F); // = |y1r/2 + rho*/4| + |y1i/2 - rho/4| - y0r/2

    logmax_num_im0 = _mm_max_epi16(logmax_num_im0, xmm2); // max, nominator done

    // 1 term for denominator of LLR
    xmm2 = _mm_adds_epi16(C, y0r_over2);
    __m128i logmax_den_im0 = _mm_adds_epi16(D, xmm2); // = |y1r/2 - rho*/4| + |y1i/2 + rho/4| - y0r/2

    xmm2 = _mm_subs_epi16(G, y0r_over2);
    xmm2 = _mm_adds_epi16(xmm2, H); // = |y1r/2 + rho/4| + |y1i/2 + rho*/4| - y0r/2

    logmax_den_im0 = _mm_max_epi16(logmax_den_im0, xmm2); // max, denominator done

    // LLR of first bit [L1(1), L1(2), L1(3), L1(4)]
    y0r = _mm_adds_epi16(y0r, logmax_num_re0);
    y0r = _mm_subs_epi16(y0r, logmax_den_re0);

    // LLR of second bit [L2(1), L2(2), L2(3), L2(4)]
    y0i = _mm_adds_epi16(y0i, logmax_num_im0);
    y0i = _mm_subs_epi16(y0i, logmax_den_im0);

    _mm_storeu_si128(&stream0_128i_out[i], _mm_unpacklo_epi16(y0r, y0i)); // = [L1(1), L2(1), L1(2), L2(2)]

    if (i < ((length >> 1) - 1)) // false if only 2 REs remain
      _mm_storeu_si128(&stream0_128i_out[i + 1], _mm_unpackhi_epi16(y0r, y0i));

#elif defined(__x86_64__)

#endif
  }

#if defined(__x86_64__) || defined(__i386__)
  _mm_empty();
  _m_empty();
#endif
}


void nr_ulsch_compute_ML_llr(int32_t **rxdataF_comp,
                             int32_t ***rho,
                             int16_t **llr_layers,
                             uint8_t nb_antennas_rx,
                             uint32_t rb_size,
                             uint32_t nb_re,
                             uint8_t symbol,
                             uint32_t rxdataF_ext_offset,
                             uint8_t mod_order)
{
  int off = ((rb_size & 1) == 1) ? 4 : 0;
  c16_t *rxdataF_comp0 = (c16_t *)&rxdataF_comp[0][symbol * (off + (rb_size * NR_NB_SC_PER_RB))];
  c16_t *rxdataF_comp1 = (c16_t *)&rxdataF_comp[nb_antennas_rx][symbol * (off + (rb_size * NR_NB_SC_PER_RB))];
  c16_t *llr_layers0 = (c16_t *)&llr_layers[0][rxdataF_ext_offset * mod_order];
  c16_t *llr_layers1 = (c16_t *)&llr_layers[1][rxdataF_ext_offset * mod_order];
  c16_t *rho0 = (c16_t *)&rho[0][1][symbol * (off + (rb_size * NR_NB_SC_PER_RB))];
  c16_t *rho1 = (c16_t *)&rho[0][2][symbol * (off + (rb_size * NR_NB_SC_PER_RB))];

  switch (mod_order) {
    case 2:
      nr_ulsch_qpsk_qpsk((short *)rxdataF_comp0, (short *)rxdataF_comp1, (short *)llr_layers0, (short *)rho0, nb_re);
      nr_ulsch_qpsk_qpsk((short *)rxdataF_comp1, (short *)rxdataF_comp0, (short *)llr_layers1, (short *)rho1, nb_re);
      break;
    case 4:
    case 6:
      AssertFatal(1 == 0, "LLR computation is not implemented yet for ML with Qm = %d\n", mod_order);
    default:
      AssertFatal(1 == 0, "nr_ulsch_compute_llr: invalid Qm value, symbol = %d, Qm = %d\n", symbol, mod_order);
  }
}
