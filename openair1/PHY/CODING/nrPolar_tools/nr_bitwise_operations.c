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

#include "PHY/CODING/nrPolar_tools/nr_polar_defs.h"

void nr_bit2byte_uint32_8_t(uint32_t *in, uint16_t arraySize, uint8_t *out) {
	uint8_t arrayInd = ceil(arraySize / 32.0);
	for (int i = 0; i < arrayInd; i++) {
		for (int j = 0; j < 32; j++) {
			out[j+(i*32)] = (in[i] >> j) & 1;
		}
	}
}

void nr_byte2bit_uint8_32_t(uint8_t *in, uint16_t arraySize, uint32_t *out) {
	uint8_t arrayInd = ceil(arraySize / 32.0);
	for (int i = 0; i < arrayInd; i++) {
		for (int j = 0; j < 32; j++) {
			out[i]|=in[j+(i*32)];
			out[i]<<=1;
		}
	}
}
