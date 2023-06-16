/*!*****************************************************************
 * \file    sigfox_crc.c
 * \brief   Sigfox CRC driver.
 *******************************************************************
 * \copyright
 *
 * Copyright (c) 2022, UnaBiz SAS
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 *  1 Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *  2 Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *  3 Neither the name of UnaBiz SAS nor the names of its contributors may be
 *    used to endorse or promote products derived from this software without
 *    specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR
 * CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL,
 * EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO,
 * PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR
 * BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER
 * IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 *******************************************************************/

#include "core/sigfox_crc.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

#ifndef CRC_HW

/*** SIGFOX CRC functions ***/

/*******************************************************************/
SIGFOX_CRC_status_t SIGFOX_CRC_compute_crc16(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc) {
	// Local variables.
    sfx_u8 i = 0;
    sfx_u8 j = 0;
#ifdef ERROR_CODES
    SIGFOX_CRC_status_t status = SIGFOX_CRC_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
    if ((crc_data == SFX_NULL) || (crc == SFX_NULL)) {
    	EXIT_ERROR(SIGFOX_CRC_ERROR_NULL_PARAMETER);
    }
#endif
	// Compute CRC.
    (*crc) = 0;
	for (j=0 ; j<data_size ; j++) {
		(*crc) ^= (sfx_u16) ((sfx_u16) crc_data[j] << 8);
		for (i=0 ; i<8 ; i++) {
			if (((*crc) & 0x8000) != 0) {
				(*crc) = (sfx_u16) (((*crc) << 1) ^ polynom);
			}
			else {
				(*crc) = (sfx_u16) ((*crc) << 1);
			}
		}
	}
#ifdef PARAMETERS_CHECK
errors:
#endif
	RETURN();
}

#ifdef BIDIRECTIONAL
/*******************************************************************/
SIGFOX_CRC_status_t SIGFOX_CRC_compute_crc8(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u8 polynom, sfx_u8 *crc) {
	// Local variables.
	sfx_u8 i = 0;
	sfx_u8 j = 0;
#ifdef ERROR_CODES
	SIGFOX_CRC_status_t status = SIGFOX_CRC_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
    if ((crc_data == SFX_NULL) || (crc == SFX_NULL)) {
    	EXIT_ERROR(SIGFOX_CRC_ERROR_NULL_PARAMETER);
    }
#endif
	// Compute CRC.
	(*crc) = 0;
	for (j=0 ; j<data_size ; j++) {
		(*crc) = crc_data[j] ^ (*crc);
		for (i=0 ; i<8; i++) {
			if (((*crc) & 0x80) != 0) {
				(*crc) = (sfx_u8) (((*crc) << 1) ^ polynom);
			}
			else {
				(*crc) = (sfx_u8) ((*crc) << 1);
			}
		}
	}
#ifdef PARAMETERS_CHECK
errors:
#endif
	RETURN();
}
#endif

#endif /* CRC_HW */
