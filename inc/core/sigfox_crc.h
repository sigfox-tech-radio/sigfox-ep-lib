/*!*****************************************************************
 * \file    sigfox_crc.h
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

#ifndef __SIGFOX_CRC_H__
#define __SIGFOX_CRC_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

#ifndef CRC_HW

/*** SIGFOX CRC structures ***/

#ifdef ERROR_CODES
/*!******************************************************************
 * \enum SIGFOX_CRC_status_t
 * \brief Sigfox CRC driver error codes.
 *******************************************************************/
typedef enum {
	SIGFOX_CRC_SUCCESS = 0,
	SIGFOX_CRC_ERROR_NULL_PARAMETER,
	// Last index.
	SIGFOX_CRC_ERROR_LAST
} SIGFOX_CRC_status_t;
#else
typedef void SIGFOX_CRC_status_t;
#endif

/*** SIGFOX CRC functions ***/

/*!******************************************************************
 * \fn SIGFOX_CRC_status_t SIGFOX_CRC_compute_crc16(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc)
 * \brief Compute a CRC16.
 * \param[in]  	crc_data: Input data.
 * \param[in]	data_size: Number of bytes of the input data.
 * \param[in]	polynom: CRC polynom to use.
 * \param[out] 	crc: Pointer to the computed CRC16 value.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_CRC_status_t SIGFOX_CRC_compute_crc16(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc);

#ifdef BIDIRECTIONAL
/*!******************************************************************
 * \fn SIGFOX_CRC_status_t SIGFOX_CRC_compute_crc8(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u8 polynom, sfx_u8 *crc)
 * \brief Compute a CRC8.
 * \param[in]  	crc_data: Input data.
 * \param[in]	data_size: Number of bytes of the input data.
 * \param[in]	polynom: CRC polynom to use.
 * \param[out] 	crc: Pointer to the computed CRC8 value.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_CRC_status_t SIGFOX_CRC_compute_crc8(sfx_u8 *crc_data, sfx_u8 data_size, sfx_u8 polynom, sfx_u8 *crc);
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_CRC_stack_error(void)
 * \brief Generic macro which calls the error stack function for CRC errors (if enabled).
 * \param[in]  	none
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#ifdef ERROR_STACK
#define SIGFOX_CRC_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_SIGFOX_CRC, sigfox_crc_status)
#else
#define SIGFOX_CRC_stack_error(void)
#endif
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_CRC_check_status(error)
 * \brief Generic macro to check a SIGFOX_CRC function status and exit.
 * \param[in]  	error: High level error code to rise.
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#define SIGFOX_CRC_check_status(error) { if (sigfox_crc_status != SIGFOX_CRC_SUCCESS) { SIGFOX_CRC_stack_error(); EXIT_ERROR(error) } }
#endif

#endif /* CRC_HW */

#endif /* __SIGFOX_CRC_H__ */
