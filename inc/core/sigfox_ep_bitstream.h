/*!*****************************************************************
 * \file    sigfox_ep_bitstream.h
 * \brief   Sigfox bitstream builder and decoder.
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

#ifndef __SIGFOX_EP_BITSTREAM_H__
#define __SIGFOX_EP_BITSTREAM_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

/*** SIGFOX EP BITSTREAM macros ***/

#if (defined UL_PAYLOAD_SIZE)
#if (UL_PAYLOAD_SIZE == 0)
#define SIGFOX_EP_BITSTREAM_SIZE_BYTES	14
#elif (UL_PAYLOAD_SIZE == 1)
#define SIGFOX_EP_BITSTREAM_SIZE_BYTES	15
#elif (UL_PAYLOAD_SIZE == 2) || (UL_PAYLOAD_SIZE == 3) || (UL_PAYLOAD_SIZE == 4)
#define SIGFOX_EP_BITSTREAM_SIZE_BYTES	18
#elif (UL_PAYLOAD_SIZE == 5) || (UL_PAYLOAD_SIZE == 6) || (UL_PAYLOAD_SIZE == 7) || (UL_PAYLOAD_SIZE == 8)
#define SIGFOX_EP_BITSTREAM_SIZE_BYTES	22
#elif (UL_PAYLOAD_SIZE == 9) || (UL_PAYLOAD_SIZE == 10) || (UL_PAYLOAD_SIZE == 11) || (UL_PAYLOAD_SIZE == 12)
#define SIGFOX_EP_BITSTREAM_SIZE_BYTES	26
#else
#define SIGFOX_EP_BITSTREAM_SIZE_BYTES	26
#endif
#else
#define SIGFOX_EP_BITSTREAM_SIZE_BYTES	26 // Maximum value used as default.
#endif

/*** SIGFOX EP BITSTREAM structures ***/

#ifdef ERROR_CODES
/*!******************************************************************
 * \enum SIGFOX_EP_BITSTREAM_status_t
 * \brief Sigfox bitstream driver error codes.
 *******************************************************************/
typedef enum {
	SIGFOX_EP_BITSTREAM_SUCCESS = 0,
	SIGFOX_EP_BITSTREAM_ERROR_NULL_PARAMETER,
	SIGFOX_EP_BITSTREAM_ERROR_MESSAGE_TYPE,
	SIGFOX_EP_BITSTREAM_ERROR_PAYLOAD_SIZE,
	SIGFOX_EP_BITSTREAM_ERROR_KEY_TYPE,
	SIGFOX_EP_BITSTREAM_ERROR_FRAME_RANK,
	SIGFOX_EP_BITSTREAM_ERROR_MESSAGE_COUNTER,
	// Low level errors.
	SIGFOX_EP_BITSTREAM_ERROR_CRC,
	SIGFOX_EP_BITSTREAM_ERROR_MCU
} SIGFOX_EP_BITSTREAM_status_t;
#else
typedef void SIGFOX_EP_BITSTREAM_status_t;
#endif

/*!******************************************************************
 * \struct SIGFOX_EP_BITSTREAM_common_t
 * \brief Common parameters of application and control frames bitstream.
 *******************************************************************/
typedef struct {
	sfx_u8 *ep_id;
	sfx_u16 message_counter;
#ifndef MESSAGE_COUNTER_ROLLOVER
	sfx_u16 message_counter_rollover;
#endif
#ifndef SINGLE_FRAME
	SIGFOX_ul_frame_rank_t ul_frame_rank;
#endif
#ifdef PUBLIC_KEY_CAPABLE
	SIGFOX_ep_key_t ep_key_type;
#endif
} SIGFOX_EP_BITSTREAM_common_t;

#ifdef APPLICATION_MESSAGES
/*!******************************************************************
 * \struct SIGFOX_EP_BITSTREAM_application_frame_t
 * \brief Specific parameters of application frames bitstream.
 *******************************************************************/
typedef struct {
	SIGFOX_EP_BITSTREAM_common_t common_parameters;
	SIGFOX_application_message_type_t message_type;
#if !(defined UL_PAYLOAD_SIZE) || (UL_PAYLOAD_SIZE > 0)
	sfx_u8 *ul_payload;
#endif
#ifndef UL_PAYLOAD_SIZE
	sfx_u8 ul_payload_size_bytes;
#endif
#ifdef BIDIRECTIONAL
	sfx_bool bidirectional_flag;
#endif
} SIGFOX_EP_BITSTREAM_application_frame_t;
#endif

#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
/*!******************************************************************
 * \struct SIGFOX_EP_BITSTREAM_control_frame_t
 * \brief Specific parameters of control frames bitstream.
 *******************************************************************/
typedef struct {
	SIGFOX_EP_BITSTREAM_common_t common_parameters;
	SIGFOX_control_message_type_t message_type;
	sfx_s16 temperature_tenth_degrees;
	sfx_u16 voltage_tx_mv;
	sfx_u16 voltage_idle_mv;
#ifdef BIDIRECTIONAL
	sfx_s16 rssi_dbm; // For DL confirmation only.
#endif
} SIGFOX_EP_BITSTREAM_control_frame_t;
#endif

#ifdef BIDIRECTIONAL
/*!******************************************************************
 * \struct SIGFOX_EP_BITSTREAM_dl_frame_t
 * \brief Specific parameters of downlink frames bitstream.
 *******************************************************************/
typedef struct {
	sfx_u8 *dl_phy_content; // Raw bytes from radio.
	sfx_u8 *ep_id;
	sfx_u16 message_counter; // Message counter of the corresponding uplink frame that requested bidirectional procedure.
#ifdef PUBLIC_KEY_CAPABLE
	SIGFOX_ep_key_t ep_key_type;
#endif
} SIGFOX_EP_BITSTREAM_dl_frame_t;
#endif

/*** SIGFOX BISTREAM functions ***/

#ifdef APPLICATION_MESSAGES
/*!******************************************************************
 * \fn SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_application_frame(SIGFOX_EP_BITSTREAM_application_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes)
 * \brief Build an application frame bistream.
 * \param[in]  	input: Application frame input parameters.
 * \param[out] 	bitstream: Computed bistream.
 * \param[out]	bitstream_size_bytes: Size of the computed bitstream in bytes.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_application_frame(SIGFOX_EP_BITSTREAM_application_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes);
#endif

#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
/*!******************************************************************
 * \fn SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_control_frame(SIGFOX_EP_BITSTREAM_control_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes)
 * \brief Build a control frame bistream.
 * \param[in]  	input: Control frame input parameters.
 * \param[out] 	bitstream: Computed bistream.
 * \param[out]	bitstream_size_bytes: Size of the computed bitstream in bytes.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_control_frame(SIGFOX_EP_BITSTREAM_control_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes);
#endif

#ifdef BIDIRECTIONAL
/*!******************************************************************
 * \fn SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_decode_downlink_frame(SIGFOX_EP_BITSTREAM_dl_frame_t *input, sfx_bool *dl_frame_valid, sfx_u8 *dl_payload)
 * \brief Authenticate a downlink frame.
 * \param[in]  	input: Received downlink frame input parameters.
 * \param[out] 	dl_frame_valid: Pointer to the authentication result.
 * \param[out]	dl_payload:	Contains the extracted DL user payload if dl_frame_valid is returned with SFX_TRUE value.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_decode_downlink_frame(SIGFOX_EP_BITSTREAM_dl_frame_t *input, sfx_bool *dl_frame_valid, sfx_u8 *dl_payload);
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_EP_BITSTREAM_stack_error(void)
 * \brief Generic macro which calls the error stack function for bitstream errors (if enabled).
 * \param[in]  	none
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#ifdef ERROR_STACK
#define SIGFOX_EP_BITSTREAM_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_BITSTREAM, bitstream_status)
#else
#define SIGFOX_EP_BITSTREAM_stack_error(void)
#endif
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_EP_BITSTREAM_check_status(error)
 * \brief Generic macro to check a SIGFOX_EP_BITSTREAM function status and exit.
 * \param[in]  	error: High level error code to rise.
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#define SIGFOX_EP_BITSTREAM_check_status(error) { if (bitstream_status != SIGFOX_EP_BITSTREAM_SUCCESS) { SIGFOX_EP_BITSTREAM_stack_error(); EXIT_ERROR(error) } }
#endif

#endif /* __SIGFOX_EP_BITSTREAM_H__ */
