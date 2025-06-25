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

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

/*** SIGFOX EP BITSTREAM structures ***/

#ifdef SIGFOX_EP_ERROR_CODES
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
    // Low level drivers errors.
    // Activate the SIGFOX_EP_ERROR_STACK flag and use the SIGFOX_EP_API_unstack_error() function to get more details.
    SIGFOX_EP_BITSTREAM_ERROR_DRIVER_SIGFOX_CRC,
    SIGFOX_EP_BITSTREAM_ERROR_DRIVER_MCU_API,
    // Last index.
    SIGFOX_EP_BITSTREAM_ERROR_LAST
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
#ifndef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
    sfx_u16 message_counter_rollover;
#endif
#ifndef SIGFOX_EP_SINGLE_FRAME
    SIGFOX_ul_frame_rank_t ul_frame_rank;
#endif
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    SIGFOX_ep_key_t ep_key_type;
#endif
} SIGFOX_EP_BITSTREAM_common_t;

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
/*!******************************************************************
 * \struct SIGFOX_EP_BITSTREAM_application_frame_t
 * \brief Specific parameters of application frames bitstream.
 *******************************************************************/
typedef struct {
    SIGFOX_EP_BITSTREAM_common_t common_parameters;
    SIGFOX_application_message_type_t message_type;
#if (!(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0))
    sfx_u8 *ul_payload;
#endif
#ifndef SIGFOX_EP_UL_PAYLOAD_SIZE
    sfx_u8 ul_payload_size_bytes;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_bool bidirectional_flag;
#endif
} SIGFOX_EP_BITSTREAM_application_frame_t;
#endif

#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL))
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
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_s16 rssi_dbm; // For DL confirmation only.
#endif
} SIGFOX_EP_BITSTREAM_control_frame_t;
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \struct SIGFOX_EP_BITSTREAM_dl_frame_t
 * \brief Specific parameters of downlink frames bitstream.
 *******************************************************************/
typedef struct {
    sfx_u8 *dl_phy_content; // Raw bytes from radio.
    sfx_u8 *ep_id;
    sfx_u16 message_counter; // Message counter of the corresponding uplink frame that requested bidirectional procedure.
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    SIGFOX_ep_key_t ep_key_type;
#endif
#ifdef SIGFOX_EP_CERTIFICATION
    sfx_bool dl_decoding_enable; // Enable or disable downlink frame decoding (dewhitening, BCH, CRC and AUTH).
#endif
} SIGFOX_EP_BITSTREAM_dl_frame_t;
#endif

/*** SIGFOX BISTREAM functions ***/

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
/*!******************************************************************
 * \fn SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_application_frame(SIGFOX_EP_BITSTREAM_application_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes)
 * \brief Build an application frame bitstream.
 * \param[in]   input: Application frame input parameters.
 * \param[out]  bitstream: Computed bitstream.
 * \param[out]  bitstream_size_bytes: Size of the computed bitstream in bytes.
 * \retval      Function execution status.
 *******************************************************************/
SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_application_frame(SIGFOX_EP_BITSTREAM_application_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes);
#endif

#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL))
/*!******************************************************************
 * \fn SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_control_frame(SIGFOX_EP_BITSTREAM_control_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes)
 * \brief Build a control frame bitstream.
 * \param[in]   input: Control frame input parameters.
 * \param[out]  bitstream: Computed bitstream.
 * \param[out]  bitstream_size_bytes: Size of the computed bitstream in bytes.
 * \retval      Function execution status.
 *******************************************************************/
SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_control_frame(SIGFOX_EP_BITSTREAM_control_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes);
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*!******************************************************************
 * \fn SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_decode_downlink_frame(SIGFOX_EP_BITSTREAM_dl_frame_t *input, sfx_bool *dl_frame_valid, sfx_u8 *dl_payload)
 * \brief Authenticate a downlink frame.
 * \param[in]   input: Received downlink frame input parameters.
 * \param[out]  dl_frame_valid: Pointer to the authentication result.
 * \param[out]  dl_payload: Contains the extracted DL user payload if dl_frame_valid is returned with SIGFOX_TRUE value.
 * \retval      Function execution status.
 *******************************************************************/
SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_decode_downlink_frame(SIGFOX_EP_BITSTREAM_dl_frame_t *input, sfx_bool *dl_frame_valid, sfx_u8 *dl_payload);
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_EP_BITSTREAM_stack_error(void)
 * \brief Generic macro which calls the error stack function for bitstream errors (if enabled).
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
#ifdef SIGFOX_EP_ERROR_STACK
#define SIGFOX_EP_BITSTREAM_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_SIGFOX_EP_BITSTREAM, sigfox_ep_bitstream_status)
#else
#define SIGFOX_EP_BITSTREAM_stack_error(void)
#endif
#endif

#ifdef SIGFOX_EP_ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_EP_BITSTREAM_check_status(error)
 * \brief Generic macro to check a SIGFOX_EP_BITSTREAM function status and exit.
 * \param[in]   error: High level error code to rise.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
#define SIGFOX_EP_BITSTREAM_check_status(error) { if (sigfox_ep_bitstream_status != SIGFOX_EP_BITSTREAM_SUCCESS) { SIGFOX_EP_BITSTREAM_stack_error(); SIGFOX_EXIT_ERROR(error) } }
#endif

#endif /* __SIGFOX_EP_BITSTREAM_H__ */
