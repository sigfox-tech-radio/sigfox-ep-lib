/*!*****************************************************************
 * \file    sigfox_ep_api.h
 * \brief   Sigfox End-Point library API.
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

#ifndef __SIGFOX_EP_API_H__
#define __SIGFOX_EP_API_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

/*** SIGFOX EP API structures ***/

#ifdef ERROR_CODES
/*!******************************************************************
 * \enum SIGFOX_EP_API_status_t
 * \brief Sigfox EP library error codes.
 *******************************************************************/
typedef enum {
	// Core library errors.
	SIGFOX_EP_API_SUCCESS = 0,
	SIGFOX_EP_API_ERROR_NULL_PARAMETER,
	SIGFOX_EP_API_ERROR_RC,
	SIGFOX_EP_API_ERROR_MESSAGE_TYPE,
	SIGFOX_EP_API_ERROR_UL_PAYLOAD_SIZE,
	SIGFOX_EP_API_ERROR_DL_PAYLOAD_SIZE,
	SIGFOX_EP_API_ERROR_DL_PAYLOAD_UNAVAILABLE,
	SIGFOX_EP_API_ERROR_EP_ID,
	SIGFOX_EP_API_ERROR_EP_PAC,
	SIGFOX_EP_API_ERROR_EP_KEY,
	SIGFOX_EP_API_ERROR_STATE,
	SIGFOX_EP_API_ERROR_RF_MODE,
	SIGFOX_EP_API_ERROR_MODULATION,
	SIGFOX_EP_API_ERROR_BIT_RATE,
	SIGFOX_EP_API_ERROR_TX_POWER,
	SIGFOX_EP_API_ERROR_NUMBER_OF_FRAMES,
	SIGFOX_EP_API_ERROR_T_IFU,
	SIGFOX_EP_API_ERROR_T_CONF,
	SIGFOX_EP_API_ERROR_MESSAGE_COUNTER_ROLLOVER,
	SIGFOX_EP_API_ERROR_TX_FORBIDDEN,
	SIGFOX_EP_API_ERROR_VERSION,
	// Low level errors.
	// Activate the ERROR_STACK flag and use the SIGFOX_EP_API_unstack_error() function to get more details.
	SIGFOX_EP_API_ERROR_MCU,
	SIGFOX_EP_API_ERROR_RF,
	SIGFOX_EP_API_ERROR_BITSTREAM,
	SIGFOX_EP_API_ERROR_FREQUENCY,
	SIGFOX_EP_API_ERROR_NVM,
	SIGFOX_EP_API_ERROR_TX_CONTROL
} SIGFOX_EP_API_status_t;
#else
typedef void SIGFOX_EP_API_status_t;
#endif

#ifdef ASYNCHRONOUS
/*!******************************************************************
 * \enum SIGFOX_EP_API_state_t
 * \brief Sigfox EP library state.
 *******************************************************************/
typedef enum {
	SIGFOX_EP_API_STATE_CLOSED,
	SIGFOX_EP_API_STATE_READY,
#ifdef REGULATORY
	SIGFOX_EP_API_STATE_REGULATORY,
#endif
	SIGFOX_EP_API_STATE_UL_MODULATION_PENDING,
#if !(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0) || (defined BIDIRECTIONAL))
	SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER,
#endif
#ifdef BIDIRECTIONAL
	SIGFOX_EP_API_STATE_DL_TIMER,
	SIGFOX_EP_API_STATE_DL_LISTENING,
	SIGFOX_EP_API_STATE_DL_CONFIRMATION_TIMER,
	SIGFOX_EP_API_STATE_DL_CONFIRMATION,
#endif
	SIGFOX_EP_API_STATE_LAST
} SIGFOX_EP_API_state_t;
#endif

#ifdef ASYNCHRONOUS
/*!******************************************************************
 * \brief Sigfox EP library callback functions.
 * \fn SIGFOX_EP_API_process_cb_t:		Will be called each time a low level IRQ is handled by the library. Warning: runs in a IRQ context. Should only change variables state, and call as soon as possible @ref SIGFOX_EP_API_process.
 * \fn SIGFOX_EP_API_uplink_cplt_cb_t:	Will be called when the TX sequence is done (1 to 3 frames). Optional, could be set to NULL.
 * \fn SIGFOX_EP_API_downlink_cplt_cb:	Will be called if a valid downlink frame has been received. Optional, could be set to NULL.
 * \fn SIGFOX_EP_API_message_cplt_cb:	Will be called when the whole message process is finished, indicating that the library is ready again for a new operation. Optional, could be set to NULL.
 *******************************************************************/
typedef void (*SIGFOX_EP_API_process_cb_t)(void);
typedef void (*SIGFOX_EP_API_uplink_cplt_cb_t)(void);
#ifdef BIDIRECTIONAL
typedef void (*SIGFOX_EP_API_downlink_cplt_cb)(void);
#endif
typedef void (*SIGFOX_EP_API_message_cplt_cb)(void);
#endif

/*!******************************************************************
 * \struct SIGFOX_EP_API_config_t
 * \brief Sigfox EP library configuration structure.
 *******************************************************************/
typedef struct {
	const SIGFOX_rc_t *rc;
#ifdef ASYNCHRONOUS
	SIGFOX_EP_API_process_cb_t process_cb;
#endif
#ifndef MESSAGE_COUNTER_ROLLOVER
	SIGFOX_message_counter_rollover_t message_counter_rollover;
#endif
} SIGFOX_EP_API_config_t;

#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
/*!******************************************************************
 * \struct SIGFOX_EP_API_common_t
 * \brief Common parameters of application and control messages structures.
 *******************************************************************/
typedef struct {
#ifndef UL_BIT_RATE_BPS
	SIGFOX_ul_bit_rate_t ul_bit_rate;
#endif
#ifndef TX_POWER_DBM_EIRP
	sfx_s8 tx_power_dbm_eirp;
#endif
#ifndef SINGLE_FRAME
	sfx_u8 number_of_frames;
#ifndef T_IFU_MS
	sfx_u16 t_ifu_ms;
#endif
#endif
#ifdef PUBLIC_KEY_CAPABLE
	SIGFOX_ep_key_t ep_key_type;
#endif
} SIGFOX_EP_API_common_t;
#endif

#ifdef APPLICATION_MESSAGES
/*!******************************************************************
 * \struct SIGFOX_EP_API_application_message_t
 * \brief Application message data.
 *******************************************************************/
typedef struct {
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	SIGFOX_EP_API_common_t common_parameters;
#endif
	SIGFOX_application_message_type_t type;
#ifdef ASYNCHRONOUS
	SIGFOX_EP_API_uplink_cplt_cb_t uplink_cplt_cb;
#ifdef BIDIRECTIONAL
	SIGFOX_EP_API_downlink_cplt_cb downlink_cplt_cb;
#endif
	SIGFOX_EP_API_message_cplt_cb message_cplt_cb;
#endif
#if !(defined UL_PAYLOAD_SIZE) || (UL_PAYLOAD_SIZE > 0)
	sfx_u8 *ul_payload;
#endif
#ifndef UL_PAYLOAD_SIZE
	sfx_u8 ul_payload_size_bytes;
#endif
#ifdef BIDIRECTIONAL
	sfx_u8 bidirectional_flag;
#ifndef T_CONF_MS
	sfx_u16 t_conf_ms;
#endif
#endif
} SIGFOX_EP_API_application_message_t;
#endif

#ifdef CONTROL_KEEP_ALIVE_MESSAGE
/*!******************************************************************
 * \struct SIGFOX_EP_API_control_message_t
 * \brief Control message data.
 *******************************************************************/
typedef struct {
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	SIGFOX_EP_API_common_t common_parameters;
#endif
	SIGFOX_control_message_type_t type;
#ifdef ASYNCHRONOUS
	SIGFOX_EP_API_uplink_cplt_cb_t uplink_cplt_cb;
	SIGFOX_EP_API_message_cplt_cb message_cplt_cb;
#endif
} SIGFOX_EP_API_control_message_t;
#endif

/*!******************************************************************
 * \union SIGFOX_EP_API_message_status_t
 * \brief Message status bitfield.
 *******************************************************************/
typedef union {
	struct {
		unsigned app_frame_1 : 1;
		unsigned app_frame_2 : 1;
		unsigned app_frame_3 : 1;
		unsigned downlink_frame : 1;
		unsigned ack_frame : 1;
		unsigned error : 1;
	};
	sfx_u8 all;
} SIGFOX_EP_API_message_status_t;

/*** SIGFOX EP API functions ***/

/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_open(SIGFOX_EP_API_config_t *config)
 * \brief Open the EP library.
 * \param[in]  	config: Pointer to the library configuration.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_open(SIGFOX_EP_API_config_t *config);

/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_close(void)
 * \brief Close the EP library.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_close(void);

#ifdef ASYNCHRONOUS
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_process(void)
 * \brief Main process function of the library. This function should be called as soon as possible when the process callback is triggered.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_process(void);
#endif

#ifdef APPLICATION_MESSAGES
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_send_application_message(SIGFOX_EP_API_application_message_t *application_message)
 * \brief Send an application message over Sigfox network.
 * \param[in]  	application_message: Pointer to the application message data.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_send_application_message(SIGFOX_EP_API_application_message_t *application_message);
#endif

#ifdef CONTROL_KEEP_ALIVE_MESSAGE
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_send_control_message(SIGFOX_EP_API_control_message_t *control_message)
 * \brief Send a control message over Sigfox network.
 * \param[in]  	control_message: Pointer to the control message data.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_send_control_message(SIGFOX_EP_API_control_message_t *control_message);
#endif

#ifdef BIDIRECTIONAL
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_get_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 *rssi_dbm)
 * \brief Get the current message status.
 * \param[in]  	dl_payload: Byte array that will contain the downlink payload.
 * \param[in] 	dl_payload_size: Number of bytes to read.
 * \param[in]	dl_rssi_dbm: Pointer to 16-bits signed value that will contain the RSSI of the received downlink frame.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 *rssi_dbm);
#endif

/*!******************************************************************
 * \fn SIGFOX_EP_API_message_status_t SIGFOX_EP_API_get_message_status(void)
 * \brief Get the current message status.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Status of the last transmission.
 *******************************************************************/
SIGFOX_EP_API_message_status_t SIGFOX_EP_API_get_message_status(void);

#ifdef ASYNCHRONOUS
/*!******************************************************************
 * \fn SIGFOX_EP_API_state_t SIGFOX_EP_API_get_state(void)
 * \brief Get the current library state.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Current library state.
 *******************************************************************/
SIGFOX_EP_API_state_t SIGFOX_EP_API_get_state(void);
#endif

#ifdef VERBOSE
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_get_ep_id(sfx_u8* ep_id, sfx_u8 ep_id_size_bytes)
 * \brief Get EP ID.
 * \param[in]  	ep_id_size_bytes: Number of bytes of the ID to read (full size is SIGFOX_EP_ID_SIZE_BYTES).
 * \param[out] 	ep_id: End-point ID.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_ep_id(sfx_u8* ep_id, sfx_u8 ep_id_size_bytes);
#endif

#ifdef VERBOSE
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes)
 * \brief Get initial PAC.
 * \param[in]  	initial_pac_size_bytes: Number of bytes of the PAC to read (full size is SIGFOX_EP_PAC_SIZE_BYTES).
 * \param[out] 	initial_pac: Initial PAC.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes);
#endif

#ifdef VERBOSE
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_get_version(SIGFOX_version_t version_type, sfx_u8 **version, sfx_u8 *version_size_char)
 * \brief Get EP library version.
 * \param[in]  	version_type: Version to get.
 * \param[out] 	version: Version string.
 * \param[out]	version_size_char: Pointer that will contain the string size.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_version(SIGFOX_version_t version_type, sfx_u8 **version, sfx_u8 *version_size_char);
#endif

#ifdef VERBOSE
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_get_flags(sfx_u8 **ep_flags, sfx_u8 *ep_flags_size_char)
 * \brief Get EP library flags.
 * \param[in]  	none
 * \param[out] 	ep_flags: Flags string.
 * \param[out]	ep_flags_size_char: Pointer that will contain the string size.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_flags(sfx_u8 **ep_flags, sfx_u8 *ep_flags_size_char);
#endif

#ifdef ERROR_STACK
/*!******************************************************************
 * \fn  SIGFOX_EP_API_status_t SIGFOX_EP_API_unstack_error(SIGFOX_ERROR_t *error_ptr)
 * \brief Read and clear the last (newest) error stored in the internal error stack.
 * \brief This function can be called multiple times to unstack all errors which previously occurred during library execution, until it returns SIGFOX_EP_API_SUCCESS.
 * \param[in]  	none
 * \param[out] 	error_ptr: Pointer that will contain the last error in the stack.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_unstack_error(SIGFOX_ERROR_t *error_ptr);
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_EP_API_stack_error(void)
 * \brief Generic macro which calls the error stack function for EP library errors (if enabled).
 * \param[in]  	none
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#ifdef ERROR_STACK
#define SIGFOX_EP_API_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_EP_LIBRARY, ep_api_status)
#else
#define SIGFOX_EP_API_stack_error(void)
#endif
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_EP_API_check_status(error)
 * \brief Generic macro to check a SIGFOX_EP_API function status and exit.
 * \param[in]  	error: High level error code to rise.
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#define SIGFOX_EP_API_check_status(error) { if (ep_api_status != SIGFOX_EP_API_SUCCESS) { SIGFOX_EP_API_stack_error(); EXIT_ERROR(error) } }
#endif

#endif /* __SIGFOX_EP_API_H__ */
