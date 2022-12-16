/*!*****************************************************************
 * \file    sigfox_ep_api.c
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

#include "sigfox_ep_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "manuf/mcu_api.h"
#include "manuf/rf_api.h"
#include "core/sigfox_ep_bitstream.h"
#include "core/sigfox_ep_frequency.h"
#include "sigfox_rc.h"
#ifdef CERTIFICATION
#include "sigfox_ep_api_test.h"
#endif
#ifdef REGULATORY
#include "core/sigfox_tx_control.h"
#endif
#ifdef VERBOSE
#include "sigfox_ep_version.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

/*** SIGFOX EP API local macros ***/

#ifdef APPLICATION_MESSAGES
#ifdef UL_PAYLOAD_SIZE
#define SIGFOX_EP_API_UL_PAYLOAD_SIZE	UL_PAYLOAD_SIZE
#else
#define SIGFOX_EP_API_UL_PAYLOAD_SIZE	SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES
#endif
#endif

/*** SIGFOX EP API local structures ***/

#ifndef ASYNCHRONOUS
/*******************************************************************/
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

/*******************************************************************/
typedef union {
	struct {
		unsigned error_stack_initialized : 1;
		unsigned synchronous : 1;
		unsigned send_message_request : 1;
		unsigned radio_woken_up : 1;
		unsigned ack_message : 1;
		unsigned control_message : 1;
		unsigned tx_control_pre_check_running : 1;
		unsigned tx_control_post_check_running : 1;
		unsigned tx_forbidden : 1;
		unsigned frame_success : 1;
		unsigned nvm_write_pending : 1;
		unsigned process_running : 1;
	};
	sfx_u16 all;
} SIGFOX_EP_API_internal_flags_t;

/*******************************************************************/
typedef union {
	struct {
		unsigned mcu_process : 1;
		unsigned mcu_timer1_cplt : 1;
		unsigned mcu_timer2_cplt : 1;
		unsigned rf_process : 1;
		unsigned rf_tx_cplt : 1;
		unsigned rf_rx_data_received : 1;
		unsigned tx_control_process : 1;
		unsigned tx_control_pre_check_cplt : 1;
		unsigned tx_control_post_check_cplt : 1;
		unsigned low_level_error : 1;
	};
	sfx_u16 all;
} SIGFOX_EP_API_irq_flags_t;

/*******************************************************************/
typedef struct {
	const SIGFOX_rc_t *rc_ptr;
	sfx_u8 ep_id[SIGFOX_EP_ID_SIZE_BYTES];
	SIGFOX_EP_API_state_t state;
	SIGFOX_EP_API_message_status_t message_status;
	SIGFOX_EP_API_internal_flags_t internal_flags;
	volatile SIGFOX_EP_API_irq_flags_t irq_flags;
	sfx_u16 message_counter;
	sfx_u16 random_value;
	sfx_u32 ul_frequency_hz;
#ifndef MESSAGE_COUNTER_ROLLOVER
	sfx_u16 message_counter_rollover;
#endif
#ifdef ASYNCHRONOUS
	SIGFOX_EP_API_process_cb_t process_cb;
	SIGFOX_EP_API_uplink_cplt_cb_t uplink_cplt_cb;
#ifdef BIDIRECTIONAL
	SIGFOX_EP_API_downlink_cplt_cb downlink_cplt_cb;
#endif
	SIGFOX_EP_API_message_cplt_cb message_cplt_cb;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status_from_callback;
#endif
#endif
	// Message data.
	SIGFOX_EP_API_status_t (*sending_function_ptr)(void);
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	SIGFOX_EP_API_common_t *common_parameters_ptr;
#endif
#ifdef APPLICATION_MESSAGES
	SIGFOX_EP_API_application_message_t *application_message_ptr;
#ifdef ASYNCHRONOUS
	SIGFOX_EP_API_application_message_t local_application_message;
#if !(defined UL_PAYLOAD_SIZE) || (UL_PAYLOAD_SIZE > 0)
	sfx_u8 local_ul_payload[SIGFOX_EP_API_UL_PAYLOAD_SIZE];
#endif
#endif /* ASYNCHRONOUS */
#endif /* APPLICATION_MESSAGES */
#ifdef BIDIRECTIONAL
	// Downlink variables.
	sfx_bool dl_status;
	sfx_u8 dl_payload[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
	sfx_s16 dl_rssi_dbm;
#endif /* BIDIRECTIONAL */
#ifdef CONTROL_KEEP_ALIVE_MESSAGE
	SIGFOX_EP_API_control_message_t *control_message_ptr;
#ifdef ASYNCHRONOUS
	SIGFOX_EP_API_control_message_t local_control_message;
#endif
#endif
#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
	sfx_s16 temperature_tenth_degrees;
	sfx_u16 voltage_tx_mv;
	sfx_u16 voltage_idle_mv;
#endif
#ifndef SINGLE_FRAME
	SIGFOX_ul_frame_rank_t ul_frame_rank;
	sfx_u8 frame_success_count;
#endif
#if !(defined SINGLE_FRAME) || (defined BIDIRECTIONAL)
	sfx_u32 interframe_ms; // Tifu, Tifb or Tconf.
#endif
#ifdef CERTIFICATION
	SIGFOX_EP_API_TEST_parameters_t test_parameters;
#endif
} SIGFOX_EP_API_context_t;

/*** SIGFOX EP API local global variables ***/

static SIGFOX_EP_API_context_t sigfox_ep_api_ctx = {
	.rc_ptr = SFX_NULL,
	.state = SIGFOX_EP_API_STATE_CLOSED,
	.internal_flags.all = 0,
	.irq_flags.all = 0,
	.message_status.all = 0,
#ifndef MESSAGE_COUNTER_ROLLOVER
	.message_counter_rollover = 0,
#endif
#ifdef ASYNCHRONOUS
#ifdef ERROR_CODES
	.status_from_callback = SIGFOX_EP_API_SUCCESS,
#endif /* ERROR_CODES */
#endif /* ASYNCHRONOUS */
	.sending_function_ptr = SFX_NULL,
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP)
	.common_parameters_ptr = SFX_NULL,
#endif
#ifdef APPLICATION_MESSAGES
	.application_message_ptr = SFX_NULL,
#ifdef BIDIRECTIONAL
	.dl_status = SFX_FALSE,
	.dl_rssi_dbm = 0,
#endif /* BIDIRECTIONAL */
#endif /* APPLICATION_MESSAGES */
#ifdef CONTROL_KEEP_ALIVE_MESSAGE
	.control_message_ptr = SFX_NULL,
#endif
#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
	.temperature_tenth_degrees = 0,
	.voltage_tx_mv = 0,
	.voltage_idle_mv = 0,
#endif
#ifndef SINGLE_FRAME
	.ul_frame_rank = SIGFOX_UL_FRAME_RANK_1,
	.frame_success_count = 0,
#endif
};
#ifdef VERBOSE
static const sfx_u8 SIGFOX_EP_API_VERSION[] = SIGFOX_EP_VERSION;
static const sfx_u8 SIGFOX_EP_API_FLAGS[] = SIGFOX_EP_FLAGS;
#endif

/*** SIGFOX EP API local functions ***/

/*******************************************************************/
#ifdef ERROR_CODES
#define _CHECK_LIBRARY_STATE(state_condition) { if (sigfox_ep_api_ctx.state state_condition) { status = SIGFOX_EP_API_ERROR_STATE; goto error_state; } }
#else
#define _CHECK_LIBRARY_STATE(state_condition) { if (sigfox_ep_api_ctx.state state_condition) { goto error_state; } }
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
#define _PROCESS_CALLBACK(void) { \
	if ((sigfox_ep_api_ctx.process_cb != SFX_NULL) && (sigfox_ep_api_ctx.internal_flags.process_running == 0)) { \
		sigfox_ep_api_ctx.process_cb(); \
	} \
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
#define _UPLINK_CPLT_CALLBACK(void) { \
	if ((sigfox_ep_api_ctx.uplink_cplt_cb != SFX_NULL) && (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY)) { \
		sigfox_ep_api_ctx.uplink_cplt_cb(); \
	} \
}
#endif

#if (defined ASYNCHRONOUS) && (defined BIDIRECTIONAL)
/*******************************************************************/
#define _DOWNLINK_CPLT_CALLBACK(void) { \
	if ((sigfox_ep_api_ctx.downlink_cplt_cb != SFX_NULL) && (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY)) { \
		sigfox_ep_api_ctx.downlink_cplt_cb(); \
	} \
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
#define _MESSAGE_CPLT_CALLBACK(void) { \
	if ((sigfox_ep_api_ctx.message_cplt_cb != SFX_NULL) && (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY)) { \
		sigfox_ep_api_ctx.message_cplt_cb(); \
	} \
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
static void _MCU_API_process_cb(void) {
    // Set local flag.
    sigfox_ep_api_ctx.irq_flags.mcu_process = 1;
    _PROCESS_CALLBACK();
}

#ifdef ASYNCHRONOUS
#ifdef ERROR_CODES
/*******************************************************************/
static void _MCU_API_error_cb(MCU_API_status_t mcu_status) {
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	// Set local error code.
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
	// Do not call the process if the given status is not an error.
	return;
errors:
	sigfox_ep_api_ctx.status_from_callback = status;
	sigfox_ep_api_ctx.irq_flags.low_level_error = 1;
	_PROCESS_CALLBACK();
}
#else
/*******************************************************************/
static void _MCU_API_error_cb(void) {
	sigfox_ep_api_ctx.irq_flags.low_level_error = 1;
	_PROCESS_CALLBACK();
}
#endif
#endif

#if (defined ASYNCHRONOUS) && ((!(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0))) || (defined BIDIRECTIONAL))
/*******************************************************************/
static void _MCU_API_timer1_cplt_cb(void) {
	// Set local flag.
	sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt = 1;
	_PROCESS_CALLBACK();
}
#endif

#if (defined ASYNCHRONOUS) && (defined BIDIRECTIONAL)
/*******************************************************************/
static void _MCU_API_timer2_cplt_cb(void) {
	// Set local flag.
	sigfox_ep_api_ctx.irq_flags.mcu_timer2_cplt = 1;
	_PROCESS_CALLBACK();
}
#endif

/*******************************************************************/
static void _RF_API_process_cb(void) {
    // Set local flag.
    sigfox_ep_api_ctx.irq_flags.rf_process = 1;
    _PROCESS_CALLBACK();
}

#ifdef ASYNCHRONOUS
#ifdef ERROR_CODES
/*******************************************************************/
static void _RF_API_error_cb(RF_API_status_t rf_status) {
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	// Set local error code.
	RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
	// Do not call the process if the given status is not an error.
	return;
errors:
	sigfox_ep_api_ctx.status_from_callback = status;
	sigfox_ep_api_ctx.irq_flags.low_level_error = 1;
	_PROCESS_CALLBACK();
}
#else
/*******************************************************************/
static void _RF_API_error_cb(void) {
	sigfox_ep_api_ctx.irq_flags.low_level_error = 1;
	_PROCESS_CALLBACK();
}
#endif
#endif

/*******************************************************************/
static void _RF_API_tx_cplt_cb(void) {
	// Set local flag.
	sigfox_ep_api_ctx.irq_flags.rf_tx_cplt = 1;
	_PROCESS_CALLBACK();
}
#endif

#if (defined ASYNCHRONOUS) && (defined BIDIRECTIONAL)
/*******************************************************************/
static void _RF_API_rx_data_received_cb(void) {
	// Set local flag.
	sigfox_ep_api_ctx.irq_flags.rf_rx_data_received = 1;
	_PROCESS_CALLBACK();
}
#endif

#if (defined ASYNCHRONOUS) && (defined REGULATORY)
/*******************************************************************/
static void _SIGFOX_TX_CONTROL_process_cb(void) {
	// Set local flag and result.
	sigfox_ep_api_ctx.irq_flags.tx_control_process = 1;
	_PROCESS_CALLBACK();
}
#endif

#if (defined ASYNCHRONOUS) && (defined REGULATORY)
/*******************************************************************/
static void _SIGFOX_TX_CONTROL_pre_check_cplt_cb(void) {
	// Set local flag and result.
	sigfox_ep_api_ctx.irq_flags.tx_control_pre_check_cplt = 1;
	_PROCESS_CALLBACK();
}
#endif

#if (defined ASYNCHRONOUS) && (defined REGULATORY)
/*******************************************************************/
static void _SIGFOX_TX_CONTROL_post_check_cplt_cb(void) {
	// Set local flag and result.
	sigfox_ep_api_ctx.irq_flags.tx_control_post_check_cplt = 1;
	_PROCESS_CALLBACK();
}
#endif

#if (defined PARAMETERS_CHECK) && (defined ERROR_CODES) && (!(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE))
/*******************************************************************/
static SIGFOX_EP_API_status_t _check_common_parameters(SIGFOX_EP_API_common_t *common_params) {
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	// Check parameter.
	if (common_params == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
	// Check bit rate.
#ifdef UL_BIT_RATE_BPS
	sfx_u8 idx = 0;
	sfx_bool bit_rate_valid = SFX_FALSE;
	// Check if the given value is allowed.
	for (idx=0 ; idx<SIGFOX_UL_BIT_RATE_LAST ; idx++) {
		if (SIGFOX_UL_BIT_RATE_BPS_LIST[idx] == UL_BIT_RATE_BPS) {
			bit_rate_valid = SFX_TRUE;
			break;
		}
	}
	if (bit_rate_valid == SFX_FALSE) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_BIT_RATE);
	}
	// Check RC capability.
	if ((((sigfox_ep_api_ctx.rc_ptr -> uplink_bit_rate_capability) >> idx) & 0x01) == 0) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_BIT_RATE);
	}
#else /* UL_BIT_RATE */
	// Check if the given bit rate exists.
	if ((common_params -> ul_bit_rate) >= SIGFOX_UL_BIT_RATE_LAST) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_BIT_RATE);
	}
	// Check RC capability.
	if ((((sigfox_ep_api_ctx.rc_ptr -> uplink_bit_rate_capability) >> (common_params -> ul_bit_rate)) & 0x01) == 0) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_BIT_RATE);
	}
#endif /* UL_BIT_RATE */
	// Check TX power regarding RC rule.
#ifdef TX_POWER_DBM_EIRP
	if (TX_POWER_DBM_EIRP > ((sigfox_ep_api_ctx.rc_ptr) -> tx_power_dbm_eirp_max)) {
#else
	if (((common_params -> tx_power_dbm_eirp)) > ((sigfox_ep_api_ctx.rc_ptr) -> tx_power_dbm_eirp_max)) {
#endif
		EXIT_ERROR(SIGFOX_EP_API_ERROR_TX_POWER);
	}
#ifndef SINGLE_FRAME
	// Check number of frames.
	if ((common_params -> number_of_frames) > SIGFOX_UL_FRAME_RANK_LAST) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NUMBER_OF_FRAMES);
	}
#ifndef T_IFU_MS
	// Check interframe delay.
	if ((common_params -> t_ifu_ms) > SIGFOX_T_IFU_MAX_MS) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_T_IFU);
	}
#endif
#endif /* MULTIPLE_FRAMES */
#ifdef PUBLIC_KEY_CAPABLE
	// Check key type.
	if ((common_params -> ep_key_type) >= SIGFOX_EP_KEY_LAST) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_EP_KEY);
	}
#endif
errors:
	return status;
}
#endif

#if (defined APPLICATION_MESSAGES) && (defined PARAMETERS_CHECK) && (defined ERROR_CODES)
/*******************************************************************/
static SIGFOX_EP_API_status_t _check_application_message(SIGFOX_EP_API_application_message_t *app_msg) {
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	// Check parameter.
	if (app_msg == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
#if !(defined UL_PAYLOAD_SIZE) || (UL_PAYLOAD_SIZE > 0)
	// Check payload.
	if ((app_msg -> type) == SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY) {
		// Payload is required.
		if (((app_msg -> ul_payload) == SFX_NULL)) {
			EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
		}
	}
#endif
	// Check message type.
	if ((app_msg -> type) >= SIGFOX_APPLICATION_MESSAGE_TYPE_LAST) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_MESSAGE_TYPE);
	}
#ifndef UL_PAYLOAD_SIZE
	if ((app_msg -> type) == SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY) {
		// Length is required.
		if (((app_msg -> ul_payload_size_bytes) == 0) || ((app_msg -> ul_payload_size_bytes) > SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES)) {
			EXIT_ERROR(SIGFOX_EP_API_ERROR_UL_PAYLOAD_SIZE);
		}
	}
#endif
#if (defined BIDIRECTIONAL) && !(defined T_CONF_MS)
	// Check downlink parameters.
	if ((app_msg -> bidirectional_flag) != 0) {
		// Check DL confirmation delay.
		if (((app_msg -> t_conf_ms) < SIGFOX_T_CONF_MIN_MS) || ((app_msg -> t_conf_ms) > SIGFOX_T_CONF_MAX_MS)) {
			EXIT_ERROR(SIGFOX_EP_API_ERROR_T_CONF);
		}
	}
#endif
errors:
	return status;
}
#endif

#if (defined CONTROL_KEEP_ALIVE_MESSAGE) && (defined PARAMETERS_CHECK) && (defined ERROR_CODES)
/*******************************************************************/
static SIGFOX_EP_API_status_t _check_control_message(SIGFOX_EP_API_control_message_t *ctrl_msg) {
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	// Check parameter.
	if (ctrl_msg == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
	// Check message type.
	if ((ctrl_msg -> type) >= SIGFOX_CONTROL_MESSAGE_TYPE_LAST) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_MESSAGE_TYPE);
	}
errors:
	return status;
}
#endif

#ifdef APPLICATION_MESSAGES
/*******************************************************************/
static SIGFOX_EP_API_status_t _store_application_message(SIGFOX_EP_API_application_message_t *application_message) {
	// Local variables.
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#if (defined ASYNCHRONOUS) && (((defined UL_PAYLOAD_SIZE) && (UL_PAYLOAD_SIZE > 0)) || !(defined UL_PAYLOAD_SIZE))
	sfx_u8 idx = 0;
#endif
#if (defined PARAMETERS_CHECK) && (defined ERROR_CODES)
	// Check parameters.
	status = _check_application_message(application_message);
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	status = _check_common_parameters(&(application_message -> common_parameters));
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#endif
#endif /* PARAMETERS_CHECK and ERROR_CODES*/
#ifdef ASYNCHRONOUS
	// In asynchronous mode, all the data has to be stored locally since the client pointer could be removed.
	// Common parameters.
#ifndef UL_BIT_RATE_BPS
	sigfox_ep_api_ctx.local_application_message.common_parameters.ul_bit_rate = ((application_message -> common_parameters).ul_bit_rate);
#endif
#ifndef TX_POWER_DBM_EIRP
	sigfox_ep_api_ctx.local_application_message.common_parameters.tx_power_dbm_eirp = ((application_message -> common_parameters).tx_power_dbm_eirp);
#endif
#ifndef SINGLE_FRAME
	// Number of frames and inter frame delay.
	sigfox_ep_api_ctx.local_application_message.common_parameters.number_of_frames = ((application_message -> common_parameters).number_of_frames);
#ifndef T_IFU_MS
	sigfox_ep_api_ctx.local_application_message.common_parameters.t_ifu_ms = ((application_message -> common_parameters).t_ifu_ms);
	sigfox_ep_api_ctx.interframe_ms = sigfox_ep_api_ctx.local_application_message.common_parameters.t_ifu_ms;
#else
	sigfox_ep_api_ctx.interframe_ms = T_IFU_MS;
#endif
#ifdef BIDIRECTIONAL
	// Force Tifb in case of bidirectional.
	if ((application_message -> bidirectional_flag) != 0) {
		sigfox_ep_api_ctx.interframe_ms = SIGFOX_T_IFB_MS;
	}
#endif
#endif /* SINGLE_FRAME */
#ifdef PUBLIC_KEY_CAPABLE
	sigfox_ep_api_ctx.local_application_message.common_parameters.ep_key_type = (application_message -> common_parameters).ep_key_type;
#endif
	// Message type.
	sigfox_ep_api_ctx.local_application_message.type = (application_message -> type);
	// Store callbacks (even if NULL).
	sigfox_ep_api_ctx.uplink_cplt_cb = (application_message -> uplink_cplt_cb);
	sigfox_ep_api_ctx.message_cplt_cb = (application_message -> message_cplt_cb);
#ifdef BIDIRECTIONAL
	sigfox_ep_api_ctx.downlink_cplt_cb = (application_message -> downlink_cplt_cb);
#endif
	// UL payload.
#ifdef UL_PAYLOAD_SIZE
#if (UL_PAYLOAD_SIZE > 0)
	for (idx=0 ; idx<UL_PAYLOAD_SIZE ; idx++) {
		sigfox_ep_api_ctx.local_ul_payload[idx] = (application_message -> ul_payload)[idx];
	}
#endif
#else /* UL_PAYLOAD_SIZE */
	if ((application_message -> type) == SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY) {
		for (idx=0 ; idx<(application_message -> ul_payload_size_bytes) ; idx++) {
			sigfox_ep_api_ctx.local_ul_payload[idx] = (application_message -> ul_payload)[idx];
		}
		sigfox_ep_api_ctx.local_application_message.ul_payload_size_bytes = (application_message -> ul_payload_size_bytes);
	}
	else {
		sigfox_ep_api_ctx.local_application_message.ul_payload_size_bytes = 0;
	}
#endif
#ifdef BIDIRECTIONAL
	// Downlink parameters.
	sigfox_ep_api_ctx.local_application_message.bidirectional_flag = (application_message -> bidirectional_flag);
#ifndef T_CONF_MS
	sigfox_ep_api_ctx.local_application_message.t_conf_ms = (application_message -> t_conf_ms);
#endif
#endif
	// Update pointers to local data.
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	sigfox_ep_api_ctx.common_parameters_ptr = &(sigfox_ep_api_ctx.local_application_message.common_parameters);
#endif
	sigfox_ep_api_ctx.application_message_ptr = &(sigfox_ep_api_ctx.local_application_message);
#if !(defined UL_PAYLOAD_SIZE) || (UL_PAYLOAD_SIZE > 0)
	(sigfox_ep_api_ctx.application_message_ptr) -> ul_payload = (sfx_u8*) sigfox_ep_api_ctx.local_ul_payload;
#endif
#else /* ASYNCHRONOUS */
	// In blocking mode, the message pointer will directly address the client data since it will be kept during processing.
	sigfox_ep_api_ctx.application_message_ptr = application_message;
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	sigfox_ep_api_ctx.common_parameters_ptr = &(application_message -> common_parameters);
#endif
#endif /* ASYNCHRONOUS */
#if (defined PARAMETERS_CHECK) && (defined ERROR_CODES)
errors:
#endif
	RETURN();
}
#endif

#ifdef CONTROL_KEEP_ALIVE_MESSAGE
/*******************************************************************/
static SIGFOX_EP_API_status_t _store_control_message(SIGFOX_EP_API_control_message_t *control_message) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#if (defined PARAMETERS_CHECK) && (defined ERROR_CODES)
	// Check parameters.
	status = _check_control_message(control_message);
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	status = _check_common_parameters(&(control_message -> common_parameters));
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#endif
#endif /* PARAMETERS_CHECK and ERROR_CODES */
#ifdef ASYNCHRONOUS
	// In asynchronous mode, all the data has to be stored locally since the client pointer could be removed, and the message pointer will address the local data.
	// Common parameters.
#ifndef UL_BIT_RATE_BPS
	sigfox_ep_api_ctx.local_control_message.common_parameters.ul_bit_rate = ((control_message -> common_parameters).ul_bit_rate);
#endif
#ifndef TX_POWER_DBM_EIRP
	sigfox_ep_api_ctx.local_control_message.common_parameters.tx_power_dbm_eirp = ((control_message -> common_parameters).tx_power_dbm_eirp);
#endif
#ifndef SINGLE_FRAME
	// Number of frames and inter frame delay.
	sigfox_ep_api_ctx.local_control_message.common_parameters.number_of_frames = ((control_message -> common_parameters).number_of_frames);
#ifndef T_IFU_MS
	sigfox_ep_api_ctx.local_control_message.common_parameters.t_ifu_ms = ((control_message -> common_parameters).t_ifu_ms);
	sigfox_ep_api_ctx.interframe_ms = sigfox_ep_api_ctx.local_control_message.common_parameters.t_ifu_ms;
#else
	sigfox_ep_api_ctx.interframe_ms = T_IFU_MS;
#endif
#endif /* SINGLE_FRAME */
#ifdef PUBLIC_KEY_CAPABLE
	sigfox_ep_api_ctx.local_application_message.common_parameters.ep_key_type = (control_message -> common_parameters).ep_key_type;
#endif
	// Message type.
	sigfox_ep_api_ctx.local_control_message.type = (control_message -> type);
	// Store callbacks (even if NULL).
	sigfox_ep_api_ctx.uplink_cplt_cb = (control_message -> uplink_cplt_cb);
	sigfox_ep_api_ctx.message_cplt_cb = (control_message -> message_cplt_cb);
	// Update pointer to local data.
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	sigfox_ep_api_ctx.common_parameters_ptr = &(sigfox_ep_api_ctx.local_control_message.common_parameters);
#endif
	sigfox_ep_api_ctx.control_message_ptr = &(sigfox_ep_api_ctx.local_control_message);
#else /* ASYNCHRONOUS */
	// In blocking mode, the message pointer will directly address the client data since it will be kept during processing.
	sigfox_ep_api_ctx.control_message_ptr = control_message;
#if !(defined SINGLE_FRAME) || !(defined UL_BIT_RATE_BPS) || !(defined TX_POWER_DBM_EIRP) || (defined PUBLIC_KEY_CAPABLE)
	sigfox_ep_api_ctx.common_parameters_ptr = &(control_message -> common_parameters);
#endif
#endif /* ASYNCHRONOUS */
#if (defined PARAMETERS_CHECK) && (defined ERROR_CODES)
errors:
#endif
	RETURN();
}
#endif

/*******************************************************************/
static SIGFOX_EP_API_status_t _compute_next_ul_frequency(void) {
	// Local variables.
	sfx_u32 ul_frequency_hz = 0;
	sfx_u16 random_value = 0;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	SIGFOX_EP_FREQUENCY_status_t frequency_status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#ifndef SINGLE_FRAME
	SIGFOX_EP_FREQUENCY_uplink_signal_t frequency_parameters;
#endif
	// Prepare frequency parameters.
#ifndef SINGLE_FRAME
	frequency_parameters.ul_frame_rank = sigfox_ep_api_ctx.ul_frame_rank;
#ifdef BIDIRECTIONAL
#ifdef APPLICATION_MESSAGES
	if (sigfox_ep_api_ctx.application_message_ptr == SFX_NULL) {
		frequency_parameters.bidirectional_flag = 0;
	}
	else {
		frequency_parameters.bidirectional_flag = (sigfox_ep_api_ctx.internal_flags.control_message != 0) ? 0 : ((sigfox_ep_api_ctx.application_message_ptr) -> bidirectional_flag);
	}
#else
	frequency_parameters.bidirectional_flag = 0;
#endif
#endif
#endif
	// Compute frequency.
#ifndef SINGLE_FRAME
#ifdef ERROR_CODES
	// Compute frequency.
	frequency_status = SIGFOX_EP_FREQUENCY_compute_uplink(&frequency_parameters, &ul_frequency_hz);
	SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_FREQUENCY);
#else
	SIGFOX_EP_FREQUENCY_compute_uplink(&frequency_parameters, &ul_frequency_hz);
#endif
#else /* MULTIPLE_FRAMES */
#ifdef ERROR_CODES
	// Compute frequency.
	frequency_status = SIGFOX_EP_FREQUENCY_compute_uplink(&ul_frequency_hz);
	SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_FREQUENCY);
#else
	SIGFOX_EP_FREQUENCY_compute_uplink(&ul_frequency_hz);
#endif
#endif /* MULTIPLE_FRAMES */
	// Update random value.
#ifdef ERROR_CODES
	frequency_status = SIGFOX_EP_FREQUENCY_get_random_value(&random_value);
	SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_FREQUENCY);
#else
	SIGFOX_EP_FREQUENCY_get_random_value(&random_value);
#endif
	sigfox_ep_api_ctx.random_value = random_value;
#ifdef CERTIFICATION
	// Bypass frequency if needed.
	if ((sigfox_ep_api_ctx.test_parameters.tx_frequency_hz) != 0) {
		ul_frequency_hz = sigfox_ep_api_ctx.test_parameters.tx_frequency_hz;
	}
#endif
	// Update global variables.
	sigfox_ep_api_ctx.ul_frequency_hz = ul_frequency_hz;
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}

#ifdef BIDIRECTIONAL
/*******************************************************************/
static sfx_bool _is_downlink_required(void) {
	// Local variables.
	sfx_bool downlink_required = SFX_FALSE;
#ifdef APPLICATION_MESSAGES
	// Check message pointer.
	if (sigfox_ep_api_ctx.application_message_ptr == SFX_NULL) goto errors;
	// Check bidirectional flag.
	if (((sigfox_ep_api_ctx.application_message_ptr -> bidirectional_flag) != 0) && (sigfox_ep_api_ctx.internal_flags.control_message == 0)) {
		downlink_required = SFX_TRUE;
	}
errors:
#endif
	return downlink_required;
}
#endif

/*******************************************************************/
static sfx_bool _is_last_frame_of_uplink_sequence(void) {
	// Local variables.
	sfx_bool last_uplink_frame = SFX_FALSE;
#ifdef SINGLE_FRAME
	last_uplink_frame = SFX_TRUE;
#else
	if (sigfox_ep_api_ctx.ul_frame_rank >= (((sigfox_ep_api_ctx.common_parameters_ptr) -> number_of_frames) - 1)) {
		last_uplink_frame = SFX_TRUE;
	}
#endif
	return last_uplink_frame;
}

#if (defined REGULATORY) || (defined ASYNCHRONOUS)
/*******************************************************************/
static sfx_bool _is_last_frame_of_message_sequence(void) {
	// Local variables.
	sfx_bool last_message_frame = SFX_FALSE;
	// Check flags.
#ifdef BIDIRECTIONAL
#ifdef APPLICATION_MESSAGES
	if ((sigfox_ep_api_ctx.application_message_ptr) == SFX_NULL) {
		// Do not check bidirectional flag.
		if ((sigfox_ep_api_ctx.internal_flags.ack_message != 0) || (_is_last_frame_of_uplink_sequence() == SFX_TRUE)) {
			last_message_frame = SFX_TRUE;
		}
	}
	else {
		// Check bidirectional flag.
		if ((sigfox_ep_api_ctx.internal_flags.ack_message != 0) || ((_is_last_frame_of_uplink_sequence() == SFX_TRUE) && (((sigfox_ep_api_ctx.application_message_ptr) -> bidirectional_flag) == 0))) {
			last_message_frame = SFX_TRUE;
		}
	}
#endif
#else
	last_message_frame = _is_last_frame_of_uplink_sequence();
#endif
	return last_message_frame;
}
#endif

/*******************************************************************/
static SIGFOX_EP_API_status_t _radio_wake_up(void) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	RF_API_status_t rf_status = RF_API_SUCCESS;
#endif
	// Wake-up radio if in sleep.
	if (sigfox_ep_api_ctx.internal_flags.radio_woken_up == 0) {
#ifdef ERROR_CODES
		rf_status = RF_API_wake_up();
		RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
		RF_API_wake_up();
#endif
		// Update flag.
		sigfox_ep_api_ctx.internal_flags.radio_woken_up = 1;
	}
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _radio_sleep(void) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	RF_API_status_t rf_status = RF_API_SUCCESS;
#endif
	// Turn radio off if active.
	if (sigfox_ep_api_ctx.internal_flags.radio_woken_up != 0) {
#ifdef ERROR_CODES
		rf_status = RF_API_sleep();
		RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
		RF_API_sleep();
#endif
		// Update flag.
		sigfox_ep_api_ctx.internal_flags.radio_woken_up = 0;
	}
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}

/*******************************************************************/
static void _set_common_bitstream_parameters(SIGFOX_EP_BITSTREAM_common_t *bitsteam_common_parameter_ptr, sfx_u8 *ep_id) {
	// EP ID and key.
	bitsteam_common_parameter_ptr -> ep_id = ep_id;
	// Message counter.
	bitsteam_common_parameter_ptr -> message_counter = sigfox_ep_api_ctx.message_counter;
#ifndef MESSAGE_COUNTER_ROLLOVER
	// Message counter rollover.
	bitsteam_common_parameter_ptr -> message_counter_rollover = sigfox_ep_api_ctx.message_counter_rollover;
#endif
#ifndef SINGLE_FRAME
	// Frame rank.
	bitsteam_common_parameter_ptr -> ul_frame_rank = sigfox_ep_api_ctx.ul_frame_rank;
#endif
#ifdef PUBLIC_KEY_CAPABLE
	bitsteam_common_parameter_ptr -> ep_key_type = (sigfox_ep_api_ctx.common_parameters_ptr) -> ep_key_type;
#endif
}

#ifdef REGULATORY
/*******************************************************************/
static void _set_tx_control_parameters(SIGFOX_TX_CONTROL_check_type check_type, SIGFOX_TX_CONTROL_parameters_t *tx_control_params) {
	// Update TX control parameters.
	tx_control_params -> type = check_type;
	tx_control_params -> bitstream_length_bytes = SIGFOX_EP_BITSTREAM_SIZE_BYTES; // TODO dynamic computation.
	tx_control_params -> last_message_frame = _is_last_frame_of_message_sequence();
#ifndef UL_BIT_RATE_BPS
	tx_control_params -> ul_bit_rate_bps = SIGFOX_UL_BIT_RATE_BPS_LIST[(sigfox_ep_api_ctx.common_parameters_ptr) -> ul_bit_rate];
#endif
#ifndef SINGLE_FRAME
	tx_control_params -> ul_frame_rank = sigfox_ep_api_ctx.ul_frame_rank;
	tx_control_params -> number_of_frames = (sigfox_ep_api_ctx.common_parameters_ptr) -> number_of_frames;
#endif
#ifdef BIDIRECTIONAL
	tx_control_params -> ack_message = sigfox_ep_api_ctx.internal_flags.ack_message;
#endif
#if !(defined SINGLE_FRAME) || (defined BIDIRECTIONAL)
	tx_control_params -> interframe_ms = sigfox_ep_api_ctx.interframe_ms; // Tifu, Tifb or Tconf.
#endif
#ifdef CERTIFICATION
#ifdef SPECTRUM_ACCESS_FH
	tx_control_params -> fh_timer_enable = sigfox_ep_api_ctx.test_parameters.fh_timer_enable;
#endif
#ifdef SPECTRUM_ACCESS_LBT
	tx_control_params -> lbt_enable = sigfox_ep_api_ctx.test_parameters.lbt_enable;
#endif
#endif
#ifdef ASYNCHRONOUS
	tx_control_params -> cplt_cb = (SIGFOX_TX_CONTROL_check_cplt_cb_t) ((check_type == SIGFOX_TX_CONTROL_TYPE_PRE_CHECK) ? &_SIGFOX_TX_CONTROL_pre_check_cplt_cb : &_SIGFOX_TX_CONTROL_post_check_cplt_cb);
#endif
}
#endif

/*******************************************************************/
static SIGFOX_EP_API_status_t _send_frame(sfx_u8 *bitstream, sfx_u8 bitstream_size) {
	// Local variables.
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	RF_API_status_t rf_status = RF_API_SUCCESS;
#endif
	RF_API_radio_parameters_t radio_params;
	RF_API_tx_data_t tx_data;
	// Build radio parameters.
	radio_params.rf_mode = RF_API_MODE_TX;
	radio_params.modulation = RF_API_MODULATION_DBPSK;
	radio_params.frequency_hz = sigfox_ep_api_ctx.ul_frequency_hz;
#ifdef UL_BIT_RATE_BPS
	radio_params.bit_rate_bps = UL_BIT_RATE_BPS;
#else
	radio_params.bit_rate_bps = SIGFOX_UL_BIT_RATE_BPS_LIST[(sigfox_ep_api_ctx.common_parameters_ptr) -> ul_bit_rate];
#endif
#ifdef TX_POWER_DBM_EIRP
	radio_params.tx_power_dbm_eirp = TX_POWER_DBM_EIRP;
#else
	radio_params.tx_power_dbm_eirp = (sigfox_ep_api_ctx.common_parameters_ptr) -> tx_power_dbm_eirp;
#endif
#ifdef BIDIRECTIONAL
	radio_params.deviation_hz = 0;
#endif
	// Wake-up radio.
#ifdef ERROR_CODES
	status = _radio_wake_up();
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_radio_wake_up();
#endif
	// Start radio.
#ifdef ERROR_CODES
	rf_status = RF_API_init(&radio_params);
	RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
	RF_API_init(&radio_params);
#endif
	// Send frame.
	tx_data.bitstream = (sfx_u8*) bitstream;
	tx_data.bitstream_size_bytes = bitstream_size;
#ifdef ASYNCHRONOUS
	tx_data.cplt_cb = (RF_API_tx_cplt_cb_t) &_RF_API_tx_cplt_cb;
#endif
#ifdef ERROR_CODES
	rf_status = RF_API_send(&tx_data);
	RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
	RF_API_send(&tx_data);
#endif
#ifndef ASYNCHRONOUS
	// Note: thanks to the RF_API_check_status the next lines are not executed in case of RF API error.
	// Increment success count.
	sigfox_ep_api_ctx.internal_flags.frame_success = 1;
#ifndef SINGLE_FRAME
	sigfox_ep_api_ctx.frame_success_count++;
#endif
#endif /* BLOCKING */
#ifdef ERROR_CODES
errors:
#endif
#ifndef ASYNCHRONOUS
	sigfox_ep_api_ctx.irq_flags.rf_tx_cplt = 1; // Set flag manually in blocking mode.
#endif
	RETURN();
}

#ifdef APPLICATION_MESSAGES
/*******************************************************************/
static SIGFOX_EP_API_status_t _send_application_frame(void) {
	// Local variables.
	sfx_u8 idx = 0;
	sfx_u8 ep_id[SIGFOX_EP_ID_SIZE_BYTES];
	SIGFOX_EP_BITSTREAM_application_frame_t bitstream_parameters;
	sfx_u8 bitstream[SIGFOX_EP_BITSTREAM_SIZE_BYTES];
	sfx_u8 bitstream_size;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	SIGFOX_EP_BITSTREAM_status_t bitstream_status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
	// Create local EP-ID.
	for (idx=0 ; idx<SIGFOX_EP_ID_SIZE_BYTES ; idx++) ep_id[idx] = sigfox_ep_api_ctx.ep_id[idx];
	// Prepare common bitstream parameters.
	_set_common_bitstream_parameters(&(bitstream_parameters.common_parameters), ep_id);
	// Prepare specific bitstream parameters for application frame.
	bitstream_parameters.message_type = ((sigfox_ep_api_ctx.application_message_ptr) -> type);
#if !(defined UL_PAYLOAD_SIZE) || (UL_PAYLOAD_SIZE > 0)
	bitstream_parameters.ul_payload = ((sigfox_ep_api_ctx.application_message_ptr) -> ul_payload);
#endif
#ifndef UL_PAYLOAD_SIZE
	bitstream_parameters.ul_payload_size_bytes = ((sigfox_ep_api_ctx.application_message_ptr) -> ul_payload_size_bytes);
#endif
#ifdef BIDIRECTIONAL
	bitstream_parameters.bidirectional_flag = ((sigfox_ep_api_ctx.application_message_ptr) -> bidirectional_flag);
#endif
	// Build frame.
#ifdef ERROR_CODES
	bitstream_status = SIGFOX_EP_BITSTREAM_build_application_frame(&bitstream_parameters, bitstream, &bitstream_size);
	SIGFOX_EP_BITSTREAM_check_status(SIGFOX_EP_API_ERROR_BITSTREAM);
#else
	SIGFOX_EP_BITSTREAM_build_application_frame(&bitstream_parameters, bitstream, &bitstream_size);
#endif
	// Send frame.
#ifdef ERROR_CODES
	status = _send_frame(bitstream, bitstream_size);
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_send_frame(bitstream, bitstream_size);
#endif
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
/*******************************************************************/
static SIGFOX_EP_API_status_t _read_mcu_voltage_temperature(void) {
	// Local variables.
	sfx_s16 temperature_tenth_degrees = 0;
	sfx_u16 voltage_tx_mv = 0;
	sfx_u16 voltage_idle_mv = 0;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
#endif
#ifdef SINGLE_FRAME
	// Read MCU data.
#ifdef ERROR_CODES
	mcu_status = MCU_API_get_voltage_temperature(&voltage_idle_mv, &voltage_tx_mv, &temperature_tenth_degrees);
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
	MCU_API_get_voltage_temperature(&voltage_idle_mv, &voltage_tx_mv, &temperature_tenth_degrees);
#endif
	// Update global variables.
	sigfox_ep_api_ctx.voltage_idle_mv = voltage_idle_mv;
	sigfox_ep_api_ctx.voltage_tx_mv = voltage_tx_mv;
	sigfox_ep_api_ctx.temperature_tenth_degrees = temperature_tenth_degrees;
#else /* SINGLE_FRAME */
	if (sigfox_ep_api_ctx.ul_frame_rank == SIGFOX_UL_FRAME_RANK_1) {
		// Read MCU data.
#ifdef ERROR_CODES
		mcu_status = MCU_API_get_voltage_temperature(&voltage_idle_mv, &voltage_tx_mv, &temperature_tenth_degrees);
		MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
		MCU_API_get_voltage_temperature(&voltage_idle_mv, &voltage_tx_mv, &temperature_tenth_degrees);
#endif
		// Update global variables.
		sigfox_ep_api_ctx.voltage_idle_mv = voltage_idle_mv;
		sigfox_ep_api_ctx.voltage_tx_mv = voltage_tx_mv;
		sigfox_ep_api_ctx.temperature_tenth_degrees = temperature_tenth_degrees;
	}
#endif /* SINGLE_FRAME */
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

#ifdef CONTROL_KEEP_ALIVE_MESSAGE
/*******************************************************************/
static SIGFOX_EP_API_status_t _send_control_keep_alive_frame(void) {
	// Local variables.
	sfx_u8 idx = 0;
	sfx_u8 ep_id[SIGFOX_EP_ID_SIZE_BYTES];
	SIGFOX_EP_BITSTREAM_control_frame_t bitstream_parameters;
	sfx_u8 bitstream[SIGFOX_EP_BITSTREAM_SIZE_BYTES];
	sfx_u8 bitstream_size;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	SIGFOX_EP_BITSTREAM_status_t bitstream_status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
	// Create local EP-ID.
	for (idx=0 ; idx<SIGFOX_EP_ID_SIZE_BYTES ; idx++) ep_id[idx] = sigfox_ep_api_ctx.ep_id[idx];
	// Read MCU data.
#ifdef ERROR_CODES
	status = _read_mcu_voltage_temperature();
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_read_mcu_voltage_temperature();
#endif
	// Prepare common bitstream parameters.
	_set_common_bitstream_parameters(&(bitstream_parameters.common_parameters), ep_id);
	// Prepare specific bitstream parameters for control keep-alive.
	bitstream_parameters.message_type = SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE;
	bitstream_parameters.voltage_idle_mv = sigfox_ep_api_ctx.voltage_idle_mv;
	bitstream_parameters.voltage_tx_mv = sigfox_ep_api_ctx.voltage_tx_mv;
	bitstream_parameters.temperature_tenth_degrees = sigfox_ep_api_ctx.temperature_tenth_degrees;
#ifdef BIDIRECTIONAL
	bitstream_parameters.rssi_dbm = 0; // Unused.
#endif
	// Build frame.
#ifdef ERROR_CODES
	bitstream_status = SIGFOX_EP_BITSTREAM_build_control_frame(&bitstream_parameters, bitstream, &bitstream_size);
	SIGFOX_EP_BITSTREAM_check_status(SIGFOX_EP_API_ERROR_BITSTREAM);
#else
	SIGFOX_EP_BITSTREAM_build_control_frame(&bitstream_parameters, bitstream, &bitstream_size);
#endif
	// Send frame.
#ifdef ERROR_CODES
	status = _send_frame(bitstream, bitstream_size);
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_send_frame(bitstream, bitstream_size);
#endif
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

#ifdef BIDIRECTIONAL
/*******************************************************************/
static SIGFOX_EP_API_status_t _send_dl_confirmation_frame(void) {
	// Local variables.
	sfx_u8 idx = 0;
	sfx_u8 ep_id[SIGFOX_EP_ID_SIZE_BYTES];
	SIGFOX_EP_BITSTREAM_control_frame_t bitstream_parameters;
	sfx_u8 bitstream[SIGFOX_EP_BITSTREAM_SIZE_BYTES];
	sfx_u8 bitstream_size;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	SIGFOX_EP_BITSTREAM_status_t bitstream_status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
	// Create local EP-ID.
	for (idx=0 ; idx<SIGFOX_EP_ID_SIZE_BYTES ; idx++) ep_id[idx] = sigfox_ep_api_ctx.ep_id[idx];
	// Read MCU data.
#ifdef ERROR_CODES
	status = _read_mcu_voltage_temperature();
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_read_mcu_voltage_temperature();
#endif
	// Prepare common bitstream parameters.
	_set_common_bitstream_parameters(&(bitstream_parameters.common_parameters), ep_id);
	// Prepare specific bitstream parameters for DL confirmation.
	bitstream_parameters.message_type = SIGFOX_CONTROL_MESSAGE_TYPE_DL_CONFIRMATION;
	bitstream_parameters.voltage_idle_mv = sigfox_ep_api_ctx.voltage_idle_mv;
	bitstream_parameters.voltage_tx_mv = sigfox_ep_api_ctx.voltage_tx_mv;
	bitstream_parameters.temperature_tenth_degrees = sigfox_ep_api_ctx.temperature_tenth_degrees;
	bitstream_parameters.rssi_dbm = sigfox_ep_api_ctx.dl_rssi_dbm;
	// Build frame.
#ifdef ERROR_CODES
	bitstream_status = SIGFOX_EP_BITSTREAM_build_control_frame(&bitstream_parameters, bitstream, &bitstream_size);
	SIGFOX_EP_BITSTREAM_check_status(SIGFOX_EP_API_ERROR_BITSTREAM);
#else
	SIGFOX_EP_BITSTREAM_build_control_frame(&bitstream_parameters, bitstream, &bitstream_size);
#endif
	// Send frame.
#ifdef ERROR_CODES
	status = _send_frame(bitstream, bitstream_size);
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_send_frame(bitstream, bitstream_size);
#endif
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

#ifdef BIDIRECTIONAL
/*******************************************************************/
static SIGFOX_EP_API_status_t _check_dl_frame(void) {
	// Local variables.
	sfx_u8 idx = 0;
	sfx_u8 ep_id[SIGFOX_EP_ID_SIZE_BYTES];
	sfx_u8 dl_phy_content[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES];
	sfx_u8 dl_payload[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
	sfx_s16 dl_rssi_dbm = 0;
	SIGFOX_EP_BITSTREAM_dl_frame_t bitstream_parameters;
	sfx_bool dl_status = SFX_FALSE;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	RF_API_status_t rf_status = RF_API_SUCCESS;
	SIGFOX_EP_BITSTREAM_status_t bitstream_status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
	// Create local EP-ID.
	for (idx=0 ; idx<SIGFOX_EP_ID_SIZE_BYTES ; idx++) ep_id[idx] = sigfox_ep_api_ctx.ep_id[idx];
	// Read DL-PHY content received by the radio.
#ifdef ERROR_CODES
	rf_status = RF_API_get_dl_phy_content_and_rssi(dl_phy_content, SIGFOX_DL_PHY_CONTENT_SIZE_BYTES, &dl_rssi_dbm);
	RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
	RF_API_get_dl_phy_content_and_rssi(dl_phy_content, SIGFOX_DL_PHY_CONTENT_SIZE_BYTES, &dl_rssi_dbm);
#endif
	// Prepare bitstream parameters.
	bitstream_parameters.dl_phy_content = (sfx_u8*) dl_phy_content;
	bitstream_parameters.ep_id = (sfx_u8*) ep_id;
	bitstream_parameters.message_counter = sigfox_ep_api_ctx.message_counter;
#ifdef PUBLIC_KEY_CAPABLE
	bitstream_parameters.ep_key_type = (sigfox_ep_api_ctx.common_parameters_ptr) -> ep_key_type;
#endif
	// Check DL frame.
#ifdef ERROR_CODES
	bitstream_status = SIGFOX_EP_BITSTREAM_decode_downlink_frame(&bitstream_parameters, &dl_status, (sfx_u8*) dl_payload);
	SIGFOX_EP_BITSTREAM_check_status(SIGFOX_EP_API_ERROR_BITSTREAM);
#else
	SIGFOX_EP_BITSTREAM_decode_downlink_frame(&bitstream_parameters, &dl_status, (sfx_u8*) dl_payload);
#endif
	// Update data in global context.
	if (dl_status == SFX_TRUE) {
		for (idx=0 ; idx<SIGFOX_DL_PAYLOAD_SIZE_BYTES ; idx++) sigfox_ep_api_ctx.dl_payload[idx] = dl_payload[idx];
		sigfox_ep_api_ctx.dl_rssi_dbm = dl_rssi_dbm;
	}
	sigfox_ep_api_ctx.dl_status = dl_status;
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

#if (!(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0))) || (defined BIDIRECTIONAL)
/*******************************************************************/
static SIGFOX_EP_API_status_t _start_timer1(sfx_u16 duration_ms) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
#endif
	MCU_API_timer_t mcu_timer;
	// Start timer.
	mcu_timer.instance = MCU_API_TIMER_1;
	mcu_timer.duration_ms = duration_ms;
#ifdef ASYNCHRONOUS
	mcu_timer.cplt_cb = (MCU_API_timer_cplt_cb_t) &_MCU_API_timer1_cplt_cb;
#endif
#ifdef ERROR_CODES
	mcu_status = MCU_API_timer_start(&mcu_timer);
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
	MCU_API_timer_start(&mcu_timer);
#endif
#ifdef ERROR_CODES
errors:
	return status;
#endif
	RETURN();
}
#endif

#ifdef BIDIRECTIONAL
/*******************************************************************/
static SIGFOX_EP_API_status_t _start_timer2(sfx_u16 duration_ms) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
#endif
	MCU_API_timer_t mcu_timer;
	// Start timer.
	mcu_timer.instance = MCU_API_TIMER_2;
	mcu_timer.duration_ms = duration_ms;
#ifdef ASYNCHRONOUS
	mcu_timer.cplt_cb = (MCU_API_timer_cplt_cb_t) &_MCU_API_timer2_cplt_cb;
#endif
	// Start timer.
#ifdef ERROR_CODES
	mcu_status = MCU_API_timer_start(&mcu_timer);
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
	MCU_API_timer_start(&mcu_timer);
#endif
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

/*******************************************************************/
static SIGFOX_EP_API_status_t _end_transmission(void);

/*******************************************************************/
static SIGFOX_EP_API_status_t _start_transmission(void) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#ifdef REGULATORY
	SIGFOX_TX_CONTROL_status_t tx_control_status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#endif
#ifdef REGULATORY
	SIGFOX_TX_CONTROL_parameters_t tx_control_params;
	SIGFOX_TX_CONTROL_result_t tx_control_pre_check_result = SIGFOX_TX_CONTROL_RESULT_FORBIDDEN;
#endif
	// Reset TX success flag.
	sigfox_ep_api_ctx.internal_flags.frame_success = 0;
	// Compute frequency.
	// This must be done here since the frame 1 frequency has to be known for eventual bidirectional procedure, even if the frame itself is not sent (due to error or TX control).
#ifdef ERROR_CODES
	status = _compute_next_ul_frequency();
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_compute_next_ul_frequency();
#endif
#ifdef REGULATORY
	// Check if radio is required for pre-check.
	if (SIGFOX_TX_CONTROL_is_radio_required(SIGFOX_TX_CONTROL_TYPE_PRE_CHECK) == SFX_TRUE) {
#ifdef ERROR_CODES
		status = _radio_wake_up();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_radio_wake_up();
#endif
	}
	_set_tx_control_parameters(SIGFOX_TX_CONTROL_TYPE_PRE_CHECK, &tx_control_params);
	// Start TX control.
#ifdef ERROR_CODES
	tx_control_status = SIGFOX_TX_CONTROL_check(&tx_control_params);
	SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_TX_CONTROL);
#else
	SIGFOX_TX_CONTROL_check(&tx_control_params);
#endif
	sigfox_ep_api_ctx.internal_flags.tx_control_pre_check_running = 1;
#ifndef ASYNCHRONOUS
	sigfox_ep_api_ctx.irq_flags.tx_control_pre_check_cplt = 1; // Set flag manually in blocking mode.
#endif
	// Read result.
#ifdef ERROR_CODES
	tx_control_status = SIGFOX_TX_CONTROL_get_result(&tx_control_pre_check_result);
	SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_TX_CONTROL);
#else
	SIGFOX_TX_CONTROL_get_result(&tx_control_pre_check_result);
#endif
	// Update state according to result.
	switch (tx_control_pre_check_result) {
	case SIGFOX_TX_CONTROL_RESULT_ALLOWED:
		// Send frame.
#ifdef ERROR_CODES
		status = sigfox_ep_api_ctx.sending_function_ptr();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		sigfox_ep_api_ctx.sending_function_ptr();
#endif
		// Update state.
		sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_UL_MODULATION_PENDING;
		break;
	case SIGFOX_TX_CONTROL_RESULT_FORBIDDEN:
		// Set error flags.
		sigfox_ep_api_ctx.message_status.error = 1;
		sigfox_ep_api_ctx.internal_flags.tx_forbidden = 1;
#ifdef ERROR_STACK
		// Add error to stack.
		SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_EP_LIBRARY, SIGFOX_EP_API_ERROR_TX_FORBIDDEN);
#endif
		// Try next frame.
#ifdef ERROR_CODES
		status = _end_transmission();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_end_transmission();
#endif
		break;
	case SIGFOX_TX_CONTROL_RESULT_PENDING:
		// Wait for completion.
		sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_REGULATORY;
		break;
	default:
		EXIT_ERROR(SIGFOX_EP_API_ERROR_TX_CONTROL);
		break;
	}
#else /* REGULATORY */
	// Send frame.
#ifdef ERROR_CODES
	status = sigfox_ep_api_ctx.sending_function_ptr();
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	sigfox_ep_api_ctx.sending_function_ptr();
#endif
	// Update state.
	sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_UL_MODULATION_PENDING;
#endif /* REGULATORY */
#if (defined ERROR_CODES) || (defined REGULATORY)
errors:
#endif
	RETURN();
}

/*******************************************************************/
static void _update_message_status(void) {
#ifdef SINGLE_FRAME
#ifdef BIDIRECTIONAL
	// Update message status.
	if (sigfox_ep_api_ctx.internal_flags.ack_message != 0) {
		sigfox_ep_api_ctx.message_status.ack_frame = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
	}
	else {
		sigfox_ep_api_ctx.message_status.app_frame_1 = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
	}
#else /* BIDIRECTIONAL */
	// Update message status.
	sigfox_ep_api_ctx.message_status.app_frame_1 = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
#endif /* BIDIRECTIONAL */
#else /* SINGLE_FRAME */
#ifdef BIDIRECTIONAL
	// Update message status.
	if (sigfox_ep_api_ctx.internal_flags.ack_message != 0) {
		sigfox_ep_api_ctx.message_status.ack_frame = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
	}
	else {
		switch (sigfox_ep_api_ctx.ul_frame_rank) {
		case SIGFOX_UL_FRAME_RANK_1:
			sigfox_ep_api_ctx.message_status.app_frame_1 = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
			break;
		case SIGFOX_UL_FRAME_RANK_2:
			sigfox_ep_api_ctx.message_status.app_frame_2 = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
			break;
		case SIGFOX_UL_FRAME_RANK_3:
			sigfox_ep_api_ctx.message_status.app_frame_3 = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
			break;
		default:
			break;
		}
	}
#else /* BIDIRECTIONAL */
	switch (sigfox_ep_api_ctx.ul_frame_rank) {
	case SIGFOX_UL_FRAME_RANK_1:
		sigfox_ep_api_ctx.message_status.app_frame_1 = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
		break;
	case SIGFOX_UL_FRAME_RANK_2:
		sigfox_ep_api_ctx.message_status.app_frame_2 = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
		break;
	case SIGFOX_UL_FRAME_RANK_3:
		sigfox_ep_api_ctx.message_status.app_frame_3 = (sigfox_ep_api_ctx.internal_flags.frame_success != 0) ? 1 : 0;
		break;
	default:
		break;
	}
#endif /* BIDIRECTIONAL */
#endif /* SINGLE_FRAME */
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _write_nvm(void) {
	// Local variable.
	sfx_u8 nvm_data[SIGFOX_NVM_DATA_SIZE_BYTES];
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
#endif
	// Write message counter and random value in NVM when at least one frame has been successfully transmitted.
#ifdef SINGLE_FRAME
	if (sigfox_ep_api_ctx.internal_flags.frame_success != 0) {
#else
	if (sigfox_ep_api_ctx.frame_success_count > 0) {
#endif
		// Build local NVM array.
		nvm_data[SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_MSB] = (sfx_u8) ((sigfox_ep_api_ctx.message_counter >> 8) & 0x00FF);
		nvm_data[SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_LSB] = (sfx_u8) ((sigfox_ep_api_ctx.message_counter >> 0) & 0x00FF);
		nvm_data[SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_MSB] = (sfx_u8) ((sigfox_ep_api_ctx.random_value >> 8) & 0x00FF);
		nvm_data[SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_LSB] = (sfx_u8) ((sigfox_ep_api_ctx.random_value >> 0) & 0x00FF);
		// Write NVM.
#ifdef ERROR_CODES
		mcu_status = MCU_API_set_nvm(nvm_data, SIGFOX_NVM_DATA_SIZE_BYTES);
		MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
		MCU_API_set_nvm(nvm_data, SIGFOX_NVM_DATA_SIZE_BYTES);
#endif
		// Update flags.
		sigfox_ep_api_ctx.internal_flags.nvm_write_pending = 0;
	}
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _compute_state_after_transmission(void) {
	// Local variables.
	SIGFOX_EP_API_state_t state = SIGFOX_EP_API_STATE_READY;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#if (defined BIDIRECTIONAL) && !(defined SINGLE_FRAME)
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
#endif
#endif
#ifdef SINGLE_FRAME
#ifdef BIDIRECTIONAL
	// Compute next state.
	if (_is_downlink_required() == SFX_TRUE) {
		// Abort downlink sequence if the frame has not been sent due to error or TX control.
		if (sigfox_ep_api_ctx.internal_flags.frame_success == 0) {
			// Force bidirectional to 0 in order to call the message completion callback.
			if (sigfox_ep_api_ctx.application_message_ptr != SFX_NULL) {
				(sigfox_ep_api_ctx.application_message_ptr) -> bidirectional_flag = 0;
			}
			// Force state to ready.
			state = SIGFOX_EP_API_STATE_READY;
		}
		else {
			// Start DL_T_W timer.
#ifdef ERROR_CODES
			status = _start_timer2(((sigfox_ep_api_ctx.rc_ptr) -> spectrum_access) -> dl_t_w_ms);
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_timer2(((sigfox_ep_api_ctx.rc_ptr) -> spectrum_access) -> dl_t_w_ms);
#endif
			state = SIGFOX_EP_API_STATE_DL_TIMER;
		}
	}
	else {
		state = SIGFOX_EP_API_STATE_READY;
	}
#else /* BIDIRECTIONAL */
	// Compute next state.
	state = SIGFOX_EP_API_STATE_READY;
#endif /* BIDIRECTIONAL */
#else /* SINGLE_FRAME */
#ifdef BIDIRECTIONAL
	// Check bidirectional flag.
	if ((sigfox_ep_api_ctx.ul_frame_rank == SIGFOX_UL_FRAME_RANK_1) && (_is_downlink_required() == SFX_TRUE)) {
		// Start DL_T_W timer.
#ifdef ERROR_CODES
		status = _start_timer2(((sigfox_ep_api_ctx.rc_ptr) -> spectrum_access) -> dl_t_w_ms);
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_start_timer2(((sigfox_ep_api_ctx.rc_ptr) -> spectrum_access) -> dl_t_w_ms);
#endif
	}
	// Compute next state.
	if (_is_last_frame_of_uplink_sequence() == SFX_TRUE) {
		// Check bidirectional condition.
		if (_is_downlink_required() == SFX_TRUE) {
			// Abort downlink sequence if none frame has been sent due to error or TX control.
			if (sigfox_ep_api_ctx.frame_success_count == 0) {
				// Stop DL_T_W timer.
#ifdef ERROR_CODES
				mcu_status = MCU_API_timer_stop(MCU_API_TIMER_2);
				MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
				MCU_API_timer_stop(MCU_API_TIMER_2);
#endif
				// Force bidirectional to 0 in order to call the message completion callback.
				if (sigfox_ep_api_ctx.application_message_ptr != SFX_NULL) {
					(sigfox_ep_api_ctx.application_message_ptr) -> bidirectional_flag = 0;
				}
				// Force state to ready.
				state = SIGFOX_EP_API_STATE_READY;
			}
			else {
				state = SIGFOX_EP_API_STATE_DL_TIMER;
			}
		}
		else {
			state = SIGFOX_EP_API_STATE_READY;
		}
	}
	else {
		// Compute next state.
		if (_is_downlink_required() == SFX_TRUE) {
			sigfox_ep_api_ctx.interframe_ms = SIGFOX_T_IFB_MS;
			state = SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER;
		}
		else {
			// Compute inter-frame duration.
#ifdef T_IFU_MS
			sigfox_ep_api_ctx.interframe_ms = T_IFU_MS;
#else
			sigfox_ep_api_ctx.interframe_ms = ((sigfox_ep_api_ctx.common_parameters_ptr) -> t_ifu_ms);
#endif
		}
		// Start inter-frame timer if required.
		if ((sigfox_ep_api_ctx.interframe_ms) != 0) {
#ifdef ERROR_CODES
			status = _start_timer1(sigfox_ep_api_ctx.interframe_ms);
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_timer1(sigfox_ep_api_ctx.interframe_ms);
#endif
			// Update state.
			state = SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER;
		}
		else {
			// Inter-frame delay set to 0, directly start next frame.
#ifdef ERROR_CODES
			status = _start_transmission();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_transmission();
#endif
		}
	}
#else /* BIDIRECTIONAL */
	// Compute next state.
	if (_is_last_frame_of_uplink_sequence() == SFX_TRUE) {
		state = SIGFOX_EP_API_STATE_READY;
	}
	else {
		// Compute inter-frame duration.
#ifdef T_IFU_MS
		sigfox_ep_api_ctx.interframe_ms = T_IFU_MS;
#if (T_IFU_MS > 0)
		// Start inter-frame timer.
#ifdef ERROR_CODES
		status = _start_timer1(sigfox_ep_api_ctx.interframe_ms);
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_start_timer1(sigfox_ep_api_ctx.interframe_ms);
#endif
		// Update state.
		state = SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER;
#else /* T_IFU_MS > 0 */
		// Inter-frame delay set to 0, directly start next frame.
#ifdef ERROR_CODES
		status = _start_transmission();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_start_transmission();
#endif
#endif /* T_IFU_MS > 0 */
#else /* T_IFU_MS */
		sigfox_ep_api_ctx.interframe_ms = ((sigfox_ep_api_ctx.common_parameters_ptr) -> t_ifu_ms);
		// Start inter-frame timer if required.
		if ((sigfox_ep_api_ctx.interframe_ms) != 0) {
#ifdef ERROR_CODES
			status = _start_timer1(sigfox_ep_api_ctx.interframe_ms);
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_timer1(sigfox_ep_api_ctx.interframe_ms);
#endif
			// Update state.
			state = SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER;
		}
		else {
			// Inter-frame delay set to 0, directly start next frame.
#ifdef ERROR_CODES
			status = _start_transmission();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_transmission();
#endif
		}
#endif /* T_IFU_MS */
	}
#endif /* BIDIRECTIONAL */
#endif /* SINGLE_FRAME */
#ifdef ASYNCHRONOUS
	// Manage API callbacks.
	if ((_is_last_frame_of_uplink_sequence() == SFX_TRUE) && (sigfox_ep_api_ctx.internal_flags.ack_message == 0)) {
		// Call uplink completion callback (except for ACK message).
		_UPLINK_CPLT_CALLBACK();
	}
	if (_is_last_frame_of_message_sequence() == SFX_TRUE) {
		// Call message completion callback.
		_MESSAGE_CPLT_CALLBACK();
	}
#endif /* ASYNCHRONOUS */
#ifndef SINGLE_FRAME
	// Increment frame rank.
	sigfox_ep_api_ctx.ul_frame_rank++;
#endif
	// Update global state.
	sigfox_ep_api_ctx.state = state;
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _end_transmission(void) {
	// Local variables.
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	RF_API_status_t rf_status = RF_API_SUCCESS;
#ifdef REGULATORY
	SIGFOX_TX_CONTROL_status_t tx_control_status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#endif
#ifdef REGULATORY
	SIGFOX_TX_CONTROL_parameters_t tx_control_params;
	SIGFOX_TX_CONTROL_result_t tx_control_post_check_result = SIGFOX_TX_CONTROL_RESULT_FORBIDDEN;
#endif
	// Update message status.
	_update_message_status();
	// Stop RF.
	// Note: if TX control returned forbidden result, the RF_API_de_init function was already called by the TX control itself.
	if (sigfox_ep_api_ctx.internal_flags.frame_success != 0) {
#ifdef ERROR_CODES
		rf_status = RF_API_de_init();
		RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
		RF_API_de_init();
#endif
	}
	// Manage radio state and NVM.
	if (_is_last_frame_of_uplink_sequence() == SFX_TRUE) {
#ifdef REGULATORY
		if (SIGFOX_TX_CONTROL_is_radio_required(SIGFOX_TX_CONTROL_TYPE_POST_CHECK) == SFX_FALSE) {
#ifdef ERROR_CODES
			status = _radio_sleep();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_radio_sleep();
#endif
		}
#else /* REGULATORY */
#ifdef ERROR_CODES
		status = _radio_sleep();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_radio_sleep();
#endif
#endif /* REGULATORY */
		// Write NVM.
#ifdef ERROR_CODES
		status = _write_nvm();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_write_nvm();
#endif
	}
#ifdef REGULATORY
	// Set parameters.
	_set_tx_control_parameters(SIGFOX_TX_CONTROL_TYPE_POST_CHECK, &tx_control_params);
#ifdef ERROR_CODES
	tx_control_status = SIGFOX_TX_CONTROL_check(&tx_control_params);
	SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_TX_CONTROL);
#else
	SIGFOX_TX_CONTROL_check(&tx_control_params);
#endif
	sigfox_ep_api_ctx.internal_flags.tx_control_post_check_running = 1;
#ifndef ASYNCHRONOUS
	sigfox_ep_api_ctx.irq_flags.tx_control_post_check_cplt = 1; // Set flag manually in blocking mode.
#endif
	// Read result.
#ifdef ERROR_CODES
	tx_control_status = SIGFOX_TX_CONTROL_get_result(&tx_control_post_check_result);
	SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_TX_CONTROL);
#else
	SIGFOX_TX_CONTROL_get_result(&tx_control_post_check_result);
#endif
	// Update state according to result.
	switch (tx_control_post_check_result) {
	case SIGFOX_TX_CONTROL_RESULT_FORBIDDEN:
		sigfox_ep_api_ctx.message_status.error = 1;
		sigfox_ep_api_ctx.internal_flags.tx_forbidden = 1;
#ifdef ERROR_STACK
		// Add error to stack.
		SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_EP_LIBRARY, SIGFOX_EP_API_ERROR_TX_FORBIDDEN);
#endif
		// Note: no break since "forbidden" and "allowed" cases are treated the same way (except for error flag).
	case SIGFOX_TX_CONTROL_RESULT_ALLOWED:
		// Compute next state.
#ifdef ERROR_CODES
		status = _compute_state_after_transmission();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_compute_state_after_transmission();
#endif
		break;
	case SIGFOX_TX_CONTROL_RESULT_PENDING:
		// Wait for completion.
		sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_REGULATORY;
		break;
	default:
		EXIT_ERROR(SIGFOX_EP_API_ERROR_TX_CONTROL);
		break;
	}
#else /* REGULATORY */
	// Compute next state.
#ifdef ERROR_CODES
	status = _compute_state_after_transmission();
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_compute_state_after_transmission();
#endif
#endif /* REGULATORY */
#if (defined ERROR_CODES) || (defined REGULATORY)
errors:
#endif
	RETURN();
}

#ifdef BIDIRECTIONAL
/*******************************************************************/
static SIGFOX_EP_API_status_t _start_reception(RF_API_rx_data_t *rx_data_ptr) {
	// Local variables.
	RF_API_radio_parameters_t radio_params;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	RF_API_status_t rf_status = RF_API_SUCCESS;
	SIGFOX_EP_FREQUENCY_status_t frequency_status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
	// Set radio configuration for downlink reception.
	radio_params.rf_mode = RF_API_MODE_RX;
	radio_params.modulation = RF_API_MODULATION_GFSK;
	radio_params.bit_rate_bps = SIGFOX_DL_BIT_RATE_BPS;
	radio_params.deviation_hz = SIGFOX_DL_GFSK_DEVIATION_HZ;
	// Set uplink frequency.
#ifdef ERROR_CODES
	frequency_status = SIGFOX_EP_FREQUENCY_compute_downlink(&radio_params.frequency_hz);
	SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_FREQUENCY);
#else
	SIGFOX_EP_FREQUENCY_compute_downlink(&radio_params.frequency_hz);
#endif
#ifdef ERROR_CODES
	rf_status = RF_API_init(&radio_params);
	RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
	RF_API_init(&radio_params);
#endif
	// Start DL reception.
#ifdef ERROR_CODES
	rf_status = RF_API_receive(rx_data_ptr);
	RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
	RF_API_receive(rx_data_ptr);
#endif
#ifdef ERROR_CODES
errors:
#endif
	RETURN();
}
#endif

/*******************************************************************/
static void _message_prepare(void) {
	// Increment message counter.
	// Note: operation is only performed in RAM first, it will be write in NVM when at least 1 frame has been successfully transmitted.
	// Note: the nvm_write_pending flag is checked to prevent multiple increments if the previous value was not written in NVM (due to TX control forbidden or error).
	if (sigfox_ep_api_ctx.internal_flags.nvm_write_pending == 0) {
#ifdef MESSAGE_COUNTER_ROLLOVER
		sigfox_ep_api_ctx.message_counter = (sigfox_ep_api_ctx.message_counter + 1) % MESSAGE_COUNTER_ROLLOVER;
#else
		sigfox_ep_api_ctx.message_counter = (sigfox_ep_api_ctx.message_counter + 1) % (sigfox_ep_api_ctx.message_counter_rollover);
#endif
		// Update flag.
		sigfox_ep_api_ctx.internal_flags.nvm_write_pending = 1;
	}
	sigfox_ep_api_ctx.internal_flags.tx_forbidden = 0;
#ifndef SINGLE_FRAME
	// Reset frame rank.
	sigfox_ep_api_ctx.ul_frame_rank = SIGFOX_UL_FRAME_RANK_1;
	sigfox_ep_api_ctx.frame_success_count = 0;
#endif
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _internal_process(void) {
	// Local variables.
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#if (defined ASYNCHRONOUS) || (defined BIDIRECTIONAL)
	RF_API_status_t rf_status = RF_API_SUCCESS;
#endif
#if (defined ASYNCHRONOUS) || (defined BIDIRECTIONAL) || (!(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0)))
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
#endif
#ifdef REGULATORY
	SIGFOX_TX_CONTROL_status_t tx_control_status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#endif /* ERROR_CODES */
#ifdef BIDIRECTIONAL
	RF_API_rx_data_t rx_data;
#endif
#ifdef REGULATORY
	SIGFOX_TX_CONTROL_result_t tx_control_result = SIGFOX_TX_CONTROL_RESULT_FORBIDDEN;
#endif
	// Prepare RX data structure.
#ifdef BIDIRECTIONAL
	rx_data.dl_phy_content_size = SIGFOX_DL_PHY_CONTENT_SIZE_BYTES;
#ifdef ASYNCHRONOUS
	rx_data.data_received_cb = (RF_API_rx_data_received_cb_t) &_RF_API_rx_data_received_cb;
#else
	rx_data.data_received = SFX_FALSE;
#endif
#endif
	// Check library is opened.
	_CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
#ifdef ASYNCHRONOUS
	// Check error flag in case the process was called after a low level error callback.
	if (sigfox_ep_api_ctx.irq_flags.low_level_error != 0) {
		// Clear flag.
		sigfox_ep_api_ctx.irq_flags.low_level_error = 0;
		// Exit.
		EXIT_ERROR(sigfox_ep_api_ctx.status_from_callback);
	}
#endif
	// Check low level process flags.
#ifdef ASYNCHRONOUS
    if (sigfox_ep_api_ctx.irq_flags.mcu_process != 0) {
#ifdef ERROR_CODES
        mcu_status = MCU_API_process();
        sigfox_ep_api_ctx.irq_flags.mcu_process = 0;
        MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
        MCU_API_process();
        sigfox_ep_api_ctx.irq_flags.mcu_process = 0;
#endif
    }
	if (sigfox_ep_api_ctx.irq_flags.rf_process != 0) {
#ifdef ERROR_CODES
		rf_status = RF_API_process();
		sigfox_ep_api_ctx.irq_flags.rf_process = 0;
	    RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
	    RF_API_process();
	    sigfox_ep_api_ctx.irq_flags.rf_process = 0;
#endif
	}
#ifdef REGULATORY
	if (sigfox_ep_api_ctx.irq_flags.tx_control_process != 0) {
#ifdef ERROR_CODES
		tx_control_status = SIGFOX_TX_CONTROL_process();
		sigfox_ep_api_ctx.irq_flags.tx_control_process = 0;
		SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_TX_CONTROL);
#else
		SIGFOX_TX_CONTROL_process();
		sigfox_ep_api_ctx.irq_flags.tx_control_process = 0;
#endif
	}
#endif
#endif
	// Perform internal state machine.
	switch (sigfox_ep_api_ctx.state) {
	case SIGFOX_EP_API_STATE_READY:
		// Check pending requests.
		if (sigfox_ep_api_ctx.internal_flags.send_message_request != 0) {
#ifdef ERROR_CODES
			status = _start_transmission();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_transmission();
#endif
			// Clear request.
			sigfox_ep_api_ctx.internal_flags.send_message_request = 0;
		}
		break;
#ifdef REGULATORY
	case SIGFOX_EP_API_STATE_REGULATORY:
		// Check completion flags.
		if ((sigfox_ep_api_ctx.irq_flags.tx_control_post_check_cplt != 0) || (sigfox_ep_api_ctx.irq_flags.tx_control_pre_check_cplt != 0)) {
			// Check pre-check flags.
			if (sigfox_ep_api_ctx.irq_flags.tx_control_pre_check_cplt != 0) {
				// Clear flags.
				sigfox_ep_api_ctx.irq_flags.tx_control_pre_check_cplt = 0;
				sigfox_ep_api_ctx.internal_flags.tx_control_pre_check_running = 0;
				// Read result.
#ifdef ERROR_CODES
				tx_control_status = SIGFOX_TX_CONTROL_get_result(&tx_control_result);
				SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_TX_CONTROL);
#else
				SIGFOX_TX_CONTROL_get_result(&tx_control_result);
#endif
				// Check TX control status.
				if (tx_control_result == SIGFOX_TX_CONTROL_RESULT_ALLOWED) {
					// Send frame.
#ifdef ERROR_CODES
					status = sigfox_ep_api_ctx.sending_function_ptr();
					CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
					sigfox_ep_api_ctx.sending_function_ptr();
#endif
					// Update state.
					sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_UL_MODULATION_PENDING;
				}
				else {
					// Set error flags.
					sigfox_ep_api_ctx.message_status.error = 1;
					sigfox_ep_api_ctx.internal_flags.tx_forbidden = 1;
#ifdef ERROR_STACK
					// Add error to stack.
					SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_EP_LIBRARY, SIGFOX_EP_API_ERROR_TX_FORBIDDEN);
#endif
					// Compute next state.
#ifdef ERROR_CODES
					status = _end_transmission();
					CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
					_end_transmission();
#endif
				}
			}
			// Check post-check flag.
			if (sigfox_ep_api_ctx.irq_flags.tx_control_post_check_cplt != 0) {
				// Clear flags.
				sigfox_ep_api_ctx.irq_flags.tx_control_post_check_cplt = 0;
				sigfox_ep_api_ctx.internal_flags.tx_control_post_check_running = 0;
				// Read result.
#ifdef ERROR_CODES
				tx_control_status = SIGFOX_TX_CONTROL_get_result(&tx_control_result);
				SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_TX_CONTROL);
#else
				SIGFOX_TX_CONTROL_get_result(&tx_control_result);
#endif
				// Set error flags if needed.
				if (tx_control_result == SIGFOX_TX_CONTROL_RESULT_FORBIDDEN) {
					sigfox_ep_api_ctx.message_status.error = 1;
					sigfox_ep_api_ctx.internal_flags.tx_forbidden = 1;
#ifdef ERROR_STACK
					// Add error to stack.
					SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_EP_LIBRARY, SIGFOX_EP_API_ERROR_TX_FORBIDDEN);
#endif
				}
				// Switch to next state whatever the result.
#ifdef ERROR_CODES
				status = _compute_state_after_transmission();
				CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
				_compute_state_after_transmission();
#endif
			}
		}
		break;
#endif
	case SIGFOX_EP_API_STATE_UL_MODULATION_PENDING:
		// Check end of transmission flag..
		if (sigfox_ep_api_ctx.irq_flags.rf_tx_cplt != 0) {
			// Clear flag.
			sigfox_ep_api_ctx.irq_flags.rf_tx_cplt = 0;
#ifdef ASYNCHRONOUS
			// Increment success count.
			sigfox_ep_api_ctx.internal_flags.frame_success = 1;
#ifndef SINGLE_FRAME
			sigfox_ep_api_ctx.frame_success_count++;
#endif
#endif
			// End transmission.
#ifdef ERROR_CODES
			status = _end_transmission();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_end_transmission();
#endif
		}
		break;
#if !(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0) || (defined BIDIRECTIONAL))
	case SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER:
#ifndef ASYNCHRONOUS
		// Wait for timer completion.
#ifdef ERROR_CODES
		mcu_status = MCU_API_timer_wait_cplt(MCU_API_TIMER_1);
		MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
		MCU_API_timer_wait_cplt(MCU_API_TIMER_1);
#endif
		sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt = 1; // Set flag manually in blocking mode.
#endif
		// Check timer completion flag.
		if (sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt != 0) {
			// Stop timer.
#ifdef ERROR_CODES
			mcu_status = MCU_API_timer_stop(MCU_API_TIMER_1);
			MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
			MCU_API_timer_stop(MCU_API_TIMER_1);
#endif
			sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt = 0;
			// Start next frame.
#ifdef ERROR_CODES
			status = _start_transmission();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_transmission();
#endif
		}
		break;
#endif
#ifdef BIDIRECTIONAL
	case SIGFOX_EP_API_STATE_DL_TIMER:
#ifndef ASYNCHRONOUS
		// Wait for timer completion.
#ifdef ERROR_CODES
		mcu_status = MCU_API_timer_wait_cplt(MCU_API_TIMER_2);
		MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
		MCU_API_timer_wait_cplt(MCU_API_TIMER_2);
#endif
		sigfox_ep_api_ctx.irq_flags.mcu_timer2_cplt = 1; // Set flag manually in blocking mode.
#endif /* BLOCKING */
		// Check timer completion flag.
		if (sigfox_ep_api_ctx.irq_flags.mcu_timer2_cplt != 0) {
			// Stop timer.
#ifdef ERROR_CODES
			mcu_status = MCU_API_timer_stop(MCU_API_TIMER_2);
			MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
			MCU_API_timer_stop(MCU_API_TIMER_2);
#endif
			sigfox_ep_api_ctx.irq_flags.mcu_timer2_cplt = 0;
			// Start DL_T_RX timer.
#ifdef ERROR_CODES
			status = _start_timer1(((sigfox_ep_api_ctx.rc_ptr) -> spectrum_access) -> dl_t_rx_ms);
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_timer1(((sigfox_ep_api_ctx.rc_ptr) -> spectrum_access) -> dl_t_rx_ms);
#endif
			// Wake-up radio.
#ifdef ERROR_CODES
			status = _radio_wake_up();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_radio_wake_up();
#endif
#ifdef ASYNCHRONOUS
			// Start DL reception.
#ifdef ERROR_CODES
			status = _start_reception(&rx_data);
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_reception(&rx_data);
#endif
#endif /* ASYNCHRONOUS */
			// Update state.
			sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_DL_LISTENING;
		}
		break;
	case SIGFOX_EP_API_STATE_DL_LISTENING:
#ifndef ASYNCHRONOUS
		// Wait for incoming DL frame or timeout.
#ifdef ERROR_CODES
		status = _start_reception(&rx_data);
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_start_reception(&rx_data);
#endif
		// If the function exits with dl_frame_received flag set to 0, it means that the DL window timer has elapsed.
		sigfox_ep_api_ctx.irq_flags.rf_rx_data_received = rx_data.data_received;
		sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt = !(rx_data.data_received);
#endif /* BLOCKING */
		if (sigfox_ep_api_ctx.irq_flags.rf_rx_data_received != 0) {
			// Clear flag.
			sigfox_ep_api_ctx.irq_flags.rf_rx_data_received = 0;
			// Decode downlink frame.
#ifdef ERROR_CODES
			status = _check_dl_frame();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_check_dl_frame();
#endif
			// Restart reception if the frame authentication failed.
			if ((sigfox_ep_api_ctx.dl_status) == SFX_FALSE) {
#ifdef ERROR_CODES
				rf_status = RF_API_de_init();
				RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
				RF_API_de_init();
#endif
#ifdef ASYNCHRONOUS
#ifdef ERROR_CODES
				status = _start_reception(&rx_data);
				CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
				_start_reception(&rx_data);
#endif
#endif /* ASYNCHRONOUS */
			}
		}
		if (((sigfox_ep_api_ctx.dl_status) == SFX_TRUE) || (sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt != 0)) {
			// Stop timer.
#ifdef ERROR_CODES
			mcu_status = MCU_API_timer_stop(MCU_API_TIMER_1);
			MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
			MCU_API_timer_stop(MCU_API_TIMER_1);
#endif
			sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt = 0;
			// Stop radio.
#ifdef ERROR_CODES
			rf_status = RF_API_de_init();
			RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
			RF_API_de_init();
#endif
			// RX sequence done.
#ifdef ERROR_CODES
			status = _radio_sleep();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_radio_sleep();
#endif
			// Update state.
			if ((sigfox_ep_api_ctx.dl_status) == SFX_TRUE) {
				// Update message status.
				sigfox_ep_api_ctx.message_status.downlink_frame = 1;
#ifdef ASYNCHRONOUS
				_DOWNLINK_CPLT_CALLBACK();
#endif
				// Start DL confirmation timer.
#ifdef T_CONF_MS
				sigfox_ep_api_ctx.interframe_ms = T_CONF_MS;
#else
				sigfox_ep_api_ctx.interframe_ms = (sigfox_ep_api_ctx.application_message_ptr) -> t_conf_ms;
#endif
#ifdef ERROR_CODES
				status = _start_timer1(sigfox_ep_api_ctx.interframe_ms);
				CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
				_start_timer1(sigfox_ep_api_ctx.interframe_ms);
#endif
				sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_DL_CONFIRMATION_TIMER;
			}
			else {
				// Dowlink timeout: set error flag.
				sigfox_ep_api_ctx.message_status.error = 1;
#ifdef ASYNCHRONOUS
				_MESSAGE_CPLT_CALLBACK();
#endif
				// Force state to ready.
				sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_READY;
			}
		}
		break;
	case SIGFOX_EP_API_STATE_DL_CONFIRMATION_TIMER:
#ifndef ASYNCHRONOUS
		// Wait for timer completion.
#ifdef ERROR_CODES
		mcu_status = MCU_API_timer_wait_cplt(MCU_API_TIMER_1);
		MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
		MCU_API_timer_wait_cplt(MCU_API_TIMER_1);
#endif
		sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt = 1; // Set flag manually in blocking mode.
#endif
		// Check timer completion flag.
		if (sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt != 0) {
			// Stop timer.
#ifdef ERROR_CODES
			mcu_status = MCU_API_timer_stop(MCU_API_TIMER_1);
			MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
			MCU_API_timer_stop(MCU_API_TIMER_1);
#endif
			sigfox_ep_api_ctx.irq_flags.mcu_timer1_cplt = 0;
			// Configure DL confirmation message.
			_message_prepare();
			sigfox_ep_api_ctx.internal_flags.control_message = 1;
			sigfox_ep_api_ctx.internal_flags.ack_message = 1;
#ifndef SINGLE_FRAME
			(sigfox_ep_api_ctx.common_parameters_ptr) -> number_of_frames = 1;
#endif
			// Update send function pointer.
			sigfox_ep_api_ctx.sending_function_ptr = &_send_dl_confirmation_frame;
			// Start confirmation frame.
#ifdef ERROR_CODES
			status = _start_transmission();
			CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
			_start_transmission();
#endif
		}
		break;
#endif
	default:
		// Unknown state.
		EXIT_ERROR(SIGFOX_EP_API_ERROR_STATE);
	}
	// Return here if no error occurred.
	RETURN();
error_state:
errors:
#ifdef ERROR_CODES
	// Notify error to low level drivers.
	MCU_API_error();
	RF_API_error();
#endif
	// Do not set the error flag and call the message completion callback on the first process call.
	if (sigfox_ep_api_ctx.internal_flags.send_message_request == 0) {
		// Set error flag and call completion callback.
		sigfox_ep_api_ctx.message_status.error = 1;
#ifdef ASYNCHRONOUS
		_MESSAGE_CPLT_CALLBACK();
#endif
	}
	// Force state to ready after error.
	sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_READY;
	RETURN();
}

#ifdef APPLICATION_MESSAGES
/*******************************************************************/
SIGFOX_EP_API_status_t _send_application_message(SIGFOX_EP_API_application_message_t *application_message) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
	// Check library state.
	_CHECK_LIBRARY_STATE(!= SIGFOX_EP_API_STATE_READY);
	// Reset message status.
	sigfox_ep_api_ctx.message_status.all = 0;
	// Reset IRQ flags.
	sigfox_ep_api_ctx.irq_flags.all = 0;
	// Store application message parameters locally.
#ifdef ERROR_CODES
	status = _store_application_message(application_message);
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_store_application_message(application_message);
#endif
	// Reset context.
	_message_prepare();
	// Set internal flags.
	sigfox_ep_api_ctx.internal_flags.send_message_request = 1;
	sigfox_ep_api_ctx.internal_flags.control_message = 0;
	sigfox_ep_api_ctx.internal_flags.ack_message = 0;
	// Set sending function pointer.
	sigfox_ep_api_ctx.sending_function_ptr = &_send_application_frame;
	// Trigger TX.
#ifdef ERROR_CODES
	status = _internal_process();
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_internal_process();
#endif
#ifdef ASYNCHRONOUS
	if (sigfox_ep_api_ctx.internal_flags.synchronous != 0) {
#endif
	// Block until library goes back to READY state.
	while (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY) {
#ifdef ERROR_CODES
		status = _internal_process();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_internal_process();
#endif
	}
#ifdef ASYNCHRONOUS
	}
#endif
#if !(defined ASYNCHRONOUS) && (defined REGULATORY) && (defined ERROR_CODES)
	// Force status to error if any frame has not been sent due to TX control.
	// Note: other errors are caught in the errors label.
	if (sigfox_ep_api_ctx.internal_flags.tx_forbidden != 0) {
		status = SIGFOX_EP_API_ERROR_TX_FORBIDDEN;
	}
#endif /* REGULATORY and BLOCKING */
	// Return here if no error occurred.
	sigfox_ep_api_ctx.internal_flags.send_message_request = 0;
	RETURN();
#ifdef ERROR_CODES
errors:
	// Set error flag except for state error (in order to keep the message status if a message sequence is ongoing).
	sigfox_ep_api_ctx.message_status.error = 1;
#endif
error_state:
	// Clear request.
	sigfox_ep_api_ctx.internal_flags.send_message_request = 0;
	RETURN();
}
#endif

#ifdef CONTROL_KEEP_ALIVE_MESSAGE
/*******************************************************************/
SIGFOX_EP_API_status_t _send_control_message(SIGFOX_EP_API_control_message_t *control_message) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
	// Check library state.
	_CHECK_LIBRARY_STATE(!= SIGFOX_EP_API_STATE_READY);
	// Reset message status.
	sigfox_ep_api_ctx.message_status.all = 0;
	// Reset IRQ flags.
	sigfox_ep_api_ctx.irq_flags.all = 0;
	// Store control message parameters locally.
#ifdef ERROR_CODES
	status = _store_control_message(control_message);
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_store_control_message(control_message);
#endif
	// Reset context.
	_message_prepare();
	/// Set internal flags.
	sigfox_ep_api_ctx.internal_flags.send_message_request = 1;
	sigfox_ep_api_ctx.internal_flags.control_message = 1;
	sigfox_ep_api_ctx.internal_flags.ack_message = 0;
	// Set sending function pointer.
	switch (control_message -> type) {
#ifdef CONTROL_KEEP_ALIVE_MESSAGE
	case SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE:
		sigfox_ep_api_ctx.sending_function_ptr = &_send_control_keep_alive_frame;
		break;
#endif
	default:
		// Downlink confirmation is not allowed.
		EXIT_ERROR(SIGFOX_EP_API_ERROR_MESSAGE_TYPE);
	}
	// Trigger TX.
#ifdef ERROR_CODES
	status = _internal_process();
	CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
	_internal_process();
#endif
#ifdef ASYNCHRONOUS
	if (sigfox_ep_api_ctx.internal_flags.synchronous != 0) {
#endif
	// Block until library goes back to READY state.
	while (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY) {
#ifdef ERROR_CODES
		status = _internal_process();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_internal_process();
#endif
	}
#ifdef ASYNCHRONOUS
	}
#endif
#if !(defined ASYNCHRONOUS) && (defined REGULATORY) && (defined ERROR_CODES)
	// Force status to error if any frame has not been sent due to TX control.
	// Note: other errors are caught in the errors label.
	if (sigfox_ep_api_ctx.internal_flags.tx_forbidden != 0) {
		status = SIGFOX_EP_API_ERROR_TX_FORBIDDEN;
	}
#endif /* REGULATORY and BLOCKING */
	// Return here if no error occurred.
	sigfox_ep_api_ctx.internal_flags.send_message_request = 0;
	RETURN();
errors:
	// Set error flag except for state error (in order to keep the message status if a message sequence is ongoing).
	sigfox_ep_api_ctx.message_status.error = 1;
error_state:
	// Clear request.
	sigfox_ep_api_ctx.internal_flags.send_message_request = 0;
	RETURN();
}
#endif

/*** SIGFOX EP API functions ***/

/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_open(SIGFOX_EP_API_config_t *config) {
	// Local variables.
	sfx_u8 idx = 0;
	sfx_u8 ep_id[SIGFOX_EP_ID_SIZE_BYTES];
	sfx_u8 nvm_data[SIGFOX_NVM_DATA_SIZE_BYTES];
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
	SIGFOX_EP_FREQUENCY_status_t frequency_status = SIGFOX_EP_FREQUENCY_SUCCESS;
#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
	RF_API_status_t rf_status = RF_API_SUCCESS;
#endif
#ifdef REGULATORY
	SIGFOX_TX_CONTROL_status_t tx_control_status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#endif /* ERROR_CODES */
#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
	MCU_API_config_t mcu_config;
	RF_API_config_t rf_config;
#endif
#ifdef REGULATORY
	SIGFOX_TX_CONTROL_config_t tx_control_config;
#endif
#if (defined MESSAGE_COUNTER_ROLLOVER) && (defined PARAMETERS_CHECK)
	sfx_bool message_counter_rollover_valid = SFX_FALSE;
#endif
#ifdef ERROR_STACK
	// Init error stack.
	SIGFOX_ERROR_init();
	sigfox_ep_api_ctx.internal_flags.error_stack_initialized = 1;
#endif
	// Check state.
	_CHECK_LIBRARY_STATE(!= SIGFOX_EP_API_STATE_CLOSED);
#ifdef PARAMETERS_CHECK
	// Check parameter.
	if (config == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
#endif
	// Check (even if PARAMETERS_CHECK flag is disabled) and store RC.
	if ((config -> rc) == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_RC);
	}
	sigfox_ep_api_ctx.rc_ptr = (config -> rc);
#ifdef MESSAGE_COUNTER_ROLLOVER
#ifdef PARAMETERS_CHECK
	// Check message counter macro value.
	for (idx=0 ; idx<SIGFOX_MESSAGE_COUNTER_ROLLOVER_LAST ; idx++) {
		if (SIGFOX_MESSAGE_COUNTER_ROLLOVER_LIST[idx] == MESSAGE_COUNTER_ROLLOVER) {
			message_counter_rollover_valid = SFX_TRUE;
			break;
		}
	}
	if (message_counter_rollover_valid == SFX_FALSE) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_MESSAGE_COUNTER_ROLLOVER);
	}
#endif /* PARAMETERS_CHECK */
#else /* MESSAGE_COUNTER_ROLLOVER */
#ifdef PARAMETERS_CHECK
	// Check message counter rollover index.
	if ((config -> message_counter_rollover) >= SIGFOX_MESSAGE_COUNTER_ROLLOVER_LAST) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_MESSAGE_COUNTER_ROLLOVER);
	}
#endif /* PARAMETERS_CHECK */
	// Store message counter rollover.
	sigfox_ep_api_ctx.message_counter_rollover = SIGFOX_MESSAGE_COUNTER_ROLLOVER_LIST[config -> message_counter_rollover];
#endif  /* MESSAGE_COUNTER_ROLLOVER */
	// Init MCU API.
#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
	mcu_config.rc = sigfox_ep_api_ctx.rc_ptr;
#ifdef ASYNCHRONOUS
	mcu_config.process_cb = (MCU_API_process_cb_t) &_MCU_API_process_cb;
	mcu_config.error_cb = (MCU_API_error_cb_t) &_MCU_API_error_cb;
#endif
#ifdef ERROR_CODES
	mcu_status = MCU_API_open(&mcu_config);
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
	MCU_API_open(&mcu_config);
#endif
#endif
	// Init RF API.
#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
	rf_config.rc = sigfox_ep_api_ctx.rc_ptr;
#ifdef ASYNCHRONOUS
	rf_config.process_cb = (RF_API_process_cb_t) &_RF_API_process_cb;
	rf_config.error_cb = &_RF_API_error_cb;
#endif
#ifdef ERROR_CODES
	rf_status = RF_API_open(&rf_config);
	RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
	RF_API_open(&rf_config);
#endif
#endif
#ifdef REGULATORY
	// Init TX CONTROL driver.
	tx_control_config.rc = sigfox_ep_api_ctx.rc_ptr;
#ifdef ASYNCHRONOUS
	tx_control_config.process_cb = &_SIGFOX_TX_CONTROL_process_cb;
#endif
#ifdef ERROR_CODES
	tx_control_status = SIGFOX_TX_CONTROL_open(&tx_control_config);
	SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_TX_CONTROL);
#else
	SIGFOX_TX_CONTROL_open(&tx_control_config);
#endif
#endif /* REGULATORY */
#ifdef ASYNCHRONOUS
	// Store process callback (even if NULL).
	sigfox_ep_api_ctx.process_cb = (config -> process_cb);
	// Update library behavior.
	sigfox_ep_api_ctx.internal_flags.synchronous = (sigfox_ep_api_ctx.process_cb == SFX_NULL) ? 1 : 0;
#endif
	// Read last message counter and last random value value in NVM.
#ifdef ERROR_CODES
	mcu_status = MCU_API_get_nvm(nvm_data, SIGFOX_NVM_DATA_SIZE_BYTES);
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
	MCU_API_get_nvm(nvm_data, SIGFOX_NVM_DATA_SIZE_BYTES);
#endif
	sigfox_ep_api_ctx.message_counter = 0;
	sigfox_ep_api_ctx.message_counter |= ((((sfx_u16) (nvm_data[SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_MSB])) << 8) & 0xFF00);
	sigfox_ep_api_ctx.message_counter |= ((((sfx_u16) (nvm_data[SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_LSB])) << 0) & 0x00FF);
	sigfox_ep_api_ctx.random_value = 0;
	sigfox_ep_api_ctx.random_value |= ((((sfx_u16) (nvm_data[SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_MSB])) << 8) & 0xFF00);
	sigfox_ep_api_ctx.random_value |= ((((sfx_u16) (nvm_data[SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_LSB])) << 0) & 0x00FF);
	// Read device ID.
#ifdef ERROR_CODES
	mcu_status = MCU_API_get_ep_id(ep_id, SIGFOX_EP_ID_SIZE_BYTES);
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
	MCU_API_get_ep_id(ep_id, SIGFOX_EP_ID_SIZE_BYTES);
#endif
	for (idx=0 ; idx<SIGFOX_EP_ID_SIZE_BYTES ; idx++) {
		sigfox_ep_api_ctx.ep_id[idx] = ep_id[idx];
	}
	// Init frequency driver.
#ifdef ERROR_CODES
	frequency_status = SIGFOX_EP_FREQUENCY_init(sigfox_ep_api_ctx.rc_ptr, ep_id, sigfox_ep_api_ctx.random_value);
	SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_FREQUENCY);
#else
	SIGFOX_EP_FREQUENCY_init(sigfox_ep_api_ctx.rc_ptr, ep_id, sigfox_ep_api_ctx.random_value);
#endif
	// Update library state if no error occured.
	sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_READY;
error_state:
errors:
	RETURN();
}

/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_close(void) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#ifdef LOW_LEVEL_OPEN_CLOSE
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
	RF_API_status_t rf_status = RF_API_SUCCESS;
#endif
#endif
	// Check library is opened.
	_CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
#ifdef LOW_LEVEL_OPEN_CLOSE
	// Close MCU.
#ifdef ERROR_CODES
	mcu_status = MCU_API_close();
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
	MCU_API_close();
#endif
	// Close RF.
#ifdef ERROR_CODES
	rf_status = RF_API_close();
	RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
	RF_API_close();
#endif
#endif
	// Update library state if no error occured.
	sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_CLOSED;
error_state:
#if (defined LOW_LEVEL_OPEN_CLOSE) && (defined ERROR_CODES)
errors:
#endif
	RETURN();
}

#ifdef ASYNCHRONOUS
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_process(void) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
	// Set process flag.
	sigfox_ep_api_ctx.internal_flags.process_running = 1;
	// Run the internal process.
	while (sigfox_ep_api_ctx.irq_flags.all != 0) {
#ifdef ERROR_CODES
		status = _internal_process();
		CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
		_internal_process();
#endif
	}
#ifdef ERROR_CODES
errors:
#endif
	// Reset process flag.
	sigfox_ep_api_ctx.internal_flags.process_running = 0;
	RETURN();
}
#endif

#ifdef APPLICATION_MESSAGES
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_send_application_message(SIGFOX_EP_API_application_message_t *application_message) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef CERTIFICATION
	// Disable all test parameters.
	sigfox_ep_api_ctx.test_parameters.tx_frequency_hz = 0;
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	sigfox_ep_api_ctx.test_parameters.fh_timer_enable = SFX_TRUE;
#endif
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
	sigfox_ep_api_ctx.test_parameters.lbt_enable = SFX_TRUE;
#endif
#endif
	// Send message.
#ifdef ERROR_CODES
	status = _send_application_message(application_message);
#else
	_send_application_message(application_message);
#endif
	RETURN();
}
#endif

#if (defined CERTIFICATION) && (defined APPLICATION_MESSAGES)
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_TEST_send_application_message(SIGFOX_EP_API_application_message_t *application_message, SIGFOX_EP_API_TEST_parameters_t *test_parameters) {
	// Local variables.
	sfx_u8 idx = 0;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	// Check parameters.
	if (test_parameters == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
#endif
	// Check EP-ID.
	for (idx=0 ; idx<SIGFOX_EP_ID_SIZE_BYTES ; idx++) {
		if (sigfox_ep_api_ctx.ep_id[idx] != SIGFOX_EP_TEST_ID[idx]) {
			EXIT_ERROR(SIGFOX_EP_API_ERROR_EP_ID);
		}
	}
	// Update local test parameters.
	sigfox_ep_api_ctx.test_parameters.tx_frequency_hz = (test_parameters -> tx_frequency_hz);
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	sigfox_ep_api_ctx.test_parameters.fh_timer_enable = (test_parameters -> fh_timer_enable);
#endif
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
	sigfox_ep_api_ctx.test_parameters.lbt_enable = (test_parameters -> lbt_enable);
#endif
	// Send message.
#ifdef ERROR_CODES
	status = _send_application_message(application_message);
#else
	_send_application_message(application_message);
#endif
errors:
	RETURN();
}
#endif

#ifdef CONTROL_KEEP_ALIVE_MESSAGE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_send_control_message(SIGFOX_EP_API_control_message_t *control_message) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef CERTIFICATION
	// Disable all test parameters.
	sigfox_ep_api_ctx.test_parameters.tx_frequency_hz = 0;
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	sigfox_ep_api_ctx.test_parameters.fh_timer_enable = SFX_TRUE;
#endif
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
	sigfox_ep_api_ctx.test_parameters.lbt_enable = SFX_TRUE;
#endif
#endif
	// Send message.
#ifdef ERROR_CODES
	status = _send_control_message(control_message);
#else
	_send_control_message(control_message);
#endif
	RETURN();
}
#endif

#if (defined CERTIFICATION) && (defined CONTROL_KEEP_ALIVE_MESSAGE)
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_TEST_send_control_message(SIGFOX_EP_API_control_message_t *control_message, SIGFOX_EP_API_TEST_parameters_t *test_parameters) {
	// Local variables.
	sfx_u8 idx = 0;
#ifdef ERROR_CODES
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	// Check parameters.
	if (test_parameters == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
#endif
	// Check EP-ID.
	for (idx=0 ; idx<SIGFOX_EP_ID_SIZE_BYTES ; idx++) {
		if (sigfox_ep_api_ctx.ep_id[idx] != SIGFOX_EP_TEST_ID[idx]) {
			EXIT_ERROR(SIGFOX_EP_API_ERROR_EP_ID);
		}
	}
	// Update local test parameters.
	sigfox_ep_api_ctx.test_parameters.tx_frequency_hz = (test_parameters -> tx_frequency_hz);
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	sigfox_ep_api_ctx.test_parameters.fh_timer_enable = (test_parameters -> fh_timer_enable);
#endif
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
	sigfox_ep_api_ctx.test_parameters.lbt_enable = (test_parameters -> lbt_enable);
#endif
	// Send message.
#ifdef ERROR_CODES
	status = _send_control_message(control_message);
#else
	_send_control_message(control_message);
#endif
errors:
	RETURN();
}
#endif

#ifdef BIDIRECTIONAL
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 *dl_rssi_dbm) {
	// Local variables.
	sfx_u8 idx = 0;
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
	// Check library is opened.
	_CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
#ifdef PARAMETERS_CHECK
	if ((dl_payload == SFX_NULL) || (dl_rssi_dbm == SFX_NULL)) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
	if (dl_payload_size > SIGFOX_DL_PAYLOAD_SIZE_BYTES) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_DL_PAYLOAD_SIZE);
	}
#endif
	// Check downlink status.
	if (sigfox_ep_api_ctx.message_status.downlink_frame == 0) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_DL_PAYLOAD_UNAVAILABLE);
	}
	// Copy local bytes into given buffer.
	for (idx=0 ; idx<dl_payload_size ; idx++) {
		dl_payload[idx] = sigfox_ep_api_ctx.dl_payload[idx];
	}
	// Copy local RSSI.
	(*dl_rssi_dbm) = sigfox_ep_api_ctx.dl_rssi_dbm;
error_state:
errors:
	RETURN();
}
#endif

/*******************************************************************/
SIGFOX_EP_API_message_status_t SIGFOX_EP_API_get_message_status(void) {
	// Return current message status.
	return sigfox_ep_api_ctx.message_status;
}

#ifdef ASYNCHRONOUS
/*******************************************************************/
SIGFOX_EP_API_state_t SIGFOX_EP_API_get_state(void) {
	// Return current library state.
	return sigfox_ep_api_ctx.state;
}
#endif

#ifdef VERBOSE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_ep_id(sfx_u8* ep_id, sfx_u8 ep_id_size_bytes) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	if (ep_id == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
	if (ep_id_size_bytes > SIGFOX_EP_ID_SIZE_BYTES) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_EP_ID);
	}
#endif
	// Check library is opened.
	_CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
	// Read ID directly from MCU driver.
#ifdef ERROR_CODES
	mcu_status = MCU_API_get_ep_id(ep_id, ep_id_size_bytes);
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
	MCU_API_get_ep_id(ep_id, ep_id_size_bytes);
#endif
error_state:
#if (defined PARAMETERS_CHECK) || (defined ERROR_CODES)
errors:
#endif
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	if (initial_pac == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
	if (initial_pac_size_bytes > SIGFOX_EP_PAC_SIZE_BYTES) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_EP_PAC);
	}
#endif
	// Check library is opened.
	_CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
	// Read PAC directly from MCU driver.
#ifdef ERROR_CODES
	mcu_status = MCU_API_get_initial_pac(initial_pac, initial_pac_size_bytes);
	MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
	MCU_API_get_initial_pac(initial_pac, initial_pac_size_bytes);
#endif
error_state:
#if (defined PARAMETERS_CHECK) || (defined ERROR_CODES)
errors:
#endif
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_version(SIGFOX_version_t version_type, sfx_u8 **version, sfx_u8 *version_size_char) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
	MCU_API_status_t mcu_status = MCU_API_SUCCESS;
	RF_API_status_t rf_status = RF_API_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	if ((version == SFX_NULL) || (version_size_char == SFX_NULL)) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
#endif
	// Check library is opened.
	_CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
	// Check version type.
	switch (version_type) {
	case SIGFOX_VERSION_EP_LIBRARY:
		(*version) = (sfx_u8*) SIGFOX_EP_API_VERSION;
		(*version_size_char) = (sfx_u8) sizeof(SIGFOX_EP_API_VERSION);
		break;
	case SIGFOX_VERSION_MCU_DRIVER:
#ifdef ERROR_CODES
		mcu_status = MCU_API_get_version(version, version_size_char);
		MCU_API_check_status(SIGFOX_EP_API_ERROR_MCU);
#else
		MCU_API_get_version(version, version_size_char);
#endif
		break;
	case SIGFOX_VERSION_RF_DRIVER:
#ifdef ERROR_CODES
		rf_status = RF_API_get_version(version, version_size_char);
		RF_API_check_status(SIGFOX_EP_API_ERROR_RF);
#else
		RF_API_get_version(version, version_size_char);
#endif
		break;
	default:
		EXIT_ERROR(SIGFOX_EP_API_ERROR_VERSION);
		break;
	}
error_state:
errors:
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_flags(sfx_u8 **ep_flags, sfx_u8 *ep_flags_size_char) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	if ((ep_flags == SFX_NULL) || (ep_flags_size_char == SFX_NULL)) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
#endif
	(*ep_flags) = (sfx_u8*) SIGFOX_EP_API_FLAGS;
	(*ep_flags_size_char) = (sfx_u8) sizeof(SIGFOX_EP_API_FLAGS);
#ifdef PARAMETERS_CHECK
errors:
#endif
	RETURN();
}
#endif

#ifdef ERROR_STACK
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_unstack_error(SIGFOX_ERROR_t *error_ptr) {
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	if (error_ptr == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
	}
#endif
	// Check error stack has been initialized.
	if (sigfox_ep_api_ctx.internal_flags.error_stack_initialized == 0) {
		EXIT_ERROR(SIGFOX_EP_API_ERROR_STATE);
	}
	// Directly call error stack driver.
	SIGFOX_ERROR_unstack(error_ptr);
errors:
	RETURN();
}
#endif
