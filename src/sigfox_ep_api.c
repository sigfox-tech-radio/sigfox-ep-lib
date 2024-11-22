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

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "manuf/mcu_api.h"
#include "manuf/rf_api.h"
#include "core/sigfox_ep_bitstream.h"
#include "core/sigfox_ep_frequency.h"
#ifdef SIGFOX_EP_CERTIFICATION
#include "sigfox_ep_api_test.h"
#endif
#ifdef SIGFOX_EP_REGULATORY
#include "core/sigfox_tx_control.h"
#endif
#ifdef SIGFOX_EP_VERBOSE
#include "sigfox_ep_lib_version.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

/*** SIGFOX EP API local macros ***/

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#define SIGFOX_EP_API_UL_PAYLOAD_SIZE   SIGFOX_EP_UL_PAYLOAD_SIZE
#else
#define SIGFOX_EP_API_UL_PAYLOAD_SIZE   SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES
#endif
#endif

/*** SIGFOX EP API local structures ***/

#ifndef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
typedef enum {
    SIGFOX_EP_API_STATE_CLOSED,
    SIGFOX_EP_API_STATE_READY,
#ifdef SIGFOX_EP_REGULATORY
    SIGFOX_EP_API_STATE_REGULATORY,
#endif
    SIGFOX_EP_API_STATE_UL_MODULATION_PENDING,
#if !(defined SIGFOX_EP_SINGLE_FRAME) && (!(defined SIGFOX_EP_T_IFU_MS) || (SIGFOX_EP_T_IFU_MS > 0) || (defined SIGFOX_EP_BIDIRECTIONAL))
    SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER,
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    SIGFOX_EP_API_STATE_DL_TIMER,
    SIGFOX_EP_API_STATE_DL_LISTENING,
    SIGFOX_EP_API_STATE_DL_CONFIRMATION_TIMER,
#endif
    SIGFOX_EP_API_STATE_LAST
} SIGFOX_EP_API_state_t;
#endif

/*******************************************************************/
typedef union {
    struct {
        sfx_u8 error_stack_initialized :1;
        sfx_u8 synchronous :1;
        sfx_u8 send_message_request :1;
        sfx_u8 radio_woken_up :1;
        sfx_u8 dl_conf_message :1;
        sfx_u8 control_message :1;
        sfx_u8 tx_control_pre_check_running :1;
        sfx_u8 tx_control_post_check_running :1;
        sfx_u8 frame_success :1;
        sfx_u8 nvm_write_pending :1;
        sfx_u8 process_running :1;
    } field;
    sfx_u16 all;
} SIGFOX_EP_API_internal_flags_t;

/*******************************************************************/
typedef union {
    struct {
        sfx_u8 mcu_api_process :1;
        sfx_u8 mcu_timer1_cplt :1;
        sfx_u8 mcu_timer2_cplt :1;
        sfx_u8 rf_api_process :1;
        sfx_u8 rf_tx_cplt :1;
        sfx_u8 rf_rx_data_received :1;
        sfx_u8 tx_control_process :1;
        sfx_u8 tx_control_pre_check_cplt :1;
        sfx_u8 tx_control_post_check_cplt :1;
        sfx_u8 low_level_error :1;
    } field;
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
#ifndef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
    sfx_u16 message_counter_rollover;
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
    SIGFOX_EP_API_process_cb_t process_cb;
    SIGFOX_EP_API_uplink_cplt_cb_t uplink_cplt_cb;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    SIGFOX_EP_API_downlink_cplt_cb_t downlink_cplt_cb;
#endif
    SIGFOX_EP_API_message_cplt_cb_t message_cplt_cb;
#ifdef SIGFOX_EP_ERROR_CODES
    volatile MCU_API_status_t mcu_api_status_from_callback;
    volatile RF_API_status_t rf_api_status_from_callback;
#endif
#endif
    // Message data.
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE)
    SIGFOX_EP_API_common_t *common_parameters_ptr;
#endif
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    SIGFOX_EP_API_application_message_t *application_message_ptr;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    SIGFOX_EP_API_application_message_t local_application_message;
#if !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    sfx_u8 local_ul_payload[SIGFOX_EP_API_UL_PAYLOAD_SIZE];
#endif
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Downlink variables.
    RF_API_rx_data_t rx_data;
    sfx_bool dl_status;
    sfx_u8 dl_payload[SIGFOX_DL_PAYLOAD_SIZE_BYTES];
    sfx_s16 dl_rssi_dbm;
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    SIGFOX_EP_API_control_message_t *control_message_ptr;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    SIGFOX_EP_API_control_message_t local_control_message;
#endif
#endif
#if (defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL)
    sfx_s16 temperature_tenth_degrees;
    sfx_u16 voltage_tx_mv;
    sfx_u16 voltage_idle_mv;
#endif
    sfx_u8 bitstream[SIGFOX_UL_BITSTREAM_SIZE_BYTES];
    sfx_u8 bitstream_size;
#ifndef SIGFOX_EP_SINGLE_FRAME
    SIGFOX_ul_frame_rank_t ul_frame_rank;
    sfx_u8 frame_success_count;
#endif
#if !(defined SIGFOX_EP_SINGLE_FRAME) || (defined SIGFOX_EP_BIDIRECTIONAL)
    sfx_u32 interframe_ms; // Tifu, Tifb or Tconf.
#endif
#ifdef SIGFOX_EP_CERTIFICATION
    SIGFOX_EP_API_TEST_parameters_t test_parameters;
#endif
} SIGFOX_EP_API_context_t;

/*** SIGFOX EP API local global variables ***/

static SIGFOX_EP_API_context_t sigfox_ep_api_ctx = { .rc_ptr = SIGFOX_NULL, .state = SIGFOX_EP_API_STATE_CLOSED, .internal_flags.all = 0, .irq_flags.all = 0, .message_status.all = 0,
#ifndef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
    .message_counter_rollover = 0,
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
#ifdef SIGFOX_EP_ERROR_CODES
    .mcu_api_status_from_callback = MCU_API_SUCCESS, .rf_api_status_from_callback = RF_API_SUCCESS,
#endif /* SIGFOX_EP_ERROR_CODES */
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP)
    .common_parameters_ptr = SIGFOX_NULL,
#endif
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    .application_message_ptr = SIGFOX_NULL,
#ifdef SIGFOX_EP_BIDIRECTIONAL
    .dl_status = SIGFOX_FALSE, .dl_rssi_dbm = 0,
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    .control_message_ptr = SIGFOX_NULL,
#endif
#if (defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL)
    .temperature_tenth_degrees = 0, .voltage_tx_mv = 0, .voltage_idle_mv = 0,
#endif
#ifndef SIGFOX_EP_SINGLE_FRAME
    .ul_frame_rank = SIGFOX_UL_FRAME_RANK_1, .frame_success_count = 0,
#endif
    };
#ifdef SIGFOX_EP_VERBOSE
static const sfx_u8 SIGFOX_EP_API_VERSION[] = SIGFOX_EP_LIB_VERSION;
static const sfx_u8 SIGFOX_EP_API_FLAGS[] = SIGFOX_EP_FLAGS;
#endif

/*** SIGFOX EP API local functions ***/

/*******************************************************************/
#define _CHECK_LIBRARY_STATE(state_condition) { if (sigfox_ep_api_ctx.state state_condition) { SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_STATE); } }

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
#define _PROCESS_CALLBACK(void) { \
    if ((sigfox_ep_api_ctx.process_cb != SIGFOX_NULL) && (sigfox_ep_api_ctx.internal_flags.field.process_running == 0)) { \
        sigfox_ep_api_ctx.process_cb(); \
    } \
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
#define _UPLINK_CPLT_CALLBACK(void) { \
    if ((sigfox_ep_api_ctx.uplink_cplt_cb != SIGFOX_NULL) && (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY)) { \
        sigfox_ep_api_ctx.uplink_cplt_cb(); \
    } \
}
#endif

#if (defined SIGFOX_EP_ASYNCHRONOUS) && (defined SIGFOX_EP_BIDIRECTIONAL)
/*******************************************************************/
#define _DOWNLINK_CPLT_CALLBACK(void) { \
    if ((sigfox_ep_api_ctx.downlink_cplt_cb != SIGFOX_NULL) && (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY)) { \
        sigfox_ep_api_ctx.downlink_cplt_cb(); \
    } \
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
#define _MESSAGE_CPLT_CALLBACK(void) { \
    if ((sigfox_ep_api_ctx.message_cplt_cb != SIGFOX_NULL) && (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY)) { \
        sigfox_ep_api_ctx.message_cplt_cb(); \
    } \
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
static void _MCU_API_process_cb(void) {
    // Set local flag.
    sigfox_ep_api_ctx.irq_flags.field.mcu_api_process = 1;
    _PROCESS_CALLBACK();
}

#ifdef SIGFOX_EP_ASYNCHRONOUS
#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
static void _MCU_API_error_cb(MCU_API_status_t mcu_api_status) {
    // Do not call the process if the given status is not an error.
    if (mcu_api_status != MCU_API_SUCCESS) {
        // Store error and set flag.
        sigfox_ep_api_ctx.mcu_api_status_from_callback = mcu_api_status;
        sigfox_ep_api_ctx.irq_flags.field.low_level_error = 1;
        // Ask for process.
        _PROCESS_CALLBACK();
    }
}
#else
/*******************************************************************/
static void _MCU_API_error_cb(void) {
    sigfox_ep_api_ctx.irq_flags.field.low_level_error = 1;
    _PROCESS_CALLBACK();
}
#endif
#endif

#if (defined SIGFOX_EP_ASYNCHRONOUS) && ((!(defined SIGFOX_EP_SINGLE_FRAME) && (!(defined SIGFOX_EP_T_IFU_MS) || (SIGFOX_EP_T_IFU_MS > 0))) || (defined SIGFOX_EP_BIDIRECTIONAL))
/*******************************************************************/
static void _MCU_API_timer1_cplt_cb(void) {
    // Set local flag.
    sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt = 1;
    _PROCESS_CALLBACK();
}
#endif

#if (defined SIGFOX_EP_ASYNCHRONOUS) && (defined SIGFOX_EP_BIDIRECTIONAL)
/*******************************************************************/
static void _MCU_API_timer2_cplt_cb(void) {
    // Set local flag.
    sigfox_ep_api_ctx.irq_flags.field.mcu_timer2_cplt = 1;
    _PROCESS_CALLBACK();
}
#endif

/*******************************************************************/
static void _RF_API_process_cb(void) {
    // Set local flag.
    sigfox_ep_api_ctx.irq_flags.field.rf_api_process = 1;
    _PROCESS_CALLBACK();
}

#ifdef SIGFOX_EP_ASYNCHRONOUS
#ifdef SIGFOX_EP_ERROR_CODES
/*******************************************************************/
static void _RF_API_error_cb(RF_API_status_t rf_api_status) {
    // Do not call the process if the given status is not an error.
    if (rf_api_status != RF_API_SUCCESS) {
        // Store error and set flag.
        sigfox_ep_api_ctx.rf_api_status_from_callback = rf_api_status;
        sigfox_ep_api_ctx.irq_flags.field.low_level_error = 1;
        // Ask for process.
        _PROCESS_CALLBACK();
    }
}
#else
/*******************************************************************/
static void _RF_API_error_cb(void) {
    sigfox_ep_api_ctx.irq_flags.field.low_level_error = 1;
    _PROCESS_CALLBACK();
}
#endif
#endif

/*******************************************************************/
static void _RF_API_tx_cplt_cb(void) {
    // Set local flag.
    sigfox_ep_api_ctx.irq_flags.field.rf_tx_cplt = 1;
    _PROCESS_CALLBACK();
}
#endif

#if (defined SIGFOX_EP_ASYNCHRONOUS) && (defined SIGFOX_EP_BIDIRECTIONAL)
/*******************************************************************/
static void _RF_API_rx_data_received_cb(void) {
    // Set local flag.
    sigfox_ep_api_ctx.irq_flags.field.rf_rx_data_received = 1;
    _PROCESS_CALLBACK();
}
#endif

#if (defined SIGFOX_EP_ASYNCHRONOUS) && (defined SIGFOX_EP_REGULATORY)
/*******************************************************************/
static void _SIGFOX_TX_CONTROL_process_cb(void) {
    // Set local flag and result.
    sigfox_ep_api_ctx.irq_flags.field.tx_control_process = 1;
    _PROCESS_CALLBACK();
}
#endif

#if (defined SIGFOX_EP_ASYNCHRONOUS) && (defined SIGFOX_EP_REGULATORY)
/*******************************************************************/
static void _SIGFOX_TX_CONTROL_pre_check_cplt_cb(void) {
    // Set local flag and result.
    sigfox_ep_api_ctx.irq_flags.field.tx_control_pre_check_cplt = 1;
    _PROCESS_CALLBACK();
}
#endif

#if (defined SIGFOX_EP_ASYNCHRONOUS) && (defined SIGFOX_EP_REGULATORY)
/*******************************************************************/
static void _SIGFOX_TX_CONTROL_post_check_cplt_cb(void) {
    // Set local flag and result.
    sigfox_ep_api_ctx.irq_flags.field.tx_control_post_check_cplt = 1;
    _PROCESS_CALLBACK();
}
#endif

#if (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES) && (!(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE))
/*******************************************************************/
static SIGFOX_EP_API_status_t _check_common_parameters(SIGFOX_EP_API_common_t *common_params) {
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    // Check parameter.
    if (common_params == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
    // Check bit rate.
#ifdef SIGFOX_EP_UL_BIT_RATE_BPS
    sfx_u8 idx = 0;
    sfx_bool bit_rate_valid = SIGFOX_FALSE;
    // Check if the given value is allowed.
    for (idx = 0; idx < SIGFOX_UL_BIT_RATE_LAST; idx++) {
        if (SIGFOX_UL_BIT_RATE_BPS_LIST[idx] == SIGFOX_EP_UL_BIT_RATE_BPS) {
            bit_rate_valid = SIGFOX_TRUE;
            break;
        }
    }
    if (bit_rate_valid == SIGFOX_FALSE) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_UL_BIT_RATE);
    }
    // Check RC capability.
    if ((((sigfox_ep_api_ctx.rc_ptr->uplink_bit_rate_capability) >> idx) & 0x01) == 0) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_UL_BIT_RATE);
    }
#else /* UL_BIT_RATE */
    // Check if the given bit rate exists.
    if ((common_params->ul_bit_rate) >= SIGFOX_UL_BIT_RATE_LAST) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_UL_BIT_RATE);
    }
    // Check RC capability.
    if ((((sigfox_ep_api_ctx.rc_ptr->uplink_bit_rate_capability) >> (common_params->ul_bit_rate)) & 0x01) == 0) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_UL_BIT_RATE);
    }
#endif /* UL_BIT_RATE */
    // Check TX power regarding RC rule.
#ifdef SIGFOX_EP_TX_POWER_DBM_EIRP
    if (SIGFOX_EP_TX_POWER_DBM_EIRP > ((sigfox_ep_api_ctx.rc_ptr)->tx_power_dbm_eirp_max)) {
#else
    if (((common_params->tx_power_dbm_eirp)) > ((sigfox_ep_api_ctx.rc_ptr)->tx_power_dbm_eirp_max)) {
#endif
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_TX_POWER);
    }
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Check number of frames.
    if (((common_params->number_of_frames) == 0) || ((common_params->number_of_frames) > SIGFOX_UL_FRAME_RANK_LAST)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NUMBER_OF_FRAMES);
    }
#ifndef SIGFOX_EP_T_IFU_MS
    // Check interframe delay.
    if ((common_params->t_ifu_ms) > SIGFOX_T_IFU_MAX_MS) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_T_IFU);
    }
#endif
#endif /* MULTIPLE_FRAMES */
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    // Check key type.
    if ((common_params->ep_key_type) >= SIGFOX_EP_KEY_LAST) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_EP_KEY);
    }
#endif
errors:
    return status;
}
#endif

#if (defined SIGFOX_EP_APPLICATION_MESSAGES) && (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
/*******************************************************************/
static SIGFOX_EP_API_status_t _check_application_message(SIGFOX_EP_API_application_message_t *app_msg) {
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    // Check parameter.
    if (app_msg == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
#if !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    // Check payload.
    if ((app_msg->type) == SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY) {
        // Payload is required.
        if ((app_msg->ul_payload) == SIGFOX_NULL) {
            SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
        }
    }
#endif
    // Check message type.
    if ((app_msg->type) >= SIGFOX_APPLICATION_MESSAGE_TYPE_LAST) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_MESSAGE_TYPE);
    }
#ifndef SIGFOX_EP_UL_PAYLOAD_SIZE
    if ((app_msg->type) == SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY) {
        // Length is required.
        if (((app_msg->ul_payload_size_bytes) == 0) || ((app_msg->ul_payload_size_bytes) > SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES)) {
            SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_UL_PAYLOAD_SIZE);
        }
    }
#endif
#if (defined SIGFOX_EP_BIDIRECTIONAL) && !(defined SIGFOX_EP_T_CONF_MS)
    // Check downlink parameters.
    if ((app_msg->bidirectional_flag) == SIGFOX_TRUE) {
        // Check DL confirmation delay.
        if (((app_msg->t_conf_ms) < SIGFOX_T_CONF_MIN_MS) || ((app_msg->t_conf_ms) > SIGFOX_T_CONF_MAX_MS)) {
            SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_T_CONF);
        }
    }
#endif
errors:
    return status;
}
#endif

#if (defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) && (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
/*******************************************************************/
static SIGFOX_EP_API_status_t _check_control_message(SIGFOX_EP_API_control_message_t *ctrl_msg) {
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    // Check parameter.
    if (ctrl_msg == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
    // Check message type.
    if ((ctrl_msg->type) != SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_MESSAGE_TYPE);
    }
errors:
    return status;
}
#endif

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
/*******************************************************************/
static SIGFOX_EP_API_status_t _store_application_message(SIGFOX_EP_API_application_message_t *application_message) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#if (defined SIGFOX_EP_ASYNCHRONOUS) && (((defined SIGFOX_EP_UL_PAYLOAD_SIZE) && (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE))
    sfx_u8 idx = 0;
#endif
#if (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
    // Check parameters.
    status = _check_application_message(application_message);
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE)
    status = _check_common_parameters(&(application_message->common_parameters));
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#endif
#endif /* SIGFOX_EP_PARAMETERS_CHECK and SIGFOX_EP_ERROR_CODES*/
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // In asynchronous mode, all the data has to be stored locally since the client pointer could be removed.
    // Common parameters.
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
    sigfox_ep_api_ctx.local_application_message.common_parameters.ul_bit_rate = ((application_message->common_parameters).ul_bit_rate);
#endif
#ifndef SIGFOX_EP_TX_POWER_DBM_EIRP
    sigfox_ep_api_ctx.local_application_message.common_parameters.tx_power_dbm_eirp = ((application_message->common_parameters).tx_power_dbm_eirp);
#endif
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Number of frames and inter-frame delay.
    sigfox_ep_api_ctx.local_application_message.common_parameters.number_of_frames = ((application_message->common_parameters).number_of_frames);
#ifndef SIGFOX_EP_T_IFU_MS
    sigfox_ep_api_ctx.local_application_message.common_parameters.t_ifu_ms = ((application_message->common_parameters).t_ifu_ms);
#endif
#endif /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    sigfox_ep_api_ctx.local_application_message.common_parameters.ep_key_type = (application_message->common_parameters).ep_key_type;
#endif
    // Message type.
    sigfox_ep_api_ctx.local_application_message.type = (application_message->type);
    // Store callbacks (even if NULL).
    sigfox_ep_api_ctx.uplink_cplt_cb = (application_message->uplink_cplt_cb);
    sigfox_ep_api_ctx.message_cplt_cb = (application_message->message_cplt_cb);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sigfox_ep_api_ctx.downlink_cplt_cb = (application_message->downlink_cplt_cb);
#endif
    // UL payload.
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#if (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    for (idx = 0; idx < SIGFOX_EP_UL_PAYLOAD_SIZE; idx++) {
        sigfox_ep_api_ctx.local_ul_payload[idx] = (application_message->ul_payload)[idx];
    }
#endif
#else /* SIGFOX_EP_UL_PAYLOAD_SIZE */
    if ((application_message->type) == SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY) {
        for (idx = 0; idx < (application_message->ul_payload_size_bytes); idx++) {
            sigfox_ep_api_ctx.local_ul_payload[idx] = (application_message->ul_payload)[idx];
        }
        sigfox_ep_api_ctx.local_application_message.ul_payload_size_bytes = (application_message->ul_payload_size_bytes);
    }
    else {
        sigfox_ep_api_ctx.local_application_message.ul_payload_size_bytes = 0;
    }
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Downlink parameters.
    sigfox_ep_api_ctx.local_application_message.bidirectional_flag = (application_message->bidirectional_flag);
#ifndef SIGFOX_EP_T_CONF_MS
    sigfox_ep_api_ctx.local_application_message.t_conf_ms = (application_message->t_conf_ms);
#endif
#endif
    // Update pointers to local data.
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE)
    sigfox_ep_api_ctx.common_parameters_ptr = &(sigfox_ep_api_ctx.local_application_message.common_parameters);
#endif
    sigfox_ep_api_ctx.application_message_ptr = &(sigfox_ep_api_ctx.local_application_message);
#if !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    (sigfox_ep_api_ctx.application_message_ptr)->ul_payload = (sfx_u8*) sigfox_ep_api_ctx.local_ul_payload;
#endif
#else /* SIGFOX_EP_ASYNCHRONOUS */
    // In blocking mode, the message pointer will directly address the client data since it will be kept during processing.
    sigfox_ep_api_ctx.application_message_ptr = application_message;
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    sigfox_ep_api_ctx.control_message_ptr = SIGFOX_NULL;
#endif
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE)
    sigfox_ep_api_ctx.common_parameters_ptr = &(application_message->common_parameters);
#endif
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#if (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
errors:
    if (status != SIGFOX_EP_API_SUCCESS) {
        sigfox_ep_api_ctx.message_status.field.execution_error = 1;
    }
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
/*******************************************************************/
static SIGFOX_EP_API_status_t _store_control_message(SIGFOX_EP_API_control_message_t *control_message) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#if (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
    // Check parameters.
    status = _check_control_message(control_message);
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE)
    status = _check_common_parameters(&(control_message->common_parameters));
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#endif
#endif /* SIGFOX_EP_PARAMETERS_CHECK and SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // In asynchronous mode, all the data has to be stored locally since the client pointer could be removed, and the message pointer will address the local data.
    // Common parameters.
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
    sigfox_ep_api_ctx.local_control_message.common_parameters.ul_bit_rate = ((control_message->common_parameters).ul_bit_rate);
#endif
#ifndef SIGFOX_EP_TX_POWER_DBM_EIRP
    sigfox_ep_api_ctx.local_control_message.common_parameters.tx_power_dbm_eirp = ((control_message->common_parameters).tx_power_dbm_eirp);
#endif
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Number of frames and inter frame delay.
    sigfox_ep_api_ctx.local_control_message.common_parameters.number_of_frames = ((control_message->common_parameters).number_of_frames);
#ifndef SIGFOX_EP_T_IFU_MS
    sigfox_ep_api_ctx.local_control_message.common_parameters.t_ifu_ms = ((control_message->common_parameters).t_ifu_ms);
#endif
#endif /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    sigfox_ep_api_ctx.local_control_message.common_parameters.ep_key_type = (control_message->common_parameters).ep_key_type;
#endif
    // Message type.
    sigfox_ep_api_ctx.local_control_message.type = (control_message->type);
    // Store callbacks (even if NULL).
    sigfox_ep_api_ctx.uplink_cplt_cb = (control_message->uplink_cplt_cb);
    sigfox_ep_api_ctx.message_cplt_cb = (control_message->message_cplt_cb);
    // Update pointer to local data.
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE)
    sigfox_ep_api_ctx.common_parameters_ptr = &(sigfox_ep_api_ctx.local_control_message.common_parameters);
#endif
    sigfox_ep_api_ctx.control_message_ptr = &(sigfox_ep_api_ctx.local_control_message);
#else /* SIGFOX_EP_ASYNCHRONOUS */
    // In blocking mode, the message pointer will directly address the client data since it will be kept during processing.
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    sigfox_ep_api_ctx.application_message_ptr = SIGFOX_NULL;
#endif
    sigfox_ep_api_ctx.control_message_ptr = control_message;
#if !(defined SIGFOX_EP_SINGLE_FRAME) || !(defined SIGFOX_EP_UL_BIT_RATE_BPS) || !(defined SIGFOX_EP_TX_POWER_DBM_EIRP) || (defined SIGFOX_EP_PUBLIC_KEY_CAPABLE)
    sigfox_ep_api_ctx.common_parameters_ptr = &(control_message->common_parameters);
#endif
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#if (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
errors:
    if (status != SIGFOX_EP_API_SUCCESS) {
        sigfox_ep_api_ctx.message_status.field.execution_error = 1;
    }
#endif
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
static SIGFOX_EP_API_status_t _compute_next_ul_frequency(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_FREQUENCY_status_t sigfox_ep_frequency_status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#ifndef SIGFOX_EP_SINGLE_FRAME
    SIGFOX_EP_FREQUENCY_uplink_signal_t frequency_parameters;
#endif
    // Prepare frequency parameters.
#ifndef SIGFOX_EP_SINGLE_FRAME
    frequency_parameters.ul_frame_rank = sigfox_ep_api_ctx.ul_frame_rank;
    frequency_parameters.number_of_frames = (sigfox_ep_api_ctx.common_parameters_ptr)->number_of_frames;
#ifdef SIGFOX_EP_BIDIRECTIONAL
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    if (sigfox_ep_api_ctx.application_message_ptr == SIGFOX_NULL) {
        frequency_parameters.bidirectional_flag = SIGFOX_FALSE;
    }
    else {
        frequency_parameters.bidirectional_flag = (sigfox_ep_api_ctx.internal_flags.field.control_message != 0) ? SIGFOX_FALSE : ((sigfox_ep_api_ctx.application_message_ptr)->bidirectional_flag);
    }
#else
    frequency_parameters.bidirectional_flag = SIGFOX_FALSE;
#endif
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#endif /* SIGFOX_EP_SINGLE_FRAME */
    // Compute frequency.
#ifdef SIGFOX_EP_CERTIFICATION
    // Bypass frequency if needed.
    if ((sigfox_ep_api_ctx.test_parameters.tx_frequency_hz) != 0) {
        sigfox_ep_api_ctx.ul_frequency_hz = sigfox_ep_api_ctx.test_parameters.tx_frequency_hz;
    }
    else {
#endif /* SIGFOX_EP_CERTIFICATION */
#ifndef SIGFOX_EP_SINGLE_FRAME
#ifdef SIGFOX_EP_ERROR_CODES
        // Compute frequency.
        sigfox_ep_frequency_status = SIGFOX_EP_FREQUENCY_compute_uplink(&frequency_parameters, &sigfox_ep_api_ctx.ul_frequency_hz);
        SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_EP_FREQUENCY);
#else
    SIGFOX_EP_FREQUENCY_compute_uplink(&frequency_parameters, &sigfox_ep_api_ctx.ul_frequency_hz);
#endif
#else /* MULTIPLE_FRAMES */
#ifdef SIGFOX_EP_ERROR_CODES
    // Compute frequency.
    sigfox_ep_frequency_status = SIGFOX_EP_FREQUENCY_compute_uplink(&sigfox_ep_api_ctx.ul_frequency_hz);
    SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_EP_FREQUENCY);
#else
    SIGFOX_EP_FREQUENCY_compute_uplink(&sigfox_ep_api_ctx.ul_frequency_hz);
#endif
#endif /* MULTIPLE_FRAMES */
#ifdef SIGFOX_EP_CERTIFICATION
    }
#endif
    // Update random value.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_frequency_status = SIGFOX_EP_FREQUENCY_get_random_value(&sigfox_ep_api_ctx.random_value);
    SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_EP_FREQUENCY);
#else
    SIGFOX_EP_FREQUENCY_get_random_value(&sigfox_ep_api_ctx.random_value);
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static sfx_bool _is_downlink_required(void) {
    // Local variables.
    sfx_bool downlink_required = SIGFOX_FALSE;
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    // Check message pointer.
    if (sigfox_ep_api_ctx.application_message_ptr == SIGFOX_NULL) goto errors;
    // Check bidirectional flag.
    if (((sigfox_ep_api_ctx.application_message_ptr->bidirectional_flag) == SIGFOX_TRUE) && (sigfox_ep_api_ctx.internal_flags.field.control_message == 0)) {
        downlink_required = SIGFOX_TRUE;
    }
#ifdef SIGFOX_EP_CERTIFICATION
    // Check downlink bypass flag.
    if (sigfox_ep_api_ctx.test_parameters.flags.field.dl_enable == 0) {
        downlink_required = SIGFOX_FALSE;
    }
#endif
errors:
#endif
    return downlink_required;
}
#endif

/*******************************************************************/
static sfx_bool _is_last_frame_of_uplink_sequence(void) {
    // Local variables.
    sfx_bool last_uplink_frame = SIGFOX_FALSE;
#ifdef SIGFOX_EP_SINGLE_FRAME
    last_uplink_frame = SIGFOX_TRUE;
#else
    if (sigfox_ep_api_ctx.ul_frame_rank >= ((sfx_u8) (((sigfox_ep_api_ctx.common_parameters_ptr)->number_of_frames) - 1))) {
        last_uplink_frame = SIGFOX_TRUE;
    }
#endif
    return last_uplink_frame;
}

#if (defined SIGFOX_EP_REGULATORY) || (defined SIGFOX_EP_ASYNCHRONOUS)
/*******************************************************************/
static sfx_bool _is_last_frame_of_message_sequence(void) {
    // Local variables.
    sfx_bool last_message_frame = SIGFOX_FALSE;
    // Check flags.
#ifdef SIGFOX_EP_BIDIRECTIONAL
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    if ((sigfox_ep_api_ctx.application_message_ptr) == SIGFOX_NULL) {
        // Do not check bidirectional flag.
        if ((sigfox_ep_api_ctx.internal_flags.field.dl_conf_message != 0) || (_is_last_frame_of_uplink_sequence() == SIGFOX_TRUE)) {
            last_message_frame = SIGFOX_TRUE;
        }
    }
    else {
        // Check bidirectional flag.
        if ((sigfox_ep_api_ctx.internal_flags.field.dl_conf_message != 0) || ((_is_last_frame_of_uplink_sequence() == SIGFOX_TRUE) && (_is_downlink_required() == SIGFOX_FALSE))) {
            last_message_frame = SIGFOX_TRUE;
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
static void _set_common_bitstream_parameters(SIGFOX_EP_BITSTREAM_common_t *bitsteam_common_parameter_ptr) {
    // EP ID.
    bitsteam_common_parameter_ptr->ep_id = (sfx_u8*) sigfox_ep_api_ctx.ep_id;
    // Message counter.
    bitsteam_common_parameter_ptr->message_counter = sigfox_ep_api_ctx.message_counter;
#ifndef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
    // Message counter rollover.
    bitsteam_common_parameter_ptr->message_counter_rollover = sigfox_ep_api_ctx.message_counter_rollover;
#endif
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Frame rank.
    bitsteam_common_parameter_ptr->ul_frame_rank = sigfox_ep_api_ctx.ul_frame_rank;
#endif
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    // EP key.
    bitsteam_common_parameter_ptr->ep_key_type = (sigfox_ep_api_ctx.common_parameters_ptr)->ep_key_type;
#endif
}

#ifdef SIGFOX_EP_REGULATORY
/*******************************************************************/
static void _set_tx_control_parameters(SIGFOX_TX_CONTROL_check_type check_type, SIGFOX_TX_CONTROL_parameters_t *tx_control_params) {
    // Update TX control parameters.
    tx_control_params->type = check_type;
    tx_control_params->bitstream_size_bytes = sigfox_ep_api_ctx.bitstream_size;
    tx_control_params->last_message_frame = _is_last_frame_of_message_sequence();
#ifndef SIGFOX_EP_UL_BIT_RATE_BPS
    tx_control_params->ul_bit_rate_bps = SIGFOX_UL_BIT_RATE_BPS_LIST[(sigfox_ep_api_ctx.common_parameters_ptr)->ul_bit_rate];
#endif
#ifndef SIGFOX_EP_SINGLE_FRAME
    tx_control_params->ul_frame_rank = sigfox_ep_api_ctx.ul_frame_rank;
    tx_control_params->number_of_frames = (sigfox_ep_api_ctx.common_parameters_ptr)->number_of_frames;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    tx_control_params->dl_conf_message = sigfox_ep_api_ctx.internal_flags.field.dl_conf_message;
#endif
#if !(defined SIGFOX_EP_SINGLE_FRAME) || (defined SIGFOX_EP_BIDIRECTIONAL)
    tx_control_params->interframe_ms = sigfox_ep_api_ctx.interframe_ms; // Tifu, Tifb or Tconf.
#endif
#ifdef SIGFOX_EP_CERTIFICATION
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_FH
    tx_control_params->fh_check_enable = sigfox_ep_api_ctx.test_parameters.flags.field.tx_control_fh_enable;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT
    tx_control_params->lbt_check_enable = sigfox_ep_api_ctx.test_parameters.flags.field.tx_control_lbt_enable;
    tx_control_params->lbt_cs_max_duration_first_frame_ms = sigfox_ep_api_ctx.test_parameters.lbt_cs_max_duration_first_frame_ms;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LDC
    tx_control_params->ldc_check_enable = sigfox_ep_api_ctx.test_parameters.flags.field.tx_control_ldc_enable;
#endif
#endif /* SIGFOX_EP_CERTIFICATION */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    tx_control_params->cplt_cb = (SIGFOX_TX_CONTROL_check_cplt_cb_t) ((check_type == SIGFOX_TX_CONTROL_TYPE_PRE_CHECK) ? &_SIGFOX_TX_CONTROL_pre_check_cplt_cb : &_SIGFOX_TX_CONTROL_post_check_cplt_cb);
#endif
}
#endif

#if (defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL)
/*******************************************************************/
static SIGFOX_EP_API_status_t _read_mcu_voltage_temperature(void) {
    // Local variables.
    sfx_s16 temperature_tenth_degrees = 0;
    sfx_u16 voltage_tx_mv = 0;
    sfx_u16 voltage_idle_mv = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_SINGLE_FRAME
    // Read MCU data.
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_get_voltage_temperature(&voltage_idle_mv, &voltage_tx_mv, &temperature_tenth_degrees);
    MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
    MCU_API_get_voltage_temperature(&voltage_idle_mv, &voltage_tx_mv, &temperature_tenth_degrees);
#endif
    // Update global variables.
    sigfox_ep_api_ctx.voltage_idle_mv = voltage_idle_mv;
    sigfox_ep_api_ctx.voltage_tx_mv = voltage_tx_mv;
    sigfox_ep_api_ctx.temperature_tenth_degrees = temperature_tenth_degrees;
#else /* SIGFOX_EP_SINGLE_FRAME */
    if (sigfox_ep_api_ctx.ul_frame_rank == SIGFOX_UL_FRAME_RANK_1) {
        // Read MCU data.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_get_voltage_temperature(&voltage_idle_mv, &voltage_tx_mv, &temperature_tenth_degrees);
        MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
        MCU_API_get_voltage_temperature(&voltage_idle_mv, &voltage_tx_mv, &temperature_tenth_degrees);
#endif
        // Update global variables.
        sigfox_ep_api_ctx.voltage_idle_mv = voltage_idle_mv;
        sigfox_ep_api_ctx.voltage_tx_mv = voltage_tx_mv;
        sigfox_ep_api_ctx.temperature_tenth_degrees = temperature_tenth_degrees;
    }
#endif /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
/*******************************************************************/
static SIGFOX_EP_API_status_t _build_application_frame(void) {
    // Local variables.
    SIGFOX_EP_BITSTREAM_application_frame_t bitstream_parameters;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_BITSTREAM_status_t sigfox_ep_bitstream_status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
    // Prepare common bitstream parameters.
    _set_common_bitstream_parameters(&(bitstream_parameters.common_parameters));
    // Prepare specific bitstream parameters for application frame.
    bitstream_parameters.message_type = ((sigfox_ep_api_ctx.application_message_ptr)->type);
#if !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    bitstream_parameters.ul_payload = ((sigfox_ep_api_ctx.application_message_ptr)->ul_payload);
#endif
#ifndef SIGFOX_EP_UL_PAYLOAD_SIZE
    bitstream_parameters.ul_payload_size_bytes = ((sigfox_ep_api_ctx.application_message_ptr)->ul_payload_size_bytes);
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    bitstream_parameters.bidirectional_flag = ((sigfox_ep_api_ctx.application_message_ptr)->bidirectional_flag);
#endif
    // Build frame.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_bitstream_status = SIGFOX_EP_BITSTREAM_build_application_frame(&bitstream_parameters, (sfx_u8*) sigfox_ep_api_ctx.bitstream, &(sigfox_ep_api_ctx.bitstream_size));
    SIGFOX_EP_BITSTREAM_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_EP_BITSTREAM);
#else
    SIGFOX_EP_BITSTREAM_build_application_frame(&bitstream_parameters, (sfx_u8*) sigfox_ep_api_ctx.bitstream, &(sigfox_ep_api_ctx.bitstream_size));
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
/*******************************************************************/
static SIGFOX_EP_API_status_t _build_control_keep_alive_frame(void) {
    // Local variables.
    SIGFOX_EP_BITSTREAM_control_frame_t bitstream_parameters;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_BITSTREAM_status_t sigfox_ep_bitstream_status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
    // Read MCU data.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _read_mcu_voltage_temperature();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _read_mcu_voltage_temperature();
#endif
    // Prepare common bitstream parameters.
    _set_common_bitstream_parameters(&(bitstream_parameters.common_parameters));
    // Prepare specific bitstream parameters for control keep-alive.
    bitstream_parameters.message_type = SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE;
    bitstream_parameters.voltage_idle_mv = sigfox_ep_api_ctx.voltage_idle_mv;
    bitstream_parameters.voltage_tx_mv = sigfox_ep_api_ctx.voltage_tx_mv;
    bitstream_parameters.temperature_tenth_degrees = sigfox_ep_api_ctx.temperature_tenth_degrees;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    bitstream_parameters.rssi_dbm = 0; // Unused.
#endif
    // Build frame.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_bitstream_status = SIGFOX_EP_BITSTREAM_build_control_frame(&bitstream_parameters, (sfx_u8*) sigfox_ep_api_ctx.bitstream, &(sigfox_ep_api_ctx.bitstream_size));
    SIGFOX_EP_BITSTREAM_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_EP_BITSTREAM);
#else
    SIGFOX_EP_BITSTREAM_build_control_frame(&bitstream_parameters, (sfx_u8*) sigfox_ep_api_ctx.bitstream, &(sigfox_ep_api_ctx.bitstream_size));
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static SIGFOX_EP_API_status_t _build_dl_confirmation_frame(void) {
    SIGFOX_EP_BITSTREAM_control_frame_t bitstream_parameters;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    SIGFOX_EP_BITSTREAM_status_t sigfox_ep_bitstream_status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
    // Read MCU data.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _read_mcu_voltage_temperature();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _read_mcu_voltage_temperature();
#endif
    // Prepare common bitstream parameters.
    _set_common_bitstream_parameters(&(bitstream_parameters.common_parameters));
    // Prepare specific bitstream parameters for DL confirmation.
    bitstream_parameters.message_type = SIGFOX_CONTROL_MESSAGE_TYPE_DL_CONFIRMATION;
    bitstream_parameters.voltage_idle_mv = sigfox_ep_api_ctx.voltage_idle_mv;
    bitstream_parameters.voltage_tx_mv = sigfox_ep_api_ctx.voltage_tx_mv;
    bitstream_parameters.temperature_tenth_degrees = sigfox_ep_api_ctx.temperature_tenth_degrees;
    bitstream_parameters.rssi_dbm = sigfox_ep_api_ctx.dl_rssi_dbm;
    // Build frame.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_bitstream_status = SIGFOX_EP_BITSTREAM_build_control_frame(&bitstream_parameters, (sfx_u8*) sigfox_ep_api_ctx.bitstream, &(sigfox_ep_api_ctx.bitstream_size));
    SIGFOX_EP_BITSTREAM_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_EP_BITSTREAM);
#else
    SIGFOX_EP_BITSTREAM_build_control_frame(&bitstream_parameters, (sfx_u8*) sigfox_ep_api_ctx.bitstream, &(sigfox_ep_api_ctx.bitstream_size));
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
static SIGFOX_EP_API_status_t _build_bitstream(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_ERROR_SEND_FUNCTION_POINTER;
#endif
#ifdef SIGFOX_EP_APPLICATION_MESSAGES
    if ((sigfox_ep_api_ctx.internal_flags.field.control_message == 0) && (sigfox_ep_api_ctx.internal_flags.field.dl_conf_message == 0)) {
#ifdef SIGFOX_EP_ERROR_CODES
        status = _build_application_frame();
#else
        _build_application_frame();
#endif
        goto end;
    }
#endif
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    if ((sigfox_ep_api_ctx.internal_flags.field.control_message != 0) && (sigfox_ep_api_ctx.internal_flags.field.dl_conf_message == 0)) {
#ifdef SIGFOX_EP_ERROR_CODES
        status = _build_control_keep_alive_frame();
#else
        _build_control_keep_alive_frame();
#endif
        goto end;
    }
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    if ((sigfox_ep_api_ctx.internal_flags.field.control_message != 0) && (sigfox_ep_api_ctx.internal_flags.field.dl_conf_message != 0)) {
#ifdef SIGFOX_EP_ERROR_CODES
        status = _build_dl_confirmation_frame();
#else
        _build_dl_confirmation_frame();
#endif
        goto end;
    }
#endif
end:
    SIGFOX_RETURN();
}

#if !(defined SIGFOX_EP_SINGLE_FRAME) || (defined SIGFOX_EP_BIDIRECTIONAL)
/*******************************************************************/
static void _compute_interframe(void) {
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Use Tifu by default.
#ifdef SIGFOX_EP_T_IFU_MS
    sigfox_ep_api_ctx.interframe_ms = SIGFOX_EP_T_IFU_MS;
#else
    sigfox_ep_api_ctx.interframe_ms = ((sigfox_ep_api_ctx.common_parameters_ptr)->t_ifu_ms);
#endif
#endif /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Check DL-CONF flag.
    if (sigfox_ep_api_ctx.internal_flags.field.dl_conf_message != 0) {
        // Use Tconf.
#ifdef SIGFOX_EP_T_CONF_MS
        sigfox_ep_api_ctx.interframe_ms = SIGFOX_EP_T_CONF_MS;
#else
        sigfox_ep_api_ctx.interframe_ms = ((sigfox_ep_api_ctx.application_message_ptr)->t_conf_ms);
#endif
    }
    else {
#ifndef SIGFOX_EP_SINGLE_FRAME
        // Override with Tifb in case of bidirectional.
        if (_is_downlink_required() == SIGFOX_TRUE) {
            sigfox_ep_api_ctx.interframe_ms = SIGFOX_T_IFB_MS;
        }
#else
        sigfox_ep_api_ctx.interframe_ms = 0;
#endif /* SIGFOX_EP_SINGLE_FRAME */
    }
#endif /* SIGFOX_EP_BIDIRECTIONAL */
}
#endif

/*******************************************************************/
static SIGFOX_EP_API_status_t _radio_wake_up(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
    // Wake-up radio if in sleep.
    if (sigfox_ep_api_ctx.internal_flags.field.radio_woken_up == 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        rf_api_status = RF_API_wake_up();
        RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
        RF_API_wake_up();
#endif
        // Update flag.
        sigfox_ep_api_ctx.internal_flags.field.radio_woken_up = 1;
    }
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _radio_sleep(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
    // Turn radio off if active.
    if (sigfox_ep_api_ctx.internal_flags.field.radio_woken_up != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        rf_api_status = RF_API_sleep();
        RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
        RF_API_sleep();
#endif
        // Update flag.
        sigfox_ep_api_ctx.internal_flags.field.radio_woken_up = 0;
    }
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _send_frame(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
    RF_API_radio_parameters_t radio_params;
    RF_API_tx_data_t tx_data;
    // Build radio parameters.
    radio_params.rf_mode = RF_API_MODE_TX;
    radio_params.modulation = RF_API_MODULATION_DBPSK;
    radio_params.frequency_hz = sigfox_ep_api_ctx.ul_frequency_hz;
#ifdef SIGFOX_EP_UL_BIT_RATE_BPS
    radio_params.bit_rate_bps = SIGFOX_EP_UL_BIT_RATE_BPS;
#else
    radio_params.bit_rate_bps = SIGFOX_UL_BIT_RATE_BPS_LIST[(sigfox_ep_api_ctx.common_parameters_ptr)->ul_bit_rate];
#endif
#ifdef SIGFOX_EP_TX_POWER_DBM_EIRP
    radio_params.tx_power_dbm_eirp = SIGFOX_EP_TX_POWER_DBM_EIRP;
#else
    radio_params.tx_power_dbm_eirp = (sigfox_ep_api_ctx.common_parameters_ptr)->tx_power_dbm_eirp;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    radio_params.deviation_hz = 0;
#endif
    // Wake-up radio.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _radio_wake_up();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _radio_wake_up();
#endif
    // Start radio.
#ifdef SIGFOX_EP_ERROR_CODES
    rf_api_status = RF_API_init(&radio_params);
    RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
    RF_API_init(&radio_params);
#endif
    // Send frame.
    tx_data.bitstream = (sfx_u8*) sigfox_ep_api_ctx.bitstream;
    tx_data.bitstream_size_bytes = sigfox_ep_api_ctx.bitstream_size;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    tx_data.cplt_cb = (RF_API_tx_cplt_cb_t) &_RF_API_tx_cplt_cb;
#endif
#ifdef SIGFOX_EP_ERROR_CODES
    rf_api_status = RF_API_send(&tx_data);
    RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
    RF_API_send(&tx_data);
#endif
#ifndef SIGFOX_EP_ASYNCHRONOUS
    // Note: thanks to the RF_API_check_status the next lines are not executed in case of RF API error.
    // Increment success count.
    sigfox_ep_api_ctx.internal_flags.field.frame_success = 1;
#ifndef SIGFOX_EP_SINGLE_FRAME
    sigfox_ep_api_ctx.frame_success_count++;
#endif
#endif /* BLOCKING */
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
#ifndef SIGFOX_EP_ASYNCHRONOUS
    sigfox_ep_api_ctx.irq_flags.field.rf_tx_cplt = 1; // Set flag manually in blocking mode.
#endif
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static SIGFOX_EP_API_status_t _check_dl_frame(void) {
    // Local variables.
    sfx_u8 dl_phy_content[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES];
    sfx_s16 dl_rssi_dbm = 0;
    SIGFOX_EP_BITSTREAM_dl_frame_t bitstream_parameters;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
    SIGFOX_EP_BITSTREAM_status_t sigfox_ep_bitstream_status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
    // Read DL-PHY content received by the radio.
#ifdef SIGFOX_EP_ERROR_CODES
    rf_api_status = RF_API_get_dl_phy_content_and_rssi(dl_phy_content, SIGFOX_DL_PHY_CONTENT_SIZE_BYTES, &dl_rssi_dbm);
    RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
    RF_API_get_dl_phy_content_and_rssi(dl_phy_content, SIGFOX_DL_PHY_CONTENT_SIZE_BYTES, &dl_rssi_dbm);
#endif
    // Update RSSI in global context.
    sigfox_ep_api_ctx.dl_rssi_dbm = dl_rssi_dbm;
    // Prepare bitstream parameters.
    bitstream_parameters.dl_phy_content = (sfx_u8*) dl_phy_content;
    bitstream_parameters.ep_id = (sfx_u8*) sigfox_ep_api_ctx.ep_id;
    bitstream_parameters.message_counter = sigfox_ep_api_ctx.message_counter;
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    bitstream_parameters.ep_key_type = (sigfox_ep_api_ctx.common_parameters_ptr)->ep_key_type;
#endif
#ifdef SIGFOX_EP_CERTIFICATION
    bitstream_parameters.dl_decoding_enable = (sigfox_ep_api_ctx.test_parameters.flags.field.dl_decoding_enable == 0) ? SIGFOX_FALSE : SIGFOX_TRUE;
#endif
    // Check DL frame.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_bitstream_status = SIGFOX_EP_BITSTREAM_decode_downlink_frame(&bitstream_parameters, &sigfox_ep_api_ctx.dl_status, (sfx_u8*) sigfox_ep_api_ctx.dl_payload);
    SIGFOX_EP_BITSTREAM_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_EP_BITSTREAM);
#else
    SIGFOX_EP_BITSTREAM_decode_downlink_frame(&bitstream_parameters, &sigfox_ep_api_ctx.dl_status, (sfx_u8*) sigfox_ep_api_ctx.dl_payload);
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#if (!(defined SIGFOX_EP_SINGLE_FRAME) && (!(defined SIGFOX_EP_T_IFU_MS) || (SIGFOX_EP_T_IFU_MS > 0))) || (defined SIGFOX_EP_BIDIRECTIONAL)
/*******************************************************************/
static SIGFOX_EP_API_status_t _start_timer(sfx_u32 duration_ms, MCU_API_timer_instance_t timer_instance, MCU_API_timer_reason_t timer_reason) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
    MCU_API_timer_t mcu_timer;
#ifdef SIGFOX_EP_LATENCY_COMPENSATION
#ifdef SIGFOX_EP_ERROR_CODES
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
    sfx_u32 rf_latency[RF_API_LATENCY_LAST];
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_u32 mcu_latency[MCU_API_LATENCY_LAST];
#endif
    sfx_s32 offset = 0;
    sfx_u8 idx = 0;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check reason.
    if (timer_reason >= MCU_API_TIMER_REASON_LAST) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_TIMER_REASON);
    }
#endif
    // Build timer structure.
    mcu_timer.instance = timer_instance;
    mcu_timer.reason = timer_reason;
    mcu_timer.duration_ms = duration_ms;
#ifdef SIGFOX_EP_LATENCY_COMPENSATION
    // Read RF latencies.
    for (idx = 0; idx < RF_API_LATENCY_LAST; idx++) {
        // Reset latency.
        rf_latency[idx] = 0;
#ifdef SIGFOX_EP_ERROR_CODES
        rf_api_status = RF_API_get_latency(idx, &(rf_latency[idx]));
        RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
        RF_API_get_latency(idx, &(rf_latency[idx]));
#endif
    }
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Read MCU latencies.
    for (idx = 0; idx < MCU_API_LATENCY_LAST; idx++) {
        // Reset latency.
        mcu_latency[idx] = 0;
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_get_latency(idx, &(mcu_latency[idx]));
        MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
        MCU_API_get_latency(idx, &(mcu_latency[idx]));
#endif
    }
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    // Compute offset.
    switch (timer_reason) {
#if !(defined SIGFOX_EP_SINGLE_FRAME) && (!(defined SIGFOX_EP_T_IFU_MS) || (SIGFOX_EP_T_IFU_MS > 0) || (defined SIGFOX_EP_BIDIRECTIONAL))
    case MCU_API_TIMER_REASON_T_IFX:
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_DE_INIT_TX];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_SEND_STOP];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_SEND_START];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_INIT_TX];
        break;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case MCU_API_TIMER_REASON_T_W:
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_SEND_STOP];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_DE_INIT_TX];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_WAKE_UP];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_INIT_RX];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_RECEIVE_START];
        break;
    case MCU_API_TIMER_REASON_T_CONF:
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_RECEIVE_STOP];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_DE_INIT_RX];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_SLEEP];
        offset -= (sfx_s32) mcu_latency[MCU_API_LATENCY_GET_VOLTAGE_TEMPERATURE];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_WAKE_UP];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_INIT_TX];
        offset -= (sfx_s32) rf_latency[RF_API_LATENCY_SEND_START];
        break;
    case MCU_API_TIMER_REASON_T_RX:
        offset += (sfx_s32) rf_latency[RF_API_LATENCY_WAKE_UP];
        offset += (sfx_s32) rf_latency[RF_API_LATENCY_INIT_RX];
        offset += (sfx_s32) rf_latency[RF_API_LATENCY_RECEIVE_START];
        offset += (sfx_s32) rf_latency[RF_API_LATENCY_RECEIVE_STOP];
        break;
#endif /* SIGFOX_EP_BIDIRECTIONAL */
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_TIMER_REASON);
        break;
    }
    // Apply offset.
    mcu_timer.duration_ms = (((((sfx_s32) duration_ms) + offset) > 0) ? ((sfx_u32) (((sfx_s32) duration_ms) + offset)) : 0);
#endif /* SIGFOX_EP_LATENCY_COMPENSATION */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Start timer.
    switch (timer_instance) {
    case MCU_API_TIMER_1:
        mcu_timer.cplt_cb = (MCU_API_timer_cplt_cb_t) &_MCU_API_timer1_cplt_cb;
        break;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case MCU_API_TIMER_2:
        mcu_timer.cplt_cb = (MCU_API_timer_cplt_cb_t) &_MCU_API_timer2_cplt_cb;
        break;
#endif
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_TIMER_INSTANCE);
        break;
    }
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_timer_start(&mcu_timer);
    MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
    MCU_API_timer_start(&mcu_timer);
#endif
#if (defined SIGFOX_EP_PARAMETERS_CHECK) || (defined SIGFOX_EP_LATENCY_COMPENSATION) || (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_ERROR_CODES)
errors:
#endif
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
static SIGFOX_EP_API_status_t _end_transmission(void);

/*******************************************************************/
static SIGFOX_EP_API_status_t _start_transmission(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#ifdef SIGFOX_EP_REGULATORY
    SIGFOX_TX_CONTROL_status_t sigfox_tx_control_status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#endif
#ifdef SIGFOX_EP_REGULATORY
    SIGFOX_TX_CONTROL_parameters_t tx_control_params;
    SIGFOX_TX_CONTROL_result_t tx_control_pre_check_result = SIGFOX_TX_CONTROL_RESULT_FORBIDDEN;
#endif
    // Reset TX success flag.
    sigfox_ep_api_ctx.internal_flags.field.frame_success = 0;
    // Compute bitstream.
    // This must be done here since TX control could need bitstream size.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _build_bitstream();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _build_bitstream();
#endif
    // Compute frequency.
    // This must be done here since the frame 1 frequency has to be known for eventual bidirectional procedure, even if the frame itself is not sent (due to error or TX control).
#ifdef SIGFOX_EP_ERROR_CODES
    status = _compute_next_ul_frequency();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _compute_next_ul_frequency();
#endif
#ifdef SIGFOX_EP_REGULATORY
    // Check if radio is required for pre-check.
    if (SIGFOX_TX_CONTROL_is_radio_required(SIGFOX_TX_CONTROL_TYPE_PRE_CHECK) == SIGFOX_TRUE) {
#ifdef SIGFOX_EP_ERROR_CODES
        status = _radio_wake_up();
        SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _radio_wake_up();
#endif
    }
    _set_tx_control_parameters(SIGFOX_TX_CONTROL_TYPE_PRE_CHECK, &tx_control_params);
    // Start TX control.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_tx_control_status = SIGFOX_TX_CONTROL_check(&tx_control_params);
    SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
#else
    SIGFOX_TX_CONTROL_check(&tx_control_params);
#endif
    sigfox_ep_api_ctx.internal_flags.field.tx_control_pre_check_running = 1;
#ifndef SIGFOX_EP_ASYNCHRONOUS
    sigfox_ep_api_ctx.irq_flags.field.tx_control_pre_check_cplt = 1; // Set flag manually in blocking mode.
#endif
    // Read result.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_tx_control_status = SIGFOX_TX_CONTROL_get_result(&tx_control_pre_check_result);
    SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
#else
    SIGFOX_TX_CONTROL_get_result(&tx_control_pre_check_result);
#endif
    // Update state according to result.
    switch (tx_control_pre_check_result) {
    case SIGFOX_TX_CONTROL_RESULT_ALLOWED:
        // Send frame.
#ifdef SIGFOX_EP_ERROR_CODES
        status = _send_frame();
        SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _send_frame();
#endif
        // Update state.
        sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_UL_MODULATION_PENDING;
        break;
    case SIGFOX_TX_CONTROL_RESULT_FORBIDDEN:
        // Set network error flag.
        sigfox_ep_api_ctx.message_status.field.network_error = 1;
#ifdef SIGFOX_EP_ERROR_STACK
        // Add error to stack.
        SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_SIGFOX_EP_API, SIGFOX_EP_API_ERROR_TX_FORBIDDEN);
#endif
        // Try next frame.
#ifdef SIGFOX_EP_ERROR_CODES
        status = _end_transmission();
        SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _end_transmission();
#endif
        break;
    case SIGFOX_TX_CONTROL_RESULT_PENDING:
        // Wait for completion.
        sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_REGULATORY;
        break;
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
        break;
    }
#else /* SIGFOX_EP_REGULATORY */
    // Send frame.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _send_frame();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _send_frame();
#endif
    // Update state.
    sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_UL_MODULATION_PENDING;
#endif /* SIGFOX_EP_REGULATORY */
#if (defined SIGFOX_EP_ERROR_CODES) || (defined SIGFOX_EP_REGULATORY)
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
static void _update_message_status(void) {
#ifdef SIGFOX_EP_SINGLE_FRAME
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Update message status.
    if (sigfox_ep_api_ctx.internal_flags.field.dl_conf_message != 0) {
        sigfox_ep_api_ctx.message_status.field.dl_conf_frame = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
    }
    else {
        sigfox_ep_api_ctx.message_status.field.ul_frame_1 = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
    }
#else /* SIGFOX_EP_BIDIRECTIONAL */
    // Update message status.
    sigfox_ep_api_ctx.message_status.field.ul_frame_1 = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#else /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Update message status.
    if (sigfox_ep_api_ctx.internal_flags.field.dl_conf_message != 0) {
        sigfox_ep_api_ctx.message_status.field.dl_conf_frame = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
    }
    else {
        switch (sigfox_ep_api_ctx.ul_frame_rank) {
        case SIGFOX_UL_FRAME_RANK_1:
            sigfox_ep_api_ctx.message_status.field.ul_frame_1 = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
            break;
        case SIGFOX_UL_FRAME_RANK_2:
            sigfox_ep_api_ctx.message_status.field.ul_frame_2 = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
            break;
        case SIGFOX_UL_FRAME_RANK_3:
            sigfox_ep_api_ctx.message_status.field.ul_frame_3 = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
            break;
        default:
            break;
        }
    }
#else /* SIGFOX_EP_BIDIRECTIONAL */
    switch (sigfox_ep_api_ctx.ul_frame_rank) {
    case SIGFOX_UL_FRAME_RANK_1:
        sigfox_ep_api_ctx.message_status.field.ul_frame_1 = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
        break;
    case SIGFOX_UL_FRAME_RANK_2:
        sigfox_ep_api_ctx.message_status.field.ul_frame_2 = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
        break;
    case SIGFOX_UL_FRAME_RANK_3:
        sigfox_ep_api_ctx.message_status.field.ul_frame_3 = (sfx_u8) ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) ? 1 : 0);
        break;
    default:
        break;
    }
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#endif /* SIGFOX_EP_SINGLE_FRAME */
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _write_nvm(void) {
    // Local variable.
    sfx_u8 nvm_data[SIGFOX_NVM_DATA_SIZE_BYTES];
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
    // Write message counter and random value in NVM when at least one frame has been successfully transmitted.
#ifdef SIGFOX_EP_SINGLE_FRAME
    if ((sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) && (sigfox_ep_api_ctx.internal_flags.field.nvm_write_pending != 0)) {
#else
    if ((sigfox_ep_api_ctx.frame_success_count > 0) && (sigfox_ep_api_ctx.internal_flags.field.nvm_write_pending != 0)) {
#endif
        // Reset flag.
        sigfox_ep_api_ctx.internal_flags.field.nvm_write_pending = 0;
        // Build local NVM array.
        nvm_data[SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_MSB] = (sfx_u8) ((sigfox_ep_api_ctx.message_counter >> 8) & 0x00FF);
        nvm_data[SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_LSB] = (sfx_u8) ((sigfox_ep_api_ctx.message_counter >> 0) & 0x00FF);
        nvm_data[SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_MSB] = (sfx_u8) ((sigfox_ep_api_ctx.random_value >> 8) & 0x00FF);
        nvm_data[SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_LSB] = (sfx_u8) ((sigfox_ep_api_ctx.random_value >> 0) & 0x00FF);
        // Write NVM.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_set_nvm(nvm_data, SIGFOX_NVM_DATA_SIZE_BYTES);
        MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
        MCU_API_set_nvm(nvm_data, SIGFOX_NVM_DATA_SIZE_BYTES);
#endif
    }
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _compute_state_after_transmission(void) {
    // Local variables.
    SIGFOX_EP_API_state_t state = SIGFOX_EP_API_STATE_READY;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#if (defined SIGFOX_EP_BIDIRECTIONAL) && !(defined SIGFOX_EP_SINGLE_FRAME)
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#endif /* SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_SINGLE_FRAME
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Compute next state.
    if (_is_downlink_required() == SIGFOX_TRUE) {
        // Abort downlink sequence if the frame has not been sent due to error or TX control.
        if (sigfox_ep_api_ctx.internal_flags.field.frame_success == 0) {
            // Force bidirectional to 0 in order to call the message completion callback.
            if (sigfox_ep_api_ctx.application_message_ptr != SIGFOX_NULL) {
                (sigfox_ep_api_ctx.application_message_ptr)->bidirectional_flag = SIGFOX_FALSE;
            }
            // Force state to ready.
            state = SIGFOX_EP_API_STATE_READY;
        }
        else {
            // Wait for DL_T_W timer.
            state = SIGFOX_EP_API_STATE_DL_TIMER;
        }
    }
    else {
        state = SIGFOX_EP_API_STATE_READY;
    }
#else /* SIGFOX_EP_BIDIRECTIONAL */
    // Compute next state.
    state = SIGFOX_EP_API_STATE_READY;
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#else /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Compute next state.
    if (_is_last_frame_of_uplink_sequence() == SIGFOX_TRUE) {
        // Check bidirectional condition.
        if (_is_downlink_required() == SIGFOX_TRUE) {
            // Abort downlink sequence if none frame has been sent due to error or TX control.
            if (sigfox_ep_api_ctx.frame_success_count == 0) {
                // Stop DL_T_W timer.
#ifdef SIGFOX_EP_ERROR_CODES
                mcu_api_status = MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_W);
                MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
                MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_W);
#endif
                // Force bidirectional to 0 in order to call the message completion callback.
                if (sigfox_ep_api_ctx.application_message_ptr != SIGFOX_NULL) {
                    (sigfox_ep_api_ctx.application_message_ptr)->bidirectional_flag = SIGFOX_FALSE;
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
        // Start inter-frame timer if required.
        if ((sigfox_ep_api_ctx.interframe_ms) != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_timer(sigfox_ep_api_ctx.interframe_ms, MCU_API_TIMER_INSTANCE_T_IFX, MCU_API_TIMER_REASON_T_IFX);
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_timer(sigfox_ep_api_ctx.interframe_ms, MCU_API_TIMER_INSTANCE_T_IFX, MCU_API_TIMER_REASON_T_IFX);
#endif
            // Update state.
            state = SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER;
        }
        else {
            // Inter-frame delay set to 0, directly start next frame.
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_transmission();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_transmission();
#endif
        }
    }
#else /* SIGFOX_EP_BIDIRECTIONAL */
    // Compute next state.
    if (_is_last_frame_of_uplink_sequence() == SIGFOX_TRUE) {
        state = SIGFOX_EP_API_STATE_READY;
    }
    else {
        // Start inter-frame timer if required.
        if ((sigfox_ep_api_ctx.interframe_ms) != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_timer(sigfox_ep_api_ctx.interframe_ms, MCU_API_TIMER_INSTANCE_T_IFX, MCU_API_TIMER_REASON_T_IFX);
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_timer(sigfox_ep_api_ctx.interframe_ms, MCU_API_TIMER_INSTANCE_T_IFX, MCU_API_TIMER_REASON_T_IFX);
#endif
            // Update state.
            state = SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER;
        }
        else {
            // Inter-frame delay set to 0, directly start next frame.
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_transmission();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_transmission();
#endif
        }
    }
#endif /* SIGFOX_EP_BIDIRECTIONAL */
#endif /* SIGFOX_EP_SINGLE_FRAME */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Manage API callbacks.
    if ((_is_last_frame_of_uplink_sequence() == SIGFOX_TRUE) && (sigfox_ep_api_ctx.internal_flags.field.dl_conf_message == 0)) {
        // Call uplink completion callback (except for ACK message).
        _UPLINK_CPLT_CALLBACK();
    }
    if (_is_last_frame_of_message_sequence() == SIGFOX_TRUE) {
        // Call message completion callback.
        _MESSAGE_CPLT_CALLBACK();
    }
#endif /* SIGFOX_EP_ASYNCHRONOUS */
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Increment frame rank.
    sigfox_ep_api_ctx.ul_frame_rank++;
#endif
    // Update global state.
    sigfox_ep_api_ctx.state = state;
#if (defined SIGFOX_EP_ERROR_CODES) && (!defined SIGFOX_EP_SINGLE_FRAME)
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _end_transmission(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#ifdef SIGFOX_EP_REGULATORY
    SIGFOX_TX_CONTROL_status_t sigfox_tx_control_status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#endif
#ifdef SIGFOX_EP_REGULATORY
    SIGFOX_TX_CONTROL_parameters_t tx_control_params;
    SIGFOX_TX_CONTROL_result_t tx_control_post_check_result = SIGFOX_TX_CONTROL_RESULT_FORBIDDEN;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_u32 dl_t_w_ms = 0;
#endif
    // Update message status.
    _update_message_status();
    // Stop RF.
    // Note: if TX control returned forbidden result, the RF_API_de_init function was already called by the TX control itself.
    if (sigfox_ep_api_ctx.internal_flags.field.frame_success != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        rf_api_status = RF_API_de_init();
        RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
        RF_API_de_init();
#endif
    }
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Set downlink T_W value.
#ifdef SIGFOX_EP_CERTIFICATION
    dl_t_w_ms = (sigfox_ep_api_ctx.test_parameters.dl_t_w_ms != 0) ? sigfox_ep_api_ctx.test_parameters.dl_t_w_ms : (sfx_u32) (((sigfox_ep_api_ctx.rc_ptr)->spectrum_access)->dl_t_w_ms);
#else
    dl_t_w_ms = (((sigfox_ep_api_ctx.rc_ptr)->spectrum_access)->dl_t_w_ms);
#endif
    // Start Tw timer if required.
#ifdef SIGFOX_EP_SINGLE_FRAME
    if (_is_downlink_required() == SIGFOX_TRUE) {
#else
    if ((sigfox_ep_api_ctx.ul_frame_rank == SIGFOX_UL_FRAME_RANK_1) && (_is_downlink_required() == SIGFOX_TRUE)) {
#endif
#ifdef SIGFOX_EP_ERROR_CODES
        status = _start_timer(dl_t_w_ms, MCU_API_TIMER_INSTANCE_T_W, MCU_API_TIMER_REASON_T_W);
        SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _start_timer(dl_t_w_ms, MCU_API_TIMER_INSTANCE_T_W, MCU_API_TIMER_REASON_T_W);
#endif
    }
#endif
    // Manage radio state and NVM.
    if (_is_last_frame_of_uplink_sequence() == SIGFOX_TRUE) {
#ifdef SIGFOX_EP_REGULATORY
        if (SIGFOX_TX_CONTROL_is_radio_required(SIGFOX_TX_CONTROL_TYPE_POST_CHECK) == SIGFOX_FALSE) {
#ifdef SIGFOX_EP_ERROR_CODES
            status = _radio_sleep();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _radio_sleep();
#endif
        }
#else /* SIGFOX_EP_REGULATORY */
#ifdef SIGFOX_EP_ERROR_CODES
        status = _radio_sleep();
        SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _radio_sleep();
#endif
#endif /* SIGFOX_EP_REGULATORY */
        // Write NVM.
#ifdef SIGFOX_EP_ERROR_CODES
        status = _write_nvm();
        SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _write_nvm();
#endif
    }
#ifdef SIGFOX_EP_REGULATORY
    // Set parameters.
    _set_tx_control_parameters(SIGFOX_TX_CONTROL_TYPE_POST_CHECK, &tx_control_params);
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_tx_control_status = SIGFOX_TX_CONTROL_check(&tx_control_params);
    SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
#else
    SIGFOX_TX_CONTROL_check(&tx_control_params);
#endif
    sigfox_ep_api_ctx.internal_flags.field.tx_control_post_check_running = 1;
#ifndef SIGFOX_EP_ASYNCHRONOUS
    sigfox_ep_api_ctx.irq_flags.field.tx_control_post_check_cplt = 1; // Set flag manually in blocking mode.
#endif
    // Read result.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_tx_control_status = SIGFOX_TX_CONTROL_get_result(&tx_control_post_check_result);
    SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
#else
    SIGFOX_TX_CONTROL_get_result(&tx_control_post_check_result);
#endif
    // Update state according to result.
    switch (tx_control_post_check_result) {
    case SIGFOX_TX_CONTROL_RESULT_FORBIDDEN:
        // Set network error flag.
        sigfox_ep_api_ctx.message_status.field.network_error = 1;
#ifdef SIGFOX_EP_ERROR_STACK
        // Add error to stack.
        SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_SIGFOX_EP_API, SIGFOX_EP_API_ERROR_TX_FORBIDDEN);
#endif
        // Compute next state.
#ifdef SIGFOX_EP_ERROR_CODES
        status = _compute_state_after_transmission();
        SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _compute_state_after_transmission();
#endif
        break;
    case SIGFOX_TX_CONTROL_RESULT_ALLOWED:
        // Compute next state.
#ifdef SIGFOX_EP_ERROR_CODES
        status = _compute_state_after_transmission();
        SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _compute_state_after_transmission();
#endif
        break;
    case SIGFOX_TX_CONTROL_RESULT_PENDING:
        // Wait for completion.
        sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_REGULATORY;
        break;
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
        break;
    }
#else /* SIGFOX_EP_REGULATORY */
    // Compute next state.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _compute_state_after_transmission();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _compute_state_after_transmission();
#endif
#endif /* SIGFOX_EP_REGULATORY */
#if (defined SIGFOX_EP_ERROR_CODES) || (defined SIGFOX_EP_REGULATORY)
errors:
#endif
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static SIGFOX_EP_API_status_t _start_reception(void) {
    // Local variables.
    RF_API_radio_parameters_t radio_params;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
    SIGFOX_EP_FREQUENCY_status_t sigfox_ep_frequency_status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
    // Reset downlink status.
    sigfox_ep_api_ctx.dl_status = SIGFOX_FALSE;
    // Set radio configuration for downlink reception.
    radio_params.rf_mode = RF_API_MODE_RX;
    radio_params.modulation = RF_API_MODULATION_GFSK;
    radio_params.bit_rate_bps = SIGFOX_DL_BIT_RATE_BPS;
    radio_params.deviation_hz = SIGFOX_DL_GFSK_DEVIATION_HZ;
    // Set downlink frequency.
#ifdef SIGFOX_EP_CERTIFICATION
    // Bypass frequency if needed.
    if ((sigfox_ep_api_ctx.test_parameters.rx_frequency_hz) != 0) {
        radio_params.frequency_hz = sigfox_ep_api_ctx.test_parameters.rx_frequency_hz;
    }
    else {
#endif
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_frequency_status = SIGFOX_EP_FREQUENCY_compute_downlink(&radio_params.frequency_hz);
    SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_EP_FREQUENCY);
#else
    SIGFOX_EP_FREQUENCY_compute_downlink(&radio_params.frequency_hz);
#endif
#ifdef SIGFOX_EP_CERTIFICATION
    }
#endif
#ifdef SIGFOX_EP_ERROR_CODES
    rf_api_status = RF_API_init(&radio_params);
    RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
    RF_API_init(&radio_params);
#endif
    // Prepare RX data structure.
#ifdef SIGFOX_EP_ASYNCHRONOUS
    sigfox_ep_api_ctx.rx_data.data_received_cb = (RF_API_rx_data_received_cb_t) &_RF_API_rx_data_received_cb;
#else
    sigfox_ep_api_ctx.rx_data.data_received = SIGFOX_FALSE;
#endif
    // Start DL reception.
#ifdef SIGFOX_EP_ERROR_CODES
    rf_api_status = RF_API_receive(&sigfox_ep_api_ctx.rx_data);
    RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
    RF_API_receive(&sigfox_ep_api_ctx.rx_data);
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static SIGFOX_EP_API_status_t _start_dl_sequence(void) {
    // Local variables.
    sfx_u32 dl_t_rx_ms = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
    // Set downlink T_RX value.
#ifdef SIGFOX_EP_CERTIFICATION
    dl_t_rx_ms = (sigfox_ep_api_ctx.test_parameters.dl_t_rx_ms != 0) ? (sigfox_ep_api_ctx.test_parameters.dl_t_rx_ms) : (sfx_u32) (((sigfox_ep_api_ctx.rc_ptr)->spectrum_access)->dl_t_rx_ms);
#else
    dl_t_rx_ms = (((sigfox_ep_api_ctx.rc_ptr)->spectrum_access)->dl_t_rx_ms);
#endif
    // Start DL_T_RX timer.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _start_timer(dl_t_rx_ms, MCU_API_TIMER_INSTANCE_T_RX, MCU_API_TIMER_REASON_T_RX);
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _start_timer(dl_t_rx_ms, MCU_API_TIMER_INSTANCE_T_RX, MCU_API_TIMER_REASON_T_RX);
#endif
    // Wake-up radio.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _radio_wake_up();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _radio_wake_up();
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
static void _message_prepare(void) {
    // Increment message counter.
    // Note: operation is only performed in RAM first, it will be write in NVM when at least 1 frame has been successfully transmitted.
    // Note: the nvm_write_pending flag is checked to prevent multiple increments if the previous value was not written in NVM (due to TX control forbidden or error).
#ifdef SIGFOX_EP_CERTIFICATION
    // Do not increment message counter when UL is disabled.
    if (sigfox_ep_api_ctx.test_parameters.flags.field.ul_enable != 0) {
#endif
    if (sigfox_ep_api_ctx.internal_flags.field.nvm_write_pending == 0) {
#ifdef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
        sigfox_ep_api_ctx.message_counter = (sfx_u16) ((sigfox_ep_api_ctx.message_counter + 1) % SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER);
#else
        sigfox_ep_api_ctx.message_counter = (sfx_u16) ((sigfox_ep_api_ctx.message_counter + 1) % (sigfox_ep_api_ctx.message_counter_rollover));
#endif
        // Update flag.
        sigfox_ep_api_ctx.internal_flags.field.nvm_write_pending = 1;
    }
#ifdef SIGFOX_EP_CERTIFICATION
    }
#endif
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Reset frame rank.
    sigfox_ep_api_ctx.ul_frame_rank = SIGFOX_UL_FRAME_RANK_1;
    sigfox_ep_api_ctx.frame_success_count = 0;
#endif
#if !(defined SIGFOX_EP_SINGLE_FRAME) || (defined SIGFOX_EP_BIDIRECTIONAL)
    // Compute interframe duration.
    _compute_interframe();
#endif
}

/*******************************************************************/
static SIGFOX_EP_API_status_t _internal_process(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_BIDIRECTIONAL)
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_BIDIRECTIONAL) || (!(defined SIGFOX_EP_SINGLE_FRAME) && (!(defined SIGFOX_EP_T_IFU_MS) || (SIGFOX_EP_T_IFU_MS > 0)))
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_REGULATORY
    SIGFOX_TX_CONTROL_status_t sigfox_tx_control_status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#endif /* SIGFOX_EP_ERROR_CODES */
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_bool local_rx_data_received_flag = SIGFOX_FALSE;
    sfx_bool local_mcu_timer1_cplt_flag = SIGFOX_FALSE;
#endif
#ifdef SIGFOX_EP_REGULATORY
    SIGFOX_TX_CONTROL_result_t tx_control_result = SIGFOX_TX_CONTROL_RESULT_FORBIDDEN;
#endif
    // Check library is opened.
    _CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Check error flag in case the process was called after a low level error callback.
    if (sigfox_ep_api_ctx.irq_flags.field.low_level_error != 0) {
        // Clear flag.
        sigfox_ep_api_ctx.irq_flags.field.low_level_error = 0;
        // Check MCU status.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = sigfox_ep_api_ctx.mcu_api_status_from_callback;
        MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
        goto errors;
#endif
        // Check RF status.
#ifdef SIGFOX_EP_ERROR_CODES
        rf_api_status = sigfox_ep_api_ctx.rf_api_status_from_callback;
        RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
        goto errors;
#endif
    }
#endif
    // Check low level process flags.
#ifdef SIGFOX_EP_ASYNCHRONOUS
    if (sigfox_ep_api_ctx.irq_flags.field.mcu_api_process != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_process();
        sigfox_ep_api_ctx.irq_flags.field.mcu_api_process = 0;
        MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
        MCU_API_process();
        sigfox_ep_api_ctx.irq_flags.field.mcu_api_process = 0;
#endif
    }
    if (sigfox_ep_api_ctx.irq_flags.field.rf_api_process != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        rf_api_status = RF_API_process();
        sigfox_ep_api_ctx.irq_flags.field.rf_api_process = 0;
        RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
        RF_API_process();
        sigfox_ep_api_ctx.irq_flags.field.rf_api_process = 0;
#endif
    }
#ifdef SIGFOX_EP_REGULATORY
    if (sigfox_ep_api_ctx.irq_flags.field.tx_control_process != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        sigfox_tx_control_status = SIGFOX_TX_CONTROL_process();
        sigfox_ep_api_ctx.irq_flags.field.tx_control_process = 0;
        SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
#else
        SIGFOX_TX_CONTROL_process();
        sigfox_ep_api_ctx.irq_flags.field.tx_control_process = 0;
#endif
    }
#endif
#endif
    // Perform internal state machine.
    switch (sigfox_ep_api_ctx.state) {
    case SIGFOX_EP_API_STATE_READY:
        // Check pending requests.
        if (sigfox_ep_api_ctx.internal_flags.field.send_message_request != 0) {
#ifdef SIGFOX_EP_CERTIFICATION
            // Check uplink bypass flag.
            if (sigfox_ep_api_ctx.test_parameters.flags.field.ul_enable != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
                status = _start_transmission();
                SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
                _start_transmission();
#endif
            }
            else {
                // Bypass uplink sequence.
#ifdef SIGFOX_EP_BIDIRECTIONAL
                if (_is_downlink_required() == SIGFOX_TRUE) {
                    // Directly start downlink sequence.
#ifdef SIGFOX_EP_ERROR_CODES
                    status = _start_dl_sequence();
                    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
                    _start_dl_sequence();
#endif
                    // Start DL reception.
#ifdef SIGFOX_EP_ERROR_CODES
                    status = _start_reception();
                    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
                    _start_reception();
#endif
                    // Update state.
                    sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_DL_LISTENING;
                }
                else {
                    // End of sequence.
#ifdef SIGFOX_EP_ASYNCHRONOUS
                    sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_UL_MODULATION_PENDING; // Hack to trigger message completion.
                    _MESSAGE_CPLT_CALLBACK();
#endif
                    sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_READY;
                }
#else
                // End of sequence.
#ifdef SIGFOX_EP_ASYNCHRONOUS
                _MESSAGE_CPLT_CALLBACK();
#endif
                sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_READY;
#endif
            }
#else
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_transmission();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_transmission();
#endif
#endif /* SIGFOX_EP_CERTIFICATION */
            // Clear request.
            sigfox_ep_api_ctx.internal_flags.field.send_message_request = 0;
        }
        break;
#ifdef SIGFOX_EP_REGULATORY
    case SIGFOX_EP_API_STATE_REGULATORY:
        // Check completion flags.
        if ((sigfox_ep_api_ctx.irq_flags.field.tx_control_post_check_cplt != 0) || (sigfox_ep_api_ctx.irq_flags.field.tx_control_pre_check_cplt != 0)) {
            // Check pre-check flags.
            if (sigfox_ep_api_ctx.irq_flags.field.tx_control_pre_check_cplt != 0) {
                // Clear flags.
                sigfox_ep_api_ctx.irq_flags.field.tx_control_pre_check_cplt = 0;
                sigfox_ep_api_ctx.internal_flags.field.tx_control_pre_check_running = 0;
                // Read result.
#ifdef SIGFOX_EP_ERROR_CODES
                sigfox_tx_control_status = SIGFOX_TX_CONTROL_get_result(&tx_control_result);
                SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
#else
                SIGFOX_TX_CONTROL_get_result(&tx_control_result);
#endif
                // Check TX control status.
                if (tx_control_result == SIGFOX_TX_CONTROL_RESULT_ALLOWED) {
                    // Send frame.
#ifdef SIGFOX_EP_ERROR_CODES
                    status = _send_frame();
                    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
                    _send_frame();
#endif
                    // Update state.
                    sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_UL_MODULATION_PENDING;
                }
                else {
                    // Set network error flag.
                    sigfox_ep_api_ctx.message_status.field.network_error = 1;
#ifdef SIGFOX_EP_ERROR_STACK
                    // Add error to stack.
                    SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_SIGFOX_EP_API, SIGFOX_EP_API_ERROR_TX_FORBIDDEN);
#endif
                    // Compute next state.
#ifdef SIGFOX_EP_ERROR_CODES
                    status = _end_transmission();
                    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
                    _end_transmission();
#endif
                }
            }
            // Check post-check flag.
            if (sigfox_ep_api_ctx.irq_flags.field.tx_control_post_check_cplt != 0) {
                // Clear flags.
                sigfox_ep_api_ctx.irq_flags.field.tx_control_post_check_cplt = 0;
                sigfox_ep_api_ctx.internal_flags.field.tx_control_post_check_running = 0;
                // Read result.
#ifdef SIGFOX_EP_ERROR_CODES
                sigfox_tx_control_status = SIGFOX_TX_CONTROL_get_result(&tx_control_result);
                SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
#else
                SIGFOX_TX_CONTROL_get_result(&tx_control_result);
#endif
                // Set error flags if needed.
                if (tx_control_result == SIGFOX_TX_CONTROL_RESULT_FORBIDDEN) {
                    // Set network error flag.
                    sigfox_ep_api_ctx.message_status.field.network_error = 1;
#ifdef SIGFOX_EP_ERROR_STACK
                    // Add error to stack.
                    SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_SIGFOX_EP_API, SIGFOX_EP_API_ERROR_TX_FORBIDDEN);
#endif
                }
                // Switch to next state whatever the result.
#ifdef SIGFOX_EP_ERROR_CODES
                status = _compute_state_after_transmission();
                SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
                _compute_state_after_transmission();
#endif
            }
        }
        break;
#endif
    case SIGFOX_EP_API_STATE_UL_MODULATION_PENDING:
        // Check end of transmission flag..
        if (sigfox_ep_api_ctx.irq_flags.field.rf_tx_cplt != 0) {
            // Clear flag.
            sigfox_ep_api_ctx.irq_flags.field.rf_tx_cplt = 0;
#ifdef SIGFOX_EP_ASYNCHRONOUS
            // Increment success count.
            sigfox_ep_api_ctx.internal_flags.field.frame_success = 1;
#ifndef SIGFOX_EP_SINGLE_FRAME
            sigfox_ep_api_ctx.frame_success_count++;
#endif
#endif
            // End transmission.
#ifdef SIGFOX_EP_ERROR_CODES
            status = _end_transmission();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _end_transmission();
#endif
        }
        break;
#if !(defined SIGFOX_EP_SINGLE_FRAME) && (!(defined SIGFOX_EP_T_IFU_MS) || (SIGFOX_EP_T_IFU_MS > 0) || (defined SIGFOX_EP_BIDIRECTIONAL))
    case SIGFOX_EP_API_STATE_UL_INTER_FRAME_TIMER:
#ifndef SIGFOX_EP_ASYNCHRONOUS
        // Wait for timer completion.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_timer_wait_cplt(MCU_API_TIMER_INSTANCE_T_IFX);
        MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
        MCU_API_timer_wait_cplt(MCU_API_TIMER_INSTANCE_T_IFX);
#endif
        sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt = 1; // Set flag manually in blocking mode.
#endif
        // Check timer completion flag.
        if (sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt != 0) {
            // Stop timer.
#ifdef SIGFOX_EP_ERROR_CODES
            mcu_api_status = MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_IFX);
            MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
            MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_IFX);
#endif
            sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt = 0;
            // Start next frame.
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_transmission();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_transmission();
#endif
        }
        break;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case SIGFOX_EP_API_STATE_DL_TIMER:
#ifndef SIGFOX_EP_ASYNCHRONOUS
        // Wait for timer completion.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_timer_wait_cplt(MCU_API_TIMER_INSTANCE_T_W);
        MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
        MCU_API_timer_wait_cplt(MCU_API_TIMER_INSTANCE_T_W);
#endif
        sigfox_ep_api_ctx.irq_flags.field.mcu_timer2_cplt = 1; // Set flag manually in blocking mode.
#endif /* BLOCKING */
        // Check timer completion flag.
        if (sigfox_ep_api_ctx.irq_flags.field.mcu_timer2_cplt != 0) {
            // Stop timer.
#ifdef SIGFOX_EP_ERROR_CODES
            mcu_api_status = MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_W);
            MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
            MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_W);
#endif
            sigfox_ep_api_ctx.irq_flags.field.mcu_timer2_cplt = 0;
            // Start DL sequence.
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_dl_sequence();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_dl_sequence();
#endif
            // Start DL reception.
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_reception();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_reception();
#endif
            // Update state.
            sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_DL_LISTENING;
        }
        break;
    case SIGFOX_EP_API_STATE_DL_LISTENING:
#ifndef SIGFOX_EP_ASYNCHRONOUS
        // If the _start_reception() function exited with dl_frame_received flag set to 0, it means that the DL window timer has elapsed.
        sigfox_ep_api_ctx.irq_flags.field.rf_rx_data_received = sigfox_ep_api_ctx.rx_data.data_received;
        sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt = !(sigfox_ep_api_ctx.rx_data.data_received);
#endif
        // Capture IRQ flags once to secure following multiple checks sequence.
        if (sigfox_ep_api_ctx.irq_flags.field.rf_rx_data_received != 0) {
            // Set local flag and clear IRQ.
            local_rx_data_received_flag = SIGFOX_TRUE;
            sigfox_ep_api_ctx.irq_flags.field.rf_rx_data_received = 0;
        }
        if (sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt != 0) {
            // Set local flag and clear IRQ.
            local_mcu_timer1_cplt_flag = SIGFOX_TRUE;
            sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt = 0;
        }
        // De init radio if any frame has been received or timeout is reached.
        if ((local_rx_data_received_flag == SIGFOX_TRUE) || (local_mcu_timer1_cplt_flag == SIGFOX_TRUE)) {
#ifdef SIGFOX_EP_ERROR_CODES
            rf_api_status = RF_API_de_init();
            RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
            RF_API_de_init();
#endif
        }
        // Perform authentication if frame has been received.
        if (local_rx_data_received_flag == SIGFOX_TRUE) {
            // Decode downlink frame.
#ifdef SIGFOX_EP_ERROR_CODES
            status = _check_dl_frame();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _check_dl_frame();
#endif
        }
        if (((sigfox_ep_api_ctx.dl_status) == SIGFOX_TRUE) || (local_mcu_timer1_cplt_flag == SIGFOX_TRUE)) {
            // RX sequence done.
#ifdef SIGFOX_EP_ERROR_CODES
            status = _radio_sleep();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _radio_sleep();
#endif
            // Stop Trx timer.
#ifdef SIGFOX_EP_ERROR_CODES
            mcu_api_status = MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_RX);
            MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
            MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_RX);
#endif
            // Update state.
            if ((sigfox_ep_api_ctx.dl_status) == SIGFOX_TRUE) {
                // Update message status.
                sigfox_ep_api_ctx.message_status.field.dl_frame = 1;
#ifdef SIGFOX_EP_ASYNCHRONOUS
                _DOWNLINK_CPLT_CALLBACK();
#endif
#ifdef SIGFOX_EP_CERTIFICATION
                // Check DL-CONF bypass flag.
                if (sigfox_ep_api_ctx.test_parameters.flags.field.dl_conf_enable != 0) {
#endif
                // Configure DL confirmation message.
                sigfox_ep_api_ctx.internal_flags.field.control_message = 1;
                sigfox_ep_api_ctx.internal_flags.field.dl_conf_message = 1;
                _message_prepare();
#ifndef SIGFOX_EP_SINGLE_FRAME
                (sigfox_ep_api_ctx.common_parameters_ptr)->number_of_frames = 1;
#endif
                // Start DL confirmation timer.
#ifdef SIGFOX_EP_ERROR_CODES
                status = _start_timer(sigfox_ep_api_ctx.interframe_ms, MCU_API_TIMER_INSTANCE_T_CONF, MCU_API_TIMER_REASON_T_CONF);
                SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
                _start_timer(sigfox_ep_api_ctx.interframe_ms, MCU_API_TIMER_INSTANCE_T_CONF, MCU_API_TIMER_REASON_T_CONF);
#endif
                sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_DL_CONFIRMATION_TIMER;
#ifdef SIGFOX_EP_CERTIFICATION
                }
                else {
                    // Do not send DL-CONF.
#ifdef SIGFOX_EP_ASYNCHRONOUS
                    _MESSAGE_CPLT_CALLBACK();
#endif
                    // Force state to ready.
                    sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_READY;
                }
#endif
            }
            else {
                // Downlink timeout: set error flag.
                sigfox_ep_api_ctx.message_status.field.network_error = 1;
#ifdef SIGFOX_EP_ERROR_STACK
                // Add error to stack.
                SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_SIGFOX_EP_API, SIGFOX_EP_API_ERROR_DOWNLINK_TIMEOUT);
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
                _MESSAGE_CPLT_CALLBACK();
#endif
                // Force state to ready.
                sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_READY;
            }
        }
        // Frame not authenticated and timeout not reached yet: restart reception.
        if ((local_rx_data_received_flag == SIGFOX_TRUE) && (sigfox_ep_api_ctx.dl_status == SIGFOX_FALSE) && (local_mcu_timer1_cplt_flag == SIGFOX_FALSE)) {
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_reception();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_reception();
#endif
        }
        break;
    case SIGFOX_EP_API_STATE_DL_CONFIRMATION_TIMER:
#ifndef SIGFOX_EP_ASYNCHRONOUS
        // Wait for timer completion.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_timer_wait_cplt(MCU_API_TIMER_INSTANCE_T_CONF);
        MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
        MCU_API_timer_wait_cplt(MCU_API_TIMER_INSTANCE_T_CONF);
#endif
        sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt = 1; // Set flag manually in blocking mode.
#endif
        // Check timer completion flag.
        if (sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt != 0) {
            // Stop timer.
#ifdef SIGFOX_EP_ERROR_CODES
            mcu_api_status = MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_CONF);
            MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
            MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_T_CONF);
#endif
            sigfox_ep_api_ctx.irq_flags.field.mcu_timer1_cplt = 0;
            // Start confirmation frame.
#ifdef SIGFOX_EP_ERROR_CODES
            status = _start_transmission();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
            _start_transmission();
#endif
        }
        break;
#endif
    default:
        // Unknown state.
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_STATE);
    }
    // Return here if no error occurred.
    SIGFOX_RETURN();
errors:
    // Reset IRQ flags.
    sigfox_ep_api_ctx.irq_flags.all = 0;
    // Reset internal flags.
    sigfox_ep_api_ctx.internal_flags.field.radio_woken_up = 0;
    sigfox_ep_api_ctx.internal_flags.field.tx_control_pre_check_running = 0;
    sigfox_ep_api_ctx.internal_flags.field.tx_control_post_check_running = 0;
    // Set error flag.
    sigfox_ep_api_ctx.message_status.field.execution_error = 1;
    // Write NVM (without checking status in order to keep current error).
    _write_nvm();
#ifdef SIGFOX_EP_ERROR_CODES
    // Notify error to low level drivers.
    MCU_API_error();
    RF_API_error();
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Call completion callback (except on first process call).
    if (sigfox_ep_api_ctx.internal_flags.field.send_message_request == 0) {
        _MESSAGE_CPLT_CALLBACK();
    }
#endif
    // Force state to ready after error.
    sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_READY;
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
/*******************************************************************/
static SIGFOX_EP_API_status_t _send_application_message(SIGFOX_EP_API_application_message_t *application_message) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
    // Check library state.
    _CHECK_LIBRARY_STATE(!= SIGFOX_EP_API_STATE_READY);
    // Reset message status.
    sigfox_ep_api_ctx.message_status.all = 0;
    // Reset IRQ flags.
    sigfox_ep_api_ctx.irq_flags.all = 0;
#if (defined SIGFOX_EP_ASYNCHRONOUS) && (defined SIGFOX_EP_ERROR_CODES)
    // Reset low level status.
    sigfox_ep_api_ctx.mcu_api_status_from_callback = MCU_API_SUCCESS;
    sigfox_ep_api_ctx.rf_api_status_from_callback = RF_API_SUCCESS;
#endif
    // Store application message parameters locally.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _store_application_message(application_message);
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _store_application_message(application_message);
#endif
    // Set internal flags and prepare message.
    sigfox_ep_api_ctx.internal_flags.field.send_message_request = 1;
    sigfox_ep_api_ctx.internal_flags.field.control_message = 0;
    sigfox_ep_api_ctx.internal_flags.field.dl_conf_message = 0;
    _message_prepare();
    // Trigger TX.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _internal_process();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _internal_process();
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
    if (sigfox_ep_api_ctx.internal_flags.field.synchronous != 0) {
#endif
    // Block until library goes back to READY state.
    while (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY) {
#ifdef SIGFOX_EP_ERROR_CODES
            status = _internal_process();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _internal_process();
#endif
    }
#ifdef SIGFOX_EP_ASYNCHRONOUS
    }
#endif
errors:
    sigfox_ep_api_ctx.internal_flags.field.send_message_request = 0;
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
/*******************************************************************/
static SIGFOX_EP_API_status_t _send_control_message(SIGFOX_EP_API_control_message_t *control_message) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
    // Check library state.
    _CHECK_LIBRARY_STATE(!= SIGFOX_EP_API_STATE_READY);
    // Reset message status.
    sigfox_ep_api_ctx.message_status.all = 0;
    // Reset IRQ flags.
    sigfox_ep_api_ctx.irq_flags.all = 0;
#if (defined SIGFOX_EP_ASYNCHRONOUS) && (defined SIGFOX_EP_ERROR_CODES)
    // Reset low level status.
    sigfox_ep_api_ctx.mcu_api_status_from_callback = MCU_API_SUCCESS;
    sigfox_ep_api_ctx.rf_api_status_from_callback = RF_API_SUCCESS;
#endif
    // Store control message parameters locally.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _store_control_message(control_message);
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _store_control_message(control_message);
#endif
    // Reset context.
    /// Set internal flags and prepare message.
    sigfox_ep_api_ctx.internal_flags.field.send_message_request = 1;
    sigfox_ep_api_ctx.internal_flags.field.control_message = 1;
    sigfox_ep_api_ctx.internal_flags.field.dl_conf_message = 0;
    _message_prepare();
    // Trigger TX.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _internal_process();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
    _internal_process();
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
    if (sigfox_ep_api_ctx.internal_flags.field.synchronous != 0) {
#endif
    // Block until library goes back to READY state.
    while (sigfox_ep_api_ctx.state != SIGFOX_EP_API_STATE_READY) {
#ifdef SIGFOX_EP_ERROR_CODES
            status = _internal_process();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _internal_process();
#endif
    }
#ifdef SIGFOX_EP_ASYNCHRONOUS
    }
#endif
errors:
    // Clear request.
    sigfox_ep_api_ctx.internal_flags.field.send_message_request = 0;
    SIGFOX_RETURN();
}
#endif

/*** SIGFOX EP API functions ***/

/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_open(SIGFOX_EP_API_config_t *config) {
    // Local variables.
    sfx_u8 idx = 0;
    sfx_u8 ep_id[SIGFOX_EP_ID_SIZE_BYTES];
    sfx_u8 nvm_data[SIGFOX_NVM_DATA_SIZE_BYTES];
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
    SIGFOX_EP_FREQUENCY_status_t sigfox_ep_frequency_status = SIGFOX_EP_FREQUENCY_SUCCESS;
#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_REGULATORY
    SIGFOX_TX_CONTROL_status_t sigfox_tx_control_status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#endif /* SIGFOX_EP_ERROR_CODES */
#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
    MCU_API_config_t mcu_config;
    RF_API_config_t rf_config;
#endif
#ifdef SIGFOX_EP_REGULATORY
    SIGFOX_TX_CONTROL_config_t tx_control_config;
#endif
#if (defined SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER) && (defined SIGFOX_EP_PARAMETERS_CHECK)
    sfx_bool message_counter_rollover_valid = SIGFOX_FALSE;
#endif
#ifdef SIGFOX_EP_ERROR_STACK
    // Init error stack.
    if (sigfox_ep_api_ctx.internal_flags.field.error_stack_initialized == 0) {
        SIGFOX_ERROR_init();
        sigfox_ep_api_ctx.internal_flags.field.error_stack_initialized = 1;
    }
#endif
    // Check state.
    _CHECK_LIBRARY_STATE(!= SIGFOX_EP_API_STATE_CLOSED);
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if (config == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Check (even if SIGFOX_EP_PARAMETERS_CHECK flag is disabled) and store RC.
    if ((config->rc) == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_RC);
    }
    sigfox_ep_api_ctx.rc_ptr = (config->rc);
#ifdef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check message counter macro value.
    for (idx = 0; idx < SIGFOX_MESSAGE_COUNTER_ROLLOVER_LAST; idx++) {
        if (SIGFOX_MESSAGE_COUNTER_ROLLOVER_LIST[idx] == SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER) {
            message_counter_rollover_valid = SIGFOX_TRUE;
            break;
        }
    }
    if (message_counter_rollover_valid == SIGFOX_FALSE) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_MESSAGE_COUNTER_ROLLOVER);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
#else /* SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER */
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check message counter rollover index.
    if ((config->message_counter_rollover) >= SIGFOX_MESSAGE_COUNTER_ROLLOVER_LAST) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_MESSAGE_COUNTER_ROLLOVER);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Store message counter rollover.
    sigfox_ep_api_ctx.message_counter_rollover = SIGFOX_MESSAGE_COUNTER_ROLLOVER_LIST[config->message_counter_rollover];
#endif  /* SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER */
    // Init MCU API.
#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
    mcu_config.rc = sigfox_ep_api_ctx.rc_ptr;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    mcu_config.process_cb = (MCU_API_process_cb_t) &_MCU_API_process_cb;
    mcu_config.error_cb = (MCU_API_error_cb_t) &_MCU_API_error_cb;
#endif
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_open(&mcu_config);
    MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
    MCU_API_open(&mcu_config);
#endif
#endif
    // Init RF API.
#if (defined SIGFOX_EP_ASYNCHRONOUS) || (defined SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE)
    rf_config.rc = sigfox_ep_api_ctx.rc_ptr;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    rf_config.process_cb = (RF_API_process_cb_t) &_RF_API_process_cb;
    rf_config.error_cb = &_RF_API_error_cb;
#endif
#ifdef SIGFOX_EP_ERROR_CODES
    rf_api_status = RF_API_open(&rf_config);
    RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
    RF_API_open(&rf_config);
#endif
#endif
#ifdef SIGFOX_EP_REGULATORY
    // Init TX CONTROL driver.
    tx_control_config.rc = sigfox_ep_api_ctx.rc_ptr;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    tx_control_config.process_cb = &_SIGFOX_TX_CONTROL_process_cb;
#endif
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_tx_control_status = SIGFOX_TX_CONTROL_open(&tx_control_config);
    SIGFOX_TX_CONTROL_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_TX_CONTROL);
#else
    SIGFOX_TX_CONTROL_open(&tx_control_config);
#endif
#endif /* SIGFOX_EP_REGULATORY */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Store process callback (even if NULL).
    sigfox_ep_api_ctx.process_cb = (config->process_cb);
    // Update library behavior.
    sigfox_ep_api_ctx.internal_flags.field.synchronous = (sigfox_ep_api_ctx.process_cb == SIGFOX_NULL) ? 1 : 0;
#endif
    // Read last message counter and last random value value in NVM.
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_get_nvm(nvm_data, SIGFOX_NVM_DATA_SIZE_BYTES);
    MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
    MCU_API_get_nvm(nvm_data, SIGFOX_NVM_DATA_SIZE_BYTES);
#endif
    sigfox_ep_api_ctx.message_counter = 0;
    sigfox_ep_api_ctx.message_counter = (sfx_u16) (sigfox_ep_api_ctx.message_counter | ((((sfx_u16) nvm_data[SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_MSB]) << 8) & 0xFF00));
    sigfox_ep_api_ctx.message_counter = (sfx_u16) (sigfox_ep_api_ctx.message_counter | ((((sfx_u16) nvm_data[SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_LSB]) << 0) & 0x00FF));
    sigfox_ep_api_ctx.random_value = 0;
    sigfox_ep_api_ctx.random_value = (sfx_u16) (sigfox_ep_api_ctx.random_value | ((((sfx_u16) nvm_data[SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_MSB]) << 8) & 0xFF00));
    sigfox_ep_api_ctx.random_value = (sfx_u16) (sigfox_ep_api_ctx.random_value | ((((sfx_u16) nvm_data[SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_LSB]) << 0) & 0x00FF));
    // Read device ID.
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_get_ep_id(ep_id, SIGFOX_EP_ID_SIZE_BYTES);
    MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
    MCU_API_get_ep_id(ep_id, SIGFOX_EP_ID_SIZE_BYTES);
#endif
    for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
        sigfox_ep_api_ctx.ep_id[idx] = ep_id[idx];
    }
    // Init frequency driver.
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_ep_frequency_status = SIGFOX_EP_FREQUENCY_init(sigfox_ep_api_ctx.rc_ptr, ep_id, sigfox_ep_api_ctx.random_value);
    SIGFOX_EP_FREQUENCY_check_status(SIGFOX_EP_API_ERROR_DRIVER_SIGFOX_EP_FREQUENCY);
#else
    SIGFOX_EP_FREQUENCY_init(sigfox_ep_api_ctx.rc_ptr, ep_id, sigfox_ep_api_ctx.random_value);
#endif
    // Update library state if no error occurred.
    sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_READY;
errors:
    SIGFOX_RETURN();
}

/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_close(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
#endif
    // Check library state.
    _CHECK_LIBRARY_STATE(!= SIGFOX_EP_API_STATE_READY);
#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
    // Close MCU.
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_close();
    MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
    MCU_API_close();
#endif
    // Close RF.
#ifdef SIGFOX_EP_ERROR_CODES
    rf_api_status = RF_API_close();
    RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
    RF_API_close();
#endif
#endif
    // Update library state if no error occurred.
    sigfox_ep_api_ctx.state = SIGFOX_EP_API_STATE_CLOSED;
errors:
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_process(void) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
    // Set process flag.
    sigfox_ep_api_ctx.internal_flags.field.process_running = 1;
    // Run the internal process.
    while (sigfox_ep_api_ctx.irq_flags.all != 0) {
#ifdef SIGFOX_EP_ERROR_CODES
        status = _internal_process();
        SIGFOX_CHECK_STATUS(SIGFOX_EP_API_SUCCESS);
#else
        _internal_process();
#endif
    }
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    // Reset process flag.
    sigfox_ep_api_ctx.internal_flags.field.process_running = 0;
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_send_application_message(SIGFOX_EP_API_application_message_t *application_message) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_CERTIFICATION
    // Disable all test parameters.
    sigfox_ep_api_ctx.test_parameters.tx_frequency_hz = 0;
    sigfox_ep_api_ctx.test_parameters.flags.all = 0xFF;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sigfox_ep_api_ctx.test_parameters.rx_frequency_hz = 0;
    sigfox_ep_api_ctx.test_parameters.dl_t_w_ms = 0;
    sigfox_ep_api_ctx.test_parameters.dl_t_rx_ms = 0;
#endif
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
    sigfox_ep_api_ctx.test_parameters.lbt_cs_max_duration_first_frame_ms = 0;
#endif
#endif
    // Send message.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _send_application_message(application_message);
#else
    _send_application_message(application_message);
#endif
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_CERTIFICATION) && (defined SIGFOX_EP_APPLICATION_MESSAGES)
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_TEST_send_application_message(SIGFOX_EP_API_application_message_t *application_message, SIGFOX_EP_API_TEST_parameters_t *test_parameters) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
    if (test_parameters == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Update local test parameters.
    sigfox_ep_api_ctx.test_parameters.tx_frequency_hz = (test_parameters->tx_frequency_hz);
    sigfox_ep_api_ctx.test_parameters.flags = (test_parameters->flags);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sigfox_ep_api_ctx.test_parameters.rx_frequency_hz = (test_parameters->rx_frequency_hz);
    sigfox_ep_api_ctx.test_parameters.dl_t_w_ms = (test_parameters->dl_t_w_ms);
    sigfox_ep_api_ctx.test_parameters.dl_t_rx_ms = (test_parameters->dl_t_rx_ms);
#endif
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
    sigfox_ep_api_ctx.test_parameters.lbt_cs_max_duration_first_frame_ms = (test_parameters->lbt_cs_max_duration_first_frame_ms);
#endif
    // Send message.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _send_application_message(application_message);
#else
    _send_application_message(application_message);
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_send_control_message(SIGFOX_EP_API_control_message_t *control_message) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_CERTIFICATION
    // Disable all test parameters.
    sigfox_ep_api_ctx.test_parameters.tx_frequency_hz = 0;
    sigfox_ep_api_ctx.test_parameters.flags.all = 0xFF;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sigfox_ep_api_ctx.test_parameters.rx_frequency_hz = 0;
    sigfox_ep_api_ctx.test_parameters.dl_t_w_ms = 0;
    sigfox_ep_api_ctx.test_parameters.dl_t_rx_ms = 0;
#endif
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
    sigfox_ep_api_ctx.test_parameters.lbt_cs_max_duration_first_frame_ms = 0;
#endif
#endif
    // Send message.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _send_control_message(control_message);
#else
    _send_control_message(control_message);
#endif
    SIGFOX_RETURN();
}
#endif

#if (defined SIGFOX_EP_CERTIFICATION) && (defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE)
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_TEST_send_control_message(SIGFOX_EP_API_control_message_t *control_message, SIGFOX_EP_API_TEST_parameters_t *test_parameters) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
    if (test_parameters == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Update local test parameters.
    sigfox_ep_api_ctx.test_parameters.tx_frequency_hz = (test_parameters->tx_frequency_hz);
    sigfox_ep_api_ctx.test_parameters.flags = (test_parameters->flags);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sigfox_ep_api_ctx.test_parameters.rx_frequency_hz = (test_parameters->rx_frequency_hz);
    sigfox_ep_api_ctx.test_parameters.dl_t_w_ms = (test_parameters->dl_t_w_ms);
    sigfox_ep_api_ctx.test_parameters.dl_t_rx_ms = (test_parameters->dl_t_rx_ms);
#endif
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT)
    sigfox_ep_api_ctx.test_parameters.lbt_cs_max_duration_first_frame_ms = (test_parameters->lbt_cs_max_duration_first_frame_ms);
#endif
    // Send message.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _send_control_message(control_message);
#else
    _send_control_message(control_message);
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 *dl_rssi_dbm) {
    // Local variables.
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
    // Check library is opened.
    _CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if ((dl_payload == SIGFOX_NULL) || (dl_rssi_dbm == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
    if (dl_payload_size > SIGFOX_DL_PAYLOAD_SIZE_BYTES) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_DL_PAYLOAD_SIZE);
    }
#endif
    // Check downlink status.
    if (sigfox_ep_api_ctx.message_status.field.dl_frame == 0) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_DL_PAYLOAD_UNAVAILABLE);
    }
    // Copy local bytes into given buffer.
    for (idx = 0; idx < dl_payload_size; idx++) {
        dl_payload[idx] = sigfox_ep_api_ctx.dl_payload[idx];
    }
    // Copy local RSSI.
    (*dl_rssi_dbm) = sigfox_ep_api_ctx.dl_rssi_dbm;
errors:
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
SIGFOX_EP_API_message_status_t SIGFOX_EP_API_get_message_status(void) {
    // Return current message status.
    return sigfox_ep_api_ctx.message_status;
}

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
SIGFOX_EP_API_state_t SIGFOX_EP_API_get_state(void) {
    // Return current library state.
    return sigfox_ep_api_ctx.state;
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_ep_id(sfx_u8 *ep_id, sfx_u8 ep_id_size_bytes) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if (ep_id == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
    if (ep_id_size_bytes > SIGFOX_EP_ID_SIZE_BYTES) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_EP_ID);
    }
#endif
    // Check library is opened.
    _CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
    // Read ID directly from MCU driver.
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_get_ep_id(ep_id, ep_id_size_bytes);
    MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
    MCU_API_get_ep_id(ep_id, ep_id_size_bytes);
#endif
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if (initial_pac == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
    if (initial_pac_size_bytes > SIGFOX_EP_PAC_SIZE_BYTES) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_EP_PAC);
    }
#endif
    // Check library is opened.
    _CHECK_LIBRARY_STATE(== SIGFOX_EP_API_STATE_CLOSED);
    // Read PAC directly from MCU driver.
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_get_initial_pac(initial_pac, initial_pac_size_bytes);
    MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
    MCU_API_get_initial_pac(initial_pac, initial_pac_size_bytes);
#endif
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_version(SIGFOX_version_t version_type, sfx_u8 **version, sfx_u8 *version_size_char) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if ((version == SIGFOX_NULL) || (version_size_char == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
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
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_get_version(version, version_size_char);
        MCU_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_MCU_API);
#else
        MCU_API_get_version(version, version_size_char);
#endif
        break;
    case SIGFOX_VERSION_RF_DRIVER:
#ifdef SIGFOX_EP_ERROR_CODES
        rf_api_status = RF_API_get_version(version, version_size_char);
        RF_API_check_status(SIGFOX_EP_API_ERROR_DRIVER_RF_API);
#else
        RF_API_get_version(version, version_size_char);
#endif
        break;
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_VERSION);
        break;
    }
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_VERBOSE
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_get_flags(sfx_u8 **ep_flags, sfx_u8 *ep_flags_size_char) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if ((ep_flags == SIGFOX_NULL) || (ep_flags_size_char == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
#endif
    (*ep_flags) = (sfx_u8*) SIGFOX_EP_API_FLAGS;
    (*ep_flags_size_char) = (sfx_u8) sizeof(SIGFOX_EP_API_FLAGS);
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_ERROR_STACK
/*******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_unstack_error(SIGFOX_ERROR_t *error_ptr) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_API_status_t status = SIGFOX_EP_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if (error_ptr == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_NULL_PARAMETER);
    }
#endif
    // Check error stack has been initialized.
    if (sigfox_ep_api_ctx.internal_flags.field.error_stack_initialized == 0) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_API_ERROR_STACK_NOT_INITIALIZED);
    }
    // Directly call error stack driver.
    SIGFOX_ERROR_unstack(error_ptr);
errors:
    SIGFOX_RETURN();
}
#endif
