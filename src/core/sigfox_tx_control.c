/*!*****************************************************************
 * \file    sigfox_tx_control.c
 * \brief   Sigfox TX control driver for regulatory operations.
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

#include "core/sigfox_tx_control.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "manuf/mcu_api.h"
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT
#include "manuf/rf_api.h"
#endif
#include "sigfox_error.h"

#ifdef SIGFOX_EP_REGULATORY

/*** SIGFOX TX CONTROL local macros ***/

#ifdef SIGFOX_EP_SPECTRUM_ACCESS_FH
#define SIGFOX_TX_CONTROL_FH_TIMER_MS               20000
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LDC
#define SIGFOX_TX_CONTROL_LDC_INTERFRAME_MIN_MS     50
#define SIGFOX_TX_CONTROL_LDC_TX_DURATION_MAX_MS    4000
#endif
#ifdef SIGFOX_EP_UL_BIT_RATE_BPS
#define SIGFOX_TX_CONTROL_FRAME_DURATION_MS         ((((sfx_u32) sigfox_tx_control_ctx.params.bitstream_size_bytes) * 8 * 1000) / (SIGFOX_EP_UL_BIT_RATE_BPS))
#else
#define SIGFOX_TX_CONTROL_FRAME_DURATION_MS         ((((sfx_u32) sigfox_tx_control_ctx.params.bitstream_size_bytes) * 8 * 1000) / ((sfx_u32) sigfox_tx_control_ctx.params.ul_bit_rate_bps))
#endif

/*** SIGFOX TX CONTROL local structures ***/

/*******************************************************************/
typedef union {
    struct {
        sfx_u8 pre_check_running :1;
        sfx_u8 post_check_running :1;
        sfx_u8 mcu_timer_cplt :1;
        sfx_u8 rf_channel_free :1;
    } field;
    sfx_u8 all;
} SIGFOX_TX_CONTROL_flags_t;

/*******************************************************************/
typedef struct {
    const SIGFOX_rc_t *rc;
    volatile SIGFOX_TX_CONTROL_flags_t flags;
    volatile SIGFOX_TX_CONTROL_result_t result;
    SIGFOX_TX_CONTROL_parameters_t params;
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Process callback for asynchronous mode.
    SIGFOX_TX_CONTROL_process_cb_t process_cb;
#endif
} SIGFOX_TX_CONTROL_context_t;

/*** SIGFOX TX CONTROL local global variables ***/

static SIGFOX_TX_CONTROL_context_t sigfox_tx_control_ctx = { .flags.all = 0, };

/*** SIGFOX TX CONTROL local functions ***/

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
#define _PROCESS_CALLBACK(void) { \
    if (sigfox_tx_control_ctx.process_cb != SIGFOX_NULL) { \
        sigfox_tx_control_ctx.process_cb(); \
    } \
}
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
#define _COMPLETION_CALLBACK(void) { \
    if (sigfox_tx_control_ctx.params.cplt_cb != SIGFOX_NULL) { \
        sigfox_tx_control_ctx.params.cplt_cb(); \
    } \
}
#endif

#if ((defined SIGFOX_EP_ASYNCHRONOUS) && ((defined SIGFOX_EP_SPECTRUM_ACCESS_LBT) || (defined SIGFOX_EP_SPECTRUM_ACCESS_FH)))
/*******************************************************************/
static void _MCU_API_timer_cplt_cb(void) {
    // Set local flag.
    sigfox_tx_control_ctx.flags.field.mcu_timer_cplt = 1;
    _PROCESS_CALLBACK();
}
#endif

#if ((defined SIGFOX_EP_ASYNCHRONOUS) && (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT))
/*******************************************************************/
static void _RF_API_channel_free_cb(void) {
    // Set local flag.
    sigfox_tx_control_ctx.flags.field.rf_channel_free = 1;
    _PROCESS_CALLBACK();
}
#endif

/*******************************************************************/
static SIGFOX_TX_CONTROL_status_t _store_parameters(SIGFOX_TX_CONTROL_parameters_t *params) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_TX_CONTROL_status_t status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
    if (params == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_TX_CONTROL_ERROR_NULL_PARAMETER);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    sigfox_tx_control_ctx.params.type = (params->type);
    sigfox_tx_control_ctx.params.bitstream_size_bytes = (params->bitstream_size_bytes);
    sigfox_tx_control_ctx.params.last_message_frame = (params->last_message_frame);
#ifndef SIGFOX_EP_SINGLE_FRAME
    sigfox_tx_control_ctx.params.ul_frame_rank = (params->ul_frame_rank);
    sigfox_tx_control_ctx.params.number_of_frames = (params->number_of_frames);
#endif
#ifndef UL_BTT_RATE
    sigfox_tx_control_ctx.params.ul_bit_rate_bps = (params->ul_bit_rate_bps);
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sigfox_tx_control_ctx.params.dl_conf_message = (params->dl_conf_message);
#endif
#ifdef SIGFOX_EP_INTERFRAME_TIMER_REQUIRED
    sigfox_tx_control_ctx.params.interframe_ms = (params->interframe_ms);
#endif
#ifdef SIGFOX_EP_CERTIFICATION
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_FH
    sigfox_tx_control_ctx.params.fh_check_enable = (params->fh_check_enable);
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT
    sigfox_tx_control_ctx.params.lbt_check_enable = (params->lbt_check_enable);
    sigfox_tx_control_ctx.params.lbt_cs_max_duration_first_frame_ms = (params->lbt_cs_max_duration_first_frame_ms);
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LDC
    sigfox_tx_control_ctx.params.ldc_check_enable = (params->ldc_check_enable);
#endif
#endif /* SIGFOX_EP_CERTIFICATION */
#ifdef SIGFOX_EP_ASYNCHRONOUS
    sigfox_tx_control_ctx.params.cplt_cb = (params->cplt_cb);
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}

/*** SIGFOX TX CONTROL functions ***/

/*******************************************************************/
SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_open(SIGFOX_TX_CONTROL_config_t *config) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_TX_CONTROL_status_t status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
    if (config == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_TX_CONTROL_ERROR_NULL_PARAMETER);
    }
    if ((config->rc) == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_TX_CONTROL_ERROR_NULL_PARAMETER);
    }
#ifdef SIGFOX_EP_ASYNCHRONOUS
    if ((config->process_cb) == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_TX_CONTROL_ERROR_NULL_PARAMETER);
    }
#endif
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Reset flags.
    sigfox_tx_control_ctx.flags.all = 0;
    // Store RC pointer.
    sigfox_tx_control_ctx.rc = (config->rc);
#ifdef SIGFOX_EP_ASYNCHRONOUS
    sigfox_tx_control_ctx.process_cb = (config->process_cb);
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_check(SIGFOX_TX_CONTROL_parameters_t *params) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_TX_CONTROL_status_t status = SIGFOX_TX_CONTROL_SUCCESS;
#if ((defined SIGFOX_EP_SPECTRUM_ACCESS_FH) || (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT))
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
#endif /* SIGFOX_EP_ERROR_CODES */
#if ((defined SIGFOX_EP_SPECTRUM_ACCESS_FH) || (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT))
    MCU_API_timer_t mcu_timer;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT
    RF_API_radio_parameters_t radio_params;
    RF_API_carrier_sense_parameters_t cs_params;
    sfx_u32 cs_max_duration_ms = 0;
#ifndef SIGFOX_EP_ASYNCHRONOUS
    sfx_bool channel_free = SIGFOX_FALSE;
#endif
#endif
#if (((defined SIGFOX_EP_SPECTRUM_ACCESS_LBT) || ((defined SIGFOX_EP_SPECTRUM_ACCESS_LDC) && (defined SIGFOX_EP_INTERFRAME_TIMER_REQUIRED))) && !(defined SIGFOX_EP_SINGLE_FRAME))
    sfx_u8 number_of_repeated_frames = 0;
#endif
#if ((defined SIGFOX_EP_SPECTRUM_ACCESS_LDC) && !(defined SIGFOX_EP_SINGLE_FRAME) && (defined SIGFOX_EP_INTERFRAME_TIMER_REQUIRED))
    sfx_u32 transmission_time_ms = 0;
#endif
    // Store and check parameters.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _store_parameters(params);
    SIGFOX_CHECK_STATUS(SIGFOX_TX_CONTROL_SUCCESS);
#else
    _store_parameters(params);
#endif
#if (((defined SIGFOX_EP_SPECTRUM_ACCESS_LBT) || ((defined SIGFOX_EP_SPECTRUM_ACCESS_LDC) && (defined SIGFOX_EP_INTERFRAME_TIMER_REQUIRED))) && !(defined SIGFOX_EP_SINGLE_FRAME))
    number_of_repeated_frames = (sfx_u8) (sigfox_tx_control_ctx.params.number_of_frames - 1);
#endif
    // Reset flags.
    sigfox_tx_control_ctx.flags.all = 0;
    // Reset result.
    sigfox_tx_control_ctx.result = SIGFOX_TX_CONTROL_RESULT_ALLOWED;
    // Perform required check.
    switch (((sigfox_tx_control_ctx.rc)->spectrum_access)->type) {
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_DC
    case SIGFOX_SPECTRUM_ACCESS_TYPE_DC:
        break;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LDC
    case SIGFOX_SPECTRUM_ACCESS_TYPE_LDC:
#ifdef SIGFOX_EP_CERTIFICATION
        // Bypass check if required.
        if ((params->ldc_check_enable) == SIGFOX_FALSE) break;
#endif
#if ((defined SIGFOX_EP_INTERFRAME_TIMER_REQUIRED) && !(defined SIGFOX_EP_SINGLE_FRAME))
        // Execute LDC check in case of pre-check.
        if ((sigfox_tx_control_ctx.params.type) == SIGFOX_TX_CONTROL_TYPE_PRE_CHECK) {
            // Check interframe duration.
            if (sigfox_tx_control_ctx.params.interframe_ms < SIGFOX_TX_CONTROL_LDC_INTERFRAME_MIN_MS) {
                // Compute transmission time.
                transmission_time_ms = ((sfx_u32) sigfox_tx_control_ctx.params.number_of_frames) * SIGFOX_TX_CONTROL_FRAME_DURATION_MS;
                transmission_time_ms += ((sfx_u32) number_of_repeated_frames) * sigfox_tx_control_ctx.params.interframe_ms;
                // Check transmission time.
                if (transmission_time_ms > SIGFOX_TX_CONTROL_LDC_TX_DURATION_MAX_MS) {
                    // Regulation constraint not met.
                    sigfox_tx_control_ctx.result = SIGFOX_TX_CONTROL_RESULT_FORBIDDEN;
                }
            }
        }
#endif
        break;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_FH
    case SIGFOX_SPECTRUM_ACCESS_TYPE_FH:
#ifdef SIGFOX_EP_CERTIFICATION
        // Bypass check if required.
        if ((params->fh_check_enable) == SIGFOX_FALSE) break;
#endif
        // Start FH timer in case of post-check.
        if ((sigfox_tx_control_ctx.params.type) == SIGFOX_TX_CONTROL_TYPE_POST_CHECK) {
            // Only start timer on the last frame of the message sequence.
            if (sigfox_tx_control_ctx.params.last_message_frame == SIGFOX_TRUE) {
                mcu_timer.instance = MCU_API_TIMER_INSTANCE_FH;
                mcu_timer.reason = MCU_API_TIMER_REASON_FH;
                mcu_timer.duration_ms = SIGFOX_TX_CONTROL_FH_TIMER_MS;
#ifdef SIGFOX_EP_ASYNCHRONOUS
                mcu_timer.cplt_cb = (MCU_API_timer_cplt_cb_t) &_MCU_API_timer_cplt_cb;
#endif
#ifdef SIGFOX_EP_ERROR_CODES
                mcu_api_status = MCU_API_timer_start(&mcu_timer);
                MCU_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_MCU_API);
#else
                MCU_API_timer_start(&mcu_timer);
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
                sigfox_tx_control_ctx.result = SIGFOX_TX_CONTROL_RESULT_PENDING;
#else
                // Wait FH timer completion.
#ifdef SIGFOX_EP_ERROR_CODES
                mcu_api_status = MCU_API_timer_wait_cplt(MCU_API_TIMER_INSTANCE_FH);
                MCU_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_MCU_API);
#else
                MCU_API_timer_wait_cplt(MCU_API_TIMER_INSTANCE_FH);
#endif
                // Stop timer.
#ifdef SIGFOX_EP_ERROR_CODES
                mcu_api_status = MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_FH);
                MCU_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_MCU_API);
#else
                MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_FH);
#endif
                sigfox_tx_control_ctx.result = SIGFOX_TX_CONTROL_RESULT_ALLOWED;
#endif
            }
        }
        break;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT
    case SIGFOX_SPECTRUM_ACCESS_TYPE_LBT:
#ifdef SIGFOX_EP_CERTIFICATION
        // Bypass check if required.
        if ((params->lbt_check_enable) == SIGFOX_FALSE) break;
#endif
        // Execute LBT in case of pre-check.
        if ((sigfox_tx_control_ctx.params.type) == SIGFOX_TX_CONTROL_TYPE_PRE_CHECK) {
            // Compute carrier sense maximum duration.
#ifdef SIGFOX_EP_SINGLE_FRAME
            // Use user-defined timeout for first frame.
            cs_max_duration_ms = (sigfox_tx_control_ctx.rc -> spectrum_access) -> cs_max_duration_first_frame_ms;
#else
            // Uplink user message.
            if (sigfox_tx_control_ctx.params.ul_frame_rank == SIGFOX_UL_FRAME_RANK_1) {
                // Use user-defined timeout for first frame.
                cs_max_duration_ms = (sigfox_tx_control_ctx.rc->spectrum_access)->cs_max_duration_first_frame_ms;
#ifdef SIGFOX_EP_CERTIFICATION
                // Override with test value if not null.
                if (sigfox_tx_control_ctx.params.lbt_cs_max_duration_first_frame_ms != 0) {
                    cs_max_duration_ms = sigfox_tx_control_ctx.params.lbt_cs_max_duration_first_frame_ms;
                }
#endif
            }
            else {
                // Use T_LF.
                cs_max_duration_ms = SIGFOX_T_LF_MS;
#ifdef SIGFOX_EP_INTERFRAME_TIMER_REQUIRED
                cs_max_duration_ms -= ((sfx_u32) number_of_repeated_frames) * (sigfox_tx_control_ctx.params.interframe_ms); // Remove inter-frame delay(s).
#endif
                cs_max_duration_ms -= ((sfx_u32) (number_of_repeated_frames - 1)) * SIGFOX_TX_CONTROL_FRAME_DURATION_MS; // Remove frame 2 duration.
                cs_max_duration_ms /= ((sfx_u32) number_of_repeated_frames); // Divide remaining time equitably for all repeated frames.
            }
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
            // Override in case of DL-ACK frame.
            if (sigfox_tx_control_ctx.params.dl_conf_message == SIGFOX_TRUE) {
                // DL confirmation message.
                cs_max_duration_ms = SIGFOX_T_CONF_MAX_MS - (sigfox_tx_control_ctx.params.interframe_ms); // Tconf.
            }
#endif
            // Set radio configuration.
            radio_params.rf_mode = RF_API_MODE_RX;
            radio_params.frequency_hz = ((sigfox_tx_control_ctx.rc)->f_ul_hz);
            radio_params.modulation = RF_API_MODULATION_NONE;
            radio_params.bit_rate_bps = 0;
#ifdef SIGFOX_EP_BIDIRECTIONAL
            radio_params.deviation_hz = 0;
#endif
            // Set carrier sense parameters.
            cs_params.bandwidth_hz = ((sigfox_tx_control_ctx.rc)->spectrum_access)->cs_bandwidth_hz;
            cs_params.threshold_dbm = ((sigfox_tx_control_ctx.rc)->spectrum_access)->cs_threshold_dbm;
            cs_params.min_duration_ms = ((sigfox_tx_control_ctx.rc)->spectrum_access)->cs_min_duration_ms;
#ifdef SIGFOX_EP_ASYNCHRONOUS
            cs_params.channel_free_cb = (RF_API_channel_free_cb_t) &_RF_API_channel_free_cb;
#else
            cs_params.channel_free = &channel_free;
#endif
            // Init radio for LBT check.
#ifdef SIGFOX_EP_ERROR_CODES
            rf_api_status = RF_API_init(&radio_params);
            RF_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_RF_API);
#else
            RF_API_init(&radio_params);
#endif
            // Start timer to manage carrier sense timeout.
            mcu_timer.instance = MCU_API_TIMER_INSTANCE_LBT;
            mcu_timer.reason = MCU_API_TIMER_REASON_LBT;
            mcu_timer.duration_ms = cs_max_duration_ms;
#ifdef SIGFOX_EP_ASYNCHRONOUS
            mcu_timer.cplt_cb = (MCU_API_timer_cplt_cb_t) &_MCU_API_timer_cplt_cb;
#endif
#ifdef SIGFOX_EP_ERROR_CODES
            mcu_api_status = MCU_API_timer_start(&mcu_timer);
            MCU_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_MCU_API);
#else
            MCU_API_timer_start(&mcu_timer);
#endif
            sigfox_tx_control_ctx.result = SIGFOX_TX_CONTROL_RESULT_PENDING;
            // Start carrier sense.
#ifdef SIGFOX_EP_ERROR_CODES
            rf_api_status = RF_API_carrier_sense(&cs_params);
            RF_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_RF_API);
#else
            RF_API_carrier_sense(&cs_params);
#endif
#ifdef SIGFOX_EP_ASYNCHRONOUS
            // Result will be known later after carrier sense.
            sigfox_tx_control_ctx.result = SIGFOX_TX_CONTROL_RESULT_PENDING;
#else /* SIGFOX_EP_ASYNCHRONOUS */
            // Stop radio.
#ifdef SIGFOX_EP_ERROR_CODES
            rf_api_status = RF_API_de_init();
            RF_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_RF_API);
#else
            RF_API_de_init();
#endif
            // Stop timer.
#ifdef SIGFOX_EP_ERROR_CODES
            mcu_api_status = MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_LBT);
            MCU_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_MCU_API);
#else
            MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_LBT);
#endif
            // Update result.
            sigfox_tx_control_ctx.result = (channel_free == SIGFOX_FALSE) ? SIGFOX_TX_CONTROL_RESULT_FORBIDDEN : SIGFOX_TX_CONTROL_RESULT_ALLOWED;
#endif /* SIGFOX_EP_ASYNCHRONOUS */
        }
        break;
#endif
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_TX_CONTROL_ERROR_SPECTRUM_ACCESS);
        break;
    }
#ifdef SIGFOX_EP_ASYNCHRONOUS
    // Directly call the completion callback if the result is already available.
    if (sigfox_tx_control_ctx.result != SIGFOX_TX_CONTROL_RESULT_PENDING) {
        // Call completion callback.
        _COMPLETION_CALLBACK();
    }
#endif
errors:
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_ASYNCHRONOUS
/*******************************************************************/
SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_process(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_TX_CONTROL_status_t status = SIGFOX_TX_CONTROL_SUCCESS;
#if ((defined SIGFOX_EP_SPECTRUM_ACCESS_FH) || (defined SIGFOX_EP_SPECTRUM_ACCESS_LBT))
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT
    RF_API_status_t rf_api_status = RF_API_SUCCESS;
#endif
#endif
    // Check flags and update result.
    switch (((sigfox_tx_control_ctx.rc)->spectrum_access)->type) {
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_DC
    case SIGFOX_SPECTRUM_ACCESS_TYPE_DC:
        // Nothing to do.
        break;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LDC
    case SIGFOX_SPECTRUM_ACCESS_TYPE_LDC:
        // Nothing to do.
        break;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_FH
    case SIGFOX_SPECTRUM_ACCESS_TYPE_FH:
        if ((sigfox_tx_control_ctx.params.type) == SIGFOX_TX_CONTROL_TYPE_POST_CHECK) {
            // Check FH timer completion.
            if (sigfox_tx_control_ctx.flags.field.mcu_timer_cplt != 0) {
                // Clear flag.
                sigfox_tx_control_ctx.flags.field.mcu_timer_cplt = 0;
                // Stop timer.
#ifdef SIGFOX_EP_ERROR_CODES
                mcu_api_status = MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_FH);
                MCU_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_MCU_API);
#else
                MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_FH);
#endif
                // Update result.
                sigfox_tx_control_ctx.result = SIGFOX_TX_CONTROL_RESULT_ALLOWED;
                // Call completion callback.
                _COMPLETION_CALLBACK();
            }
        }
        break;
#endif
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT
    case SIGFOX_SPECTRUM_ACCESS_TYPE_LBT:
        if ((sigfox_tx_control_ctx.params.type) == SIGFOX_TX_CONTROL_TYPE_PRE_CHECK) {
            // Check flags.
            if ((sigfox_tx_control_ctx.flags.field.rf_channel_free != 0) || (sigfox_tx_control_ctx.flags.field.mcu_timer_cplt != 0)) {
                // Update result according to flags.
                if (sigfox_tx_control_ctx.flags.field.mcu_timer_cplt != 0) {
                    // Clear flag and update result.
                    sigfox_tx_control_ctx.flags.field.mcu_timer_cplt = 0;
                    sigfox_tx_control_ctx.result = SIGFOX_TX_CONTROL_RESULT_FORBIDDEN;
                }
                if (sigfox_tx_control_ctx.flags.field.rf_channel_free != 0) {
                    // Clear flag and update result.
                    sigfox_tx_control_ctx.flags.field.rf_channel_free = 0;
                    sigfox_tx_control_ctx.result = SIGFOX_TX_CONTROL_RESULT_ALLOWED;
                }
                // Stop radio.
#ifdef SIGFOX_EP_ERROR_CODES
                rf_api_status = RF_API_de_init();
                RF_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_RF_API);
#else
                RF_API_de_init();
#endif
                // Stop timer.
#ifdef SIGFOX_EP_ERROR_CODES
                mcu_api_status = MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_LBT);
                MCU_API_check_status(SIGFOX_TX_CONTROL_ERROR_DRIVER_MCU_API);
#else
                MCU_API_timer_stop(MCU_API_TIMER_INSTANCE_LBT);
#endif
                // Call completion callback.
                _COMPLETION_CALLBACK();
            }
        }
        break;
#endif
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_TX_CONTROL_ERROR_SPECTRUM_ACCESS);
        break;
    }
errors:
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
sfx_bool SIGFOX_TX_CONTROL_is_radio_required(SIGFOX_TX_CONTROL_check_type check_type) {
    // Local variables.
    sfx_bool is_radio_required = SIGFOX_FALSE;
#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT
    // Radio is only required for LBT pre-check.
    if (((((sigfox_tx_control_ctx.rc)->spectrum_access)->type) == SIGFOX_SPECTRUM_ACCESS_TYPE_LBT) && (check_type == SIGFOX_TX_CONTROL_TYPE_PRE_CHECK)) {
        is_radio_required = SIGFOX_TRUE;
    }
#else
    SIGFOX_UNUSED(check_type);
#endif
    return is_radio_required;
}

/*******************************************************************/
SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_get_result(SIGFOX_TX_CONTROL_result_t *result) {
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_TX_CONTROL_status_t status = SIGFOX_TX_CONTROL_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if (result == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_TX_CONTROL_ERROR_NULL_PARAMETER);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Update result.
    (*result) = sigfox_tx_control_ctx.result;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}

#endif /* SIGFOX_EP_REGULATORY */
