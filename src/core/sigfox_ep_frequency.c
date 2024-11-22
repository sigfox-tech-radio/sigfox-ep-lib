/*!*****************************************************************
 * \file    sigfox_ep_frequency.c
 * \brief   Sigfox uplink and downlink frequency manager.
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

#include "core/sigfox_ep_frequency.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

/*** SIGFOX EP FREQUENCY local macros ***/

// Define the number of bits of the random value used to select uplink frequency.
// Note: for 192kHz the maximum value to avoid overflow is 14: 192000 * (2^14 - 1) is still on 32 bits.
#define SIGFOX_EP_FREQUENCY_RANDOM_VALUE_SIZE_BITS      14
#define SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MASK           ((1 << SIGFOX_EP_FREQUENCY_RANDOM_VALUE_SIZE_BITS) - 1)
#define SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MAX            ((sfx_u32) SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MASK)
#define SIGFOX_EP_FREQUENCY_RANDOM_MULTIPLIER           61
#define SIGFOX_EP_FREQUENCY_RANDOM_GENERATION_LIMIT     1000
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH)
#define SIGFOX_EP_FREQUENCY_FH_MICRO_CHANNEL_ERROR      0xFF
#endif

/*** SIGFOX EP FREQUENCY local structures ***/

/*******************************************************************/
typedef struct {
    const SIGFOX_rc_t *rc;
    sfx_u16 random_value;
    sfx_u16 random_offset;
#ifndef SIGFOX_EP_SINGLE_FRAME
    SIGFOX_EP_FREQUENCY_uplink_signal_t *input;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_u32 frame_1_frequency_hz;
#endif
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH) && !(defined SIGFOX_EP_SINGLE_FRAME)
    sfx_bool micro_channel_table_initialized;
    sfx_u8 micro_channel_frame_count[SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL];
#endif
} SIGFOX_EP_FREQUENCY_context_t;

/*** SIGFOX EP FREQUENCY local global variables ***/

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH) && !(defined SIGFOX_EP_SINGLE_FRAME)
static SIGFOX_EP_FREQUENCY_context_t sigfox_ep_frequency_ctx = { .micro_channel_table_initialized = SIGFOX_FALSE, };
#else
static SIGFOX_EP_FREQUENCY_context_t sigfox_ep_frequency_ctx;
#endif

/*** SIGFOX EP FREQUENCY local functions ***/

/*!******************************************************************
 * \fn void _COMPUTE_NEW_RANDOM_FREQUENCY()
 * \brief Generic macro to call the random frequency generator function.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
#ifdef SIGFOX_EP_ERROR_CODES
#define _COMPUTE_NEW_RANDOM_FREQUENCY() { status = _compute_new_random_frequency(ul_frequency_hz); SIGFOX_CHECK_STATUS(SIGFOX_EP_FREQUENCY_SUCCESS); }
#else
#define _COMPUTE_NEW_RANDOM_FREQUENCY() { _compute_new_random_frequency(ul_frequency_hz); }
#endif

/*******************************************************************/
static sfx_u32 _get_baseband_frequency_range_hz(void) {
    return (sfx_u32) (SIGFOX_MACRO_CHANNEL_WIDTH_HZ - (2 * ((sigfox_ep_frequency_ctx.rc)->epsilon_hz)));
}

/*******************************************************************/
static sfx_u32 _get_baseband_frequency_min_allowed_hz(void) {
    // Local variables.
    sfx_u32 baseband_frequency_min_hz = ((sigfox_ep_frequency_ctx.rc)->epsilon_hz);
#if (defined SIGFOX_EP_BIDIRECTIONAL) && !(defined SIGFOX_EP_SINGLE_FRAME)
    // Add downlink guard band depending on bidirectional flag and number of frames.
    if (((sigfox_ep_frequency_ctx.input)->bidirectional_flag) == SIGFOX_TRUE) {
        // Frame 3 has a negative offset.
        if (((sigfox_ep_frequency_ctx.input)->number_of_frames) > 2) {
            // Increase minimum baseband frequency for frame 3.
            baseband_frequency_min_hz += (((sigfox_ep_frequency_ctx.rc)->spectrum_access)->delta_f_mf_hz);
        }
    }
#endif
    return baseband_frequency_min_hz;
}

/*******************************************************************/
static sfx_u32 _get_baseband_frequency_max_allowed_hz(void) {
    // Local variables.
    sfx_u32 baseband_frequency_max_hz = SIGFOX_MACRO_CHANNEL_WIDTH_HZ - ((sfx_u32) ((sigfox_ep_frequency_ctx.rc)->epsilon_hz));
#if (defined SIGFOX_EP_BIDIRECTIONAL) && !(defined SIGFOX_EP_SINGLE_FRAME)
    // Add downlink guard band depending on bidirectional flag and number of frames.
    if (((sigfox_ep_frequency_ctx.input)->bidirectional_flag) == SIGFOX_TRUE) {
        // Frame 2 has a positive offset.
        if (((sigfox_ep_frequency_ctx.input)->number_of_frames) > 1) {
            // Reduce maximum baseband frequency for frame 2.
            baseband_frequency_max_hz -= (((sigfox_ep_frequency_ctx.rc)->spectrum_access)->delta_f_mf_hz);
        }
    }
#endif
    return baseband_frequency_max_hz;
}

/*******************************************************************/
static sfx_bool _is_baseband_frequency_allowed(sfx_u32 baseband_frequency_hz) {
    return ((baseband_frequency_hz >= _get_baseband_frequency_min_allowed_hz()) && (baseband_frequency_hz <= _get_baseband_frequency_max_allowed_hz()));
}

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH) && !(defined SIGFOX_EP_SINGLE_FRAME)
/*******************************************************************/
static sfx_bool _is_micro_channel_free(sfx_u8 micro_channel_index) {
    // Thanks to the FH timer, a micro-channel is free when all frames of the message have been sent.
    sfx_bool micro_channel_free = (sigfox_ep_frequency_ctx.micro_channel_frame_count[micro_channel_index] >= SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL_OPERATED) ? SIGFOX_TRUE : SIGFOX_FALSE;
    return micro_channel_free;
}
#endif

#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH)
/*******************************************************************/
static sfx_u8 _get_micro_channel_index(sfx_u32 baseband_frequency_hz) {
    // Local variables.
    sfx_u8 micro_channel_index = 0;
    // Compute index.
    micro_channel_index = (sfx_u8) ((baseband_frequency_hz + (SIGFOX_FH_MACRO_CHANNEL_DELTA / 2)) / SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ);
    micro_channel_index %= SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL;
    // Check micro-channels mask.
    if (((SIGFOX_FH_MICRO_CHANNEL_MASK >> micro_channel_index) & 0x01) == 0) {
        micro_channel_index = SIGFOX_EP_FREQUENCY_FH_MICRO_CHANNEL_ERROR;
        goto errors;
    }
    // Check low side micro-channel guard band.
    if ((baseband_frequency_hz - (SIGFOX_FH_MACRO_CHANNEL_GUARD_BAND_HZ + (SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ * ((sfx_u32) (micro_channel_index - 1))))) < SIGFOX_FH_MICRO_CHANNEL_GUARD_BAND_HZ) {
        micro_channel_index = SIGFOX_EP_FREQUENCY_FH_MICRO_CHANNEL_ERROR;
        goto errors;
    }
    // Check high side micro-channel guard band.
    if (((SIGFOX_FH_MACRO_CHANNEL_GUARD_BAND_HZ + (SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ * ((sfx_u32) micro_channel_index))) - baseband_frequency_hz) < SIGFOX_FH_MICRO_CHANNEL_GUARD_BAND_HZ) {
        micro_channel_index = SIGFOX_EP_FREQUENCY_FH_MICRO_CHANNEL_ERROR;
        goto errors;
    }
errors:
    return micro_channel_index;
}
#endif

/*******************************************************************/
static SIGFOX_EP_FREQUENCY_status_t _compute_new_random_frequency(sfx_u32 *frequency_hz) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
    sfx_u32 baseband_frequency_hz = 0;
    sfx_u32 random_value_temp = 0;
    sfx_u16 random_generation_count = 0;
    sfx_bool baseband_frequency_allowed = SIGFOX_FALSE;
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH)
    sfx_u8 micro_channel_index = 0;
#endif
    // Call the random algorithm until a valid frequency is computed.
    do {
        // Generate new random value.
        random_value_temp = (SIGFOX_EP_FREQUENCY_RANDOM_MULTIPLIER * (sfx_u32) sigfox_ep_frequency_ctx.random_value) + (sfx_u32) sigfox_ep_frequency_ctx.random_offset;
        sigfox_ep_frequency_ctx.random_value = (sfx_u16) (random_value_temp & SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MASK);
        random_generation_count++;
        // Convert to baseband frequency.
        baseband_frequency_hz = ((sigfox_ep_frequency_ctx.rc)->epsilon_hz);
        baseband_frequency_hz += (_get_baseband_frequency_range_hz() * ((sfx_u32) sigfox_ep_frequency_ctx.random_value)) / (SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MAX);
        // Check limits.
        baseband_frequency_allowed = _is_baseband_frequency_allowed(baseband_frequency_hz);
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH)
        // Additional check on micro-channel in case of FH.
        if ((((sigfox_ep_frequency_ctx.rc)->spectrum_access->type) == SIGFOX_SPECTRUM_ACCESS_TYPE_FH) && (baseband_frequency_allowed == SIGFOX_TRUE)) {
            // Get micro channel index.
            micro_channel_index = _get_micro_channel_index(baseband_frequency_hz);
            baseband_frequency_allowed = (micro_channel_index == SIGFOX_EP_FREQUENCY_FH_MICRO_CHANNEL_ERROR) ? SIGFOX_FALSE : SIGFOX_TRUE;
#ifndef SIGFOX_EP_SINGLE_FRAME
            // For multiple frames, additional check is required on the micro-channel index.
            if (baseband_frequency_allowed == SIGFOX_TRUE) {
                baseband_frequency_allowed = _is_micro_channel_free(micro_channel_index);
            }
#endif
        }
#endif /* SIGFOX_EP_REGULATORY and SIGFOX_EP_SPECTRUM_ACCESS_FH */
        // Check request count.
        if (random_generation_count > SIGFOX_EP_FREQUENCY_RANDOM_GENERATION_LIMIT) {
            SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_RANDOM_GENERATION);
        }
    }
    while (baseband_frequency_allowed == SIGFOX_FALSE);
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH) && !(defined SIGFOX_EP_SINGLE_FRAME)
    // Blacklist selected micro-channel.
    sigfox_ep_frequency_ctx.micro_channel_frame_count[micro_channel_index] = 0;
#endif
    // Convert to absolute frequency.
    (*frequency_hz) = ((sigfox_ep_frequency_ctx.rc)->f_ul_hz) - (SIGFOX_MACRO_CHANNEL_WIDTH_HZ / 2) + baseband_frequency_hz;
#ifdef SIGFOX_EP_BIDIRECTIONAL
    // Store frame 1 frequency for future computation if bidirectional.
#ifdef SIGFOX_EP_SINGLE_FRAME
    sigfox_ep_frequency_ctx.frame_1_frequency_hz = (*frequency_hz);
#else
    if (((sigfox_ep_frequency_ctx.input)->ul_frame_rank) == SIGFOX_UL_FRAME_RANK_1) {
        sigfox_ep_frequency_ctx.frame_1_frequency_hz = (*frequency_hz);
    }
#endif /* SIGFOX_EP_SINGLE_FRAME */
#endif /* SIGFOX_EP_BIDIRECTIONAL */
errors:
    SIGFOX_RETURN();
}

#if (defined SIGFOX_EP_BIDIRECTIONAL) && (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
/*******************************************************************/
SIGFOX_EP_FREQUENCY_status_t _check_frame_1_frequency(void) {
    // Local variables.
    SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
    // Check frequency range.
    if (sigfox_ep_frequency_ctx.frame_1_frequency_hz < (((sigfox_ep_frequency_ctx.rc)->f_ul_hz) - (SIGFOX_MACRO_CHANNEL_WIDTH_HZ / 2))) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_FRAME_1_FREQUENCY);
    }
    if (sigfox_ep_frequency_ctx.frame_1_frequency_hz > (((sigfox_ep_frequency_ctx.rc)->f_ul_hz) + (SIGFOX_MACRO_CHANNEL_WIDTH_HZ / 2))) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_FRAME_1_FREQUENCY);
    }
errors:
    return status;
}
#endif

/*** SIGFOX EP FREQUENCY functions ***/

/*******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_init(const SIGFOX_rc_t *rc, sfx_u8 *ep_id, sfx_u16 last_random_value) {
    // Local variables.
    sfx_u16 ep_id_16bits = 0;
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH) && !(defined SIGFOX_EP_SINGLE_FRAME)
    sfx_u8 idx = 0;
#endif
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameter.
    if ((rc == SIGFOX_NULL) || (ep_id == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Store RC and last random value.
    sigfox_ep_frequency_ctx.rc = rc;
    sigfox_ep_frequency_ctx.random_value = last_random_value;
    // Compute random offset with EP-ID.
    ep_id_16bits = (sfx_u16) (((((sfx_u16) ep_id[2]) << 8) & 0xFF00) | (((sfx_u16) ep_id[3]) & 0x00FF));
    // Note: the term (ep_id_16bits + 1) could overflow to 0 if ep_id_16bits = 0xFFFF.
    // In this case, the resulting random offset is 0 but will be set to 1 by the next operation (odd check)
    sigfox_ep_frequency_ctx.random_offset = (SIGFOX_EP_FREQUENCY_RANDOM_MULTIPLIER * (ep_id_16bits + 1)) & SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MASK;
    // Ensure random offset if odd.
    if (((sigfox_ep_frequency_ctx.random_offset) % 2) == 0) {
        sigfox_ep_frequency_ctx.random_offset++;
    }
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sigfox_ep_frequency_ctx.frame_1_frequency_hz = 0;
#endif
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH) && !(defined SIGFOX_EP_SINGLE_FRAME)
    // Init micro-channels frame count (only once).
    if (sigfox_ep_frequency_ctx.micro_channel_table_initialized == SIGFOX_FALSE) {
        for (idx = 0; idx < SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL; idx++) {
            sigfox_ep_frequency_ctx.micro_channel_frame_count[idx] = SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL_OPERATED;
        }
        sigfox_ep_frequency_ctx.micro_channel_table_initialized = SIGFOX_TRUE;
    }
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
#ifdef SIGFOX_EP_SINGLE_FRAME
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_uplink(sfx_u32 *ul_frequency_hz) {
#else
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_uplink(SIGFOX_EP_FREQUENCY_uplink_signal_t *input, sfx_u32 *ul_frequency_hz) {
#endif
#ifdef SIGFOX_EP_ERROR_CODES
    // Local variables.
    SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH) && !(defined SIGFOX_EP_SINGLE_FRAME)
    sfx_u8 idx = 0;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    // Check parameters.
#ifndef SIGFOX_EP_SINGLE_FRAME
    if (input == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
    }
#endif
    if (ul_frequency_hz == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
    }
#ifndef SIGFOX_EP_SINGLE_FRAME
    if ((input->ul_frame_rank) >= SIGFOX_UL_FRAME_RANK_LAST) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_FRAME_RANK);
    }
    if ((input->number_of_frames) > SIGFOX_UL_FRAME_RANK_LAST) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NUMBER_OF_FRAMES);
    }
    if ((input->ul_frame_rank) >= (input->number_of_frames)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_FRAME_RANK);
    }
#endif
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Update local pointer.
    sigfox_ep_frequency_ctx.input = input;
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH)
    // The FH timer (or Tw for DL-CONF) has necessarily been launched before a first frame, allowing all micro-channels to be used again.
    if ((input->ul_frame_rank) == SIGFOX_UL_FRAME_RANK_1) {
        for (idx = 0; idx < SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL; idx++) {
            sigfox_ep_frequency_ctx.micro_channel_frame_count[idx] = SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL_OPERATED;
        }
    }
#endif
#endif /* MULTIPLE_FRAME */
    // Compute frequency.
#ifdef SIGFOX_EP_SINGLE_FRAME
    // Single frame is always random.
    _COMPUTE_NEW_RANDOM_FREQUENCY();
#else /* SIGFOX_EP_SINGLE_FRAME */
    switch (input->ul_frame_rank) {
    case SIGFOX_UL_FRAME_RANK_1:
        // First frame is always random.
        _COMPUTE_NEW_RANDOM_FREQUENCY();
        break;
    case SIGFOX_UL_FRAME_RANK_2:
#ifdef SIGFOX_EP_BIDIRECTIONAL
        // Check bidirectional flag.
        if ((input->bidirectional_flag) == SIGFOX_TRUE) {
#if (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
            // Check frame 1 frequency.
            status = _check_frame_1_frequency();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_FREQUENCY_SUCCESS);
#endif
            // Add fixed frequency offset.
            (*ul_frequency_hz) = sigfox_ep_frequency_ctx.frame_1_frequency_hz + (((sigfox_ep_frequency_ctx.rc)->spectrum_access)->delta_f_mf_hz);
        }
        else {
            // Frequency is random.
            _COMPUTE_NEW_RANDOM_FREQUENCY();
        }
#else /* SIGFOX_EP_BIDIRECTIONAL */
        // Frequency is random.
        _COMPUTE_NEW_RANDOM_FREQUENCY();
#endif
        break;
    case SIGFOX_UL_FRAME_RANK_3:
#ifdef SIGFOX_EP_BIDIRECTIONAL
        // Check bidirectional flag.
        if ((input->bidirectional_flag) == SIGFOX_TRUE) {
#if (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
            // Check frame 1 frequency.
            status = _check_frame_1_frequency();
            SIGFOX_CHECK_STATUS(SIGFOX_EP_FREQUENCY_SUCCESS);
#endif
            // Add fixed frequency offset.
            (*ul_frequency_hz) = sigfox_ep_frequency_ctx.frame_1_frequency_hz - (((sigfox_ep_frequency_ctx.rc)->spectrum_access)->delta_f_mf_hz);
        }
        else {
            // Frequency is random.
            _COMPUTE_NEW_RANDOM_FREQUENCY();
        }
#else /* SIGFOX_EP_BIDIRECTIONAL */
        // Frequency is random.
        _COMPUTE_NEW_RANDOM_FREQUENCY();
#endif
        break;
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_FRAME_RANK);
    }
#endif /* SIGFOX_EP_SINGLE_FRAME */
#if (defined SIGFOX_EP_REGULATORY) && (defined SIGFOX_EP_SPECTRUM_ACCESS_FH) && !(defined SIGFOX_EP_SINGLE_FRAME)
    // Increment all micro-channels frame count.
    for (idx = 0; idx < SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL; idx++) {
        sigfox_ep_frequency_ctx.micro_channel_frame_count[idx]++;
    }
#endif
#if (defined SIGFOX_EP_PARAMETERS_CHECK) || ((defined SIGFOX_EP_SINGLE_FRAME) && (defined SIGFOX_EP_ERROR_CODES)) || !(defined SIGFOX_EP_SINGLE_FRAME)
errors:
#endif
    SIGFOX_RETURN();
}

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_downlink(sfx_u32 *dl_frequency_hz) {
    // Local variables.
    sfx_s32 delta_f_gap_hz = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if ((dl_frequency_hz == SIGFOX_NULL) || (sigfox_ep_frequency_ctx.rc == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
    }
#endif
#if (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
    // Check frame 1 frequency.
    status = _check_frame_1_frequency();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_FREQUENCY_SUCCESS);
#endif
    // Compute uplink and downlink bands offset.
    delta_f_gap_hz = ((sfx_s32) ((sigfox_ep_frequency_ctx.rc)->f_dl_hz)) - ((sfx_s32) ((sigfox_ep_frequency_ctx.rc)->f_ul_hz));
    // Compute downlink frequency.
    (*dl_frequency_hz) = (sfx_u32) (((sfx_s32) sigfox_ep_frequency_ctx.frame_1_frequency_hz) + delta_f_gap_hz);
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
#endif

/*******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_get_random_value(sfx_u16 *random_value) {
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#ifdef SIGFOX_EP_PARAMETERS_CHECK
    if (random_value == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
    }
#endif /* SIGFOX_EP_PARAMETERS_CHECK */
    // Update random value.
    (*random_value) = sigfox_ep_frequency_ctx.random_value;
#ifdef SIGFOX_EP_PARAMETERS_CHECK
errors:
#endif
    SIGFOX_RETURN();
}
