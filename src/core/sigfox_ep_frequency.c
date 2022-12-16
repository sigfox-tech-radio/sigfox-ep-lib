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

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

/*** SIGFOX EP FREQUENCY local macros ***/

// Define the number of bits of the random value used to select uplink frequency.
// Note: for 192kHz the maximum value to avoid overflow is 14: 192000 * (2^14 - 1) is still on 32 bits.
#define SIGFOX_EP_FREQUENCY_RANDOM_VALUE_SIZE_BITS		14
#define SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MASK			((1 << SIGFOX_EP_FREQUENCY_RANDOM_VALUE_SIZE_BITS) - 1)
#define SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MAX			SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MASK
#define SIGFOX_EP_FREQUENCY_RANDOM_MULTIPLIER			61
#define SIGFOX_EP_FREQUENCY_RANDOM_GENERATION_LIMIT		1000
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
#define SIGFOX_EP_FREQUENCY_FH_MICRO_CHANNEL_ERROR		0xFF
#endif

/*** SIGFOX EP FREQUENCY local structures ***/

/*******************************************************************/
typedef struct {
	const SIGFOX_rc_t *rc;
	sfx_u32 random_value;
	sfx_u32 random_offset;
	sfx_u32 baseband_frequency_min_hz;
	sfx_u32 baseband_frequency_max_hz;
	sfx_u32 baseband_bandwidth_hz;
#ifndef SINGLE_FRAME
	SIGFOX_EP_FREQUENCY_uplink_signal_t *input;
#endif
#ifdef BIDIRECTIONAL
	sfx_u32 frame_1_frequency_hz;
#endif
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	sfx_bool micro_channel_table_initialized;
	sfx_u8 micro_channel_frame_count[SIGFOX_FH_MICRO_CHANNEL_NUMBER];
#endif
} SIGFOX_EP_FREQUENCY_context_t;

/*** SIGFOX EP FREQUENCY local global variables ***/

#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
static SIGFOX_EP_FREQUENCY_context_t sigfox_ep_frequency_ctx = {
	.micro_channel_table_initialized = SFX_FALSE,
};
#else
static SIGFOX_EP_FREQUENCY_context_t sigfox_ep_frequency_ctx;
#endif

/*** SIGFOX EP FREQUENCY local functions ***/

/*!******************************************************************
 * \fn void _COMPUTE_NEW_RANDOM_FREQUENCY()
 * \brief Generic macro to call the random frequency generator function.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
#ifdef ERROR_CODES
#define _COMPUTE_NEW_RANDOM_FREQUENCY() { status = _compute_new_random_frequency(ul_frequency_hz); CHECK_STATUS(SIGFOX_EP_FREQUENCY_SUCCESS); }
#else
#define _COMPUTE_NEW_RANDOM_FREQUENCY() { _compute_new_random_frequency(ul_frequency_hz); }
#endif

/*******************************************************************/
static void _compute_baseband_frequency_range(void) {
	// Remove guard band.
	sigfox_ep_frequency_ctx.baseband_frequency_min_hz = ((sigfox_ep_frequency_ctx.rc) -> macro_channel_guard_band_hz);
	sigfox_ep_frequency_ctx.baseband_frequency_max_hz = SIGFOX_MACRO_CHANNEL_WIDTH_HZ - ((sigfox_ep_frequency_ctx.rc) -> macro_channel_guard_band_hz);
#if (defined BIDIRECTIONAL) && !(defined SINGLE_FRAME)
	// Add offset in case of bidirectional procedure.
	sigfox_ep_frequency_ctx.baseband_frequency_min_hz += (((sigfox_ep_frequency_ctx.rc) -> spectrum_access) -> delta_f_mf_hz) * ((sigfox_ep_frequency_ctx.input) -> bidirectional_flag);
	sigfox_ep_frequency_ctx.baseband_frequency_max_hz -= (((sigfox_ep_frequency_ctx.rc) -> spectrum_access) -> delta_f_mf_hz) * ((sigfox_ep_frequency_ctx.input) -> bidirectional_flag);
#endif
	// Compute bandwidth.
	sigfox_ep_frequency_ctx.baseband_bandwidth_hz = (sigfox_ep_frequency_ctx.baseband_frequency_max_hz - sigfox_ep_frequency_ctx.baseband_frequency_min_hz);
}

#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH) && !(defined SINGLE_FRAME)
/*******************************************************************/
static sfx_bool _is_micro_channel_free(sfx_u8 micro_channel_index) {
	// Thanks to the FH timer, a micro-channel is free when all frames of the message have been sent.
	sfx_bool micro_channel_free = (sigfox_ep_frequency_ctx.micro_channel_frame_count[micro_channel_index] >= SIGFOX_UL_FRAME_RANK_LAST) ? SFX_TRUE : SFX_FALSE;
	return micro_channel_free;
}
#endif

#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
/*******************************************************************/
static sfx_u8 _get_micro_channel_index(sfx_u32 baseband_frequency_hz) {
	// Local variables.
	sfx_u8 micro_channel_index = 0;
	// Compute index.
	micro_channel_index = ((baseband_frequency_hz + (SIGFOX_FH_MACRO_CHANNEL_DELTA / 2)) / SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ);
	micro_channel_index %= SIGFOX_FH_MICRO_CHANNEL_NUMBER;
	// Check micro-channels mask.
	if (((SIGFOX_FH_MICRO_CHANNEL_MASK >> micro_channel_index) & 0x01) == 0) {
		micro_channel_index = SIGFOX_EP_FREQUENCY_FH_MICRO_CHANNEL_ERROR;
		goto errors;
	}
	// Check low side micro-channel guard band.
	if ((baseband_frequency_hz - (SIGFOX_FH_MACRO_CHANNEL_GUARD_BAND_HZ + (SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ * (micro_channel_index - 1)))) < SIGFOX_FH_MICRO_CHANNEL_GUARD_BAND_HZ) {
		micro_channel_index = SIGFOX_EP_FREQUENCY_FH_MICRO_CHANNEL_ERROR;
		goto errors;
	}
	// Check high side micro-channel guard band.
	if (((SIGFOX_FH_MACRO_CHANNEL_GUARD_BAND_HZ + (SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ * micro_channel_index)) - baseband_frequency_hz) < SIGFOX_FH_MICRO_CHANNEL_GUARD_BAND_HZ) {
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
#ifdef ERROR_CODES
	SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
	sfx_u32 baseband_frequency_hz = 0;
	sfx_u32 random_generation_count = 0;
	sfx_bool baseband_frequency_allowed = SFX_FALSE;
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	sfx_u8 micro_channel_index = 0;
#endif
	// Call the random algorithm until a valid frequency is computed.
	do {
		// Generate new random value.
		sigfox_ep_frequency_ctx.random_value = (SIGFOX_EP_FREQUENCY_RANDOM_MULTIPLIER * sigfox_ep_frequency_ctx.random_value) + sigfox_ep_frequency_ctx.random_offset;
		sigfox_ep_frequency_ctx.random_value &= SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MASK;
		random_generation_count++;
		// Convert to baseband frequency.
		baseband_frequency_hz = ((sfx_u32) sigfox_ep_frequency_ctx.baseband_frequency_min_hz);
		baseband_frequency_hz += (((sfx_u32) sigfox_ep_frequency_ctx.random_value) * ((sfx_u32) (sigfox_ep_frequency_ctx.baseband_bandwidth_hz))) / (SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MAX);
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
		// Additional check on micro-channel in case of FCC
		if (((sigfox_ep_frequency_ctx.rc) -> spectrum_access -> type) == SIGFOX_SPECTRUM_ACCESS_TYPE_FH) {
			// Get micro channel index.
			micro_channel_index = _get_micro_channel_index(baseband_frequency_hz);
			if (micro_channel_index != SIGFOX_EP_FREQUENCY_FH_MICRO_CHANNEL_ERROR) {
#ifdef SINGLE_FRAME
				// Thanks to the FH timer, a given micro-channels is always free in single frame mode.
				baseband_frequency_allowed = SFX_TRUE;
#else
				// Micro-channel index is valid, check if it is free.
				baseband_frequency_allowed = _is_micro_channel_free(micro_channel_index);
#endif
			}
			else {
				// Frequency is in guard band.
				baseband_frequency_allowed = SFX_FALSE;
			}
		}
		else {
			// Frequency is valid.
			baseband_frequency_allowed = SFX_TRUE;
		}
#else
		baseband_frequency_allowed = SFX_TRUE;
#endif
		// Check request count.
		if (random_generation_count > SIGFOX_EP_FREQUENCY_RANDOM_GENERATION_LIMIT) {
			EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_RANDOM_GENERATION);
		}
	}
	while (baseband_frequency_allowed == SFX_FALSE);
	// Convert to absolute frequency.
	(*frequency_hz) = ((sigfox_ep_frequency_ctx.rc) -> f_ul_hz) - (SIGFOX_MACRO_CHANNEL_WIDTH_HZ / 2) + baseband_frequency_hz;
#ifdef BIDIRECTIONAL
	// Store frame 1 frequency for future computation if bidirectional.
#ifdef SINGLE_FRAME
	sigfox_ep_frequency_ctx.frame_1_frequency_hz = (*frequency_hz);
#else
	if (((sigfox_ep_frequency_ctx.input) -> ul_frame_rank) == SIGFOX_UL_FRAME_RANK_1) {
		sigfox_ep_frequency_ctx.frame_1_frequency_hz = (*frequency_hz);
	}
#endif /* SINGLE_FRAME */
#endif /* BIDIRECTIONAL */
errors:
	RETURN();
}

/*** SIGFOX EP FREQUENCY functions ***/

/*******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_init(const SIGFOX_rc_t *rc, sfx_u8 *ep_id, sfx_u16 last_random_value) {
	// Local variables.
	sfx_u32 ep_id_16bits = 0;
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	sfx_u8 idx = 0;
#endif
#ifdef ERROR_CODES
	SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	// Check parameter.
	if ((rc == SFX_NULL) || (ep_id == SFX_NULL)) {
		EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
	}
#endif /* PARAMETERS_CHECK */
	// Store RC and last random value.
	sigfox_ep_frequency_ctx.rc = rc;
	sigfox_ep_frequency_ctx.random_value = last_random_value;
	// Compute random offset with EP-ID.
	ep_id_16bits = ((((sfx_u32) ep_id[2]) << 8) & 0xFF00) | (((sfx_u32) ep_id[3]) & 0x00FF);
	sigfox_ep_frequency_ctx.random_offset = (SIGFOX_EP_FREQUENCY_RANDOM_MULTIPLIER * (ep_id_16bits + 1)) & SIGFOX_EP_FREQUENCY_RANDOM_VALUE_MASK;
	if (((sigfox_ep_frequency_ctx.random_offset) % 2) == 0) {
		sigfox_ep_frequency_ctx.random_offset++;
	}
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	// Init micro-channels frame count (only once).
	if (sigfox_ep_frequency_ctx.micro_channel_table_initialized == SFX_FALSE) {
		for (idx=0 ; idx<SIGFOX_FH_MICRO_CHANNEL_NUMBER ; idx++) {
			sigfox_ep_frequency_ctx.micro_channel_frame_count[idx] = SIGFOX_FH_MICRO_CHANNEL_USED * ((SIGFOX_FH_MICRO_CHANNEL_MASK >> idx) & 0x01);
		}
		sigfox_ep_frequency_ctx.micro_channel_table_initialized = SFX_TRUE;
	}
#endif
#ifdef PARAMETERS_CHECK
errors:
#endif
	RETURN();
}

/*******************************************************************/
#ifdef SINGLE_FRAME
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_uplink(sfx_u32 *ul_frequency_hz) {
#else
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_uplink(SIGFOX_EP_FREQUENCY_uplink_signal_t *input, sfx_u32 *ul_frequency_hz) {
#endif
#ifdef ERROR_CODES
	// Local variables.
	SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	sfx_u32 baseband_frequency_hz = 0;
	sfx_u8 micro_channel_index = 0;
#endif
#ifdef PARAMETERS_CHECK
	// Check parameters.
#ifndef SINGLE_FRAME
	if (input == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
	}
#endif
	if (ul_frequency_hz == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
	}
#ifndef SINGLE_FRAME
	if ((input -> ul_frame_rank) >= SIGFOX_UL_FRAME_RANK_LAST) {
		EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_FRAME_RANK);
	}
#endif
#endif /* PARAMETERS_CHECK */
#ifndef SINGLE_FRAME
	// Update local pointers.
	sigfox_ep_frequency_ctx.input = input;
#endif
	// Update frequency range.
	_compute_baseband_frequency_range();
	// Compute frequency.
#ifdef SINGLE_FRAME
	// Single frame is always random.
	_COMPUTE_NEW_RANDOM_FREQUENCY();
#else /* SINGLE_FRAME */
	switch (input -> ul_frame_rank) {
	case SIGFOX_UL_FRAME_RANK_1:
#ifdef BIDIRECTIONAL
		// Reset frame 1 frequency (for error management on frame 2 and 3).
		sigfox_ep_frequency_ctx.frame_1_frequency_hz = 0;
#endif
		// First frame is always random.
		_COMPUTE_NEW_RANDOM_FREQUENCY();
		break;
	case SIGFOX_UL_FRAME_RANK_2:
#ifdef BIDIRECTIONAL
		// Check bidirectional flag.
		if ((input -> bidirectional_flag) == SFX_TRUE) {
			if (sigfox_ep_frequency_ctx.frame_1_frequency_hz != 0) {
				// Add fixed frequency offset.
				(*ul_frequency_hz) = sigfox_ep_frequency_ctx.frame_1_frequency_hz + (((sigfox_ep_frequency_ctx.rc) -> spectrum_access) -> delta_f_mf_hz);
			}
			else {
				EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_FRAME_1_FREQUENCY);
			}
		}
		else {
			// Frequency is random.
			_COMPUTE_NEW_RANDOM_FREQUENCY();
		}
#else /* BIDIRECTIONAL */
		// Frequency is random.
		_COMPUTE_NEW_RANDOM_FREQUENCY();
#endif
		break;
	case SIGFOX_UL_FRAME_RANK_3:
#ifdef BIDIRECTIONAL
		// Check bidirectional flag.
		if ((input -> bidirectional_flag) == SFX_TRUE) {
			// Check if frame 1 frequency was correctly computed.
			if (sigfox_ep_frequency_ctx.frame_1_frequency_hz != 0) {
				// Add fixed frequency offset.
				(*ul_frequency_hz) = sigfox_ep_frequency_ctx.frame_1_frequency_hz - (((sigfox_ep_frequency_ctx.rc) -> spectrum_access) -> delta_f_mf_hz);
			}
			else {
				EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_FRAME_1_FREQUENCY);
			}
		}
		else {
			// Frequency is random.
			_COMPUTE_NEW_RANDOM_FREQUENCY();
		}
#else /* BIDIRECTIONAL */
		// Frequency is random.
		_COMPUTE_NEW_RANDOM_FREQUENCY();
#endif
		break;
	default:
		EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_FRAME_RANK);
	}
#endif /* SINGLE_FRAME */
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	// Reset selected micro-channel frame count.
	baseband_frequency_hz = (*ul_frequency_hz) + (SIGFOX_MACRO_CHANNEL_WIDTH_HZ / 2);
	baseband_frequency_hz -= ((sigfox_ep_frequency_ctx.rc) -> f_ul_hz);
	micro_channel_index = _get_micro_channel_index(baseband_frequency_hz);
	sigfox_ep_frequency_ctx.micro_channel_frame_count[micro_channel_index] = 0;
	// Increment all micro channel frame count.
	for (micro_channel_index=0 ; micro_channel_index<SIGFOX_FH_MICRO_CHANNEL_NUMBER ; micro_channel_index++) {
		sigfox_ep_frequency_ctx.micro_channel_frame_count[micro_channel_index]++;
	}
#endif
#if (defined PARAMETERS_CHECK) || ((defined SINGLE_FRAME) && (defined ERROR_CODES)) || !(defined SINGLE_FRAME)
errors:
#endif
	RETURN();
}

#ifdef BIDIRECTIONAL
/*******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_downlink(sfx_u32* dl_frequency_hz) {
	// Local variables.
	sfx_s32 ul_dl_offset_hz = 0;
#ifdef ERROR_CODES
	SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	if ((dl_frequency_hz == SFX_NULL) || (sigfox_ep_frequency_ctx.rc == SFX_NULL)) {
		EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
	}
#endif
	// Compute uplink and downlink bands offset.
	ul_dl_offset_hz = ((sfx_s32) ((sigfox_ep_frequency_ctx.rc) -> f_dl_hz)) - ((sfx_s32) ((sigfox_ep_frequency_ctx.rc) -> f_ul_hz));
	// Compute downlink frequency.
	(*dl_frequency_hz) = (sfx_u32) (((sfx_s32) sigfox_ep_frequency_ctx.frame_1_frequency_hz) + ul_dl_offset_hz);
#ifdef PARAMETERS_CHECK
errors:
#endif
	RETURN();
}
#endif

/*******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_get_random_value(sfx_u16 *random_value) {
#ifdef ERROR_CODES
	SIGFOX_EP_FREQUENCY_status_t status = SIGFOX_EP_FREQUENCY_SUCCESS;
#endif
#ifdef PARAMETERS_CHECK
	if (random_value == SFX_NULL) {
		EXIT_ERROR(SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER);
	}
#endif /* PARAMETERS_CHECK */
	// Update random value.
	(*random_value) = sigfox_ep_frequency_ctx.random_value;
#ifdef PARAMETERS_CHECK
errors:
#endif
	RETURN();
}

