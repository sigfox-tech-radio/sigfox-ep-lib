/*!*****************************************************************
 * \file    sigfox_ep_frequency.h
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

#ifndef __SIGFOX_EP_FREQUENCY_H__
#define __SIGFOX_EP_FREQUENCY_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

/*** SIGFOX EP FREQUENCY structures ***/

#ifdef ERROR_CODES
/*!******************************************************************
 * \enum SIGFOX_EP_FREQUENCY_status_t
 * \brief Sigfox frequency error codes.
 *******************************************************************/
typedef enum {
	SIGFOX_EP_FREQUENCY_SUCCESS = 0,
	SIGFOX_EP_FREQUENCY_ERROR_NULL_PARAMETER,
	SIGFOX_EP_FREQUENCY_ERROR_FRAME_RANK,
	SIGFOX_EP_FREQUENCY_ERROR_NUMBER_OF_FRAMES,
	SIGFOX_EP_FREQUENCY_ERROR_SPECTRUM_ACCESS_TYPE,
	SIGFOX_EP_FREQUENCY_ERROR_RANDOM_GENERATION,
	SIGFOX_EP_FREQUENCY_ERROR_FRAME_1_FREQUENCY,
} SIGFOX_EP_FREQUENCY_status_t;
#else
typedef void SIGFOX_EP_FREQUENCY_status_t;
#endif

#ifndef SINGLE_FRAME
/*!******************************************************************
 * \struct SIGFOX_EP_FREQUENCY_uplink_signal_t
 * \brief Frequency computation input parameters.
 *******************************************************************/
typedef struct {
	SIGFOX_ul_frame_rank_t ul_frame_rank;
	sfx_u8 number_of_frames;
#ifdef BIDIRECTIONAL
	sfx_bool bidirectional_flag;
#endif
} SIGFOX_EP_FREQUENCY_uplink_signal_t;
#endif

/*** SIGFOX EP FREQUENCY functions ***/

/*!******************************************************************
 * \fn SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_init(const SIGFOX_rc_t *rc, sfx_u8 *ep_id, sfx_u16 last_random_value)
 * \brief Init the frequecy driver.
 * \param[in]	rc: Radio configuration.
 * \param[in]	ep_id: Device ID (used to randomize the frequency algorithm).
 * \param[in]  	last_random_value: Last random value (read in NVM at device start-up).
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_init(const SIGFOX_rc_t *rc, sfx_u8 *ep_id, sfx_u16 last_random_value);

#ifdef SINGLE_FRAME
/*!******************************************************************
 * \fn SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_uplink(sfx_u32 *ul_frequency_hz)
 * \brief Compute the next Sigfox signal uplink frequency.
 * \param[in]  	none
 * \param[out]  ul_frequency_hz: Pointer that will contain the computed frequency in Hz.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_uplink(sfx_u32 *ul_frequency_hz);
#else
/*!******************************************************************
 * \fn SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_uplink(SIGFOX_EP_FREQUENCY_uplink_signal_t *input, sfx_u32 *ul_frequency_hz)
 * \brief Compute the next Sigfox signal uplink frequency according to the input parameters.
 * \param[in]  	input: Pointer to the signal parameters structure.
 * \param[out]  ul_frequency_hz: Pointer that will contain the computed frequency in Hz.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_uplink(SIGFOX_EP_FREQUENCY_uplink_signal_t *input, sfx_u32 *ul_frequency_hz);
#endif

#ifdef BIDIRECTIONAL
/*!******************************************************************
 * \fn SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_downlink(sfx_u32* dl_frequency_hz)
 * \brief Compute the Sigfox downlink frequency.
 * \param[in]  	none
 * \param[in]  	dl_frequency_hz: Pointer that will contain the downlink frequency in Hz (computed according to the last data given to the SIGFOX_EP_FREQUENCY_compute_uplink function).
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_compute_downlink(sfx_u32* dl_frequency_hz);
#endif

/*!******************************************************************
 * \fn SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_get_random_value(sfx_u16 *random_value)
 * \brief Return the current random value of the frequency driver (to be stored in NVM).
 * \param[in]	none
 * \param[out]  random_value: Pointer that will contain current random value.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_FREQUENCY_status_t SIGFOX_EP_FREQUENCY_get_random_value(sfx_u16 *random_value);

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_EP_FREQUENCY_stack_error(void)
 * \brief Generic macro which calls the error stack function for frequency errors (if enabled).
 * \param[in]  	none
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#ifdef ERROR_STACK
#define SIGFOX_EP_FREQUENCY_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_FREQUENCY, frequency_status)
#else
#define SIGFOX_EP_FREQUENCY_stack_error(void)
#endif
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_EP_FREQUENCY_check_status(error)
 * \brief Generic macro to check a SIGFOX_EP_FREQUENCY function status and exit.
 * \param[in]  	error: High level error code to rise.
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#define SIGFOX_EP_FREQUENCY_check_status(error) { if (frequency_status != SIGFOX_EP_FREQUENCY_SUCCESS) { SIGFOX_EP_FREQUENCY_stack_error(); EXIT_ERROR(error) } }
#endif

#endif /* __SIGFOX_EP_FREQUENCY_H__ */
