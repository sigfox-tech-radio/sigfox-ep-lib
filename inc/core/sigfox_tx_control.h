/*!*****************************************************************
 * \file    sigfox_tx_control.h
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

#ifndef __SIGFOX_TX_CONTROL_H__
#define __SIGFOX_TX_CONTROL_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

#ifdef REGULATORY

/*** SIGFOX TX CONTROL structures ***/

#ifdef ERROR_CODES
/*!******************************************************************
 * \enum SIGFOX_TX_CONTROL_status_t
 * \brief Sigfox TX control driver error codes.
 *******************************************************************/
typedef enum {
	SIGFOX_TX_CONTROL_SUCCESS = 0,
	SIGFOX_TX_CONTROL_ERROR_NULL_PARAMETER,
	SIGFOX_TX_CONTROL_ERROR_SPECTRUM_ACCESS,
	// Low level errors.
	SIGFOX_TX_CONTROL_ERROR_MCU,
	SIGFOX_TX_CONTROL_ERROR_RF
} SIGFOX_TX_CONTROL_status_t;
#else
typedef void SIGFOX_TX_CONTROL_status_t;
#endif

typedef enum {
	SIGFOX_TX_CONTROL_RESULT_ALLOWED,
	SIGFOX_TX_CONTROL_RESULT_FORBIDDEN,
	SIGFOX_TX_CONTROL_RESULT_PENDING
} SIGFOX_TX_CONTROL_result_t;

#ifdef ASYNCHRONOUS
/*!******************************************************************
 * \brief Sigfox TX control driver callback functions.
 * \fn SIGFOX_TX_CONTROL_process_cb_t		To be called when the TX control needs to be processed.
 * \fn SIGFOX_TX_CONTROL_check_cplt_cb_t	To be called when the TX control check is complete.
 *******************************************************************/
typedef void (*SIGFOX_TX_CONTROL_process_cb_t)(void);
typedef void (*SIGFOX_TX_CONTROL_check_cplt_cb_t)(void);
#endif

/*!******************************************************************
 * \enum SIGFOX_TX_CONTROL_check_type
 * \brief TX control check type.
 *******************************************************************/
typedef enum {
	SIGFOX_TX_CONTROL_TYPE_PRE_CHECK,
	SIGFOX_TX_CONTROL_TYPE_POST_CHECK,
	SIGFOX_TX_CONTROL_TYPE_LAST
} SIGFOX_TX_CONTROL_check_type;

/*!******************************************************************
 * \struct SIGFOX_TX_CONTROL_parameters_t
 * \brief Parameters for TX control check.
 *******************************************************************/
typedef struct {
	SIGFOX_TX_CONTROL_check_type type;
	sfx_u8 bitstream_length_bytes;
	sfx_bool last_message_frame;
#ifndef SINGLE_FRAME
	SIGFOX_ul_frame_rank_t ul_frame_rank;
	sfx_u8 number_of_frames;
#endif
#ifndef UL_BIT_RATE
	sfx_u16 ul_bit_rate_bps;
#endif
#ifdef BIDIRECTIONAL
	sfx_bool ack_message;
#endif
#if !(defined SINGLE_FRAME) || (defined BIDIRECTIONAL)
	sfx_u32 interframe_ms; // Tifu, Tifb or Tconf.
#endif
#ifdef CERTIFICATION
#ifdef SPECTRUM_ACCESS_FH
	sfx_bool fh_timer_enable;
#endif
#ifdef SPECTRUM_ACCESS_LBT
	sfx_bool lbt_enable;
#endif
#endif
#ifdef ASYNCHRONOUS
	SIGFOX_TX_CONTROL_check_cplt_cb_t cplt_cb;
#endif
} SIGFOX_TX_CONTROL_parameters_t;

/*!******************************************************************
 * \struct SIGFOX_TX_CONTROL_config_t
 * \brief Sigfox TX control configuration structure.
 *******************************************************************/
typedef struct {
	const SIGFOX_rc_t *rc;
#ifdef ASYNCHRONOUS
	SIGFOX_TX_CONTROL_process_cb_t process_cb;
#endif
} SIGFOX_TX_CONTROL_config_t;

/*** SIGFOX TX CONTROL functions ***/

/*!******************************************************************
 * \fn SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_open(SIGFOX_TX_CONTROL_config_t *config)
 * \brief Open the TX control driver.
 * \param[in]  	config: Pointer to the TX control configuration.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_open(SIGFOX_TX_CONTROL_config_t *config);

/*!******************************************************************
 * \fn SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_check(SIGFOX_TX_CONTROL_parameters_t *params)
 * \brief Start TX control operation.
 * \brief In asynchronous mode, the check completion should be notified by calling the given cplt_cb() function.
 * \param[in]  	params: Pointers to the TX control parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_check(SIGFOX_TX_CONTROL_parameters_t *params);

#ifdef ASYNCHRONOUS
/*!******************************************************************
 * \fn SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_process(void)
 * \brief Process TX CONTROL driver, this function will be call by SIGFOX_EP_API_process just after the process_callback has been sent to process TX control interruptions in main context.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_process(void);
#endif

/*!******************************************************************
 * \fn sfx_bool SIGFOX_TX_CONTROL_is_radio_required(SIGFOX_TX_CONTROL_check_type check_type)
 * \brief Indicate if the RF_API is required to perform the given check.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		SFX_TRUE if the radio is required; SFX_FALSE otherwise.
 *******************************************************************/
sfx_bool SIGFOX_TX_CONTROL_is_radio_required(SIGFOX_TX_CONTROL_check_type check_type);

/*!******************************************************************
 * \fn SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_get_result(SIGFOX_TX_CONTROL_result_t *result)
 * \brief Get last TX control operation result
 * \param[in]	none
 * \param[out] 	result: Pointer that will contain the TX control result.
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_TX_CONTROL_status_t SIGFOX_TX_CONTROL_get_result(SIGFOX_TX_CONTROL_result_t *result);

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_TX_CONTROL_stack_error(void)
 * \brief Generic macro which calls the error stack function for TX control errors (if enabled).
 * \param[in]  	none
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#ifdef ERROR_STACK
#define SIGFOX_TX_CONTROL_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_TX_CONTROL, tx_control_status)
#else
#define SIGFOX_TX_CONTROL_stack_error(void)
#endif
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void SIGFOX_TX_CONTROL_check_status(error)
 * \brief Generic macro to check a SIGFOX_TX_CONTROL function status and exit.
 * \param[in]  	error: High level error code to rise.
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#define SIGFOX_TX_CONTROL_check_status(error) { if (tx_control_status != SIGFOX_TX_CONTROL_SUCCESS) { SIGFOX_TX_CONTROL_stack_error(); EXIT_ERROR(error) } }
#endif

#endif /* REGULATORY */

#endif /* __SIGFOX_TX_CONTROL_H__ */
