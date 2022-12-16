/*!*****************************************************************
 * \file    sigfox_ep_api_test.h
 * \brief   Sigfox End-Point library test API.
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

#ifndef __SIGFOX_EP_API_TEST_H__
#define __SIGFOX_EP_API_TEST_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_ep_api.h"
#include "sigfox_types.h"

#ifdef CERTIFICATION

/*** SIGFOX EP API TEST structures ***/

/*!******************************************************************
 * \struct SIGFOX_EP_API_TEST_parameters_t
 * \brief Specific parameters for test (RFP ADDON).
 *******************************************************************/
typedef struct {
	sfx_u32 tx_frequency_hz; // If non-zero, bypass the frequency driver of the core library.
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_FH)
	sfx_bool fh_timer_enable; // Enable or disable FH timer.
#endif
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
	sfx_bool lbt_enable;
#endif
} SIGFOX_EP_API_TEST_parameters_t;

#ifdef APPLICATION_MESSAGES
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_TEST_send_application_message(SIGFOX_EP_API_application_message_t *application_message, SIGFOX_EP_API_TEST_parameters_t *test_parameters)
 * \brief Send an application message over Sigfox network.
 * \param[in]  	application_message: Pointer to the application message data.
 * \param[in]	test_parameters: Pointer to the test parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_TEST_send_application_message(SIGFOX_EP_API_application_message_t *application_message, SIGFOX_EP_API_TEST_parameters_t *test_parameters);
#endif

#ifdef CONTROL_KEEP_ALIVE_MESSAGE
/*!******************************************************************
 * \fn SIGFOX_EP_API_status_t SIGFOX_EP_API_TEST_send_control_message(SIGFOX_EP_API_control_message_t *control_message, SIGFOX_EP_API_TEST_parameters_t *test_parameters)
 * \brief Send a control message over Sigfox network.
 * \param[in]  	control_message: Pointer to the control message data.
 * \param[in]	test_parameters: Pointer to the test parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
SIGFOX_EP_API_status_t SIGFOX_EP_API_TEST_send_control_message(SIGFOX_EP_API_control_message_t *control_message, SIGFOX_EP_API_TEST_parameters_t *test_parameters);
#endif

#endif /* CERTIFICATION */

#endif /* __SIGFOX_EP_API_TEST_H__ */
