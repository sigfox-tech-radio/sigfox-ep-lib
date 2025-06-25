/*!*****************************************************************
 * \file    sigfox_types.c
 * \brief   Sigfox types definition.
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

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

/*** SIGFOX TYPES global variables ***/

#if (!(defined SIGFOX_EP_UL_BIT_RATE_BPS) || ((defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)))
/*!******************************************************************
 * \var SIGFOX_UL_BIT_RATE_BPS_LIST
 * \brief Sigfox bit rates value.
 *******************************************************************/
const sfx_u16 SIGFOX_UL_BIT_RATE_BPS_LIST[SIGFOX_UL_BIT_RATE_LAST] = { 100, 600 };
#endif

#if (!(defined SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER) || (defined SIGFOX_EP_PARAMETERS_CHECK))
/*!******************************************************************
 * \var SIGFOX_MESSAGE_COUNTER_ROLLOVER_LIST
 * \brief Sigfox message counter value.
 *******************************************************************/
const sfx_u16 SIGFOX_MESSAGE_COUNTER_ROLLOVER_LIST[SIGFOX_MESSAGE_COUNTER_ROLLOVER_LAST] = { 128, 256, 512, 1024, 2048, 4096 };
#endif

#ifdef SIGFOX_EP_CERTIFICATION
/*!******************************************************************
 * \var SIGFOX_EP_TEST_ID
 * \var SIGFOX_EP_TEST_KEY
 * \brief End-point IDs and keys used for test.
 *******************************************************************/
const sfx_u8 SIGFOX_EP_TEST_ID[SIGFOX_EP_ID_SIZE_BYTES] = { 0xFE, 0xDC, 0xBA, 0x98 };
const sfx_u8 SIGFOX_EP_TEST_KEY[SIGFOX_EP_KEY_SIZE_BYTES] = { 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF, 0x01, 0x23, 0x45, 0x67, 0x89, 0xAB, 0xCD, 0xEF };
#endif

#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
/*!******************************************************************
 * \var SIGFOX_EP_PUBLIC_KEY
 * \brief End-point public key used for test.
 *******************************************************************/
const sfx_u8 SIGFOX_EP_PUBLIC_KEY[SIGFOX_EP_KEY_SIZE_BYTES] = { 0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF };
#endif
