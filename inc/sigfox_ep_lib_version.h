/*!*****************************************************************
 * \file    sigfox_ep_lib_version.h
 * \brief   Sigfox End-Point library version.
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

#ifndef __SIGFOX_EP_LIB_VERSION_H__
#define __SIGFOX_EP_LIB_VERSION_H__

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif

/*** Main version ***/

#define SIGFOX_EP_LIB_VERSION       "v4.0"

/*** Compilation flags ***/

#define SIGFOX_VERSION_SEPARATOR    " "

#ifdef SIGFOX_EP_DISABLE_FLAGS_FILE
#define SIGFOX_EP_DISABLE_FLAGS_FILE_OPT            SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_DISABLE_FLAGS_FILE"
#else
#define SIGFOX_EP_DISABLE_FLAGS_FILE_OPT
#endif

#ifdef SIGFOX_EP_RC1_ZONE
#define SIGFOX_EP_RC1_ZONE_OPT                      SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_RC1_ZONE"
#else
#define SIGFOX_EP_RC1_ZONE_OPT
#endif

#ifdef SIGFOX_EP_RC2_ZONE
#define SIGFOX_EP_RC2_ZONE_OPT                      SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_RC2_ZONE"
#else
#define SIGFOX_EP_RC2_ZONE_OPT
#endif

#ifdef SIGFOX_EP_RC3_LBT_ZONE
#define SIGFOX_EP_RC3_LBT_ZONE_OPT                  SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_RC3_LBT_ZONE"
#else
#define SIGFOX_EP_RC3_LBT_ZONE_OPT
#endif

#ifdef SIGFOX_EP_RC3_LDC_ZONE
#define SIGFOX_EP_RC3_LDC_ZONE_OPT                  SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_RC3_LDC_ZONE"
#else
#define SIGFOX_EP_RC3_LDC_ZONE_OPT
#endif

#ifdef SIGFOX_EP_RC4_ZONE
#define SIGFOX_EP_RC4_ZONE_OPT                      SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_RC4_ZONE"
#else
#define SIGFOX_EP_RC4_ZONE_OPT
#endif

#ifdef SIGFOX_EP_RC5_ZONE
#define SIGFOX_EP_RC5_ZONE_OPT                      SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_RC5_ZONE"
#else
#define SIGFOX_EP_RC5_ZONE_OPT
#endif

#ifdef SIGFOX_EP_RC6_ZONE
#define SIGFOX_EP_RC6_ZONE_OPT                      SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_RC6_ZONE"
#else
#define SIGFOX_EP_RC6_ZONE_OPT
#endif

#ifdef SIGFOX_EP_RC7_ZONE
#define SIGFOX_EP_RC7_ZONE_OPT                      SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_RC7_ZONE"
#else
#define SIGFOX_EP_RC7_ZONE_OPT
#endif

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
#define SIGFOX_EP_APPLICATION_MESSAGES_OPT          SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_APPLICATION_MESSAGES"
#else
#define SIGFOX_EP_APPLICATION_MESSAGES_OPT
#endif

#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
#define SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE_OPT    SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE"
#else
#define SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE_OPT
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
#define SIGFOX_EP_BIDIRECTIONAL_OPT                 SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_BIDIRECTIONAL"
#else
#define SIGFOX_EP_BIDIRECTIONAL_OPT
#endif

#ifdef SIGFOX_EP_ASYNCHRONOUS
#define SIGFOX_EP_ASYNCHRONOUS_OPT                  SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_ASYNCHRONOUS"
#else
#define SIGFOX_EP_ASYNCHRONOUS_OPT
#endif

#ifdef SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
#define SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE_OPT          SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE"
#else
#define SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE_OPT
#endif

#ifdef SIGFOX_EP_REGULATORY
#define SIGFOX_EP_REGULATORY_OPT                    SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_REGULATORY"
#else
#define SIGFOX_EP_REGULATORY_OPT
#endif

#ifdef SIGFOX_EP_LATENCY_COMPENSATION
#define SIGFOX_EP_LATENCY_COMPENSATION_OPT          SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_LATENCY_COMPENSATION"
#else
#define SIGFOX_EP_LATENCY_COMPENSATION_OPT
#endif

#ifdef SIGFOX_EP_SINGLE_FRAME
#define SIGFOX_EP_SINGLE_FRAME_OPT                  SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_SINGLE_FRAME"
#else
#define SIGFOX_EP_SINGLE_FRAME_OPT
#endif

#ifdef SIGFOX_EP_UL_BIT_RATE_BPS
#define SIGFOX_EP_UL_BIT_RATE_BPS_OPT               SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_UL_BIT_RATE_BPS"
#else
#define SIGFOX_EP_UL_BIT_RATE_BPS_OPT
#endif

#ifdef SIGFOX_EP_T_IFU_MS
#define SIGFOX_EP_T_IFU_MS_OPT                      SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_T_IFU_MS"
#else
#define SIGFOX_EP_T_IFU_MS_OPT
#endif

#ifdef SIGFOX_EP_T_CONF_MS
#define SIGFOX_EP_T_CONF_MS_OPT                     SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_T_CONF_MS"
#else
#define SIGFOX_EP_T_CONF_MS_OPT
#endif

#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#define SIGFOX_EP_UL_PAYLOAD_SIZE_OPT               SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_UL_PAYLOAD_SIZE"
#else
#define SIGFOX_EP_UL_PAYLOAD_SIZE_OPT
#endif

#ifdef SIGFOX_EP_AES_HW
#define SIGFOX_EP_AES_HW_OPT                        SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_AES_HW"
#else
#define SIGFOX_EP_AES_HW_OPT
#endif

#ifdef SIGFOX_EP_CRC_HW
#define SIGFOX_EP_CRC_HW_OPT                        SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_CRC_HW"
#else
#define SIGFOX_EP_CRC_HW_OPT
#endif

#ifdef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
#define SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER_OPT      SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER"
#else
#define SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER_OPT
#endif

#ifdef SIGFOX_EP_PARAMETERS_CHECK
#define SIGFOX_EP_PARAMETERS_CHECK_OPT              SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_PARAMETERS_CHECK"
#else
#define SIGFOX_EP_PARAMETERS_CHECK_OPT
#endif

#ifdef SIGFOX_EP_CERTIFICATION
#define SIGFOX_EP_CERTIFICATION_OPT                 SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_CERTIFICATION"
#else
#define SIGFOX_EP_CERTIFICATION_OPT
#endif

#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
#define SIGFOX_EP_PUBLIC_KEY_CAPABLE_OPT            SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_PUBLIC_KEY_CAPABLE"
#else
#define SIGFOX_EP_PUBLIC_KEY_CAPABLE_OPT
#endif

#ifdef SIGFOX_EP_VERBOSE
#define SIGFOX_EP_VERBOSE_OPT                       SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_VERBOSE"
#else
#define SIGFOX_EP_VERBOSE_OPT
#endif

#ifdef SIGFOX_EP_ERROR_CODES
#define SIGFOX_EP_ERROR_CODES_OPT                   SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_ERROR_CODES"
#else
#define SIGFOX_EP_ERROR_CODES_OPT
#endif

#ifdef SIGFOX_EP_ERROR_STACK
#define SIGFOX_EP_ERROR_STACK_OPT                   SIGFOX_VERSION_SEPARATOR "SIGFOX_EP_ERROR_STACK"
#else
#define SIGFOX_EP_ERROR_STACK_OPT
#endif

#define SIGFOX_EP_FLAGS \
    SIGFOX_EP_DISABLE_FLAGS_FILE_OPT \
    SIGFOX_EP_RC1_ZONE_OPT \
    SIGFOX_EP_RC2_ZONE_OPT \
    SIGFOX_EP_RC3_LBT_ZONE_OPT \
    SIGFOX_EP_RC3_LDC_ZONE_OPT \
    SIGFOX_EP_RC4_ZONE_OPT \
    SIGFOX_EP_RC5_ZONE_OPT \
    SIGFOX_EP_RC6_ZONE_OPT \
    SIGFOX_EP_RC7_ZONE_OPT \
    SIGFOX_EP_APPLICATION_MESSAGES_OPT \
    SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE_OPT \
    SIGFOX_EP_BIDIRECTIONAL_OPT \
    SIGFOX_EP_ASYNCHRONOUS_OPT \
    SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE_OPT \
    SIGFOX_EP_REGULATORY_OPT \
    SIGFOX_EP_LATENCY_COMPENSATION_OPT \
    SIGFOX_EP_SINGLE_FRAME_OPT \
    SIGFOX_EP_UL_BIT_RATE_BPS_OPT \
    SIGFOX_EP_T_IFU_MS_OPT \
    SIGFOX_EP_T_CONF_MS_OPT \
    SIGFOX_EP_UL_PAYLOAD_SIZE_OPT \
    SIGFOX_EP_AES_HW_OPT \
    SIGFOX_EP_CRC_HW_OPT \
    SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER_OPT \
    SIGFOX_EP_PARAMETERS_CHECK_OPT \
    SIGFOX_EP_CERTIFICATION_OPT \
    SIGFOX_EP_PUBLIC_KEY_CAPABLE_OPT \
    SIGFOX_EP_VERBOSE_OPT \
    SIGFOX_EP_ERROR_CODES_OPT \
    SIGFOX_EP_ERROR_STACK_OPT \

#endif /* __SIGFOX_EP_LIB_VERSION_H__ */
