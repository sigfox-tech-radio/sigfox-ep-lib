/*!*****************************************************************
 * \file    sigfox_ep_flags.h
 * \brief   Sigfox End-Point library compilations flags definition.
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

#ifndef __SIGFOX_EP_FLAGS_H__
#define __SIGFOX_EP_FLAGS_H__

/*** Library compilation flags ***/

/*!******************************************************************
 * \def SIGFOX_EP_RC1_ZONE
 * \brief Support radio configuration zone 1 (Europe, Middle-East and Africa).
 *******************************************************************/
#define SIGFOX_EP_RC1_ZONE

/*!******************************************************************
 * \def SIGFOX_EP_RC2_ZONE
 * \brief Support radio configuration zone 2 (Brazil, Canada, Mexico, Puerto Rico and USA).
 *******************************************************************/
#define SIGFOX_EP_RC2_ZONE

/*!******************************************************************
 * \def SIGFOX_EP_RC3_LBT_ZONE
 * \brief Support radio configuration zone 3 (Japan) with LBT.
 *******************************************************************/
#define SIGFOX_EP_RC3_LBT_ZONE

/*!******************************************************************
 * \def SIGFOX_EP_RC3_LDC_ZONE
 * \brief Support radio configuration zone 3 (Japan) with LDC.
 *******************************************************************/
#define SIGFOX_EP_RC3_LDC_ZONE

/*!******************************************************************
 * \def SIGFOX_EP_RC4_ZONE
 * \brief Support radio configuration zone 4 (Latin America and Asia Pacific).
 *******************************************************************/
#define SIGFOX_EP_RC4_ZONE

/*!******************************************************************
 * \def SIGFOX_EP_RC5_ZONE
 * \brief Support radio configuration zone 5 (South-Corea).
 *******************************************************************/
#define SIGFOX_EP_RC5_ZONE

/*!******************************************************************
 * \def SIGFOX_EP_RC6_ZONE
 * \brief Support radio configuration zone 6 (India).
 *******************************************************************/
#define SIGFOX_EP_RC6_ZONE

/*!******************************************************************
 * \def SIGFOX_EP_RC7_ZONE
 * \brief Support radio configuration zone 7 (Russia).
 *******************************************************************/
#define SIGFOX_EP_RC7_ZONE

/*!******************************************************************
 * \def SIGFOX_EP_APPLICATION_MESSAGES
 * \brief Support uplink application messages if defined.
 *******************************************************************/
#define SIGFOX_EP_APPLICATION_MESSAGES

/*!******************************************************************
 * \def SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
 * \brief Support uplink control keep alive message if defined.
 *******************************************************************/
#define SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE

/*!******************************************************************
 * \def SIGFOX_EP_BIDIRECTIONAL
 * \brief Support bidirectional procedure (downlink) if defined. Only applicable to application messages. Otherwise all messages will be uplink only.
 *******************************************************************/
#define SIGFOX_EP_BIDIRECTIONAL

/*!******************************************************************
 * \def SIGFOX_EP_ASYNCHRONOUS
 * \brief Asynchronous mode if defined, blocking mode otherwise.
 *******************************************************************/
#define SIGFOX_EP_ASYNCHRONOUS

/*!******************************************************************
 * \def SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE
 * \brief Enable MCU and RF open/close functions if defined.
 *******************************************************************/
#define SIGFOX_EP_LOW_LEVEL_OPEN_CLOSE

/*!******************************************************************
 * \def SIGFOX_EP_REGULATORY
 * \brief Enable radio regulatory control (DC, FH or LBT check) if defined.
 *******************************************************************/
#define SIGFOX_EP_REGULATORY

/*!******************************************************************
 * \def SIGFOX_EP_LATENCY_COMPENSATION
 * \brief If defined, enable radio latency compensation to improve MCU timers accuracy.
 *******************************************************************/
#define SIGFOX_EP_LATENCY_COMPENSATION

/*!******************************************************************
 * \def SIGFOX_EP_SINGLE_FRAME
 * \brief Send 1 frame per message (N=1) if defined. Otherwise number of frames per message is dynamically given when sending a message (N=1, N=2 or N=3).
 *******************************************************************/
//#define SIGFOX_EP_SINGLE_FRAME

/*!******************************************************************
 * \def SIGFOX_EP_UL_BIT_RATE_BPS
 * \brief If defined, give the only uplink bit rate supported (100 or 600 depending on the RC). Otherwise, value is dynamically given when sending a message.
 *******************************************************************/
//#define SIGFOX_EP_UL_BIT_RATE_BPS             100

/*!******************************************************************
 * \def SIGFOX_EP_TX_POWER_DBM_EIRP
 * \brief If defined, give the only TX power supported by the radio. Otherwise the value is dynamically given when sending a message.
 *******************************************************************/
//#define SIGFOX_EP_TX_POWER_DBM_EIRP           14

/*!******************************************************************
 * \def SIGFOX_EP_T_IFU_MS
 * \brief If defined, give the fixed inter-frame delay used between uplink frames of a same message (0 to 2000ms). Otherwise value is dynamically given when sending a message.
 * \brief Value 0 disables the delay and associated timers to optimize memory space.
 *******************************************************************/
//#define SIGFOX_EP_T_IFU_MS                    500

/*!******************************************************************
 * \def SIGFOX_EP_T_CONF_MS
 * \brief If defined, give the fixed delay between downlink frame reception and uplink confirmation message (1400 to 4000ms). Otherwise value is dynamically given when sending a message.
 *******************************************************************/
//#define SIGFOX_EP_T_CONF_MS                   2000

/*!******************************************************************
 * \def SIGFOX_EP_UL_PAYLOAD_SIZE
 * \brief If defined, give the only uplink payload length supported (0 to 12). Otherwise, all uplink payload lengths are dynamically supported.
 * \brief Value 0 enables the bit 0, bit 1 and empty messages.
 *******************************************************************/
//#define SIGFOX_EP_UL_PAYLOAD_SIZE             0

/*!******************************************************************
 * \def SIGFOX_EP_AES_HW
 * \brief If defined, enable hardware AES through MCU API function. Otherwise the embedded driver from TI is used.
 *******************************************************************/
#define SIGFOX_EP_AES_HW

/*!******************************************************************
 * \def SIGFOX_EP_CRC_HW
 * \brief If defined, enable hardware CRC through MCU API functions. Otherwise the embedded driver is used.
 *******************************************************************/
//#define SIGFOX_EP_CRC_HW

/*!******************************************************************
 * \def SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
 * \brief If defined, give the only message counter rollover value supported. Otherwise, value is dynamically given when opening the library.
 *******************************************************************/
//#define SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER    4096

/*!******************************************************************
 * \def SIGFOX_EP_PARAMETERS_CHECK
 * \brief Enable parameters check if defined.
 *******************************************************************/
#define SIGFOX_EP_PARAMETERS_CHECK

/*!******************************************************************
 * \def SIGFOX_EP_CERTIFICATION
 * \brief Enable certification features if defined.
 *******************************************************************/
#define SIGFOX_EP_CERTIFICATION

/*!******************************************************************
 * \def SIGFOX_EP_PUBLIC_KEY_CAPABLE
 * \brief Enable public key switch feature if defined.
 *******************************************************************/
#define SIGFOX_EP_PUBLIC_KEY_CAPABLE

/*!******************************************************************
 * \def SIGFOX_EP_VERBOSE
 * \brief Enable credentials (ID/PAC) API access and version control functions if defined.
 *******************************************************************/
#define SIGFOX_EP_VERBOSE

/*!******************************************************************
 * \def SIGFOX_EP_ERROR_CODES
 * \brief Use return codes if defined, otherwise all functions return void.
 *******************************************************************/
#define SIGFOX_EP_ERROR_CODES

/*!******************************************************************
 * \def SIGFOX_EP_ERROR_STACK
 * \brief If defined, store low level errors in a stack (the macro gives the depth). Errors can be read with the SIGFOX_EP_API_unstack_error() function.
 *******************************************************************/
#define SIGFOX_EP_ERROR_STACK                   32

#endif /* __SIGFOX_EP_FLAGS_H__ */
