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

#define RC1									/*!< \brief Support radio configuration zone 1 (Europe, Middle-East and Africa). > */
#define RC2									/*!< \brief Support radio configuration zone 2 (Brazil, Canada, Mexico, Puerto Rico and USA). > */
#define RC3C								/*!< \brief Support radio configuration zone 3 (Japan) with LBT. > */
#define RC3D								/*!< \brief Support radio configuration zone 3 (Japan) with DC. */
#define RC4									/*!< \brief Support radio configuration zone 4 (Latin America and Asia Pacific). > */
#define RC5									/*!< \brief Support radio configuration zone 5 (South-Corea). > */
#define RC6									/*!< \brief Support radio configuration zone 6 (India). > */
#define RC7									/*!< \brief Support radio configuration zone 7 (Russia). > */

#define APPLICATION_MESSAGES				/*!< \brief Support uplink application messages if defined. > */
#define CONTROL_KEEP_ALIVE_MESSAGE			/*!< \brief Support uplink control keep alive message if defined. > */

#define BIDIRECTIONAL						/*!< \brief Support bidirectional procedure (downlink) if defined. Only applicable to application messages. > */
											/*!< \brief Otherwise all messages will be uplink only. > */

#define ASYNCHRONOUS						/*!< \brief Asynchronous mode if defined, blocking mode otherwise. > */

#define LOW_LEVEL_OPEN_CLOSE				/*!< \brief Enable MCU and RF open/close functions if defined. > */

#define REGULATORY							/*!< \brief Enable radio regulatory control (DC, FH or LBT check) if defined. > */

//#define SINGLE_FRAME						/*!< \brief Send 1 frame per message (N=1) if defined. > */
											/*!< \brief Otherwise number of frames per message is dynamically given when sending a message (N=1, N=2 or N=3). > */

//#define UL_BIT_RATE_BPS			100		/*!< \brief If defined, give the only uplink bit rate supported (100 or 600 depending on the RC). > */
											/*!< \brief Otherwise, value is dynamically given when sending a message. > */

//#define TX_POWER_DBM_EIRP			16		/*!< \brief If defined, give the only TX power supported by the radio. > */
											/*!< \brief Otherwise the value is dynamically given when sending a message. > */

//#define T_IFU_MS					500		/*!< \brief If defined, give the fixed inter-frame delay used between uplink frames of a same message (0 to 2000ms). > */
											/*!< \brief Value 0 disables the delay and associated timers to optimize memory space. > */
											/*!< \brief Otherwise value is dynamically given when sending a message. > */

//#define T_CONF_MS					2000	/*!< \brief If defined, give the fixed delay between downlink frame reception and uplink confirmation message (1400 to 4000ms). > */
											/*!< \brief Otherwise value is dynamically given when sending a message. > */

//#define UL_PAYLOAD_SIZE			0		/*!< \brief If defined, give the only uplink payload length supported (0 to 12). > */
											/*!< \brief Value 0 enables the bit 0, bit 1 and empty messages. > */
											/*!< \brief Otherwise, all uplink payload lengths are dynamically supported. > */

//#define CRC_HW							/*!< \brief If defined, enable hardware CRC through MCU API functions. Otherwise the embedded driver is used. > */

//#define MESSAGE_COUNTER_ROLLOVER	4096	/*!< \brief If defined, give the only message counter rollover value supported. > */
											/*!< \brief Otherwise, value is dynamically given when opening the library. > */

#define PARAMETERS_CHECK					/*!< \brief Enable parameters check if defined. > */

#define CERTIFICATION						/*!< \brief Enable certification features if defined. > */

#define PUBLIC_KEY_CAPABLE					/*!< \brief Enable public key switch feature if defined. > */

#define VERBOSE								/*!< \brief Enable credentials (ID/PAC) API access and version control functions if defined. > */

#define ERROR_CODES							/*!< \brief Use return codes if defined, otherwise all functions return void. > */

#define ERROR_STACK					32		/*!< \brief If defined, store low level errors in a stack (the macro gives the depth). > */
											/*!< \brief Errors can be read with the SIGFOX_EP_API_unstack_error() function. > */

#endif /* __SIGFOX_EP_FLAGS_H__ */
