/*!*****************************************************************
 * \file    sigfox_types.h
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

#ifndef __SIGFOX_TYPES_H__
#define __SIGFOX_TYPES_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif

/*** SIGFOX TYPES types ***/

/*!******************************************************************
 * \typedef sfx_u8
 * \brief Sigfox 8-bits unsigned type.
 *******************************************************************/
typedef unsigned char	sfx_u8;

/*!******************************************************************
 * \typedef sfx_s8
 * \brief Sigfox 8-bits signed type.
 *******************************************************************/
typedef signed char		sfx_s8;

/*!******************************************************************
 * \typedef sfx_u16
 * \brief Sigfox 16-bits unsigned type.
 *******************************************************************/
typedef unsigned short	sfx_u16;

/*!******************************************************************
 * \typedef sfx_s16
 * \brief Sigfox 16-bits signed type.
 *******************************************************************/
typedef signed short	sfx_s16;

/*!******************************************************************
 * \typedef sfx_u32
 * \brief Sigfox 32-bits unsigned type.
 *******************************************************************/
typedef unsigned long	sfx_u32;

/*!******************************************************************
 * \typedef sfx_s32
 * \brief Sigfox 32-bits signed type.
 *******************************************************************/
typedef signed long		sfx_s32;

/*!******************************************************************
 * \enum sfx_bool
 * \brief Sigfox boolean type.
 *******************************************************************/
typedef enum {
	SFX_FALSE = 0,
	SFX_TRUE
} sfx_bool;

/*** SIGFOX TYPES second-level compilation flags definition ***/

#if (defined RC1_ZONE) || (defined RC6_ZONE) || (defined RC7_ZONE)
#define SPECTRUM_ACCESS_DC
#endif
#if (defined RC2_ZONE) || (defined RC4_ZONE)
#define SPECTRUM_ACCESS_FH
#endif
#if (defined RC3C_ZONE)
#define SPECTRUM_ACCESS_LBT
#define SPECTRUM_ACCESS_LBT_M80
#endif
#if (defined RC3D_ZONE)
#define SPECTRUM_ACCESS_LDC
#endif
#if (defined RC5_ZONE)
#define SPECTRUM_ACCESS_LBT
#define SPECTRUM_ACCESS_LBT_M65
#endif
#if (!(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0))) || (defined BIDIRECTIONAL) || (defined REGULATORY) || (defined CERTIFICATION)
#define TIMER_REQUIRED
#endif

/*** SIGFOX TYPES macros ***/

#define SFX_NULL											((void*) 0)
#define SFX_UNUSED(x)										((void) x)

// Sigfox credentials size.
#define SIGFOX_EP_ID_SIZE_BYTES								4
#define SIGFOX_EP_PAC_SIZE_BYTES							8
#define SIGFOX_EP_KEY_SIZE_BYTES							16

// Sigfox uplink payload maximum size.
#define SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES					12

// Sigfox uplink bitstream size.
#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL) || !(defined UL_PAYLOAD_SIZE)
#define SIGFOX_UL_BITSTREAM_SIZE_BYTES						26 // Maximum value used as default.
#else
#if (UL_PAYLOAD_SIZE == 0)
#define SIGFOX_UL_BITSTREAM_SIZE_BYTES						14
#elif (UL_PAYLOAD_SIZE == 1)
#define SIGFOX_UL_BITSTREAM_SIZE_BYTES						15
#elif (UL_PAYLOAD_SIZE == 2) || (UL_PAYLOAD_SIZE == 3) || (UL_PAYLOAD_SIZE == 4)
#define SIGFOX_UL_BITSTREAM_SIZE_BYTES						18
#elif (UL_PAYLOAD_SIZE == 5) || (UL_PAYLOAD_SIZE == 6) || (UL_PAYLOAD_SIZE == 7) || (UL_PAYLOAD_SIZE == 8)
#define SIGFOX_UL_BITSTREAM_SIZE_BYTES						22
#elif (UL_PAYLOAD_SIZE == 9) || (UL_PAYLOAD_SIZE == 10) || (UL_PAYLOAD_SIZE == 11) || (UL_PAYLOAD_SIZE == 12)
#define SIGFOX_UL_BITSTREAM_SIZE_BYTES						26
#else
#define SIGFOX_UL_BITSTREAM_SIZE_BYTES						26
#endif
#endif

#ifndef SINGLE_FRAME
// Maximum delay between each frame of a same uplink message (for all RC).
#define SIGFOX_T_IFU_MAX_MS									2000
#ifdef BIDIRECTIONAL
// Fixed inter-frame delay when requesting a downlink.
#define SIGFOX_T_IFB_MS										500
#endif
#ifdef SPECTRUM_ACCESS_LBT
// Maximum duration of repeated frames transmission in case of LBT.
#define SIGFOX_T_LF_MS										8000
#endif
#endif

#ifdef BIDIRECTIONAL
// Downlink bit rate.
#define SIGFOX_DL_BIT_RATE_BPS								600
// Downlink GFSK deviation.
#define SIGFOX_DL_GFSK_DEVIATION_HZ							800
// Downlink frame size.
#define SIGFOX_DL_PAYLOAD_SIZE_BYTES						8
#define SIGFOX_DL_PHY_CONTENT_SIZE_BYTES					15
// Downlink preamble.
#define SIGFOX_DL_PR_PATTERN								0xAA
// Downlink synchronization word.
#define SIGFOX_DL_FT										{0xB2, 0X27}
#define SIGFOX_DL_FT_SIZE_BYTES								2
// Delay between downlink frame reception and uplink confirmation control message (for all RC).
#define SIGFOX_T_CONF_MIN_MS								1400
#define SIGFOX_T_CONF_MAX_MS								4000
#endif

// Sigfox operational band width (for all RC).
#define SIGFOX_MACRO_CHANNEL_WIDTH_HZ						192000

#ifdef SPECTRUM_ACCESS_FH
// Sigfox frequency hopping macros.
#define SIGFOX_FH_MACRO_CHANNEL_NUMBER						9
#define SIGFOX_FH_MACRO_CHANNEL_WIDTH_HZ					200000
#define SIGFOX_FH_MACRO_CHANNEL_SPACING_HZ					300000
#define SIGFOX_FH_MACRO_CHANNEL_DELTA						(SIGFOX_FH_MACRO_CHANNEL_WIDTH_HZ - SIGFOX_MACRO_CHANNEL_WIDTH_HZ)

#define SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL			8
#define SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL_OPERATED	6
#define SIGFOX_FH_MICRO_CHANNEL_MASK						0x7E
#define SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ					(SIGFOX_FH_MACRO_CHANNEL_WIDTH_HZ / SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL)
#define SIGFOX_FH_MICRO_CHANNEL_GUARD_BAND_HZ				2000
#define SIGFOX_FH_MICRO_CHANNEL_OPERATED_WIDTH_HZ			(SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ - (2 * SIGFOX_FH_MICRO_CHANNEL_GUARD_BAND_HZ))
#define SIGFOX_FH_MICRO_CHANNEL_NUMBER						(SIGFOX_FH_MACRO_CHANNEL_NUMBER * SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL_OPERATED)

#define SIGFOX_FH_MACRO_CHANNEL_GUARD_BAND_HZ				(SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ - (SIGFOX_FH_MACRO_CHANNEL_DELTA / 2))

#define SIGFOX_FH_RC2_FIRST_MICRO_CHANNEL_FREQUENCY_HZ		902137500
#define SIGFOX_FH_RC2_OPERATED_MACRO_CHANNEL_INDEX			0

#define SIGFOX_FH_RC4_FIRST_MICRO_CHANNEL_FREQUENCY_HZ		920137500
#define SIGFOX_FH_RC4_OPERATED_MACRO_CHANNEL_INDEX			2

#define SIGFOX_FH_MICRO_CHANNEL_HOP 						((SIGFOX_FH_MACRO_CHANNEL_SPACING_HZ - SIGFOX_FH_MACRO_CHANNEL_WIDTH_HZ) + ((SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL - SIGFOX_FH_MICRO_CHANNEL_PER_MACRO_CHANNEL_OPERATED) * SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ))
#endif

/*** SIGFOX TYPES structures ***/

#if !(defined UL_BIT_RATE_BPS) || (defined PARAMETERS_CHECK)
/*!******************************************************************
 * \enum SIGFOX_bit_rate_t
 * \brief Sigfox signals bit rates list.
 *******************************************************************/
typedef enum {
    SIGFOX_UL_BIT_RATE_100BPS,
	SIGFOX_UL_BIT_RATE_600BPS,
	SIGFOX_UL_BIT_RATE_LAST
} SIGFOX_ul_bit_rate_t;
#endif

#ifdef APPLICATION_MESSAGES
/*!******************************************************************
 * \enum SIGFOX_application_message_type_t
 * \brief Sigfox application message types.
 *******************************************************************/
typedef enum {
#ifdef UL_PAYLOAD_SIZE
#if (UL_PAYLOAD_SIZE == 0)
	SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY,
	SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0,
	SIGFOX_APPLICATION_MESSAGE_TYPE_BIT1,
#else
	SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY,
#endif
#else
	SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY,
	SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0,
	SIGFOX_APPLICATION_MESSAGE_TYPE_BIT1,
    SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY,
#endif
	SIGFOX_APPLICATION_MESSAGE_TYPE_LAST
} SIGFOX_application_message_type_t;
#endif

#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
/*!******************************************************************
 * \enum SIGFOX_control_message_type_t
 * \brief Sigfox control message types.
 *******************************************************************/
typedef enum {
#ifdef CONTROL_KEEP_ALIVE_MESSAGE
	SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE,
#endif
#ifdef BIDIRECTIONAL
	SIGFOX_CONTROL_MESSAGE_TYPE_DL_CONFIRMATION,
#endif
	SIGFOX_CONTROL_MESSAGE_TYPE_LAST
} SIGFOX_control_message_type_t;
#endif

#ifndef SINGLE_FRAME
/*!******************************************************************
 * \enum SIGFOX_ul_frame_rank_t
 * \brief Sigfox uplink frame rank list.
 *******************************************************************/
typedef enum {
    SIGFOX_UL_FRAME_RANK_1 = 0,
    SIGFOX_UL_FRAME_RANK_2,
    SIGFOX_UL_FRAME_RANK_3,
	SIGFOX_UL_FRAME_RANK_LAST
} SIGFOX_ul_frame_rank_t;
#endif

#if !(defined MESSAGE_COUNTER_ROLLOVER) || (defined PARAMETERS_CHECK)
/*!******************************************************************
 * \enum SIGFOX_message_counter_rollover_t
 * \brief Sigfox message counter rollover values list.
 *******************************************************************/
typedef enum {
	SIGFOX_MESSAGE_COUNTER_ROLLOVER_128 = 0,
	SIGFOX_MESSAGE_COUNTER_ROLLOVER_256,
	SIGFOX_MESSAGE_COUNTER_ROLLOVER_512,
	SIGFOX_MESSAGE_COUNTER_ROLLOVER_1024,
	SIGFOX_MESSAGE_COUNTER_ROLLOVER_2048,
	SIGFOX_MESSAGE_COUNTER_ROLLOVER_4096,
	SIGFOX_MESSAGE_COUNTER_ROLLOVER_LAST
} SIGFOX_message_counter_rollover_t;
#endif

/*!******************************************************************
 * \enum SIGFOX_spectrum_access_type_t
 * \brief Spectrum access types list.
 *******************************************************************/
typedef enum {
#ifdef SPECTRUM_ACCESS_DC
	SIGFOX_SPECTRUM_ACCESS_TYPE_DC,
#endif
#ifdef SPECTRUM_ACCESS_LDC
	SIGFOX_SPECTRUM_ACCESS_TYPE_LDC,
#endif
#ifdef SPECTRUM_ACCESS_FH
	SIGFOX_SPECTRUM_ACCESS_TYPE_FH,
#endif
#ifdef SPECTRUM_ACCESS_LBT
	SIGFOX_SPECTRUM_ACCESS_TYPE_LBT,
#endif
	SIGFOX_SPECTRUM_ACCESS_TYPE_LAST
} SIGFOX_spectrum_access_type_t;

/*!******************************************************************
 * \struct SIGFOX_spectrum_access_t
 * \brief Spectrum access parameters structure.
 *******************************************************************/
typedef struct {
	SIGFOX_spectrum_access_type_t type;
#ifdef BIDIRECTIONAL
	// Common parameters.
	sfx_u16 dl_t_w_ms;
	sfx_u16 dl_t_rx_ms;
#ifndef SINGLE_FRAME
	sfx_u16 delta_f_mf_hz;
#endif
#endif
#if (defined SPECTRUM_ACCESS_LBT) && (defined REGULATORY)
	// For LBT.
	sfx_u32 cs_bandwidth_hz;
	sfx_s8 cs_threshold_dbm;
	sfx_u32 cs_min_duration_ms;
	sfx_u32 cs_max_duration_first_frame_ms;
#endif
} SIGFOX_spectrum_access_t;

/*!******************************************************************
 * \struct SIGFOX_rc_t
 * \brief Sigfox radio configuration structure.
 *******************************************************************/
typedef struct {
    sfx_u32 f_ul_hz;
#ifdef BIDIRECTIONAL
    sfx_u32 f_dl_hz;
#endif
    sfx_u16 epsilon_hz;
#ifdef PARAMETERS_CHECK
    sfx_u8 uplink_bit_rate_capability;
    sfx_s8 tx_power_dbm_eirp_max;
#endif
    const SIGFOX_spectrum_access_t *spectrum_access;
} SIGFOX_rc_t;

/*!******************************************************************
 * \enum SIGFOX_nvm_data_index_t
 * \brief Sigfox NVM fields index.
 *******************************************************************/
typedef enum {
	SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_LSB = 0,
	SIGFOX_NVM_DATA_INDEX_RANDOM_VALUE_MSB,
	SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_LSB,
	SIGFOX_NVM_DATA_INDEX_MESSAGE_COUNTER_MSB,
	SIGFOX_NVM_DATA_SIZE_BYTES
} SIGFOX_nvm_data_index_t;

#ifdef PUBLIC_KEY_CAPABLE
/*!******************************************************************
 * \enum SIGFOX_ep_key_t
 * \brief Sigfox end-point key type.
 *******************************************************************/
typedef enum {
	SIGFOX_EP_KEY_PRIVATE,
	SIGFOX_EP_KEY_PUBLIC,
	SIGFOX_EP_KEY_LAST
} SIGFOX_ep_key_t;
#endif

#ifdef VERBOSE
/*!******************************************************************
 * \enum SIGFOX_version_t
 * \brief Sigfox EP library components version.
 *******************************************************************/
typedef enum {
	SIGFOX_VERSION_EP_LIBRARY = 0,
	SIGFOX_VERSION_MCU_DRIVER,
	SIGFOX_VERSION_RF_DRIVER,
	SIGFOX_VERSION_LAST
} SIGFOX_version_t;
#endif

/*** SIGFOX TYPES global variables ***/

#if !(defined UL_BIT_RATE_BPS) || ((defined PARAMETERS_CHECK) && (defined ERROR_CODES))
/*!******************************************************************
 * \var SIGFOX_EP_API_UL_BIT_RATE_BPS_LIST
 * \brief Sigfox bit rates value.
 *******************************************************************/
extern const sfx_u16 SIGFOX_UL_BIT_RATE_BPS_LIST[SIGFOX_UL_BIT_RATE_LAST];
#endif

#if !(defined MESSAGE_COUNTER_ROLLOVER) || (defined PARAMETERS_CHECK)
/*!******************************************************************
 * \var SIGFOX_EP_API_MESSAGE_COUNTER_ROLLOVER_LIST
 * \brief Sigfox message counter value.
 *******************************************************************/
extern const sfx_u16 SIGFOX_MESSAGE_COUNTER_ROLLOVER_LIST[SIGFOX_MESSAGE_COUNTER_ROLLOVER_LAST];
#endif

#ifdef CERTIFICATION
/*!******************************************************************
 * \var SIGFOX_EP_TEST_ID
 * \var SIGFOX_EP_TEST_KEY
 * \brief End-point IDs and keys used for test.
 *******************************************************************/
extern const sfx_u8 SIGFOX_EP_TEST_ID[SIGFOX_EP_ID_SIZE_BYTES];
extern const sfx_u8 SIGFOX_EP_TEST_KEY[SIGFOX_EP_KEY_SIZE_BYTES];
#endif

#ifdef PUBLIC_KEY_CAPABLE
/*!******************************************************************
 * \var SIGFOX_EP_PUBLIC_KEY
 * \brief End-point public key used for test.
 *******************************************************************/
extern const sfx_u8 SIGFOX_EP_PUBLIC_KEY[SIGFOX_EP_KEY_SIZE_BYTES];
#endif

/*** SIGFOX TYPES unwanted flag combinations and values ***/

#if !(defined RC1_ZONE) && !(defined RC2_ZONE) && !(defined RC3C_ZONE) && !(defined RC3D_ZONE) && !(defined RC4_ZONE) && !(defined RC5_ZONE) && !(defined RC6_ZONE) && !(defined RC7_ZONE)
#error "SIGFOX EP LIB flags error: None RC defined"
#endif
#if !(defined APPLICATION_MESSAGES) && !(defined CONTROL_KEEP_ALIVE_MESSAGE)
#error "SIGFOX EP LIB flags error: None message type defined"
#endif
#if !(defined APPLICATION_MESSAGES) && (defined BIDIRECTIONAL)
#error "SIGFOX EP LIB flags error: Bidirectional communication only applies when APPLICATION_MESSAGES is enabled"
#endif
#if (defined UL_BIT_RATE_BPS) && (UL_BIT_RATE_BPS != 100) && (UL_BIT_RATE_BPS != 600)
#error "SIGFOX EP LIB flags error: Invalid UL_BIT_RATE_BPS value"
#endif
#if (defined T_IFU_MS) && (defined SINGLE_FRAME)
#error "SIGFOX EP LIB flags error: T_IFU_MS only applies when SINGLE_FRAME is disabled"
#endif
#ifndef SINGLE_FRAME
#if (defined T_IFU_MS) && (T_IFU_MS > SIGFOX_T_IFU_MAX_MS)
#error "SIGFOX EP LIB flags error: Invalid T_IFU_MS value"
#endif
#endif
#if (defined T_CONF_MS) && !(defined BIDIRECTIONAL)
#error "SIGFOX EP LIB flags error: T_CONF_MS only applies when BIDIRECTIONAL is enabled"
#endif
#ifdef BIDIRECTIONAL
#if (defined T_CONF_MS) && ((T_CONF_MS < SIGFOX_T_CONF_MIN_MS) || (T_CONF_MS > SIGFOX_T_CONF_MAX_MS))
#error "SIGFOX EP LIB flags error: Invalid T_CONF_MS value"
#endif
#endif
#if (defined UL_PAYLOAD_SIZE) && (UL_PAYLOAD_SIZE > SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES)
#error "SIGFOX EP LIB flags error: Unsupported UL_PAYLOAD_SIZE value"
#endif
#if (defined MESSAGE_COUNTER_ROLLOVER) && (MESSAGE_COUNTER_ROLLOVER != 128) && (MESSAGE_COUNTER_ROLLOVER != 256) && (MESSAGE_COUNTER_ROLLOVER != 512) && (MESSAGE_COUNTER_ROLLOVER != 1024) && (MESSAGE_COUNTER_ROLLOVER != 2048) && (MESSAGE_COUNTER_ROLLOVER != 4096)
#error "SIGFOX EP LIB flags error: Invalid MESSAGE_COUNTER_ROLLOVER value"
#endif
#if (defined ERROR_STACK) && !(defined ERROR_CODES)
#error "SIGFOX EP LIB flags error: ERROR_CODES are required to use ERROR_STACK"
#endif

#endif /* __SIGFOX_TYPES_H__ */
