/*!*****************************************************************
 * \file    sigfox_rc.h
 * \brief   Sigfox radio configuration zones definition.
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

#ifndef __SIGFOX_RC_H__
#define __SIGFOX_RC_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

#ifdef SPECTRUM_ACCESS_DC
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_DC
 * \brief Duty cycle spectrum access parameters.
 *******************************************************************/
static const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_DC = {
	.type = SIGFOX_SPECTRUM_ACCESS_TYPE_DC,
#ifdef BIDIRECTIONAL
	.dl_t_w_ms = 20000,
	.dl_t_rx_ms = 25000,
#ifndef SINGLE_FRAME
	.delta_f_mf_hz = 6000,
#endif
#endif
};
#endif

#ifdef SPECTRUM_ACCESS_LDC
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_LDC
 * \brief Low Duty cycle spectrum access parameters.
 *******************************************************************/
static const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_LDC = {
	.type = SIGFOX_SPECTRUM_ACCESS_TYPE_LDC,
#ifdef BIDIRECTIONAL
	.dl_t_w_ms = 19000,
	.dl_t_rx_ms = 33500,
#ifndef SINGLE_FRAME
	.delta_f_mf_hz = 6000,
#endif
#endif
};
#endif

#ifdef SPECTRUM_ACCESS_FH
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_FH
 * \brief Frequency hopping spectrum access definition.
 *******************************************************************/
static const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_FH = {
	.type = SIGFOX_SPECTRUM_ACCESS_TYPE_FH,
#ifdef BIDIRECTIONAL
	.dl_t_w_ms = 20000,
	.dl_t_rx_ms = 25000,
#ifndef SINGLE_FRAME
	.delta_f_mf_hz = SIGFOX_FH_MICRO_CHANNEL_WIDTH_HZ,
#endif
#endif
};
#endif

#ifdef SPECTRUM_ACCESS_LBT_M80
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_LBT_M80
 * \brief Listen before talk spectrum access -80dBm definition.
 *******************************************************************/
static const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_LBT_M80 = {
	.type = SIGFOX_SPECTRUM_ACCESS_TYPE_LBT,
#ifdef BIDIRECTIONAL
	.dl_t_w_ms = 19000,
	.dl_t_rx_ms = 33500,
#ifndef SINGLE_FRAME
	.delta_f_mf_hz = 6000,
#endif
#endif
#ifdef REGULATORY
	.cs_bandwidth_hz = 200000,
	.cs_threshold_dbm = -80,
	.cs_min_duration_ms = 5,
	.cs_max_duration_first_frame_ms = 5000, // Could be changed by the user.
#endif
};
#endif

#ifdef SPECTRUM_ACCESS_LBT_M65
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_LBT_M65
 * \brief Listen before talk spectrum access -65dBm definition.
 *******************************************************************/
static const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_LBT_M65 = {
	.type = SIGFOX_SPECTRUM_ACCESS_TYPE_LBT,
#ifdef BIDIRECTIONAL
	.dl_t_w_ms = 19000,
	.dl_t_rx_ms = 33500,
#ifndef SINGLE_FRAME
	.delta_f_mf_hz = 6000,
#endif
#endif
#ifdef REGULATORY
	.cs_bandwidth_hz = 200000,
	.cs_threshold_dbm = -65,
	.cs_min_duration_ms = 5,
	.cs_max_duration_first_frame_ms = 5000, // Could be changed by the user.
#endif
};
#endif

#ifdef RC1
// Sigfox network reference frequency inaccuracy in Hz (1.62ppm).
#define SIGFOX_EPSILON_SNW_RC1_HZ		1410
// End-point reference frequency inaccuracy in Hz (20ppm by default, could be changed according to effective device oscillator accuracy).
#define SIGFOX_EPSILON_EP_RC1_HZ		17370

/*!******************************************************************
 * \var SIGFOX_RC1
 * \brief RC1 radio configuration structure.
 *******************************************************************/
static const SIGFOX_rc_t SIGFOX_RC1 = {
	.f_ul_hz = 868130000,
#ifdef BIDIRECTIONAL
	.f_dl_hz = 869525000,
#endif
	.epsilon_hz = (SIGFOX_EPSILON_SNW_RC1_HZ + SIGFOX_EPSILON_EP_RC1_HZ),
	.spectrum_access = &SIGFOX_SPECTRUM_ACCESS_DC,
#ifdef PARAMETERS_CHECK
	.uplink_bit_rate_capability = (1 << SIGFOX_UL_BIT_RATE_100BPS) | (1 << SIGFOX_UL_BIT_RATE_600BPS),
	.tx_power_dbm_eirp_max = 16,
#endif
};
#endif

#ifdef RC2
/*!******************************************************************
 * \var SIGFOX_RC2
 * \brief RC2 radio configuration structure.
 *******************************************************************/
static const SIGFOX_rc_t SIGFOX_RC2 = {
	.f_ul_hz = 902200000,
#ifdef BIDIRECTIONAL
	.f_dl_hz = 905200000,
#endif
	.epsilon_hz = SIGFOX_FH_MACRO_CHANNEL_GUARD_BAND_HZ, // Fixed to 1 micro-channel bandwidth. Should not be changed.
	.spectrum_access = &SIGFOX_SPECTRUM_ACCESS_FH,
#ifdef PARAMETERS_CHECK
	.uplink_bit_rate_capability = (1 << SIGFOX_UL_BIT_RATE_600BPS),
	.tx_power_dbm_eirp_max = 24,
#endif
};
#endif

#ifdef RC3C
// Sigfox network reference frequency inaccuracy in Hz (1.62ppm).
#define SIGFOX_EPSILON_SNW_RC3C_HZ		1500
// End-point reference frequency inaccuracy in Hz (20ppm by default, could be changed according to effective device oscillator accuracy).
#define SIGFOX_EPSILON_EP_RC3C_HZ		18470

/*!******************************************************************
 * \var SIGFOX_RC3C
 * \brief RC3C radio configuration structure.
 *******************************************************************/
static const SIGFOX_rc_t SIGFOX_RC3C = {
	.f_ul_hz = 923200000,
#ifdef BIDIRECTIONAL
	.f_dl_hz = 922200000,
#endif
	.epsilon_hz = (SIGFOX_EPSILON_SNW_RC3C_HZ + SIGFOX_EPSILON_EP_RC3C_HZ),
	.spectrum_access = &SIGFOX_SPECTRUM_ACCESS_LBT_M80,
#ifdef PARAMETERS_CHECK
	.uplink_bit_rate_capability = (1 << SIGFOX_UL_BIT_RATE_100BPS) | (1 << SIGFOX_UL_BIT_RATE_600BPS),
	.tx_power_dbm_eirp_max = 16,
#endif
};
#endif

#ifdef RC3D
// Sigfox network reference frequency inaccuracy in Hz (1.62ppm).
#define SIGFOX_EPSILON_SNW_RC3D_HZ		1500
// End-point reference frequency inaccuracy in Hz (20ppm by default, could be changed according to effective device oscillator accuracy).
#define SIGFOX_EPSILON_EP_RC3D_HZ		18470

/*!******************************************************************
 * \var SIGFOX_RC3D
 * \brief RC3D radio configuration structure.
 *******************************************************************/
static const SIGFOX_rc_t SIGFOX_RC3D = {
	.f_ul_hz = 923200000,
#ifdef BIDIRECTIONAL
	.f_dl_hz = 922200000,
#endif
	.epsilon_hz = (SIGFOX_EPSILON_SNW_RC3D_HZ + SIGFOX_EPSILON_EP_RC3D_HZ),
	.spectrum_access = &SIGFOX_SPECTRUM_ACCESS_LDC,
#ifdef PARAMETERS_CHECK
	.uplink_bit_rate_capability = (1 << SIGFOX_UL_BIT_RATE_100BPS) | (1 << SIGFOX_UL_BIT_RATE_600BPS),
	.tx_power_dbm_eirp_max = 16,
#endif
};
#endif

#ifdef RC4
/*!******************************************************************
 * \var SIGFOX_RC4
 * \brief RC4 radio configuration structure.
 *******************************************************************/
static const SIGFOX_rc_t SIGFOX_RC4 = {
	.f_ul_hz = 920800000,
#ifdef BIDIRECTIONAL
	.f_dl_hz = 922300000,
#endif
	.epsilon_hz = SIGFOX_FH_MACRO_CHANNEL_GUARD_BAND_HZ, // Fixed to 1 micro-channel bandwidth. Should not be changed.
	.spectrum_access = &SIGFOX_SPECTRUM_ACCESS_FH,
#ifdef PARAMETERS_CHECK
	.uplink_bit_rate_capability = (1 << SIGFOX_UL_BIT_RATE_600BPS),
	.tx_power_dbm_eirp_max = 24,
#endif
};
#endif

#ifdef RC5
// Sigfox network reference frequency inaccuracy in Hz (1.62ppm).
#define SIGFOX_EPSILON_SNW_RC5_HZ		1500
// End-point reference frequency inaccuracy in Hz (20ppm by default, could be changed according to effective device oscillator accuracy).
#define SIGFOX_EPSILON_EP_RC5_HZ		18470

/*!******************************************************************
 * \var SIGFOX_RC5
 * \brief RC5 radio configuration structure.
 *******************************************************************/
static const SIGFOX_rc_t SIGFOX_RC5 = {
	.f_ul_hz = 923300000,
#ifdef BIDIRECTIONAL
	.f_dl_hz = 922300000,
#endif
	.epsilon_hz = (SIGFOX_EPSILON_SNW_RC5_HZ + SIGFOX_EPSILON_EP_RC5_HZ),
	.spectrum_access = &SIGFOX_SPECTRUM_ACCESS_LBT_M65,
#ifdef PARAMETERS_CHECK
	.uplink_bit_rate_capability = (1 << SIGFOX_UL_BIT_RATE_100BPS) | (1 << SIGFOX_UL_BIT_RATE_600BPS),
	.tx_power_dbm_eirp_max = 14,
#endif
};
#endif

#ifdef RC6
// Sigfox network reference frequency inaccuracy in Hz (1.62ppm).
#define SIGFOX_EPSILON_SNW_RC6_HZ		1410
// End-point reference frequency inaccuracy in Hz (20ppm by default, could be changed according to effective device oscillator accuracy).
#define SIGFOX_EPSILON_EP_RC6_HZ		17310

/*!******************************************************************
 * \var SIGFOX_RC6
 * \brief RC6 radio configuration structure.
 *******************************************************************/
static const SIGFOX_rc_t SIGFOX_RC6 = {
	.f_ul_hz = 865200000,
#ifdef BIDIRECTIONAL
	.f_dl_hz = 866300000,
#endif
	.epsilon_hz = (SIGFOX_EPSILON_SNW_RC6_HZ + SIGFOX_EPSILON_EP_RC6_HZ),
	.spectrum_access = &SIGFOX_SPECTRUM_ACCESS_DC,
#ifdef PARAMETERS_CHECK
	.uplink_bit_rate_capability = (1 << SIGFOX_UL_BIT_RATE_100BPS) | (1 << SIGFOX_UL_BIT_RATE_600BPS),
	.tx_power_dbm_eirp_max = 16,
#endif
};
#endif

#ifdef RC7
/*!******************************************************************
 * \var SIGFOX_RC7
 * \brief RC7 radio configuration structure.
 *******************************************************************/
// Sigfox network reference frequency inaccuracy in Hz (1.62ppm).
#define SIGFOX_EPSILON_SNW_RC7_HZ		1410
// End-point reference frequency inaccuracy in Hz (20ppm by default, could be changed according to effective device oscillator accuracy).
#define SIGFOX_EPSILON_EP_RC7_HZ		17380

static const SIGFOX_rc_t SIGFOX_RC7 = {
	.f_ul_hz = 868800000,
#ifdef BIDIRECTIONAL
	.f_dl_hz = 869100000,
#endif
	.epsilon_hz = (SIGFOX_EPSILON_SNW_RC7_HZ + SIGFOX_EPSILON_EP_RC7_HZ),
	.spectrum_access = &SIGFOX_SPECTRUM_ACCESS_DC,
#ifdef PARAMETERS_CHECK
	.uplink_bit_rate_capability = (1 << SIGFOX_UL_BIT_RATE_100BPS) | (1 << SIGFOX_UL_BIT_RATE_600BPS),
	.tx_power_dbm_eirp_max = 16,
#endif
};
#endif

#endif /* __SIGFOX_RC_H__ */
