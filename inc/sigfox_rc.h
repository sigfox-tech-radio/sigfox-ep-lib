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

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

/*** SIGFOX RC global variables ***/

#ifdef SIGFOX_EP_SPECTRUM_ACCESS_DC
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_DC
 * \brief Duty cycle spectrum access parameters.
 *******************************************************************/
extern const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_DC;
#endif

#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LDC
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_LDC
 * \brief Low Duty cycle spectrum access parameters.
 *******************************************************************/
extern const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_LDC;
#endif

#ifdef SIGFOX_EP_SPECTRUM_ACCESS_FH
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_FH
 * \brief Frequency hopping spectrum access definition.
 *******************************************************************/
extern const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_FH;
#endif

#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT_M80
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_LBT_M80
 * \brief Listen before talk spectrum access -80dBm definition.
 *******************************************************************/
extern const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_LBT_M80;
#endif

#ifdef SIGFOX_EP_SPECTRUM_ACCESS_LBT_M65
/*!******************************************************************
 * \var SIGFOX_SPECTRUM_ACCESS_LBT_M65
 * \brief Listen before talk spectrum access -65dBm definition.
 *******************************************************************/
extern const SIGFOX_spectrum_access_t SIGFOX_SPECTRUM_ACCESS_LBT_M65;
#endif

#ifdef SIGFOX_EP_RC1_ZONE
/*!******************************************************************
 * \var SIGFOX_RC1
 * \brief RC1 radio configuration structure.
 *******************************************************************/
extern const SIGFOX_rc_t SIGFOX_RC1;
#endif

#ifdef SIGFOX_EP_RC2_ZONE
/*!******************************************************************
 * \var SIGFOX_RC2
 * \brief RC2 radio configuration structure.
 *******************************************************************/
extern const SIGFOX_rc_t SIGFOX_RC2;
#endif

#ifdef SIGFOX_EP_RC3_LBT_ZONE
/*!******************************************************************
 * \var SIGFOX_RC3_LBT
 * \brief RC3 LBT radio configuration structure.
 *******************************************************************/
extern const SIGFOX_rc_t SIGFOX_RC3_LBT;
#endif

#ifdef SIGFOX_EP_RC3_LDC_ZONE
/*!******************************************************************
 * \var SIGFOX_RC3_LDC
 * \brief RC3 LDC radio configuration structure.
 *******************************************************************/
extern const SIGFOX_rc_t SIGFOX_RC3_LDC;
#endif

#ifdef SIGFOX_EP_RC4_ZONE
/*!******************************************************************
 * \var SIGFOX_RC4
 * \brief RC4 radio configuration structure.
 *******************************************************************/
extern const SIGFOX_rc_t SIGFOX_RC4;
#endif

#ifdef SIGFOX_EP_RC5_ZONE
/*!******************************************************************
 * \var SIGFOX_RC5
 * \brief RC5 radio configuration structure.
 *******************************************************************/
extern const SIGFOX_rc_t SIGFOX_RC5;
#endif

#ifdef SIGFOX_EP_RC6_ZONE
/*!******************************************************************
 * \var SIGFOX_RC6
 * \brief RC6 radio configuration structure.
 *******************************************************************/
extern const SIGFOX_rc_t SIGFOX_RC6;
#endif

#ifdef SIGFOX_EP_RC7_ZONE
/*!******************************************************************
 * \var SIGFOX_RC7
 * \brief RC7 radio configuration structure.
 *******************************************************************/
extern const SIGFOX_rc_t SIGFOX_RC7;
#endif

#endif /* __SIGFOX_RC_H__ */
