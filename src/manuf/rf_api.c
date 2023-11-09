/*!*****************************************************************
 * \file    rf_api.c
 * \brief   Radio drivers.
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

#include "manuf/rf_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

/*** RF API functions ***/

#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
RF_API_status_t RF_API_open(RF_API_config_t *rf_api_config) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
RF_API_status_t RF_API_close(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
RF_API_status_t RF_API_process(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}
#endif

/*******************************************************************/
RF_API_status_t RF_API_wake_up(void) {
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_sleep(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_init(RF_API_radio_parameters_t *radio_parameters) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_de_init(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}

/*******************************************************************/
RF_API_status_t RF_API_send(RF_API_tx_data_t *tx_data) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}

#ifdef BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_receive(RF_API_rx_data_t *rx_data) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef BIDIRECTIONAL
/*******************************************************************/
RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
/*******************************************************************/
RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION)
/*******************************************************************/
RF_API_status_t RF_API_get_latency(RF_API_latency_t latency_type, sfx_u32 *latency_ms) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef CERTIFICATION
/*******************************************************************/
RF_API_status_t RF_API_start_continuous_wave(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	RF_API_status_t status = RF_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef ERROR_CODES
/*******************************************************************/
void RF_API_error(void) {
	/* To be implemented by the device manufacturer */
}
#endif
