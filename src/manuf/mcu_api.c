/*!*****************************************************************
 * \file    mcu_api.c
 * \brief   MCU drivers.
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

#include "manuf/mcu_api.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"

/*** MCU API functions ***/

#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
/*******************************************************************/
MCU_API_status_t MCU_API_open(MCU_API_config_t *mcu_api_config) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef LOW_LEVEL_OPEN_CLOSE
/*******************************************************************/
MCU_API_status_t MCU_API_close(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef ASYNCHRONOUS
/*******************************************************************/
MCU_API_status_t MCU_API_process(void) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef TIMER_REQUIRED
/*******************************************************************/
MCU_API_status_t MCU_API_timer_start(MCU_API_timer_t *timer) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef TIMER_REQUIRED
/*******************************************************************/
MCU_API_status_t MCU_API_timer_stop(MCU_API_timer_instance_t timer_instance) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
/*******************************************************************/
MCU_API_status_t MCU_API_timer_status(MCU_API_timer_instance_t timer_instance, sfx_bool *timer_has_elapsed) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
/*******************************************************************/
MCU_API_status_t MCU_API_timer_wait_cplt(MCU_API_timer_instance_t timer_instance) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

/*******************************************************************/
MCU_API_status_t MCU_API_aes_128_cbc_encrypt(MCU_API_encryption_data_t *aes_data) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}

#ifdef CRC_HW
/*******************************************************************/
MCU_API_status_t MCU_API_compute_crc16(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined CRC_HW) && (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_compute_crc8(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u8 *crc) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

/*******************************************************************/
MCU_API_status_t MCU_API_get_ep_id(sfx_u8 *ep_id, sfx_u8 ep_id_size_bytes) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}

/*******************************************************************/
MCU_API_status_t MCU_API_get_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}

/*******************************************************************/
MCU_API_status_t MCU_API_set_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}

#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle_mv, sfx_u16 *voltage_tx_mv, sfx_s16 *temperature_tenth_degrees) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined CERTIFICATION) && (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
MCU_API_status_t MCU_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION) && (defined BIDIRECTIONAL)
/*******************************************************************/
MCU_API_status_t MCU_API_get_latency(MCU_API_latency_t latency_type, sfx_u32 *latency_ms) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef VERBOSE
/*******************************************************************/
MCU_API_status_t MCU_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char) {
	/* To be implemented by the device manufacturer */
#ifdef ERROR_CODES
	MCU_API_status_t status = MCU_API_SUCCESS;
#endif
	RETURN();
}
#endif

#ifdef ERROR_CODES
/*******************************************************************/
void MCU_API_error(void) {
	/* To be implemented by the device manufacturer */
}
#endif
