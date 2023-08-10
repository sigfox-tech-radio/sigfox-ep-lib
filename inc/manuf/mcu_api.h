/*!*****************************************************************
 * \file    mcu_api.h
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

#ifndef __MCU_API_H__
#define __MCU_API_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

/*** MCU API macros ***/

#ifdef TIMER_REQUIRED
// Timer instances mapping.
#if !(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0) || (defined BIDIRECTIONAL))
#define MCU_API_TIMER_INSTANCE_T_IFX		MCU_API_TIMER_1
#endif
#ifdef BIDIRECTIONAL
#define MCU_API_TIMER_INSTANCE_T_CONF		MCU_API_TIMER_1
#define MCU_API_TIMER_INSTANCE_T_W			MCU_API_TIMER_2
#define MCU_API_TIMER_INSTANCE_T_RX			MCU_API_TIMER_1
#endif
#ifdef REGULATORY
#define MCU_API_TIMER_INSTANCE_FH			MCU_API_TIMER_1
#define MCU_API_TIMER_INSTANCE_LBT			MCU_API_TIMER_1
#endif
#ifdef CERTIFICATION
#define MCU_API_TIMER_INSTANCE_ADDON_RFP	MCU_API_TIMER_3
#endif
#endif /* TIMER_REQUIRED */

/*** MCU API structures ***/

#ifdef ERROR_CODES
/*!******************************************************************
 * \enum MCU_API_status_t
 * \brief MCU driver error codes.
 *******************************************************************/
typedef enum {
    MCU_API_SUCCESS = 0,
	MCU_API_ERROR,
	// Additional custom error codes can be added here (up to sfx_u32).
	// They will be logged in the library error stack if the ERROR_STACK flag is defined.
	// Last index.
	MCU_API_ERROR_LAST
} MCU_API_status_t;
#else
typedef void MCU_API_status_t;
#endif

#ifdef ASYNCHRONOUS
/********************************
 * \brief MCU driver callback functions.
 * \fn MCU_API_process_cb_t		To be called when the MCU driver needs to be processed.
 * \fn MCU_API_error_cb_t		To be called when an error occurs during MCU operation.
 * \fn MCU_API_timer_cplt_cb_t	To be called when a timer elapses.
 *******************************/
typedef void (*MCU_API_process_cb_t)(void);
#ifdef ERROR_CODES
typedef void (*MCU_API_error_cb_t)(MCU_API_status_t status);
#else
typedef void (*MCU_API_error_cb_t)(void);
#endif
#if (!(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0))) || (defined BIDIRECTIONAL) || (defined REGULATORY) || (defined CERTIFICATION)
typedef void (*MCU_API_timer_cplt_cb_t)(void);
#endif
#endif

#ifdef TIMER_REQUIRED
/*!******************************************************************
 * \enum MCU_API_timer_instance_t
 * \brief MCU timer instances.
 *******************************************************************/
typedef enum {
#if (!(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0))) || (defined BIDIRECTIONAL) || (defined REGULATORY)
	MCU_API_TIMER_1,
#endif
#ifdef BIDIRECTIONAL
	MCU_API_TIMER_2,
#endif
#ifdef CERTIFICATION
	MCU_API_TIMER_3,
#endif
	MCU_API_TIMER_LAST
} MCU_API_timer_instance_t;
#endif

#ifdef TIMER_REQUIRED
/*!******************************************************************
 * \enum MCU_API_timer_mapping_t
 * \brief MCU timer instances mapping.
 *******************************************************************/
typedef enum {
#if !(defined SINGLE_FRAME) && (!(defined T_IFU_MS) || (T_IFU_MS > 0) || (defined BIDIRECTIONAL))
	MCU_API_TIMER_REASON_T_IFX,
#endif
#ifdef BIDIRECTIONAL
	MCU_API_TIMER_REASON_T_CONF,
	MCU_API_TIMER_REASON_T_W,
	MCU_API_TIMER_REASON_T_RX,
#endif
#ifdef REGULATORY
	MCU_API_TIMER_REASON_FH,
	MCU_API_TIMER_REASON_LBT,
#endif
#ifdef CERTIFICATION
	MCU_API_TIMER_REASON_ADDON_RFP,
#endif
	MCU_API_TIMER_REASON_LAST
} MCU_API_timer_reason_t;
#endif

#ifdef TIMER_REQUIRED
/*!******************************************************************
 * \struct MCU_API_timer_t
 * \brief MCU API timer structure.
 *******************************************************************/
typedef struct {
	MCU_API_timer_instance_t instance;
	MCU_API_timer_reason_t reason;
	sfx_u32 duration_ms;
#ifdef ASYNCHRONOUS
	MCU_API_timer_cplt_cb_t cplt_cb;
#endif
} MCU_API_timer_t;
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION) && (defined BIDIRECTIONAL)
/*!******************************************************************
 * \enum MCU_API_latency_t
 * \brief MCU latency sources.
 *******************************************************************/
typedef enum {
	MCU_API_LATENCY_GET_VOLTAGE_TEMPERATURE = 0,
	MCU_API_LATENCY_LAST
} MCU_API_latency_t;
#endif

/*!******************************************************************
 * \struct MCU_API_encryption_data_t
 * \brief MCU API encryption data structure.
 *******************************************************************/
typedef struct {
	sfx_u8 *data;
	sfx_u8 data_size_bytes;
#ifdef PUBLIC_KEY_CAPABLE
	SIGFOX_ep_key_t key;
#endif
} MCU_API_encryption_data_t;

/*!******************************************************************
 * \struct MCU_API_config_t
 * \brief MCU API configuration structure.
 *******************************************************************/
typedef struct {
	const SIGFOX_rc_t *rc;
#ifdef ASYNCHRONOUS
	MCU_API_process_cb_t process_cb;
	MCU_API_error_cb_t error_cb;
#endif
} MCU_API_config_t;

/*** MCU API functions ***/

#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_open(MCU_API_config_t *mcu_api_config)
 * \brief Open the MCU driver.
 * \param[in]  	mcu_api_config: Pointer to the MCU API configuration.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_open(MCU_API_config_t *mcu_api_config);
#endif

#ifdef LOW_LEVEL_OPEN_CLOSE
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_close(void)
 * \brief Close the MCU driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_close(void);
#endif

#ifdef ASYNCHRONOUS
/*!******************************************************************
 * \fn void MCU_API_process(void)
 * \brief Process MCU driver, this function will be call by SIGFOX_EP_API_process just after the process_callback has been sent to process MCU interruptions in main context.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_process(void);
#endif

#ifdef TIMER_REQUIRED
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_timer_start(MCU_API_timer_t *timer)
 * \brief Start a timer. Timer completion should be notified by calling the given cplt_cb() function.
 * \param[in]	timer: Pointer to the timer structure to start.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_timer_start(MCU_API_timer_t *timer);
#endif

#ifdef TIMER_REQUIRED
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_timer_stop(MCU_API_timer_instance_t timer_instance)
 * \brief Stop a timer.
 * \param[in]	timer_instance: Timer to stop.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_timer_stop(MCU_API_timer_instance_t timer_instance);
#endif

#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_timer_status(MCU_API_timer_instance_t timer_instance, sfx_bool *timer_has_elapsed)
 * \brief Get timer status.
 * \brief This functions is never called by the core library: it is provided to manage the timeout in the RF_API_receive() function in blocking mode.
 * \param[in]	timer_instance: Timer to read.
 * \param[out] 	timer_has_elapsed: Pointer to boolean variable that will contain the timer status.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_timer_status(MCU_API_timer_instance_t timer_instance, sfx_bool *timer_has_elapsed);
#endif

#if (defined TIMER_REQUIRED) && !(defined ASYNCHRONOUS)
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_timer_wait_cplt(MCU_API_timer_instance_t timer_instance)
 * \brief Blocking function waiting for timer completion.
 * \param[in]	timer_instance: Timer to wait for.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_timer_wait_cplt(MCU_API_timer_instance_t timer_instance);
#endif

/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_aes_128_cbc_encrypt(MCU_API_encryption_data_t *aes_data)
 * \brief Function performing the AES-128 encryption algorithm with specified key (for uplink UL-AUTH field computing and downlink frame authentication).
 * \param[in]  	aes_data: AES data structure.
 * \param[out] 	none.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_aes_128_cbc_encrypt(MCU_API_encryption_data_t *aes_data);

#ifdef CRC_HW
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_compute_crc16(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc)
 * \brief Compute a CRC16.
 * \param[in]  	data: Input data.
 * \param[in]	data_size: Number of bytes of the input data.
 * \param[in]	polynom: CRC polynom to use.
 * \param[out] 	crc: Pointer to the computed CRC16 value.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_compute_crc16(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u16 *crc);
#endif

#if (defined CRC_HW) && (defined BIDIRECTIONAL)
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_compute_crc8(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u8 *crc)
 * \brief Compute a CRC8.
 * \param[in]  	data: Input data.
 * \param[in]	data_size: Number of bytes of the input data.
 * \param[in]	polynom: CRC polynom to use.
 * \param[out] 	crc: Pointer to the computed CRC8 value.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_compute_crc8(sfx_u8 *data, sfx_u8 data_size, sfx_u16 polynom, sfx_u8 *crc);
#endif

/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_get_ep_id(sfx_u8 *ep_id, sfx_u8 ep_id_size_bytes)
 * \brief Get end-point ID.
 * \param[in]  	ep_id_size_bytes: Number of bytes of the ID to read.
 * \param[out] 	ep_id: End-point ID.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_get_ep_id(sfx_u8 *ep_id, sfx_u8 ep_id_size_bytes);

/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_get_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes)
 * \brief Read NVM data.
 * \param[in]  	nvm_data_size_bytes: Number of bytes to read.
 * \param[out] 	nvm_data: Read NVM data.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_get_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes);

/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_set_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes)
 * \brief Write NVM data.
 * \param[in] 	nvm_data: NVM data to write.
 * \param[in]  	nvm_data_size_bytes: Number of bytes to write.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_set_nvm(sfx_u8 *nvm_data, sfx_u8 nvm_data_size_bytes);

#if (defined CONTROL_KEEP_ALIVE_MESSAGE) || (defined BIDIRECTIONAL)
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle_mv, sfx_u16 *voltage_tx_mv, sfx_s16 *temperature_tenth_degrees)
 * \brief Get voltage and temperature measurements (used in control message payload).
 * \param[out] 	voltage_idle_mv: Device power supply voltage in mV measured in idle state.
 * \param[out] 	voltage_tx_mv: Device power supply voltage in mV measured during TX operation.
 * \param[out] 	temperature_tenth_degrees: Device temperature in 1/10 degrees.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_get_voltage_temperature(sfx_u16 *voltage_idle_mv, sfx_u16 *voltage_tx_mv, sfx_s16 *temperature_tenth_degrees);
#endif

#ifdef CERTIFICATION
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm)
 * \brief Print a downlink frame (only used by the RFP addon during downlink test modes).
 * \param[in]  	dl_payload: Downlink payload to print.
 * \param[in] 	dl_payload_size: Number of bytes to print.
 * \param[in]	rssi_dbm: RSSI of the received downlink frame (16-bits signed value).
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_print_dl_payload(sfx_u8 *dl_payload, sfx_u8 dl_payload_size, sfx_s16 rssi_dbm);
#endif

#ifdef VERBOSE
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes)
 * \brief Get device initital PAC code.
 * \param[in] 	initial_pac_size_bytes: Number of bytes of the PAC to read (full size is SIGFOX_EP_PAC_SIZE_BYTES).
 * \param[out] 	initial_pac: Initial PAC.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_get_initial_pac(sfx_u8 *initial_pac, sfx_u8 initial_pac_size_bytes);
#endif

#if (defined TIMER_REQUIRED) && (defined LATENCY_COMPENSATION) && (defined BIDIRECTIONAL)
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_get_latency(MCU_API_latency_t latency_type, sfx_u32 *latency_ms)
 * \brief Read MCU latency in milliseconds.
 * \brief This functions is called by the core library to compensate the durations in the MCU_API_timer_start() function.
 * \param[in]	latency_type: Type of latency to get.
 * \param[out] 	latency_ms: Pointer to integer that will contain the MCU latency in milliseconds.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_get_latency(MCU_API_latency_t latency_type, sfx_u32 *latency_ms);
#endif

#ifdef VERBOSE
/*!******************************************************************
 * \fn MCU_API_status_t MCU_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char)
 * \brief Get MCU driver version.
 * \param[in]  	none
 * \param[out] 	version: MCU driver version.
 * \param[out]	version_size_char: Pointer that will contain the string size.
 * \retval		Function execution status.
 *******************************************************************/
MCU_API_status_t MCU_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char);
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void MCU_API_error(void)
 * \brief Function called by the library if any error occurred during the processing.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void MCU_API_error(void);
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void MCU_API_stack_error(void)
 * \brief Generic macro which calls the error stack function for MCU errors (if enabled).
 * \param[in]  	none
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#ifdef ERROR_STACK
#define MCU_API_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_MCU_API, mcu_api_status)
#else
#define MCU_API_stack_error(void)
#endif
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void MCU_API_check_status(error)
 * \brief Generic macro to check an MCU_API function status and exit.
 * \param[in]  	error: High level error code to rise.
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#define MCU_API_check_status(error) { if (mcu_api_status != MCU_API_SUCCESS) { MCU_API_stack_error(); EXIT_ERROR(error) } }
#endif

#endif /* __MCU_API_H__ */
