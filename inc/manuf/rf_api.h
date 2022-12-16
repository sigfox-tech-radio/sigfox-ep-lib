/*!*****************************************************************
 * \file    rf_api.h
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

#ifndef __RF_API_H__
#define __RF_API_H__

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

/*** RF API structures ***/

#ifdef ERROR_CODES
/*!******************************************************************
 * \enum RF_API_status_t
 * \brief RF driver error codes.
 *******************************************************************/
typedef enum {
    RF_API_SUCCESS = 0,
    RF_API_ERROR
	// Additional custom error codes can be added here (up to sfx_u32).
	// They will be logged in the library error stack if the ERROR_STACK flag is defined.
} RF_API_status_t;
#else
typedef void RF_API_status_t;
#endif

#ifdef ASYNCHRONOUS
/********************************
 * \brief RF driver callback functions.
 * \fn RF_API_process_cb_t			To be called when the RF driver needs to be processed.
 * \fn RF_API_error_cb_t			To be called when an error occurs during RF operation.
 * \fn RF_API_tx_cplt_cb_t			To be called when a frame transmission is complete.
 * \fn RF_API_rx_data_received_cb_t	To be called when a diwnlink frame is received.
 * \fn RF_API_channel_free_cb_t		To be called when the carrier sense operation is complete.
 *******************************/
typedef void (*RF_API_process_cb_t)(void);
#ifdef ERROR_CODES
typedef void (*RF_API_error_cb_t)(RF_API_status_t status);
#else
typedef void (*RF_API_error_cb_t)(void);
#endif
typedef void (*RF_API_tx_cplt_cb_t)(void);
#ifdef BIDIRECTIONAL
typedef void (*RF_API_rx_data_received_cb_t)(void);
#endif
#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
typedef void (*RF_API_channel_free_cb_t)(void);
#endif
#endif

/*!******************************************************************
 * \enum RF_API_mode_t
 * \brief RF modes list.
 *******************************************************************/
typedef enum {
	RF_API_MODE_TX,
#if (defined BIDIRECTIONAL) || ((defined REGULATORY && (defined SPECTRUM_ACCESS_LBT)))
	RF_API_MODE_RX,
#endif
	RF_API_MODE_LAST
} RF_API_mode_t;

/*!******************************************************************
 * \enum RF_API_modulation_t
 * \brief RF modulations list.
 *******************************************************************/
typedef enum {
    RF_API_MODULATION_NONE,
	RF_API_MODULATION_DBPSK,
	RF_API_MODULATION_GFSK,
	RF_API_MODULATION_LAST
} RF_API_modulation_t;

/*!******************************************************************
 * \struct RF_API_radio_parameters_t
 * \brief Radio parameters structure.
 *******************************************************************/
typedef struct {
	RF_API_mode_t rf_mode;
	sfx_u32 frequency_hz;
	RF_API_modulation_t modulation;
	sfx_u16 bit_rate_bps;
	sfx_s8 tx_power_dbm_eirp;
#ifdef BIDIRECTIONAL
	sfx_u32 deviation_hz;
#endif
} RF_API_radio_parameters_t;

/*!******************************************************************
 * \struct RF_API_tx_data_t
 * \brief RF TX data structure.
 *******************************************************************/
typedef struct {
	sfx_u8 *bitstream;
	sfx_u8 bitstream_size_bytes;
#ifdef ASYNCHRONOUS
	RF_API_tx_cplt_cb_t cplt_cb;
#endif
} RF_API_tx_data_t;

/*!******************************************************************
 * \struct RF_API_rx_data_t
 * \brief RF RX data structure.
 *******************************************************************/
typedef struct {
	sfx_u8 dl_phy_content_size;
#if (defined ASYNCHRONOUS) && (defined BIDIRECTIONAL)
	RF_API_rx_data_received_cb_t data_received_cb;
#else
	sfx_bool data_received;
#endif
} RF_API_rx_data_t;

#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
/*!******************************************************************
 * \struct RF_API_carrier_sense_parameters_t
 * \brief RF carrier sense parameters structure.
 *******************************************************************/
typedef struct {
	sfx_u32 bandwidth_hz;
	sfx_s8 threshold_dbm;
	sfx_u32 min_duration_ms;
#ifdef ASYNCHRONOUS
	RF_API_channel_free_cb_t channel_free_cb;
#else
	sfx_bool *channel_free;
#endif
} RF_API_carrier_sense_parameters_t;
#endif

/*!******************************************************************
 * \struct RF_API_config_t
 * \brief RF API configuration structure.
 *******************************************************************/
typedef struct {
	const SIGFOX_rc_t *rc;
#ifdef ASYNCHRONOUS
	RF_API_process_cb_t process_cb;
	RF_API_error_cb_t error_cb;
#endif
} RF_API_config_t;

/*** RF API functions ***/

#if (defined ASYNCHRONOUS) || (defined LOW_LEVEL_OPEN_CLOSE)
/*!******************************************************************
 * \fn RF_API_status_t RF_API_open(RF_API_config_t *rf_api_config)
 * \brief Open the RF driver.
 * \param[in]  	rf_api_config: Pointer to the RF API configuration.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_open(RF_API_config_t *rf_api_config);
#endif

#ifdef LOW_LEVEL_OPEN_CLOSE
/*!******************************************************************
 * \fn RF_API_status_t RF_API_close(void)
 * \brief Close the RF driver.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_close(void);
#endif

#ifdef ASYNCHRONOUS
/*!******************************************************************
 * \fn RF_API_status_t RF_API_process(void)
 * \brief Process RF driver, this function will be call by SIGFOX_EP_API_process just after the process_callback has been sent to process RF interruptions in main context.
 * \param[in]   none
 * \param[out]  none
 * \retval      Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_process(void);
#endif

/*!******************************************************************
 * \fn RF_API_status_t RF_API_wake_up(void)
 * \brief Wake-up the radio before each overall TX or RX sequence.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_wake_up(void);

/*!******************************************************************
 * \fn RF_API_status_t RF_API_sleep(void)
 * \brief Release the radio after each overall TX or RX sequence.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_sleep(void);

/*!******************************************************************
 * \fn RF_API_status_t RF_API_init(RF_API_radio_parameters_t *radio_parameters)
 * \brief Initialize the radio operation before each individual frame transmission or reception.
 * \param[in]  	radio_parameters: Pointers to the radio parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_init(RF_API_radio_parameters_t *radio_parameters);

/*!******************************************************************
 * \fn RF_API_status_t RF_API_de_init(void)
 * \brief Stop the radio operation after each individual frame transmission or reception.
 * \param[in]  	rf_mode: Radio mode.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_de_init(void);

/*!******************************************************************
 * \fn RF_API_status_t RF_API_send(RF_API_tx_data_t *tx_data)
 * \brief Sending a bitstream over the air.
 * \brief In blocking mode, this function blocks until the full bitstream is sent.
 * \brief In asynchronous, this function only starts the transmission. End of transmission should be notified through the cplt_cb() callback.
 * \param[in]	tx_data: Pointer to the TX parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_send(RF_API_tx_data_t *tx_data);

#ifdef BIDIRECTIONAL
/*!******************************************************************
 * \fn RF_API_status_t RF_API_receive(RF_API_rx_data_t *rx_data)
 * \brief Start downlink reception. Could be called multiple times if several downlink frames are received during the RX window.
 * \brief In blocking mode, this function blocks until a valid downlink data is received or the MCU_API_TIMER_2 has elapsed.
 * \brief In asynchronous mode, this function only starts the reception. Data reception should be notified through the rx_data_received() callback.
 * \param[in]	rx_data: Pointer to the RX parameters.
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_receive(RF_API_rx_data_t *rx_data);
#endif

#ifdef BIDIRECTIONAL
/*!******************************************************************
 * \fn RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm)
 * \brief Read DL-PHY content and RSSI received by the radio.
 * \brief In blocking mode, this function will be called only if the data_received parameter of the RF_API_receive() function is returned with SFX_TRUE value.
 * \brief in asynchronous mode, this function will be called only if the data_received_cb callback is triggered during reception.
 * \param[in]	dl_phy_content_size: Number of bytes to copy in dl_phy_content.
 * \param[out]	dl_phy_content: Array to be filled with the received DL-PHY content.
 * \param[out]	dl_rssi_dbm: Pointer to 16-bits signed value to be filled with the DL RSSI in dBm.
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_get_dl_phy_content_and_rssi(sfx_u8 *dl_phy_content, sfx_u8 dl_phy_content_size, sfx_s16 *dl_rssi_dbm);
#endif

#if (defined REGULATORY) && (defined SPECTRUM_ACCESS_LBT)
/*!******************************************************************
 * \fn RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params)
 * \brief In blocking mode, the function until the LBT condition is met or the MCU_API_TIMER_1 has elapsed.
 * \brief In asynchronous mode, this function only starts the carrier sense operation. Channel free event should be notified through the channel_free_cb() callback.
 * \param[in]	carrier_sense_params: Pointer to the carrier sense parameters.
 * \param[out] 	none
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_carrier_sense(RF_API_carrier_sense_parameters_t *carrier_sense_params);
#endif

#ifdef VERBOSE
/*!******************************************************************
 * \fn RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char)
 * \brief Get RF driver version.
 * \param[in]  	none
 * \param[out] 	version: RF driver version.
 * \param[out]	version_size_char: Pointer tha will contain the string size.
 * \retval		Function execution status.
 *******************************************************************/
RF_API_status_t RF_API_get_version(sfx_u8 **version, sfx_u8 *version_size_char);
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void RF_API_error(void)
 * \brief Function called by the library if any error occurred during the processing.
 * \param[in]  	none
 * \param[out] 	none
 * \retval		none
 *******************************************************************/
void RF_API_error(void);
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void RF_API_stack_error(void)
 * \brief Generic macro which calls the error stack function for RF errors (if enabled).
 * \param[in]  	none
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#ifdef ERROR_STACK
#define RF_API_stack_error(void) SIGFOX_ERROR_stack(SIGFOX_ERROR_SOURCE_RF, rf_status)
#else
#define RF_API_stack_error(void)
#endif
#endif

#ifdef ERROR_CODES
/*!******************************************************************
 * \fn void RF_API_check_status(error)
 * \brief Generic macro to check an RF_API function status and exit.
 * \param[in]  	error: High level error code to rise.
 * \param[out]	none
 * \retval		none
 *******************************************************************/
#define RF_API_check_status(error) { if (rf_status != RF_API_SUCCESS) { RF_API_stack_error(); EXIT_ERROR(error) } }
#endif

#endif /* __RF_API_H__ */
