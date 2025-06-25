/*!*****************************************************************
 * \file    sigfox_ep_bitstream.c
 * \brief   Sigfox bitstream builder and decoder.
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

#include "core/sigfox_ep_bitstream.h"

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "manuf/mcu_api.h"
#ifndef SIGFOX_EP_CRC_HW
#include "core/sigfox_crc.h"
#endif
#include "sigfox_types.h"
#include "sigfox_error.h"
#ifndef SIGFOX_EP_AES_HW
#include "core/TI_aes_128_encr_only.h"
#endif

/*** SIGFOX EP BITSTREAM local macros ***/

#define SIGFOX_EP_BITSTREAM_UL_PR_INDEX         0
#define SIGFOX_EP_BITSTREAM_UL_PR_SIZE_BYTES    2
#define SIGFOX_EP_BITSTREAM_UL_PR_VALUE         0xAA

#define SIGFOX_EP_BITSTREAM_UL_FT_INDEX         (SIGFOX_EP_BITSTREAM_UL_PR_INDEX + SIGFOX_EP_BITSTREAM_UL_PR_SIZE_BYTES)
#define SIGFOX_EP_BITSTREAM_UL_FT_SIZE_BYTES    2

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE

#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 0)
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1         0xA06B
#ifndef SIGFOX_EP_SINGLE_FRAME
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2         0xA6E0
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3         0xA034
static const sfx_u16 SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[3] = {SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3};
#else
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION               SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1;
#endif /* SIGFOX_EP_SINGLE_FRAME */
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE == 0 */

#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 1)
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1         0xA08D
#ifndef SIGFOX_EP_SINGLE_FRAME
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2         0xA0D2
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3         0xA302
static const sfx_u16 SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[3] = {SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3};
#else
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION               SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1;
#endif /* SIGFOX_EP_SINGLE_FRAME */
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE == 1 */

#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 2) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 3) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 4)
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1         0xA35F
#ifndef SIGFOX_EP_SINGLE_FRAME
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2         0xA598
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3         0xA5A3
static const sfx_u16 SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[3] = {SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3};
#else
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION               SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1;
#endif /* SIGFOX_EP_SINGLE_FRAME */
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE == 2 to 4 */

#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 5) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 6) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 7) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 8)
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1         0xA611
#ifndef SIGFOX_EP_SINGLE_FRAME
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2         0xA6BF
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3         0xA72C
static const sfx_u16 SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[3] = {SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3};
#else
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION               SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1;
#endif /* SIGFOX_EP_SINGLE_FRAME */
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE == 5 to 8 */

#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 9) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 10) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 11) || (SIGFOX_EP_UL_PAYLOAD_SIZE == 12)
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1         0xA94C
#ifndef SIGFOX_EP_SINGLE_FRAME
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2         0xA971
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3         0xA997
static const sfx_u16 SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[3] = {SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3};
#else
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION               SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1;
#endif /* SIGFOX_EP_SINGLE_FRAME */
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE == 9 to 12 */

#else /* SIGFOX_EP_UL_PAYLOAD_SIZE */

#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1_TABLE       {0xA06B, 0xA08D, 0xA35F, 0xA611, 0xA94C}
#ifndef SIGFOX_EP_SINGLE_FRAME
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2_TABLE       {0xA6E0, 0xA0D2, 0xA598, 0xA6BF, 0xA971}
#define SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3_TABLE       {0xA034, 0xA302, 0xA5A3, 0xA72C, 0xA997}
static const sfx_u16 SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[3][5] = { SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1_TABLE, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK2_TABLE, SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK3_TABLE };
#else
static const sfx_u16 SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[5] = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_RANK1_TABLE;
#endif /* SIGFOX_EP_SINGLE_FRAME */

#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
#endif /* SIGFOX_EP_APPLICATION_MESSAGES */

#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL))
#ifdef SIGFOX_EP_SINGLE_FRAME
static const sfx_u16 SIGFOX_EP_BITSTREAM_UL_FT_CONTROL = 0xAF67;
#else
static const sfx_u16 SIGFOX_EP_BITSTREAM_UL_FT_CONTROL_TABLE[3] = { 0xAF67, 0xAFC9, 0xB1BE };
#endif
#endif

#define SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX          (SIGFOX_EP_BITSTREAM_UL_FT_INDEX + SIGFOX_EP_BITSTREAM_UL_FT_SIZE_BYTES)
#define SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_SIZE_BYTES     2
#define SIGFOX_EP_BITSTREAM_BF_BIT_INDEX                5
#define SIGFOX_EP_BITSTREAM_LI_BIT_INDEX                6

#define SIGFOX_EP_BITSTREAM_EP_ID_INDEX                 (SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_SIZE_BYTES)

#define SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX            (SIGFOX_EP_BITSTREAM_EP_ID_INDEX + SIGFOX_EP_ID_SIZE_BYTES)

#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
#define SIGFOX_EP_BITSTREAM_CONTROL_KEEP_ALIVE_CT                       0x08
#define SIGFOX_EP_BITSTREAM_CONTROL_KEEP_ALIVE_PAYLOAD_SIZE_BYTES       7
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
#define SIGFOX_EP_BITSTREAM_CONTROL_DL_CONFIRMATION_CT                  0x09
#define SIGFOX_EP_BITSTREAM_CONTROL_DL_CONFIRMATION_PAYLOAD_SIZE_BYTES  8
#endif

#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE))
static const sfx_u8 SIGFOX_EP_BITSTREAM_UL_ENCRYPTION_LOOP_LUT[SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES + 1] = { 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 1, 2, 2 };
#endif

#define SIGFOX_EP_BITSTREAM_UL_CRC_SIZE_BYTES   2
#define SIGFOX_EP_BITSTREAM_UL_CRC_POLYNOM      0x1021

#ifndef SIGFOX_EP_SINGLE_FRAME
static const sfx_u8 SIGFOX_EP_BITSTREAM_UL_CONVOLUTION_LUT_IN[4] = { 0, 0, 1, 1 };
static const sfx_u8 SIGFOX_EP_BITSTREAM_UL_CONVOLUTION_LUT_IN_OUT[3][4] = { { 0, 0, 0, 0 }, { 0, 1, 1, 0 }, { 0, 1, 0, 1 } };
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
#define SIGFOX_EP_BITSTREAM_DL_ECC_INDEX        0
#define SIGFOX_EP_BITSTREAM_DL_ECC_SIZE_BYTES   4

#define SIGFOX_EP_BITSTREAM_DL_PAYLOAD_INDEX    (SIGFOX_EP_BITSTREAM_DL_ECC_INDEX + SIGFOX_EP_BITSTREAM_DL_ECC_SIZE_BYTES)

#define SIGFOX_EP_BITSTREAM_DL_AUTH_INDEX       (SIGFOX_EP_BITSTREAM_DL_PAYLOAD_INDEX + SIGFOX_DL_PAYLOAD_SIZE_BYTES)
#define SIGFOX_EP_BITSTREAM_DL_AUTH_SIZE_BYTES  2

#define SIGFOX_EP_BITSTREAM_DL_CRC_INDEX        (SIGFOX_EP_BITSTREAM_DL_AUTH_INDEX + SIGFOX_EP_BITSTREAM_DL_AUTH_SIZE_BYTES)
#define SIGFOX_EP_BITSTREAM_DL_CRC_SIZE_BYTES   1
#define SIGFOX_EP_BITSTREAM_DL_CRC_POLYNOM      0x2F

#define SIGFOX_EP_BITSTREAM_DL_DEWHITENING_MASK 0x01FF

#define SIGFOX_EP_BITSTREAM_DL_BCH_LENGTH       15
static const sfx_u8 SIGFOX_EP_BITSTREAM_DL_POW_ALPHA_LUT[SIGFOX_EP_BITSTREAM_DL_BCH_LENGTH] = { 1, 2, 4, 8, 3, 6, 12, 11, 5, 10, 7, 14, 15, 13, 9 };
static const sfx_u8 SIGFOX_EP_BITSTREAM_DL_LOG_ALPHA_LUT[SIGFOX_EP_BITSTREAM_DL_BCH_LENGTH + 1] = { 15, 0, 1, 4, 2, 8, 5, 10, 3, 14, 9, 7, 6, 13, 11, 12 };
#endif

/*** SIGFOX EP BITSTREAM local structures ***/

/*******************************************************************/
typedef struct {
    sfx_u8 *bitstream;
    sfx_u16 ul_ft;
    sfx_u8 ul_li;
    sfx_u8 ul_auth_size_bytes;
#if (defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE)
    sfx_u8 ul_payload_size_bytes;
#endif
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    SIGFOX_ep_key_t key;
#endif
} SIGFOX_EP_BITSTREAM_context_t;

/*** SIGFO EP BITSTREAM local global variables ***/

static SIGFOX_EP_BITSTREAM_context_t sigfox_ep_bitstream_ctx;

/*** SIGFOX EP BITSTREAM local functions ***/

/*******************************************************************/
static SIGFOX_EP_BITSTREAM_status_t _aes_encrypt(void) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_BITSTREAM_status_t status = SIGFOX_EP_BITSTREAM_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#endif
#ifdef SIGFOX_EP_AES_HW
    MCU_API_encryption_data_t encryption_data;
#else
    sfx_u8 aes_key[SIGFOX_EP_KEY_SIZE_BYTES];
#endif
    sfx_u8 idx = 0;
    sfx_u8 aes_data[SIGFOX_EP_KEY_SIZE_BYTES];
#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE >= 11))
    sfx_u8 encryption_idx = 0;
#endif
#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE))
    sfx_u8 ul_auth_input_size_bytes = (sfx_u8) (SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_SIZE_BYTES + SIGFOX_EP_ID_SIZE_BYTES + (sigfox_ep_bitstream_ctx.ul_payload_size_bytes));
#else
    sfx_u8 ul_auth_input_size_bytes = (sfx_u8) (SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_SIZE_BYTES + SIGFOX_EP_ID_SIZE_BYTES + SIGFOX_EP_UL_PAYLOAD_SIZE);
#endif
    sfx_u8 ul_auth_input_index = 0;
    // First initialization vector is null.
    for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
        aes_data[idx] = 0x00;
    }
#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE >= 11))
    // Encryption loop.
#if( (defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE))
    for (encryption_idx = 0; encryption_idx < SIGFOX_EP_BITSTREAM_UL_ENCRYPTION_LOOP_LUT[sigfox_ep_bitstream_ctx.ul_payload_size_bytes]; encryption_idx++) {
#else
        for (encryption_idx = 0; encryption_idx < 2; encryption_idx++) {
#endif
#endif /* Multiple loops required */
        // Build input data.
        for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
            // XOR operation with initialization vector.
            aes_data[idx] ^= (sigfox_ep_bitstream_ctx.bitstream)[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + ul_auth_input_index];
            ul_auth_input_index = (sfx_u8) ((ul_auth_input_index + 1) % ul_auth_input_size_bytes);
        }
#ifdef SIGFOX_EP_AES_HW
        // Build input structure.
        encryption_data.data = (sfx_u8*) aes_data;
        encryption_data.data_size_bytes = SIGFOX_EP_KEY_SIZE_BYTES;
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
        encryption_data.key = sigfox_ep_bitstream_ctx.key;
#endif
        // Perform encryption.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_aes_128_cbc_encrypt(&encryption_data);
        MCU_API_check_status(SIGFOX_EP_BITSTREAM_ERROR_DRIVER_MCU_API);
#else
        MCU_API_aes_128_cbc_encrypt(&encryption_data);
#endif
#else /* SIGFOX_EP_AES_HW */
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    // Warning: TI AES algorithm modifies the key during computation, so it must be read before each encryption step.
    switch (sigfox_ep_bitstream_ctx.key) {
    case SIGFOX_EP_KEY_PRIVATE:
        // Read device private key.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_get_ep_key(aes_key, SIGFOX_EP_KEY_SIZE_BYTES);
        MCU_API_check_status(SIGFOX_EP_BITSTREAM_ERROR_DRIVER_MCU_API);
#else
        MCU_API_get_ep_key(aes_key, SIGFOX_EP_KEY_SIZE_BYTES);
#endif
        break;
    case SIGFOX_EP_KEY_PUBLIC:
        // Use public key.
        for (idx = 0; idx < SIGFOX_EP_KEY_SIZE_BYTES; idx++) {
            aes_key[idx] = SIGFOX_EP_PUBLIC_KEY[idx];
        }
        break;
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_KEY_TYPE);
        break;
    }
#else /* SIGFOX_EP_PUBLIC_KEY_CAPABLE */
    // Read device private key.
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_get_ep_key(aes_key, SIGFOX_EP_KEY_SIZE_BYTES);
    MCU_API_check_status(SIGFOX_EP_BITSTREAM_ERROR_DRIVER_MCU_API);
#else
    MCU_API_get_ep_key(aes_key, SIGFOX_EP_KEY_SIZE_BYTES);
#endif
#endif /* SIGFOX_EP_PUBLIC_KEY_CAPABLE */
    aes_encrypt(aes_data, (sfx_u8*) aes_key);
#endif /* SIGFOX_EP_AES_HW */
#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE >= 11))
    }
#endif
    // Add UL-AUTH field to bitstream.
    for (idx = 0; idx < (sigfox_ep_bitstream_ctx.ul_auth_size_bytes); idx++) {
#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE))
        (sigfox_ep_bitstream_ctx.bitstream)[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + sigfox_ep_bitstream_ctx.ul_payload_size_bytes + idx] = aes_data[idx];
#else
        (sigfox_ep_bitstream_ctx.bitstream)[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + SIGFOX_EP_UL_PAYLOAD_SIZE + idx] = aes_data[idx];
#endif
    }
#if ((defined SIGFOX_EP_ERROR_CODES) || ((defined SIGFOX_EP_PUBLIC_KEY_CAPABLE) && !(defined SIGFOX_EP_AES_HW)))
errors:
#endif
    SIGFOX_RETURN();
}

/*******************************************************************/
static SIGFOX_EP_BITSTREAM_status_t _add_crc16(void) {
    // Local variables.
    sfx_u16 ul_crc = 0;
#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE))
    sfx_u8 crc_input_size_bytes = (sfx_u8) (SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_SIZE_BYTES + SIGFOX_EP_ID_SIZE_BYTES + sigfox_ep_bitstream_ctx.ul_payload_size_bytes + sigfox_ep_bitstream_ctx.ul_auth_size_bytes);
#else
    sfx_u8 crc_input_size_bytes = (sfx_u8) (SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_SIZE_BYTES + SIGFOX_EP_ID_SIZE_BYTES + SIGFOX_EP_UL_PAYLOAD_SIZE + sigfox_ep_bitstream_ctx.ul_auth_size_bytes);
#endif
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_BITSTREAM_status_t status = SIGFOX_EP_BITSTREAM_SUCCESS;
#ifdef SIGFOX_EP_CRC_HW
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#else
    SIGFOX_CRC_status_t sigfox_crc_status = SIGFOX_CRC_SUCCESS;
#endif
#endif
    // Compute CRC.
#ifdef SIGFOX_EP_CRC_HW
#ifdef SIGFOX_EP_ERROR_CODES
    mcu_api_status = MCU_API_compute_crc16(&(sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX]), crc_input_size_bytes, SIGFOX_EP_BITSTREAM_UL_CRC_POLYNOM, &ul_crc);
    MCU_API_check_status(SIGFOX_EP_BITSTREAM_ERROR_DRIVER_MCU_API);
#else
    MCU_API_compute_crc16(&(sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX]), crc_input_size_bytes, SIGFOX_EP_BITSTREAM_UL_CRC_POLYNOM, &ul_crc);
#endif
#else
#ifdef SIGFOX_EP_ERROR_CODES
    sigfox_crc_status = SIGFOX_CRC_compute_crc16(&(sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX]), crc_input_size_bytes, SIGFOX_EP_BITSTREAM_UL_CRC_POLYNOM, &ul_crc);
    SIGFOX_CRC_check_status(SIGFOX_EP_BITSTREAM_ERROR_DRIVER_SIGFOX_CRC);
#else
    SIGFOX_CRC_compute_crc16(&(sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX]), crc_input_size_bytes, SIGFOX_EP_BITSTREAM_UL_CRC_POLYNOM, &ul_crc);
#endif
#endif
    ul_crc ^= 0xFFFF;
    // Add CRC16 field to bitstream.
#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE))
    sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + sigfox_ep_bitstream_ctx.ul_payload_size_bytes + sigfox_ep_bitstream_ctx.ul_auth_size_bytes + 0] = (sfx_u8) ((ul_crc >> 8) & 0xFF);
    sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + sigfox_ep_bitstream_ctx.ul_payload_size_bytes + sigfox_ep_bitstream_ctx.ul_auth_size_bytes + 1] = (sfx_u8) ((ul_crc >> 0) & 0xFF);
#else
    sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + SIGFOX_EP_UL_PAYLOAD_SIZE + sigfox_ep_bitstream_ctx.ul_auth_size_bytes + 0] = (sfx_u8) ((ul_crc >> 8) & 0xFF);
    sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + SIGFOX_EP_UL_PAYLOAD_SIZE + sigfox_ep_bitstream_ctx.ul_auth_size_bytes + 1] = (sfx_u8) ((ul_crc >> 0) & 0xFF);
#endif
#ifdef SIGFOX_EP_ERROR_CODES
errors:
#endif
    SIGFOX_RETURN();
}

#ifndef SIGFOX_EP_SINGLE_FRAME
/*******************************************************************/
static void _convolve(SIGFOX_ul_frame_rank_t ul_frame_rank) {
    // Local variables.
    sfx_u8 initial_bitstream[SIGFOX_UL_BITSTREAM_SIZE_BYTES];
    sfx_u8 idx = 0;
    sfx_u8 mask = 0;
    sfx_u8 state = 0;
#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE))
    sfx_u8 convolution_input_size_bytes = (sfx_u8) (SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_SIZE_BYTES + SIGFOX_EP_ID_SIZE_BYTES + sigfox_ep_bitstream_ctx.ul_payload_size_bytes + sigfox_ep_bitstream_ctx.ul_auth_size_bytes + SIGFOX_EP_BITSTREAM_UL_CRC_SIZE_BYTES);
#else
    sfx_u8 convolution_input_size_bytes = (sfx_u8) (SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_SIZE_BYTES + SIGFOX_EP_ID_SIZE_BYTES + SIGFOX_EP_UL_PAYLOAD_SIZE + sigfox_ep_bitstream_ctx.ul_auth_size_bytes + SIGFOX_EP_BITSTREAM_UL_CRC_SIZE_BYTES);
#endif
    // Init buffers.
    for (idx = 0; idx < convolution_input_size_bytes; idx++) {
        initial_bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + idx] = sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + idx];
        sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + idx] = 0;
    }
    for (idx = 0; idx < convolution_input_size_bytes; idx++) {
        for (mask = 0x80; mask != 0; mask >>= 1) {
            if (initial_bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + idx] & mask) {
                sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + idx] |= (sfx_u8) (mask * ((SIGFOX_EP_BITSTREAM_UL_CONVOLUTION_LUT_IN_OUT[ul_frame_rank][state] == 0) ? 1 : 0));
                state = (sfx_u8) (SIGFOX_EP_BITSTREAM_UL_CONVOLUTION_LUT_IN[state] + 2);
            }
            else {
                sigfox_ep_bitstream_ctx.bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + idx] |= (sfx_u8) (mask * SIGFOX_EP_BITSTREAM_UL_CONVOLUTION_LUT_IN_OUT[ul_frame_rank][state]);
                state = SIGFOX_EP_BITSTREAM_UL_CONVOLUTION_LUT_IN[state];
            }
        }
    }
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static sfx_u16 _dewhitening_pn(sfx_u16 pn_value) {
    // Local variables.
    sfx_u8 msb = 0;
    sfx_u8 roll = 0;
    // Compute algorithm.
    for (roll = 8; roll > 0; roll--) {
        msb = (sfx_u8) (pn_value & 0x0021);
        pn_value >>= 1;
        pn_value = (sfx_u16) (pn_value | ((msb == 0x20 || msb == 0x01) ? 0x0100 : 0x0000));
    }
    return pn_value;
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static void _dewhitening(SIGFOX_EP_BITSTREAM_dl_frame_t *input, sfx_u8 *local_bitstream) {
    // Local variables.
    sfx_u8 idx = 0;
    sfx_u32 ep_id_32bits = 0;
    sfx_u32 pn = 0;
    // Convert EP ID array to 32-bits value.
    for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
        ep_id_32bits |= (sfx_u32) ((input->ep_id[SIGFOX_EP_ID_SIZE_BYTES - 1 - idx]) << (8 * idx));
    }
    // Set initial value as device ID * message counter.
    pn = ep_id_32bits * ((sfx_u32) (input->message_counter));
    // Apply 9-bits mask.
    pn &= SIGFOX_EP_BITSTREAM_DL_DEWHITENING_MASK;
    if (pn == 0) {
        pn = SIGFOX_EP_BITSTREAM_DL_DEWHITENING_MASK;
    }
    // Perform dewhitening algorithm.
    for (idx = 0; idx < 8; idx++) {
        pn = _dewhitening_pn((sfx_u16) pn);
        local_bitstream[idx + 0] ^= (sfx_u8) (pn >> (idx + 1));
        local_bitstream[idx + 1] ^= (sfx_u8) ((((sfx_s32) pn) & ((1 << (idx + 1)) - 1)) << (7 - idx));
    }
    for (idx = 0; idx < 5; idx++) {
        pn = _dewhitening_pn((sfx_u16) pn);
        local_bitstream[idx + 9] ^= (sfx_u8) (pn >> (idx + 1));
        local_bitstream[idx + 10] ^= (sfx_u8) ((((sfx_s32) pn) & ((1 << (idx + 1)) - 1)) << (7 - idx));
    }
    pn = _dewhitening_pn((sfx_u16) pn);
    local_bitstream[14] ^= (sfx_u8) (pn >> (idx + 1));
    local_bitstream[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES] = 0;
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
static void _decode_bch(sfx_u8 *local_bitstream) {
    // Local variables.
    sfx_u8 i = 0;
    sfx_u8 j = 0;
    sfx_u8 syndrome = 0;
    // Perform ECC algorithm.
    for (i = 0; i < 8; i++) {
        // Compute code syndrome.
        syndrome = 0;
        for (j = 0; j < SIGFOX_EP_BITSTREAM_DL_BCH_LENGTH; j++) {
            syndrome ^= (sfx_u8) (SIGFOX_EP_BITSTREAM_DL_POW_ALPHA_LUT[j] * ((local_bitstream[j] >> i) & 1));
        }
        // The BCH(15,11) code can only correct a single error.
        // If the syndrome is not zero, it indicates the bit in error.
        // If the syndrome is zero, there is no error. To simplify the code, word 16 is changed.
        // Word 16 is actually a dummy word.
        local_bitstream[SIGFOX_EP_BITSTREAM_DL_LOG_ALPHA_LUT[syndrome]] ^= (sfx_u8) (1 << i);
    }
}
#endif

#if ((defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES))
/*******************************************************************/
static SIGFOX_EP_BITSTREAM_status_t _check_common_parameters(SIGFOX_EP_BITSTREAM_common_t *common_params) {
    // Local variables.
    SIGFOX_EP_BITSTREAM_status_t status = SIGFOX_EP_BITSTREAM_SUCCESS;
    // Check parameter.
    if (common_params == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_NULL_PARAMETER);
    }
    // Check ID.
    if ((common_params->ep_id) == SIGFOX_NULL) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_NULL_PARAMETER);
    }
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Check frame rank.
    if ((common_params->ul_frame_rank) >= SIGFOX_UL_FRAME_RANK_LAST) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_FRAME_RANK);
    }
#endif
    // Check message counter.
#ifdef SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER
    if ((common_params -> message_counter) >= SIGFOX_EP_MESSAGE_COUNTER_ROLLOVER) {
#else
    if ((common_params->message_counter) >= (common_params->message_counter_rollover)) {
#endif
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_MESSAGE_COUNTER);
    }
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    // Check key type.
    if ((common_params->ep_key_type) >= SIGFOX_EP_KEY_LAST) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_KEY_TYPE);
    }
#endif
errors:
    return status;
}
#endif

#if ((defined SIGFOX_EP_APPLICATION_MESSAGES) && (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES))
/*******************************************************************/
static SIGFOX_EP_BITSTREAM_status_t _check_application_parameters(SIGFOX_EP_BITSTREAM_application_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes) {
    // Local variables.
    SIGFOX_EP_BITSTREAM_status_t status = SIGFOX_EP_BITSTREAM_SUCCESS;
    // Check parameters.
    if ((input == SIGFOX_NULL) || (bitstream == SIGFOX_NULL) || (bitstream_size_bytes == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_NULL_PARAMETER);
    }
    // Check common parameters.
    status = _check_common_parameters(&(input->common_parameters));
    SIGFOX_CHECK_STATUS(SIGFOX_EP_BITSTREAM_SUCCESS);
#if !(defined SIGFOX_EP_UL_PAYLOAD_SIZE) || (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    if ((input->message_type) == SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY) {
        // Payload is required.
        if ((input->ul_payload) == SIGFOX_NULL) {
            SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_NULL_PARAMETER);
        }
    }
#endif
#ifndef SIGFOX_EP_UL_PAYLOAD_SIZE
    if ((input->message_type) == SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY) {
        // Length is required.
        if (((input->ul_payload_size_bytes) == 0) || ((input->ul_payload_size_bytes) > SIGFOX_UL_PAYLOAD_MAX_SIZE_BYTES)) {
            SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_PAYLOAD_SIZE);
        }
    }
#endif
errors:
    return status;
}
#endif

#if (((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL)) && (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES))
/*******************************************************************/
static SIGFOX_EP_BITSTREAM_status_t _check_control_parameters(SIGFOX_EP_BITSTREAM_control_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes) {
    // Local variables.
    SIGFOX_EP_BITSTREAM_status_t status = SIGFOX_EP_BITSTREAM_SUCCESS;
    // Check parameters.
    if ((input == SIGFOX_NULL) || (bitstream == SIGFOX_NULL) || (bitstream_size_bytes == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_NULL_PARAMETER);
    }
    // Check common parameters.
    status = _check_common_parameters(&(input->common_parameters));
    SIGFOX_CHECK_STATUS(SIGFOX_EP_BITSTREAM_SUCCESS);
errors:
    return status;
}
#endif

#if ((defined SIGFOX_EP_BIDIRECTIONAL) && (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES))
/*******************************************************************/
static SIGFOX_EP_BITSTREAM_status_t _check_dl_parameters(SIGFOX_EP_BITSTREAM_dl_frame_t *input, sfx_u8 *dl_payload) {
    // Local variables.
    SIGFOX_EP_BITSTREAM_status_t status = SIGFOX_EP_BITSTREAM_SUCCESS;
    // Check parameters.
    if ((input == SIGFOX_NULL) || ((input->dl_phy_content) == SIGFOX_NULL) || ((input->ep_id) == SIGFOX_NULL) || (dl_payload == SIGFOX_NULL)) {
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_NULL_PARAMETER);
    }
errors:
    return status;
}
#endif

/*** SIGFOX EP BITSTREAM functions ***/

#ifdef SIGFOX_EP_APPLICATION_MESSAGES
/*******************************************************************/
SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_application_frame(SIGFOX_EP_BITSTREAM_application_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes) {
    // Local variables.
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_BITSTREAM_status_t status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
#if ((defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES))
    status = _check_application_parameters(input, bitstream, bitstream_size_bytes);
    SIGFOX_CHECK_STATUS(SIGFOX_EP_BITSTREAM_SUCCESS);
#endif
    // Default values.
    sigfox_ep_bitstream_ctx.ul_li = 0;
    sigfox_ep_bitstream_ctx.ul_auth_size_bytes = 2;
    sigfox_ep_bitstream_ctx.bitstream = bitstream;
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    sigfox_ep_bitstream_ctx.key = (input->common_parameters.ep_key_type);
#endif
#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE))
    // UL payload size.
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
    sigfox_ep_bitstream_ctx.ul_payload_size_bytes = SIGFOX_EP_UL_PAYLOAD_SIZE;
#else
    sigfox_ep_bitstream_ctx.ul_payload_size_bytes = (input->ul_payload_size_bytes);
#endif
#endif
    // UL-PR.
    for (idx = 0; idx < SIGFOX_EP_BITSTREAM_UL_PR_SIZE_BYTES; idx++) {
        bitstream[SIGFOX_EP_BITSTREAM_UL_PR_INDEX + idx] = SIGFOX_EP_BITSTREAM_UL_PR_VALUE;
    }
    // UL-FT and UL-LI.
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#ifdef SIGFOX_EP_SINGLE_FRAME
    sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION;
#else
    sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[((input -> common_parameters).ul_frame_rank)];
#endif /* SIGFOX_EP_SINGLE_FRAME */
#if (SIGFOX_EP_UL_PAYLOAD_SIZE == 0)
    switch (input -> message_type) {
    case SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0:
        sigfox_ep_bitstream_ctx.ul_li = 2;
        break;
    case SIGFOX_APPLICATION_MESSAGE_TYPE_BIT1:
        sigfox_ep_bitstream_ctx.ul_li = 3;
        break;
    default:
        // LI=0 for empty frame.
        break;
    }
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE = 0 */
#if (SIGFOX_EP_UL_PAYLOAD_SIZE >= 2)
    sigfox_ep_bitstream_ctx.ul_li = (sfx_u8) (3 - ((SIGFOX_EP_UL_PAYLOAD_SIZE - 1) % 4));
    sigfox_ep_bitstream_ctx.ul_auth_size_bytes = (sfx_u8) (sigfox_ep_bitstream_ctx.ul_li + 2);
#endif
#else /* SIGFOX_EP_UL_PAYLOAD_SIZE */
    switch (input->message_type) {
    case SIGFOX_APPLICATION_MESSAGE_TYPE_EMPTY:
        // Empty frame.
#ifdef SIGFOX_EP_SINGLE_FRAME
        sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[0];
#else
        sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[((input->common_parameters).ul_frame_rank)][0];
#endif
        break;
    case SIGFOX_APPLICATION_MESSAGE_TYPE_BIT0:
        // Empty and bit frames.
#ifdef SIGFOX_EP_SINGLE_FRAME
        sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[0];
#else
        sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[((input->common_parameters).ul_frame_rank)][0];
#endif
        sigfox_ep_bitstream_ctx.ul_li = 2;
        break;
    case SIGFOX_APPLICATION_MESSAGE_TYPE_BIT1:
        // Empty and bit frames.
#ifdef SIGFOX_EP_SINGLE_FRAME
        sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[0];
#else
        sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[((input->common_parameters).ul_frame_rank)][0];
#endif
        sigfox_ep_bitstream_ctx.ul_li = 3;
        break;
    case SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY:
        if (sigfox_ep_bitstream_ctx.ul_payload_size_bytes == 1) {
            // 1-byte frame.
#ifdef SIGFOX_EP_SINGLE_FRAME
            sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[1];
#else
            sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[((input->common_parameters).ul_frame_rank)][1];
#endif
        }
        else {
            // 2 to 12-bytes frames.
#ifdef SIGFOX_EP_SINGLE_FRAME
            sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[((sigfox_ep_bitstream_ctx.ul_payload_size_bytes - 1) / 4) + 2];
#else
            sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_APPLICATION_TABLE[((input->common_parameters).ul_frame_rank)][((sigfox_ep_bitstream_ctx.ul_payload_size_bytes - 1) / 4) + 2];
#endif
            sigfox_ep_bitstream_ctx.ul_li = (sfx_u8) (3 - ((sigfox_ep_bitstream_ctx.ul_payload_size_bytes - 1) % 4));
            sigfox_ep_bitstream_ctx.ul_auth_size_bytes = (sfx_u8) (sigfox_ep_bitstream_ctx.ul_li + 2);
        }
        break;
    default:
#ifdef SIGFOX_EP_ERROR_CODES
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_MESSAGE_TYPE);
#else
        goto errors;
#endif
        break;
    }
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
    bitstream[SIGFOX_EP_BITSTREAM_UL_FT_INDEX + 0] = (sfx_u8) ((sigfox_ep_bitstream_ctx.ul_ft & 0xFF00) >> 8);
    bitstream[SIGFOX_EP_BITSTREAM_UL_FT_INDEX + 1] = (sfx_u8) ((sigfox_ep_bitstream_ctx.ul_ft & 0x00FF) >> 0);
    // LI and BF.
    bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX] = (sfx_u8) (sigfox_ep_bitstream_ctx.ul_li << SIGFOX_EP_BITSTREAM_LI_BIT_INDEX);
#ifdef SIGFOX_EP_BIDIRECTIONAL
    bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX] = (sfx_u8) (bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX] | (((input->bidirectional_flag) & 0x01) << SIGFOX_EP_BITSTREAM_BF_BIT_INDEX));
#endif
    // Message counter.
    bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + 0] |= (sfx_u8) ((((input->common_parameters).message_counter) & 0x0F00) >> 8);
    bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + 1] = (sfx_u8) ((((input->common_parameters).message_counter) & 0x00FF) >> 0);
    // EP-ID (LSByte first).
    for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
        bitstream[SIGFOX_EP_BITSTREAM_EP_ID_INDEX + idx] = ((input->common_parameters).ep_id)[SIGFOX_EP_ID_SIZE_BYTES - 1 - idx];
    }
    (*bitstream_size_bytes) = (SIGFOX_EP_BITSTREAM_EP_ID_INDEX + SIGFOX_EP_ID_SIZE_BYTES);
#ifdef SIGFOX_EP_UL_PAYLOAD_SIZE
#if (SIGFOX_EP_UL_PAYLOAD_SIZE > 0)
    // UL-PAYLOAD (MSByte first).
    for (idx = 0; idx < SIGFOX_EP_UL_PAYLOAD_SIZE; idx++) {
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + idx] = (input -> ul_payload)[idx];
    }
    (*bitstream_size_bytes) = (sfx_u8) ((*bitstream_size_bytes) + SIGFOX_EP_UL_PAYLOAD_SIZE);
#endif
#else /* SIGFOX_EP_UL_PAYLOAD_SIZE */
    // UL-PAYLOAD (MSByte first).
    if ((input->message_type) == SIGFOX_APPLICATION_MESSAGE_TYPE_BYTE_ARRAY) {
        for (idx = 0; idx < sigfox_ep_bitstream_ctx.ul_payload_size_bytes; idx++) {
            bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + idx] = (input->ul_payload)[idx];
        }
    }
    else {
        sigfox_ep_bitstream_ctx.ul_payload_size_bytes = 0; // No payload field for EMPTY and BIT message types.
    }
    (*bitstream_size_bytes) = (sfx_u8) ((*bitstream_size_bytes) + sigfox_ep_bitstream_ctx.ul_payload_size_bytes);
#endif /* SIGFOX_EP_UL_PAYLOAD_SIZE */
    // UL_AUTH.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _aes_encrypt();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_BITSTREAM_SUCCESS);
#else
    _aes_encrypt();
#endif
    (*bitstream_size_bytes) = (sfx_u8) ((*bitstream_size_bytes) + sigfox_ep_bitstream_ctx.ul_auth_size_bytes);
    // CRC.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _add_crc16();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_BITSTREAM_SUCCESS);
#else
    _add_crc16();
#endif
    (*bitstream_size_bytes) = (sfx_u8) ((*bitstream_size_bytes) + SIGFOX_EP_BITSTREAM_UL_CRC_SIZE_BYTES);
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Convolution.
    _convolve((input->common_parameters).ul_frame_rank);
#endif
#if (defined SIGFOX_EP_ERROR_CODES) || !(defined SIGFOX_EP_UL_PAYLOAD_SIZE)
errors:
#endif
    SIGFOX_RETURN();
}
#endif

#if ((defined SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE) || (defined SIGFOX_EP_BIDIRECTIONAL))
/*******************************************************************/
SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_build_control_frame(SIGFOX_EP_BITSTREAM_control_frame_t *input, sfx_u8 *bitstream, sfx_u8 *bitstream_size_bytes) {
    // Local variables.
    sfx_u8 idx = 0;
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_BITSTREAM_status_t status = SIGFOX_EP_BITSTREAM_SUCCESS;
#endif
#if (defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES)
    status = _check_control_parameters(input, bitstream, bitstream_size_bytes);
    SIGFOX_CHECK_STATUS(SIGFOX_EP_BITSTREAM_SUCCESS);
#endif
    // Default values
    sigfox_ep_bitstream_ctx.ul_li = 0;
    sigfox_ep_bitstream_ctx.ul_auth_size_bytes = 2;
    sigfox_ep_bitstream_ctx.bitstream = bitstream;
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
    sigfox_ep_bitstream_ctx.key = (input->common_parameters).ep_key_type;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    sfx_s8 rssi_plus_100 = (sfx_s8) ((input->rssi_dbm) + 100);
#endif
    // UL-PR.
    for (idx = 0; idx < SIGFOX_EP_BITSTREAM_UL_PR_SIZE_BYTES; idx++) {
        bitstream[SIGFOX_EP_BITSTREAM_UL_PR_INDEX + idx] = SIGFOX_EP_BITSTREAM_UL_PR_VALUE;
    }
    // UL-FT.
#ifdef SIGFOX_EP_SINGLE_FRAME
    sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_CONTROL;
#else
    sigfox_ep_bitstream_ctx.ul_ft = SIGFOX_EP_BITSTREAM_UL_FT_CONTROL_TABLE[((input->common_parameters).ul_frame_rank)];
#endif
    bitstream[SIGFOX_EP_BITSTREAM_UL_FT_INDEX + 0] = (sfx_u8) ((sigfox_ep_bitstream_ctx.ul_ft & 0xFF00) >> 8);
    bitstream[SIGFOX_EP_BITSTREAM_UL_FT_INDEX + 1] = (sfx_u8) ((sigfox_ep_bitstream_ctx.ul_ft & 0x00FF) >> 0);
    // Payload size.
    switch (input->message_type) {
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    case SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE:
        sigfox_ep_bitstream_ctx.ul_payload_size_bytes = SIGFOX_EP_BITSTREAM_CONTROL_KEEP_ALIVE_PAYLOAD_SIZE_BYTES;
        break;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case SIGFOX_CONTROL_MESSAGE_TYPE_DL_CONFIRMATION:
        sigfox_ep_bitstream_ctx.ul_payload_size_bytes = SIGFOX_EP_BITSTREAM_CONTROL_DL_CONFIRMATION_PAYLOAD_SIZE_BYTES;
        break;
#endif
    default:
        SIGFOX_EXIT_ERROR(SIGFOX_EP_BITSTREAM_ERROR_MESSAGE_TYPE);
    }
    sigfox_ep_bitstream_ctx.ul_li = (sfx_u8) (3 - ((sigfox_ep_bitstream_ctx.ul_payload_size_bytes - 1) % 4));
    sigfox_ep_bitstream_ctx.ul_auth_size_bytes = (sfx_u8) (sigfox_ep_bitstream_ctx.ul_li + 2);
    bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX] = (sfx_u8) (sigfox_ep_bitstream_ctx.ul_li << SIGFOX_EP_BITSTREAM_LI_BIT_INDEX);
    // Message counter.
    bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + 0] |= (sfx_u8) ((((input->common_parameters).message_counter) & 0x0F00) >> 8);
    bitstream[SIGFOX_EP_BITSTREAM_LI_BF_REP_MC_INDEX + 1] = (sfx_u8) ((((input->common_parameters).message_counter) & 0x00FF) >> 0);
    // EP-ID (LSByte first).
    for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
        bitstream[SIGFOX_EP_BITSTREAM_EP_ID_INDEX + idx] = ((input->common_parameters).ep_id)[SIGFOX_EP_ID_SIZE_BYTES - 1 - idx];
    }
    (*bitstream_size_bytes) = (SIGFOX_EP_BITSTREAM_EP_ID_INDEX + SIGFOX_EP_ID_SIZE_BYTES);
    // UL-PAYLOAD (MSByte first).
    switch (input->message_type) {
#ifdef SIGFOX_EP_CONTROL_KEEP_ALIVE_MESSAGE
    case SIGFOX_CONTROL_MESSAGE_TYPE_KEEP_ALIVE:
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 0] = SIGFOX_EP_BITSTREAM_CONTROL_KEEP_ALIVE_CT;
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 1] = (sfx_u8) (((input->voltage_idle_mv) & 0x00FF) >> 0);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 2] = (sfx_u8) (((input->voltage_idle_mv) & 0xFF00) >> 8);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 3] = (sfx_u8) (((input->voltage_tx_mv) & 0x00FF) >> 0);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 4] = (sfx_u8) (((input->voltage_tx_mv) & 0xFF00) >> 8);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 5] = (sfx_u8) ((((sfx_u16) (input->temperature_tenth_degrees)) & 0x00FF) >> 0);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 6] = (sfx_u8) ((((sfx_u16) (input->temperature_tenth_degrees)) & 0xFF00) >> 8);
        break;
#endif
#ifdef SIGFOX_EP_BIDIRECTIONAL
    case SIGFOX_CONTROL_MESSAGE_TYPE_DL_CONFIRMATION:
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 0] = SIGFOX_EP_BITSTREAM_CONTROL_DL_CONFIRMATION_CT;
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 1] = (sfx_u8) (((input->voltage_idle_mv) & 0x00FF) >> 0);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 2] = (sfx_u8) (((input->voltage_idle_mv) & 0xFF00) >> 8);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 3] = (sfx_u8) (((input->voltage_tx_mv) & 0x00FF) >> 0);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 4] = (sfx_u8) (((input->voltage_tx_mv) & 0xFF00) >> 8);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 5] = (sfx_u8) ((((sfx_u16) (input->temperature_tenth_degrees)) & 0x00FF) >> 0);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 6] = (sfx_u8) ((((sfx_u16) (input->temperature_tenth_degrees)) & 0xFF00) >> 8);
        bitstream[SIGFOX_EP_BITSTREAM_UL_PAYLOAD_INDEX + 7] = (sfx_u8) (rssi_plus_100);
        break;
#endif
    default:
        break;
    }
    (*bitstream_size_bytes) = (sfx_u8) ((*bitstream_size_bytes) + sigfox_ep_bitstream_ctx.ul_payload_size_bytes);
    // UL_AUTH.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _aes_encrypt();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_BITSTREAM_SUCCESS);
#else
    _aes_encrypt();
#endif
    (*bitstream_size_bytes) = (sfx_u8) ((*bitstream_size_bytes) + sigfox_ep_bitstream_ctx.ul_auth_size_bytes);
    // CRC.
#ifdef SIGFOX_EP_ERROR_CODES
    status = _add_crc16();
    SIGFOX_CHECK_STATUS(SIGFOX_EP_BITSTREAM_SUCCESS);
#else
    _add_crc16();
#endif
    (*bitstream_size_bytes) = (sfx_u8) ((*bitstream_size_bytes) + SIGFOX_EP_BITSTREAM_UL_CRC_SIZE_BYTES);
#ifndef SIGFOX_EP_SINGLE_FRAME
    // Convolution.
    _convolve((input->common_parameters).ul_frame_rank);
#endif
errors:
    SIGFOX_RETURN();
}
#endif

#ifdef SIGFOX_EP_BIDIRECTIONAL
/*******************************************************************/
SIGFOX_EP_BITSTREAM_status_t SIGFOX_EP_BITSTREAM_decode_downlink_frame(SIGFOX_EP_BITSTREAM_dl_frame_t *input, sfx_bool *dl_frame_valid, sfx_u8 *dl_payload) {
    // Local variables.
#ifdef SIGFOX_EP_ERROR_CODES
    SIGFOX_EP_BITSTREAM_status_t status = SIGFOX_EP_BITSTREAM_SUCCESS;
    MCU_API_status_t mcu_api_status = MCU_API_SUCCESS;
#ifndef SIGFOX_EP_CRC_HW
    SIGFOX_CRC_status_t sigfox_crc_status = SIGFOX_CRC_SUCCESS;
#endif
#endif
#ifdef SIGFOX_EP_AES_HW
    MCU_API_encryption_data_t encryption_data;
#else
    sfx_u8 aes_key[SIGFOX_EP_KEY_SIZE_BYTES];
#endif
    sfx_u8 idx = 0;
    sfx_u8 byte_idx = 0;
    sfx_u8 local_bitstream[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES + 1];
    sfx_u8 dl_crc;
    sfx_u8 aes_data[SIGFOX_EP_KEY_SIZE_BYTES];
    // Reset result.
    (*dl_frame_valid) = SIGFOX_FALSE;
#if ((defined SIGFOX_EP_PARAMETERS_CHECK) && (defined SIGFOX_EP_ERROR_CODES))
    // Check parameters.
    status = _check_dl_parameters(input, dl_payload);
    SIGFOX_CHECK_STATUS(SIGFOX_EP_BITSTREAM_SUCCESS);
#endif
    // Copy bitstream to local buffer.
    for (idx = 0; idx < SIGFOX_DL_PHY_CONTENT_SIZE_BYTES; idx++) {
        local_bitstream[idx] = input->dl_phy_content[idx];
    }
    local_bitstream[SIGFOX_DL_PHY_CONTENT_SIZE_BYTES] = 0;
#ifdef SIGFOX_EP_CERTIFICATION
    if ((input->dl_decoding_enable) == SIGFOX_TRUE) {
#endif
        // Step 1: de-whitening.
        _dewhitening(input, local_bitstream);
        // Step 2: ECC.
        _decode_bch(local_bitstream);
        // Step 3: CRC check.
#ifdef SIGFOX_EP_CRC_HW
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_compute_crc8(&(local_bitstream[SIGFOX_EP_BITSTREAM_DL_PAYLOAD_INDEX]), (SIGFOX_DL_PAYLOAD_SIZE_BYTES + SIGFOX_EP_BITSTREAM_DL_AUTH_SIZE_BYTES), SIGFOX_EP_BITSTREAM_DL_CRC_POLYNOM, &dl_crc);
        MCU_API_check_status(SIGFOX_EP_BITSTREAM_ERROR_DRIVER_MCU_API);
#else
        MCU_API_compute_crc8(&(local_bitstream[SIGFOX_EP_BITSTREAM_DL_PAYLOAD_INDEX]), (SIGFOX_DL_PAYLOAD_SIZE_BYTES + SIGFOX_EP_BITSTREAM_DL_AUTH_SIZE_BYTES), SIGFOX_EP_BITSTREAM_DL_CRC_POLYNOM, &dl_crc);
#endif
#else
#ifdef SIGFOX_EP_ERROR_CODES
        sigfox_crc_status = SIGFOX_CRC_compute_crc8(&(local_bitstream[SIGFOX_EP_BITSTREAM_DL_PAYLOAD_INDEX]), (SIGFOX_DL_PAYLOAD_SIZE_BYTES + SIGFOX_EP_BITSTREAM_DL_AUTH_SIZE_BYTES), SIGFOX_EP_BITSTREAM_DL_CRC_POLYNOM, &dl_crc);
        SIGFOX_CRC_check_status(SIGFOX_EP_BITSTREAM_ERROR_DRIVER_SIGFOX_CRC);
#else
        SIGFOX_CRC_compute_crc8(&(local_bitstream[SIGFOX_EP_BITSTREAM_DL_PAYLOAD_INDEX]), (SIGFOX_DL_PAYLOAD_SIZE_BYTES + SIGFOX_EP_BITSTREAM_DL_AUTH_SIZE_BYTES), SIGFOX_EP_BITSTREAM_DL_CRC_POLYNOM, &dl_crc);
#endif
#endif
        if (dl_crc != local_bitstream[SIGFOX_EP_BITSTREAM_DL_CRC_INDEX]) goto errors;
        // Step 4: authentication check.
        // Build input data.
        // EP-ID (LSByte first).
        for (idx = 0; idx < SIGFOX_EP_ID_SIZE_BYTES; idx++) {
            aes_data[byte_idx++] = input->ep_id[SIGFOX_EP_ID_SIZE_BYTES - 1 - idx];
        }
        // Message counter.
        aes_data[byte_idx++] = (sfx_u8) (((input->message_counter) >> 0) & 0xFF);
        aes_data[byte_idx++] = (sfx_u8) (((input->message_counter) >> 8) & 0x0F);
        // DL-PAYLOAD.
        for (idx = 0; idx < SIGFOX_DL_PAYLOAD_SIZE_BYTES; idx++) {
            aes_data[byte_idx++] = local_bitstream[SIGFOX_EP_BITSTREAM_DL_PAYLOAD_INDEX + idx];
        }
        // EP-ID (LSByte first).
        aes_data[byte_idx++] = input->ep_id[3];
        aes_data[byte_idx++] = input->ep_id[2];
#ifdef SIGFOX_EP_AES_HW
        // Build input structure.
        encryption_data.data = aes_data;
        encryption_data.data_size_bytes = SIGFOX_EP_KEY_SIZE_BYTES;
#ifdef SIGFOX_EP_PUBLIC_KEY_CAPABLE
        encryption_data.key = (input->ep_key_type);
#endif
        // Perform encryption.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_aes_128_cbc_encrypt(&encryption_data);
        MCU_API_check_status(SIGFOX_EP_BITSTREAM_ERROR_DRIVER_MCU_API);
#else
        MCU_API_aes_128_cbc_encrypt(&encryption_data);
#endif
#else /* SIGFOX_EP_AES_HW */
        // Read device private key.
#ifdef SIGFOX_EP_ERROR_CODES
        mcu_api_status = MCU_API_get_ep_key(aes_key, SIGFOX_EP_KEY_SIZE_BYTES);
        MCU_API_check_status(SIGFOX_EP_BITSTREAM_ERROR_DRIVER_MCU_API);
#else
        MCU_API_get_ep_key(aes_key, SIGFOX_EP_KEY_SIZE_BYTES);
#endif
        // Perform encryption.
        aes_encrypt(aes_data, (sfx_u8*) aes_key);
#endif /* SIGFOX_EP_AES_HW */
        // Compare received and computed DL-AUTH fields.
        if ((local_bitstream[SIGFOX_EP_BITSTREAM_DL_AUTH_INDEX] != aes_data[0]) || (local_bitstream[SIGFOX_EP_BITSTREAM_DL_AUTH_INDEX + 1] != aes_data[1])) {
            goto errors;
        }
#ifdef SIGFOX_EP_CERTIFICATION
    }
#endif
    // Valid frame received: extract DL payload.
    for (idx = 0; idx < SIGFOX_DL_PAYLOAD_SIZE_BYTES; idx++) {
        dl_payload[idx] = local_bitstream[SIGFOX_EP_BITSTREAM_DL_PAYLOAD_INDEX + idx];
    }
    // Update result.
    (*dl_frame_valid) = SIGFOX_TRUE;
errors:
    SIGFOX_RETURN();
}
#endif
