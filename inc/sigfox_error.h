/*!*****************************************************************
 * \file    sigfox_error.h
 * \brief   Sigfox error driver.
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

#ifndef __SIGFOX_ERROR_H__
#define __SIGFOX_ERROR_H__

#ifndef SIGFOX_EP_DISABLE_FLAGS_FILE
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

#ifdef SIGFOX_EP_ERROR_STACK

/*** SIGFOX ERROR structures ***/

/*!******************************************************************
 * \enum SIGFOX_ERROR_source_t
 * \brief Sigfox library low level errors sources.
 *******************************************************************/
typedef enum {
    SIGFOX_ERROR_SOURCE_NONE = 0,
    // Library.
    SIGFOX_ERROR_SOURCE_SIGFOX_EP_API,
    // Internal library drivers.
    SIGFOX_ERROR_SOURCE_SIGFOX_EP_BITSTREAM,
    SIGFOX_ERROR_SOURCE_SIGFOX_CRC,
    SIGFOX_ERROR_SOURCE_SIGFOX_EP_FREQUENCY,
    SIGFOX_ERROR_SOURCE_SIGFOX_TX_CONTROL,
    // Manufacturer functions.
    SIGFOX_ERROR_SOURCE_MCU_API,
    SIGFOX_ERROR_SOURCE_RF_API,
    // HW functions.
    SIGFOX_ERROR_SOURCE_HW_API,
    // Last index.
    SIGFOX_ERROR_SOURCE_LAST
} SIGFOX_ERROR_source_t;
#endif

#ifdef SIGFOX_EP_ERROR_STACK
/*!******************************************************************
 * \struct SIGFOX_ERROR_t
 * \brief Sigfox library low level error.
 *******************************************************************/
typedef struct {
    SIGFOX_ERROR_source_t source;
    sfx_u32 code;
} SIGFOX_ERROR_t;
#endif

/*** SIGFOX ERROR functions ***/

#ifdef SIGFOX_EP_ERROR_STACK
/*!******************************************************************
 * \fn void SIGFOX_ERROR_init(void)
 * \brief Init error stack.
 * \param[in]   none
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void SIGFOX_ERROR_init(void);
#endif

#ifdef SIGFOX_EP_ERROR_STACK
/*!******************************************************************
 * \fn void SIGFOX_ERROR_stack(SIGFOX_ERROR_source_t source, sfx_u32 code)
 * \brief Store a new error in the internal stack.
 * \param[in]   source: Error source.
 * \param[in]   code: Error code.
 * \param[out]  none
 * \retval      none
 *******************************************************************/
void SIGFOX_ERROR_stack(SIGFOX_ERROR_source_t source, sfx_u32 code);
#endif

#ifdef SIGFOX_EP_ERROR_STACK
/*!******************************************************************
 * \fn  void SIGFOX_ERROR_unstack(SIGFOX_ERROR_t *error_ptr)
 * \brief Read and clear the last (newest) error stored in the internal error stack.
 * \brief This function can be called multiple times to unstack all errors which previously occurred during library execution, until it returns SIGFOX_EP_API_SUCCESS.
 * \param[in]   none
 * \param[out]  error_ptr: Pointer that will contain the last error in the stack.
 * \retval      none
 *******************************************************************/
void SIGFOX_ERROR_unstack(SIGFOX_ERROR_t *error_ptr);
#endif

#endif /* __SIGFOX_ERROR_H__ */
