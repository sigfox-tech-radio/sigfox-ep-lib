/*!*****************************************************************
 * \file    sigfox_error.c
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

#include "sigfox_error.h"

#ifdef USE_SIGFOX_EP_FLAGS_H
#include "sigfox_ep_flags.h"
#endif
#include "sigfox_types.h"

#ifdef ERROR_STACK
/*** SIGFOX EP API local structures ***/

/*******************************************************************/
typedef struct {
	SIGFOX_ERROR_t error_stack[ERROR_STACK];
	sfx_u32 error_stack_idx;
} SIGFOX_ERROR_context_t;
#endif

#ifdef ERROR_STACK
/*** SIGFOX EP API local global variables ***/

static SIGFOX_ERROR_context_t sigfox_error_ctx;
#endif

/*** SIGFOX ERROR functions ***/

#ifdef ERROR_STACK
/*******************************************************************/
void SIGFOX_ERROR_init(void) {
	// Reset all errors.
	for (sigfox_error_ctx.error_stack_idx=0 ; sigfox_error_ctx.error_stack_idx<ERROR_STACK ; sigfox_error_ctx.error_stack_idx++) {
		sigfox_error_ctx.error_stack[sigfox_error_ctx.error_stack_idx].source = SIGFOX_ERROR_SOURCE_NONE;
		sigfox_error_ctx.error_stack[sigfox_error_ctx.error_stack_idx].code = 0;
	}
	// Reset index.
	sigfox_error_ctx.error_stack_idx = 0;
}
#endif

#ifdef ERROR_STACK
/*******************************************************************/
void SIGFOX_ERROR_stack(SIGFOX_ERROR_source_t source, sfx_u32 code) {
	// Store new error.
	sigfox_error_ctx.error_stack[sigfox_error_ctx.error_stack_idx].source = source;
	sigfox_error_ctx.error_stack[sigfox_error_ctx.error_stack_idx].code = code;
	// Increment index.
	sigfox_error_ctx.error_stack_idx++;
	if (sigfox_error_ctx.error_stack_idx >= ERROR_STACK) {
		sigfox_error_ctx.error_stack_idx = 0;
	}
}
#endif

#ifdef ERROR_STACK
/*******************************************************************/
void SIGFOX_ERROR_unstack(SIGFOX_ERROR_t *error_ptr) {
	// Set index to last error.
	if (sigfox_error_ctx.error_stack_idx > 0) {
		sigfox_error_ctx.error_stack_idx--;
	}
	else {
		sigfox_error_ctx.error_stack_idx = (ERROR_STACK - 1);
	}
	// Read last error.
	(error_ptr -> source) = sigfox_error_ctx.error_stack[sigfox_error_ctx.error_stack_idx].source;
	(error_ptr -> code) = sigfox_error_ctx.error_stack[sigfox_error_ctx.error_stack_idx].code;
	// Clear error.
	sigfox_error_ctx.error_stack[sigfox_error_ctx.error_stack_idx].source = SIGFOX_ERROR_SOURCE_NONE;
	sigfox_error_ctx.error_stack[sigfox_error_ctx.error_stack_idx].code = 0;
}
#endif
