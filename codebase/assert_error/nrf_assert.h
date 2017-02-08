/*
 *  nrf_assert.h
 *
 *  Created on: 16-Jan-2017
 *
 *  Copyright (c) 2017, Appiko
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software without
 *  specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CODEBASE_ASSERT_ERROR_NRF_ASSERT_H_
#define CODEBASE_ASSERT_ERROR_NRF_ASSERT_H_

/**
 * @addtogroup group_assert_error
 * @{
 *
 * @defgroup group_assertion Static and runtime assertion
 * @brief Handler which prints runtime assertion info over Segger RTT channel
 *      and includes assert.h for static assertion wrapper.
 * @{
 */

#include "stdint.h"
#include "stdbool.h"
#include "assert.h"
#include "common_util.h"

/** @brief Function called for handling runtime assertions.
 *     It prints the assertion log and stays in an infinite loop.
 *
 * @param line_num The line number where the assertion is called
 * @param file_name Pointer to the file name
 */
void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name);

/** @brief Macro for runtime assertion of an expression. If the
 *      expression is false the @ref assert_nrf_callback function
 *      is called to log the event. */
#define ASSERT(expression)        if ((expression) == false)              \
{                                                                         \
    assert_nrf_callback((uint16_t)__LINE__, (uint8_t *)__FILE__);         \
}

#endif /* CODEBASE_ASSERT_ERROR_NRF_ASSERT_H_ */

/** @} */
/** @} */
