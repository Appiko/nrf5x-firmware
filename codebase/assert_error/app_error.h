/*
 *  app_error.h
 *
 *  Created on: 06-Feb-2017
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

/**
 * @addtogroup group_assert_error
 * @{
 *
 * @defgroup group_app_error Application error handler
 * @brief Handler which prints application error info over Segger RTT channel in DEBUG modes
 * @{
 */

#ifndef CODEBASE_ASSERT_ERROR_APP_ERROR_H_
#define CODEBASE_ASSERT_ERROR_APP_ERROR_H_

#include <stdint.h>
#include <stdbool.h>

/**@brief Function called when an error occurs. If DEBUG flag is present
 *  the error information is printed on RTT, otherwise nothing is done.
 *
 * @param[in] error_code  Error code supplied to the handler.
 * @param[in] line_num    Line number where the handler is called.
 * @param[in] p_file_name Pointer to the file name.
 */
void app_error_handler(uint32_t error_code, uint32_t line_num, const uint8_t * p_file_name);

/**@brief Macro calls error handler if the provided error code is not NRF_SUCCESS.
 *  The behavior of this call depends on whether DEBUG flag is present.
 *
 * @param[in] ERR_CODE Error code supplied to the error handler.
 */
#define APP_ERROR_CHECK(ERR_CODE)                       \
do                                                      \
{                                                       \
    const uint32_t LOCAL_ERR_CODE = (ERR_CODE);         \
    if (LOCAL_ERR_CODE != NRF_SUCCESS)                  \
    {                                                   \
        app_error_handler((ERR_CODE), __LINE__, (uint8_t*) __FILE__);  \
    }                                                   \
} while (0)

#endif /* CODEBASE_ASSERT_ERROR_APP_ERROR_H_ */

/** @} */
/** @} */

