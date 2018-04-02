/*
 *  app_error.h
 *
 *  Created on: 31-Mar-2018
 *
 *  Copyright (c) 2018, Appiko
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
 * @addtogroup group_sd_assist
 * @{
 *
 * @defgroup group_app_error Application error handler
 * @brief Handler which prints SoftDevice application error info over
 *  the configured logging channel when DEBUG flag is present
 * @{
 */

#ifndef CODEBASE_SD_ASSIST_APP_ERROR_H_
#define CODEBASE_SD_ASSIST_APP_ERROR_H_

#include <stdint.h>
#include <stdbool.h>

#ifndef SOFTDEVICE_PRESENT
#error "App Error check is meant to be used with a SoftDevice"
#endif

#include "nrf_error.h"

/**@brief Function called when an error occurs. If DEBUG flag is present
 *  the error information is printed on log_printf, otherwise nothing is done.
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

/**
 * @brief Callback to be invoked in case of fault i.e. unrecoverable errors
 *  occurring within the application or SoftDevice. The only way to recover
 *  is through a reset.
 * @param id Fault identifier
 * @param pc The program counter of the instruction that triggered the fault.
 * @param info Optional additional information regarding the fault.
 */
void app_error_fault_handler(uint32_t id, uint32_t pc, uint32_t info);

#endif /* CODEBASE_SD_ASSIST_APP_ERROR_H_ */

/** @} */
/** @} */

