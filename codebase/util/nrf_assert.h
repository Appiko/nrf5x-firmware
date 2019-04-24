/**
 *  nrf_assert.h : static and runtime assertion
 *  Copyright (C) 2019  Appiko
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef CODEBASE_ASSERT_ERROR_NRF_ASSERT_H_
#define CODEBASE_ASSERT_ERROR_NRF_ASSERT_H_

/**
 * @addtogroup group_util
 * @{
 *
 * @defgroup group_assertion Static and runtime assertion
 * @brief Handler which prints runtime assertion info over the configuted
 *      logging means and includes assert.h for static assertion wrapper.
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
