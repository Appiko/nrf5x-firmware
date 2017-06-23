/*
 *  nrf_util.h
 *
 *  Created on: 07-Feb-2017
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
 * @addtogroup group_util
 * @{
 *
 * @defgroup group_nrf_util nRF specific utilities
 * @brief Helpers specific to the nRF SoCs.
 * @{
 */

#ifndef CODEBASE_NRF_UTIL_H_
#define CODEBASE_NRF_UTIL_H_



/**@brief Priority levels that the application can use based on whether the
 * SoftDevice (SD) is used.
 *
__For nRF51__

| With SD  | Without SD  | Priority |
|----------|-------------|----------|
| SD high  | App highest | 0        |
| App high | App high    | 1        |
| SD low   | App mid     | 2        |
| App low  | App low     | 3        |
| Thread   | Thread      | 4        |

__For nRF52__

| With SD    | Without SD    | Priority |
|------------|---------------|----------|
| SD high    | App highest   | 0        |
| SD mid     | -Not defined- | 1        |
| App high   | App high      | 2        |
| App mid    | -Not defined- | 3        |
| SD low     | App mid       | 4        |
| SD lowest  | -Not defined- | 5        |
| App low    | App low       | 6        |
| App lowest | App lowest    | 7        |
| Thread     | Thread        | 15       |

*/
#if __CORTEX_M == (0x00U)
typedef enum
{
    APP_IRQ_PRIORITY_HIGHEST = 0,
    APP_IRQ_PRIORITY_HIGH    = 1,
    APP_IRQ_PRIORITY_MID     = 2,
    APP_IRQ_PRIORITY_LOW     = 3,
    APP_IRQ_PRIORITY_THREAD  = 4
} app_irq_priority_t;
#elif __CORTEX_M == (0x04U)
typedef enum
{
    APP_IRQ_PRIORITY_HIGHEST = 0,
    APP_IRQ_PRIORITY_HIGH    = 2,
    APP_IRQ_PRIORITY_MID     = 4,
    APP_IRQ_PRIORITY_LOW     = 6,
    APP_IRQ_PRIORITY_LOWEST  = 7,
    APP_IRQ_PRIORITY_THREAD  = 15
} app_irq_priority_t;
#endif

#endif /* CODEBASE_NRF_UTIL_H_ */

/**
 * @}
 * @}
 */
