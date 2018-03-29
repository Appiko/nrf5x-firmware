/*
 *  hal_wdt.h
 *
 *  Created on: 30-Jan-2018
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

#ifndef CODEBASE_HAL_HAL_WDT_H_
#define CODEBASE_HAL_HAL_WDT_H_
/**
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_wdt_driver Watchdog timer HAL
 * @brief Hardware abstraction layer the watchdog timer peripheral in nRF5x SoCs
 *
 * @{
 */

#include <stdint.h>

/**
 * Initialize the WDT peripheral
 * @param period_ms The period in ms after which the WDT bites if not fed
 * @param wdt_timeout_handler The handler to call in case WDT bites.
 * @note The SoC will reset about 60 us after the handler is called. This 
 * handler can be used to do general house keeping before the reset.
 */
void hal_wdt_init(uint32_t period_ms, void (*wdt_timeout_handler)(void));

/**
 * Starts the WDT. 
 * @warning The WDT cannot be stopped once started.
 */
void hal_wdt_start(void);

/**
 * Feeds the WDT so that it does not bite.
 * @note The WDT feeding should preferably be done in the main thread level,
 * not in an interrupt routine. 
 */
void hal_wdt_feed(void);

#endif /* CODEBASE_HAL_HAL_WDT_H_ */
/**
 * @}
 * @}
 */
