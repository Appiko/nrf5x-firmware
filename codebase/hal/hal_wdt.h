/**
 *  hal_wdt.h : Watchdog timer HAL
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
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
