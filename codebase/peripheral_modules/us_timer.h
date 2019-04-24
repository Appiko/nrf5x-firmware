/**
 *  us_timer.h : Microsecond timer
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

/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_us_timer Microsecond timer
 * @brief Driver to use micro-second timers using the TIMER peripheral
 *
 * @note This module utilizes the PCLK1M clock for its functioning
 * @{
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_US_TIMER_H_
#define CODEBASE_PERIPHERAL_MODULES_US_TIMER_H_

#include <stdint.h>
#include <stdbool.h>
#include "nrf.h"
#include "nrf_peripherals.h"
#include "common_util.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef TIMER_USED_US_TIMER 
#define TIMER_USED_US_TIMER 3
#endif


/** Specify which TIMER peripheral would be used for the us timer module */
#define US_TIMER_USED           TIMER_USED_US_TIMER

///The number of CC registers in the TIMER peripheral used for us timer
#define US_TIMER_CC_COUNT       CONCAT_3(TIMER, US_TIMER_USED, _CC_NUM)

/**
 * @brief Enumeration used for specifying the timers that can be used with this TIMER peripheral
 */
typedef enum {
    US_TIMER0,  //!< Microsecond Timer 0
    US_TIMER1,  //!< Microsecond Timer 1
    US_TIMER2,  //!< Microsecond Timer 2
    US_TIMER3,  //!< Microsecond Timer 3
#if (US_TIMER_CC_COUNT == 6)
    US_TIMER4,  //!< Microsecond Timer 4
    US_TIMER5,  //!< Microsecond Timer 5
#endif
    US_TIMER_MAX//!< Not a timer, just used to find the number of timers
}us_timer_num;

/**
 * @brief Enumeration to specify the mode of operation of the timer
 */
typedef enum {
    US_SINGLE_CALL, //!< One shot call of the timer
    US_REPEATED_CALL//!< Repeated call of the timer
}us_timer_mode;

/**
 * @brief Initialize the TIMER peripheral to use as a micro-second timer.
 * @param irq_priority The priority of the interrupt level at which the callback
 * function is called
 */
void us_timer_init(uint32_t irq_priority);

/**
 * Start a micro-second timer
 * @param id        ID of the timer to be used from @ref us_timer_num
 * @param mode      Mode of the timer as specified in @ref us_timer_mode
 * @param time_us   The number microseconds after which the timer expires
 * @param handler   Pointer to a function which needs to be called when the timer expires
 *
 * @note Starting an already started will restart the timer with the current number of ticks passed.
 * @ref us_timer_get_on_status can be used to check if a timer is already running.
 */
void us_timer_start(us_timer_num id, us_timer_mode mode, uint32_t time_us, void (*handler)(void));

/**
 * Stop a micro-second timer
 * @param id        ID of the timer to be stopped
 *
 * @note Stopping an already stopped timer will not be an issue
 */
void us_timer_stop(us_timer_num id);

/**
 * Returns if a timer is on
 * @param id        ID of timer being enquired
 * @return          Boolean value indicating if a timer is ON
 */
bool us_timer_get_on_status(us_timer_num id);

#endif /* CODEBASE_PERIPHERAL_MODULES_US_TIMER_H_ */
/**
 * @}
 * @}
 */

