/*
 *  us_timer.h
 *
 *  Created on: 26-Jul-2017
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

/** Specify which TIMER peripheral would be used for the us timer module */
#define US_TIMER_USED           3

///The number of CC registers in the TIMER peripheral used for us timer
#define US_TIMER_CC_COUNT       CONCAT_3(TIMER, US_TIMER_USED, _CC_NUM)

/**
 * @enum timer_num
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
 * @enum timer_mode
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

