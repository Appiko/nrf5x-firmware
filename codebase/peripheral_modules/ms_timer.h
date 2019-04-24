/**
 *  ms_timer.h : Millisecond timer
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
 * @defgroup group_ms_timer Millisecond timer
 * @brief Driver to use milli-second timers using the RTC peripheral
 *
 * @warning This module needs the LFCLK to be on and running to be able to work
 * @{
 */

#ifndef CODEBASE_MS_TIMER_H_
#define CODEBASE_MS_TIMER_H_

#include <stdint.h>
#include <stdbool.h>
#include "common_util.h"
#include "nrf_peripherals.h"
#include "nrf.h"
#include "hal_clocks.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef RTC_USED_MS_TIMER 
#define RTC_USED_MS_TIMER 1
#endif

/** Specify which RTC peripheral would be used for the ms timer module */
#define MS_TIMER_RTC_USED           RTC_USED_MS_TIMER

///The number of CC registers in the RTC peripheral used for MS timer
#define MS_TIMER_CC_COUNT           CONCAT_3(RTC, MS_TIMER_RTC_USED, _CC_NUM)

/** The frequency used by the RTC running the ms timer. Must be a power of 2 and at max 32768 Hz */
#ifndef MS_TIMER_FREQ
#define MS_TIMER_FREQ               32768
#endif

/* Check if MS_TIMER_FREQ is between 8 Hz and 32768 Hz as these are valid RTC prescalar values */
#if ((MS_TIMER_FREQ < 8) || (MS_TIMER_FREQ > LFCLK_FREQ))
#error MS_TIMER_FREQ value should be between 8 and 32768
#endif

/** Macro to find out the rounded number of MS_TIMER ticks for the passed time in milli-seconds */
#define MS_TIMER_TICKS_MS(ms)                ((uint32_t) ROUNDED_DIV( (MS_TIMER_FREQ*(uint64_t)(ms)) , 1000) )

/** @brief Enumeration used for specifying the timers that can be used with a RTC peripheral
 */
typedef enum {
	MS_TIMER0,  //!< Millisecond Timer 0
	MS_TIMER1,  //!< Millisecond Timer 1
	MS_TIMER2,  //!< Millisecond Timer 2
#if (MS_TIMER_CC_COUNT == 4)
	MS_TIMER3,  //!< Millisecond Timer 3
#endif
	MS_TIMER_MAX//!< Not a timer, just used to find the number of timers
}ms_timer_num;

/**
 * @brief Enumeration to specify the mode of operation of the timer
 */
typedef enum {
	MS_SINGLE_CALL, 	//!< One shot call of the timer
	MS_REPEATED_CALL	//!< Repeated call of the timer
}ms_timer_mode;

/**
 * Initialize the RTC peripheral of ID @ref MS_TIMER_RTC_USED to use
 *  as a milli-second timer.
 * @param irq_priority The priority of the interrupt level at which the callback function is called
 * @warning LFCLK should be initialized before ms timer module is initialized.
 *  Use @ref lfclk_init for this.
 */
void ms_timer_init(uint32_t irq_priority);

/**
 * Start a milli-second timer
 * @param id		ID of the timer to be used from @ref ms_timer_num
 * @param mode  	Mode of the timer as specified in @ref ms_timer_mode
 * @param ticks 	The number of ticks at @ref MS_TIMER_FREQ after which the timer expires
 * @param handler 	Pointer to a function which needs to be called when the timer expires
 *
 * @note Starting an already started will restart the timer with the current number of ticks passed.
 * @ref ms_timer_get_on_status can be used to check if a timer is already running.
 */
void ms_timer_start(ms_timer_num id, ms_timer_mode mode, uint64_t ticks, void (*handler)(void));

/**
 * Stop a milli-second timer
 * @param id		ID of the timer to be stopped
 *
 * @note Stopping an already stopped timer will not be an issue
 */
void ms_timer_stop(ms_timer_num id);

/**
 * Returns if a timer is on
 * @param id		ID of timer being enquired
 * @return			Boolean value indicating if a timer is ON
 */
bool ms_timer_get_on_status(ms_timer_num id);

/**
 * @brief Return the current count (24 bit) of the RTC timer used
 * @return The 24 bit count value. The most significant byte is 0.
 * @note This operation takes 5 CPU cycles (ref: nRF5x reference manual)
 */
inline uint32_t ms_timer_get_current_count(void){
  return CONCAT_2(NRF_RTC,MS_TIMER_RTC_USED)->COUNTER;
}

#endif /* CODEBASE_MS_TIMER_H_ */
/**
 * @}
 * @}
 */
