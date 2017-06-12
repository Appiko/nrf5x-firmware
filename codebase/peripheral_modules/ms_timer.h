/*
 *  ms_timer.h
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

/** Specify which RTC peripheral would be used for the ms timer module */
#define MS_TIMER_RTC_USED           1

///The number of CC registers in the RTC peripheral used for MS timer
#define MS_TIMER_CC_COUNT           CONCAT_3(RTC, MS_TIMER_RTC_USED, _CC_NUM)

/** The frequency used by the RTC running the ms timer. Must be a power of 2 and at max 32768 Hz */
#define MS_TIMER_FREQ				32768
/* Check if MS_TIMER_FREQ is power of 2 and less than or equal to 32.768 kHz */
#if (((IS_POWER_OF_TWO(MS_TIMER_FREQ)) && (MS_TIMER_FREQ<=LFCLK_FREQ))==false)
#error MS_TIMER_FREQ must be a power of 2 with a maximum frequency of 32768 Hz
#endif
/** Macro to find out the number of RTC ticks for the passed time in milli-seconds */
#define RTC_TICKS_MS(ms)				((uint32_t) ROUNDED_DIV( (MS_TIMER_FREQ*ms) , 1000) )
/** Macro to find out the number of RTC ticks for the passed time in multiples of 0.625 ms */
#define RTC_TICKS_625(ms)				((uint32_t) ROUNDED_DIV( (MS_TIMER_FREQ*ms) , 1600) )
/** Macro to find out the number of RTC ticks for the passed time in multiples of 1.25 ms */
#define RTC_TICKS_1250(ms)				((uint32_t) ROUNDED_DIV( (MS_TIMER_FREQ*ms) , 800) )

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
 * @param ticks 	The number of ticks of the RTC after which the timer expires
 * @param handler 	Pointer to a function which needs to be called when the timer expires
 *
 * @note Starting an already started will restart the timer with the current number of ticks passed.
 * @ref ms_timer_get_on_status can be used to check if a timer is already running.
 */
void ms_timer_start(ms_timer_num id, ms_timer_mode mode, uint32_t ticks, void (*handler)(void));

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

#endif /* CODEBASE_MS_TIMER_H_ */
/**
 * @}
 * @}
 */
