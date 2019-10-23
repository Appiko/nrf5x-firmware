/**
 *  profiler_timer.h : Profiler timer
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
 * @defgroup group_profiler_timer Profiler timer
 * @brief Driver to use timer for code profiling and time-stamping
 *
 * @warning Verify in the nrf5xxxx_peripheral.h file that the timer used
 *  can work up to 32 bit resolution
 * @{
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_PROFILER_TIMER_H_
#define CODEBASE_PERIPHERAL_MODULES_PROFILER_TIMER_H_

#include <stdint.h>
#include "stdbool.h"
#include "nrf.h"
#include "log.h"
#include "common_util.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef TIMER_USED_PROFILE_TIMER 
#define TIMER_USED_PROFILE_TIMER 0
#endif

/** Specify which timer would be used for the profiler timer module */
#define PROFILER_TIMER  CONCAT_2(NRF_TIMER,TIMER_USED_PROFILE_TIMER)

/** Print the current time in micro-seconds from the startup (beginning of TIMER0).
 * This is used for time stamping at different parts in the code
 * @todo Make the @ref PRINT_TIME self configuring based on prescalar used */
#define PRINT_TIME      do{ PROFILER_TIMER->TASKS_CAPTURE[3] = 1; \
                        printfcomma(PROFILER_TIMER->CC[3]/16); \
                        log_printf("us\n"); }while(0)

/** @anchor profile-start-stop
 * @name Definitions for marking the beginning and ending of code to profile
 * @todo Make the @ref PROFILE_START and @ref PROFILE_STOP self configuring based on prescalar used
 * @{*/
/** Point of start of profiling */
#define PROFILE_START   do{ PROFILER_TIMER->TASKS_CAPTURE[2] = 1; }while(0)
/** Point of end of profiling. The time from start is displayed in nano-seconds with an accuracy of 62.5 ns */
#define PROFILE_STOP    do{ PROFILER_TIMER->TASKS_CAPTURE[3] = 1; \
                        printfcomma((PROFILER_TIMER->CC[3] - PROFILER_TIMER->CC[2])/16);      \
                        log_printf(".%03d",(int)((((PROFILER_TIMER->CC[3] - PROFILER_TIMER->CC[2]) & 0x0F)*125)/2)); \
                        log_printf("us\n"); }while(0)
/** @} */

/** Initialize the timer to run at 16 MHz and with full 32 bit width */
void profiler_timer_init(void);

/**
 * Takes in an unsigned integer and prints it with a ',' after every three digits
 * @param num   Number to be printed with commas
 */
void printfcomma(uint32_t num);

/**
 * The time from start-up in micro-seconds
 * @return Time from start, the four most significant bits are always zero
 * @todo Take care of overflow of timer
 * @warning The time overflows after 268 seconds
 */
inline uint32_t read_time_us(void)
{
    PROFILER_TIMER->TASKS_CAPTURE[3] = 1;
    return (PROFILER_TIMER->CC[3] / 16);
}

/**
 * Check if the profiler timer is already initialized
 * @return bool value indicating the on state if profiler timer
 */
bool profiler_timer_is_on(void);

/** @brief Fully stop the profiling timer to save power. @ref profiler_timer_init
 *      needs to be called again before using it.
 */
void profiler_timer_deinit();

#endif /* CODEBASE_PERIPHERAL_MODULES_PROFILER_TIMER_H_ */
/**
 * @}
 * @}
 */
