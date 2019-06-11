/*
 *  time_tracker.h : Module to keep track of time in MS_TIEMR ticks
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

#ifndef CODEBASE_PERIPHERAL_MODULES_TIME_TRACKER_H_
#define CODEBASE_PERIPHERAL_MODULES_TIME_TRACKER_H_
/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_time_tracker Time tracker module
 *
 * @brief Module to keep track of time in MS_TIMER ticks format. 
 * @{
 */


#include "stdint.h"
#include "ms_timer.h"
#include "nvm_logger.h"

#ifndef MS_TIMER_FREQ
#error "Time Tracker module requires MS Timer module"
#endif

#ifndef TIME_TRACKER_NVM_NO_PAGES_USED 
#define TIME_TRACKER_NVM_NO_PAGES_USED 2
#endif
#ifndef TIME_TRACKER_NVM_PAGE_USED_0 
#define TIME_TRACKER_NVM_PAGE_USED_0 NVM_LOG_PAGE0
#endif

#ifndef TIME_TRACKER_NVM_PAGE_USED_1 
#define TIME_TRACKER_NVM_PAGE_USED_1 NVM_LOG_PAGE1
#endif

#define TIME_TRACKER_TIME_NOT_SET 0xFFFFFFFF

/**
 * @brief Function to initiate the time tracker module.
 * @param time_log NVM Log which is to be used to keep track of time
 * @return NVM Log id which is being used to keep track of time.
 * @retval If given time_log is not available nvm module will assign new log id.
 */
uint32_t time_tracker_init (uint32_t time_log);

/**
 * @brief Function to set current time
 * @param time_ticks Current time in ticks
 */
void time_tracker_set_time (uint32_t time_ticks);

/**
 * @brief Function to update time
 * @param ticks MS Timer Ticks which has to be added current time
 */
void time_tracker_update_time (uint32_t ticks);

/**
 * @brief Function to get check how many time slots are active right now.
 * @return Last time entry in log.
 */
uint32_t time_tracker_get_current_time ();

#endif /* CODEBASE_PERIPHERAL_MODULES_TIME_TRACKER_H_ */

/**
 * @}
 * @}
 */
