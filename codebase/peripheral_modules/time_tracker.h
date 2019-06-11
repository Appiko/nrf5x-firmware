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

typedef struct
{
    uint8_t dd;
    uint8_t mm;
    uint8_t yy;
}time_tracker_ddmmyy_t;

/**
 * @brief Function to initiate the time tracker module.
 * @param time_log NVM Log which is to be used to keep track of time
 * @return NVM Log id which is being used to keep track of time.
 * @retval If given time_log is not available nvm module will assign new log id.
 */
uint32_t time_tracker_init (uint32_t time_log);

/**
 * @brief Function to set current time
 * @param date_yymmdd Structure pointer of data type @ref time_tracker_ddmmyy_t
 * which contains current date.
 * @param time_s Current time in seconds
 */
void time_tracker_set_date_time (time_tracker_ddmmyy_t * p_date_ddmmyy, uint32_t time_s);

/**
 * @brief Function to update time
 * @param ticks MS Timer Ticks which has to be added current time
 */
void time_tracker_update_time (uint32_t ticks);

/**
 * @brief Function to get current time in ms.
 * @return Last time entry in log.
 */
uint32_t time_tracker_get_current_time_s ();

/**
 * @brief Function to get current date
 * @return Structure pointer of data type @ref time_tracker_ddmmyy_t which 
 * stores current date
 */
time_tracker_ddmmyy_t * time_tracker_get_current_date ();

#endif /* CODEBASE_PERIPHERAL_MODULES_TIME_TRACKER_H_ */

/**
 * @}
 * @}
 */
