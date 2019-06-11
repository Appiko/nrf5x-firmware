/*
 *  time_tracker.c : Module to keep track of time in MS_TIEMR ticks
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

#include "time_tracker.h"



#define DAY_TICK_LENGTH (24 * 3600 * MS_TIMER_TICKS_MS(1000))


static uint32_t current_time = TIME_TRACKER_TIME_NOT_SET;

static uint32_t log_id;

uint32_t time_tracker_init (uint32_t time_log)
{
    log_id = time_log;
    log_config_t log = 
    {
        .log_id = log_id,
        .entry_size = sizeof(uint32_t),
        .start_page = TIME_TRACKER_NVM_PAGE_USED_0,
        .no_of_pages = TIME_TRACKER_NVM_NO_PAGES_USED,
        
    };
    //check if log is available
    //if log is already being used check if it contains time track and fetch the
    log_id = nvm_logger_log_init (&log);
    if(nvm_logger_is_log_empty (log_id) == false)
    {
        //last entry from log to current time
        nvm_logger_fetch_tail_data (log_id, &current_time, 1);
    }
    return log_id;
}

void time_tracker_set_time (uint32_t time_ticks)
{
    current_time = time_ticks;
    nvm_logger_feed_data (log_id, &current_time);
    
}


void time_tracker_update_time (uint32_t ticks)
{
    //update current time
    current_time = (current_time + ticks) % DAY_TICK_LENGTH;
    //add current time entry in time tracking log
    nvm_logger_feed_data (log_id, &current_time);
}


uint32_t time_tracker_get_current_time ()
{
    //return active_slots
    return current_time;
}
