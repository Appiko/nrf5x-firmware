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

#define NO_OF_MONTHS 12

typedef struct
{
    uint32_t log_time;
    time_tracker_ddmmyy_t log_date;
}date_time_log_t;

static uint32_t last_date[] = {0xff, 31, 28, 31, 30, 31, 30, 31, 31, 30, 31, 30, 31};

static time_tracker_ddmmyy_t current_date;

static uint32_t current_time = TIME_TRACKER_TIME_NOT_SET;

static date_time_log_t date_time;

static uint32_t log_id;

void update_date ()
{
    //check if it's a last day of year
    if(current_date.mm == NO_OF_MONTHS &&
       current_date.dd == last_date[[NO_OF_MONTHS]])
    {
    //update year, and check if updated year is leap year
        current_date.yy++;
    //if it's leap year, make feb's last date 29
        last_date[2] = (current_date.yy%4 == 0) ? 29 : 28;
    //set date 01/01/updated year
        current_date.dd = 1;
        current_date.mm = 1;
        return;        
    }

    //check if it's a last day of on going month
    if(current_date.dd == last_date[current_date.mm])
    {
    //if it is, update month
        current_date.mm++;
    //set date 01/update month/running year
        current_date.dd = 1;
        return;
    }
    
    //update date
    current_date.dd++;
    return;    
}

uint32_t time_tracker_init (uint32_t time_log)
{
    log_id = time_log;
    log_config_t log = 
    {
        .log_id = log_id,
        .entry_size = sizeof(date_time_log_t),
        .start_page = TIME_TRACKER_NVM_PAGE_USED_0,
        .no_of_pages = TIME_TRACKER_NVM_NO_PAGES_USED,
        
    };
    //check if log is available
    //if log is already being used check if it contains time track and fetch the
    log_id = nvm_logger_log_init (&log);
    if(nvm_logger_is_log_empty (log_id) == false)
    {
        //last entry from log to current time
        nvm_logger_fetch_tail_data (log_id, &date_time, 1);
        current_time = date_time.log_time;
        memcpy(&current_date, &date_time.log_date, sizeof(time_tracker_ddmmyy_t));
    }
    return log_id;
}

void time_tracker_set_date_time (time_tracker_ddmmyy_t date_ddmmyy, uint32_t time_s)
{
    current_time = MS_TIMER_TICKS_MS(time_s * 1000);
    date_time.log_time = current_time;
    memcpy(&date_time.log_date, &current_date, sizeof(time_tracker_ddmmyy_t));
    nvm_logger_feed_data (log_id, &date_time);
    
}


void time_tracker_update_time (uint32_t ticks)
{
    //update current time
    current_time = (current_time + ticks);
    if(current_time > DAY_TICK_LENGTH)
    {
        update_date ();
        current_time = current_time - DAY_TICK_LENGTH;
    }
    //add current date and time entry in time tracking log
    date_time.log_time = current_time;
    memcpy(&date_time.log_date, &current_date, sizeof(time_tracker_ddmmyy_t));
    nvm_logger_feed_data (log_id, &date_time);
}


uint32_t time_tracker_get_current_time_s ()
{
    return (current_time/MS_TIMER_FREQ);
}
