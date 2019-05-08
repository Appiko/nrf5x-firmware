/*
 *  radio_trigger.h : Module to send trigger wireless-ly using radio peripheral.
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


#ifndef RADIO_TRIGGER_H
#define RADIO_TRIGGER_H

#include "stdint.h"
#include "stdbool.h"
#include "nrf_util.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif

#ifndef TIMER_USED_RADIO_TRIGGER 
#define TIMER_USED_RADIO_TRIGGER 0 //
#endif

#ifndef TIMER_CHANNEL_USED_RADIO_TRIGGER_0
#define TIMER_CHANNEL_USED_RADIO_TRIGGER_0 0
#endif

#ifndef TIMER_CHANNEL_USED_RADIO_TRIGGER_1
#define TIMER_CHANNEL_USED_RADIO_TRIGGER_1 1
#endif

#ifndef TIMER_CHANNEL_USED_RADIO_TRIGGER_2
#define TIMER_CHANNEL_USED_RADIO_TRIGGER_2 2
#endif


typedef enum
{
    RADIO_TRIGGER_Tx,
    RADIO_TRIGGER_Rx,
}radio_trigger_dir_t;


typedef struct 
{
    radio_trigger_dir_t comm_direction;
    uint32_t comm_freq;
    uint32_t tx_on_time_ms;
    uint32_t tx_on_freq_us;
    uint32_t rx_on_time_ms;
    app_irq_priority_t irq_priority;
    void (* radio_trigger_tx_callback) (void * data, uint32_t len);
    void (* radio_trigger_rx_callback) (void * data, uint32_t len);
    
}radio_trigger_init_t;

void radio_trigger_init (radio_trigger_init_t * radio_trig_init); 

void radio_trigger_yell ();

void radio_trigger_listen ();

void radio_trigger_shut ();

void radio_trigger_memorize_data (void * data, uint32_t len);

bool is_radio_trigger_availabel ();
#endif /* RADIO_TRIGGER_H */
