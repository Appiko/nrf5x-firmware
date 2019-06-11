/*
 *  oper_manage.c : Module to keep track of operation conditions for a device
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

#include "oper_manage.h"
#include "simple_adc.h"

typedef enum
{
    AMBI_LIGHT,
    TIME_OF_DAY,
}oper_list_t;

static uint32_t light_sense_pin;

static uint32_t arr_start_cond [OPER_MANAGE_MAX_SLOTS];
static uint32_t arr_end_cond [OPER_MANAGE_MAX_SLOTS];

static oper_list_t arr_oper_cond [OPER_MANAGE_MAX_SLOTS];

volatile uint8_t active_slots = OPER_MANAGE_INVALID_SLOTS;

void oper_manage_set_light_sense_pin (uint32_t light_sense_adc)
{
    light_sense_pin = light_sense_adc;
    
}


void oper_manage_set_slot (oper_manage_slot_t * new_slot)
{
    if(new_slot->slot_no < OPER_MANAGE_MAX_SLOTS)
    {
        arr_start_cond[new_slot->slot_no] = new_slot->start_cond;
        arr_end_cond[new_slot->slot_no] = new_slot->end_cond;
        uint32_t current_time = time_tracker_get_current_time();
        if(current_time >=  new_slot->start_cond 
           && current_time <= new_slot->end_cond)
        {
            active_slots |= 1 << new_slot->slot_no;
        }
    }
}

uint8_t oper_manage_check_update ()
{
    uint32_t current_time = time_tracker_get_current_time();
    uint32_t light_val = simple_adc_get_value (SIMPLE_ADC_GAIN1_6, light_sense_pin);
    for(uint32_t slot_no = 0; slot_no < OPER_MANAGE_MAX_SLOTS; slot_no++)
    {
        if(arr_start_cond[slot_no] == arr_end_cond[slot_no])
        {
            active_slots &= (0xFF & (0 << slot_no));
        }
        else if(arr_oper_cond[slot_no] == TIME_OF_DAY)
        {
            if(current_time >= arr_start_cond[slot_no] 
               && current_time <= arr_end_cond[slot_no])
            {
                active_slots |= 1 << slot_no;
            }
            else
            {
                active_slots &= (0xFF & (0 << slot_no));
            }
        }
        else if(arr_oper_cond[slot_no] == AMBI_LIGHT)
        {
            if(light_val >= arr_start_cond[slot_no] 
               && light_val <= arr_end_cond[slot_no])
            {
                active_slots |= 1 << slot_no;
            }
            else
            {
                active_slots &= (0xFF & (0 << slot_no));
            }            
        }
    }
    return active_slots;
}

uint8_t oper_manage_get_active_slots ()
{
    return active_slots;
}
