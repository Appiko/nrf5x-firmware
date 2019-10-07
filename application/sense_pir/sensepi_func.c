/*
 *  sensepi_func.c : File to handle sensing functionality of sensepi
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

#include "stdbool.h"
#include "sensepi_func.h"
#include "sensepi_ble.h"
#include "sensepi_store_config.h"

#include "pir_sense.h"
#include "mcp4012_x.h"
#include "cam_trigger.h"
#include "radio_trigger.h"
#include "time_tracker.h"
#include "oper_manage.h"
#include "ms_timer.h"


#define FEEDBACK_TIME MS_TIMER_TICKS_MS(300 * 1000)


typedef struct
{
    uint32_t amplification;
    uint32_t threshold;
}motion_sensitivity_t;

typedef enum
{
    DISABLE,
    ENABLE,
}mod_state_t;

sensepi_ble_config_t g_ble_settings;
/** Variable to store current active setting for PIR sensing */
settings_list_t g_motion_current_settings = SETTINGS0;
settings_list_t g_motion_prev_settings = SETTINGS0;

/** Variable to store current active setting for timer trigger */
settings_list_t g_timer_current_settings = SETTINGS1;
settings_list_t g_timer_prev_settings = SETTINGS1;

/** Array to store sensitivity values for PIR sensing */
motion_sensitivity_t g_arr_motion_sensitivity[MAX_SETTINGS];

/** Array to store timer values for Timer triggering */
uint32_t g_arr_timer_value[MAX_SETTINGS];

static bool g_arr_radio_trig[MAX_SETTINGS];

static uint8_t prev_setting_flags = 0;

static sensepi_func_clk_t g_current_clk_src;

/** Array to store state of each trigger : [pir_timer] : {OFF,ON} */
static mod_state_t g_arr_mod_state[MOTION_AND_TIMER];


/** Function to assign variables */
void assign_varaibles ();


/** Function to handle setting switch */
void switch_setting (uint8_t setting_flags);

void feedback_add_ticks (uint32_t ticks);

static bool g_is_feedback_on = false;
static uint32_t g_feedback_ticks = 0;

/**Functions */
/** Motion related functions */
void mod_motion_start (void);

void mod_motion_switch_setting ();

void mod_motion_add_ticks (uint32_t ticks);

void mod_motion_stop (void);

void mod_motion_switch_clk (sensepi_func_clk_t clk);

/** Timer related functions */
void mod_timer_start (void);

void mod_timer_switch_setting ();

void mod_timer_add_ticks (uint32_t ticks);

void mod_timer_stop (void);

/** Radio related functions */
void mod_radio_start (void);

void mod_radio_switch_setting (settings_list_t setting);

void mod_radio_stop (void);

void feedback_add_ticks (uint32_t ticks)
{
    if(g_feedback_ticks < FEEDBACK_TIME)
    {
        g_feedback_ticks += ticks;
        g_is_feedback_on = true;
    }
    else
    {
        g_is_feedback_on = false;
    }
}

void assign_cam_mode_var (uint32_t index, cam_trigger_list_t mode)
{
    switch (mode)
    {
        case CAM_TRIGGER_SINGLE_SHOT:
            break;
        case CAM_TRIGGER_MULTI_SHOT:
            break;
        case CAM_TRIGGER_LONG_PRESS:
            break;
        case CAM_TRIGGER_VIDEO:
            break;
        case CAM_TRIGGER_HALF_PRESS:
            break;
        case CAM_TRIGGER_NO_PRESS:
            break;
    }
        
}

void assign_oper_slots (uint32_t index)
{
    oper_manage_slot_t l_slot;
    l_slot.slot_no = index;
    
    if(g_ble_settings.generic_settings[index].trig_sel == MOTION_ONLY)
    {
        l_slot.oper_sel =
            g_ble_settings.trigger_oper_cond_sel[MOTION_ONLY];

    }
    else if(g_ble_settings.generic_settings[index].trig_sel == TIMER_ONLY)
    {
        l_slot.oper_sel =
            g_ble_settings.trigger_oper_cond_sel[TIMER_ONLY];
    }
    //in union start time and start light conditions shares the memory
    l_slot.start_cond =
        g_ble_settings.generic_settings[index].cam_setting.oper_cond.time_cond.start_time;

    //in union end time and end light conditions shares the memory
    l_slot.end_cond = 
        g_ble_settings.generic_settings[index].cam_setting.oper_cond.time_cond.end_time;
    oper_manage_set_slot (&l_slot);
}

void assign_varaibles ()
{
    cam_trigger_config_t l_cam_trig;
    for(uint32_t i = 0; i < MAX_SETTINGS; i++)
    {
        g_arr_radio_trig[i] = g_ble_settings.generic_settings[i].cam_setting.cam_trigger.radio_trig_en;
        
        l_cam_trig.setup_number = i;
        
        l_cam_trig.trig_mode =
            g_ble_settings.generic_settings[i].cam_setting.cam_trigger.mode;
        
        
        l_cam_trig.pre_focus_en =
            g_ble_settings.generic_settings[i].cam_setting.cam_trigger.pre_focus_en;
        
        l_cam_trig.prf_press_duration_100ms = 
            g_ble_settings.generic_settings[i].cam_setting.cam_trigger.prf_pulse_duration_100ms;
        
        l_cam_trig.trig_press_duration_100ms = 
            g_ble_settings.generic_settings[i].cam_setting.cam_trigger.trig_pulse_duration_100ms;

        if(g_ble_settings.generic_settings[i].trig_sel == MOTION_ONLY)
        {
            l_cam_trig.trig_duration_100ms = 
                g_ble_settings.generic_settings[i].func_setting.detection_func.inter_trig_time;
            
            g_arr_motion_sensitivity[i].amplification = 
                g_ble_settings.generic_settings[i].func_setting.detection_func.amplification;
            
            g_arr_motion_sensitivity[i].threshold = 
                g_ble_settings.generic_settings[i].func_setting.detection_func.threshold;
        }
        else if(g_ble_settings.generic_settings[i].trig_sel == TIMER_ONLY)
        {
            l_cam_trig.trig_duration_100ms = 0;
            
            g_arr_timer_value[i] =
                g_ble_settings.generic_settings[i].func_setting.timer_duration;
        }
        
        assign_cam_mode_var (i, l_cam_trig.trig_mode);
        assign_oper_slots (i);
        
        cam_trigger_set_trigger (&l_cam_trig);
        
    }
}

void switch_setting (uint8_t setting_flags)
{
    bool l_setting;
    if((setting_flags != OPER_MANAGE_INVALID_SLOTS) && (setting_flags != prev_setting_flags))
    {
        prev_setting_flags = setting_flags;
        g_arr_mod_state[MOTION_ONLY] = DISABLE;
        g_arr_mod_state[TIMER_ONLY] = DISABLE;
        for(uint32_t i = 0; i < MAX_SETTINGS; i++)
        {
            l_setting = setting_flags & (1 << i);
            if(l_setting)
            {
                if(g_ble_settings.generic_settings[i].trig_sel == MOTION_ONLY)
                {
                    g_motion_current_settings = i;
                    g_arr_mod_state[MOTION_ONLY] = ENABLE;
                }
                if(g_ble_settings.generic_settings[i].trig_sel == TIMER_ONLY)
                {
                    g_timer_current_settings = i;
                    g_arr_mod_state[TIMER_ONLY] = ENABLE;
                }
            }
        }
    }
    else
    {
        return;
    }
}


void mod_motion_start (void)
{
    //start
}

void mod_motion_switch_setting ()
{
    if(g_motion_prev_settings != g_motion_current_settings)
    {
        g_motion_prev_settings = g_motion_current_settings;
        
        mcp4012_set_value (g_arr_motion_sensitivity[g_motion_current_settings].amplification);
        
        //Update PIR threshold
//        pir_sense_update_thresholds (g_arr_motion_sensitivity[g_motion_current_settings].threshold);
        
    }
}

void mod_motion_add_ticks (uint32_t ticks)
{
}

void mod_motion_stop (void)
{
    //stop everything
    //reset flag
}

void mod_motion_switch_clk (sensepi_func_clk_t clk)
{
}


void mod_timer_start ()
{
    //start
}

void mod_timer_switch_setting ()
{
    if(g_timer_prev_settings != g_timer_current_settings)
    {
        g_timer_prev_settings = g_timer_current_settings;
        
        //ms timer update
    }
}

void sensepi_func_init (sensepi_func_config_t * sensepi_func_config)
{
    //initialize amplifier
    //initialize camera trigger module
    //initialize pir sense module
    //set camera settings for all ble settings
    
    //function to assign variables' values according to given settings
    
    //initialize radio trigger
    
}

void sensepi_func_update_settings (sensepi_ble_config_t * ble_settings)
{
    //update ble settings
    //set camera settings according to new ble settings
    //function to assign variables' values according to given settings
    
    //update radio trigger

}

void sensepi_func_switch_clock (sensepi_func_clk_t clk_src)
{
    //check if new clock source is same as old
    //if no update the pir clock source
    if(clk_src != g_current_clk_src)
    {
        
//        pir_sense_switch_clk (clk_src);
        g_current_clk_src  = clk_src;
    }
    //if yes return
    else
    {
        return;
    }
}

void sensepi_func_start ()
{
    //start pir sensing if pir module is active
    //start timer trigger if timer module is active
}

void sensepi_func_stop ()
{
    //stop pir sense
    //stop timer
}

sensepi_func_clk_t sensepi_func_get_current_clk ()
{
    //return current clock source
    return g_current_clk_src;
}

void sensepi_func_add_ticks (uint32_t ticks)
{
    //pass ticks to every sub-modules' individual add tick function 
    time_tracker_update_time (ticks);
    uint8_t l_new_slots = 0;
    l_new_slots = oper_manage_check_update ();
    switch_setting (l_new_slots);
    if(g_arr_mod_state[MOTION_ONLY])
    {
        mod_motion_switch_setting ();
    }
    if(g_arr_mod_state[TIMER_ONLY])
    {
        mod_timer_switch_setting ();
    }
    
    
}
