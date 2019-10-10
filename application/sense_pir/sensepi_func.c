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

#include <string.h>

#include "stdbool.h"
#include "sensepi_func.h"
#include "sensepi_ble.h"
#include "sensepi_store_config.h"

#include "pir_sense.h"
#include "mcp4012_x.h"
#include "cam_trigger.h"
#include "radio_trigger.h"
#include "time_tracker.h"
#include "slot_manage.h"
#include "ms_timer.h"
#include "hal_pin_analog_input.h"
#include "log.h"


#define FEEDBACK_TIME MS_TIMER_TICKS_MS(300 * 1000)

#define PIR_SENSE_FREQ  40

#define APP_TIME_TRACKER_LOG 1

#define DEFAULT_THRESHOLD 800

#define SWITCH_SETTING_DURATION MS_TIMER_TICKS_MS(60 * 1000)

#define TIMER_USED CONCAT_2(MS_TIMER,MS_TIMER_USED_SENSEPI)

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
volatile settings_list_t g_motion_current_settings = SETTINGS0;
volatile settings_list_t g_motion_prev_settings = MAX_SETTINGS;

/** Variable to store current active setting for timer trigger */
volatile settings_list_t g_timer_current_settings = SETTINGS1;
volatile settings_list_t g_timer_prev_settings = MAX_SETTINGS;

/** Array to store sensitivity values for PIR sensing */
motion_sensitivity_t g_arr_motion_sensitivity[MAX_SETTINGS];

/** Array to store timer values for Timer triggering */
uint32_t g_arr_timer_value[MAX_SETTINGS];

static bool g_arr_radio_trig[MAX_SETTINGS];

static uint8_t prev_setting_flags = 0;

volatile sensepi_func_modes_t g_current_mode;

static pir_sense_cfg g_pir_config;

static uint32_t g_add_ticks_switch_setting = 0;

static uint32_t g_add_ticks_motion_feedback = 0;

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

void mod_motion_switch_mode (sensepi_func_modes_t clk);

void mod_motion_handler ();

/** Timer related functions */
void mod_timer_start (void);

void mod_timer_switch_setting ();

void mod_timer_add_ticks (uint32_t ticks);

void mod_timer_stop (void);

void mod_timer_switch_mode (sensepi_func_modes_t clk);

void mod_timer_handler ();

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

void assign_cam_mode_var (uint32_t index)
{
    cam_trigger_config_t l_cam_trig;
    l_cam_trig.setup_number = index;

    l_cam_trig.trig_mode =
        g_ble_settings.generic_settings[index].cam_setting.cam_trigger.mode;


    l_cam_trig.pre_focus_en =
        g_ble_settings.generic_settings[index].cam_setting.cam_trigger.pre_focus_en;

    l_cam_trig.prf_press_duration_100ms = 
        g_ble_settings.generic_settings[index].cam_setting.cam_trigger.prf_pulse_duration_100ms;

    l_cam_trig.trig_press_duration_100ms = 
        g_ble_settings.generic_settings[index].cam_setting.cam_trigger.trig_pulse_duration_100ms;

    switch (l_cam_trig.trig_mode)
    {
        case CAM_TRIGGER_SINGLE_SHOT:            
            break;
        case CAM_TRIGGER_MULTI_SHOT:
            l_cam_trig.trig_params.multishot_params.num_shots = 
                g_ble_settings.generic_settings[index].cam_setting.cam_trigger.mode_setting.multishot.num_shots;
            l_cam_trig.trig_params.multishot_params.shot_interval_100ms = 
                g_ble_settings.generic_settings[index].cam_setting.cam_trigger.mode_setting.multishot.shot_interval_100ms;
            break;
        case CAM_TRIGGER_LONG_PRESS:
            l_cam_trig.trig_params.long_press_param = 
                g_ble_settings.generic_settings[index].cam_setting.cam_trigger.mode_setting.long_press_duration_100ms;
            break;
        case CAM_TRIGGER_VIDEO:
            l_cam_trig.trig_params.video_params.video_duration_1s = 
                g_ble_settings.generic_settings[index].cam_setting.cam_trigger.mode_setting.video.video_duration_1s;
            l_cam_trig.trig_params.video_params.extension_time = 
                g_ble_settings.generic_settings[index].cam_setting.cam_trigger.mode_setting.video.extension_time;
            l_cam_trig.trig_params.video_params.num_extensions = 
                g_ble_settings.generic_settings[index].cam_setting.cam_trigger.mode_setting.video.num_extensions;
            
            break;
        case CAM_TRIGGER_HALF_PRESS:
            break;
        case CAM_TRIGGER_NO_PRESS:
            break;
    }
    if(g_ble_settings.generic_settings[index].trig_sel == MOTION_ONLY)
    {
        l_cam_trig.trig_duration_100ms = 
            g_ble_settings.generic_settings[index].func_setting.detection_func.inter_trig_time;

    }
    else if(g_ble_settings.generic_settings[index].trig_sel == TIMER_ONLY)
    {
        l_cam_trig.trig_duration_100ms = 0;
    }
        
    cam_trigger_set_trigger (&l_cam_trig);

}

void assign_slots (uint32_t index)
{
    slot_manage_slot_t l_slot;
    l_slot.slot_no = index;
    
    if(g_ble_settings.generic_settings[index].trig_sel == MOTION_ONLY)
    {
        l_slot.slot_sel =
            g_ble_settings.trigger_oper_cond_sel[MOTION_ONLY];

    }
    else if(g_ble_settings.generic_settings[index].trig_sel == TIMER_ONLY)
    {
        l_slot.slot_sel =
            g_ble_settings.trigger_oper_cond_sel[TIMER_ONLY];
    }
    //in union start time and start light conditions shares the memory
    l_slot.start_cond =
        g_ble_settings.generic_settings[index].cam_setting.oper_cond.time_cond.start_time;

    //in union end time and end light conditions shares the memory
    l_slot.end_cond = 
        g_ble_settings.generic_settings[index].cam_setting.oper_cond.time_cond.end_time;
    slot_manage_set_slot (&l_slot);
}

void assign_varaibles ()
{
    for(uint32_t i = 0; i < MAX_SETTINGS; i++)
    {
        g_arr_radio_trig[i] = g_ble_settings.generic_settings[i].cam_setting.cam_trigger.radio_trig_en;

        if(g_ble_settings.generic_settings[i].trig_sel == MOTION_ONLY)
        {

            g_arr_motion_sensitivity[i].amplification = 
                g_ble_settings.generic_settings[i].func_setting.detection_func.amplification;

            g_arr_motion_sensitivity[i].threshold = 
                g_ble_settings.generic_settings[i].func_setting.detection_func.threshold;
        }
        else if(g_ble_settings.generic_settings[i].trig_sel == TIMER_ONLY)
        {
            g_arr_timer_value[i] =
                g_ble_settings.generic_settings[i].func_setting.timer_duration;
        }
        assign_cam_mode_var (i);
        assign_slots (i);       
    }
}

void switch_setting (uint8_t setting_flags)
{
    bool l_setting;
    if((setting_flags != SLOT_MANAGE_INVALID_SLOTS) && (setting_flags != prev_setting_flags))
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

void cam_module_handler (uint32_t cam_trigger)
{
    switch(cam_trigger)
    {
        case SETTINGS0 : 
            break;
        case SETTINGS1 : 
            break;
        case SETTINGS2 : 
            break;
        case SETTINGS3 : 
            break;
        case SETTINGS4 : 
            break;
        case SETTINGS5 : 
            break;
        case SETTINGS6 : 
            break;
        case SETTINGS7 : 
            break;
    }
}


void mod_motion_handler ()
{
    if(g_is_feedback_on == 1)
    {
        log_printf ("Feedback\n");
        //LED 
    }
    if(cam_trigger_is_on () != true)
    {
        cam_trigger (g_motion_current_settings);
    }
}

void mod_motion_start (void)
{
    g_is_feedback_on = true;
    //start
    g_pir_config.threshold =
        g_arr_motion_sensitivity[g_motion_current_settings].threshold;
    pir_sense_start (&g_pir_config);
}

void mod_motion_switch_setting ()
{
    if(g_motion_prev_settings != g_motion_current_settings)
    {
        g_motion_prev_settings = g_motion_current_settings;
        
        mcp4012_set_value (g_arr_motion_sensitivity[g_motion_current_settings].amplification);
        
        //Update PIR threshold
        pir_sense_update_threshold (g_arr_motion_sensitivity[g_motion_current_settings].threshold);
        
    }
}

void mod_motion_add_ticks (uint32_t ticks)
{
    g_add_ticks_motion_feedback += ticks;
    if(g_add_ticks_motion_feedback >= FEEDBACK_TIME)
    {
        g_is_feedback_on = false;
    }
}

void mod_motion_stop (void)
{
    //stop everything
    pir_sense_stop ();
    //reset flag
    g_is_feedback_on = false;
    g_add_ticks_motion_feedback = 0;
    g_add_ticks_switch_setting = 0;
}

void mod_motion_switch_mode (sensepi_func_modes_t mode)
{
    if(mode == SENSEPI_FUNC_NORMAL_MODE)
    {
        pir_sense_switch_clock (PIR_SENSE_LF_CLK);
    }
    if(mode == SENSEPI_FUNC_TESTING_MODE)
    {
        pir_sense_switch_clock (PIR_SENSE_HF_CLK);
    }
}


void mod_timer_handler ()
{
    if(cam_trigger_is_on () == false)
    {
        cam_trigger (g_timer_current_settings);
    }
}

void mod_timer_start ()
{
    //start
    ms_timer_start (TIMER_USED, MS_REPEATED_CALL, g_arr_timer_value[g_timer_current_settings],
                    mod_timer_handler);
}

void mod_timer_switch_setting ()
{
    if(g_timer_prev_settings != g_timer_current_settings)
    {
        g_timer_prev_settings = g_timer_current_settings;
        
        //ms timer update
        ms_timer_start (TIMER_USED, MS_REPEATED_CALL, g_arr_timer_value[g_timer_current_settings],
                        mod_timer_handler);
        
    }
}

void mod_timer_stop ()
{
    ms_timer_stop (TIMER_USED);
}

void sensepi_func_init (sensepi_func_config_t * sensepi_func_config)
{
    memcpy (&g_ble_settings, &sensepi_func_config->ble_config,
            sizeof(sensepi_ble_config_t));
    
    assign_varaibles ();
    
    time_tracker_init (APP_TIME_TRACKER_LOG);
    time_tracker_ddmmyy_t today = 
    {
        .dd = g_ble_settings.current_date.dd,
        .mm = g_ble_settings.current_date.mm,
        .yy = g_ble_settings.current_date.yy,
    };
    
    time_tracker_set_date_time (&today, g_ble_settings.current_time);
    
    
    slot_manage_set_light_sense_pin ( sensepi_func_config->light_sense_hw);
    
    //initialize amplifier
    mcp4012_init (sensepi_func_config->amp_hw.cs_pin, sensepi_func_config->amp_hw.ud_pin,
                  sensepi_func_config->amp_hw.ck_pin);
    
    //initialize camera trigger module
    cam_trigger_setup_t l_cam_trig_setup = 
    {
        .trigger_pin = sensepi_func_config->cam_hw.trigger_pin,
        .focus_pin = sensepi_func_config->cam_hw.focus_pin,
        .cam_trigger_done_handler = cam_module_handler,
    };
    cam_trigger_init (&l_cam_trig_setup);
    
    //initialize pir sense module
    {
        g_pir_config.clk_src = PIR_SENSE_LF_CLK;
        g_pir_config.pir_offset_analog_in = (sensepi_func_config->pir_hw.analog_offset_pin);
        g_pir_config.pir_signal_analog_in = (sensepi_func_config->pir_hw.analog_signal_pin);
        g_pir_config.sense_interval_ms = PIR_SENSE_FREQ;
        g_pir_config.threshold = DEFAULT_THRESHOLD;
        g_pir_config.irq_priority = APP_IRQ_PRIORITY_HIGHEST;
        g_pir_config.handler = mod_motion_handler;
    }
        
    //initialize radio trigger
    radio_trigger_init_t l_radio_trig_init = 
    {
        .comm_direction = RADIO_TRIGGER_Tx,
        .comm_freq = g_ble_settings.radio_control.radio_channel,
        .tx_on_freq_us = g_ble_settings.radio_control.radio_oper_freq_100us,
        .tx_on_time_ms = g_ble_settings.radio_control.radio_oper_duration_25ms,
        .irq_priority = APP_IRQ_PRIORITY_MID,
        .radio_trigger_tx_callback = NULL,
    };
    radio_trigger_init (&l_radio_trig_init);
}

void sensepi_func_update_settings (sensepi_ble_config_t * ble_settings)
{
    //update ble settings
    memcpy (&g_ble_settings, ble_settings, sizeof(sensepi_ble_config_t));
    
    //set camera settings according to new ble settings
    assign_varaibles ();
        
    //update radio trigger

}

void sensepi_func_switch_mode (sensepi_func_modes_t mode)
{
    g_current_mode  = mode;
    if(g_arr_mod_state[MOTION_ONLY])
    {
        mod_motion_switch_mode (mode);
    }
    else if(g_arr_mod_state[TIMER_ONLY])
    {
        mod_timer_switch_mode (mode);
    }
  
}

void sensepi_func_start ()
{
    g_is_feedback_on = true;
    g_add_ticks_motion_feedback = 0;
    g_add_ticks_switch_setting = 0;
    switch_setting (slot_manage_check_update ());
    //start pir sensing if pir module is active
    if(g_arr_mod_state[MOTION_ONLY])
    {
        mod_motion_start ();
    }
    //start timer trigger if timer module is active
    if(g_arr_mod_state[TIMER_ONLY])
    {
        mod_timer_start ();
    }
}

void sensepi_func_stop ()
{
    //stop pir sense
    mod_motion_stop ();

    //stop timer
    mod_timer_stop ();
}

sensepi_func_modes_t sensepi_func_get_current_mode ()
{
    //return current clock source
    return g_current_mode;
}

void sensepi_func_add_ticks (uint32_t ticks)
{
    //pass ticks to every sub-modules' individual add tick function 
    time_tracker_update_time (ticks);
    g_add_ticks_switch_setting += ticks;
    if(g_add_ticks_switch_setting >= SWITCH_SETTING_DURATION)
    {
        uint8_t l_new_slots = 0;
        l_new_slots = slot_manage_check_update ();
        switch_setting (l_new_slots);
        g_add_ticks_switch_setting = 0;
    }
    g_add_ticks_motion_feedback += ticks;
    if(g_add_ticks_motion_feedback >= FEEDBACK_TIME )
    {
        g_is_feedback_on = false;
    }
    if(g_arr_mod_state[MOTION_ONLY])
    {
        mod_motion_switch_setting ();
    }
    if(g_arr_mod_state[TIMER_ONLY])
    {
        mod_timer_switch_setting ();
    }
    
}
