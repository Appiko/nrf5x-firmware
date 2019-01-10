/* 
 * File:   cam_trigger.c
 * Copyright (c) 2018 Appiko
 * Created on 4 January, 2019, 5:04 PM
 * Author:  Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE
 */



#include <string.h>

#include "cam_trigger.h"
#include "out_pattern_gen.h"
#include "ms_timer.h"
#include "hal_gpio.h"
#include "stdbool.h"
#include "log.h"

#define MAX_SETUP_NO 8

#define MODE_EXTRACT(a)  (a & 0xFF)

#define VAL1_POS 8
#define VAL1_EXTRACT(a)  (a & 0x00FFFF00) >> VAL1_POS

#define VAL2_POS 24
#define VAL2_EXTRACT(a)  (a & 0xFF000000) >> VAL2_POS

//FROM SENSE_PIR FIRMWARE

/** Number of transitions required for single shot operation */
#define SINGLE_SHOT_TRANSITIONS 2
/** Ticks duration required for single shot operation */
#define SINGLE_SHOT_DURATION MS_TIMER_TICKS_MS(250)
/** Number of transitions required for bulb operation */
#define BULB_SHOT_TRANSITIONS 2
/** Number of transitions required for focus operation */
#define FOCUS_TRANSITIONS 2
/** Number of transitions required for Video operation triggered by Timer */
#define VIDEO_WITHOUT_EXTN_TRANSITIONS 4
/** Ticks duration of trigger pulse to start a video */
#define VIDEO_START_PULSE MS_TIMER_TICKS_MS(250)
/** Ticks duration of trigger pulse to end a video */
#define VIDEO_END_PULSE MS_TIMER_TICKS_MS(300)
/** Number of maximum extensions for Video */
#define NO_OF_VIDEO_EXTN_ALLOWED 3
/** Index for end video out_gen_config */
#define VIDEO_WE_END_INDEX (MAX_SETUP_NO - 1)
/** Index for video extension out_gen_config */
#define VIDEO_WE_EXTN_INDEX (MAX_SETUP_NO - 2)
/** Duration for which PIR will be active for video extension */
#define VIDEO_END_PART 2000

#define MAX_NO_OF_SHOTS OUT_GEN_MAX_TRANSITIONS/SINGLE_SHOT_TRANSITIONS

void out_gen_done_handler (uint32_t state);
enum
{
    DISABLE,
    ENABLE,
};


enum
{
    FOCUS_PIN,
    TRIGGER_PIN,
    NO_OF_PINS
};

enum
{
    END_TRIG,
    VIDEO_EXTENSIONS,
    
};

/** Array which is partially copied while generating out_gen_config for multi-shot */
static const bool multishot_generic[NO_OF_PINS][OUT_GEN_MAX_TRANSITIONS] ={
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1}
};


/** Counter to keep track of extensions */
static uint32_t video_extn_cnt = NO_OF_VIDEO_EXTN_ALLOWED;
/** Video flag */
static bool video_on_flag;
/** Video operation duration */
static int32_t video_oper_time;
/** Video Extension time */
static uint32_t video_extn_len = 1;
 /* extension length will be set in trigger_set function. Once set, Extension length will remain same.*/
static out_gen_config_t video_ext_config = 
{
    .done_handler = out_gen_done_handler,
    .next_out = 
    {{1,1},
    {1,1}},
    .num_transitions = 1,
    .out_gen_state = VIDEO_EXTENSIONS
};
/* Stop length will be calculated every time. Stop config will be automatically started once start 
part or extensions are over*/
static out_gen_config_t video_end_config = 
{
    .next_out = 
    {{1,0,1,1},
    {1,1,1,1}},
    .num_transitions = 3,
    .transitions_durations = {MS_TIMER_TICKS_MS(VIDEO_END_PART), VIDEO_END_PULSE,
    1},
    .done_handler = out_gen_done_handler,
    .out_gen_state = END_TRIG
};


static uint32_t callback_state;
void (*cam_trigger_handler) (uint32_t callback_state);


void out_gen_done_handler (uint32_t state)
{
    log_printf("%s : %d\n", __func__, state);
    switch(state)
    {
        case END_TRIG :
            video_on_flag = DISABLE;
            video_extn_cnt = NO_OF_VIDEO_EXTN_ALLOWED;
            break;
        case VIDEO_EXTENSIONS :
            video_on_flag = ENABLE;
            out_gen_start (&video_end_config);
            break;
    }
    if(video_extn_cnt != 0)
    {
        cam_trigger_handler (callback_state);
    }
}


static bool OUT_GEN_DEFAULT_STATE[] = {1,1};

static uint32_t arr_out_pin[NO_OF_PINS];

static out_gen_config_t arr_out_gen_config[MAX_SETUP_NO];

static uint32_t setup_nos;

void single_shot (cam_trigger_setup_t * cam_trigger_setup)
{
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS;
    int32_t time_remain;
    
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_100ms * 100) - SINGLE_SHOT_DURATION;
    
    if(time_remain <= 0)
    {
        time_remain = 1;
    }

    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = END_TRIG,
        .transitions_durations = { SINGLE_SHOT_DURATION, time_remain },
        .next_out = {{0, 1, 1},
                     {0, 1, 1}},
    };
    memcpy (&arr_out_gen_config[cam_trigger_setup->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
}

void multi_shot (cam_trigger_setup_t * cam_trigger_setup, uint32_t time_between_shots_100ms,
                 uint32_t no_of_shots)
{

    if(no_of_shots > MAX_NO_OF_SHOTS)
    {
        no_of_shots = MAX_NO_OF_SHOTS;
    }

    int32_t time_remain;
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS * no_of_shots;
    //Time for trigger pulse and time till next trigger for each burst
    uint32_t repeat_delay_array[SINGLE_SHOT_TRANSITIONS] = 
    {SINGLE_SHOT_DURATION,
    MS_TIMER_TICKS_MS (time_between_shots_100ms * 100) - SINGLE_SHOT_DURATION};
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = END_TRIG,
    };

    for (uint32_t i = 0; i < no_of_shots; i++)
    {
        memcpy (local_out_gen_config.transitions_durations + i*SINGLE_SHOT_TRANSITIONS,
                repeat_delay_array, SINGLE_SHOT_TRANSITIONS * sizeof (uint32_t));

        for (uint32_t j = 0; j < NO_OF_PINS; j++)
        {
            memcpy (*(local_out_gen_config.next_out + j), *(multishot_generic + j),
                    no_of_shots * SINGLE_SHOT_TRANSITIONS * sizeof (bool));
            //An extra '1' at the end for the remaining of the inter-trigger time
            local_out_gen_config.next_out[j][no_of_shots * SINGLE_SHOT_TRANSITIONS] = 1;
        }
    }
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_100ms * 100)
        - SINGLE_SHOT_DURATION*no_of_shots - 
        ((MS_TIMER_TICKS_MS (time_between_shots_100ms * 100) - SINGLE_SHOT_DURATION)
        *(no_of_shots - 1));    
    if(time_remain <= 0)
    {
        time_remain = 1;
    }
    local_out_gen_config.transitions_durations[number_of_transition - 1] = time_remain;

    memcpy (&arr_out_gen_config[cam_trigger_setup->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
}



void long_press (cam_trigger_setup_t * cam_trigger_setup, uint32_t expousure_time_100ms)
{
    int32_t time_remain;
    uint32_t number_of_transition = BULB_SHOT_TRANSITIONS;
    uint32_t bulb_time_ticks = MS_TIMER_TICKS_MS ((expousure_time_100ms * 100));

    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_100ms * 100) - bulb_time_ticks;    
    if(time_remain <= 0)
    {
        time_remain = 1;
    }

    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = END_TRIG,
        .transitions_durations =
            {bulb_time_ticks, time_remain},
        .next_out =
            {
                {0, 1, 1},
                {0, 1, 1}
            },
    };
    
    memcpy (&arr_out_gen_config[cam_trigger_setup->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
}

void video_without_extn (cam_trigger_setup_t * cam_trigger_setup, uint32_t video_len_s)
{
    int32_t time_remain;
    video_len_s = MS_TIMER_TICKS_MS(video_len_s * 1000);

    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_100ms * 100) - 
        (video_len_s);

    if(time_remain <= 0)
    {
        time_remain = 1;
    }
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = VIDEO_WITHOUT_EXTN_TRANSITIONS,
        .next_out = 
        {{0,1,0,1,1},
        {1,1,1,1,1}},
        .transitions_durations = {VIDEO_START_PULSE,
                    (video_len_s),
                    VIDEO_END_PULSE,  time_remain},
        .done_handler = out_gen_done_handler,
        .out_gen_state = END_TRIG,
    };
    
    memcpy (&arr_out_gen_config[cam_trigger_setup->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));

}

void video_with_extn (cam_trigger_setup_t * cam_trigger_setup, uint32_t video_len_s,
                      uint32_t extn_len_s)
{
    video_extn_len = MS_TIMER_TICKS_MS(extn_len_s * 1000);
    video_ext_config.transitions_durations[0] = (video_extn_len);

    video_len_s = video_len_s * 1000;
    video_oper_time = cam_trigger_setup->trig_duration_100ms * 100 - video_len_s;
    video_oper_time = MS_TIMER_TICKS_MS(video_oper_time);
    if(video_oper_time < 0)
    {
        video_oper_time = 1;
    }
    video_end_config.transitions_durations[2] = (video_oper_time);
    

    int32_t video_len_check;
    video_len_check = video_len_s - VIDEO_END_PART;
    video_len_s = (video_len_check < 0) ? 1 : video_len_check;
    
    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = 2,
        .next_out = 
        {{0,1,1},
        {1,1,1}},
        .transitions_durations = {VIDEO_START_PULSE, MS_TIMER_TICKS_MS(video_len_s)},
        .done_handler = out_gen_done_handler,
        .out_gen_state = VIDEO_EXTENSIONS,
    };

    memcpy (&arr_out_gen_config[cam_trigger_setup->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
    
    
}

void half_press (cam_trigger_setup_t * cam_trigger_setup)
{
    int32_t time_remain;
    uint32_t number_of_transition = FOCUS_TRANSITIONS;
    
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_100ms * 100) - 
        SINGLE_SHOT_DURATION;
    if(time_remain <= 0)
    {
        time_remain =1;
    }
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = END_TRIG,
        .transitions_durations =
            {SINGLE_SHOT_DURATION, time_remain},
        .next_out =
            {
                {0, 1, 1},
                {1, 1, 1}
            },
    };
    
    memcpy (&arr_out_gen_config[cam_trigger_setup->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
}

void cam_trigger_init(cam_trigger_config_t * cam_trigger_config)
{
    cam_trigger_handler = cam_trigger_config->cam_trigger_done_handler;
    
    arr_out_pin[FOCUS_PIN] = cam_trigger_config->focus_pin;
    arr_out_pin[TRIGGER_PIN] = cam_trigger_config->trigger_pin;
    
    setup_nos = cam_trigger_config->no_of_setups;
    out_gen_init (NO_OF_PINS, arr_out_pin, OUT_GEN_DEFAULT_STATE);
}

void cam_trigger_set_trigger (cam_trigger_t * cam_trigger, cam_trigger_setup_t * cam_trigger_setup)
{
    callback_state = cam_trigger_setup->setup_number;
    
    switch(cam_trigger->trig_mode)
    {
        case SINGLE_SHOT : 
            single_shot (cam_trigger_setup);
            break;
        case MULTI_SHOT:
            multi_shot (cam_trigger_setup, cam_trigger->trig_param1,
                        cam_trigger->trig_param2);
            break;
        case LONG_PRESS :
            long_press (cam_trigger_setup,  (uint32_t)(cam_trigger->trig_param1 
                | (cam_trigger->trig_param2 << 16)));
            break;
        case VIDEO:
            if(cam_trigger->trig_param2 == 0)
            {
                video_without_extn (cam_trigger_setup, cam_trigger->trig_param1);
            }
            else
            {
                video_on_flag = DISABLE;
                video_with_extn (cam_trigger_setup, cam_trigger->trig_param1,
                                    cam_trigger->trig_param2);
            }
            break;
        case HALF_PRESS :
            half_press (cam_trigger_setup);
            break;
    }
}

void cam_trigger (uint32_t setup_number)
{
    static int32_t video_itt;
    if(out_gen_is_on () == DISABLE && video_on_flag == DISABLE)
    {
        out_gen_start (&arr_out_gen_config[setup_number]);
        video_itt = video_oper_time;
    }
    else if(video_on_flag == ENABLE)
    {
        video_itt = video_itt - video_extn_len;
        if(video_itt <= 0)
        {
            video_itt = 1;
        }
        video_end_config.transitions_durations[2] = (video_itt);
        
        uint32_t ticks_done = out_gen_get_ticks ();
        video_ext_config.transitions_durations[0]  = (video_extn_len > ticks_done) 
            ? (video_extn_len - ticks_done) : video_extn_len;
        
        out_gen_start (&video_ext_config);
        
        video_extn_cnt --;
    }
}

