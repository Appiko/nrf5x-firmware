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
#define VIDEO_WE_END_INDEX (CAM_TRIGGER_MAX_SETUP_NO - 1)
/** Index for video extension out_gen_config */
#define VIDEO_WE_EXTN_INDEX (CAM_TRIGGER_MAX_SETUP_NO - 2)
/** Duration for which PIR will be active for video extension */
#define VIDEO_END_PART 2000

#define MAX_NO_OF_SHOTS OUT_GEN_MAX_TRANSITIONS/SINGLE_SHOT_TRANSITIONS

void out_gen_done_handler (uint32_t state);


enum
{
    FOCUS_PIN,
    TRIGGER_PIN,
    NO_OF_PINS
};

typedef enum
{
    SINGLE_SHOT,
    MULTI_SHOT,
    LONG_PRESS,
    VIDEO_WO_EXT,
    HALF_PRESS,
    VIDEO_W_EXT,
}trig_modes_t;

typedef enum
{
    NON_VIDEO_EXT_IDLE,
    NON_VIDEO_EXT_RUNNING,
    VIDEO_EXT_START,
    VIDEO_EXT_END,
    VIDEO_EXT_EXTEND,
    VIDEO_EXT_ITT
}state_t;

/** Array which is partially copied while generating out_gen_config for multi-shot */
static const bool multishot_generic[NO_OF_PINS][OUT_GEN_MAX_TRANSITIONS] ={
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1,
    0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1}
};

static out_gen_config_t video_ext_end_config =
{
    .done_handler = out_gen_done_handler,
    .next_out = {{1,1},
                 {1,1}},
    .num_transitions = 1,
};

static out_gen_config_t video_itt_config =
{
    .next_out = 
    {{1,0,1,1},
    {1,1,1,1}},
    .num_transitions = 3,
    .transitions_durations = {0, VIDEO_END_PULSE, 0},
    .done_handler = out_gen_done_handler,
    .out_gen_state = VIDEO_EXT_END
};

static bool OUT_GEN_DEFAULT_STATE[] = {1,1};

void (*cam_trigger_handler) (uint32_t active_config);

static uint32_t active_config;
static state_t state = NON_VIDEO_EXT_IDLE;

static out_gen_config_t arr_out_gen_config[CAM_TRIGGER_MAX_SETUP_NO];

static struct
{
    uint32_t itt_max_duration;
    uint32_t extend_duration;
}ext[CAM_TRIGGER_MAX_SETUP_NO];

void out_gen_done_handler (uint32_t out_gen_context)
{
    log_printf("%s : %d\n", __func__, state);

    static uint32_t video_ext_count;

    switch(state)
    {
        case NON_VIDEO_EXT_IDLE : 
            break;
        case NON_VIDEO_EXT_RUNNING :
            state = NON_VIDEO_EXT_IDLE;
            cam_trigger_handler(active_config);
            break;
        case VIDEO_EXT_START :
            state = VIDEO_EXT_END;
            video_ext_count = NO_OF_VIDEO_EXTN_ALLOWED;
            video_ext_end_config.transitions_durations[0] = 
                MS_TIMER_TICKS_MS(VIDEO_END_PART);
            out_gen_start (&video_ext_end_config);
            cam_trigger_handler (active_config);
            break;
        case VIDEO_EXT_END:
            state = VIDEO_EXT_ITT;
            if( ext[active_config].itt_max_duration >
                    ((NO_OF_VIDEO_EXTN_ALLOWED-video_ext_count)*
                            ext[active_config].extend_duration))
            {
                video_itt_config.transitions_durations[2] =
                        ext[active_config].itt_max_duration -
                ((NO_OF_VIDEO_EXTN_ALLOWED-video_ext_count)*
                        ext[active_config].extend_duration);
            }
            else
            {
                video_itt_config.transitions_durations[2] = 0;
            }
            video_ext_end_config.out_gen_state = VIDEO_EXT_ITT;
            out_gen_start (&video_itt_config);
            break;
        case VIDEO_EXT_EXTEND:
            video_ext_count--;
            if(video_ext_count)
            {
                state = VIDEO_EXT_END;
                cam_trigger_handler(active_config);
                video_ext_end_config.transitions_durations[0] = 
                    MS_TIMER_TICKS_MS(VIDEO_END_PART);
                out_gen_start (&video_ext_end_config);
            }
            else
            {
                state = VIDEO_EXT_ITT;
                video_itt_config.transitions_durations[0] = 
                    MS_TIMER_TICKS_MS(VIDEO_END_PART);
                out_gen_start (&video_itt_config);
            }

            break;
        case VIDEO_EXT_ITT:
           cam_trigger_handler (active_config);
           state = NON_VIDEO_EXT_IDLE;
           break;
    }
}

void single_shot (cam_trigger_config_t * cam_trigger_config)
{
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS;
    int32_t time_remain;
    
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_config->trig_duration_100ms * 100) - SINGLE_SHOT_DURATION;
    
    if(time_remain <= 0)
    {
        time_remain = 1;
    }

    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = NON_VIDEO_EXT_RUNNING,
        .transitions_durations = { SINGLE_SHOT_DURATION, time_remain },
        .next_out = {{0, 1, 1},
                     {0, 1, 1}},
    };
    memcpy (&arr_out_gen_config[cam_trigger_config->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
}

void multi_shot (cam_trigger_config_t * cam_trigger_config, uint32_t time_between_shots_100ms,
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
        .out_gen_state = NON_VIDEO_EXT_RUNNING,
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
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_config->trig_duration_100ms * 100)
        - SINGLE_SHOT_DURATION*no_of_shots - 
        ((MS_TIMER_TICKS_MS (time_between_shots_100ms * 100) - SINGLE_SHOT_DURATION)
        *(no_of_shots - 1));    
    if(time_remain <= 0)
    {
        time_remain = 1;
    }
    local_out_gen_config.transitions_durations[number_of_transition - 1] = time_remain;

    memcpy (&arr_out_gen_config[cam_trigger_config->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
}



void long_press (cam_trigger_config_t * cam_trigger_config, uint32_t expousure_time_100ms)
{
    int32_t time_remain;
    uint32_t number_of_transition = BULB_SHOT_TRANSITIONS;
    uint32_t bulb_time_ticks = MS_TIMER_TICKS_MS ((expousure_time_100ms * 100));

    time_remain = MS_TIMER_TICKS_MS(cam_trigger_config->trig_duration_100ms * 100) - bulb_time_ticks;    
    if(time_remain <= 0)
    {
        time_remain = 1;
    }

    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = NON_VIDEO_EXT_RUNNING,
        .transitions_durations =
            {bulb_time_ticks, time_remain},
        .next_out =
            {
                {0, 1, 1},
                {0, 1, 1}
            },
    };
    
    memcpy (&arr_out_gen_config[cam_trigger_config->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
}

void video_without_extn (cam_trigger_config_t * cam_trigger_config, uint32_t video_len_s)
{
    int32_t time_remain;
    video_len_s = MS_TIMER_TICKS_MS(video_len_s * 1000);

    time_remain = MS_TIMER_TICKS_MS(cam_trigger_config->trig_duration_100ms * 100) - 
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
        .out_gen_state = NON_VIDEO_EXT_RUNNING,
    };
    
    memcpy (&arr_out_gen_config[cam_trigger_config->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));

}

void video_with_extn (cam_trigger_config_t * cam_trigger_config, uint32_t video_len_s,
                      uint32_t extn_len_s)
{
    ext[cam_trigger_config->setup_number].extend_duration =
            MS_TIMER_TICKS_MS(extn_len_s * 1000);

    video_len_s = video_len_s * 1000;
    if((cam_trigger_config->trig_duration_100ms * 100) > video_len_s)
    {
        ext[cam_trigger_config->setup_number].itt_max_duration =
                cam_trigger_config->trig_duration_100ms * 100 - video_len_s;
        ext[cam_trigger_config->setup_number].itt_max_duration =
                MS_TIMER_TICKS_MS(ext[cam_trigger_config->setup_number].itt_max_duration);
    }
    else
    {
        ext[cam_trigger_config->setup_number].itt_max_duration = 0;
    }
    
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
        .out_gen_state = VIDEO_EXT_EXTEND,
    };

    memcpy (&arr_out_gen_config[cam_trigger_config->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
}

void half_press (cam_trigger_config_t * cam_trigger_config)
{
    int32_t time_remain;
    uint32_t number_of_transition = FOCUS_TRANSITIONS;
    
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_config->trig_duration_100ms * 100) - 
        SINGLE_SHOT_DURATION;
    if(time_remain <= 0)
    {
        time_remain =1;
    }
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = number_of_transition,
        .done_handler = out_gen_done_handler,
        .out_gen_state = NON_VIDEO_EXT_RUNNING,
        .transitions_durations =
            {SINGLE_SHOT_DURATION, time_remain},
        .next_out =
            {
                {0, 1, 1},
                {1, 1, 1}
            },
    };
    
    memcpy (&arr_out_gen_config[cam_trigger_config->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));
}

void cam_trigger_init(cam_trigger_setup_t * cam_trigger_setup)
{
    cam_trigger_handler = cam_trigger_setup->cam_trigger_done_handler;
    
    uint32_t arr_out_pin[NO_OF_PINS];
    arr_out_pin[FOCUS_PIN] = cam_trigger_setup->focus_pin;
    arr_out_pin[TRIGGER_PIN] = cam_trigger_setup->trigger_pin;
    
    //Check if setup_no is a valid number
    out_gen_init (NO_OF_PINS, arr_out_pin, OUT_GEN_DEFAULT_STATE);

    state = NON_VIDEO_EXT_IDLE;
}

void cam_trigger_set_trigger (cam_trigger_config_t * cam_trigger_config)
{
    trig_modes_t mode;
    if((cam_trigger_config->trig_mode == CAM_TRIGGER_VIDEO) &&
       (cam_trigger_config->trig_param2 != 0))
    {
        mode = VIDEO_W_EXT;
    }
    else
    {
        mode = cam_trigger_config->trig_mode;
    }
    
    switch(mode)
    {
        case SINGLE_SHOT : 
            single_shot (cam_trigger_config);
            break;
        case MULTI_SHOT:
            multi_shot (cam_trigger_config, cam_trigger_config->trig_param1,
                        cam_trigger_config->trig_param2);
            break;
        case LONG_PRESS :
            long_press (cam_trigger_config,  (uint32_t)(cam_trigger_config->trig_param1 
                | (cam_trigger_config->trig_param2 << 16)));
            break;
        case VIDEO_WO_EXT:
            video_without_extn (cam_trigger_config, cam_trigger_config->trig_param1);
            break;
        case VIDEO_W_EXT:
            video_with_extn (cam_trigger_config, cam_trigger_config->trig_param1,
                                    cam_trigger_config->trig_param2);
            break;
        case HALF_PRESS :
            half_press (cam_trigger_config);
            break;
    }
}

void cam_trigger (uint32_t setup_number)
{
    if(state == NON_VIDEO_EXT_IDLE)
    {
       active_config = setup_number;

       //A non extend trigger should happen now
       if(arr_out_gen_config[setup_number].out_gen_state != VIDEO_EXT_EXTEND)
       {
            state = NON_VIDEO_EXT_RUNNING;
            out_gen_start (&arr_out_gen_config[setup_number]);
       }
       //A extendable video trigger starts now
       else
       {
           state = VIDEO_EXT_START;
           out_gen_start (&arr_out_gen_config[setup_number]);
       }
    }
    else if(state == VIDEO_EXT_END)
    {
        uint32_t ticks_done = out_gen_get_ticks ();
        video_ext_end_config.transitions_durations[0]  =
            (ext[active_config].extend_duration > ticks_done)
            ? (ext[active_config].extend_duration - ticks_done)
            : ext[active_config].extend_duration;
        video_ext_end_config.out_gen_state = VIDEO_EXT_END;
        out_gen_start (&video_ext_end_config);
        state = VIDEO_EXT_EXTEND;
    }

}

