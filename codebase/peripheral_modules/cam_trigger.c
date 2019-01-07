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
/** Duration for which PIR will be active for video extension */
#define VIDEO_PIR_ON 2000

/** Array which is partially copied while generating out_gen_config for multi-shot */
static const bool multishot_generic[OUT_GEN_MAX_NUM_OUT][OUT_GEN_MAX_TRANSITIONS] ={
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1},
    {0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1}
};




enum
{
    FOCUS_PIN,
    TRIGGER_PIN,
    NO_OF_PINS
};

void (*cam_trigger_handler)(uint32_t state);

static bool OUT_GEN_DEFAULT_STATE[] = {1,1};

static uint32_t arr_out_pin[NO_OF_PINS];

static out_gen_config_t arr_out_gen_config[MAX_SETUP_NO];

static uint32_t setup_nos;

void single_shot (cam_trigger_setup_t * cam_trigger_setup)
{
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS;
    int32_t time_remain;
    
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_ms) - SINGLE_SHOT_DURATION;
    
    if(time_remain <= 0)
    {
        time_remain = 1;
    }

    out_gen_config_t local_out_gen_config = 
    {
        .num_transitions = number_of_transition,
        .done_handler = cam_trigger_handler,
        .out_gen_state = cam_trigger_setup->done_state,
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
    int32_t time_remain;
    uint32_t number_of_transition = SINGLE_SHOT_TRANSITIONS * no_of_shots;
    //Time for trigger pulse and time till next trigger for each burst
    uint32_t repeat_delay_array[SINGLE_SHOT_TRANSITIONS] = 
    {SINGLE_SHOT_DURATION,
    MS_TIMER_TICKS_MS (time_between_shots_100ms * 100) - SINGLE_SHOT_DURATION};
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = number_of_transition,
        .done_handler = cam_trigger_handler,
        .out_gen_state = cam_trigger_setup->done_state,
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
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_ms)
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

    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_ms) - bulb_time_ticks;    
    if(time_remain <= 0)
    {
        time_remain = 1;
    }

    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = number_of_transition,
        .done_handler = cam_trigger_handler,
        .out_gen_state = cam_trigger_setup->done_state,
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
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_ms) - 
        (video_len_s);
    if(time_remain <= 0)
    {
        time_remain = 1;
        video_len_s = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_ms);
    }
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = VIDEO_WITHOUT_EXTN_TRANSITIONS,
        .next_out = {{0,1,0,1,1},
            {1,1,1,1,1}},
        .transitions_durations = {VIDEO_START_PULSE,
                    MS_TIMER_TICKS_MS(video_len_s),
                    VIDEO_END_PULSE,  time_remain},
        .done_handler = cam_trigger_handler,
        .out_gen_state = cam_trigger_setup->done_state,
    };
    
    memcpy (&arr_out_gen_config[cam_trigger_setup->setup_number], &local_out_gen_config,
            sizeof(out_gen_config_t));

}

void video_with_extn (cam_trigger_setup_t * cam_trigger_setup, uint32_t video_len,
                      uint32_t extn_len);

void half_press (cam_trigger_setup_t * cam_trigger_setup)
{
    int32_t time_remain;
    uint32_t number_of_transition = FOCUS_TRANSITIONS;
    
    time_remain = MS_TIMER_TICKS_MS(cam_trigger_setup->trig_duration_ms) - 
        SINGLE_SHOT_DURATION;
    if(time_remain <= 0)
    {
        time_remain =1;
    }
    out_gen_config_t local_out_gen_config =
    {
        .num_transitions = number_of_transition,
        .done_handler = cam_trigger_handler,
        .out_gen_state = cam_trigger_setup->done_state,
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

    memcpy(arr_out_pin, cam_trigger_config->out_gen_pin_array,
           sizeof(uint32_t) * NO_OF_PINS);

    setup_nos = cam_trigger_config->no_of_setups;
    out_gen_init (NO_OF_PINS, arr_out_pin, OUT_GEN_DEFAULT_STATE);
}

void cam_trigger_set_trigger (uint32_t cam_trigger, cam_trigger_setup_t * cam_trigger_setup)
{
    cam_trigger_list_t trig_sel = MODE_EXTRACT(cam_trigger);
    uint32_t val1 = VAL1_EXTRACT(cam_trigger);
    uint32_t val2 = VAL2_EXTRACT(cam_trigger);
    switch(trig_sel)
    {
        case SINGLE_SHOT : 
            single_shot (cam_trigger_setup);
            break;
        case MULTI_SHOT:
            multi_shot (cam_trigger_setup, val1, val2);
            break;
        case LONG_PRESS :
            long_press (cam_trigger_setup,  (uint32_t)(val1 | (val2 << 16)));
            break;
        case VIDEO_WITHOUT_EXTN :
            video_without_extn (cam_trigger_setup, val1);
            break;
        case VIDEO_WITH_EXTN :
            break;
        case HALF_PRESS :
            half_press (cam_trigger_setup);
            break;
    }
}

void cam_trigger (uint32_t setup_number)
{
    if(out_gen_is_on () == 0)
    {
        out_gen_start (&arr_out_gen_config[setup_number]);
    }
}