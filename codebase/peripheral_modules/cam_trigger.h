/* 
 * File:   cam_trigger.h
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

#ifndef CAM_TRIGGER_H
#define CAM_TRIGGER_H

#include "stdint.h"

//TODO Proper naming and documentation 
typedef enum
{
    SINGLE_SHOT,
    MULTI_SHOT,
    LONG_PRESS,
    VIDEO_WITHOUT_EXTN,
    VIDEO_WITH_EXTN,
    HALF_PRESS
}cam_trigger_list_t;


typedef struct
{
    /** Pin numbers for out_gen module */
    uint32_t *out_gen_pin_array;
    /** Number of setups required to application */
    uint32_t no_of_setups;
    /** Function pointer of function which is to be called when cam trigger\
     *  operation is finished  */
    void (* cam_trigger_done_handler)(uint32_t done_state);
}cam_trigger_config_t;

typedef struct
{
    /** Total operation duration in milliseconds */
    uint32_t trig_duration_ms;
    /** State in which application should go to after camera triggering operation\
     * is done */
    uint32_t done_state;
    /** Serial number of current setup */
    uint32_t setup_number;
}cam_trigger_setup_t;

/**
 * @breif Function to initiate output pins
 * 
 * @param cam_trigger_config Pointer to cam_trigger configuration @ref cam_trigger_config_t 
 */
void cam_trigger_init(cam_trigger_config_t * cam_trigger_config);

/**
 * @breif Function to set a camera trigger
 * 
 * @param cam_trigger Camera trigger in combined format
 * 
 * @param cam_trigger_setup Pointer to trigger setup @ref cam_trigger_setup_t
 */
void cam_trigger_set_trigger (uint32_t cam_trigger, cam_trigger_setup_t * cam_trigger_setup);

/**
 * @breif Function to trigger the camera with given setup number
 * 
 * @param setup_number Setup which is to be used to trigger the camera.
 */
void cam_trigger (uint32_t setup_number);
#endif /* CAM_TRIGGER_H */
