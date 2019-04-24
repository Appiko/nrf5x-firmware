/**
 *  cam_trigger.h : Camera Triggering Module
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

/**
 * 
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_cam_trig Camera Trigger
 * @{
 *
 * @brief Driver triggers the camera according to configuration sent by application.
 * This Driver uses out_pattern_gen module.
 *
 * The image below gives the flow of how camera triggering takes place.
 * @dot 
 * digraph State_machine_diagram {
 * ratio = "fill";
 * size = "12,8";
 * idle [shape = circle, width = 1.2, label = "Idle"];
 * run [shape = circle, width = 1.5, label = "Non Video Extension :\n All Operations"];
 * vid_st [shape = circle, width = 1.5, label = "Video Extension :\n Start\n |_|`````\n Counter = MAX"];
 * vid_end [shape = circle, width = 1.5, label = "Video Extension :\n End\n ``````"];
 * vid_ext [shape = circle, width = 1.5, label = "Video Extension :\n Extension\n ```````\n Counter--"];
 * vid_itt [shape = circle, width = 1.5, label = "Video Extension :\n Inter Trigger Time\n ``|_|```"];
 * idle -> run [label = "All triggers except video with Extensions"];
 * run -> idle [label = "Out Gen Done : All operations"];
 * idle -> vid_st [label = "Trigger : Video with Extensions"];
 * vid_st -> vid_end [label = "Out Gen Done : Start"];
 * vid_end -> vid_ext [constraint = false, label = "Triggers"];
 * vid_ext -> vid_end [label = "Out Gen Done : Extension\n & Counter > 0"];
 * vid_end -> vid_itt [label = "Out Gen Done : End"];
 * vid_ext -> vid_itt [label = "Out Gen Done : Extension\n & Counter = 0"];
 * vid_itt -> idle [label = "Out Gen Done : Inter Trigger Time"]
 * } 
 * @enddot
 */
#ifndef CAM_TRIGGER_H
#define CAM_TRIGGER_H

#include "stdint.h"
#include "stdbool.h"

#define CAM_TRIGGER_MAX_SETUP_NO 8


//TODO Proper naming and documentation 
/**
 * @breif List of Triggers available to application.
 */
typedef enum
{
    /** Global Single Shot Trigger */
    CAM_TRIGGER_SINGLE_SHOT,
    /** Global Multi Shot Trigger */
    CAM_TRIGGER_MULTI_SHOT,
    /** Global Long Press Trigger */
    CAM_TRIGGER_LONG_PRESS,
    /** Global Video Trigger */
    CAM_TRIGGER_VIDEO,
    /** Global Half Press Trigger */
    CAM_TRIGGER_HALF_PRESS
}cam_trigger_list_t;

/**
 * @brief Structure to store information related to camera trigger.
 * @{
 */
typedef struct
{

    /** Mode in which camera will get triggered */
    cam_trigger_list_t trig_mode;

    /** First parameter for camera trigger */
    uint16_t trig_param1;

    /** Second parameter for camera trigger */
    uint8_t trig_param2;

    /** Total operation duration in milliseconds */
    uint32_t trig_duration_100ms;

    /** Serial number of current setup */
    uint32_t setup_number;
    
    /** Pre-Focus signal setup */
    bool pre_focus_en;

}cam_trigger_config_t;
/**
 * @}
 */

/**
 * @brief Structure to store information corresponding to hardware and application.
 * @{
 */
typedef struct
{

    /** Focus Pin */
    uint32_t focus_pin;

    /** Trigger Pin */
    uint32_t trigger_pin;

    /** Function pointer of function which is to be called when cam trigger\
     *  operation is finished  */
    void (* cam_trigger_done_handler)(uint32_t done_state);

}cam_trigger_setup_t;
/**
 * @}
 */

/**
 * @brief Function to initiate output pins
 * 
 * @param cam_trigger_setup Pointer to cam_trigger module setup @ref cam_trigger_setup_t 
 */
void cam_trigger_init(cam_trigger_setup_t * cam_trigger_setup);

/**
 * @brief Function to set a camera trigger
 * 
 * @param cam_trigger Structure pointer to the structure storing camera trigger related information.
 * 
 */
void cam_trigger_set_trigger (cam_trigger_config_t * cam_trigger);

/**
 * @brief Function to trigger the camera with given setup number
 * 
 * @param setup_number Setup which is to be used to trigger the camera.
 */
void cam_trigger (uint32_t setup_number);

/**
 * @brief Function to stop camera trigger
 */
void cam_trigger_stop (void);

/**
 * @brief Function to check status of cam_trigger module.
 * @return Status of cam_trigger module
 * @retval true : cam_trigger module is already running
 * @retval false : cam_trigger module is not running
 */
bool cam_trigger_is_on (void);
#endif /* CAM_TRIGGER_H */
/**
 * @}
 * @}
 */
