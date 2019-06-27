/**
 *  opt3101_oper.h : OPT3101 basic operations driver.
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef OPT3101_OPER_H
#define OPT3101_OPER_H

#include "register_map.h"
#include "stdint.h"
#include "nrf_util.h"

/**
 * @addtogroup group_ti_opt3101
 * @{
 *
 * @defgroup group_opt3101_oper OPT3101 basic operation driver
 * @brief Driver to handle basic operations of OPT3101
 *
 * @{
 */


/** List of operation modes for OPT3101 */
typedef enum
{
    /** Monoshot mode */
    OPT3101_OPER_MONOSHOT,
    /** Continuous mode */
    OPT3101_OPER_CONTINUOUS,
}opt3101_oper_mode_t;

//Function to initialize
/**
 * @brief Function to initialize the operation of OPT3101 with default setup
 * @return Status 
 * @retval 0 Successful
 */
uint8_t opt3101_oper_init ();

//function to start
/**
 * @brief Function to start the distance measurement
 * @return Status
 * @retval 0 Successful
 * 
 * @note This function can be used only with monoshot mode
 * 
 */
uint8_t opt3101_oper_start ();

/**
 * @brief Function to get distance from last frame
 * @return Distance
 */
uint32_t opt3101_oper_get_dist ();

//functions to modify
/**
 * @brief Function to select mode of operation
 * @param mode Mode of operation @ref opt3101_oper_mode_t
 * @return Status
 * @retval 0 Successful
 */
uint8_t opt3101_oper_sel_mode (opt3101_oper_mode_t mode);

/**
 * @brief Function to set Number of frames that are to be used for averaging 
 * @param frame_nos Number of frames used to calculate average
 * @return Status
 * @retval 0 Successful
 */
uint8_t opt3101_oper_set_frame_nos (uint32_t frame_nos);

/**
 * @brief Function to set number of sub-frames in each frame
 * @param subframe_nos Number of sub-frames in each frame
 * @return Status
 * @retval 0 Successful
 */
uint8_t opt3101_oper_set_subframe_nos (uint32_t subframe_nos);

#endif //OPT3101_OPER_H
