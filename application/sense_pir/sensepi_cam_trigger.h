/*
 *  sensepi_cam_trigger.h : Module to handle  camera triggering related functionality
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
 * @addtogroup sense_appln
 * @{
 *
 * @defgroup cam_trigg_support The support code for the PIR based Sense units.
 * @brief The PIR sense application's support file that handles camera triggering.
 *
 * @{
 *
 */

#ifndef APPLICATION_SENSEPI_CAM_TRIGGER_H
#define APPLICATION_SENSEPI_CAM_TRIGGER_H

#include "sensepi_ble.h"
#include "stdint.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Strcture to configure SensePi_PIR module
 */
typedef struct{
    sensepi_config_t * config_sensepi; ///Configuration received from mobile app
    uint32_t pir_sense_signal_input; ///Pin number for PIR signal output
    uint32_t pir_sense_offset_input; ///Pin number for PIR offset output
    uint32_t led_sense_out_pin;      ///Pin number for LED driving pin.
    uint32_t led_sense_analog_in_pin;///Pin number for light sensing.
    uint32_t led_sense_off_val;      ///Value at which LED is off.
    uint32_t amp_ud_pin;             ///Pin number for UDbar pin for MCP4012
    uint32_t amp_cs_pin;             ///Pin number for CSbar pin for MCP4012
    uint32_t amp_spi_sck_pin;            ///Pin number for local SPI clk pin for 
                                     ///MCP4012
    uint32_t *signal_out_pin_array;  ///Array of pins on which output signal is 
                                     ///to be sent.
    uint32_t signal_pin_num;         ///Number of pins in the signal_out_pin_array
}sensepi_cam_trigger_init_config_t;


/**
 * @brief Function to start PIR sensing
 */
void sensepi_cam_trigger_start();

/**
 * @brief Function to stop PIR sensing
 */
void sensepi_cam_trigger_stop();

/**
 * @brief Function to initiate SensePi_PIR module
 * @param config_sensepi_cam_trigger structure pointer of configuration for which we need to
 * configure the SensePi_PIR module
 */
void sensepi_cam_trigger_init(sensepi_cam_trigger_init_config_t * config_sensepi_cam_trigger);

/**
 * @brief Function to update the configuration at every instance when it is
 * changed in the program. 
 * @param update_config configuration to which module needs to be updated.
 */
void sensepi_cam_trigger_update(sensepi_config_t * update_config);

/**
 * @brief Function to get the current configuration to send it to mobile app.
 * @return pointer to copy of current config
 */
sensepi_config_t * sensepi_cam_trigger_get_sensepi_config();

/**
 * @brief Function to decide what to decide at current tick
 * @param interval Duration since last tick in ms
 */
void sensepi_cam_trigger_add_tick(uint32_t interval);

#ifdef __cplusplus
}
#endif

#endif /* APPLICATION_SENSEPI_CAM_TRIGGER_H */

/**
 * @}
 * @}
 */

