/*
 *  sensepi_func.h : File to handle sensing functionality of sensepi
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

#ifndef SENSEPI_FUNC_H
#define SENSEPI_FUNC_H

#include "sensepi_ble.h"


#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif 

#ifndef MS_TIMER_USED_SENSEPI
#define MS_TIMER_USED_SENSEPI 2
#endif
/** Structure to store pin numbers required for mcp4012 driver */
typedef struct 
{
    /** Pin Number for control Pin*/
    uint32_t cs_pin;
    
    /** Pin Number for Up/Down Pin*/
    uint32_t ud_pin;
    
    /** Pin Number for Clock Pin*/
    uint32_t ck_pin;

}amp_t;

/** Structure to store pin numbers required for cam trigger driver */
typedef struct
{
    /** Pin number for focus pin in audio jack */
    uint32_t focus_pin;

    /** Pin number for trigger pin in audio jack */
    uint32_t trigger_pin;

    /** Pin number for jack detect pin in audio jack */
    uint32_t jack_detect_pin;

}cam_t;

/** Structure to store pin numbers required for PIR sense driver */
typedef struct 
{
    /** Analog pin number for offset pin */
    uint32_t analog_offset_pin;

    /** Analog pin number for signal pin */
    uint32_t analog_signal_pin;

}pir_t;

/** Note : Low freq clock is used when sensor is supposed to operate in low power mode
 * while in low power mode, module uses RTC0. But when SoftDevice(BLE) is active,
 * SoftDevices use RTC0. So to keep sensing along with SoftDevices, we use High freq
 * clock(Timer 1 or 2). And anyway when SoftDevices are being used core will be
 * on so using HFCLK will not add much in power consumption  */

/** List of possible modes */
typedef enum
{
    /** In normal mode, low freq clock will be used for sensing and ble structure will be used */
    SENSEPI_FUNC_NORMAL_MODE,

    /** In testing mode, high freq clock will be used for sensing and some default config will be used */
    SENSEPI_FUNC_TESTING_MODE,

}sensepi_func_modes_t;

/** Structure containing information required to initialize SensePi func module */
typedef struct 
{
    /** Structure to store info required for mcp4012 driver */
    amp_t amp_hw;

    /** Structure to store info required for cam trigger driver */
    cam_t cam_hw;

    /** Structure to store info required for pir sense driver */
    pir_t pir_hw;
    
    /** Light sense gpio pin */
    uint32_t light_sense_hw;

    /** Structure to store default BLE configuration */
    sensepi_ble_config_t ble_config;
    
    /** Default mode */
    sensepi_func_modes_t func_mode;

}sensepi_func_config_t;

/**
 * @brief Function to initialize SensePi sensing Functionality module
 * @param sensepi_func_config Structure pointer to default structure of type
 *  @ref sensepi_func_config_t required for initialize
 */
void sensepi_func_init (sensepi_func_config_t * sensepi_func_config);

/**
 * @breif Function to update BLE settings to recent configuration received over BLE
 * @param ble_settings structure pointer to new configuration of type @ref sensepi_ble_config_t
 */
void sensepi_func_update_settings (sensepi_ble_config_t * ble_settings);

/**
 * @brief Function to switch between modes of SensePi
 * @param mode Mode which is to be set @ref sensepi_func_modes_t
 */
void sensepi_func_switch_mode (sensepi_func_modes_t mode);

/**
 * @brief Function to start the sensing functionality
 */
void sensepi_func_start ();

/**
 * @brief Function to stop the sensing functionality
 */
void sensepi_func_stop ();

/**
 * @brief Function to get current functional mode
 * @return current functional mode @ref sensepi_func_modes_t
 */
sensepi_func_modes_t sensepi_func_get_current_mdoe ();

/**
 * @bierf Function to send ticks since last add_ticks event
 * @param ticks Ticks since last add_ticks event.
 */
void sensepi_func_add_ticks (uint32_t ticks);

#endif /* SENSEPI_FUNC_H */
