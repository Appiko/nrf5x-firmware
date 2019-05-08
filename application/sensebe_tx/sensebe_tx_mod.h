/*
 *  sensebe_tx_rx_mod.h : Module to handle SenseBe's Tx Rx functionalities
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
 * @addtogroup sensebe_appln
 * @{
 *
 * @defgroup tx_rx_ctrl The support code to control IR transmission and reception.
 * @brief The active IR sense application's support file that handles motion detection\ 
 * and timer based operations.
 *
 * @{
 *
 */


#ifndef SENSEBE_TX_RX_MOD_H
#define SENSEBE_TX_RX_MOD_H

#include "ms_timer.h"
#include "sensebe_ble.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef MS_TIMER_USED_SENSBE_TX_RX 
#define MS_TIMER_USED_SENSBE_TX_RX 2
#endif


#ifdef __cplusplus
extern "C" {
#endif
        
#define SENSEBE_OPERATION_MS_TIMER CONCAT_2(MS_TIMER,MS_TIMER_USED_SENSBE_TX_RX)

typedef struct
{
    /** Pin number for Photodiode */
    uint32_t photodiode_pin;
    /** Enable pin for Photodiode */
    uint32_t photodiode_en_pin;
    
}light_sense_pin_config_t;

typedef struct
{
    /** Pin number for Focus pin */
    uint32_t focus_pin_no;
    /** Pin number for Trigger pin */
    uint32_t trigger_pin_no;
}cam_trig_pin_config_t;

typedef struct
{
    uint32_t tx_en_pin;
    uint32_t tx_in_pin;
    uint32_t tx_pwr1;
    uint32_t tx_pwr2;
}tx_pin_config_t;

typedef struct
{
    cam_trig_pin_config_t cam_config;
    light_sense_pin_config_t light_sense_config;
    tx_pin_config_t tx_transmit_config;
    sensebe_config_t * sensebe_config;
}sensebe_tx_config_t;

/**
 * @breif Function to change the range of IR transmission
 */
void sensebe_tx_rx_swicht_range ();

/**
 * @brief Function to initialize the Rx detect module
 * @param sensebe_tx_init Configuration required for sensbe_rx_detect module
 * @ref sensebe_tx_init_t
 */    
void sensebe_tx_rx_init (sensebe_tx_config_t * sensebe_tx_init);

/**
 * @brief Function to enable detection for SenseBe Rx
 */
void sensebe_tx_rx_start (void);

/**
 * @brief Function to disable detection for SenseBe Rx
 */
void sensebe_tx_rx_stop (void);

/**
 * @brief Function to handle add tick event.
 * 
 * @param interval Ticks since the last occurance of add tick event.
 */
void sensebe_tx_rx_add_ticks (uint32_t interval);

/**
 * @brief Function to update SenseBe Rx configuration to config received over BLE
 * 
 * @param sensebe_config Pointer to the Configuration received over BLE
 */
void sensebe_tx_rx_update_config (sensebe_config_t * update_sensebe_config);

/**
 * @brief Function to get last config which is being used.
 * @return Configuration pointer to the configuration which is being used.
 */
sensebe_config_t * sensebe_tx_rx_last_config ();
#ifdef __cplusplus
}
#endif

#endif /* SENSEBE_TX_RX_MOD_H */
/**
 * @}
 * @}
 */

