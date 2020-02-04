/*
 *  lrf_node_rf.h : <Write brief>
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


#ifndef LRF_NODE_RF_H
#define LRF_NODE_RF_H

#include "lrf_node_ble.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#include "rf_comm.h"
#include "rf_spi_hw.h"


#ifndef MS_TIMER_USED_LRF_NODE_MOD
#define MS_TIMER_USED_LRF_NODE_MOD 0
#endif

typedef struct 
{
    /** Center freq : kHz */
    uint32_t center_freq;
    /** Bitrate : bps */
    uint32_t bitrate;
    /** Tx Power : dBm */
    int32_t tx_power;
    /** f-dev : kHz */
    uint32_t fdev;
}lrf_node_mod_rf_params_t;

typedef lrf_node_prodc_info_t lrf_node_mod_rf_head_t;

typedef rf_comm_hw_t lrf_node_rf_hw_t;

typedef rf_spi_init_t lrf_node_rf_spi_t;

typedef struct
{
    uint32_t SDA;
    uint32_t SCK;
}lrf_node_acce_t;

typedef struct
{
    lrf_node_mod_rf_params_t radio_params;
    lrf_node_mod_rf_head_t radio_header;
    lrf_node_rf_hw_t radio_gpio;
    lrf_node_rf_spi_t radio_spi;
    lrf_node_acce_t acce_hw_params;
    uint32_t threshold_angle;
    uint32_t rf_tcxo_pin;
}lrf_node_mod_init_t;

/**
 * @brief Function to initialize module
 * @param p_rf_params Structure pointer of type @ref lrf_node_rf_params_t containing all radio parameters
 * @param p_rf_header Structure pointer of type @ref lrf_node_rf_head_t containing rf header data
 */
void lrf_node_mod_init (lrf_node_mod_init_t * p_mod_init);

/**
 * @brief Function to start Module
 * @note call lrf_node_mod_set_angle_threshold before starting this module.
 */
void lrf_node_mod_start ();

/**
 * @brief Function to stop module
 */
void lrf_node_mod_stop ();

/**
 * @brief Function to update header for RF packet
 * @param p_head Pointer to structure of type @ref lrf_node_mod_rf_head_t
 */
void lrf_node_mod_update_rf_head (lrf_node_mod_rf_head_t * p_head);

/**
 * @brief Function to update RF parameters
 * @param p_params Structure pointer of type @ref lrf_node_mod_rf_params_t
 */
void lrf_node_mod_update_rf_params (lrf_node_mod_rf_params_t * p_params);

/**
 * @brief Function to set threshold angle for LRF node.
 * @param angle Threshold angle. If node is tilted more than threshold angle, send RF pkt
 */
void lrf_node_mod_set_angle_threshold (uint8_t angle);

/**
 * @brief Function to get current angle of system
 * @return Current angle in degrees
 */
uint8_t lrf_node_mod_get_angle ();

/**
 * @brief Function to check if node is tilted.
 * @return Tilt status
 * @retval True : Node is tilted
 * @retval False : Node is not tilted
 */
uint8_t lrf_node_mod_is_tilted ();


#endif /* LRF_NODE_RF_H */
