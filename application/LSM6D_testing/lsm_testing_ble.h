/*
 *  lsm_testing_ble.h : BLE Support file for LSM6D application 
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

#ifndef BT_DONGLE_BLE_H
#define BT_DONGLE_BLE_H

#include "stdint.h"
#include "stdbool.h"
#include "ble.h"
#include "dev_id_fw_ver.h"

typedef struct
{
    uint8_t * adv_data;
    uint16_t adv_len;
    uint8_t * scan_rsp_data;
    uint16_t scan_rsp_len;
}
lsm_ble_adv_data_t;

typedef struct
{
    uint16_t avg_acce;
    uint16_t avg_smoke;
}mod_ble_data_t;

/**
 * @brief Disconnect the current active connection, if already connected
 */
void lsm_ble_disconn(void);

/**
 * @brief Function for initializing the BLE stack by enabling the
 *  SoftDevice and the BLE event interrupt
 * */
void lsm_ble_stack_init();

/**
 * @brief Create the Service and its characteristics for
 *  the SensePi device. There is a read-only characteristic
 *  that provides all the info from the device and a
 *  read-write characteristic that is used to set the
 *  operational configuration of the device.
 */
void lsm_ble_service_init(void);

/**
 * @brief Generic Access Profile initialization. The device name,
 *  and the preferred connection parameters are setup.
 */
void lsm_ble_gap_params_init(void);

/**
 * @brief Function to initializing the advertising
 * @param lsm_ble_adv_data Advaertise data and scan response data along with
 * their respective lengths.
 */
void lsm_ble_adv_init(void);

/**
 * @brief Function to start advertising.
 */
void lsm_ble_adv_start(void (*conn_func) (void));

bool lsm_is_bt_on ();

void lsm_ble_update_status_byte(mod_ble_data_t * status_byte);

#endif /* BT_DONGLE_BLE_H */
