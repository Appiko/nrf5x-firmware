/* 
 * File:   rf_rx_ble.h
 * Copyright (c) 2018 Appiko
 * Created on 28 March, 2019, 16:49 AM
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
rf_rx_ble_adv_data_t;

typedef struct
{
    uint8_t rf_rx_rssi;
}mod_ble_data_t;

/**
 * @brief Disconnect the current active connection, if already connected
 */
void rf_rx_ble_disconn(void);

/**
 * @brief Function for initializing the BLE stack by enabling the
 *  SoftDevice and the BLE event interrupt
 * */
void rf_rx_ble_stack_init();

/**
 * @brief Create the Service and its characteristics for
 *  the SensePi device. There is a read-only characteristic
 *  that provides all the info from the device and a
 *  read-write characteristic that is used to set the
 *  operational configuration of the device.
 */
void rf_rx_ble_service_init(void);

/**
 * @brief Generic Access Profile initialization. The device name,
 *  and the preferred connection parameters are setup.
 */
void rf_rx_ble_gap_params_init(void);

/**
 * @brief Function to initializing the advertising
 * @param rf_rx_ble_adv_data Advaertise data and scan response data along with
 * their respective lengths.
 */
void rf_rx_ble_adv_init(void);

/**
 * @brief Function to start advertising.
 */
void rf_rx_ble_adv_start(void (*conn_func) (void));

bool rf_rx_is_bt_on ();

void rf_rx_ble_update_status_byte(mod_ble_data_t * status_byte);

#endif /* BT_DONGLE_BLE_H */
