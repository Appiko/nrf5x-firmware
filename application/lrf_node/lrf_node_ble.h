/*
 *  lrf_node_ble.h : BLE Support file for SenseBe application 
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
 * @addtogroup lrf_node_appln
 * @{
 *
 * @defgroup ble_support The support code for the active IR based Sense units.
 * @brief The active IR sense application's support file that handles ble operations.
 *
 * @{
 *
 */

#ifndef APPLICATION_LRF_NODE_BLE_H_
#define APPLICATION_LRF_NODE_BLE_H_

#include "stdint.h"
#include "stdbool.h"
#include "ble.h"
#include "dev_id_fw_ver.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif

/**< Name of device, to be included in the advertising data. */

#define DEVICE_NAME_CHAR           'S','e','n','s','e','E','l','e',' ','N'


/** Complete 128 bit UUID of the SenseBe service
 * 3c73dc60-07f5-480d-b066-837407fbde0a */
#ifdef DEVICE_UUID_COMPLETE
#define LRF_NODE_UUID_COMPLETE        DEVICE_UUID_COMPLETE
#else
#define LRF_NODE_UUID_COMPLETE        0x0a, 0xde, 0xfb, 0x07, 0x74, 0x83, 0x66, 0xb0, 0x0d, 0x48, 0xf5, 0x07, 0x60, 0xdc, 0x73, 0x3c
#endif
/** The 16 bit UUID of the Sense Be service */
#ifdef DEVICE_UUID_SERVICE
#define LRF_NODE_UUID_SERVICE         DEVICE_UUID_SERVICE
#else
#define LRF_NODE_UUID_SERVICE         0xdc60
#endif
/** The 16 bit UUID of the read-only System Info characteristic */
#ifdef DEVICE_UUID_SYSINFO
#define LRF_NODE_UUID_SYSINFO         DEVICE_UUID_SYSINFO
#else
#define LRF_NODE_UUID_SYSINFO         0xdc61
#endif
/** The 16 bit UUID of the read-write Config characteristic */
#ifdef DEVICE_UUID_PRODUCT_INFO
#define LRF_NODE_UUID_PRODUCT_INFO   DEVICE_UUID_PRODUCT_INFO
#else
#define LRF_NODE_UUID_PRODUCT_INFO   0xdc62
#endif
/** The 16 bit UUID of the read-write Config characteristic */
#ifdef DEVICE_UUID_DPLOYMENT_FLAG
#define LRF_NODE_UUID_DPLY_FLAG       DEVICE_UUID_DPLOYMENT_FLAG
#else
#define LRF_NODE_UUID_DPLY_FLAG       0xdc63
#endif
/** The 16 bit UUID of the read-write Config characteristic */
#ifdef DEVICE_UUID_DFU_FLAG
#define LRF_NODE_UUID_DFU_FLAG        DEVICE_UUID_DFU_FLAG
#else
#define LRF_NODE_UUID_DFU_FLAG        0xdc64
#endif
/** The 16 bit UUID of the read-write Config characteristic */
#ifdef DEVICE_UUID_ALIGN_FLAG
#define LRF_NODE_UUID_ALIGN_FLAG     DEVICE_UUID_ALIGN_FLAG
#else
#define LRF_NODE_UUID_ALIGN_FLAG     0xdc65
#endif


typedef struct {
    dev_id_t id;
    uint8_t battery_status;
    fw_ver_t fw_ver;
} __attribute__((packed)) lrf_node_sysinfo;



typedef struct
{
    uint8_t app_id;
    uint16_t prod_id;
}__attribute__((packed)) lrf_node_prodc_info_t;

typedef struct
{
    uint8_t deploy_flag;
    uint8_t align_flag;
}__attribute__((packed)) lrf_node_flag_dply_t;


typedef struct
{
    uint8_t dfu_flag;
}lrf_node_flag_dfu_t;

typedef struct
{
    uint8_t adv_data[31];
    uint16_t adv_len;
    uint8_t scan_rsp_data[31];
    uint16_t scan_rsp_len;
}__attribute__((packed)) lrf_node_ble_adv_data_t;

/**
 * @brief Initialize the handlers to pass the BLE SD events
 *  and the configuration received from the mobile app
 * @param ble_sd_evt Handler to send the BLE events to the application
 * @param dply_flag_update Handler to send Deployment flag to the application
 * @param dfu_flag_update Handler to send DFU flag to the application
 */
void lrf_node_ble_init(void (*ble_sd_evt)(ble_evt_t * evt), 
        void (* dply_flag_update)(uint8_t dply_flag), 
        void (* dfu_flag_update)(uint8_t dfu_flag),
        void (* product_info_update)(lrf_node_prodc_info_t * product_info));

/**
 * @brief Updates the characteristic that stores the sysinfo
 * @param sysinfo A pointer to the Sense Pi info for the updation
 */
void lrf_node_ble_update_sysinfo(lrf_node_sysinfo * sysinfo);

/**
 * @brief Updates the characteristic that stores the SensePi config
 * @param config A pointer to the new Sense Pi configuration
 */
void lrf_node_ble_update_product_info(lrf_node_prodc_info_t * product_info);

/**
 * @brief Function to update Deployment and Alignment data
 * @param p_dply_align A pointer to data.
 */
void lrf_node_ble_update_dply_alignment(lrf_node_flag_dply_t * p_dply_align);

/**
 * @brief Disconnect the current active connection, if already connected
 */
void lrf_node_ble_disconn(void);

/**
 * @brief Function for initializing the BLE stack by enabling the
 *  SoftDevice and the BLE event interrupt
 * */
void lrf_node_ble_stack_init(void);

/**
 * @brief Create the Service and its characteristics for
 *  the SensePi device. There is a read-only characteristic
 *  that provides all the info from the device and a
 *  read-write characteristic that is used to set the
 *  operational configuration of the device.
 */
void lrf_node_ble_service_init(void);

/**
 * @brief Generic Access Profile initialization. The device name,
 *  and the preferred connection parameters are setup.
 */
void lrf_node_ble_gap_params_init(void);

/**
 * @brief Function to initializing the advertising
 * @param lrf_node_ble_adv_data Advaertise data and scan response data along with
 * their respective lengths.
 */
void lrf_node_ble_adv_init(lrf_node_ble_adv_data_t * lrf_node_ble_adv_data);

/**
 * @brief Function to start advertising.
 */
void lrf_node_ble_adv_start(void);

/**
 * @brief Function to set advertising data.
 * @param lrf_node_ble_adv_data Structure where advertising data is to be stored
 * @param device_name User defined device name.
 * @param batt_type Battery type @ref battery_type_t
 */
void lrf_node_ble_set_adv_data(lrf_node_ble_adv_data_t * lrf_node_ble_adv_data,
             uint8_t * device_name);


/**
 * @brief Function to send the notification
 * @param p_align
 */
void lrf_node_align_notify (lrf_node_flag_dply_t * p_align);

#endif /* APPLICATION_LRF_NODE_BLE_H_ */

/**
 * @}
 * @}
 */
