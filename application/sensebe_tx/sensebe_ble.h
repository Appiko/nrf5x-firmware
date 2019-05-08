/*
 *  sensebe_ble.h : BLE Support file for SenseBe application 
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
 * @defgroup ble_support The support code for the active IR based Sense units.
 * @brief The active IR sense application's support file that handles ble operations.
 *
 * @{
 *
 */

#ifndef APPLICATION_SENSEBE_BLE_H_
#define APPLICATION_SENSEBE_BLE_H_

#include "stdint.h"
#include "stdbool.h"
#include "ble.h"
#include "dev_id_fw_ver.h"

typedef struct
{
    dev_id_t id;
    uint8_t battery_status;
    fw_ver_t fw_ver;
    uint8_t board_sel;
}__attribute__ ((packed)) sensebe_sysinfo ;
/**
 * @brief Enum of all posiible modes of operation.
 */
typedef enum
{
    TIMER_ONLY,     ///Trigger only on timer
    MOTION_ONLY,       ///Trigger only on motion detection
    MOTION_AND_TIMER,  ///Trigger on both motion detection or timer.
}trigger_conf_t;

/**
 * @brief Enum to select operation speed for Device
 */
typedef enum 
{
    LIGHTNING,  ///Lightning mode
    FAST, ///Fast mode
    NORM,     ///Normal mode
    SLOWMO,     ///Slow mode
}device_speed_t;

/**
 * @brief Strcture for Operation time.
 */
typedef struct
{
    /**
     * @note 
     * If LSB is 1 then SensePi is operational above the threshold light intensity.
     * If LSB is 0 then SensePi is operational below the threshold light intensity.
     * This one bit value is used to decide when SensePi should operate.
     */
    uint8_t day_or_night: 1;
    /** In Oper_Time: Threshold for light intensity. */
    uint8_t threshold: 7;
}__attribute__ ((packed)) oper_time_t;

/**
 * @brief Structure to control camera operations
 */
typedef struct
{
    /** To decide if pre_focus pulse of 1.5 sec is required or not */
    uint8_t pre_focus : 1;
    /**
     * MODE DATA FORMAT:
     *  |31  (bits) 24|23  (bits)     8|7 (bits) 0| 
     *  |:-----------:|:--------------:|:--------:| 
     *  |Smaller Value|Larger value    |Mode      | 
     *  |1 Byte       |    2 Bytes     | 1 Byte   | 
     *  
     *  Modes: \n
     *  \n Mode 0 : Single Shot \n
     *  Larger Value    : -- \n
     *  Smaller Value   : -- \n
     *  \n Mode 1 : Multishot Burst \n
     *  Larger Value    : Time between 2 shots in s (with a resolution of 0.1s).
     *                   min = 0.5s, max = 60s, unit of (32000/32768) ms\n
     *  Smaller Value   : Number of shots. min = 2, max = 255\n
     *  \n Mode 2 : Bulb Exposure\n
     *  Smaller + Larger Value : Time for exposure in s (with a resolution of 0.1s)
     *                               min = 0.5s, max = (2^24-1), unit of (32000/32768) ms\n
     *  \n Mode 3 : Video\n
     *  Larger Value    : Duration of video in sec; min = 1s, max = 10000s\n
     *  Smaller Value   : Extentation in sec; min = 1s, max = 250s\n
     *  \n Mode 4 : Set focus\n
     *  Larger Value    : --\n
     *  Smaller Value   : --\n
     */
    uint8_t mode : 7;
    uint16_t larger_value;
    uint8_t smaller_value;
}__attribute__ ((packed)) cam_oper_t;

/**
 * @brief Enum to store the list of camera triggers possible
 */
typedef enum
{
    TIMER_ALL,        

//    MOTION_ALL,         //at rx side

//    RADIO_ALL,        //at tx side

//    TIMER_DAY,
//    TIMER_NIGHT,
            
//    MOTION_DAY,
//    MOTION_NIGHT,

//    RADIO_DAY,
//    RADIO_NIGHT,
    MAX_TRIGGERS,
}cam_trig_ls_t;

/**
 * @brief Structure to configure Timer triggering.
 */
typedef struct
{
    /** In timer_conf: Interval between two triggers. */
    uint16_t timer_interval;
    /** To decide in which light condition timer should operate. */
    oper_time_t oper_time;
}__attribute__ ((packed)) timer_conf_t;

/***/
typedef struct
{
    /** To decide in which light condition IR transmitter should operate. */
    oper_time_t oper_time;
    /** To decide if IR transmitter should be Enabled */
    uint8_t is_enable: 1;
    /**
     * @brief To decide the time between 2 signals for IR transmitter. 
     * @TIME 
     * 0  5ms\
     * 1  25ms\
     * 2  50ms\
     * 3  100ms\
     */
    uint8_t ir_tx_speed: 2;
    /**
     * @breif To decide the transmission power for IR transmitter.
     * @POWER
     * 0    Low\
     * 1    Mid\
     * 2,3  High\ 
     */
    uint8_t ir_tx_pwr: 2;
}__attribute__ ((packed)) ir_tx_conf_t;

/**
 * @brief Strcture which is used for data transfer over BLE.
 */
typedef struct 
{
    /** In sensebe_conf: Mode of operation */
    trigger_conf_t trig_conf;
    /** Array of all the possible triggers */
    cam_oper_t cam_trigs[MAX_TRIGGERS];      
    /** In sensebe_conf: Device Spped*/
    device_speed_t speed;
    /** In sensebe_conf: Timer Configuration */
    timer_conf_t  timer_conf;
    /** In sensebe_conf: IR transmitter Configuration*/
    ir_tx_conf_t ir_tx_conf;
}__attribute__ ((packed)) sensebe_config_t ;

typedef struct
{
    uint8_t * adv_data;
    uint16_t adv_len;
    uint8_t * scan_rsp_data;
    uint16_t scan_rsp_len;
}
sensebe_ble_adv_data_t;

/**
 * @brief Initialize the handlers to pass the BLE SD events
 *  and the configuration received from the mobile app
 * @param ble_sd_evt Handler to send the BLE events to the application
 * @param config_update Handler to send SensePi config to the application
 */
void sensebe_ble_init(void (*ble_sd_evt)(ble_evt_t * evt), 
        void (* config_update)(sensebe_config_t * cfg));

/**
 * @brief Updates the characteristic that stores the sysinfo
 * @param sysinfo A pointer to the Sense Pi info for the updation
 */
void sensebe_ble_update_sysinfo(sensebe_sysinfo * sysinfo);

/**
 * @brief Updates the characteristic that stores the SensePi config
 * @param config A pointer to the new Sense Pi configuration
 */
void sensebe_ble_update_config(sensebe_config_t * config);

/**
 * @brief Disconnect the current active connection, if already connected
 */
void sensebe_ble_disconn(void);

/**
 * @brief Function for initializing the BLE stack by enabling the
 *  SoftDevice and the BLE event interrupt
 * */
void sensebe_ble_stack_init(void);

/**
 * @brief Create the Service and its characteristics for
 *  the SensePi device. There is a read-only characteristic
 *  that provides all the info from the device and a
 *  read-write characteristic that is used to set the
 *  operational configuration of the device.
 */
void sensebe_ble_service_init(void);

/**
 * @brief Generic Access Profile initialization. The device name,
 *  and the preferred connection parameters are setup.
 */
void sensebe_ble_gap_params_init(void);

/**
 * @brief Function to initializing the advertising
 * @param sensebe_ble_adv_data Advaertise data and scan response data along with
 * their respective lengths.
 */
void sensebe_ble_adv_init(sensebe_ble_adv_data_t * sensebe_ble_adv_data);

/**
 * @brief Function to start advertising.
 */
void sensebe_ble_adv_start(void);

#endif /* APPLICATION_SENSEBE_BLE_H_ */

/**
 * @}
 * @}
 */
