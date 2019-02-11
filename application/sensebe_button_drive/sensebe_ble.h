/*
 *  sensebe_ble.h
 *
 *  Created on: 09-May-2018
 *
 *  Copyright (c) 2018, Appiko
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software without
 *  specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

/**
 * @addtogroup sense_appln
 * @{
 *
 * @defgroup ble_support The support code for the PIR based Sense units.
 * @brief The PIR sense application's support file that handles ble operations.
 *
 * @{
 *
 */

#ifndef APPLICATION_SENSE_PIR_SENSEBE_BLE_H_
#define APPLICATION_SENSE_PIR_SENSEBE_BLE_H_

#include "stdint.h"
#include "stdbool.h"
#include "ble.h"
#include "dev_id_fw_ver.h"

typedef struct
{
    dev_id_t id;
    uint8_t battery_status;
    fw_ver_t fw_ver;
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
 * @brief Strcture to configure PIR sensing. 
 */
typedef struct
{
    /** To decide in which light condition PIR should operate. */
    oper_time_t oper_time;     
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
    uint8_t mode;
    uint16_t larger_value;
    uint8_t smaller_value;
    /** In tssp_Conf: Detection window duration in resolution of 100ms. */
    uint16_t detect_window;
    /** In tssp_Conf: Time between triggers by a PIR in s (with a resolution of 0.1s). */
    uint16_t intr_trig_timer; 
}__attribute__ ((packed)) tssp_conf_t;

/**
 * @brief Structure to configure Timer triggering.
 */
typedef struct
{
    /** In timer_conf: Interval between two triggers. */
    uint16_t timer_interval;
    oper_time_t oper_time;
    uint8_t mode;
    uint16_t larger_value;
    uint8_t smaller_value;
}__attribute__ ((packed)) timer_conf_t;

/**
 * @brief Strcture which is used for data transfer over BLE.
 */
typedef struct 
{
    /** In sensebe_conf: Mode of operation */
    trigger_conf_t trig_conf;
    /** In sensebe_conf: TSSP Configuration */
    tssp_conf_t  tssp_conf;
    /** In sensebe_conf: Time configuration */
    timer_conf_t  timer_conf;
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

#endif /* APPLICATION_SENSE_PIR_SENSEBE_BLE_H_ */

/**
 * @}
 * @}
 */
