/*
 *  sensepi_ble.h
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

#ifndef APPLICATION_SENSE_PIR_SENSEPI_BLE_H_
#define APPLICATION_SENSE_PIR_SENSEPI_BLE_H_

#include "stdint.h"
#include "stdbool.h"
#include "ble.h"

typedef struct
{
    uint8_t prod_code[2];
    uint8_t prod_rev[2];
    uint8_t factory_code[2];
    uint8_t year[2];
    uint8_t month[2];
    uint8_t day[2];
    uint8_t serial_no[4];
}__attribute__ ((packed)) device_id_t ;

typedef struct
{
    device_id_t id;
    bool is_battery_low;
}__attribute__ ((packed)) sensepi_sysinfo ;

typedef enum
{
    PIR_ONLY,
    TIMER_ONLY,
    PIR_AND_TIMER,
}trigger_conf_t;

typedef struct
{
    uint8_t day_or_night: 1;
    uint8_t threshold: 7;
}__attribute__ ((packed)) oper_time_t;

typedef struct
{
    oper_time_t oper_time;
    uint32_t mode;
    uint8_t threshold;
    uint8_t amplification;
    uint16_t intr_trig_timer; 
}__attribute__ ((packed)) pir_conf_t;

typedef struct
{
    uint16_t timer_interval;
    oper_time_t oper_time;
    uint32_t mode;
}__attribute__ ((packed)) timer_conf_t;

typedef struct
{
    trigger_conf_t trig_conf;
    pir_conf_t * pir_conf;
    timer_conf_t * timer_conf;
}__attribute__ ((packed)) sensepi_config ;

/**
 * @brief Initialize the handlers to pass the BLE SD events
 *  and the configuration received from the mobile app
 * @param ble_sd_evt Handler to send the BLE events to the application
 * @param config_update Handler to send SensePi config to the application
 */
void sensepi_ble_init(void (*ble_sd_evt)(ble_evt_t * evt), 
        void (* config_update)(sensepi_config * cfg));

/**
 * @brief Updates the characteristic that stores the sysinfo
 * @param sysinfo A pointer to the Sense Pi info for the updation
 */
void sensepi_ble_update_sysinfo(sensepi_sysinfo * sysinfo);

/**
 * @brief Disconnect the current active connection, if already connected
 */
void sensepi_ble_disconn(void);

/**
 * @brief Function for initializing the BLE stack by enabling the
 *  SoftDevice and the BLE event interrupt
 * */
void sensepi_ble_stack_init(void);

/**
 * @brief Create the Service and its characteristics for
 *  the SensePi device. There is a read-only characteristic
 *  that provides all the info from the device and a
 *  read-write characteristic that is used to set the
 *  operational configuration of the device.
 */
void sensepi_ble_service_init(void);

/**
 * @brief Generic Access Profile initialization. The device name,
 *  and the preferred connection parameters are setup.
 */
void sensepi_ble_gap_params_init(void);

/**@brief Function to initializing the advertising
 */
void sensepi_ble_adv_init(void);

/**
 * @brief Function to start advertising.
 */
void sensepi_ble_adv_start(void);

#endif /* APPLICATION_SENSE_PIR_SENSEPI_BLE_H_ */
