/*
 *  sensepi_ble.c
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

#include "nrf_nvic.h"
#include "ble.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "boards.h"
#include "stddef.h"
#include "common_util.h"
#include "nrf_util.h"
#include "sensepi_ble.h"
#include "log.h"
#include "evt_sd_handler.h"
#include "string.h"

/**< Name of device, to be included in the advertising data. */
#define DEVICE_NAME_CHAR           'S','e','n','s','e','P','i'
const uint8_t device_name[] = { DEVICE_NAME_CHAR };

/** Complete 128 bit UUID of the SensePi service
 * 3c73dc50-07f5-480d-b066-837407fbde0a */
#define SENSEPI_UUID_COMPLETE        0x0a, 0xde, 0xfb, 0x07, 0x74, 0x83, 0x66, 0xb0, 0x0d, 0x48, 0xf5, 0x07, 0x50, 0xdc, 0x73, 0x3c

/** The 16 bit UUID of the Sense Pi service */
#define SENSEPI_UUID_SERVICE         0xdc50

/** The 16 bit UUID of the read-only System Info characteristic */
#define SENSEPI_UUID_SYSINFO         0xdc51
/** The 16 bit UUID of the read-write Config characteristic */
#define SENSEPI_UUID_CONFIG          0xdc52

/**< Interval between advertisement packets (0.5 seconds). */
#define ADVERTISING_INTERVAL       MSEC_TO_UNITS(500, UNIT_0_625_MS)
/**< Minimum acceptable connection interval (0.2 seconds). */
#define MIN_CONN_INTERVAL          MSEC_TO_UNITS(200, UNIT_1_25_MS)
/**< Maximum acceptable connection interval (0.5 second). */
#define MAX_CONN_INTERVAL          MSEC_TO_UNITS(500, UNIT_1_25_MS)
/**< Slave latency (number of skippable conn events by the slave). */
#define SLAVE_LATENCY              0
/**< Connection supervisory timeout (4 seconds). */
#define CONN_SUP_TIMEOUT           MSEC_TO_UNITS(4000, UNIT_10_MS)

/** The data to be sent in the advertising payload. It is of the format
 *  of a sequence of {Len, type, data} */
#define ADV_DATA   {                                        \
                       0x02, BLE_GAP_AD_TYPE_FLAGS, BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE,    \
                       sizeof(device_name) + 1, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, DEVICE_NAME_CHAR,   \
                       0x11, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, SENSEPI_UUID_COMPLETE     \
                   }


/** The data to be sent in the scan response payload. It is of the format
 *  of a sequence of {Len, type, data} */
///TODO Dynamically update the device ID in the scan response data
#define SCAN_RSP_DATA  {                                        \
                           0x02, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, 0   ,     \
                           0x11, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME, 'S', 'P','0', '4', '0', '0' , '1', '8', '0', '5', '1', '1', '0', '0', '0', '1' \
                       }

/** Handle to specify the advertising state to the soft device */
uint8_t h_adv;

/** Handle to specify the connection state to the soft device */
uint16_t h_conn;

/** Handle to specify the attribute of the Sense Pi service */
uint16_t h_sensepi_service;

/** Handle to specify the attribute of the characteristic with
 * the system information containing @ref sensepi_dev_info*/
ble_gatts_char_handles_t h_sysinfo_char;

/** Handle to specify the attribute of the characteristic with the
 * configuration parameters which are specified @ref sensepi_config */
ble_gatts_char_handles_t h_config_char;

/** Handler to pass the BLE SoftDevice events to the application */
void (* sensepi_ble_sd_evt)(ble_evt_t * evt);
/** Handler to pass the received SensePi configuration to the application */
void (* sensepi_config_update)(sensepi_config * cfg);

sensepi_sysinfo curr_sysinfo;

///Called everytime the radio is switched off as per the
/// radio notification by the SoftDevice
void SWI1_IRQHandler(void)
{
//    log_printf("radio going down\n");
}

/**
 * @brief Handler which will address all the SoC related events
 *  generated by the SoftdDevice.
 * @param evt The pointer to the buffer containing all the data
 *  related to the event
 */
static void ble_evt_handler(ble_evt_t * evt)
{
    switch(evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        h_conn = evt->evt.gap_evt.conn_handle;
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        h_conn = BLE_CONN_HANDLE_INVALID;
        break;
    case BLE_GATTS_EVT_WRITE:
    {
        sensepi_config * config =
                (sensepi_config *) evt->evt.gatts_evt.params.write.data;
        sensepi_config_update(config);
    }
        break;
    }
    sensepi_ble_sd_evt(evt);
}

/**
 * @brief Handler which will address all the SoC related events
 *  generated by the SoftdDevice.
 * @param evt_id The ID of the SoC event generated
 */
static void soc_evt_handler(uint32_t evt_id)
{
    log_printf("soc evt %x\n", evt_id);
}

void sensepi_ble_init(void (*ble_sd_evt)(ble_evt_t * evt),
        void (* config_update)(sensepi_config * cfg))
{
    sensepi_ble_sd_evt = ble_sd_evt;
    sensepi_config_update = config_update;
}

void sensepi_ble_disconn(void)
{
    if(h_conn != BLE_CONN_HANDLE_INVALID)
    {
        uint32_t err_code;
        err_code = sd_ble_gap_disconnect(h_conn,
                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
}

void sensepi_ble_update_sysinfo(sensepi_sysinfo * sysinfo)
{
    uint32_t err_code;
    ble_gatts_value_t val =
    {
        .len = sizeof(sensepi_sysinfo),
        .offset = 0,
        .p_value = (uint8_t *) sysinfo
    };
    err_code = sd_ble_gatts_value_set(h_conn,
            h_sysinfo_char.value_handle, &val);
    APP_ERROR_CHECK(err_code);
}

void sensepi_ble_stack_init(void)
{
    uint32_t err_code;
    const nrf_clock_lf_cfg_t cfg = BOARD_LFCLKSRC_STRUCT;

    err_code = sd_softdevice_enable(&cfg, app_error_fault_handler);
    APP_ERROR_CHECK(err_code);

    uint32_t app_ram_start = 0x20001c00;
    log_printf("Init %x", app_ram_start);
    err_code = sd_ble_enable(&app_ram_start);
    log_printf(" RAM needed %x\n", app_ram_start);
    APP_ERROR_CHECK(err_code);

    evt_sd_handler_init(ble_evt_handler, soc_evt_handler);

    //So that the application wakes up after every radio activity
    err_code = sd_radio_notification_cfg_set(
            NRF_RADIO_NOTIFICATION_TYPE_INT_ON_INACTIVE,
            NRF_RADIO_NOTIFICATION_DISTANCE_NONE);
    APP_ERROR_CHECK(err_code);

    // Initialize Radio Notification software interrupt
    err_code = sd_nvic_ClearPendingIRQ(SWI1_IRQn);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_SetPriority(SWI1_IRQn, APP_IRQ_PRIORITY_LOWEST);
    APP_ERROR_CHECK(err_code);

    err_code = sd_nvic_EnableIRQ(SWI1_IRQn);
    APP_ERROR_CHECK(err_code);

    h_conn = BLE_CONN_HANDLE_INVALID;
}

void sensepi_ble_service_init(void)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    uint8_t uuid_type;

    /**** Create the Sense Pi service *****/
    ble_uuid128_t base_uuid = {{SENSEPI_UUID_COMPLETE}};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &uuid_type);
    APP_ERROR_CHECK(err_code);

    ble_uuid.type = uuid_type;
    ble_uuid.uuid = SENSEPI_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
            &ble_uuid, &h_sensepi_service);
    APP_ERROR_CHECK(err_code);

    /**** Create the read-only characteristic *****/
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_gatts_attr_md_t attr_md;

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.char_props.write = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = uuid_type;
    ble_uuid.uuid = (SENSEPI_UUID_SYSINFO);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(sensepi_sysinfo);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(sensepi_sysinfo);
    attr_char_value.p_value = NULL;

    err_code = sd_ble_gatts_characteristic_add(
        h_sensepi_service, &char_md, &attr_char_value, &h_sysinfo_char);
    APP_ERROR_CHECK(err_code);

    /**** Create the read-write characterisitc *****/
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = uuid_type;
    ble_uuid.uuid = (SENSEPI_UUID_CONFIG);

    memset(&attr_md, 0, sizeof(attr_md));

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.write_perm);
    attr_md.vloc = BLE_GATTS_VLOC_STACK;
    attr_md.rd_auth = 0;
    attr_md.wr_auth = 0;
    attr_md.vlen = 0;

    memset(&attr_char_value, 0, sizeof(attr_char_value));

    attr_char_value.p_uuid = &ble_uuid;
    attr_char_value.p_attr_md = &attr_md;
    attr_char_value.init_len = sizeof(sensepi_config);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(sensepi_config);
    attr_char_value.p_value = NULL;

    err_code = sd_ble_gatts_characteristic_add(
        h_sensepi_service, &char_md, &attr_char_value,&h_config_char);
    APP_ERROR_CHECK(err_code);
}

void sensepi_ble_gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(
        &sec_mode, (const uint8_t *)device_name, sizeof(device_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void sensepi_ble_adv_init(void)
{
    h_adv = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
    uint32_t err_code;
    uint8_t adv_buffer[] = ADV_DATA;
    uint8_t scan_rsp_buffer[] = SCAN_RSP_DATA;

    ble_gap_adv_data_t adv_payload;

    adv_payload.adv_data.p_data = adv_buffer;
    adv_payload.adv_data.len = sizeof(adv_buffer);

    adv_payload.scan_rsp_data.p_data = scan_rsp_buffer;
    adv_payload.scan_rsp_data.len = sizeof(scan_rsp_buffer);

    ble_gap_adv_params_t adv_params;

    memset(&adv_params, 0, sizeof(adv_params));

    //Set channel 37, 38 and 39 as advertising channels
    memset(adv_params.channel_mask, 0, 5);

    //Set the advertising to timeout in 180s
    adv_params.duration = BLE_GAP_ADV_TIMEOUT_LIMITED_MAX;

    //Any device can scan request and connect
    adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;

    //Period between sending advertising packets
    adv_params.interval = ADVERTISING_INTERVAL;

    //Use 1Mbps physical layer to be backward compatible
    adv_params.primary_phy = BLE_GAP_PHY_1MBPS;

    //The advertisement would be unidirected connectable and scannable
    adv_params.properties.type =
            BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;

    adv_params.p_peer_addr = NULL;
    adv_params.scan_req_notification = 0;

    err_code = sd_ble_gap_adv_set_configure(&h_adv,
            (ble_gap_adv_data_t const *)&adv_payload,
            (ble_gap_adv_params_t const *) &adv_params);
    APP_ERROR_CHECK(err_code);
}

void sensepi_ble_adv_start(void)
{
    uint32_t err_code;
    err_code = sd_ble_gap_adv_start(h_adv, BLE_CONN_CFG_TAG_DEFAULT);
    APP_ERROR_CHECK(err_code);
}

