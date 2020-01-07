/*
 *  lrf_node_ble.c : BLE Support file for SenseBe application 
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

#include "nrf_nvic.h"
#include "ble.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "boards.h"
#include "stddef.h"
#include "common_util.h"
#include "nrf_util.h"
#include "lrf_node_ble.h"
#include "log.h"
#include "evt_sd_handler.h"
#include "string.h"

#if ISR_MANAGER == 1
#include "isr_manager.h"
#endif

///** Complete 128 bit UUID of the SenseBe service
// * 3c73dc60-07f5-480d-b066-837407fbde0a */
//#ifdef DEVICE_UUID_COMPLETE
//#define SENSEBERXUUID_COMPLETE        {DEVICE_UUID_COMPLETE}
//#else
//#define SENSEBERXUUID_COMPLETE        {0x0a, 0xde, 0xfb, 0x07, 0x74, 0x83, 0x66, 0xb0, 0x0d, 0x48, 0xf5, 0x07, 0x60, 0xdc, 0x73, 0x3c}
//#endif
///** The 16 bit UUID of the Sense Be service */
//#ifdef DEVICE_UUID_SERVICE
//#define SENSEBERXUUID_SERVICE         DEVICE_UUID_SERVICE
//#else
//#define SENSEBERXUUID_SERVICE         0xdc60
//#endif
///** The 16 bit UUID of the read-only System Info characteristic */
//#ifdef DEVICE_UUID_SYSINFO
//#define SENSEBERXUUID_SYSINFO         DEVICE_UUID_SYSINFO
//#else
//#define SENSEBERXUUID_SYSINFO         0xdc61
//#endif
///** The 16 bit UUID of the read-write Config characteristic */
//#ifdef DEVICE_UUID_CONFIG
//#define SENSEBERXUUID_CONFIG         DEVICE_UUID_CONFIG
//#else
//#define SENSEBERXUUID_CONFIG         0xdc62
//#endif
const uint8_t ble_device_name[] = { DEVICE_NAME_CHAR };

#define COMPLETE_UUID   {LRF_NODE_UUID_COMPLETE}
#define SERVICE_UUID    LRF_NODE_UUID_SERVICE
#define SYSINFO_UUID    LRF_NODE_UUID_SYSINFO
#define PRODINFO_UUID   LRF_NODE_UUID_PRODUCT_INFO
#define DPLY_FLAG_UUID  LRF_NODE_UUID_DPLY_FLAG
#define DFU_FLAG_UUID   LRF_NODE_UUID_DFU_FLAG

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


#define DEVICE_NAME_ADV_LOCATION 9
#define DEVICE_NAME_LEN 16

//#define BATTERY_TYPE_ADV_LOCATION 27
//#define BATTERY_LEN 2

#define UUID_SCAN_RSP_LOCATION 2
#define UUID_LEN    16

const uint8_t generic_adv_data[] = {0x02,
    BLE_GAP_AD_TYPE_FLAGS, BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE,
    0x03, BLE_GAP_AD_TYPE_16BIT_SERVICE_UUID_MORE_AVAILABLE,
    0x60, 0xdc, 0x11, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,
    'x','x','x','x','x','x','x','x','x','x','x','x','x','x','x','x',
//    0x05, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 'x','x'
};

const uint8_t generic_scn_rsp[] = {    0x11, 
BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, LRF_NODE_UUID_COMPLETE };

/** Handle to specify the advertising state to the soft device */
uint8_t h_adv;

/** Handle to specify the connection state to the soft device */
uint16_t h_conn;

/** Handle to specify the attribute of the Sense Pi service */
uint16_t h_lrf_node_service;

/** Handle to specify the attribute of the characteristic with
 * the system information containing @ref lrf_node_dev_info*/
ble_gatts_char_handles_t h_sysinfo_char;

/** Handle to specify the attribute of the characteristic with the
 * configuration parameters which are specified @ref lrf_node_config_t */
ble_gatts_char_handles_t h_product_char;
ble_gatts_char_handles_t h_dfu_char;
ble_gatts_char_handles_t h_dply_char;
ble_gatts_char_handles_t h_align_char;

/** Handler to pass the BLE SoftDevice events to the application */
void (* lrf_node_ble_sd_evt)(ble_evt_t * evt);
/** Handler to pass the received DFU status flag to the application */
void (* lrf_node_dfu_update)(uint8_t dfu_flag);
/** Handler to pass the received Deployment status flag to the application */
void (* lrf_node_dply_update)(uint8_t dply_flag);

lrf_node_sysinfo curr_sysinfo;

///Called everytime the radio is switched off as per the
/// radio notification by the SoftDevice
#if ISR_MANAGER == 1
void lrf_node_ble_swi_Handler ()
#else
void SWI1_IRQHandler(void)
#endif
{
//    log_printf("radio going down\n");
}

/**
 * @brief Handler which will address the BLE related events
 *  generated by the SoftDevice for BLE related activities.
 *  The event is passed on to the application for its activities.
 * @param evt The pointer to the buffer containing all the data
 *  related to the event
 */
static void ble_evt_handler(ble_evt_t * evt)
{
    uint32_t err_code;
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
        log_printf("Written to : 0x%x\n", evt->evt.gatts_evt.params.write.handle);
        log_printf("Value handles : 0x%x 0x%x\n", h_dply_char.value_handle,
                   h_dfu_char.value_handle);
//        lrf_node_config_t * config =
//                (lrf_node_config_t *) evt->evt.gatts_evt.params.write.data;
//        lrf_node_config_t_update(config);
        if (evt->evt.gatts_evt.params.write.handle == h_dply_char.value_handle)
        {
            lrf_node_flag_dply_t * p_dply = 
                (lrf_node_flag_dply_t *) evt->evt.gatts_evt.params.write.data;
            uint8_t flag = p_dply->deploy_flag;
            lrf_node_dply_update (flag);
        }

        if (evt->evt.gatts_evt.params.write.handle == h_dfu_char.value_handle)
        {
            uint8_t flag = evt->evt.gatts_evt.params.write.data[0];
            lrf_node_dfu_update (flag);
        }
        break;
    }
    case BLE_GATTS_EVT_EXCHANGE_MTU_REQUEST:
    {
        uint16_t mtu_val = BLE_GATT_ATT_MTU_DEFAULT;
        err_code = sd_ble_gatts_exchange_mtu_reply(h_conn, mtu_val);
        APP_ERROR_CHECK(err_code);
        break;
    }
    case BLE_GAP_EVT_PHY_UPDATE_REQUEST:
    {
        ble_gap_phys_t const phys = {
            .rx_phys = BLE_GAP_PHY_AUTO,
            .tx_phys = BLE_GAP_PHY_AUTO,
        };

        err_code = sd_ble_gap_phy_update(h_conn, &phys);
        APP_ERROR_CHECK(err_code);
        break;
    }
    case BLE_GAP_EVT_PHY_UPDATE:
    {
        log_printf("Tx_get : %x  Rx_get : %x  Status : %x\n",
                   evt->evt.gap_evt.params.phy_update.tx_phy, 
                   evt->evt.gap_evt.params.phy_update.rx_phy,
                   evt->evt.gap_evt.params.phy_update.status);
        break;
    }
    }

    lrf_node_ble_sd_evt(evt);
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

void lrf_node_ble_init(void (*ble_sd_evt)(ble_evt_t * evt), 
        void (* dply_flag_update)(uint8_t dply_flag), 
        void (* dfu_flag_update)(uint8_t dfu_flag))
{
    lrf_node_ble_sd_evt = ble_sd_evt;
    lrf_node_dfu_update = dfu_flag_update;
    lrf_node_dply_update = dply_flag_update;
}

void lrf_node_ble_disconn(void)
{
    if(h_conn != BLE_CONN_HANDLE_INVALID)
    {
        uint32_t err_code;
        err_code = sd_ble_gap_disconnect(h_conn,
                BLE_HCI_REMOTE_USER_TERMINATED_CONNECTION);
        APP_ERROR_CHECK(err_code);
    }
}

void lrf_node_ble_update_sysinfo(lrf_node_sysinfo * sysinfo)
{
    uint32_t err_code;
    ble_gatts_value_t val =
    {
        .len = sizeof(lrf_node_sysinfo),
        .offset = 0,
        .p_value = (uint8_t *) sysinfo
    };
    err_code = sd_ble_gatts_value_set(h_conn,
            h_sysinfo_char.value_handle, &val);
    APP_ERROR_CHECK(err_code);
}

void lrf_node_ble_update_prodict_info(lrf_node_prodc_info_t * product_info)
{
    uint32_t err_code;
    ble_gatts_value_t val =
    {
        .len = sizeof(lrf_node_prodc_info_t),
        .offset = 0,
        .p_value = (uint8_t *) product_info
    };
    err_code = sd_ble_gatts_value_set(h_conn,
            h_product_char.value_handle, &val);
    APP_ERROR_CHECK(err_code);
}

void lrf_node_ble_update_dply_alignment(lrf_node_flag_dply_t * p_dply_align)
{
    uint32_t err_code;
    ble_gatts_value_t val =
    {
        .len = sizeof(lrf_node_flag_dply_t),
        .offset = 0,
        .p_value = (uint8_t *) p_dply_align
    };
    err_code = sd_ble_gatts_value_set(h_conn,
            h_dply_char.value_handle, &val);
    APP_ERROR_CHECK(err_code);
}


void lrf_node_ble_stack_init(void)
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

void lrf_node_ble_service_init(void)
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    uint8_t uuid_type;

    /**** Create the Sense Pi service *****/
    ble_uuid128_t base_uuid = {COMPLETE_UUID};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &uuid_type);
    APP_ERROR_CHECK(err_code);

    ble_uuid.type = uuid_type;
    ble_uuid.uuid = SERVICE_UUID;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
            &ble_uuid, &h_lrf_node_service);
    APP_ERROR_CHECK(err_code);

    /**** Create the read-only characteristics *****/
    ble_gatts_char_md_t char_md;
    ble_gatts_attr_t attr_char_value;
    ble_gatts_attr_md_t attr_md;

    /**     1st     */
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.char_props.write = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = uuid_type;
    ble_uuid.uuid = (SYSINFO_UUID);

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
    attr_char_value.init_len = sizeof(lrf_node_sysinfo);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(lrf_node_sysinfo);
    attr_char_value.p_value = NULL;

    err_code = sd_ble_gatts_characteristic_add(
        h_lrf_node_service, &char_md, &attr_char_value, &h_sysinfo_char);
    APP_ERROR_CHECK(err_code);

    /**     2nd     */
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.char_props.write = 0;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = uuid_type;
    ble_uuid.uuid = (PRODINFO_UUID);

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
    attr_char_value.init_len = sizeof(lrf_node_prodc_info_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(lrf_node_prodc_info_t);
    attr_char_value.p_value = NULL;

    err_code = sd_ble_gatts_characteristic_add(
        h_lrf_node_service, &char_md, &attr_char_value,&h_product_char);
    APP_ERROR_CHECK(err_code);
    
    /**** Create the read-write characteristic *****/
    /**     1st     */

    ble_gatts_attr_md_t cccd_md;
    memset(&cccd_md, 0, sizeof(cccd_md));

    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
//    char_md.char_props.notify = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
//    char_md.p_cccd_md = &cccd_md;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = uuid_type;
    ble_uuid.uuid = (DPLY_FLAG_UUID);

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
    attr_char_value.init_len = sizeof(lrf_node_flag_dply_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(lrf_node_flag_dply_t);
    attr_char_value.p_value = NULL;

    err_code = sd_ble_gatts_characteristic_add(
        h_lrf_node_service, &char_md, &attr_char_value,&h_dply_char);
    APP_ERROR_CHECK(err_code);
    
    /**     2nd     */
    memset(&char_md, 0, sizeof(char_md));

    char_md.char_props.read = 1;
    char_md.char_props.write = 1;
    char_md.p_char_user_desc = NULL;
    char_md.p_char_pf = NULL;
    char_md.p_user_desc_md = NULL;
    char_md.p_cccd_md = NULL;
    char_md.p_sccd_md = NULL;

    ble_uuid.type = uuid_type;
    ble_uuid.uuid = (DFU_FLAG_UUID);

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
    attr_char_value.init_len = sizeof(uint8_t);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(uint8_t);
    attr_char_value.p_value = NULL;

    err_code = sd_ble_gatts_characteristic_add(
        h_lrf_node_service, &char_md, &attr_char_value,&h_dfu_char);
    APP_ERROR_CHECK(err_code);

//    /**** Create the notification characteristic *****/
//   
//    
//    memset(&char_md, 0, sizeof(char_md));
//
//    char_md.char_props.read = 1;
//    char_md.char_props.write = 0;
//    char_md.p_char_user_desc = NULL;
//    char_md.p_char_pf = NULL;
//    char_md.p_user_desc_md = NULL;
//    char_md.p_cccd_md = &cccd_md;
//    char_md.p_sccd_md = NULL;
//
//    ble_uuid.type = uuid_type;
//    ble_uuid.uuid = (LRF_NODE_UUID_ALIGN_FLAG);
//
//    memset(&attr_md, 0, sizeof(attr_md));
//
//    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&attr_md.read_perm);
//    BLE_GAP_CONN_SEC_MODE_SET_NO_ACCESS(&attr_md.write_perm);
//    attr_md.vloc = BLE_GATTS_VLOC_STACK;
//    attr_md.rd_auth = 0;
//    attr_md.wr_auth = 0;
//    attr_md.vlen = 0;
//
//    memset(&attr_char_value, 0, sizeof(attr_char_value));
//
//    attr_char_value.p_uuid = &ble_uuid;
//    attr_char_value.p_attr_md = &attr_md;
//    attr_char_value.init_len = sizeof(uint8_t);
//    attr_char_value.init_offs = 0;
//    attr_char_value.max_len = sizeof(uint8_t);
//    attr_char_value.p_value = NULL;
//
//    err_code = sd_ble_gatts_characteristic_add(
//        h_lrf_node_service, &char_md, &attr_char_value, &h_align_char);
//    APP_ERROR_CHECK(err_code);
//
}

void lrf_node_ble_gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(
        &sec_mode, (const uint8_t *)ble_device_name, sizeof(ble_device_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

void lrf_node_ble_adv_init(lrf_node_ble_adv_data_t * lrf_node_ble_adv_data)
{
    h_adv = BLE_GAP_ADV_SET_HANDLE_NOT_SET;
    uint32_t err_code;
    
    ble_gap_adv_data_t adv_payload;

    adv_payload.adv_data.p_data = lrf_node_ble_adv_data->adv_data;
    adv_payload.adv_data.len = lrf_node_ble_adv_data->adv_len;

    adv_payload.scan_rsp_data.p_data = lrf_node_ble_adv_data->scan_rsp_data;
    adv_payload.scan_rsp_data.len = lrf_node_ble_adv_data->scan_rsp_len;

//    log_printf ("%s : %d %d %d %d\n", __func__, adv_payload.adv_data.p_data,
//                adv_payload.adv_data.len,
//                adv_payload.scan_rsp_data.p_data,
//                adv_payload.scan_rsp_data.len);
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

void lrf_node_ble_adv_start(void)
{
    uint32_t err_code;
    err_code = sd_ble_gap_adv_start(h_adv, BLE_CONN_CFG_TAG_DEFAULT);
    APP_ERROR_CHECK(err_code);
}

void lrf_node_ble_set_adv_data(lrf_node_ble_adv_data_t * lrf_node_ble_adv_data,
             uint8_t * device_name)
{
    memcpy (lrf_node_ble_adv_data->adv_data, generic_adv_data, sizeof(generic_adv_data));
    memcpy (lrf_node_ble_adv_data->scan_rsp_data, generic_scn_rsp, sizeof(generic_scn_rsp));
    memcpy (&lrf_node_ble_adv_data->adv_data[DEVICE_NAME_ADV_LOCATION], 
            device_name, DEVICE_NAME_LEN);
//    lrf_node_ble_adv_data->adv_data[BATTERY_TYPE_ADV_LOCATION] = 0;
    lrf_node_ble_adv_data->adv_len = sizeof(generic_adv_data);
    lrf_node_ble_adv_data->scan_rsp_len = sizeof(generic_scn_rsp);
}

void lrf_node_align_notify (lrf_node_flag_dply_t * p_align)
{
    uint32_t err_code;
    uint16_t data_size = sizeof(lrf_node_flag_dply_t); 
    err_code = sd_ble_gatts_sys_attr_set(h_conn, NULL, 0, 0);
    APP_ERROR_CHECK(err_code);
    ble_gatts_hvx_params_t hvx_params = 
    {
        .handle = h_dply_char.value_handle,
        .p_data = (uint8_t *)p_align,
        .p_len = (uint16_t *)&data_size,
        .type = BLE_GATT_HVX_NOTIFICATION,
        .offset = 0,
    };
    err_code = sd_ble_gatts_hvx (h_conn, &hvx_params);
    if(err_code != NRF_ERROR_INVALID_STATE)
    {
        APP_ERROR_CHECK(err_code);
    }
}
