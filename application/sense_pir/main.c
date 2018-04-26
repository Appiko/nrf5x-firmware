/*
 *  main.c
 *
 *  Created on: 30-Jan-2018
 *
 *  Copyright (c) 2018, Appiko
 *  Copyright (c) 2013, Nordic Semiconductor ASA
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
 * @addtogroup group_appln
 * @{
 *
 * @defgroup sense_appln The code for the PIR based Sense units.
 * @brief The PIR sense application's main file that makes it operate.
 *
 * @{
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>
#include "nrf.h"
#include "boards.h"

#include "log.h"
#include "nrf_util.h"
#include "hal_gpio.h"
#include "ms_timer.h"
#include "hal_nop_delay.h"
#include "hal_wdt.h"
#include "irq_msg_util.h"
#include "device_tick.h"
#include "pir_sense.h"
#include "hal_pin_analog_input.h"

#include "nrf_nvic.h"
#include "ble.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "sd_evt_handler.h"

/*      Defines         */
/** The WDT bites if not fed every 301 sec (5 min) */
#define WDT_PERIOD_MS              301000

/** Flag to specify if the Watchdog timer is used or not */
#define ENABLE_WDT                 1

#define PIR_SENSE_INTERVAL_MS      50
#define PIR_SENSE_THRESHOLD        600

#define SENSE_FAST_TICK_INTERVAL_MS      500
#define SENSE_SLOW_TICK_INTERVAL_MS      300000

#define ADV_FAST_TICK_INTERVAL_MS  50
#define ADV_SLOW_TICK_INTERVAL_MS  1100

#define CONN_FAST_TICK_INTERVAL_MS  50
#define CONN_SLOW_TICK_INTERVAL_MS  1100

/**< Name of device, to be included in the advertising data. */
#define DEVICE_NAME_CHAR           'S','e','n','s','e','P','i'
const uint8_t device_name[] = { DEVICE_NAME_CHAR };

#define SENSEPI_UUID_COMPLETE        0x0a, 0xde, 0xfb, 0x07, 0x74, 0x83, 0x66, 0xb0, 0x0d, 0x48, 0xf5, 0x07, 0x50, 0xdc, 0x73, 0x3c

#define SENSEPI_UUID_SERVICE         0xdc50

#define SENSEPI_UUID_SYSINFO         0xdc51
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
#define ADV_DATA                   {                                        \
                                       0x02, BLE_GAP_AD_TYPE_FLAGS, BLE_GAP_ADV_FLAGS_LE_ONLY_GENERAL_DISC_MODE,    \
                                       sizeof(device_name) + 1, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, DEVICE_NAME_CHAR,   \
                                       0x11, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, SENSEPI_UUID_COMPLETE     \
                                   }


/** The data to be sent in the scan response payload. It is of the format
 *  of a sequence of {Len, type, data} */
#define SCAN_RSP_DATA              {                                        \
                                       0x02, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, 0        \
                                   }
typedef enum
{
    SENSING,
    ADVERTISING,
    CONNECTED
}sense_states;

typedef enum
{
    DAY_ONLY,
    NIGHT_ONLY,
    DAY_AND_NIGHT
}oper_time_t;

typedef struct
{
    oper_time_t oper_time;
    uint32_t mode;
    uint8_t sensitivity;
    uint16_t inter_trig_time;
    bool pre_focus;
    uint8_t cam_comp;
    uint8_t cam_model;
}sensepi_config;

/*      Global constants in flash         */
/** Stores the current state of the device */
sense_states state;

/** Handle to specify the advertising state to the soft device */
uint8_t adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

/*      Function declarations        */
static void advertising_start(void);

/*      Function definitions        */
/** Function called just before reset due to WDT */
void wdt_prior_reset_callback(void){
    log_printf("WDT reset\n");
}

void SWI1_IRQHandler(void)
{
//    log_printf("rdo\n");
}

void pir_handler(int32_t adc_val)
{
    log_printf("Sensed %d\n", adc_val);
}

void sensepi_service_init()
{
    uint32_t   err_code;
    ble_uuid_t ble_uuid;
    uint8_t uuid_type;
    uint16_t service_handle;
    ble_gatts_char_handles_t sysinfo_char_handle;
    ble_gatts_char_handles_t config_char_handle;

    // Add service.
    ble_uuid128_t base_uuid = {{SENSEPI_UUID_COMPLETE}};
    err_code = sd_ble_uuid_vs_add(&base_uuid, &uuid_type);
    APP_ERROR_CHECK(err_code);

    ble_uuid.type = uuid_type;
    ble_uuid.uuid = SENSEPI_UUID_SERVICE;

    err_code = sd_ble_gatts_service_add(BLE_GATTS_SRVC_TYPE_PRIMARY,
            &ble_uuid, &service_handle);
    APP_ERROR_CHECK(err_code);

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
    attr_char_value.init_len = sizeof(sensepi_config);
    attr_char_value.init_offs = 0;
    attr_char_value.max_len = sizeof(sensepi_config);
    attr_char_value.p_value = NULL;

    err_code = sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value,
            &sysinfo_char_handle);
    APP_ERROR_CHECK(err_code);

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

    err_code = sd_ble_gatts_characteristic_add(service_handle, &char_md, &attr_char_value,
            &config_char_handle);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief Handler which will address all the SoC related events
 *  generated by the SoftdDevice.
 * @param evt The pointer to the buffer containing all the data
 *  related to the event
 */
static void ble_evt_handler(ble_evt_t * evt)
{
    log_printf("ble evt %x\n", evt->header.evt_id);
//    uint32_t err_code;
    switch(evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        irq_msg_push(MSG_STATE_CHANGE, (void *)CONNECTED);
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        irq_msg_push(MSG_STATE_CHANGE, (void *)ADVERTISING);
        break;
    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        log_printf("sup time %d ms, max intvl %d ms, min intvl %d ms, slave lat %d\n",
                10*evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout,
                (5*evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval)/4,
                (5*evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval)/4,
                evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency);
        break;
    }
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

/**
 * @brief Function for initializing the BLE stack by enabling the
 *  SoftDevice and the BLE event interrupt
 * */
void ble_stack_init(void)
{
    uint32_t err_code;
    const nrf_clock_lf_cfg_t cfg = BOARD_LFCLKSRC_STRUCT;

    err_code = sd_softdevice_enable(&cfg, app_error_fault_handler);
    APP_ERROR_CHECK(err_code);

    uint32_t app_ram_start = 0x20001C00;
    log_printf("Init %x", app_ram_start);
    err_code = sd_ble_enable(&app_ram_start);
    log_printf(" RAM needed %x\n", app_ram_start);
    APP_ERROR_CHECK(err_code);

    sd_evt_handler_init(ble_evt_handler, soc_evt_handler);

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
}


/**
 * @brief Generic Access Profile initialization. The device name, appearance,
 *  and the preferred connection parameters are setup.
 */
static void gap_params_init(void)
{
    uint32_t                err_code;
    ble_gap_conn_params_t   gap_conn_params;
    ble_gap_conn_sec_mode_t sec_mode;

    BLE_GAP_CONN_SEC_MODE_SET_OPEN(&sec_mode);

    err_code = sd_ble_gap_device_name_set(&sec_mode,
                                          (const uint8_t *)device_name,
                                          sizeof(device_name));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

/**@brief Function to initializing the advertising
 */
static void advertising_init(void)
{
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

    //TODO change to timeout and also in the advertising
    //Set the advertising to timeout in 180s
    adv_params.duration = 0; //BLE_GAP_ADV_TIMEOUT_LIMITED_MAX;

    //Any device can scan request and connect
    adv_params.filter_policy = BLE_GAP_ADV_FP_ANY;

    //Period between sending advertising packets
    adv_params.interval = ADVERTISING_INTERVAL;

    //Use 1Mbps physical layer to be backward compatible
    adv_params.primary_phy = BLE_GAP_PHY_1MBPS;

    //The advertisement would be unidirected connectable and scannable
    adv_params.properties.type = BLE_GAP_ADV_TYPE_CONNECTABLE_SCANNABLE_UNDIRECTED;

    ble_gap_addr_t own_adrs;
    err_code = sd_ble_gap_addr_get(&own_adrs);
    APP_ERROR_CHECK(err_code);

    adv_params.p_peer_addr = NULL;
    adv_params.scan_req_notification = 0;

    err_code = sd_ble_gap_adv_set_configure(&adv_handle, (ble_gap_adv_data_t const *)&adv_payload, (ble_gap_adv_params_t const *) &adv_params);
    APP_ERROR_CHECK(err_code);

    log_printf("%d |", adv_handle);
    log_printf("%x %x %x %x %x %x\n", own_adrs.addr[0], own_adrs.addr[1], own_adrs.addr[2],
            own_adrs.addr[3], own_adrs.addr[4], own_adrs.addr[5] );
}

/**
 * @brief Function to start advertising.
 */
static void advertising_start(void)
{
    uint32_t             err_code;
    err_code = sd_ble_gap_adv_start(adv_handle, BLE_CONN_CFG_TAG_DEFAULT);
    APP_ERROR_CHECK(err_code);
}

/**
 * @brief The next interval handler is used for providing a periodic tick
 *  to be used by the various modules of the application
 * @param interval The interval from the last call of this function
 */
void next_interval_handler(uint32_t interval)
{
    log_printf("interval %d\n", interval);
}

/**
 * @brief The handler that is called whenever the application transitions
 *  to a new state.
 * @param new_state The state to which the application has transitioned to.
 */
void state_change_handler(uint32_t new_state)
{
    log_printf("State change %d\n", new_state);
    switch((sense_states) new_state)
    {
    case SENSING:
        sd_softdevice_disable();

        {
            device_tick_cfg tick_cfg =
            {
                LFCLK_TICKS_MS(SENSE_FAST_TICK_INTERVAL_MS),
                LFCLK_TICKS_MS(SENSE_SLOW_TICK_INTERVAL_MS),
                DEVICE_TICK_SLOW
            };
            device_tick_init(&tick_cfg);

            pir_sense_cfg pir_cfg =
            {
                PIR_SENSE_INTERVAL_MS, PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN),
                PIN_TO_ANALOG_INPUT(PIR_AMP_OFFSET_PIN),
                PIR_SENSE_THRESHOLD, APP_IRQ_PRIORITY_HIGH, pir_handler
            };
            pir_sense_start(&pir_cfg);
        }
        break;
    case ADVERTISING:
        {
            device_tick_cfg tick_cfg =
            {
                LFCLK_TICKS_MS(ADV_FAST_TICK_INTERVAL_MS),
                LFCLK_TICKS_MS(ADV_SLOW_TICK_INTERVAL_MS),
                DEVICE_TICK_SLOW
            };
            device_tick_init(&tick_cfg);

            uint8_t is_sd_enabled;
            sd_softdevice_is_enabled(&is_sd_enabled);
            // Would be coming from the SENSEING mode
            if(is_sd_enabled == 0)
            {
                pir_sense_stop();
                ble_stack_init();
                gap_params_init();
                sd_evt_handler_init(ble_evt_handler, soc_evt_handler);
                sensepi_service_init();
                advertising_init();
            }
            advertising_start();
        }
        break;
    case CONNECTED:
        {
            device_tick_cfg tick_cfg =
            {
                LFCLK_TICKS_MS(CONN_FAST_TICK_INTERVAL_MS),
                LFCLK_TICKS_MS(CONN_SLOW_TICK_INTERVAL_MS),
                DEVICE_TICK_SLOW
            };
            device_tick_init(&tick_cfg);
            break;
        }
    }
}

/**
 * @brief Initialize and blink the LEDs momentarily. To
 *  be used at the start of the program.
 */
void leds_init(void)
{
    hal_gpio_cfg_output(LED_RED, LEDS_ACTIVE_STATE);
    hal_gpio_cfg_output(LED_GREEN, !LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(600);
    hal_gpio_pin_write(LED_RED, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_GREEN, LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(600);
    hal_gpio_pin_write(LED_RED, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_GREEN, !LEDS_ACTIVE_STATE);
}

void boot_pwr_config(void)
{
    log_printf("Reset because of ");
    if(NRF_POWER->RESETREAS == 0)
    {
        log_printf("power on or brownout, ");
    }
    if(NRF_POWER->RESETREAS & POWER_RESETREAS_DIF_Msk)
    {
        log_printf("entering into debug interface from Sys OFF, ");
    }
    if(NRF_POWER->RESETREAS & POWER_RESETREAS_DOG_Msk)
    {
        log_printf("watchdog bite, ");
    }
    if(NRF_POWER->RESETREAS & POWER_RESETREAS_LOCKUP_Msk)
    {
        log_printf("CPU lockup, ");
    }
    if(NRF_POWER->RESETREAS & POWER_RESETREAS_OFF_Msk)
    {
        log_printf("wake up from SYS OFF by GPIO, ");
    }
    if(NRF_POWER->RESETREAS & POWER_RESETREAS_RESETPIN_Msk)
    {
        log_printf("pin reset, ");
    }
    if(NRF_POWER->RESETREAS & POWER_RESETREAS_SREQ_Msk)
    {
        log_printf("software reset, ");
    }
    log_printf("\n");

    //Clear the reset reason
    NRF_POWER->RESETREAS = (POWER_RESETREAS_DIF_Msk |
                            POWER_RESETREAS_DOG_Msk |
                            POWER_RESETREAS_LOCKUP_Msk |
                            POWER_RESETREAS_OFF_Msk |
                            POWER_RESETREAS_RESETPIN_Msk |
                            POWER_RESETREAS_SREQ_Msk);

    //Enable the DCDC converter if the board supports it
#if DC_DC_CIRCUITRY == true  //Defined in the board header file
    NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Enabled << POWER_DCDCEN_DCDCEN_Pos;
#endif
    NRF_POWER->TASKS_LOWPWR = 1;
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    leds_init();

    /* Mandatory welcome message */
    log_init();
    log_printf("\n\nHello SensePi World!\n");
    boot_pwr_config();

    lfclk_init(LFCLK_SRC_Xtal);
    ms_timer_init(APP_IRQ_PRIORITY_LOW);
#if ENABLE_WDT == 1
    hal_wdt_init(WDT_PERIOD_MS, wdt_prior_reset_callback);
    hal_wdt_start();
#endif

#if 0
    ble_stack_init();
    gap_params_init();
    sd_evt_handler_init(ble_evt_handler, soc_evt_handler);
    advertising_init();
    sensepi_service_init();
    advertising_start();
#endif

    {
        irq_msg_callbacks cb =
            { next_interval_handler, state_change_handler };
        irq_msg_init(&cb);
    }

    irq_msg_push(MSG_STATE_CHANGE, (void *)ADVERTISING);

    while (true)
    {
#if ENABLE_WDT == 1
        //Since the application demands that CPU wakes up
        hal_wdt_feed();
#endif
        device_tick_process();
        irq_msg_process();
        sd_app_evt_wait();
    }
}

/** @} */
/** @} */
