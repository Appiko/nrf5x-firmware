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
#include "hal_clocks.h"

#include "log.h"
#include "hal_wdt.h"
#include "pir_sense.h"
#include "nrf_util.h"
#include "hal_pin_analog_input.h"
#include "hal_gpio.h"
#include "ms_timer.h"
#include "irq_msg_util.h"
#include "device_tick.h"
#include "hal_nop_delay.h"

#include "ble.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "sd_evt_handler.h"

/*      Defines         */
/** The WDT bites if not fed every 600 sec (10 min) */
#define WDT_PERIOD_MS              600000

#define PIR_SENSE_INTERVAL_MS      50
#define PIR_SENSE_THRESHOLD        600

#define FAST_TICK_INTERVAL_MS      500
#define SLOW_TICK_INTERVAL_MS      5000

/** Flag to specify if the Watchdog timer is used or not */
#define ENABLE_WDT                 1

/**< Name of device, to be included in the advertising data. */
#define DEVICE_NAME                "SenseMo"
#define DEVICE_NAME_CHAR           'S','e','n','s','e','M','o'

#define SENSEMO_SERVICE_UUID        0x3c, 0x73, 0xdc, 0x5c, 0x07, 0xf5, 0x48, 0x0d, 0xb0, 0x66, 0x83, 0x74, 0x07, 0xfb, 0xde, 0x0a

/**< Interval between advertisement packets (0.5 seconds). */
#define ADVERTISING_INTERVAL          MSEC_TO_UNITS(500, UNIT_0_625_MS)
/**< Minimum acceptable connection interval (0.1 seconds). */
#define MIN_CONN_INTERVAL          MSEC_TO_UNITS(100, UNIT_1_25_MS)
/**< Maximum acceptable connection interval (0.5 second). */
#define MAX_CONN_INTERVAL          MSEC_TO_UNITS(500, UNIT_1_25_MS)
/**< Slave latency (number of skippable conn events by the slave). */
#define SLAVE_LATENCY              0
/**< Connection supervisory timeout (4 seconds). */
#define CONN_SUP_TIMEOUT           MSEC_TO_UNITS(4000, UNIT_10_MS)

/** The data to be sent in the advertising payload. It is of the format
 *  of a sequence of {Len, type, data} */
#define ADV_DATA                   {                                        \
                                       0x02, BLE_GAP_AD_TYPE_FLAGS, BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE,    \
                                       sizeof(DEVICE_NAME), BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, DEVICE_NAME_CHAR,   \
                                       0x11, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, SENSEMO_SERVICE_UUID     \
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

/*      Global constants in flash         */
/** Stores the current state of the device */
sense_states state;

/** Handle to specify the advertising state to the soft device */
uint8_t adv_handle = BLE_GAP_ADV_SET_HANDLE_NOT_SET;

/*      Function declarations        */
void pir_handler(int32_t adc_val);

/*      Function definitions        */

/** Function called just before reset due to WDT */
void wdt_prior_reset_callback(void){
    log_printf("WDT reset\n");
}
#if 0
void jack_clear_handler(void)
{
    hal_gpio_pin_set(JACK_TRIGGER_PIN);
    hal_gpio_pin_set(JACK_FOCUS_PIN);
}

void pir_reset_handler(void)
{
    pir_sense_cfg cfg = {
            PIR_SENSE_INTERVAL_MS,
            PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN),
            PIN_TO_ANALOG_INPUT(PIR_AMP_OFFSET_PIN),
            PIR_SENSE_THRESHOLD,
            APP_IRQ_PRIORITY_HIGH,
            pir_handler
    };
    pir_sense_start(&cfg);
}
#endif
void pir_handler(int32_t adc_val)
{
    log_printf("Sensed %d\n", adc_val);
#if 0
    ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, LFCLK_TICKS_MS(50),
            jack_clear_handler);
    ms_timer_start(MS_TIMER0, MS_SINGLE_CALL, LFCLK_TICKS_MS(1000),
            pir_reset_handler);
    pir_sense_stop();
    hal_gpio_pin_clear(JACK_TRIGGER_PIN);
    hal_gpio_pin_clear(JACK_FOCUS_PIN);
#endif
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
                                          (const uint8_t *)DEVICE_NAME,
                                          strlen(DEVICE_NAME));
    APP_ERROR_CHECK(err_code);

    memset(&gap_conn_params, 0, sizeof(gap_conn_params));

    gap_conn_params.min_conn_interval = MIN_CONN_INTERVAL;
    gap_conn_params.max_conn_interval = MAX_CONN_INTERVAL;
    gap_conn_params.slave_latency     = SLAVE_LATENCY;
    gap_conn_params.conn_sup_timeout  = CONN_SUP_TIMEOUT;

    err_code = sd_ble_gap_ppcp_set(&gap_conn_params);
    APP_ERROR_CHECK(err_code);
}

uint8_t adv_buffer[] = ADV_DATA;
uint8_t scan_rsp_buffer[] = SCAN_RSP_DATA;
/**@brief Function to initializing the advertising
 */
static void advertising_init(void)
{
    uint32_t err_code;

    ble_gap_adv_data_t adv_payload;

    adv_payload.adv_data.p_data = adv_buffer;
    adv_payload.adv_data.len = sizeof(adv_buffer);

    adv_payload.scan_rsp_data.p_data = scan_rsp_buffer;
    adv_payload.scan_rsp_data.len = sizeof(scan_rsp_buffer);

    ble_gap_adv_params_t adv_params;

    //Set channel 37, 38 and 39 as advertising channels
    memset(adv_params.channel_mask, 0, 5);
    adv_params.channel_mask[4] = 0x60;

    //Set the advertising to timeout in 180s
    adv_params.duration = BLE_GAP_ADV_TIMEOUT_LIMITED_MAX;

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

    log_printf("%x %x |", own_adrs.addr_id_peer, own_adrs.addr_type);
    log_printf("%x %x %x %x %x %x\n", own_adrs.addr[0], own_adrs.addr[1], own_adrs.addr[2],
            own_adrs.addr[3], own_adrs.addr[4], own_adrs.addr[5] );

    adv_params.p_peer_addr = NULL;

    err_code = sd_ble_gap_adv_set_configure(&adv_handle, NULL, (ble_gap_adv_params_t const *) &adv_params);
    APP_ERROR_CHECK(err_code);

    err_code = sd_ble_gap_adv_set_configure(&adv_handle, (ble_gap_adv_data_t const *)&adv_payload, NULL);
    APP_ERROR_CHECK(err_code);
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

}

/**
 * @brief Initialize the audio jack pins as outputs with level 1
 *  as floating and level 0 as GND
 */
void jack_pins_init(void)
{
    //Standard low output, float on high
    hal_gpio_cfg(JACK_TRIGGER_PIN,
            GPIO_PIN_CNF_DIR_Output,
            GPIO_PIN_CNF_INPUT_Disconnect,
            GPIO_PIN_CNF_PULL_Disabled,
            GPIO_PIN_CNF_DRIVE_S0D1,
            GPIO_PIN_CNF_SENSE_Disabled);
    hal_gpio_pin_set(JACK_TRIGGER_PIN);

    //Standard low output, float on high
    hal_gpio_cfg(JACK_FOCUS_PIN,
            GPIO_PIN_CNF_DIR_Output,
            GPIO_PIN_CNF_INPUT_Disconnect,
            GPIO_PIN_CNF_PULL_Disabled,
            GPIO_PIN_CNF_DRIVE_S0D1,
            GPIO_PIN_CNF_SENSE_Disabled);
    hal_gpio_pin_set(JACK_FOCUS_PIN);
}

/**
 * @brief Initialize and blink the LEDs momentarily. To
 *  be used at the start of the program.
 */
void leds_init(void)
{
    hal_gpio_cfg_output(LED_RED, LEDS_ACTIVE_STATE);
    hal_gpio_cfg_output(LED_GREEN, !LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(200);
    hal_gpio_pin_write(LED_RED, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_GREEN, LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(200);
    hal_gpio_pin_write(LED_RED, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_GREEN, !LEDS_ACTIVE_STATE);
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    leds_init();

    /* Mandatory welcome message */
    log_init();
    log_printf("\n\nHello Sense World!\n");

    lfclk_init(LFCLK_SRC_Xtal);
#if ENABLE_WDT == 1
    hal_wdt_init(WDT_PERIOD_MS, wdt_prior_reset_callback);
    hal_wdt_start();
#endif
    ms_timer_init(APP_IRQ_PRIORITY_LOW);
    jack_pins_init();

    ble_stack_init();
    gap_params_init();
    sd_evt_handler_init(ble_evt_handler, soc_evt_handler);
    advertising_init();

    {
        uint32_t err_code;
        err_code = sd_power_dcdc_mode_set(NRF_POWER_DCDC_ENABLE);
        APP_ERROR_CHECK(err_code);

        irq_msg_callbacks cb =
            { next_interval_handler, state_change_handler };
        irq_msg_init(&cb);

        pir_sense_cfg pir_cfg =
        {

                PIR_SENSE_INTERVAL_MS,
                PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN),
                PIN_TO_ANALOG_INPUT(PIR_AMP_OFFSET_PIN),
                PIR_SENSE_THRESHOLD,
                APP_IRQ_PRIORITY_HIGH,
                pir_handler
        };
        pir_sense_start(&pir_cfg);

        device_tick_cfg tick_cfg =
        {
                LFCLK_TICKS_MS(FAST_TICK_INTERVAL_MS),
                LFCLK_TICKS_MS(SLOW_TICK_INTERVAL_MS),
                DEVICE_TICK_SLOW
        };
        device_tick_init(&tick_cfg);
    }

    advertising_start();

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
