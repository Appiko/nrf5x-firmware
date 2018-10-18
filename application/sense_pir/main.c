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

#include "led_sense.h"
#include "evt_sd_handler.h"
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
#include "aa_aaa_battery_check.h"
#include "button_ui.h"
#include "nrf_nvic.h"
#include "ble.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "out_pattern_gen.h"
#include "led_ui.h"
#include "sensepi_ble.h"
#include "sensepi_cam_trigger.h"
#include "dev_id_fw_ver.h"
#include "sensepi_store_config.h"
#include "hal_nvmc.h"

/* ----- Defines ----- */

/**< Name of device, to be included in the advertising data. */

#define APP_DEVICE_NAME_CHAR           'S','e','n','s','e','P','i'
const uint8_t app_device_name[] = { APP_DEVICE_NAME_CHAR };

/** Complete 128 bit UUID of the SensePi service
 * 3c73dc50-07f5-480d-b066-837407fbde0a */
#define APP_UUID_COMPLETE        0x0a, 0xde, 0xfb, 0x07, 0x74, 0x83, 0x66, 0xb0, 0x0d, 0x48, 0xf5, 0x07, 0x50, 0xdc, 0x73, 0x3c

/** The data to be sent in the advertising payload. It is of the format
 *  of a sequence of {Len, type, data} */
#define APP_ADV_DATA   {                                        \
                       0x02, BLE_GAP_AD_TYPE_FLAGS, BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE,    \
                       sizeof(app_device_name) + 1, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, APP_DEVICE_NAME_CHAR,   \
                       0x11, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, APP_UUID_COMPLETE     \
                   }

/** The data to be sent in the scan response payload. It is of the format
 *  of a sequence of {Len, type, data} */
#define APP_SCAN_RSP_DATA  {                                        \
                           0x02, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, 0   ,     \
                           0x11, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,  \
                           'x', 'x','x', 'x', 'x', 'x' , 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',   \
                           0x04, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0 ,  0 , 0 \
                       }

/** The WDT bites if not fed every 301 sec (5 min)
 * @warning All the tick intervals must be lower than this*/
#define WDT_PERIOD_MS              301000

/** Flag to specify if the Watchdog timer is used or not */
#define ENABLE_WDT                 1

/** The interval at which the PIR sensor's signal is sampled.
 * 20 Hz interval is chosen so that as per Nyquist's criterion
 * signals up to 10 Hz can be sensed. */
#define PIR_SENSE_INTERVAL_MS      50
/** The static threshold above and below which PIR detects motion */
#define PIR_SENSE_THRESHOLD        600

/** The fast tick interval in ms in the Sense mode */
#define SENSE_FAST_TICK_INTERVAL_MS      60
/** The slow tick interval in ms in the Sense mode */
#define SENSE_SLOW_TICK_INTERVAL_MS      300000

/** The fast tick interval in ms in the Advertising mode */
#define ADV_FAST_TICK_INTERVAL_MS  60
/** The slow tick interval in ms in the Advertising mode */
#define ADV_SLOW_TICK_INTERVAL_MS  1100

/** The fast tick interval in ms in the Connected mode */
#define CONN_FAST_TICK_INTERVAL_MS  60
/** The slow tick interval in ms in the Connected mode */
#define CONN_SLOW_TICK_INTERVAL_MS  1100

/** The time in ms (min*sec*ms) to timeout of the Connected mode*/
#define CONN_TIMEOUT_MS             (10*60*1000)

/** Defines the states possible in the SensePi device */
typedef enum
{
     //!< Use PIR sensor to sense motion based on the set configuration
    SENSING,
    ADVERTISING,//!< BLE advertising to get connected to an app
    CONNECTED   //!< BLE connection established with an app
}sense_states;

/* ----- Global constants in flash ----- */

/* ----- Global variables in RAM ----- */
/** Stores the current state of the device */
sense_states current_state;

/** To keep track of the amount of time in the connected state */
static uint32_t conn_count;

static sensepi_config_t sensepi_ble_default_config = {
    .pir_conf.oper_time.day_or_night = 1,
    .pir_conf.oper_time.threshold = 0b0000000,
    .pir_conf.amplification = 20,
    .pir_conf.threshold = 175,
    .pir_conf.mode = 0x00000000,
    .pir_conf.intr_trig_timer = 50,
    
    .timer_conf.oper_time.day_or_night = 1,
    .timer_conf.oper_time.threshold = 0b0000000,
    .timer_conf.mode = 0x00000000,
    .timer_conf.timer_interval = 50,
    
    .trig_conf = PIR_ONLY,
};

static uint32_t out_pin_array[] = {JACK_FOCUS_PIN, JACK_TRIGGER_PIN};

static sensepi_cam_trigger_init_config_t sensepi_cam_trigger_default_config = 
{
    .config_sensepi = &sensepi_ble_default_config,
    .led_sense_analog_in_pin = PIN_TO_ANALOG_INPUT(LED_LIGHT_SENSE),
    .led_sense_off_val = !(LEDS_ACTIVE_STATE),
    .led_sense_out_pin = LED_GREEN,
    .pir_sense_offset_input = PIN_TO_ANALOG_INPUT(PIR_AMP_OFFSET_PIN),
    .pir_sense_signal_input = PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN),
    .amp_cs_pin = MCP4012T_CS_PIN,
    .amp_ud_pin= MCP4012T_UD_PIN,
    .amp_spi_sck_pin = SPI_SCK_PIN,
    .signal_out_pin_array = out_pin_array,
    .signal_pin_num = ARRAY_SIZE(out_pin_array),
};


/* ----- Function declarations ----- */

/* ----- Function definitions ----- */
/** Function called just before reset due to WDT */
void wdt_prior_reset_callback(void){
    log_printf("WDT reset\n");
}

void prepare_init_ble_adv()
{
    uint8_t app_adv_data[] = APP_ADV_DATA;
    uint8_t app_scan_rsp_data[] = APP_SCAN_RSP_DATA;

    //Add in the firmware version
    memcpy(&app_scan_rsp_data[23], fw_ver_get(), sizeof(fw_ver_t));

    //Add the device ID
    memcpy(&app_scan_rsp_data[5], dev_id_get(), sizeof(dev_id_t));
    
    sensepi_ble_adv_data_t app_adv_data_struct =
    {
        .adv_data = app_adv_data,
        .scan_rsp_data = app_scan_rsp_data,
        .adv_len = ARRAY_SIZE(app_adv_data),
        .scan_rsp_len = ARRAY_SIZE(app_scan_rsp_data)
    };

    sensepi_ble_adv_init(&app_adv_data_struct);
}

/**
 * @brief Handler which will address all BLE related events
 *  generated by the SoftDevice for SensePi application related code.
 * @param evt The pointer to the buffer containing all the data
 *  related to the event
 */
static void ble_evt_handler(ble_evt_t * evt)
{
    log_printf("ble evt %x\n", evt->header.evt_id);
    switch (evt->header.evt_id)
    {
    case BLE_GAP_EVT_CONNECTED:
        irq_msg_push(MSG_STATE_CHANGE, (void *) CONNECTED);
        break;
    case BLE_GAP_EVT_DISCONNECTED:
        irq_msg_push(MSG_STATE_CHANGE, (void *) SENSING);
        break;
    case BLE_GAP_EVT_ADV_SET_TERMINATED:
        irq_msg_push(MSG_STATE_CHANGE, (void *) SENSING);
        break;
    case BLE_GAP_EVT_CONN_PARAM_UPDATE:
        log_printf("sup time %d s, max intvl %d ms, min intvl %d ms, slave lat %d\n",
            evt->evt.gap_evt.params.conn_param_update.conn_params.conn_sup_timeout/100,
            (5*evt->evt.gap_evt.params.conn_param_update.conn_params.max_conn_interval)/4,
            (5*evt->evt.gap_evt.params.conn_param_update.conn_params.min_conn_interval)/4,
            evt->evt.gap_evt.params.conn_param_update.conn_params.slave_latency);
        break;
    }
}

/**
 * Handler to get the configuration of SensePi from the mobile app
 * @param config Pointer to the configuration received
 */
static void get_sensepi_config_t(sensepi_config_t *config)
{
    log_printf("Trig mode %d, PIR ope time %08x, PIR mode %08x, PIR amp %d, PIR thres %d, \
             PIR int trig time %04d, Timer oper %x, Timer mode %x, timer interval %04d \n",
            config->trig_conf, config->pir_conf.oper_time, config->pir_conf.mode,
            config->pir_conf.amplification, config->pir_conf.threshold,
            config->pir_conf.intr_trig_timer,
            config->timer_conf.oper_time, config->timer_conf.mode, config->timer_conf.timer_interval);
    sensepi_cam_trigger_update(config);
}

/**
 * @brief The next interval handler is used for providing a periodic tick
 *  to be used by the various modules of the application
 * @param interval The interval from the last call of this function
 */
void next_interval_handler(uint32_t interval)
{
    log_printf("in %d\n", interval);
    button_ui_add_tick(interval);
    switch(current_state)
    {
    case SENSING:
    {
        log_printf("Nxt Evt Hndlr : SENSING\n");
        sensepi_cam_trigger_add_tick(interval);        
    }
        break;
    case ADVERTISING:
        break;
    case CONNECTED:
    {
        conn_count += interval;
        if(conn_count > MS_TIMER_TICKS_MS(CONN_TIMEOUT_MS))
        {
            sensepi_ble_disconn();
        }
    }
        break;
    }
}

/**
 * @brief The handler that is called whenever the application transitions
 *  to a new state.
 * @param new_state The state to which the application has transitioned to.
 */
void state_change_handler(uint32_t new_state)
{
    log_printf("State change %d\n", new_state);
    if(new_state == current_state)
    {
        log_printf("new state same as current state\n");
        return;
    }
    current_state = (sense_states) new_state;

    switch(current_state)
    {
    case SENSING:
        {
            sd_softdevice_disable();
            log_printf("State Change : SENSING\n");
            device_tick_cfg tick_cfg =
            {
                MS_TIMER_TICKS_MS(SENSE_FAST_TICK_INTERVAL_MS),
                MS_TIMER_TICKS_MS(SENSE_SLOW_TICK_INTERVAL_MS),
                DEVICE_TICK_SAME
            };
            device_tick_init(&tick_cfg);

            led_ui_type_stop_all(LED_UI_LOOP_SEQ);

            sensepi_cam_trigger_start();
        }
        break;
    case ADVERTISING:
        {
            sensepi_cam_trigger_stop();
            conn_count = 0;

            device_tick_cfg tick_cfg =
            {
                MS_TIMER_TICKS_MS(ADV_FAST_TICK_INTERVAL_MS),
                MS_TIMER_TICKS_MS(ADV_SLOW_TICK_INTERVAL_MS),
                DEVICE_TICK_SAME
            };
            device_tick_init(&tick_cfg);

            uint8_t is_sd_enabled;
            sd_softdevice_is_enabled(&is_sd_enabled);
            // Would be coming from the SENSING mode
            if(is_sd_enabled == 0)
            {
                sensepi_ble_stack_init();
                sensepi_ble_gap_params_init();
                sensepi_ble_service_init();
                prepare_init_ble_adv();

                sensepi_sysinfo sysinfo;
                memcpy(&sysinfo.id, dev_id_get(), sizeof(dev_id_t));
                sysinfo.battery_status = aa_aaa_battery_status();
                memcpy(&sysinfo.fw_ver, fw_ver_get(), sizeof(fw_ver_t));
                sensepi_ble_update_sysinfo(&sysinfo);

                ///Get config from sensepi_cam_trigger and send to the BLE module
                sensepi_config_t * config = sensepi_cam_trigger_get_sensepi_config();
                sensepi_ble_update_config(config);
            }
            sensepi_ble_adv_start();

            led_ui_type_stop_all(LED_UI_LOOP_SEQ);
            led_ui_loop_start(LED_SEQ_ORANGE_SEQ, LED_UI_MID_PRIORITY);
        }
        break;
    case CONNECTED:
        {
            device_tick_cfg tick_cfg =
            {
                MS_TIMER_TICKS_MS(CONN_FAST_TICK_INTERVAL_MS),
                MS_TIMER_TICKS_MS(CONN_SLOW_TICK_INTERVAL_MS),
                DEVICE_TICK_SAME
            };
            device_tick_init(&tick_cfg);

            led_ui_type_stop_all(LED_UI_LOOP_SEQ);
            led_ui_loop_start(LED_SEQ_GREEN_SEQ, LED_UI_MID_PRIORITY);
            break;
        }
    }
}

/**
 * @brief Handler for all button related events.
 * @param step The step reached by the button press
 * @param act The action of the button press i.e. if a step
 *  is crossed or the button is released
 */
void button_handler(button_ui_steps step, button_ui_action act)
{
    if(act == BUTTON_UI_ACT_CROSS)
    {
        switch(step)
        {
        case BUTTON_UI_STEP_WAKE:
            log_printf("fast\n");
            device_tick_switch_mode(DEVICE_TICK_FAST);
            button_ui_config_wake(false);
            break;
        case BUTTON_UI_STEP_PRESS:
            if(current_state == SENSING)
            {
                irq_msg_push(MSG_STATE_CHANGE, (void *) ADVERTISING);
            }
            break;
        case BUTTON_UI_STEP_LONG:
            {
                NRF_POWER->GPREGRET = 0xB1;
                log_printf("Trying to do system reset..!!");
                uint8_t is_sd_enabled;
                sd_softdevice_is_enabled(&is_sd_enabled);
                if(is_sd_enabled == 0)
                {
                    sd_nvic_SystemReset();
                }
                else
                {
                    NVIC_SystemReset ();
                }
            }
            break;
        }
    }
    else    //BUTTON_UI_ACT_RELEASE
    {
        device_tick_switch_mode(DEVICE_TICK_SLOW);
        log_printf("slow\n");
        button_ui_config_wake(true);
        switch(step)
        {
        case BUTTON_UI_STEP_WAKE:
            break;
        case BUTTON_UI_STEP_PRESS:
            break;
        case BUTTON_UI_STEP_LONG:
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

/**
 * @brief Prints the reason for the last reset, enables the internal
 *  DC-DC converter if the board supports it and puts the nRF SoC to
 *  the low power mode.
 */
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
 * @brief function to load previous sensepi configuration present in flash memory
 */
void load_last_config()
{
    if(sensepi_store_config_is_memory_empty())
    {
        sensepi_store_config_write (&sensepi_ble_default_config);
    }
    sensepi_cam_trigger_update (sensepi_store_config_get_last_config ());
}

/**
 * Different calls to sleep depending on the status of Softdevice
 */
void slumber(void)
{
    uint8_t is_sd_enabled;
    sd_softdevice_is_enabled(&is_sd_enabled);
    // Would in the SENSING mode
    if(is_sd_enabled == 0)
    {
        __WFI();
    }
    else
    {
        sd_app_evt_wait();
    }
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

    button_ui_init(BUTTON_PIN, APP_IRQ_PRIORITY_LOW,
            button_handler);
    led_sense_init(LED_GREEN,
            PIN_TO_ANALOG_INPUT(LED_LIGHT_SENSE), !LEDS_ACTIVE_STATE);

    {
        irq_msg_callbacks cb =
            { next_interval_handler, state_change_handler };
        irq_msg_init(&cb);
    }
    sensepi_cam_trigger_init(&sensepi_cam_trigger_default_config);

    current_state = ADVERTISING; //So that a state change happens
    irq_msg_push(MSG_STATE_CHANGE, (void *)SENSING);
    sensepi_ble_init(ble_evt_handler, get_sensepi_config_t);
    sensepi_store_config_check_fw_ver ();
    load_last_config ();
    while (true)
    {
#if ENABLE_WDT == 1
        //Since the application demands that CPU wakes up
        hal_wdt_feed();
#endif
        device_tick_process();
        irq_msg_process();
        slumber();
    }
}

/** @} */
/** @} */
