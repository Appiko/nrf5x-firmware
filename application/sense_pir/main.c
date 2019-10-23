/*
 *  main.c : Application for SensePi devices
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
#include "sensepi_func.h"
#include "dev_id_fw_ver.h"
#include "sensepi_store_config.h"
#include "hal_nvmc.h"
#include "time_tracker.h"

/* ----- Defines ----- */

/**< Name of device, to be included in the advertising data. */

#define APP_DEVICE_NAME_CHAR           'S','e','n','s','e','P','i'
const uint8_t app_device_name[] = { APP_DEVICE_NAME_CHAR };

/** Complete 128 bit UUID of the SensePi service
 * 3c73dc50-07f5-480d-b066-837407fbde0a */
#define APP_UUID_COMPLETE        0x0a, 0xde, 0xfb, 0x07, 0x74, 0x83, 0x66, 0xb0, 0x0d, 0x48, 0xf5, 0x07, 0x10, 0xdc, 0x73, 0x3c

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
                           'S', 'P','0', '5', '0', '0' , '1', '9', '1', '0', '1', '5', '0', '0', '0', '0',   \
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
#define SENSE_SLOW_TICK_INTERVAL_MS      30000

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

sensepi_func_modes_t current_func_mode = SENSEPI_FUNC_NORMAL_MODE;
/** To keep track of the amount of time in the connected state */
static uint32_t conn_count;

static sensepi_store_config_t memcfg;
static sensepi_store_config_t * p_store_config;

static sensepi_ble_config_t sensepi_ble_default_config = 
{
    .battery_type = BATTERY_RECHARAGEABLE,
    
//    .not_used = {0},

    .dev_name = {'S', 'e', 'n', 's', 'e', 'P', 'i', ' ', 'T', 'r', 'i', 'a', 'l'},

    .radio_control.radio_channel = CHANNEL0,
    .radio_control.radio_oper_freq_100us = 2,
    .radio_control.radio_oper_duration_25ms = 1,

    .trigger_oper_cond_sel = {TIME_OF_DAY, TIME_OF_DAY},
    
    .current_date.dd = 15,
    .current_date.mm = 10,
    .current_date.yy = 19,
    .current_time = 12 * 3600,
//    
    .generic_settings[SETTINGS0].cam_setting.cam_trigger.mode = SINGLE_SHOT,
    .generic_settings[SETTINGS0].cam_setting.cam_trigger.mode_setting = {0},
    .generic_settings[SETTINGS0].cam_setting.cam_trigger.pre_focus_en = 0,
    .generic_settings[SETTINGS0].cam_setting.cam_trigger.radio_trig_en = 0,
    .generic_settings[SETTINGS0].cam_setting.cam_trigger.trig_pulse_duration_100ms = 3,
    
    .generic_settings[SETTINGS0].cam_setting.oper_cond.time_cond.start_time = 0,
//    .generic_settings[SETTINGS0].cam_setting.oper_cond.time_cond.end_time = 0xFFFFFFFF,
    .generic_settings[SETTINGS0].cam_setting.oper_cond.time_cond.end_time = 24*3600,
//
    .generic_settings[SETTINGS0].func_setting.detection_func.amplification = 32,
    .generic_settings[SETTINGS0].func_setting.detection_func.threshold = 800,
    .generic_settings[SETTINGS0].func_setting.detection_func.inter_trig_time = 50,
    
//    .generic_settings[SETTINGS0].func_setting.timer_duration = 5000,

    .generic_settings[SETTINGS0].trig_sel = TIMER_ONLY,   
    

};

static sensepi_func_config_t sensepi_func_default_config = 
{
    .amp_hw.ud_pin = MCP4012T_UD_PIN,
    .amp_hw.cs_pin = MCP4012T_UD_PIN,
    .amp_hw.ck_pin = SPI_SCK_PIN,
    
    .pir_hw.analog_offset_pin = PIN_TO_ANALOG_INPUT(PIR_AMP_OFFSET_PIN),
    .pir_hw.analog_signal_pin = PIN_TO_ANALOG_INPUT(PIR_AMP_SIGNAL_PIN),
    
    .cam_hw.focus_pin = JACK_FOCUS_PIN,
    .cam_hw.trigger_pin = JACK_TRIGGER_PIN,
    
    .light_sense_hw = PIN_TO_ANALOG_INPUT(LED_LIGHT_SENSE),
    
    
    .func_mode = SENSEPI_FUNC_NORMAL_MODE,
    
    
};


/* ----- Function declarations ----- */

/* ----- Function definitions ----- */
/** Function called just before reset due to WDT */
void wdt_prior_reset_callback(void){
    log_printf("WDT reset\n");
}

void prepare_init_ble_adv()
{
    log_printf("%s\n", __func__);
    uint8_t app_adv_data[] = APP_ADV_DATA;
    uint8_t app_scan_rsp_data[] = APP_SCAN_RSP_DATA;

    //Add in the firmware version
//    memcpy(&app_scan_rsp_data[23], fw_ver_get(), sizeof(fw_ver_t));

    //Add the device ID
//    memcpy(&app_scan_rsp_data[5], dev_id_get(), sizeof(dev_id_t));
    
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
static void get_sensepi_config_t(sensepi_ble_config_t *config)
{
    log_printf("%s\n", __func__);
    sensepi_func_update_settings (config);
    p_store_config = &memcfg;
    memcpy (&p_store_config->ble_config, config, sizeof(sensepi_ble_config_t));
}

/**
 * @brief The next interval handler is used for providing a periodic tick
 *  to be used by the various modules of the application
 * @param interval The interval from the last call of this function
 */
void next_interval_handler(uint32_t interval)
{
//    log_printf("in %d\n", interval);
    button_ui_add_tick(interval);
    switch(current_state)
    {
    case SENSING:
    {
        log_printf("Nxt Evt Hndlr : SENSING\n");
        sensepi_func_add_ticks (interval);        
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

            p_store_config = sensepi_store_config_get_last_config ();
            if(memcmp (p_store_config, &memcfg, sizeof(sensepi_store_config_t)))
            {
                sensepi_store_config_write (&memcfg);
            }
            sensepi_func_start();
        }
        break;
    case ADVERTISING:
        {
            sensepi_func_stop();
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

                sensepi_sysinfo sysinfo = 
                {
                    .battery_voltage = 0xB2,
                    .fw_ver_int = 20000,
                    .dev_id.day = {'1','5'},
                    .dev_id.month = {'1','0'},
                    .dev_id.year = {'1','9'},
                    .dev_id.factory_code = {'0', '0' },
                    .dev_id.prod_code = {'S','P'},
                    .dev_id.prod_rev = {'0','5'},
                    .dev_id.serial_no = {'0', '0', '0', '0'},
                };
                
//                memcpy(&sysinfo.dev_id, dev_id_get(), sizeof(dev_id_t));
//                sysinfo.battery_voltage = aa_aaa_battery_status();
//                memcpy(&sysinfo.fw_ver_int, fw_ver_get(), sizeof(fw_ver_t));
                sensepi_ble_update_sysinfo(&sysinfo);

                ///Get config from sensepi_func and send to the BLE module
                memcpy (&memcfg, sensepi_store_config_get_last_config (), sizeof(sensepi_store_config_t));
                sensepi_ble_config_t * config = &memcfg.ble_config;
                
                log_printf("Memcfg : %d %d\n", config->trigger_oper_cond_sel[0], config->trigger_oper_cond_sel[1]);
                
                log_printf ("Config : 0x%x\n", *config);
                sensepi_ble_update_config(config);
            }
            sensepi_ble_adv_start();

            led_ui_type_stop_all(LED_UI_LOOP_SEQ);
            led_ui_loop_start(LED_SEQ_ORANGE_WAVE, LED_UI_MID_PRIORITY);
        }
        break;
    case CONNECTED:
        {
            log_printf("Conn\n");
            device_tick_cfg tick_cfg =
            {
                MS_TIMER_TICKS_MS(CONN_FAST_TICK_INTERVAL_MS),
                MS_TIMER_TICKS_MS(CONN_SLOW_TICK_INTERVAL_MS),
                DEVICE_TICK_SAME
            };
            device_tick_init(&tick_cfg);

            led_ui_type_stop_all(LED_UI_LOOP_SEQ);
            led_ui_loop_start(LED_SEQ_GREEN_WAVE, LED_UI_MID_PRIORITY);
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
            case BUTTON_UI_STEP_QUICK:
                if(current_state == SENSING)
                {
                    irq_msg_push(MSG_STATE_CHANGE, (void *) ADVERTISING);
                }
                break;
            case BUTTON_UI_STEP_SHORT :
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
            case BUTTON_UI_STEP_QUICK :
                break;
            case BUTTON_UI_STEP_SHORT :
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

void genenrate_default_config ()
{
    for(uint32_t i = SETTINGS1; i < MAX_SETTINGS; i++)
    {
        sensepi_ble_default_config.generic_settings[i].cam_setting.cam_trigger.mode = SINGLE_SHOT;
//        sensepi_ble_default_config.generic_settings[i].cam_setting.cam_trigger.mode_setting = {0};
        sensepi_ble_default_config.generic_settings[i].cam_setting.cam_trigger.pre_focus_en = 0;
        sensepi_ble_default_config.generic_settings[i].cam_setting.cam_trigger.radio_trig_en = 0;
        sensepi_ble_default_config.generic_settings[i].cam_setting.cam_trigger.trig_pulse_duration_100ms = 3;

        sensepi_ble_default_config.generic_settings[i].cam_setting.oper_cond.time_cond.start_time = 0;
        sensepi_ble_default_config.generic_settings[i].cam_setting.oper_cond.time_cond.end_time = 0;
//        sensepi_ble_default_config.generic_settings[i].cam_setting.oper_cond.time_cond.start_time =
//            (12*3600) + (60*i);
//        sensepi_ble_default_config.generic_settings[i].cam_setting.oper_cond.time_cond.end_time =
//            (12*3600) + (60*i) + 60;

//        sensepi_ble_default_config.generic_settings[i].func_setting.detection_func.amplification = 60;
//        sensepi_ble_default_config.generic_settings[i].func_setting.detection_func.threshold = 250;
//        sensepi_ble_default_config.generic_settings[i].func_setting.detection_func.inter_trig_time = 10;
        
        sensepi_ble_default_config.generic_settings[i].func_setting.timer_duration = 1000 * i;

        sensepi_ble_default_config.generic_settings[i].trig_sel = TIMER_ONLY;   
    }
}


/**
 * @brief function to load previous sensepi configuration present in flash memory
 */
void load_last_config()
{
    if(sensepi_store_config_is_memory_empty())
    {
        log_printf("Memory is empty\n");
        memcpy (&memcfg.ble_config, &sensepi_ble_default_config,
                sizeof(sensepi_ble_config_t));
        memcfg.fw_ver_int = FW_VER;
        p_store_config = &memcfg;
        sensepi_store_config_write (p_store_config);

    }
    p_store_config = 
        sensepi_store_config_get_last_config ();
        
    log_printf("%s - config time : %d/%d/%d %d, %d  %d\n", __func__,
               p_store_config->ble_config.current_date.dd,
               p_store_config->ble_config.current_date.mm,
               p_store_config->ble_config.current_date.yy,
               p_store_config->ble_config.current_time, p_store_config->fw_ver_int,
               FW_VER);
    memcpy (&memcfg, p_store_config, sizeof(sensepi_store_config_t));
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
//    lfclk_init(LFCLK_SRC_RC);
    ms_timer_init(APP_IRQ_PRIORITY_LOW);
    nvm_logger_mod_init ();
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
            
    genenrate_default_config ();
    sensepi_store_config_init ();
    load_last_config ();
    sensepi_store_config_check_fw_ver ();
    sensepi_ble_init(ble_evt_handler, get_sensepi_config_t);

    p_store_config = sensepi_store_config_get_last_config ();
    sensepi_func_default_config.ble_config = &p_store_config->ble_config;
    sensepi_func_init(&sensepi_func_default_config);

    current_state = ADVERTISING; //So that a state change happens
    irq_msg_push(MSG_STATE_CHANGE, (void *)SENSING);
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
