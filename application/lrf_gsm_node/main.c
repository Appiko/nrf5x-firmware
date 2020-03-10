/*
 *  main.c : main application for LRF Node device
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

#include <string.h>

#include "boards.h"
#include "nrf.h"
#include "ble.h"
#include "nrf_nvic.h"
#include "nrf_sdm.h"
#include "button_ui.h"
#include "hal_wdt.h"
#include "device_tick.h"
#include "irq_msg_util.h"

#include "rf_comm.h"
#include "rf_spi_hw.h"
#include "log.h"
#include "nrf_util.h"
#include "hal_clocks.h"
#include "ms_timer.h"
#include "hal_gpio.h"
#include "KXTJ3.h"
#include "nvm_logger.h"

#include "aa_aaa_battery_check.h"
#include "button_ui.h"
#include "hal_nop_delay.h"
#include "lrf_node_ble.h"
#include "dev_id_fw_ver.h"
#include "random_num.h"

#include "lrf_node_rf.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#include "gps_mod.h"
#include "hal_uarte.h"


#ifndef NVM_LOGGER_LOG_USED_MAIN_0
#define NVM_LOGGER_LOG_USED_MAIN_0 0
#endif

#ifndef NVM_LOGGER_PAGES_USED_MAIN_0
#define NVM_LOGGER_PAGES_USED_MAIN_0 2
#endif

#ifndef NVM_LOGGER_START_PAGE_0
#define NVM_LOGGER_START_PAGE_0 NVM_LOG_PAGE0
#endif


#ifndef NVM_LOGGER_LOG_USED_MAIN_1
#define NVM_LOGGER_LOG_USED_MAIN_1 1
#endif

#ifndef NVM_LOGGER_PAGES_USED_MAIN_1
#define NVM_LOGGER_PAGES_USED_MAIN_1 2
#endif

#ifndef NVM_LOGGER_START_PAGE_1
#define NVM_LOGGER_START_PAGE_1 NVM_LOG_PAGE2
#endif


/** The WDT bites if not fed every 301 sec (5 min)
 * @warning All the tick intervals must be lower than this*/
#define WDT_PERIOD_MS              301000

/** Flag to specify if the Watchdog timer is used or not */
#define ENABLE_WDT                 0

#define RADIO_ID 0x5A //CC1175
//#define RADIO_ID 0x58 //CC1125
//#define RADIO_ID 0x48 //CC1120
//#define RADIO_ID 0x40 //CC1121

#define FLAG_LOG_ID NVM_LOGGER_LOG_USED_MAIN_0

#define FW_VER_LOG_ID NVM_LOGGER_LOG_USED_MAIN_1

uint8_t g_device_name[] = { DEVICE_NAME_CHAR };

/** The data to be sent in the advertising payload. It is of the format
 *  of a sequence of {Len, type, data} */
#define APP_ADV_DATA   {                                        \
                       0x02, BLE_GAP_AD_TYPE_FLAGS, BLE_GAP_ADV_FLAGS_LE_ONLY_LIMITED_DISC_MODE,    \
                       sizeof(g_device_name) + 1, BLE_GAP_AD_TYPE_COMPLETE_LOCAL_NAME, DEVICE_NAME_CHAR,   \
                       0x11, BLE_GAP_AD_TYPE_128BIT_SERVICE_UUID_COMPLETE, LRF_NODE_UUID_COMPLETE     \
                   }

#define APP_SCAN_RSP_DATA  {                                        \
                           0x02, BLE_GAP_AD_TYPE_TX_POWER_LEVEL, 0   ,     \
                           0x11, BLE_GAP_AD_TYPE_SHORT_LOCAL_NAME,  \
                           'x', 'x','x', 'x', 'x', 'x' , 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x', 'x',   \
                           0x04, BLE_GAP_AD_TYPE_MANUFACTURER_SPECIFIC_DATA, 0 ,  0 , 0 \
                       }

#define SENSE_TICKS_FAST MS_TIMER_TICKS_MS(60)
#define SENSE_TICKS_SLOW MS_TIMER_TICKS_MS(5000)

#define DEPLOY_TICKS_FAST MS_TIMER_TICKS_MS(60)
#define DEPLOY_TICKS_SLOW MS_TIMER_TICKS_MS(1000)

#define DEPLOY_TIMEOUT_TICKS MS_TIMER_TICKS_MS(10*60*1000)

#define DEFAULT_UPPER_THRESHOLD_ANGLE 30
#define DEFAULT_LOWER_THRESHOLD_ANGLE 25

#define GPS_EN_PIN 26

#define GPS_SAMPLE_FREQ (2 * 60 * 60 * 1000)

typedef enum
{
    LRF_STATE_PRE_DEPLOYED,
    LRF_STATE_DEPLOYMENT,
    LRF_STATE_SENSING,
    LRF_MAX_STATES,
}lrf_node_states_t;


void check_point ();
void next_interval_handler (uint32_t ticks);

void state_change_handler (uint32_t next_state);

static lrf_node_ble_adv_data_t g_ble_adv_data;


static lrf_node_mod_init_t g_node_mod_init = 
{
    .radio_spi.csn_pin = CSN_PIN,
    .radio_spi.sclk_pin = SCLK_PIN,
    .radio_spi.miso_pin = MISO_PIN,
    .radio_spi.mosi_pin = MOSI_PIN,
    .radio_spi.irq_priority = APP_IRQ_PRIORITY_HIGHEST,

    .radio_gpio.rf_gpio0_pin = CC_GPIO0,
    .radio_gpio.rf_gpio2_pin = CC_GPIO2,
    .radio_gpio.rf_gpio3_pin = CC_GPIO3,
    .radio_gpio.rf_reset_pin = CC_RESET_PIN,
#ifdef RF_COMM_AMPLIFIRE
    .radio_gpio.rf_lna_pin = CC_LNA_PIN,
    .radio_gpio.rf_pa_pin = CC_PA_PIN,
#endif
  
    .radio_params.bitrate = 300,
    .radio_params.center_freq = 866000,
    .radio_params.fdev = 2,
    .radio_params.tx_power = 14,
    
    
    .rf_tcxo_pin = TCXO_EN_PIN,
    
    .gps.baudrate = HAL_UARTE_BAUD_9600,
    .gps.running_irq_priority = APP_IRQ_PRIORITY_HIGH,
    .gps.timeout_irq_priority = APP_IRQ_PRIORITY_LOW,
    .gps.gps_en_pin = GPS_EN_PIN,
    
    .sleep_ms = GPS_SAMPLE_FREQ

};


static struct nvm_data_t 
{
    lrf_node_prodc_info_t production_data;
    bool gf_is_deployed;
}nvm_data;

static log_config_t gc_flag_log = 
{
    .log_id = FLAG_LOG_ID,
    .entry_size = sizeof(nvm_data),
    .start_page = NVM_LOGGER_START_PAGE_1,
    .no_of_pages = NVM_LOGGER_PAGES_USED_MAIN_1
};


static log_config_t gc_fw_ver_log = 
{
    .log_id =FW_VER_LOG_ID,
    .entry_size = sizeof(fw_ver_t),
    .start_page = NVM_LOGGER_START_PAGE_0,
    .no_of_pages = NVM_LOGGER_PAGES_USED_MAIN_0,
};


static lrf_node_states_t gs_lrf_state = LRF_STATE_PRE_DEPLOYED;

static lrf_node_flag_dply_t g_dply_align;

void check_point ()
{
    lrf_node_mod_gps_only (false);
    if(nvm_data.gf_is_deployed)
    {
        irq_msg_push (MSG_STATE_CHANGE,(void *) LRF_STATE_SENSING);
    }
    else
    {
        irq_msg_push (MSG_STATE_CHANGE,(void *) LRF_STATE_PRE_DEPLOYED);
    }
}

void next_interval_handler (uint32_t ticks)
{
//    static prev_state = LRF_STATE_PRE_DEPLOYED;
    log_printf("t%d\n",ticks);
    button_ui_add_tick (ticks);
    
    if(gs_lrf_state == LRF_STATE_SENSING)
    {
        lrf_node_mod_add_ticks (ticks);
    }
    
    if((gs_lrf_state == LRF_STATE_DEPLOYMENT))
    {
        g_dply_align.gps_lat = lrf_node_mod_get_current_loc ()->lat;
        g_dply_align.gps_lng = lrf_node_mod_get_current_loc ()->lng;
        lrf_node_ble_update_dply_alignment (&g_dply_align);
        log_printf("Lat %d, Lng %d\n", g_dply_align.gps_lat, g_dply_align.gps_lng);
    }
}

void state_change_handler (uint32_t next_state)
{
    log_printf("%s : %d\n", __func__, next_state);
    gs_lrf_state = next_state;
    switch ((lrf_node_states_t)next_state)
    {
        case LRF_STATE_PRE_DEPLOYED :
            lrf_node_mod_stop ();
            break;
        case LRF_STATE_DEPLOYMENT :
            {
                lrf_node_mod_stop ();
                lrf_node_mod_gps_only (true);
                device_tick_cfg tick_cfg = {
                    DEPLOY_TICKS_FAST,
                    DEPLOY_TICKS_SLOW,
                    DEVICE_TICK_SAME
                };
                device_tick_init(&tick_cfg);

                uint8_t is_sd_enabled;
                sd_softdevice_is_enabled(&is_sd_enabled);
                // Would be coming from the SENSING mode
                if(is_sd_enabled == 0)
                {
                    lrf_node_ble_stack_init();
                    lrf_node_ble_gap_params_init();
                    lrf_node_ble_service_init();
                    lrf_node_ble_adv_init(&g_ble_adv_data);

                    lrf_node_sysinfo sysinfo;
                    memcpy(&sysinfo.id, dev_id_get(), sizeof(dev_id_t));
                    sysinfo.battery_status = aa_aaa_battery_status();
                    memcpy(&sysinfo.fw_ver, fw_ver_get(), sizeof(fw_ver_t));
                    lrf_node_ble_update_sysinfo(&sysinfo);
                    nvm_logger_fetch_tail_data (gc_flag_log.log_id, &nvm_data, 1);
                    lrf_node_ble_update_product_info(&nvm_data.production_data);
                }
                lrf_node_ble_adv_start();

            }
            break;
        case LRF_STATE_SENSING :
            {
                lrf_node_mod_start ();
            }
            break;
        case LRF_MAX_STATES :
            check_point ();
            break;
    }
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
        break;
    case BLE_GAP_EVT_DISCONNECTED:
    {

        device_tick_cfg tick_cfg =
        {
            (SENSE_TICKS_FAST),
            (SENSE_TICKS_SLOW),
            DEVICE_TICK_SAME
        };
        device_tick_init(&tick_cfg);
        check_point ();
        break;
    }
    case BLE_GAP_EVT_ADV_SET_TERMINATED:
        check_point ();
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

static void deployment_flag_update (uint8_t dply_flag)
{
    log_printf("Update flag %d %d\n", dply_flag, nvm_data.gf_is_deployed);
    if(dply_flag != (nvm_data.gf_is_deployed ))
    {
        nvm_data.gf_is_deployed = dply_flag;
        nvm_logger_feed_data (gc_flag_log.log_id, &nvm_data);
    }
    g_dply_align.deploy_flag = nvm_data.gf_is_deployed;
}


static void production_info_update (lrf_node_prodc_info_t * new_info)
{
    nvm_data.production_data.app_id = new_info->app_id;
    nvm_data.production_data.prod_id = new_info->prod_id;
    nvm_logger_feed_data (gc_flag_log.log_id, &nvm_data);
    
    
    lrf_node_mod_update_rf_head ((lrf_node_mod_rf_head_t *) new_info);
}
static void bootloader_flag_update (uint8_t dfu_flag)
{
    if(dfu_flag == true)
    {
        NRF_POWER->GPREGRET = 0xB1;
        log_printf("Trying to do system reset..!!");
        uint8_t is_sd_enabled;
        sd_softdevice_is_enabled(&is_sd_enabled);
        if(is_sd_enabled == 0)
        {
            sd_nvic_SystemReset ();
        }
        else
        {
            NVIC_SystemReset ();
        }
        
    }
}


/** Function called just before reset due to WDT */
void wdt_prior_reset_callback(void){
    log_printf("WDT reset\n");
}

/**
 * @brief Handler for all button related events.
 * @param step The step reached by the button press
 * @param act The action of the button press i.e. if a step
 *  is crossed or the button is released
 */
void magnet_detect_handler (button_ui_steps step, button_ui_action act);
void magnet_detect_handler (button_ui_steps step, button_ui_action act)
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
                if(gs_lrf_state != LRF_STATE_DEPLOYMENT)
                {
                    irq_msg_push(MSG_STATE_CHANGE, (void *) LRF_STATE_DEPLOYMENT);
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


void tx_failed_handler (uint32_t error)
{
    log_printf("%s : %d\n",__func__, error);
    if(gs_lrf_state != LRF_STATE_SENSING)
    {
        rf_comm_flush ();
    }
}

void tx_done_handler (uint32_t error)
{
    log_printf("%s : %d\n", __func__, error);
    if(gs_lrf_state != LRF_STATE_SENSING)
    {
        rf_comm_flush ();
    }
}



void firmware_check ()
{
    if(nvm_logger_is_log_empty (gc_fw_ver_log.log_id))
    {
        nvm_logger_feed_data (gc_fw_ver_log.log_id, fw_ver_get ());
    }
    fw_ver_t l_prev_ver;
    nvm_logger_fetch_tail_data (gc_fw_ver_log.log_id, &l_prev_ver, 1);
    if(memcmp (&l_prev_ver, fw_ver_get (), sizeof(fw_ver_t)) != 0)
    {
        nvm_logger_feed_data (gc_fw_ver_log.log_id, fw_ver_get ());
        nvm_logger_release_log (gc_flag_log.log_id);
        nvm_logger_log_init (&gc_flag_log);
    }
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

int main ()
{
    log_init ();
    boot_pwr_config ();
    log_printf("Hello world from LRF Node\n");    
    lfclk_init (LFCLK_SRC_Xtal);
    ms_timer_init (APP_IRQ_PRIORITY_LOW);
       
#if ENABLE_WDT == 1
    hal_wdt_init(WDT_PERIOD_MS, wdt_prior_reset_callback);
    hal_wdt_start();
#endif
 
    {
        irq_msg_callbacks cb =
            { next_interval_handler, state_change_handler };
        irq_msg_init(&cb);
    }

    device_tick_cfg tick_cfg =
    {
        (SENSE_TICKS_FAST),
        (SENSE_TICKS_SLOW),
        DEVICE_TICK_SAME
    };
    device_tick_init(&tick_cfg);

    button_ui_init(HALL_EFFECT_PIN, APP_IRQ_PRIORITY_LOW,
            magnet_detect_handler);

    nvm_logger_mod_init ();
    log_printf("Data size %d\n", gc_flag_log.entry_size);
    nvm_logger_log_init (&gc_flag_log);
    
    nvm_logger_log_init (&gc_fw_ver_log);
 
    lrf_node_ble_init(ble_evt_handler, deployment_flag_update,
                      bootloader_flag_update, production_info_update);
    lrf_node_ble_set_adv_data (&g_ble_adv_data, g_device_name);
    
    firmware_check ();
    
    if (nvm_logger_is_log_empty (gc_flag_log.log_id) == false)
    {
        nvm_logger_fetch_tail_data (gc_flag_log.log_id, &nvm_data, 1);
        log_printf("Is deployed : %d\n", nvm_data.gf_is_deployed);
        g_dply_align.deploy_flag = nvm_data.gf_is_deployed;
    }
    else
    {
        nvm_data.production_data.app_id = 0x02;
        nvm_data.production_data.prod_id = 0x00F3;
        nvm_data.gf_is_deployed = 1;
        nvm_logger_feed_data (gc_flag_log.log_id, &nvm_data);
    }

    g_node_mod_init.radio_header.app_id = nvm_data.production_data.app_id;
    g_node_mod_init.radio_header.prod_id = nvm_data.production_data.prod_id;
    
    lrf_node_mod_init (&g_node_mod_init);
    
    check_point ();
    while(1)
    {
#if ENABLE_WDT == 1
        hal_wdt_feed ();
#endif
        gps_mod_process ();
        device_tick_process ();
        irq_msg_process ();
        slumber ();
    }
}