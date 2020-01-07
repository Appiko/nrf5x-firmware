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

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif


#ifndef NVM_LOGGER_LOG_USED_MAIN
#define NVM_LOGGER_LOG_USED_MAIN 0
#endif

#ifndef NVM_LOGGER_PAGES_USED_MAIN
#define NVM_LOGGER_PAGES_USED_MAIN 2
#endif

#ifndef NVM_LOGGER_START_PAGE
#define NVM_LOGGER_START_PAGE NVM_LOG_PAGE0
#endif

#ifndef MS_TIMER_USED_MAIN
#define MS_TIMER_USED_MAIN 1
#endif


#define MS_TIMER_ID MS_TIMER_USED_MAIN

/** The WDT bites if not fed every 301 sec (5 min)
 * @warning All the tick intervals must be lower than this*/
#define WDT_PERIOD_MS              301000

/** Flag to specify if the Watchdog timer is used or not */
#define ENABLE_WDT                 1

#define RADIO_ID 0x5A //CC1175
//#define RADIO_ID 0x58 //CC1125
//#define RADIO_ID 0x48 //CC1120
//#define RADIO_ID 0x40 //CC1121

#define LOG_ID NVM_LOGGER_LOG_USED_MAIN

#define TILT_SENSE_FREQ MS_TIMER_TICKS_MS(1000)

#define ALIVE_SIGNAL_FREQ MS_TIMER_TICKS_MS(3600*1000)

#define PKT_PER_DETECT 10

#define DETECTS_BEFORE_MAINTAIN 5

#define TILT_THRESHOLD_VAL 866

#define PKT_DURATION 500

#define TILT_DETECT_ANGLE 30

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
#define SENSE_TICKS_SLOW MS_TIMER_TICKS_MS(300000)

#define DEPLOY_TICKS_FAST MS_TIMER_TICKS_MS(60)
#define DEPLOY_TICKS_SLOW MS_TIMER_TICKS_MS(1000)

#define DEPLOY_TIMEOUT_TICKS MS_TIMER_TICKS_MS(10*60*1000)

typedef enum
{
    LRF_STATE_PRE_DEPLOYED,
    LRF_STATE_DEPLOYMENT,
    LRF_STATE_SENSING,
    LRF_STATE_LOOP,
    LRF_STATE_MAINTAINENCE,
    LRF_MAX_STATES,
}lrf_node_states_t;

typedef enum
{
    LRF_ALIVE,
    LRF_SENSE,
    LRF_MAINTAIN,
}lrf_node_pkt_type_t;

void tx_done_handler (uint32_t size);

void check_point ();

void next_interval_handler (uint32_t ticks);

void state_change_handler (uint32_t next_state);

void tilt_detected ();

void ms_timer_handler (void);

static lrf_node_ble_adv_data_t g_ble_adv_data;

static rf_spi_init_t gc_spi_hw= 
{
    .csn_pin = CSN_PIN,
    .sclk_pin = SCLK_PIN,
    .miso_pin = MISO_PIN,
    .mosi_pin = MOSI_PIN,
    .irq_priority = APP_IRQ_PRIORITY_HIGHEST,
};

static rf_comm_hw_t gc_radio_hw = 
{
    .rf_gpio0_pin = CC_GPIO0,
    .rf_gpio2_pin = CC_GPIO2,
    .rf_gpio3_pin = CC_GPIO3,
    .rf_reset_pin = CC_RESET_PIN,
#ifdef RF_COMM_AMPLIFIRE
    .rf_lna_pin = CC_LNA_PIN,
    .rf_pa_pin = CC_PA_PIN,
#endif
};

static rf_comm_radio_t gc_radio_params = 
{
    .bitrate = 300,
    .center_freq = 866000,
    .freq_dev = 2,
    .tx_power = 14,
    .irq_priority = APP_IRQ_PRIORITY_LOW,
    .rf_tx_done_handler = tx_done_handler,
};

static KXTJ3_config_t gc_acce_init = 
{
    .i2c_sda = SDA_PIN,
    .i2c_sck = SCK_PIN,
    .range = KXTJ_RNG_2g,
    .resolution = KXTJ_RES_8Bit,
};

static log_config_t gc_flag_log = 
{
    .log_id = LOG_ID,
    .entry_size = sizeof(bool),
    .start_page = NVM_LOGGER_START_PAGE,
    .no_of_pages = NVM_LOGGER_PAGES_USED_MAIN
};

bool gf_is_deployed = false;

static lrf_node_states_t gs_lrf_state = LRF_STATE_PRE_DEPLOYED;

static lrf_node_pkt_type_t g_pkt_type = LRF_ALIVE;

static lrf_node_flag_dply_t g_dply_align;

static uint32_t g_sense_cnt = 0;

//min(Random delay) > PKT_Duation
//random delay = pkt_dur + random_no*2 ms
static uint32_t g_random_delay;

const uint32_t gc_cos_res5d[] = {
    1000, 996, 984, 965, 939, 906, 866, 819, 766,
    707, 642, 573, 500, 422, 342, 258, 173, 87, 0
};

static KXTJ3_g_data_t g_acce_data;

uint8_t  angle_measure (uint32_t acce_comp)
{
    acce_comp = acce_comp < 0 ? acce_comp*(-1) : acce_comp;
    static uint8_t angle_cnt = 0;
    while((gc_cos_res5d[angle_cnt] > acce_comp) && (angle_cnt < 19))
    {
        angle_cnt++;
    }
    while((gc_cos_res5d[angle_cnt] <= acce_comp) && (angle_cnt > 0))
    {
        angle_cnt--;
    }
        
    return (angle_cnt*5);
}

bool net_acce_check (uint32_t thrs_mg)
{
    uint32_t res = (g_acce_data.xg*g_acce_data.xg) 
        + (g_acce_data.yg * g_acce_data.yg) + (g_acce_data.zg*g_acce_data.zg);
    if(res  <= (thrs_mg * thrs_mg))
    {
        return 1;
    }
    else
    {
        return 0;
    }
}
void ms_timer_handler_pre_deployed ();
void ms_timer_handler_deployment ();
void ms_timer_handler_sensing ();
void ms_timer_handler_loop ();
void ms_timer_handler_maintainence ();

void ( * ms_timer_handler_state[])(void) = {
ms_timer_handler_pre_deployed, 
ms_timer_handler_deployment,
ms_timer_handler_sensing,
ms_timer_handler_loop,
ms_timer_handler_maintainence
};

uint32_t g_arr_state_durations_ticks[] = {
    MS_TIMER_TICKS_MS(3600 * 1000),
    DEPLOY_TIMEOUT_TICKS,
    TILT_SENSE_FREQ,
    PKT_DURATION,
    ALIVE_SIGNAL_FREQ
};

void ms_timer_handler_pre_deployed ()
{

}

void ms_timer_handler_deployment ()
{
    lrf_node_ble_disconn ();
    //ble timeout
    check_point ();
}

void ms_timer_handler_loop ()
{
    log_printf("detected\n");
    uint8_t pkt_data[2];
    pkt_data[0] = aa_aaa_battery_status ();
    pkt_data[1] = angle_measure (g_acce_data.xg);
    rf_comm_pkt_send (g_pkt_type, pkt_data, sizeof(pkt_data));

    static uint32_t l_pkt_cnt = 0;
    l_pkt_cnt++;
    if (l_pkt_cnt < PKT_PER_DETECT)
    {
        irq_msg_push (MSG_STATE_CHANGE, (void *)LRF_STATE_LOOP);
    }
    else
    {
        l_pkt_cnt = 0;
        irq_msg_push (MSG_STATE_CHANGE, (void *)LRF_STATE_SENSING);
    }
}

void ms_timer_handler_sensing ()
{
    memcpy (&g_acce_data,kxtj3_get_acce_value (), sizeof(KXTJ3_g_data_t));

    bool l_acce_flag = net_acce_check (1200);

//    log_printf("tilt sense : %d\n", g_sense_cnt);
//    log_printf("Acce : %d\n", net_acce_check(1500));
//    log_printf("Angle : %d\n",angle_measure (g_acce_data.xg));

    if((l_acce_flag) && (angle_measure (g_acce_data.xg) > TILT_DETECT_ANGLE))
    {
        g_sense_cnt++;
        g_pkt_type = LRF_SENSE;
        if(g_sense_cnt < DETECTS_BEFORE_MAINTAIN)
        {
            irq_msg_push (MSG_STATE_CHANGE, (void *)LRF_STATE_LOOP);
        }
        else
        {
            irq_msg_push (MSG_STATE_CHANGE, (void *)LRF_STATE_MAINTAINENCE);
        }
    }
    else
    {
        g_sense_cnt = 0;
    }
}

void ms_timer_handler_maintainence ()
{
    log_printf("alive\n");
    uint8_t pkt_data[2];
    memcpy (&g_acce_data,kxtj3_get_acce_value (), sizeof(KXTJ3_g_data_t));
    pkt_data[0] = aa_aaa_battery_status ();
    pkt_data[1] = angle_measure (g_acce_data.xg);
    for(uint32_t pkt_cnt = 0; pkt_cnt < PKT_PER_DETECT; pkt_cnt++)
    {
        g_random_delay = random_num_generate ();
        hal_nop_delay_ms (PKT_DURATION+g_random_delay);
        rf_comm_pkt_send (g_pkt_type, pkt_data, sizeof(pkt_data));
    }
}


void check_point ()
{
    if(gf_is_deployed)
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
    log_printf("t%d\n",ticks);
    button_ui_add_tick (ticks);
    uint8_t pkt_data[1];
    static uint32_t l_alive_ticks = 0;
    l_alive_ticks += ticks;
    pkt_data[0] = aa_aaa_battery_status ();
    if((gs_lrf_state == LRF_STATE_SENSING) && (l_alive_ticks > ALIVE_SIGNAL_FREQ))
    {
        g_pkt_type = LRF_ALIVE;
        rf_comm_pkt_send (g_pkt_type, pkt_data, sizeof(pkt_data));
        l_alive_ticks = 0;
    }
    if((gs_lrf_state == LRF_STATE_DEPLOYMENT))
    {
        memcpy (&g_acce_data,kxtj3_get_acce_value (), sizeof(KXTJ3_g_data_t));

        if(angle_measure (g_acce_data.xg) < TILT_DETECT_ANGLE)
        {
            g_dply_align.align_flag = 1;
        }
        else
        {
            g_dply_align.align_flag = 0;
        }
        lrf_node_ble_update_dply_alignment (&g_dply_align);
    }
}

void state_change_handler (uint32_t next_state)
{
    log_printf("%s : %d\n", __func__, next_state);
    ms_timer_stop (MS_TIMER_ID);
    gs_lrf_state = next_state;
    switch ((lrf_node_states_t)next_state)
    {
        case LRF_STATE_PRE_DEPLOYED :
            break;
        case LRF_STATE_DEPLOYMENT :
            {
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

                    lrf_node_prodc_info_t l_ble_pkt;

                    uint8_t * p_temp = (uint8_t * )&NRF_UICR->CUSTOMER[4];
                    memcpy (&l_ble_pkt, p_temp, sizeof(lrf_node_prodc_info_t));
                    lrf_node_ble_update_prodict_info(&l_ble_pkt);
                    log_printf("Here\n");
                }
                lrf_node_ble_adv_start();

                ms_timer_start (MS_TIMER_ID, MS_SINGLE_CALL,
                                g_arr_state_durations_ticks[next_state],
                                ms_timer_handler_state[next_state]);
            }
            break;
        case LRF_STATE_SENSING :
            g_pkt_type = LRF_ALIVE;
            ms_timer_start (MS_TIMER_ID, MS_REPEATED_CALL,
                            g_arr_state_durations_ticks[next_state],
                            ms_timer_handler_state[next_state]);
            break;
        case LRF_STATE_LOOP :
            g_random_delay = random_num_generate ();
            g_arr_state_durations_ticks[next_state] = MS_TIMER_TICKS_MS
                (PKT_DURATION + g_random_delay);
            ms_timer_start (MS_TIMER_ID, MS_SINGLE_CALL,
                            g_arr_state_durations_ticks[next_state],
                            ms_timer_handler_state[next_state]);
            break;
        case LRF_STATE_MAINTAINENCE :
            g_sense_cnt = 0;
            g_pkt_type = LRF_MAINTAIN;
            ms_timer_start (MS_TIMER_ID, MS_REPEATED_CALL,
                            g_arr_state_durations_ticks[next_state],
                            ms_timer_handler_maintainence);
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
    if(dply_flag != (gf_is_deployed ))
    {
        log_printf("dply %d %d\n", dply_flag, gf_is_deployed);
        gf_is_deployed = dply_flag;
        nvm_logger_feed_data (LOG_ID, &gf_is_deployed);
    }
    g_dply_align.deploy_flag = gf_is_deployed;
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


void tx_done_handler (uint32_t size)
{
    log_printf("%s\n", __func__);
}

int main ()
{
    lfclk_init (LFCLK_SRC_Xtal);
    log_init ();
    ms_timer_init (APP_IRQ_PRIORITY_LOW);
    log_printf("Hello world from LRF Node\n");    

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

    nvm_logger_mod_init ();
    button_ui_init(HALL_EFFECT_PIN, APP_IRQ_PRIORITY_LOW,
            magnet_detect_handler);

    
    nvm_logger_log_init (&gc_flag_log);
    hal_gpio_cfg_output (TCXO_EN_PIN, 0);
    hal_gpio_pin_set (TCXO_EN_PIN);
 
    lrf_node_ble_init(ble_evt_handler, deployment_flag_update,
                      bootloader_flag_update);
    lrf_node_ble_set_adv_data (&g_ble_adv_data, g_device_name);
    
    rf_spi_init (&gc_spi_hw);
    log_printf("Here\n");
    rf_comm_radio_init (&gc_radio_params, &gc_radio_hw);
    log_printf("Device ID : 0x%x\n", rf_comm_get_radio_id ());
    if(rf_comm_get_radio_id () != RADIO_ID)
    {
        log_printf("System halt : Radio not detected.\n");
        while(1);
    }
//    hal_gpio_pin_clear (TCXO_EN_PIN);
    
    if (nvm_logger_is_log_empty (LOG_ID) == false)
    {
        nvm_logger_fetch_tail_data (LOG_ID, &gf_is_deployed, 1);
        log_printf("Is deployed : %d\n", gf_is_deployed);
        g_dply_align.deploy_flag = gf_is_deployed;
    }
    
    kxtj3_init (&gc_acce_init);
    
    
    
    rf_comm_pkt_t pkt_config = 
    {
        .max_len = 10,
        .app_id = 1,
        .dev_id = 10,
    };
    rf_comm_pkt_config (&pkt_config);
    check_point ();
    while(1)
    {
#if ENABLE_WDT == 1
        hal_wdt_feed ();
#endif
        device_tick_process ();
        irq_msg_process ();
        __WFI ();
    }
}