/*
 *  main.c : Application to receive data over RF
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
 * @defgroup bluey_demo An application to demo the Bluey's capabilities
 * @brief A Bluey demo
 * @{
 */

#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include "math.h"

#include "nrf.h"
#include "nrf_nvic.h"

#include "boards.h"
#include "hal_clocks.h"
#include "ms_timer.h"
#include "hal_spim.h"
#include "hal_gpio.h"
#include "hal_nop_delay.h"
#include "log.h"
#include "nrf_util.h"
#include "pin_trace.h"
#include "rf_rx_ble.h"
#include "ble.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
#include "radio_drv.h"
#include "cc1x_utils.h"
#include "cc112x_def.h"
#include "out_pattern_gen.h"

#ifdef LOG_TEENSY
#include "hal_uart.h"
#endif

uint8_t vectcRxBuff[128], cRxData;

static mod_ble_data_t ble_data;

bool activateMagnet;

#define MAGNET_GPIO 27

/**
 * @brief Preemption priority IRQ
 */
#define GPIOTE_CHANNEL_USED 0

#define GPIO_PIN CC_GPIO2
#define x 0
//#define y 64

void stop_magnet(uint32_t z);
void reverse(uint32_t z);

uint32_t g[100];
uint32_t magnet_pin[] = {MAGNET_GPIO};
bool init_magnet_pin_val[] = {false};


out_gen_config_t outgen_conf = {
    .num_transitions = 100,
    .next_out =
    {
        {1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 1, 0, 0},
    },
};


void make_magnet() {
    for (int i = 0; i < 100; i++) {
        if (i % 2 == 0) {
            g[i] = x;
        } else {
            g[i] = (640 - x);
        }
    }
    
    memcpy(outgen_conf.transitions_durations, g, 100 * 4);
    outgen_conf.done_handler = &reverse;
    out_gen_start(&outgen_conf);
    hal_nop_delay_ms(1000);
    
    for (int i = 0; i < 100; i++) {
        if (i % 2 == 0) {
            g[i] = 65 + x;
        } else {
            g[i] = (640 - (65 + x));
        }
    }

    memcpy(outgen_conf.transitions_durations, g, 100 * 4);
    outgen_conf.done_handler = stop_magnet;
    out_gen_start(&outgen_conf);
    hal_nop_delay_ms(1000);
}

void reverse(uint32_t z) {}
    
    
    
//}
void stop_magnet(uint32_t z) {
    
    activateMagnet = false;
}


///**
// * Different calls to sleep depending on the status of Softdevice
// */

void slumber(void) {
    uint8_t is_sd_enabled;
    sd_softdevice_is_enabled(&is_sd_enabled);
    // Would in the SENSING mode
    if (is_sd_enabled == 0) {
        __WFI();
    } else {
        sd_app_evt_wait();
    }
}

void start_rx(void) {
    bool break_now = false;
    do {
        {
            break_now = true;
        }
        radio_receive_on();
    } while (break_now == false);
}

void GPIOTE_IRQHandler() {
    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
    if (radio_check_status_flag(MARC_NO_FAILURE)) {
        activateMagnet = true;
        radio_read(vectcRxBuff, (uint8_t *) & cRxData);
        //        memcpy(g_arr_mac_addr, vectcRxBuff, 1);
        log_printf("RSSI : %d\n", (int8_t) radio_get_rssi());
        {
            ble_data.rf_rx_rssi = (uint8_t) radio_get_rssi();
        }

    } else {
        log_printf("*** Data not ready.\n");
    }

    start_rx();
}


/**
 * @brief Function for application main entry.
 */
int main(void) {
    /* Initial printf */
    log_init();
    log_printf("Hello World from RF_RX..!!\n");

    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
    lfclk_init(LFCLK_SRC_Xtal);

    
    out_gen_init(1, magnet_pin, init_magnet_pin_val);

//    for(int i =0; i< 5; i++){
//        make_magnet();
//        hal_nop_delay_ms(3000);
//    }

#ifdef LOG_TEENSY
    hal_uart_init(HAL_UART_BAUD_115200, NULL);
#endif

#if DC_DC_CIRCUITRY == true  //Defined in the board header file
    NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Disabled << POWER_DCDCEN_DCDCEN_Pos;
#endif
    NRF_POWER->TASKS_LOWPWR = 1;

    //    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);

    hal_gpio_cfg_output(TCXO_EN_PIN, 1);
    hal_gpio_pin_set(TCXO_EN_PIN);
    radio_init(APPIKO_1120_0K3);
    radio_set_freq(915000);
    set_rf_packet_length(1);

    //    hal_gpio_cfg_output (LNA_EN_PIN, 1);
    hal_gpio_cfg_input(GPIO_PIN, HAL_GPIO_PULL_DOWN);
    NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_USED] = ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) & GPIOTE_CONFIG_MODE_Msk) |
            ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos) & GPIOTE_CONFIG_POLARITY_Msk) |
            ((GPIO_PIN << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled << GPIOTE_INTENSET_IN0_Pos) & GPIOTE_INTENSET_IN0_Msk;



    ble_data.rf_rx_rssi = 0;

    start_rx();


    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
    NVIC_SetPriority(GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_EnableIRQ(GPIOTE_IRQn);
    while (1) {
        slumber();
        if (activateMagnet) {
            make_magnet();
        }
    }
}

/** @} */
/** @} */
