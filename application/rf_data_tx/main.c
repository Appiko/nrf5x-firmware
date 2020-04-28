/*
 *  main.c : Application to transmit data over RF
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
#include "string.h"
#include "math.h"

#include "nrf.h"
#include "common_util.h"
#include "boards.h"
#include "hal_clocks.h"
#include "ms_timer.h"
#include "hal_nop_delay.h"
#include "log.h"
#include "nrf_util.h"

#include "hal_gpio.h"

#include "rf_comm.h"
#include "rf_spi_hw.h"

void node_rf_wakeup ();
void node_rf_sleep ();
void node_rf_tx_done (uint32_t status);
void node_rf_tx_failed (uint32_t status);


void node_rf_tx_done (uint32_t status)
{
    log_printf("Tx Done : %d\n", status);
    node_rf_sleep ();
}

void node_rf_tx_failed (uint32_t status)
{
    //in certain cases
    log_printf("Tx Failed : %d\n", status);
    {
        node_rf_sleep ();

    }
}

static uint32_t g_pin_tcxo_en = TCXO_EN_PIN;
static rf_spi_init_t rf_spi = 
{
    .csn_pin = CSN_PIN,
    .sclk_pin = SCLK_PIN,
    .miso_pin = MISO_PIN,
    .mosi_pin = MOSI_PIN,
    .irq_priority = APP_IRQ_PRIORITY_HIGHEST,
};

static rf_comm_hw_t g_rf_comm_hw = 
{
    .rf_gpio2_pin = CC_GPIO2,
    .rf_reset_pin = CC_RESET_PIN,
};

static rf_comm_radio_t g_rf_comm_radio = 
{
    .bitrate = 300,
    .tx_power = 14,
    .center_freq = 866000,
    .freq_dev = 2,
    .rf_tx_done_handler = node_rf_tx_done,
    .rf_tx_failed_handler = node_rf_tx_failed,
    .irq_priority = APP_IRQ_PRIORITY_LOW,
};

static rf_comm_pkt_t rf_pkt = 
{
    .max_len = 16,
    .app_id  = 5,
    .dev_id = 7,
};

/**
* @brief IRQ status struct declaration
*/
//S2LPIrqs xIrqStatus;
uint8_t arr_test[2];
uint16_t test_cnt;
void node_rf_wakeup ()
{
    hal_gpio_pin_set (g_pin_tcxo_en);
    hal_nop_delay_ms (50);
    rf_comm_wake();
    log_printf("Radio ID :0x%x\n", rf_comm_get_radio_id ());
    rf_comm_radio_init (&g_rf_comm_radio, &g_rf_comm_hw);

    rf_comm_idle ();
    rf_comm_enable_irq ();
}

void node_rf_sleep ()
{
    log_printf("R Sl\n");
    rf_comm_flush ();
    rf_comm_sleep ();
//    hal_nop_delay_ms (5);
    hal_gpio_pin_clear (g_pin_tcxo_en);
    rf_comm_disable_irq ();
}

void ms_timer_handler ()
{
    
    test_cnt++;
    arr_test[0] = (uint8_t) (test_cnt & 0xFF);
    arr_test[1] = (uint8_t) ((test_cnt & 0xFF00) >> 8);
    node_rf_wakeup ();
    rf_comm_pkt_send (0, arr_test, sizeof(arr_test));
    
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_init(void)
{
//    hal_gpio_cfg_output(LED_RED, !(LEDS_ACTIVE_STATE));
//    hal_gpio_cfg_output(LED_GREEN, !(LEDS_ACTIVE_STATE));
//    hal_gpio_cfg_output(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_cycle(void)
{
//    hal_gpio_pin_write(LED_RED, (LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
//    hal_nop_delay_ms(250);
//    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_GREEN, (LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
//    hal_nop_delay_ms(250);
//    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_BLUE, (LEDS_ACTIVE_STATE));
//    hal_nop_delay_ms(250);
//    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

void ms_timer_10ms (void)
{
        
}
///**
// * Different calls to sleep depending on the status of Softdevice
// */
//void slumber(void)
//{
//    uint8_t is_sd_enabled;
//    sd_softdevice_is_enabled(&is_sd_enabled);
//    // Would in the SENSING mode
//    if(is_sd_enabled == 0)
//    {
//        __WFI();
//    }
//    else
//    {
//        sd_app_evt_wait();
//    }
//}
//
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    rgb_led_init();
    rgb_led_cycle();
    /* Initial printf */
    log_init();
    log_printf("Hello World from Long range RF Comm..!!\n");

    lfclk_init (LFCLK_SRC_Xtal);
    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
//    for(uint8_t cnt = 0; cnt < ARRAY_SIZE(arr_test); cnt++)
//    {
//        arr_test[cnt] = cnt+1;
//    }
    test_cnt = 0;
    rf_spi_init (&rf_spi);
//    S2LPSpiInit ();
    rf_comm_pkt_config (&rf_pkt);
    hal_gpio_cfg_output (g_pin_tcxo_en, 0);
    node_rf_wakeup ();
    node_rf_sleep ();

//    set_rf_packet_length ((unsigned char)sizeof(arr_test));
//    radio_prepare ((unsigned char *)&test_cnt, (uint16_t)sizeof(test_cnt));
    log_printf("Here..!!\n");
//    hal_gpio_cfg_output (PA_EN_PIN, 1);


    ms_timer_start (MS_TIMER2, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(1000), ms_timer_handler);
    while(1)
    {    
        __WFI ();
    }
}

/** @} */
/** @} */
