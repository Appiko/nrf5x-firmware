/*
 *  main.c : Application to test LSM6D sensor
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

#include "boards.h"
#include "hal_clocks.h"
#include "hal_nop_delay.h"
#include "hal_gpio.h"
//#include "hal_twim.h"
#include "common_util.h"
#include "log.h"
#include "nrf_util.h"
//#include "nrf_sdm.h"
//#include "ble_adv.h"
#include "ble.h"
#include "ms_timer.h"
#include "hal_pin_analog_input.h"
#include "simple_adc.h"

#include "KXTJ3.h"

#include "kxtj_testing_ble.h"

#include "radio_drv.h"
#include "cc1x_utils.h"
#include "cc112x_def.h"
#include "hal_spi_rf.h"

#define SMOKE_DETECT_PIN 2

#define ANI_SMOKE_PIN PIN_TO_ANALOG_INPUT(SMOKE_DETECT_PIN)

#define GPIOTE_CHANNEL_USED 0

#define GPIO_PIN CC_GPIO2

#define GPIOTE_CHANNEL_MAG 1
#define MAG_BUTTON HALL_EFFECT_PIN


KXTJ3_g_data_t g_acce_data;

uint8_t g_arr_mac_addr[6] = {};

//static uint32_t my_sqrt (uint32_t num)
//{
//    float xn, xm, flag, temp;
//    xn = 0;
//    xm = 800;
//    flag = 0;
//    while (flag == 0)
//    {
//        xn = (xm + (num / xm)) / 2;
//        if (xn == xm)
//        {
//            flag = 1;
//        }
//        else
//        {
//            xm = xn;
//        }
//    }
//    temp = xn * 1000;
//    return (uint32_t) temp;
//}
//
//bool compare_percent (uint32_t data, uint32_t ref, float per)
//{
//    if ((ref - (ref * per / 100)) <= data && data <= (ref + (ref * per / 100)))
//    {
//        return 1;
//    }
//    else
//    {
//        return 0;
//    }
//}
//
//uint32_t avg_uint32_x100 (uint32_t * p_arr, uint32_t no_of_entries)
//{
//    uint32_t add = 0;
//    float avg = 0;
//    for(uint32_t cnt = 0; cnt < no_of_entries; cnt++)
//    {
//        add += p_arr[cnt];
//    }
//    avg = add/no_of_entries;
//    return (uint32_t)(avg * 100);
//}
//
////static mod_ble_data_t ble_data;
//
//void kxtj_100ms_handler ()
//{
//    
////    x_data = (LSM6DS3_accelData_in_g(x_data));
////    x_data = (x_data * ((x_data > 0) - (x_data < 0)));
////    y_data = (LSM6DS3_accelData_in_g(y_data) );
////    y_data = (y_data * ((y_data > 0) - (y_data < 0)));
////    z_data = (LSM6DS3_accelData_in_g(z_data) );
////    z_data = (z_data * ((z_data > 0) - (z_data < 0)));
//}
//
void ms_timer_handler ()
{
    log_printf("%s\n", __func__);

    memcpy (&g_acce_data, kxtj3_get_acce_value (), sizeof(KXTJ3_g_data_t));
    
    log_printf("mG data : %d %d %d\n", (g_acce_data.xg ),
               (g_acce_data.yg ),(g_acce_data.zg ));
    
    if((g_acce_data.xg > -866) && (g_acce_data.xg < 866))
    {
//        set_rf_packet_length (sizeof(uint8_t) * ARRAY_SIZE(g_arr_mac_addr));
        log_printf("Tilt Detected\n");
        radio_send (g_arr_mac_addr, sizeof(uint8_t) * ARRAY_SIZE(g_arr_mac_addr));
    }
    
    
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

void on_disconnect ()
{
    
    KXTJ3_config_t kxtj_init = 
    {
        .callback_handler = NULL,
        .gpio_intr = ACCE_INT_PIN,
        .i2c_sck = SCK_PIN,
        .i2c_sda = SDA_PIN,
        .range = KXTJ_RNG_2g,
        .resolution = KXTJ_RES_8Bit,
    };
    kxtj3_init (&kxtj_init);
    kxtj3_start ();
    hal_gpio_cfg_input (GPIO_PIN, HAL_GPIO_PULL_DISABLED);
    NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_USED] = ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) & GPIOTE_CONFIG_MODE_Msk) |
        ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)&GPIOTE_CONFIG_POLARITY_Msk) |
        ((GPIO_PIN << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled<<GPIOTE_INTENSET_IN0_Pos)&GPIOTE_INTENSET_IN0_Msk;
    
    hal_gpio_cfg_input (MAG_BUTTON, HAL_GPIO_PULL_DISABLED);
    NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_MAG] = ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) & GPIOTE_CONFIG_MODE_Msk) |
        ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)&GPIOTE_CONFIG_POLARITY_Msk) |
        ((MAG_BUTTON << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);
    NRF_GPIOTE->INTENSET |= (GPIOTE_INTENSET_IN1_Enabled<<GPIOTE_INTENSET_IN1_Pos)&GPIOTE_INTENSET_IN1_Msk;


    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_MAG] = 0;

        ms_timer_start (MS_TIMER1, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(1000), ms_timer_handler);

    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
    NVIC_SetPriority (GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ (GPIOTE_IRQn);
    NVIC_EnableIRQ (GPIOTE_IRQn);

}
/**
 * Different calls to sleep depending on the status of Softdevice
 */
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

void i2c_busscan ()
{
}


void GPIOTE_IRQHandler ()
{
    log_printf("%s\n",__func__);
//    S2LPGpioInit(&xGpioIRQ);  
    if(NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] & 
       radio_check_status_flag (MARC_NO_FAILURE) )
    {
        NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
        log_printf("Data sent\n");
//        hal_gpio_pin_toggle (TEST_LED);
//        hal_gpio_pin_clear (PA_EN_PIN);
            trxSpiCmdStrobe (SFTX);

//        S2LPGpioIrqClearStatus();
    }
    if(NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_MAG])
    {
        NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_MAG] = 0;
        log_printf ("Magnet detected\n");
        hal_gpio_pin_toggle (TEST_LED);
    }
}

/**
 * @brief Function for application main entry.
 */
int main(void)
{
    rgb_led_init();
    rgb_led_cycle();
    /* Initial printf */
    log_init();
    log_printf("Hello World from LSM6D..!!\n");

    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
    lfclk_init (LFCLK_SRC_Xtal);
    uint64_t l_dev_addr;
    l_dev_addr = NRF_FICR->DEVICEADDR[1];
    l_dev_addr = (l_dev_addr << 32) & 0x0000FFFF00000000;
    l_dev_addr = NRF_FICR->DEVICEADDR[0] | l_dev_addr;
    memcpy (g_arr_mac_addr, &l_dev_addr, sizeof(g_arr_mac_addr));
    
    log_printf("Mac address");
    for(uint32_t loc = 0;loc < sizeof(g_arr_mac_addr); loc++)
    {
        log_printf(" : %x",g_arr_mac_addr[loc]);
    }
    log_printf("\n");
    
    hal_gpio_cfg_output (TCXO_EN_PIN, 1);
    hal_gpio_pin_set (TCXO_EN_PIN);
    hal_gpio_cfg_output (TEST_LED, 0);
 
    radio_hw_config_t radio_hw = 
    {
        .miso_pin = MISO_PIN,
        .mosi_pin = MOSI_PIN,
        .csn_pin = CSN_PIN,
        .sclk_pin = SCLK_PIN,
        .reset_pin = CC_RESET_PIN
    };
    radio_init (APPIKO_1120_0K3, &radio_hw);
    radio_set_freq (915000);
    
    set_rf_packet_length (sizeof(uint8_t) * ARRAY_SIZE(g_arr_mac_addr));
        
    
    
    
//    LSM6DS3_init();
//    {
        kxtj_ble_stack_init ();
        kxtj_ble_gap_params_init ();
        kxtj_ble_adv_init ();
        kxtj_ble_adv_start (on_disconnect);
        kxtj_ble_service_init ();
//    }
    while (true)
    {
//        slumber ();
        __WFI();
    }
}

/** @} */
/** @} */
