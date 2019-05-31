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
#include "math.h"

#include "nrf.h"

#include "boards.h"
#include "hal_clocks.h"
#include "ms_timer.h"
#include "hal_spim.h"
#include "hal_gpio.h"
#include "hal_nop_delay.h"
#include "log.h"
#include "nrf_util.h"
#include "pin_trace.h"
#include "S2LP_Config.h"
#include "SDK_Configuration_Common.h"
#include "radio_drv.h"
#include "cc1x_utils.h"
#include "cc112x_def.h"
#include "spi_rf_nrf52.h"

/**
* @brief Radio structure fitting
*/
//SRadioInit xRadioInit = 
//{
//    
//    .lBandwidth = BANDWIDTH,
//    .lDatarate = DATARATE,
//    .xModulationSelect = MODULATION_SELECT,
//    .lFreqDev = FREQ_DEVIATION,
//    .lFrequencyBase = BASE_FREQUENCY,
//  
//  
//};
//
//
///**
//* @brief Packet Basic structure fitting
//*/
//PktBasicInit xBasicInit={
//  PREAMBLE_LENGTH,
//  SYNC_LENGTH,
//  SYNC_WORD,
//  VARIABLE_LENGTH,
//  EXTENDED_LENGTH_FIELD,
//  CRC_MODE,
//  EN_ADDRESS,
//  EN_FEC,
//  EN_WHITENING
//};
//
//
///**
//* @brief GPIO structure fitting
//*/
//SGpioInit xGpioIRQ={
//  S2LP_GPIO_0,
//  S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
//  S2LP_GPIO_DIG_OUT_READY
//};

//S2LPIrqs myGpioIrq = 
//{
//    
//    .IRQ_TX_DATA_SENT = 1,
//};

#define GPIOTE_CHANNEL_USED 0

#define GPIO_PIN GPIO0



/**
* @brief IRQ status struct declaration
*/
//S2LPIrqs xIrqStatus;
//volatile uint8_t arr_test[2];
volatile uint16_t test_cnt;
void ms_timer_handler ()
{
//    log_printf("%s \n",__func__);
//    S2LPCmdStrobeFlushTxFifo();
//    S2LPSpiWriteFifo(sizeof(arr_test), arr_test);
//    S2LPSpiWriteFifo(sizeof(test_cnt), (uint8_t *)&test_cnt);
//    arr_test[1] = 0xCD;
//    arr_test[0] = 0xAB;
//    hal_gpio_pin_set (PA_EN_PIN);
//    hal_nop_delay_us (500);
//    radio_send ((uint8_t *)arr_test, sizeof(arr_test));
    
    test_cnt++;
    radio_send ((uint8_t *)&test_cnt, sizeof(test_cnt));
//    radio_transmit ();
//    radio_prepare ((unsigned char *)arr_test, (uint16_t)sizeof(arr_test));
    
//    hal_nop_delay_ms(50);
//    radio_idle ();
    /* fit the TX FIFO */

    /* send the TX command */
//    S2LPGpioInit(&xGpioIRQ);  
//    S2LPCmdStrobeTx();
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_init(void)
{
    hal_gpio_cfg_output(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_cfg_output(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_cfg_output(LED_BLUE, !(LEDS_ACTIVE_STATE));
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_cycle(void)
{
    hal_gpio_pin_write(LED_RED, (LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
    hal_nop_delay_ms(250);
    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, (LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
    hal_nop_delay_ms(250);
    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, (LEDS_ACTIVE_STATE));
    hal_nop_delay_ms(250);
    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
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

void GPIOTE_IRQHandler ()
{
    log_printf("%s\n",__func__);
    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
    hal_gpio_pin_toggle (LED_RED);
//    S2LPGpioInit(&xGpioIRQ);  
    if(radio_check_status_flag (MARC_NO_FAILURE) )
    {
        log_printf("Data sent\n");
//        hal_gpio_pin_clear (PA_EN_PIN);
            trxSpiCmdStrobe (SFTX);

//        S2LPGpioIrqClearStatus();
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
    log_printf("Hello World from Long range RF Comm..!!\n");

    lfclk_init (LFCLK_SRC_Xtal);
    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
//    for(uint8_t cnt = 0; cnt < ARRAY_SIZE(arr_test); cnt++)
//    {
//        arr_test[cnt] = cnt+1;
//    }
    test_cnt = 0;
//    S2LPSpiInit ();

    radio_init(4);
    radio_set_freq (915000);
//    set_rf_packet_length ((unsigned char)sizeof(arr_test));
//    radio_prepare ((unsigned char *)&test_cnt, (uint16_t)sizeof(test_cnt));
    set_rf_packet_length (sizeof(test_cnt));
    log_printf("Here..!!\n");
    hal_gpio_cfg_output (PA_EN_PIN, 1);

    hal_gpio_cfg_input (GPIO_PIN, HAL_GPIO_PULL_DISABLED);
    NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_USED] = ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) & GPIOTE_CONFIG_MODE_Msk) |
        ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)&GPIOTE_CONFIG_POLARITY_Msk) |
        ((GPIO_PIN << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled<<GPIOTE_INTENSET_IN0_Pos)&GPIOTE_INTENSET_IN0_Msk;

        ms_timer_start (MS_TIMER1, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(1000), ms_timer_handler);

    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
    NVIC_SetPriority (GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ (GPIOTE_IRQn);
    NVIC_EnableIRQ (GPIOTE_IRQn);

//    ms_timer_start (MS_TIMER2, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(10), ms_timer_10ms);
    while(1)
    {    
        __WFI ();
    }
}

/** @} */
/** @} */
