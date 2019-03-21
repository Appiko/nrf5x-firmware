/*  Copyright (c) 2016, Appiko
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

/**
* @brief Radio structure fitting
*/
SRadioInit xRadioInit = 
{
    
    .lBandwidth = BANDWIDTH,
    .lDatarate = DATARATE,
    .xModulationSelect = MODULATION_SELECT,
    .lFreqDev = FREQ_DEVIATION,
    .lFrequencyBase = BASE_FREQUENCY,
  
  
};


/**
* @brief Packet Basic structure fitting
*/
PktBasicInit xBasicInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  VARIABLE_LENGTH,
  EXTENDED_LENGTH_FIELD,
  CRC_MODE,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};


/**
* @brief GPIO structure fitting
*/
SGpioInit xGpioIRQ={
  S2LP_GPIO_3,
  S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
  S2LP_GPIO_DIG_OUT_IRQ
};


/**
 * @brief Rx buffer declaration: how to store the received data
 */
uint8_t vectcRxBuff[128], cRxData;

/**
* @brief Preemption priority IRQ
*/
#define IRQ_PREEMPTION_PRIORITY         0x03



/**
* @brief IRQ status struct declaration
*/
S2LPIrqs xIrqStatus;

void ms_timer_handler ()
{
    log_printf("%s\n",__func__);
  
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
        if(S2LPGpioIrqCheckFlag (RX_DATA_READY))
        {
            log_printf("Data Recieved\n");
//            ms_timer_start (MS_TIMER1, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(5000), ms_timer_handler);

            cRxData = S2LPFifoReadNumberBytesRxFifo();
      
            /* Read the RX FIFO */
            S2LPSpiReadFifo(cRxData, vectcRxBuff);

            /* Flush the RX FIFO */
            S2LPCmdStrobeFlushRxFifo();      
            for(uint32_t i =0; i < cRxData; i++)
            {
                log_printf("%x  ", vectcRxBuff[i]);
            }
            log_printf("\n");
        S2LPGpioIrqClearStatus();
//        hal_nop_delay_ms(400);
            S2LPCmdStrobeRx();
        }
        else if (S2LPGpioIrqCheckFlag (RX_DATA_DISC))
        {
            log_printf ("Data discarded..!!\n");
        }
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
    log_printf("Hello World from LSM6D..!!\n");

    lfclk_init (LFCLK_SRC_Xtal);
    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
    hal_gpio_cfg_output (SDN,0);
    hal_gpio_pin_set (SDN);
    hal_gpio_pin_clear (SDN);
    hal_gpio_cfg_input (GPIO3, HAL_GPIO_PULL_DISABLED);
    S2LPSpiInit ();
    S2LPGpioInit(&xGpioIRQ);  
    log_printf("Here..!!\n");
    S2LPRadioInit(&xRadioInit);
    S2LPRadioSetMaxPALevel(S_DISABLE);
    S2LPRadioSetPALeveldBm(7,POWER_DBM);
    S2LPRadioSetPALevelMaxIndex(7);
    S2LPPktBasicInit(&xBasicInit);
    S2LPGpioIrqDeInit(NULL);
    {
        S2LPGpioIrqConfig(RX_DATA_DISC,S_ENABLE);
        S2LPGpioIrqConfig(RX_DATA_READY,S_ENABLE);

        /* payload length config */
        S2LPPktBasicSetPayloadLength(100);

        /* RX timeout config */
//  S2LPTimerSetRxTimerUs(700000);
        SET_INFINITE_RX_TIMEOUT();

    }
    S2LPGpioIrqClearStatus();




    ms_timer_start (MS_TIMER2, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(10), ms_timer_10ms);
    while(1)
    {
//        if(rx_started == false)
//        {
            S2LPCmdStrobeRx();
//
//        }
        
    
        __WFI ();
    }
}

/** @} */
/** @} */
