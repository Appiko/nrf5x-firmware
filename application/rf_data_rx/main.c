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
#include "rf_rx_ble.h"
#include "ble.h"
#include "nrf_soc.h"
#include "nrf_sdm.h"
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
  S2LP_GPIO_0,
  S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
  S2LP_GPIO_DIG_OUT_READY
};


/**
 * @brief Rx buffer declaration: how to store the received data
 */
uint8_t vectcRxBuff[128], cRxData;

uint16_t current_pkt_no;

volatile bool is_timer_on = false;

uint16_t pkt_no = 0;

uint16_t start_pkt_no = 0;

int32_t rssi_sum = 0;

/**
* @brief Preemption priority IRQ
*/
#define IRQ_PREEMPTION_PRIORITY         0x03

#define GPIOTE_CHANNEL_USED 0

#define TEST_DURATION_S 100

#define TEST_DURATION_MS TEST_DURATION_S*1000

/**
* @brief IRQ status struct declaration
*/
S2LPIrqs xIrqStatus;


static volatile bool rx_started = false;

static mod_ble_data_t ble_data;

static volatile bool is_connected = false;
void ms_timer_handler ()
{
    log_printf ("Packets dropped in %d sec : %d\n",TEST_DURATION_S, TEST_DURATION_S - pkt_no);
    log_printf ("Avg RSSI : %d\n", (int8_t)(rssi_sum/pkt_no));
    log_printf ("Test params :\n");
    log_printf ("%d, %d, %d, %d, %d, %d\n", MODULATION_SELECT, DATARATE, FREQ_DEVIATION,
                BANDWIDTH, PREAMBLE_LENGTH, SYNC_LENGTH);
    
    log_printf ("Packet no.s : S %d, C %d\n", start_pkt_no, current_pkt_no);
  
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

void on_connect ()
{
    is_connected = true;
}
///**
// * Different calls to sleep depending on the status of Softdevice
// */
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
//

void GPIOTE_IRQHandler ()
{
    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
//    log_printf("%s\n",__func__);
    if(S2LPGpioIrqCheckFlag (RX_DATA_READY))
    {
        rx_started = true;
//        log_printf("Data Recieved : %d dBm\n", S2LPRadioGetRssidBm ());
//            if(rx_started == false)
//            {
//                ms_timer_start (MS_TIMER1, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(50), ms_timer_handler);
//            }
//            rx_started = true;

        hal_gpio_pin_toggle (LED_BLUE);

        cRxData = S2LPFifoReadNumberBytesRxFifo();

        /* Read the RX FIFO */
        S2LPSpiReadFifo(cRxData, (uint8_t *)&current_pkt_no);

//        log_printf("Test Val : %d\n", current_pkt_no);
        /* Flush the RX FIFO */
        S2LPCmdStrobeFlushRxFifo();      
//        for(uint32_t i =0; i < cRxData; i++)
//        {
//            log_printf("%x  ", vectcRxBuff[i]);
//        }
//        log_printf("\n");
        pkt_no++;
        if(is_timer_on == false)
        {
            ms_timer_start (MS_TIMER0, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(TEST_DURATION_MS), ms_timer_handler);
            is_timer_on = true;
            pkt_no = 0;
            start_pkt_no = current_pkt_no;
        }
        S2LPGpioIrqClearStatus();
        {
            ble_data.rf_rx_rssi = (uint8_t)S2LPRadioGetRssidBm ();
            ble_data.pkt_no = current_pkt_no;
            ble_data.CRC_ERR = (uint8_t) S2LPGpioIrqCheckFlag (CRC_ERROR);
            rf_rx_ble_update_status_byte (&ble_data);
        }
        rssi_sum += ble_data.rf_rx_rssi;
    }
    else if (S2LPGpioIrqCheckFlag (RX_DATA_DISC))
    {
        log_printf ("Data discarded..!!\n");
    }

    S2LPGpioInit(&xGpioIRQ);  
    S2LPCmdStrobeRx();
    
}
/**
 * @brief Function for application main entry.
 */
int main(void)
{
    lfclk_init (LFCLK_SRC_Xtal);
    /* Initial printf */
    log_init();
    log_printf("Hello World from RF_RX..!!\n");
    
#if DC_DC_CIRCUITRY == true  //Defined in the board header file
    NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Disabled << POWER_DCDCEN_DCDCEN_Pos;
#endif
    NRF_POWER->TASKS_LOWPWR = 1;

    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
    S2LPSpiInit ();
    rgb_led_init();
    rgb_led_cycle();
    hal_gpio_cfg_output (SDN,0);
    hal_gpio_pin_set (SDN);
    hal_nop_delay_ms (1);
    hal_gpio_pin_clear (SDN);
    hal_gpio_cfg_input (GPIO0, HAL_GPIO_PULL_DOWN);
    NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_USED] = ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) & GPIOTE_CONFIG_MODE_Msk) |
        ((GPIOTE_CONFIG_POLARITY_LoToHi << GPIOTE_CONFIG_POLARITY_Pos)&GPIOTE_CONFIG_POLARITY_Msk) |
        ((GPIO0 << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled<<GPIOTE_INTENSET_IN0_Pos)&GPIOTE_INTENSET_IN0_Msk;

    S2LPGpioInit(&xGpioIRQ);  
    S2LPRadioInit(&xRadioInit);
//    S2LPRadioSetMaxPALevel(S_DISABLE);
//    S2LPRadioSetPALeveldBm(7,POWER_DBM);
//    S2LPRadioSetPALevelMaxIndex(7);
//    S2LPRadioSetMaxPALevel(S_ENABLE);
    S2LPPktBasicInit(&xBasicInit);
    S2LPGpioIrqDeInit(NULL);
    {
//        S2LPGpioIrqConfig(RX_DATA_DISC,S_ENABLE);
//        S2LPGpioIrqConfig(RX_DATA_READY,S_ENABLE);

        /* payload length config */
        S2LPPktBasicSetPayloadLength(2);

        /* RX timeout config */
    SET_INFINITE_RX_TIMEOUT();
    }
    log_printf("Here..!!\n");


    S2LPGpioIrqClearStatus();
    
    rf_rx_ble_stack_init ();
    rf_rx_ble_gap_params_init ();
    rf_rx_ble_adv_init ();
    rf_rx_ble_adv_start (on_connect);
    rf_rx_ble_service_init ();

    ble_data.rf_rx_rssi = 0;
    rf_rx_ble_update_status_byte (&ble_data);


            S2LPCmdStrobeRx();
    NVIC_SetPriority (GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
    NVIC_ClearPendingIRQ (GPIOTE_IRQn);
    NVIC_EnableIRQ (GPIOTE_IRQn);
    while(1)
    {
        slumber ();
    }
}

/** @} */
/** @} */
