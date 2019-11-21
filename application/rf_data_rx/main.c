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

#ifdef LOG_TEENSY
#include "hal_uart.h"
#endif

/**
 * @brief Rx buffer declaration: how to store the received data
 */
uint8_t vectcRxBuff[128], cRxData;

//uint16_t current_pkt_no;

volatile bool is_timer_on = false;

uint16_t pkt_no = 0;

uint16_t start_pkt_no = 0;

int32_t rssi_sum = 0;

/**
* @brief Preemption priority IRQ
*/
#define GPIOTE_CHANNEL_USED 0

#define GPIO_PIN CC_GPIO2

#define TESTING_MODE 0

#define TEST_DURATION_S 100

#define TEST_DURATION_MS TEST_DURATION_S*1000

/**
* @brief IRQ status struct declaration
*/
//S2LPIrqs xIrqStatus;


//static volatile bool rx_started = false;

static mod_ble_data_t ble_data;

static volatile bool is_connected = false;

uint8_t g_arr_mac_addr[6];

typedef struct
{
    uint8_t MAC[6];
    char char_id;
}lookup_entry_t;

const lookup_entry_t lookup_table[] = 
{
    {
        .MAC = {0xb5, 0x7f, 0x9f, 0x9c, 0x56, 0x4c},
            .char_id = 'A',
    },
    {
        .MAC = {0xf7, 0xdc, 0xe1, 0x70, 0x3f, 0xcc},
            .char_id = 'B',
    },
    {
        .MAC = {0xda, 0xcb, 0x3f, 0xcf, 0x5d, 0x32},
            .char_id = 'C',
    },
    {
        .MAC = {0x9a, 0x7c, 0xdd, 0xd4, 0x34, 0xe4},
            .char_id = 'D',
    },
    {
        .MAC = {0x1e, 0x99, 0x4c, 0x7b, 0xf0, 0x28},
            .char_id = 'E',
    },

};

void ms_timer_handler ()
{
    log_printf ("Packets dropped in %d sec : %d\n",TEST_DURATION_S, TEST_DURATION_S - pkt_no);
    log_printf ("Avg RSSI : %d\n", (int8_t)(rssi_sum/pkt_no));
    log_printf ("Test params :\n");
//    log_printf ("%d, %d, %d, %d, %d, %d\n", MODULATION_SELECT, DATARATE, FREQ_DEVIATION,
//                BANDWIDTH, PREAMBLE_LENGTH, SYNC_LENGTH);
    
//    log_printf ("Packet no.s : S %d, C %d\n", start_pkt_no, current_pkt_no);
    is_timer_on = false;
  
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
//    hal_nop_delay_ms(50);
//    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_GREEN, (LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
//    hal_nop_delay_ms(50);
//    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_BLUE, (LEDS_ACTIVE_STATE));
//    hal_nop_delay_ms(50);
//    hal_gpio_pin_write(LED_RED, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_GREEN, !(LEDS_ACTIVE_STATE));
//    hal_gpio_pin_write(LED_BLUE, !(LEDS_ACTIVE_STATE));
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

void start_rx(void)
{
    bool break_now = false;
    do{
//        S2LPGpioInit(&xGpioIRQ);
//        S2LPCmdStrobeRx();

//        S2LPGpioIrqGetStatus((S2LPIrqs*) &irqs);
//        log_printf ("Rx? I0x%X S0x%X", irqs, g_xStatus);
//        if(g_xStatus.MC_STATE == MC_STATE_RX)
//        if(radio_check_status_flag (MARC_NO_FAILURE))
        {
            break_now = true;
//            log_printf(" Yay\n");
        }
        radio_receive_on (); 
//        else
//        {
//            log_printf(" Nay\n");
//            log_printf("Reset..!!");
//            uint8_t is_sd_enabled;
//            sd_softdevice_is_enabled(&is_sd_enabled);
//            if(is_sd_enabled == 0)
//            {
//                sd_nvic_SystemReset();
//            }
//            else
//            {
//                NVIC_SystemReset ();
//            }
//        }

    } while(break_now == false);
}

void GPIOTE_IRQHandler ()
{
    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
//    log_printf("%s\n",__func__);
    if(radio_check_status_flag (MARC_NO_FAILURE) )
    {
//        hal_nop_delay_ms(100);
//        rx_started = true;
//        log_printf("Data Recieved : %d dBm\n", S2LPRadioGetRssidBm ());
//            if(rx_started == false)
//            {
//                ms_timer_start (MS_TIMER1, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(50), ms_timer_handler);
//            }
//            rx_started = true;

//        hal_gpio_pin_toggle (LED_BLUE);
        
        /* Read the RX FIFO */
//        S2LPSpiReadFifo(cRxData, (uint8_t *)&current_pkt_no);
        
        cRxData = sizeof(uint8_t)*ARRAY_SIZE(vectcRxBuff);
        radio_read (vectcRxBuff, (uint8_t *)&cRxData);
        memcpy (g_arr_mac_addr, vectcRxBuff, 6);
        /* Flush the RX FIFO */
        log_printf("MAC ");
        for(uint32_t i =0; i < ARRAY_SIZE(g_arr_mac_addr); i++)
        {
            log_printf(": %x  ", g_arr_mac_addr[i]);
        }
        log_printf("\n");
#ifdef LOG_TEENSY        
        bool l_match_found = false;
        uint32_t l_lkp_cnt;
        while(l_match_found == false)
        {
            if((memcmp (g_arr_mac_addr, lookup_table[l_lkp_cnt].MAC, 6)) == 0)
            {
                l_match_found = true;
                log_printf("%d : ", l_lkp_cnt);
                hal_uart_putchar (lookup_table[l_lkp_cnt].char_id);
                log_printf("\n");
            }
            l_lkp_cnt++;
        }
#endif
//        current_pkt_no = vectcRxBuff[0] | (vectcRxBuff[1] << 8);
        pkt_no++;
        if(is_timer_on == false)
        {
#if (TESTING_MODE==1)
            ms_timer_start (MS_TIMER0, MS_SINGLE_CALL, MS_TIMER_TICKS_MS(TEST_DURATION_MS), ms_timer_handler);
            is_timer_on = true;
            pkt_no = 0;
            rssi_sum = 0;
            start_pkt_no = 0;
#endif
        }
        
        log_printf("RSSI : %d\n", (int8_t)radio_get_rssi ());
//        if(is_connected)
        {
            ble_data.rf_rx_rssi = (uint8_t)radio_get_rssi ();
            ble_data.pkt_no = pkt_no;
            ble_data.mag_status = 0;
            ble_data.CRC_ERR = (uint8_t) radio_check_status_flag (MARC_PKT_DISC_CRC);
            rf_rx_ble_update_status_byte (&ble_data);
        }
        rssi_sum += ble_data.rf_rx_rssi;
    }
    else
    {
        log_printf ("*** Data not ready.\n");
    }

    start_rx();
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

#ifdef LOG_TEENSY
    hal_uart_init(HAL_UART_BAUD_115200, NULL);
#endif
    
#if DC_DC_CIRCUITRY == true  //Defined in the board header file
    NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Disabled << POWER_DCDCEN_DCDCEN_Pos;
#endif
    NRF_POWER->TASKS_LOWPWR = 1;

    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
    
    hal_gpio_cfg_output (TCXO_EN_PIN, 1);
    hal_gpio_pin_set (TCXO_EN_PIN);
    rgb_led_init();
    rgb_led_cycle();
    radio_init(APPIKO_1120_0K3);
    radio_set_freq (915000);
    set_rf_packet_length (6);

//    hal_gpio_cfg_output (LNA_EN_PIN, 1);
    hal_gpio_cfg_input (GPIO_PIN, HAL_GPIO_PULL_DOWN);
    NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_USED] = ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) & GPIOTE_CONFIG_MODE_Msk) |
        ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)&GPIOTE_CONFIG_POLARITY_Msk) |
        ((GPIO_PIN << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);
    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled<<GPIOTE_INTENSET_IN0_Pos)&GPIOTE_INTENSET_IN0_Msk;

    
    rf_rx_ble_stack_init ();
    rf_rx_ble_gap_params_init ();
    rf_rx_ble_adv_init ();
    rf_rx_ble_adv_start (on_connect);
    rf_rx_ble_service_init ();
//
    ble_data.rf_rx_rssi = 0;
    rf_rx_ble_update_status_byte (&ble_data);

    start_rx();

    log_printf("Here..!!\n");
    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
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
