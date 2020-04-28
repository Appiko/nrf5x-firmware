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
//#include "radio_drv.h"
//#include "cc1x_utils.h"
//#include "cc112x_def.h"

#include "rf_comm.h"
#include "rf_spi_hw.h"


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

uint8_t g_arr_mac_addr[255];

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
void rx_failed_handler (uint32_t error);
void rx_done_handler (uint32_t size);

static rf_spi_init_t gc_spi_hw= 
{
    .csn_pin = 23,
    .sclk_pin = 22,
    .miso_pin = 3,
    .mosi_pin = 4,
    .irq_priority = APP_IRQ_PRIORITY_HIGHEST,
};

static rf_comm_hw_t gc_radio_hw = 
{
    .rf_gpio0_pin = 26,
    .rf_gpio2_pin = 20,
    .rf_gpio3_pin = 21,
    .rf_reset_pin = 24,
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
    .rx_bandwidth = 8,
    .irq_priority = APP_IRQ_PRIORITY_LOW,
    .rf_rx_done_handler = rx_done_handler,
    .rf_rx_failed_handler = rx_failed_handler,
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

void rx_failed_handler (uint32_t error)
{
    log_printf("%s : %d\n", __func__, error);
    rf_comm_idle ();
    rf_comm_rx_enable();
}

void rx_done_handler (uint32_t size)
{
    log_printf("%s : %d\n", __func__, size);
    {
        uint8_t pkt_len;
        rf_comm_pkt_receive (g_arr_mac_addr, &pkt_len);
        cRxData = sizeof(uint8_t)*ARRAY_SIZE(vectcRxBuff);
        /* Flush the RX FIFO */
        log_printf("Data: ");
        for(uint32_t i =0; i < pkt_len; i++)
        {
            log_printf("%d  ", g_arr_mac_addr[i]);
        }
        log_printf("\n");
        ble_data.pkt_no = 99;
        bool l_match_found = false;
        uint32_t l_lkp_cnt = 0;
        while((l_match_found == false) && (l_lkp_cnt < 5))
        {
            if((memcmp (g_arr_mac_addr, lookup_table[l_lkp_cnt].MAC, 6)) == 0)
            {
                l_match_found = true;
                ble_data.pkt_no = l_lkp_cnt+1;
#ifdef LOG_TEENSY        
                log_printf("%d : ", l_lkp_cnt);
                hal_uart_putchar (lookup_table[l_lkp_cnt].char_id);
                log_printf("\n");
#endif
            }
            l_lkp_cnt++;
        }
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
        
//        if(is_connected)
        {
            ble_data.rf_rx_rssi = (uint8_t)rf_comm_get_rssi ();
            ble_data.mag_status = 0;
            ble_data.CRC_ERR = 0;
            rf_rx_ble_update_status_byte (&ble_data);
        }
        log_printf("RSSI : %d\n", (int8_t)ble_data.rf_rx_rssi);
        rssi_sum += ble_data.rf_rx_rssi;
    }
    rf_comm_idle ();
    rf_comm_rx_enable();
//    else
//    {
//        log_printf ("*** Data not ready.\n");
//    }

//    start_rx();
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
    rf_spi_init (&gc_spi_hw);
    
    rf_comm_radio_init (&gc_radio_params, &gc_radio_hw);
//    radio_init(APPIKO_1120_0K3);
//    radio_set_freq (915000);
//    set_rf_packet_length (6);

//    hal_gpio_cfg_output (LNA_EN_PIN, 1);
//    hal_gpio_cfg_input (GPIO_PIN, HAL_GPIO_PULL_DOWN);
//    NRF_GPIOTE->CONFIG[GPIOTE_CHANNEL_USED] = ((GPIOTE_CONFIG_MODE_Event << GPIOTE_CONFIG_MODE_Pos) & GPIOTE_CONFIG_MODE_Msk) |
//        ((GPIOTE_CONFIG_POLARITY_HiToLo << GPIOTE_CONFIG_POLARITY_Pos)&GPIOTE_CONFIG_POLARITY_Msk) |
//        ((GPIO_PIN << GPIOTE_CONFIG_PSEL_Pos) & GPIOTE_CONFIG_PSEL_Msk);
//    NRF_GPIOTE->INTENSET = (GPIOTE_INTENSET_IN0_Enabled<<GPIOTE_INTENSET_IN0_Pos)&GPIOTE_INTENSET_IN0_Msk;

    
    rf_rx_ble_stack_init ();
    rf_rx_ble_gap_params_init ();
    rf_rx_ble_adv_init ();
    rf_rx_ble_adv_start (on_connect);
    rf_rx_ble_service_init ();
//
    ble_data.rf_rx_rssi = 0;
    rf_rx_ble_update_status_byte (&ble_data);
    
    rf_comm_pkt_t pkt_config = 
    {
        .max_len = 6,
    };
    rf_comm_pkt_config (&pkt_config);
    rf_comm_idle ();
    
    rf_comm_rx_enable();

//    start_rx();

    log_printf("Here..!!\n");
//    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
//    NVIC_SetPriority (GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
//    NVIC_ClearPendingIRQ (GPIOTE_IRQn);
//    NVIC_EnableIRQ (GPIOTE_IRQn);
    while(1)
    {
        slumber ();
    }
}

/** @} */
/** @} */
