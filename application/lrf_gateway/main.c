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
//#include "nrf_nvic.h"

#include "boards.h"
#include "hal_clocks.h"
#include "ms_timer.h"
#include "hal_spim.h"
#include "hal_gpio.h"
#include "hal_nop_delay.h"
#include "log.h"
#include "nrf_util.h"

#include "rf_comm.h"
#include "rf_spi_hw.h"
#include "byte_frame.h"
#include "aa_aaa_battery_check.h"

#ifdef LOG_TEENSY
#include "hal_uart.h"
#include "tinyprintf.h"
#endif

#define APP_ID_POS          0
#define DEV_ID_POS          1
#define GSM_PKT_TYPE_POS    3
#define PAYLOAD_POS         4

/**
 * @brief Rx buffer declaration: how to store the received data
 */

//uint16_t current_pkt_no;


uint16_t start_pkt_no = 0;

//int32_t rssi_sum = 0;

/**
* @brief Preemption priority IRQ
*/
#define GPIOTE_CHANNEL_USED 0

#define GPIO_PIN CC_GPIO2

#define TESTING_MODE 0

#define TEST_DURATION_S 100

#define TEST_DURATION_MS TEST_DURATION_S*1000

typedef enum
{
    GSM_GATEWAY_PKT = 2,
    GSM_NODE_PKT = 1,
}gsm_pkt_types_t;


//static volatile bool rx_started = false;

//static mod_ble_data_t ble_data;



/**
 * |  HEADER   | PKT_TYPE | PAYLOAD |
 * | 1B |  2B  |  GATEWAY | Batt    |
 * |proj|Dev ID|  NODE    | Tx Pkt  |
 */
//uint8_t g_arr_gsm_pkt[128];

void rx_failed_handler (uint32_t error);
void rx_done_handler (uint32_t size);

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
    .rf_lna_pin = CC_LNA_EN_PIN,
    .rf_pa_pin = CC_PA_EN_PIN,
    .rf_hgm_pin = CC_HGM_PIN,
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


/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_init(void)
{
}

/** @brief Configure the RGB LED pins as output and turn off LED */
static void rgb_led_cycle(void)
{
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


void byte_frame_done(const uint8_t * encoded_data,uint16_t len)
{
    log_printf(" Encoded Data (%d): ",len);
    for(uint32_t i = 0; i < len; i++)
    {
        log_printf ("%d ",encoded_data[i]);
    }
    log_printf("\n");

#ifdef LOG_TEENSY
//    hal_gpio_pin_toggle (13);
    uint8_t p_data[16];
    memcpy (p_data, encoded_data, len);
    hal_uart_putdata (p_data, len);
        
//    for(uint32_t i = 0; i < len; i++)
//    {
//        tfp_printf ("%c",encoded_data[i]);
//        hal_uart_putchar (encoded_data[i]);
//    }
//    hal_gpio_pin_toggle (13);
#endif
}

void ms_timer_handler ()
{
//    g_arr_gsm_pkt[GSM_PKT_TYPE_POS] = GSM_GATEWAY_PKT;

//    g_arr_gsm_pkt[PAYLOAD_POS] = aa_aaa_battery_status ();
//    encodeFrame (g_arr_gsm_pkt, PAYLOAD_POS+1, byte_frame_done);
}

//void assign_rf_pkt (uint8_t * p_rf_pkt, uint8_t len)
//{
////    g_arr_gsm_pkt[GSM_PKT_TYPE_POS] = GSM_NODE_PKT;
////    memcpy (&g_arr_gsm_pkt[PAYLOAD_POS], p_rf_pkt, len);
//    
//}

void rx_failed_handler (uint32_t error)
{
    log_printf("%s : %d\n", __func__, error);
//    hal_gpio_pin_toggle (31);
    rf_comm_idle ();
    rf_comm_rx_enable();
}

void rx_done_handler (uint32_t size)
{
//    hal_gpio_pin_toggle (13);
    uint8_t l_arr_rf_pkt[32];
//    log_printf("%s : %d\n", __func__, size);
    {
        uint8_t pkt_len;
        l_arr_rf_pkt[0] = rf_comm_get_rssi ();
        rf_comm_pkt_receive (&l_arr_rf_pkt[1], &pkt_len);
        /* Flush the RX FIFO */
//        log_printf("Data: ");
        for(uint32_t i =0; i < pkt_len; i++)
        {
//            log_printf("%d  ", l_arr_rf_pkt[i+1]);
//#ifdef LOG_TEENSY        
//            log_printf("%d : ", l_lkp_cnt);
//            hal_uart_putchar (l_arr_rf_pkt[i]);
//            hal_uart_putchar('\n');
//#endif
        }
//        log_printf("\n");
        encodeFrame (l_arr_rf_pkt, (pkt_len+1), byte_frame_done);
//        int8_t rf_rx_rssi;
//        rf_rx_rssi = (uint8_t)rf_comm_get_rssi ();
//        log_printf("RSSI : %d\n", (int8_t)l_arr_rf_pkt[0]);
//        rssi_sum += rf_rx_rssi;
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
    hal_uart_init(HAL_UART_BAUD_1M, NULL);
#endif
    
#if DC_DC_CIRCUITRY == true  //Defined in the board header file
    NRF_POWER->DCDCEN = POWER_DCDCEN_DCDCEN_Disabled << POWER_DCDCEN_DCDCEN_Pos;
#endif
    NRF_POWER->TASKS_LOWPWR = 1;

    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
#ifdef TCXO_EN_PIN
    hal_gpio_cfg_output (TCXO_EN_PIN, 1);
    hal_gpio_pin_set (TCXO_EN_PIN);
#endif
    
    hal_gpio_cfg_output(13, 1);
//    hal_gpio_cfg_output(31, 1);
    rgb_led_init();
    rgb_led_cycle();
    rf_spi_init (&gc_spi_hw);
    
    rf_comm_radio_init (&gc_radio_params, &gc_radio_hw);
    
    rf_comm_pkt_t pkt_config = 
    {
        .max_len = 6,
    };
    rf_comm_pkt_config (&pkt_config);
    rf_comm_idle ();
    
    rf_comm_rx_enable();
    
    ms_timer_start (MS_TIMER0, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(300*1000),
                    ms_timer_handler);
    
//    start_rx();

    log_printf("Here..!!\n");
//    NRF_GPIOTE->EVENTS_IN[GPIOTE_CHANNEL_USED] = 0;
//    NVIC_SetPriority (GPIOTE_IRQn, APP_IRQ_PRIORITY_LOW);
//    NVIC_ClearPendingIRQ (GPIOTE_IRQn);
//    NVIC_EnableIRQ (GPIOTE_IRQn);
    while(1)
    {
//        slumber ();
        __WFI ();
    }
}

/** @} */
/** @} */
