/*
 *  main.c : Application to test display library
 *  Copyright (C) 2020  Appiko
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
 *
 * @{
 *
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>
#include <string.h>

#include "boards.h"
#include "nrf.h"
#include "nrf_util.h"
#include "hal_nop_delay.h"

#include "log.h"
#include "hal_uarte.h"
#include "hal_gpio.h"
#include "ms_timer.h"
//#include "display_lib.h"
//
//#include "SIM800.h"
#include "sim800_oper.h"
#define MS_TIEMR_EPD_RR 1000


void leds_init(void)
{
    hal_gpio_cfg_output(LED_1, LEDS_ACTIVE_STATE);
    hal_gpio_cfg_output(LED_2, !LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(600);
    hal_gpio_pin_write(LED_1, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_2, LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(600);
    hal_gpio_pin_write(LED_1, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_2, !LEDS_ACTIVE_STATE);
}

const char addr[] = {"http://ptsv2.com/t/appiko/post"};
//const char res[] = {"random"};
//const char port[] = {"8080"};


const char post_msg[] = {'H','i','.','.','!','!','\r','\n'};

static sim800_server_conn_t server_info = 
{
    .server_ptr = (char *)addr,
    .server_len = sizeof (addr),
//    .resource_ptr = (char *)res,
//    .resource_len = sizeof (res),
//    .port_ptr = (char *)port,
//    .port_len = sizeof(port),
};

void ms_timer_handler ()
{
    sim800_oper_add_ticks (MS_TIMER_TICKS_MS(MS_TIEMR_EPD_RR));
}

void HardFault_IRQHandler ()
{
    log_printf("%s\n",__func__);
}

int main(void)
{
    leds_init ();
    /* Mandatory welcome message */
    log_init();
    log_printf("\nSIM800 testing : 15s wait for nw\n");
    
    lfclk_init(LFCLK_SRC_Xtal);
    ms_timer_init(APP_IRQ_PRIORITY_LOW);
    sim800_oper_init (SIM800_VODAFONE);
    sim800_oper_enable_gprs ();
    sim800_oper_conns (&server_info);
    sim800_http_req_t l_http_req = 
    {
        .req_type = SIM800_HTTP_POST,
        .payload_ptr = (uint8_t *)post_msg,
        .len = (sizeof(post_msg))
    };
    sim800_oper_http_req (&l_http_req);
    
    ms_timer_start (MS_TIMER0, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(MS_TIEMR_EPD_RR), ms_timer_handler);
    while (true)
    {
        sim800_oper_process ();
        __WFI();
//        hal_uarte_process ();
    }
}

/** @} */
/** @} */
