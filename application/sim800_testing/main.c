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

void ms_timer_handler ()
{
//    log_printf("%s\n",__func__);
    sim800_oper_add_ticks (MS_TIMER_TICKS_MS(MS_TIEMR_EPD_RR));
}

//void uarte_rx_handler (uint8_t byte)
//{
//    log_printf ("%c", byte);
//    sim800_response_handler (&state_of_sim800_num1, byte);
//}
//
//void print_uart (char * str)
//{
//    uint8_t i=0;
//    while(str[i])
//    {
//        i++;
//    }
//    str[i++] = '\n';
//    hal_uarte_puts ((uint8_t*)str, i);
//}
//
void HardFault_IRQHandler ()
{
    log_printf("%s\n",__func__);
}

uint8_t init1[] = {'A','T','\n'};
//
//
uint8_t init2[] = {'A','T','+','C','F','U','N','=','1','\n','\n'};
//
uint8_t init3[] = {"AT+CPIN?\n"};
uint8_t init31[] = {"AT+CSPN?\n"};
//uint8_t init4[] = {'A','T','+','C','S','T','T','=','"','a','i','r','t','e','l',
//'g','p','r','s','.','c','o','m','"',',','"','"',',','"','"','\n'};
//uint8_t init5[] = {"AT+CIICR\n"};
//uint8_t init6[] = {"AT+CIFSR\n"};
//uint8_t init7[] = {'A','T','+','C','I','P','S','T','A','R','T','=','"','T','C','P',
//'"',',','"','e','x','p','l','o','r','e','e','m','b','e','d','d','e','d','.',
//'c','o','m','"',',','8','0','\n'};
//uint8_t init8[] = {"AT+CIPSEND=63\n"};
//uint8_t init9[] = {"GET exploreembedded.com/wiki/images/1/15/Hello.txt HTTP/1.0 \n\n"};   
//
//

int main(void)
{
    leds_init ();
    /* Mandatory welcome message */
    log_init();
    log_printf("\nSIM800 testing : 15s wait for nw\n");
    
    lfclk_init(LFCLK_SRC_Xtal);
    ms_timer_init(APP_IRQ_PRIORITY_LOW);
//    hal_uarte_init (HAL_UARTE_BAUD_57600, APP_IRQ_PRIORITY_MID);
//    hal_uarte_start_rx (uarte_rx_handler);
//    hal_uarte_putchar ('\0');
//    hal_uarte_putchar ('\n');
//    sim800_init (&state_of_sim800_num1, print_uart, 1,0);
//    sim800_AT_request (&state_of_sim800_num1);
//    hal_nop_delay_ms (15000);
//    hal_nop_delay_ms (1000);
//    hal_uarte_process ();
    
//    hal_uarte_puts (init1, sizeof(init1));
//    hal_nop_delay_ms (1000);
//    hal_uarte_process ();
//    hal_nop_delay_ms (30000);
//    hal_uarte_puts (init2, sizeof(init2));
//    hal_nop_delay_ms (15000);
//    hal_uarte_process ();
//    
//    hal_nop_delay_ms (7000);
//    hal_uarte_puts (init3, sizeof(init3));
//    hal_nop_delay_ms (700);
//    hal_uarte_process ();
//    
//    hal_uarte_puts (init31, sizeof(init31));
//    while(state_of_sim800_num1.communication_stage != proc_completed);
//    sim800_ATplusCFUN_request (&state_of_sim800_num1);
//
//    while(state_of_sim800_num1.communication_stage != proc_completed)
//    {
//        hal_nop_delay_ms (5000);
//        hal_uarte_process ();
//        log_printf("Stuck in Loop1\n");
//    }
//    sim800_ATplusCPINquestion_request (&state_of_sim800_num1);
//    hal_nop_delay_ms (5000);
//    hal_uarte_process ();
//    
//    
//    sim800_ATplusCSTT_request (&state_of_sim800_num1);
//    while(state_of_sim800_num1.communication_stage != proc_completed)
//    {
//        hal_nop_delay_ms (5000);
//        hal_uarte_process ();
//        log_printf("Stuck in Loop2\n");
//    }
////    hal_nop_delay_ms (10000);
////    hal_uarte_process ();
////    hal_nop_delay_ms (5000);
//    sim800_ATplusCIICR_request (&state_of_sim800_num1);
//    while(state_of_sim800_num1.communication_stage != proc_completed)
//    {
//        hal_nop_delay_ms (5000);
//        hal_uarte_process ();
//        log_printf("Stuck in Loop3\n");
//    }
//    sim800_ATplusCIFSR_request(&state_of_sim800_num1);
//    while(state_of_sim800_num1.communication_stage != proc_completed)
//    {
//        hal_nop_delay_ms (5000);
//        hal_uarte_process ();
//        log_printf("Stuck in Loop4\n");
//    }
//    log_printf("Here\n");
    sim800_oper_init (SIM800_VODAFONE);
    sim800_oper_enable_gprs ();
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
