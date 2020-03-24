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
#include "hal_gpio.h"
#include "ms_timer.h"
#include "display_lib.h"
//
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


void HardFault_IRQHandler ()
{
    log_printf("%s\n",__func__);
}

//function to convert given sec and store time in hh:mm:ss format from given location in string 
void ms_to_time_str (uint64_t msec, uint32_t loc, uint8_t * str)
{
    uint32_t sec = msec/1000;
    uint32_t l_hh_int;
    l_hh_int = sec/3600;
    uint32_t l_mm_int;
    l_mm_int = (sec - (l_hh_int * 3600))/60;
    uint32_t l_ss_int;
    l_ss_int = (sec - (l_hh_int * 3600) - (l_mm_int * 60));
    
    uint32_t l_ms_int;
    l_ms_int = msec%1000;
    
    str[loc++] = 0x30+l_hh_int/10;
    str[loc++] = 0x30+l_hh_int%10;
    str[loc++] = ':';
    str[loc++] = 0x30+l_mm_int/10;
    str[loc++] = 0x30+l_mm_int%10;
    str[loc++] = ':';
    str[loc++] = 0x30+l_ss_int/10;
    str[loc++] = 0x30+l_ss_int%10;
    str[loc++] = '.';
    str[loc++] = 0x30+(l_ms_int)/100;
    str[loc++] = 0x30+((l_ms_int%100)/10);
    str[loc++] = 0x30+(l_ms_int%10);
}

void ms_timer_handler (void)
{
    log_printf("%s\n",__func__);
    hal_gpio_pin_toggle (LED_1);
    static uint64_t cnt = 0;
    uint8_t l_disp_str[] = {'T','h','h',':','m','m',':','s','s','.','m','m','m','\n'};
    ms_to_time_str (cnt, 1, l_disp_str);
    cnt += MS_TIEMR_EPD_RR;
    disp_lib_putS_at ((char*)l_disp_str, sizeof(l_disp_str), 0, 3);
}

disp_lib_hw_t dev_config = 
{
    .busy_pin = 11,
    .rst_pin = 12,
    .dc_pin = 13,
    .cs_pin = 14,
    .clk_pin = 15,
    .din_pin = 16,
    .dout_pin = 17
};

UG_GUI g_gui;

char test[] = {'H','e','l','l','o',' ',
    'W','o','r','l','d','.','.','!','\n','~','E','P','D','\n'};
char test1[] = {'T','e','s','t'};

int main(void)
{
    leds_init ();
    /* Mandatory welcome message */
    log_init();
    log_printf("\nE-paper display testing\n");
    
    lfclk_init(LFCLK_SRC_Xtal);
    ms_timer_init(APP_IRQ_PRIORITY_LOW);

    disp_lib_init (&dev_config, (disp_lib_font_t *)&FONT_12X16);
    log_printf("EPD : %d %d %d %d\n", EPD_BUSY_PIN, EPD_CS_PIN,
               EPD_DC_PIN, EPD_RST_PIN);
    disp_lib_putS (test, sizeof(test));
    disp_lib_putS (test1, sizeof(test1));
    disp_lib_putC (' ');
    disp_lib_putC ('1');
    disp_lib_putC_at ('T', 0, 5);
    disp_lib_putC_at ('j', 1, 5);
    ms_timer_start (MS_TIMER0, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(MS_TIEMR_EPD_RR), ms_timer_handler);
    log_printf("Here\n");
    
    
    while (true)
    {
        __WFI();
    }
}

/** @} */
/** @} */
