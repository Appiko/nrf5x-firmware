/*
 *  main.c : Application to check NVM Logger module
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

#include "boards.h"
#include "nrf.h"
#include "hal_nvmc.h"
#include "log.h"
#include "hal_clocks.h"
#include "hal_gpio.h" 
#include "hal_nop_delay.h"
#include "ms_timer.h"
#include "hal_pin_analog_input.h"
#include "simple_adc.h"
#include "common_util.h"
#include "nvm_logger.h"
#include "nrf_util.h"


#define SAMPLE_FREQ_MS  1*60*1000
#define LOG_ID 0

typedef struct
{
    uint16_t light_sense;
    uint16_t time_passed_min;
}__attribute__((packed))light_log_t;

log_config_t default_log_config1 =
{
    .log_id = LOG_ID,
    .no_of_pages = 3,
    .entry_size = sizeof(light_log_t),
    .start_page = NVM_LOG_PAGE0,
};
light_log_t g_light_log;
uint32_t g_light_sense_analog_pin = PIN_TO_ANALOG_INPUT(LIGHT_SENSE);

void ms_timer_handler ()
{
//    hal_gpio_pin_toggle (LED_RED);
    g_light_log.light_sense = simple_adc_get_value (SIMPLE_ADC_GAIN1_6, g_light_sense_analog_pin);
    g_light_log.time_passed_min++;
    nvm_logger_feed_data (LOG_ID, &g_light_log);
    hal_nop_delay_ms (5);
//    log_printf("Time :%4d, Value :%4d\n", g_light_log.time_passed_min, g_light_log.light_sense);
}

int main()
{
    log_init();
    log_printf("\nHello NVMC\n");
{
    hal_gpio_cfg_output(LED_RED, LEDS_ACTIVE_STATE);
    hal_gpio_cfg_output(LED_GREEN, !LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(600);
    hal_gpio_pin_write(LED_RED, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_GREEN, LEDS_ACTIVE_STATE);
    hal_nop_delay_ms(600);
    hal_gpio_pin_write(LED_RED, !LEDS_ACTIVE_STATE);
    hal_gpio_pin_write(LED_GREEN, !LEDS_ACTIVE_STATE);
}
    lfclk_init(LFCLK_SRC_Xtal);
    ms_timer_init (APP_IRQ_PRIORITY_HIGH);
    nvm_logger_mod_init ();
    hal_gpio_cfg_output (LIGHT_SENSE_EN, 1);
    hal_gpio_pin_set (LIGHT_SENSE_EN);
    hal_gpio_cfg_output (LED_RED, !LEDS_ACTIVE_STATE);
    nvm_logger_log_init (&default_log_config1);
    if(nvm_logger_is_log_empty (LOG_ID) == 0)
    {
        nvm_logger_fetch_tail_data (LOG_ID, &g_light_log, 1);
        hal_nop_delay_ms (5);
    }
    log_printf("\nLight Log\nTime       Light\n");
    uint32_t l_total_logs = nvm_logger_get_total_entries (LOG_ID);
    light_log_t l_buff_light_log;
    for(uint32_t log_no = 1; log_no <= l_total_logs; log_no++)
    {
        nvm_logger_fetch_tail_data (LOG_ID, &l_buff_light_log, log_no);
        hal_nop_delay_ms (5);
        log_printf("%4d     %4d\n", l_buff_light_log.time_passed_min, l_buff_light_log.light_sense);
    }
    ms_timer_start (MS_TIMER0, MS_REPEATED_CALL,
                    MS_TIMER_TICKS_MS(SAMPLE_FREQ_MS), ms_timer_handler);
    g_light_log.light_sense = simple_adc_get_value (SIMPLE_ADC_GAIN1_6, g_light_sense_analog_pin);
    g_light_log.time_passed_min++;
    nvm_logger_feed_data (LOG_ID, &g_light_log);
    hal_nop_delay_ms (5);

    while(1)
    {
        __WFI();
    }

}
