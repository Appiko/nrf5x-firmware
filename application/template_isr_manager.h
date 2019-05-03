/**
 *  template_isr_manager.c : Template for file to handle and share ISRs as required.
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

#ifndef TEMPLATE_ISR_MANAGER_H
#define TEMPLATE_ISR_MANAGER_H

//Drivers for hal level Irq management
void hal_gpio_Handler (void);

void hal_pwm_Handler (void);

void hal_radio_Handler (void);

void hal_saadc_Handler (void);

void hal_spim_Handler (void);

void hal_twim_Handler (void);

void hal_uart_Handler (void);

void hal_wdt_Handler (void);


//Declaration for peripheral level Irq
void ble_adv_radio_Handler (void);

void button_ui_gpiote_Handler (void);

void gpio_level_handler_gpiote_Handler (void);

void ms_timer_rtc_Handler (void);

void pir_sense_saadc_Handler (void);

void tssp_detect_swi_Handler (void);

void tssp_detect_rtc_Handler (void);

void tssp_ir_tx_timer1_Handler (void);

void tssp_ir_tx_timer2_Handler (void);

void uart_printf_uart_Handler (void);

void us_timer_timer_Handler (void);

void evt_sd_handler_swi_Handler (void);

//void sensebe_ble_swi_Handler (void);

#endif //TEMPLATE_ISR_MANAGER_H
