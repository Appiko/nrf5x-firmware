/**
 *  gpio_level_handler.c : GPIO Level Handler Driver
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

#include "gpio_level_handler.h"
#include "hal_gpio.h"

#if ISR_MANAGER == 1
#include "template_isr_manage.h"
#endif

#define GPIO_BUFFER_SIZE   4

gpio_level_cfg cfg_buffer[GPIO_BUFFER_SIZE];
uint32_t gpio_len;

void gpio_level_init(gpio_level_cfg * cfg, uint32_t cfg_num, uint32_t irq_priority)
{
    if(cfg_num == 0)
    {
        NRF_GPIOTE->EVENTS_PORT = 0;

        NRF_GPIOTE->INTENCLR = GPIOTE_INTENCLR_PORT_Enabled << GPIOTE_INTENCLR_PORT_Pos;

        NVIC_ClearPendingIRQ(GPIOTE_IRQn);
        NVIC_DisableIRQ(GPIOTE_IRQn);

        ///TODO revert the GPIO configs of the previously configured pins

        return ;
    }

    uint32_t i;
    for(i = 0; i<cfg_num; i++)
    {
        cfg_buffer[i] = *(cfg + i);

        hal_gpio_cfg((cfg + i)->pin_num,
                    GPIO_PIN_CNF_DIR_Input,
                    GPIO_PIN_CNF_INPUT_Connect,
                    (cfg + i)->pull_cfg,
                    GPIO_PIN_CNF_DRIVE_S0S1,
                    ((cfg + i)->trigger_on_high)
                        ?GPIO_PIN_CNF_SENSE_High:GPIO_PIN_CNF_SENSE_Low );
    }
    gpio_len = cfg_num;

    NRF_GPIO->DETECTMODE = GPIO_DETECTMODE_DETECTMODE_LDETECT << GPIO_DETECTMODE_DETECTMODE_Pos;
    NRF_GPIOTE->INTENSET = GPIOTE_INTENSET_PORT_Enabled << GPIOTE_INTENSET_PORT_Pos;

    NVIC_ClearPendingIRQ(GPIOTE_IRQn);
    NVIC_SetPriority(GPIOTE_IRQn, irq_priority);
    NVIC_EnableIRQ(GPIOTE_IRQn);

}
#if ISR_HADNLER == 1
void gpio_level_handler_gpiote_Handler ()
#else
void GPIOTE_IRQHandler(void)
#endif
{
    NRF_GPIOTE->EVENTS_PORT = 0;
    (void) NRF_GPIOTE->EVENTS_PORT;

    uint32_t i;
    for(i = 0; i<gpio_len; i++)
    {
        uint32_t pin_num = cfg_buffer[i].pin_num;
        if(NRF_GPIO->LATCH & (1 << pin_num))
        {
            uint32_t pin_val = hal_gpio_pin_read(pin_num);
            uint32_t level_checked = cfg_buffer[i].trigger_on_high;

            NRF_GPIO->LATCH = (1 << pin_num);
            cfg_buffer[i].handler((pin_val == level_checked));
        }
    }
}
