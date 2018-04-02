/*
 *  gpio_level_handler.c
 *
 *  Created on: 30-Mar-2018
 *
 *  Copyright (c) 2018, Appiko
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

#include "gpio_level_handler.h"
#include "hal_gpio.h"

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

void GPIOTE_IRQHandler(void)
{
    NRF_GPIOTE->EVENTS_PORT = 0;
    (void) NRF_GPIOTE->EVENTS_PORT;

    uint32_t i;
    for(i = 0; i<gpio_len; i++)
    {
        uint32_t pin_num = cfg_buffer[i].pin_num;
        if(NRF_GPIO->LATCH & (1 << pin_num))
        {
            uint32_t pin_val = NRF_GPIO->IN & (1 << pin_num);
            uint32_t level_checked = cfg_buffer[i].trigger_on_high << pin_num;

            cfg_buffer[i].handler((pin_val == level_checked));
            NRF_GPIO->LATCH = (1 << pin_num);
        }
    }
}
