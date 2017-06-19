/*
 *  hal_gpio.h
 *
 *  Created on: 25-May-2017
 *
 *  Copyright (c) 2017, Appiko
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

#ifndef CODEBASE_HAL_HAL_GPIO_H_
#define CODEBASE_HAL_HAL_GPIO_H_

/**
 * @addtogroup group_hal
 * @{
 *
 * @defgroup group_gpio GPIO HAL
 * @brief Hardware abstraction layer of the General Purpose Input Output peripheral.
 * @{
 */

#include "nrf.h"

/**
 * Defines for the types of pull configurations are possible for a GPIO pin
 */
typedef enum {
  HAL_GPIO_PULL_DISABLED =  GPIO_PIN_CNF_PULL_Disabled,//!< HAL_GPIO_PULL_DISABLED
  HAL_GPIO_PULL_DOWN     =  GPIO_PIN_CNF_PULL_Pulldown,//!< HAL_GPIO_PULL_DOWN
  HAL_GPIO_PULL_UP       =  GPIO_PIN_CNF_PULL_Pullup   //!< HAL_GPIO_PULL_UP
} hal_gpio_pull_t;

/**
 * @brief Pin configuration function for nrf51 and nrf52 SoCs
 *
 * This function allows to set any aspect in PIN_CNF register.
 * Check nrf51_bitfields.h or nrf52_bitfields.h for the correct values
 * @param pin_num Specifies the pin number (allowed values 0-31).
 * @param dir   Pin direction (GPIO_PIN_CNF_DIR_Input or GPIO_PIN_CNF_DIR_Output)
 * @param input Connect or disconnect input buffer (GPIO_PIN_CNF_INPUT_Connect or GPIO_PIN_CNF_INPUT_Disconnect)
 * @param pull  Pull configuration (GPIO_PIN_CNF_PULL_Disabled, GPIO_PIN_CNF_PULL_Pulldown or GPIO_PIN_CNF_PULL_Pullup)
 * @param drive Drive configuration (GPIO_PIN_CNF_DRIVE_x0x1 where x can be S-standard, D-disconnect and H-high)
 * @param sense Pin sensing mechanism (GPIO_PIN_CNF_SENSE_Disabled, GPIO_PIN_CNF_SENSE_High or GPIO_PIN_CNF_SENSE_Low)
 */
static inline void hal_gpio_cfg(  uint32_t pin_num,
                                    uint32_t dir,
                                    uint32_t input,
                                    uint32_t pull,
                                    uint32_t drive,
                                    uint32_t sense);

/**
 * @brief Function for configuring the given GPIO pin number as output with a standard drive for 1 and 0 levels.
 * @param pin_num specifies the pin number (allowed values 0-31) to set as output
 * @param init_val specifies the output state of the pin configured
 * @note  Sense capability on the pin is disabled, and input is disconnected from the buffer as the pins are configured as output
 */
static inline void hal_gpio_cfg_output(uint32_t pin_num, uint32_t init_val);

/**
 * @brief Function for configuring the given GPIO pin number as input with the specified pull up-down or no pull.
 * @param pin_num Specifies the pin number (allowed values 0-31) to set as input
 * @param pull_config State of the pin's pull resistor (no pull, pulled down or pulled high).
 * @note  Sense capability on the pin is disabled, and input is connected to buffer so that the GPIO->IN register is readable
 */
static inline void hal_gpio_cfg_input(uint32_t pin_num, hal_gpio_pull_t pull_config);

/**
 * @brief Function for write a value a GPIO pin configured as output.
 * @param pin_num specifies the pin number [0:31] to set.
 * @param val specified the value to be written to the pin (1 or 0)
 */
static inline void hal_gpio_pin_write(uint32_t pin_num, uint32_t val);

/**
 * @brief Function for setting a GPIO pin's output to 1.
 * @param pin_num specifies the pin number [0:31] to set.
 */
static inline void hal_gpio_pin_set(uint32_t pin_num);

/**
 * @brief Function for clearing a GPIO pin's output to 0.
 * @param pin_num specifies the pin number [0:31] to clear.
 */
static inline void hal_gpio_pin_clear(uint32_t pin_num);

/**
 * @brief Function for toggling the output of a GPIO pin.
 * @param pin_num specifies the pin number [0:31] to toggle.
 */
static inline void hal_gpio_pin_toggle(uint32_t pin_num);

/**
 * @brief Function for reading the input level of a GPIO pin.
 * @note the pin must be configured as input
 * @param pin_num specifies the pin number (0-31) to read.
 * @return 0 if the pin input level is low, 1 if the pin input level is high.
 */
static inline uint32_t hal_gpio_pin_read(uint32_t pin_num);

static inline void hal_gpio_cfg(uint32_t pin_num, uint32_t dir, uint32_t input, uint32_t pull,
        uint32_t drive, uint32_t sense)
{
    NRF_GPIO->PIN_CNF[pin_num] = (dir << GPIO_PIN_CNF_DIR_Pos) | (input << GPIO_PIN_CNF_INPUT_Pos)
            | (pull << GPIO_PIN_CNF_PULL_Pos) | (drive << GPIO_PIN_CNF_DRIVE_Pos)
            | (sense << GPIO_PIN_CNF_SENSE_Pos);
}

static inline void hal_gpio_cfg_output(uint32_t pin_num, uint32_t init_val)
{
    hal_gpio_cfg(pin_num,
        GPIO_PIN_CNF_DIR_Input,
        GPIO_PIN_CNF_INPUT_Disconnect,
        GPIO_PIN_CNF_PULL_Disabled,
        GPIO_PIN_CNF_DRIVE_S0S1,
        GPIO_PIN_CNF_SENSE_Disabled);
    hal_gpio_pin_write(pin_num, init_val);
    hal_gpio_cfg(pin_num,
        GPIO_PIN_CNF_DIR_Output,
        GPIO_PIN_CNF_INPUT_Disconnect,
        GPIO_PIN_CNF_PULL_Disabled,
        GPIO_PIN_CNF_DRIVE_S0S1,
        GPIO_PIN_CNF_SENSE_Disabled);
}

static inline void hal_gpio_cfg_input(uint32_t pin_num, hal_gpio_pull_t pull_config)
{
    hal_gpio_cfg(pin_num,
        GPIO_PIN_CNF_DIR_Input,
        GPIO_PIN_CNF_INPUT_Connect, pull_config,
        GPIO_PIN_CNF_DRIVE_S0S1,
        GPIO_PIN_CNF_SENSE_Disabled);
}

static inline void hal_gpio_pin_write(uint32_t pin_num, uint32_t val)
{
    if (val)
    {
        hal_gpio_pin_set(pin_num);
    }
    else
    {
        hal_gpio_pin_clear(pin_num);
    }
}

static inline void hal_gpio_pin_set(uint32_t pin_num)
{
    NRF_GPIO->OUTSET = (1UL << pin_num);
}

static inline void hal_gpio_pin_clear(uint32_t pin_num)
{
    NRF_GPIO->OUTCLR = (1UL << pin_num);
}

static inline void hal_gpio_pin_toggle(uint32_t pin_num)
{
    const uint32_t pin_bit = 1UL << pin_num;
    const uint32_t pin_state = (NRF_GPIO->OUT & pin_bit);

    if (pin_state == 0)
    {
        // Low to high
        NRF_GPIO->OUTSET = pin_bit;
    }
    else
    {
        // High to low
        NRF_GPIO->OUTCLR = pin_bit;
    }
}

static inline uint32_t hal_gpio_pin_read(uint32_t pin_num)
{
    return ((NRF_GPIO->IN >> pin_num) & 1UL);
}

#endif /* CODEBASE_HAL_HAL_GPIO_H_ */

/**
 * @}
 * @}
 */
