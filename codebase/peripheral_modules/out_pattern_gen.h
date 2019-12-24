/**
 *  out_pattern_gen.h : Output Pattern Generator
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
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_out_pattern_gen Output pattern generator
 *
 * @brief Output pattern generator module is used for generating a one time digital
 *  signal pattern on a number of pins.
 *
 * @{
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_OUT_PATTERN_GEN_H_
#define CODEBASE_PERIPHERAL_MODULES_OUT_PATTERN_GEN_H_

#include "stdint.h"
#include "stdbool.h"

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif
#ifndef MS_TIMER_USED_OUT_GEN 
#define MS_TIMER_USED_OUT_GEN 1
#endif


/** The maximum number of transitions that can occur in the generated pattern */
#define OUT_GEN_MAX_TRANSITIONS 128

/** The maximum number of output pins for which pattern can be generated */
#define OUT_GEN_MAX_NUM_OUT     4

/** 
 * @brief Configuration structure for output pattern generation.
 */
typedef struct 
{
    /** @brief The number of transitions in this pattern. */
    uint32_t num_transitions;
    /** @brief 1 dimensional array containing the durations in terms of LFCLK
        frequency ticks for the transitions. */
    uint32_t transitions_durations[OUT_GEN_MAX_TRANSITIONS];
    /** @brief A 2 dimensional boolean array containing the next digital output
        value for the various transitions across the initialized pins. */
    bool next_out[OUT_GEN_MAX_NUM_OUT][OUT_GEN_MAX_TRANSITIONS];
    /** @brief A pointer to a handler called after a pattern is generated */
    void (*done_handler)(uint32_t out_gen_state);
    /** @brief State which is to be started with this configuration,
         passed as an argument with the @p done_handler */
    uint32_t out_gen_state;
} out_gen_config_t;

/**
 * @brief Initialize the output pattern generator module with the information of
 *  the pins on which the pattern is generated
 * @param num_out The number of pins with pattern generation
 * @param out_pins Pointer to array with the pin numbers
 * @param out_init_value Pointer to an array containing the initial pin values
 */
void out_gen_init(uint32_t num_out, uint32_t * out_pins, bool * out_init_value);

/**
 * @brief Start the generation of the pattern with the information provided
 * @param out_gen_config A pointer to configuration which is used to generate
 * pattern.
 */
void out_gen_start(out_gen_config_t * out_gen_config);

/**
 * @brief Stop the output pattern generation and sets the output pins
 *  as specified in the arguments
 * @param out_vals Pointer to an array of digital values for the pins to be
 *  set to on stopping the pattern generation.
 */
void out_gen_stop(bool * out_vals);

/**
 * @brief To know if the output pattern generator module is on
 * @return True if pattern generation is on and false if off
 */
bool out_gen_is_on(void);

/**
 * @brief Function to get ticks since last call of @ref out_gen_start
 * @return Number of ms_timer ticks since last call of @ref out_gen_start
 */
uint32_t out_gen_get_ticks(void);

#endif /* CODEBASE_PERIPHERAL_MODULES_OUT_PATTERN_GEN_H_ */

/**
 * @}
 * @}
 */
