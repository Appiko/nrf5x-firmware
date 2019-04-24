/**
 *  led_ui.h : LED UI Module
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
 * @defgroup group_led_ui Module that manages the LED UI
 *
 * @brief This module is responsible for managing the LED UI by playing and stopping
 *  the different UI sequences a single time or in a loop. This module gets the LED
 *  sequences from the led_seq module and plays through the hal_pwm module.
 *
 * @{
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_LED_UI_H_
#define CODEBASE_PERIPHERAL_MODULES_LED_UI_H_

#include "stdint.h"
#include "stdbool.h"
#include "led_seq.h"

/**@brief The LED sequence priority levels */
typedef enum {
  LED_UI_LOW_PRIORITY     = 0,//!< LED_UI_LOW_PRIORITY
  LED_UI_MID_PRIORITY     = 1,//!< LED_UI_MID_PRIORITY
  LED_UI_HIGH_PRIORITY    = 2,//!< LED_UI_HIGH_PRIORITY
} led_ui_priority_t;

/** @brief To specify if a sequence is run once or repeatedly
 */
typedef enum {
  LED_UI_LOOP_SEQ,   /// Run a sequence repeatedly
  LED_UI_SINGLE_SEQ, /// Run a sequence only once
  LED_UI_SEQ_T_SIZE  /// Only used to get num of elements in enum
}led_ui_seq_t;

/**
 * @brief Start a sequence to play once
 * @param seq The sequence to start
 * @param priority The priority of the sequence
 * @param reset When true restarts if the same sequence is already playing
 */
void led_ui_single_start(led_sequences seq,
    led_ui_priority_t priority, bool reset);

/**
 * @brief Start a sequence to play repeatedly
 * @param seq The sequence to start
 * @param priority The priority of the sequence
 */
void led_ui_loop_start(led_sequences seq,
    led_ui_priority_t priority);

/**
 * @brief Stop all sequence of a particular type
 * @param type The type to be stopped
 */
void led_ui_type_stop_all(led_ui_seq_t type);

/**
 * @brief Stops a particular sequence of a particular type
 * @param type The type to be stopped
 * @param seq The sequence to be stopped
 */
void led_ui_stop_seq(led_ui_seq_t type, led_sequences seq);

/**
 * @brief Stops LED sequences of a particular type and
 *  of a priority level or less
 * @param type The type to be stopped
 * @param priority The priority level or less to be stopped
 */
void led_ui_stop_priority(led_ui_seq_t type, uint32_t priority);

/**
 * @brief Stops all LED sequences running
 */
void led_ui_stop_everything(void);

/**
 * @brief Get the currently active LED UI sequence
 * @param type The type of sequence to return
 * @return The sequence that is active of @p type
 */
led_sequences led_ui_get_current_seq(led_ui_seq_t type);

#endif /* CODEBASE_PERIPHERAL_MODULES_LED_UI_H_ */

/**
 * @}
 * @}
 */
