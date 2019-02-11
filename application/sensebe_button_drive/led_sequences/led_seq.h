/**
 * @file led_seq.h Header to access the PWM values for the
 *                 one or more color LED(s) for the different sequences 
 *
 * Automagically created on: 29-01-2019 at 18:07
 */

#ifndef _LED_SEQ_H_
#define _LED_SEQ_H_

#include <stdint.h>

/** Specify the different LED patterns possible */
typedef enum {
  LED_SEQ_GREEN_PULSE = 0,
  LED_SEQ_DETECT_PULSE = 1,
  LED_SEQ_RAMP_OFFSET = 2,
  LED_SEQ_RED_PULSE = 3,
  LED_SEQ_ORANGE_WAVE = 4,
  LED_SEQ_ORANGE_PULSE = 5,
  LED_SEQ_DETECT_WINDOW = 6,
  LED_SEQ_DUAL_FREQ = 7,
  LED_SEQ_GREEN_WAVE = 8,
  LED_SEQ_NULL = 255,
} led_sequences;

/** @brief The different color LEDs present in this instance */
typedef enum {
  LED_COLOR_RED,
  LED_COLOR_GREEN,
  /// To specify the number of LEDs presents
  LED_COLOR_MAX
}led_seq_color;

/**
 * @brief Gets the pointer to the array containing LED pin numbers
 *  with a length of @ref LED_COLOR_MAX
 * @return Constant pointer to a const array for reason mention in brief
 */
const uint32_t * const led_seq_get_pin_ptr(void);

/**
 * @brief Gets the number of LEDs used in the sequence
 * @param seq The sequence to query the number of LEDs
 * @return Returns the number of LEDs in the sequence
 */
uint32_t led_seq_get_pin_num(led_sequences seq);

/**
 * @brief Get the number of segments in the sequence
 * @param seq The sequence whose number of segments is required
 * @return The number of segments in the parameter's sequence
 */
uint32_t led_seq_get_seg_len(led_sequences seq);

/**
 * @brief Gets the pointer to the start of values for a color for a sequence
 * @param seq The sequence whose pointer is required
 * @param color The color in question
 * @return Pointer to the start of a color in a sequence
 */
uint16_t * led_seq_get_seq_color_ptr(led_sequences seq, led_seq_color color);

/**
 * @brief Gets the pointer to the start of segment duration array for a sequence
 * @param seq The sequence whose pointer is required
 * @return Pointer to the start of duration in a sequence
 */
uint16_t * led_seq_get_seq_duration_ptr(led_sequences seq);

#endif /* _LED_SEQ_H_ */
