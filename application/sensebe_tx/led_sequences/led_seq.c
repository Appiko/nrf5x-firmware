/**
 * @file led_seq.c Contains the PWM values for the one or more color LED(s)
 *                 for the different sequences 
 *
 * Automagically created on: 06-02-2019 at 17:13 */

#include "led_seq.h"
#include "boards.h"

#define LED_STRUCT1(name, count)       const struct {            \
                                        uint16_t red[count];    \
                                      } __attribute__((packed)) name

#define LED_STRUCT2(name, count)       const struct {            \
                                        uint16_t red[count];    \
                                        uint16_t green[count];   \
                                      } __attribute__((packed)) name

LED_STRUCT2(green_pulse_seq, 7) = {
    {0, 0, 0, 0, 0, 0, 0},
    {0, 1000, 0, 1000, 0, 1000, 0}
};
const uint16_t green_pulse_duration[] = { 0, 100, 100, 100, 100, 100, 100, 600 };

LED_STRUCT2(detect_sync_seq, 4) = {
    {0, 0, 0, 0},
    {0, 1000, 1000, 0}
};
const uint16_t detect_sync_duration[] = { 0, 250, 500, 250, 1000 };

LED_STRUCT1(detect_pulse_seq, 4) = {
    {0, 1000, 1000, 0}
};
const uint16_t detect_pulse_duration[] = { 0, 250, 500, 250, 1000 };

LED_STRUCT2(ramp_offset_seq, 5) = {
    {0, 0, 1000, 1000, 0},
    {0, 1000, 1000, 0, 0}
};
const uint16_t ramp_offset_duration[] = { 0, 2000, 2000, 2000, 2000, 8000 };

LED_STRUCT1(red_pulse_seq, 7) = {
    {0, 1000, 0, 1000, 0, 1000, 0}
};
const uint16_t red_pulse_duration[] = { 0, 100, 100, 100, 100, 100, 100, 600 };

LED_STRUCT2(orange_wave_seq, 3) = {
    {0, 1000, 0},
    {0, 1000, 0}
};
const uint16_t orange_wave_duration[] = { 0, 2500, 2500, 5000 };

LED_STRUCT2(orange_pulse_seq, 7) = {
    {0, 1000, 0, 1000, 0, 1000, 0},
    {0, 1000, 0, 1000, 0, 1000, 0}
};
const uint16_t orange_pulse_duration[] = { 0, 100, 100, 100, 100, 100, 100, 600 };

LED_STRUCT1(detect_window_seq, 2) = {
    {0, 0}
};
const uint16_t detect_window_duration[] = { 0, 500, 500 };

LED_STRUCT2(dual_freq_seq, 5) = {
    {0, 1000, 0, 1000, 0},
    {0, 500, 1000, 500, 0}
};
const uint16_t dual_freq_duration[] = { 0, 2000, 2000, 2000, 2000, 8000 };

LED_STRUCT2(green_wave_seq, 5) = {
    {0, 0, 0, 0, 0},
    {0, 1000, 200, 0, 0}
};
const uint16_t green_wave_duration[] = { 0, 1000, 1000, 2000, 1000, 5000 };

const uint32_t led_num_len[] = {2, 2, 1, 2, 1, 2, 2, 1, 2, 2};
const uint32_t led_seq_len[] = {7, 4, 4, 5, 7, 3, 7, 2, 5, 5};
const uint16_t * const led_seq_ptr[] = {(uint16_t *) &green_pulse_seq, (uint16_t *) &detect_sync_seq, (uint16_t *) &detect_pulse_seq, (uint16_t *) &ramp_offset_seq, (uint16_t *) &red_pulse_seq, (uint16_t *) &orange_wave_seq, (uint16_t *) &orange_pulse_seq, (uint16_t *) &detect_window_seq, (uint16_t *) &dual_freq_seq, (uint16_t *) &green_wave_seq };
const uint16_t * const led_seq_duration_ptr[] = { green_pulse_duration, detect_sync_duration, detect_pulse_duration, ramp_offset_duration, red_pulse_duration, orange_wave_duration, orange_pulse_duration, detect_window_duration, dual_freq_duration, green_wave_duration };

const uint32_t led_pin_num[LED_COLOR_MAX] = {LED_RED, LED_GREEN};

const uint32_t * const led_seq_get_pin_ptr(void)
{
    return led_pin_num;
}

uint32_t led_seq_get_pin_num(led_sequences seq)
{
    return led_num_len[seq];
}

uint32_t led_seq_get_seg_len(led_sequences seq)
{
    return led_seq_len[seq];
}

uint16_t * led_seq_get_seq_color_ptr(led_sequences seq, led_seq_color color)
{
    return ((uint16_t *)(led_seq_ptr[seq] + color*led_seq_len[seq]));
}

uint16_t * led_seq_get_seq_duration_ptr(led_sequences seq)
{
    return (uint16_t *) led_seq_duration_ptr[seq];
}

