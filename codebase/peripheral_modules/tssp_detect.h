/* 
 * File:   tssp_detect.h
 * Copyright (c) 2018 Appiko
 * Created on 18 December, 2018, 3:27 PM
 * Author:  Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE
 */

/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_tssp_detect TSSP Detector Driver
 *
 * @brief Driver for TSSP IR beam detector.
 * @{
 */

#ifndef TSSP_DETECT_H
#define TSSP_DETECT_H

#include "stdint.h"
#include "stdbool.h"
#include "boards.h"

/** RTC used by this module */
#define TSSP_DETECT_RTC_USED NRF_RTC0

/** EGU channel used by this module */
#define TSSP_DETECT_EGU_USED NRF_EGU0

#ifndef TSSP_DETECT_FREQ
#ifdef MS_TIMER_FREQ
#define TSSP_DETECT_FREQ MS_TIMER_FREQ
#else
#define TSSP_DETECT_FREQ 32768
#endif
#endif

/**
 * @brief Structure to store information required to use this module.
 */
typedef struct 
{
    /** Pin number for enable pin */
    uint32_t rx_en_pin;

    /** Pin number for IR Rx detect pin */
    uint32_t rx_in_pin;

    /** Logic level when pulse is detected */
    bool detect_logic_level;

    /** Window duration in RTC ticks for which if no pulse is detected, some operation will done */
    uint32_t window_duration_ticks;

    /** Function pointer for a function which is to be called when no pulse is detected\ 
     *  for window duration */
    void (*tssp_missed_handler) (void);

    /** Function pointer for a function which is to be called when a pulse is detected */
    void (*tssp_detect_handler) (uint32_t ticks);

}tssp_detect_config_t;

/**
 * @brief Function to initialize IR detect sub-module
 * 
 * @param tssp_detect_config Configuration requtssped for initialization
 */
void tssp_detect_init (tssp_detect_config_t * tssp_detect_config);

/**
 * @brief Function to start IR pulse detection.
 * 
 */
void tssp_detect_window_detect (void);

/**
 * @brief Function to stop IR pulse detection
 * 
 */
void tssp_detect_pulse_stop (void);

/**
 * @brief Function to stop IR missed window detection
 */
void tssp_detect_window_stop (void);

/**
 * @brief Function to start module is pulse detecting mode.
 * 
 */
void tssp_detect_pulse_detect (void);

#endif /* TSSP_DETECT_H */
/**
 * @}
 * @}
 */