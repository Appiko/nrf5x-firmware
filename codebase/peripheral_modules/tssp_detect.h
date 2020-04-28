/**
 *  tssp_detect.h : TSSP detector driver
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

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#include "aux_clk.h"
#endif

#ifndef RTC_USED_TSSP_DETECT 
#define RTC_USED_TSSP_DETECT 0
#endif

#ifndef EGU_USED_TSSP_DETECT 
#define EGU_USED_TSSP_DETECT 0
#endif

#ifndef PPI_CH_USED_TSSP_DETECT_1 
#define PPI_CH_USED_TSSP_DETECT_1 7
#endif

#ifndef PPI_CH_USED_TSSP_DETECT_2 
#define PPI_CH_USED_TSSP_DETECT_2 8
#endif

#ifndef GPIOTE_CH_USED_TSSP_DETECT 
#define GPIOTE_CH_USED_TSSP_DETECT 7
#endif


#ifndef TSSP_DETECT_FREQ
#ifdef MS_TIMER_FREQ
#define TSSP_DETECT_FREQ MS_TIMER_FREQ
#else
#define TSSP_DETECT_FREQ 32768
#endif
#endif

/** Macro to find out the rounded number of TSSP_DETECT ticks for the passed time in milli-seconds */
#define TSSP_DETECT_TICKS_MS(ms)                ((uint32_t) ROUNDED_DIV( (TSSP_DETECT_FREQ*(uint64_t)(ms)) , 1000) )

/** List of Clock Sources that can be used to drive this module */
typedef enum
{
    /** Low freq clock : low power mode */
    TSSP_DETECT_LF_CLK = AUX_CLK_SRC_LFCLK,
    /** High freq clock : high power mode */
    TSSP_DETECT_HF_CLK = AUX_CLK_SRC_HFCLK,
}tssp_detect_clk_src_t;

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
    uint32_t window_duration_ms;

    /** Function pointer for a function which is to be called when no pulse is detected\ 
     *  for window duration */
    void (*tssp_missed_handler) (void);

    /** Function pointer for a function which is to be called when a pulse is detected */
    void (*tssp_detect_handler) (uint32_t ticks);
    
    /** Clock source to which module has to be initiated */
    tssp_detect_clk_src_t clk_src;

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

/**
 * @brief Function to Synchronize TSSP detector to IR transmitter which is being used
 * @param sync_ms Synchronization time in ms
 */
void tssp_detect_window_sync (uint32_t sync_ms);

/**
 * @brief Function to update the window duration for which beam has to be blocked
 * @param window_duration_ms New window duration for which beam has to be blocked
 */
void tssp_detect_update_window (uint32_t window_duration_ms);

/**
 * @brief Function to switch clock source
 * @param clk_src
 */
void tssp_detect_switch_clock (tssp_detect_clk_src_t clk_src);
#endif /* TSSP_DETECT_H */
/**
 * @}
 * @}
 */