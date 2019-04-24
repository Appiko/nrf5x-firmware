/**
 *  tssp_ir_tx.h : IR TX module compatible with TSSP sensors
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

#ifndef TSSP_IR_TX_H
#define TSSP_IR_TX_H

#include "nrf.h"
#include "stdint.h"

/**
 * @brief Defines for the frequency at which the timer should run
 *  for the PWM generation
 */
typedef enum
{
    TSSP_IR_TX_TIMER_FREQ_16MHz = 0, ///< PWM timer frequency of 16 MHz.
    TSSP_IR_TX_TIMER_FREQ_8MHz,      ///< PWM timer frequency of 8 MHz.
    TSSP_IR_TX_TIMER_FREQ_4MHz,      ///< PWM timer frequency of 4 MHz.
    TSSP_IR_TX_TIMER_FREQ_2MHz,      ///< PWM timer frequency of 2 MHz.
    TSSP_IR_TX_TIMER_FREQ_1MHz,      ///< PWM timer frequency of 1 MHz.
    TSSP_IR_TX_TIMER_FREQ_500kHz,    ///< PWM timer frequency of 500 kHz.
    TSSP_IR_TX_TIMER_FREQ_250kHz,    ///< PWM timer frequency of 250 kHz.
    TSSP_IR_TX_TIMER_FREQ_125kHz,    ///< PWM timer frequency of 125 kHz.
    TSSP_IR_TX_TIMER_FREQ_62500Hz,   ///< PWM timer frequency of 62500 Hz.
    TSSP_IR_TX_TIMER_FREQ_31250Hz    ///< PWM timer frequency of 31250 Hz.
} tssp_ir_tx_timer_freq_t;

#if SYS_CFG_PRESENT == 1
#include "sys_config.h"
#endif

/** PPI Channel which is being used by this module */
#ifndef PPI_CH_USED_TSSP_IR_TX_1 
#define PPI_CH_USED_TSSP_IR_TX_1 9
#endif

#ifndef PPI_CH_USED_TSSP_IR_TX_2 
#define PPI_CH_USED_TSSP_IR_TX_2 10
#endif

#ifndef PPI_CH_USED_TSSP_IR_TX_3 
#define PPI_CH_USED_TSSP_IR_TX_3 0
#endif

#ifndef PPI_CH_USED_TSSP_IR_TX_4 
#define PPI_CH_USED_TSSP_IR_TX_4 1
#endif

/** Timer peripheral used by this module */
#ifndef TIMER_USED_TSSP_IR_TX_1 
#define TIMER_USED_TSSP_IR_TX_1 2
#endif

#ifndef TIMER_USED_TSSP_IR_TX_2 
#define TIMER_USED_TSSP_IR_TX_2 1
#endif

/** TIMER channel used by this module */
#ifndef TIMER_CHANNEL_USED_TSSP_IR_TX_1_1 
#define TIMER_CHANNEL_USED_TSSP_IR_TX_1_1 0
#endif

#ifndef TIMER_CHANNEL_USED_TSSP_IR_TX_2_1 
#define TIMER_CHANNEL_USED_TSSP_IR_TX_2_1 0
#endif

#ifndef TIMER_CHANNEL_USED_TSSP_IR_TX_2_2 
#define TIMER_CHANNEL_USED_TSSP_IR_TX_2_2 3
#endif

#ifndef GPIOTE_CH_USED_TSSP_IR_TX_1 
#define GPIOTE_CH_USED_TSSP_IR_TX_1 6
#endif

#ifndef GPIOTE_CH_USED_TSSP_IR_TX_2 
#define GPIOTE_CH_USED_TSSP_IR_TX_2 0
#endif


#ifndef TSSP_IR_TX_ON_TIME_MS
#define TSSP_IR_TX_ON_TIME_MS 1
#endif
/**
 * @brief Function to initiate the IR transmitter compatible with TSSP receiver.
 * @param tssp_tx_en Enable pin for IR transmitter circuitry. 
 * @param tssp_tx_in Pin over which PWM signal has to be sent.
 */
void tssp_ir_tx_init (uint32_t tssp_tx_en, uint32_t tssp_tx_in);

/**
 * @brief Function to start transmission.
 */
void tssp_ir_tx_start (void);

/**
 * @brief Function to stop transmission.
 */
void tssp_ir_tx_stop (void);

#endif /* TSSP_IR_TX_H */
