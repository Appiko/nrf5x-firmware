/* 
 * File:   tssp_ir_tx.h
 * Copyright (c) 2018 Appiko
 * Created on 5 February, 2019, 4:11 PM
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

#ifndef TSSP_IR_TX_H
#define TSSP_IR_TX_H

#include "simple_pwm.h"
#include "nrf.h"
#include "stdint.h"


#ifndef TSSP_IR_TX_ON_TIME_MS
#define TSSP_IR_TX_ON_TIME_MS 1
#endif

/** PPI Channel which is being used by this module */
#define TSSP_TX_PPI_CHANNEL_USED 9

/** Timer peripheral used by this module */
#define TSSP_TX_TIMER_USED 2

/** SIMPLE_PWM channel used by this module */
#define TSSP_TX_SIMPLE_PWM_CHANNEL_USED SIMPLE_PWM_CHANNEL0

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
