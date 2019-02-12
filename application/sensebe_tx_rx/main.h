/* 
 * File:   main.h
 * Copyright (c) 2018 Appiko
 * Created on 11 February, 2019, 4:24 PM
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
 * @brief File to keep track of all the peripheral devices used by this module.
 */
#ifndef MAIN_H
#define MAIN_H

#define RTC_USED_MS_TIMER 1

#define RTC_USED_TSSP_DETECT 0

#define MS_TIMER_USED_DEVICE_TICKS 0

#define MS_TIMER_USED_OUT_GEN 1

#define MS_TIMER_USED_SENSBE_TX_RX 2

#define PPI_CH_USED_TSSP_DETECT_1 0

#define PPI_CH_USED_TSSP_DETECT_2 1

#define PPI_CH_USED_TSSP_IR_TX_1 2

#define PPI_CH_USED_TSSP_IR_TX_2 3

#define PPI_CH_USED_TSSP_IR_TX_3 4

#define PPI_CH_USED_TSSP_IR_TX_4 5

#define PPI_CH_USED_RANDOM 6

#define GPIOTE_CH_USED_TSSP_DETECT 0

#define GPIOTE_CH_USED_TSSP_IR_TX_1 1

#define GPIOTE_CH_USED_TSSP_IR_TX_2 2

#define GPIOTE_CH_USED_RANDOM 7

#define TIMER_USED_TSSP_IR_TX_1 0

#define TIMER_USED_TSSP_IR_TX_2 1

#define TIMER_USED_RANDOM 2

#define TIMER_CHANNEL_USED_TSSP_IR_TX_1_1 0

#define TIMER_CHANNEL_USED_TSSP_IR_TX_2_1 1

#define TIMER_CHANNEL_USED_TSSP_IR_TX_2_2 3

#define EGU_USED_TSSP_DETECT 0

#define SAADC_CHANNEL_USED_SIMPLE_ADC 1

#endif /* MAIN_H */

