/* 
 * File:   sys_config.h
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
 * @addtogroup sensebe_appln
 * @{
 *
 * @defgroup sys_config The header file to handle resource allocation.
 * @brief The header file which contains system definitions for resources allocation.
 *
 * @{
 *
 */

/**
 * @brief File to keep track of all the peripheral devices used by this module.
 */
#ifndef SYS_CONFIG_H
#define SYS_CONFIG_H
/** RTC used for MS_TIMER module */
#define RTC_USED_MS_TIMER 1
/** RTC used for TSSP detect module */
#define RTC_USED_TSSP_DETECT 0
/** MS_TIMER used for Device Ticks module */
#define MS_TIMER_USED_DEVICE_TICKS 0
/** MS_TIMER used for Out pattern Gen module */
#define MS_TIMER_USED_OUT_GEN 1
/** MS_TIMER used for SenseBe TxRx module */
#define MS_TIMER_USED_SENSBE_TX_RX 2
/** 1st PPI channel used for TSSP detect module */
#define PPI_CH_USED_TSSP_DETECT_1 0
/** 2nd PPI channel used for TSSP detect module */
#define PPI_CH_USED_TSSP_DETECT_2 1
/** 1st PPI channel used for TSSP IR transmission module */
#define PPI_CH_USED_TSSP_IR_TX_1 2
/** 2nd PPI channel used for TSSP IR transmission module */
#define PPI_CH_USED_TSSP_IR_TX_2 3
/** 3rd PPI channel used for TSSP IR transmission module */
#define PPI_CH_USED_TSSP_IR_TX_3 4
/** 4th PPI channel used for TSSP IR transmission module */
#define PPI_CH_USED_TSSP_IR_TX_4 5
/** PPI channel for future use */
#define PPI_CH_USED_RANDOM 6
/** GPIOTE channel used for TSSP detect module */
#define GPIOTE_CH_USED_TSSP_DETECT 0
/** 1st GPIOTE channel used for TSSP IR transmission module */
#define GPIOTE_CH_USED_TSSP_IR_TX_1 1
/** 1st GPIOTE channel used for TSSP IR transmission module */
#define GPIOTE_CH_USED_TSSP_IR_TX_2 2
/** GPIOTE channel for future use */
#define GPIOTE_CH_USED_RANDOM 7
/** 1st Timer used for TSSP IR transmission module */
#define TIMER_USED_TSSP_IR_TX_1 0
/** 2nd Timer used for TSSP IR transmission module */
#define TIMER_USED_TSSP_IR_TX_2 1
/**Timer for future use  */
#define TIMER_USED_RANDOM 2
/** 1st Channel from 1st timer used for TSSP IR transmission module */
#define TIMER_CHANNEL_USED_TSSP_IR_TX_1_1 0
/** 1st Channel from 2nd timer used for TSSP IR transmission module */
#define TIMER_CHANNEL_USED_TSSP_IR_TX_2_1 1
/** 2nd Channel from 2nd timer used for TSSP IR transmission module */
#define TIMER_CHANNEL_USED_TSSP_IR_TX_2_2 3
/** Event Generator Unit used for TSSP detect module */
#define EGU_USED_TSSP_DETECT 0
/** SAADC channel used for Simple ADC module */
#define SAADC_CHANNEL_USED_SIMPLE_ADC 1

#endif /* SYS_CONFIG_H */
/**
 * @}
 * @}
 */
