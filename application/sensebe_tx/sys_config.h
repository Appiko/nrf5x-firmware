/*
 *  sys_config.h : List of all the peripheral devices used by SenseBe 
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
#define PPI_CH_USED_EXTRA 6
/** GPIOTE channel used for TSSP detect module */
#define GPIOTE_CH_USED_TSSP_DETECT 0
/** 1st GPIOTE channel used for TSSP IR transmission module */
#define GPIOTE_CH_USED_TSSP_IR_TX_1 1
/** 1st GPIOTE channel used for TSSP IR transmission module */
#define GPIOTE_CH_USED_TSSP_IR_TX_2 2
/** GPIOTE channel for future use */
#define GPIOTE_CH_USED_EXTRA 7
/** 1st Timer used for TSSP IR transmission module */
#define TIMER_USED_TSSP_IR_TX_1 2
/** 2nd Timer used for TSSP IR transmission module */
#define TIMER_USED_TSSP_IR_TX_2 1
/**Timer for future use  */
#define TIMER_USED_EXTRA 2
/** 1st Channel from 1st timer used for TSSP IR transmission module */
#define TIMER_CHANNEL_USED_TSSP_IR_TX_1_1 0
/** 1st Channel from 2nd timer used for TSSP IR transmission module */
#define TIMER_CHANNEL_USED_TSSP_IR_TX_2_1 0
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
