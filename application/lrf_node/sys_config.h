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

/** Complete 128 bit UUID of the SenseBe service
 * 3c73dc60-07f5-480d-b066-837407fbde0a */
#define DEVICE_UUID_COMPLETE        0x0a, 0xde, 0xfb, 0x07, 0x74, 0x83, 0x66, 0xb0, 0x0d, 0x48, 0xf5, 0x07, 0x60, 0xdc, 0x73, 0x3c
/** The 16 bit UUID of the Sense Be service */
#define DEVICE_UUID_SERVICE         0xdc60
/** The 16 bit UUID of the read-only System Info characteristic */
#define DEVICE_UUID_SYSINFO         0xdc61
/** The 16 bit UUID of the read-write Config characteristic */
#define DEVICE_UUID_PRODUCT_INFO          0xdc62
/** The 16 bit UUID of the read-write Config characteristic */
#define DEVICE_UUID_DPLOYMENT_FLAG        0xdc63
/** The 16 bit UUID of the read-write Config characteristic */
#define DEVICE_UUID_DFU_FLAG              0xdc64
/** The 16 bit UUID of the read-write Config characteristic */
#define DEVICE_UUID_ALIGN_FLAG            0xdc65

/** PWM peripheral used for hal driver */
#define HAL_PWM_PERIPH_USED 0
/** SPIM peripheral used for hal driver */
#define HAL_SPIM_PERIPH_USED 0
/** TWIM peripheral used for hal driver */
#define HAL_TWIM_PERIPH_USED 0
/** UART peripheral used for hal driver */
#define HAL_UART_PERIPH_USED 0

/** RTC used for MS_TIMER module */
#define RTC_USED_MS_TIMER 1
/** MS_TIMER used for Device Ticks module */
#define MS_TIMER_USED_DEVICE_TICKS 0
/** MS_TIMER used for main application */
#define MS_TIMER_USED_MAIN 1

/** GPIOTE PORT channel used for button_ui */
#define GPIOTE_CH_USED_BUTTON_UI_PORT 
/** 0th GPIOTE channel used for RF communication */
#define GPIOTE_CH_USED_RF_COMM_0 0 
/** 1st GPIOTE channel used for RF communication */
#define GPIOTE_CH_USED_RF_COMM_1 1 
/** 2nd GPIOTE channel used for RF communication */
#define GPIOTE_CH_USED_RF_COMM_2 2  
/** 3rd GPIOTE channel used for RF communication */
#define GPIOTE_CH_USED_RF_COMM_3 3 
/** GPIOTE channel for future use */
#define GPIOTE_CH_USED_EXTRA 7
/** SAADC channel used for Simple ADC module */
#define SAADC_CHANNEL_USED_SIMPLE_ADC 1
/** SWI peripheral used by application */
#define SWI_LRF_NODE_BLE_USED 1
/** SWI used for Evt SD Handler module */
#define SWI_USED_EVT_SD_HANDLER 2
#endif /* SYS_CONFIG_H */
/**
 * @}
 * @}
 */
