/*
 *  sys_config.h : <Write brief>
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
 * @addtogroup sensepi_appln
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
#define DEVICE_UUID_COMPLETE        0x0a, 0xde, 0xfb, 0x07, 0x74, 0x83, 0x66, 0xb0, 0x0d, 0x48, 0xf5, 0x07, 0x10, 0xdc, 0x73, 0x3c
/** The 16 bit UUID of the Sense Be service */
#define DEVICE_UUID_SERVICE         0xdc10
/** The 16 bit UUID of the read-only System Info characteristic */
#define DEVICE_UUID_SYSINFO         0xdc11
/** The 16 bit UUID of the read-write Config characteristic */
#define DEVICE_UUID_CONFIG          0xdc12

/** PWM peripheral used for hal driver */
#define HAL_PWM_PERIPH_USED 0
/** SPIM peripheral used for hal driver */
#define HAL_SPIM_PERIPH_USED 0
/** TWIM peripheral used for hal driver */
#define HAL_TWIM_PERIPH_USED 0
/** UART peripheral used for hal driver */
#define HAL_UART_PERIPH_USED 0
/** Radio peripheral used for hal_driver */
#define HAL_RADIO_PERIPH_USED 0

/** GPIOTE PORT channel used for button_ui */
#define GPIOTE_CH_USED_BUTTON_UI_PORT 
/** RTC used for MS_TIMER module */
#define RTC_USED_MS_TIMER 1
/** RTC used for TSSP detect module */
#define RTC_USED_AUX_CLK 0
/** MS_TIMER used for Device Ticks module */
#define MS_TIMER_USED_DEVICE_TICKS 0
/** MS_TIMER used for Out pattern Gen module */
#define MS_TIMER_USED_OUT_GEN 1
/** MS_TIMER used for SenseBe TxRx module */
#define MS_TIMER_USED_SENSEPI 2

#define MS_TIMER_USED_SENSEPI_RADIO_CONTROL 3
/** Number of PPI channels used for AUX clock module */
#define PPI_CHANNELS_USED_AUX_CLK 3
/** Base number for PPI channels in AUX clock module */
#define PPI_CHANNEL_BASE_AUX_CLK 0
/** 1st PPI channel used for AUX clock module */
#define PPI_CHANNEL_USED_AUX_CLK_0  (PPI_CHANNEL_BASE_AUX_CLK + 0)
/** 2nd PPI channel used for AUX clock module */
#define PPI_CHANNEL_USED_AUX_CLK_1  (PPI_CHANNEL_BASE_AUX_CLK + 1)
/** 3rd PPI channel used for AUX clock module */
#define PPI_CHANNEL_USED_AUX_CLK_2  (PPI_CHANNEL_BASE_AUX_CLK + 2)
/** 1st PPI channel used by PIR Sense module */
#define PPI_CHANNEL_USED_PIR_SENSE_1 3
/** 2nd PPI channel used by PIR Sense module */
#define PPI_CHANNEL_USED_PIR_SENSE_2 PPI_CHANNEL_USED_AUX_CLK_0
/** 3rd PPI channel used by PIR Sense module */
#define PPI_CHANNEL_USED_PIR_SENSE_3 4
/** Timer used by radio trigger module */
#define TIMER_USED_AUX_CLK 2
/** Timer used by radio trigger module */
#define TIMER_USED_RADIO_TRIGGER 0
/** 1st timer channel used for radio trigger module */
#define TIMER_CHANNEL_USED_RADIO_TRIGGER_0 0 
/** 2nd timer channel used for radio trigger module */
#define TIMER_CHANNEL_USED_RADIO_TRIGGER_1 1
/** 3rd timer channel used for radio trigger module */
#define TIMER_CHANNEL_USED_RADIO_TRIGGER_2 2
/** SAADC channel used for Simple ADC module */
#define SAADC_CHANNEL_USED_SIMPLE_ADC 1
/** SAADC channel used for PIR Sense module */
#define SAADC_CHANNEL_USED_PIR_SENSE 0
/** SWI peripheral used by application */
#define SWI_USED_SENSEPI_BLE 1
/** SWI used for Evt SD Handler module */
#define SWI_USED_EVT_SD_HANDLER 2


#endif /* SYS_CONFIG_H */
/**
 * @}
 * @}
 */

