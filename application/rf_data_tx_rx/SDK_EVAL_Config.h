/**
 * @file    SDK_EVAL_Config.h
 * @author  VMA division - AMS
 * @version V2.0.2
 * @date    Febrary 7, 2015
 * @brief   This file contains SDK EVAL configuration and useful defines.
 * @details
 *
 * This file is used to include all or a part of the SDK Eval
 * libraries into the application program which will be used.
 *
 * THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
 * WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE
 * TIME. AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY
 * DIRECT, INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING
 * FROM THE CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE
 * CODING INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
 *
 * THIS SOURCE CODE IS PROTECTED BY A LICENSE.
 * FOR MORE INFORMATION PLEASE CAREFULLY READ THE LICENSE AGREEMENT FILE LOCATED
 * IN THE ROOT DIRECTORY OF THIS FIRMWARE PACKAGE.
 *
 * <h2><center>&copy; COPYRIGHT 2015 STMicroelectronics</center></h2>
 */


/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDK_EVAL_CONFIG_H
#define __SDK_EVAL_CONFIG_H

/* Includes ------------------------------------------------------------------*/

#ifdef __cplusplus
extern "C" {
#endif


/** @addtogroup SDK_EVAL_NUCLEO       SDK-EVAL NUCLEO
 * @brief This module is used to configure the SDK Eval board and
 * allows to manage its peripherals in a simple way.
  * @details The supported demoboard is <b><i>NUCLEO-L152RE</i></b> motherboard:
 * <ul> 
 * <li>STM32L152RE 64-pin microcontroller</li>
 * <li>Mini USB connector for power supply and I/O</li>
 * <li>ST-Link, JTAG/SWD on board debugger</li>
 * <li>User button and RESET button</li>
 * <li>1 User LED</li>
 * </ul>
 * @{
 */

/** @addtogroup SDK_EVAL_Config         SDK EVAL Config
 * @brief SDK EVAL configuration.
 * @details See the file <i>@ref SDK_EVAL_Config.h</i> for more details.
 * @{
 */

/** @defgroup SDK_EVAL_Config_Exported_Constants        SDK EVAL Config Exported Constants
 * @{
 */
 	
#define SDK_EVAL_SPI_PRESCALER	SPI_BAUDRATEPRESCALER_4
#define SDK_EVAL_UART_BAUDRATE  115200
/**
 * @}
 */


/** @defgroup SDK_EVAL_Config_Exported_Functions        SDK EVAL Config Exported Functions
 * @{
 */

void SdkEvalIdentification(void);
uint8_t SdkEvalGetVersion(void);
   
/**
 * @}
 */

/**
 * @}
 */


/**
 * @}
 */


#ifdef __cplusplus
}
#endif

#endif

/******************* (C) COPYRIGHT 2015 STMicroelectronics *****END OF FILE****/
