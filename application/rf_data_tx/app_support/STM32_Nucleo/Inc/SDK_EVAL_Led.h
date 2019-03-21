/**
 * @file    SDK_EVAL_Led.h
 * @author  AMG - RF Application team
 * @version 3.2.1
 * @date    26-September-2016
 * @brief   This file contains definitions for Software Development Kit eval board Leds.
 * @details
 *
 * In this module there are API for the management of the leds on the SDK Eval
 * motherboard.
 *
 * <b>Example:</b>
 * @code
 *
 *   SdkEvalLedInit(LED1);
 *
 *   ...
 *
 *   SdkEvalLedToggle(LED1);
 *
 *   ...
 *
 * @endcode
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
 * <h2><center>&copy; COPYRIGHT 2016 STMicroelectronics</center></h2>
 */

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __SDK_EVAL_LED_H
#define __SDK_EVAL_LED_H

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"

#ifdef __cplusplus
 extern "C" {
#endif


/** @addtogroup SDK_EVAL_NUCLEO
 * @{
 */

/** @addtogroup SDK_EVAL_Led            SDK EVAL Led
 * @brief Management of Software Development Kit eval board Leds.
 * @details See the file <i>@ref SDK_EVAL_Led.h</i> for more details.
 * @{
 */

/** @defgroup SDK_EVAL_Led_Exported_Types               SDK EVAL Led Exported Types
 * @{
 */

/**
 * @brief  Enumeration of SDK EVAL LEDs
 */
typedef enum
{
  LED1 = 0,

} SdkEvalLed;

#define LED2            LED1
#define LED3            LED1
#define LED4            LED1
#define LED5            LED1

#define GREEN LED1
#define ORANGE LED2
#define RED LED3
#define BLUE LED4
#define YELLOW LED5

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Exported_Constants                           SDK EVAL Led Exported Constants
 * @{
 */



/**
 * @}
 */

/**
 * @defgroup SDK_EVAL_Led_Exported_Macros                       SDK EVAL Led Exported Macros
 * @{
 */

/**
 * @}
 */

/** @defgroup SDK_EVAL_Led_Exported_Functions                   SDK EVAL Led Exported Functions
 * @{
 */

void SdkEvalLedInit(SdkEvalLed xLed);
void SdkEvalLedOn(SdkEvalLed xLed);
void SdkEvalLedOff(SdkEvalLed xLed);
void SdkEvalLedToggle(SdkEvalLed xLed);
FlagStatus SdkEvalLedGetState(SdkEvalLed xLed);

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

/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
