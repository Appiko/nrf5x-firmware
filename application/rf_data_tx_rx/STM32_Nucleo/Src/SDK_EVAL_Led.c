/**
 * @file    SDK_EVAL_Led.c
 * @author  VMA division - AMS
 * @version 3.2.0
 * @date    May 1, 2016
 * @brief   This file provides all the low level API to manage SDK LEDs.
 * @details
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


/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Led.h"
#include "SDK_EVAL_Config.h"

/** @addtogroup SDK_EVAL_NUCLEO
 * @{
 */


/** @addtogroup SDK_EVAL_Led
 * @{
 */

/** @defgroup SDK_EVAL_Led_Private_TypesDefinitions             SDK EVAL Led Private Types Definitions
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Private_Defines                      SDK EVAL Led Private Defines
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Private_Macros                       SDK EVAL Led Private Macros
 * @{
 */

/**
 * @}
 */


/** @defgroup SDK_EVAL_Led_Private_Variables                    SDK EVAL Led Private Variables
 * @{
 */



/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_Led_Private_FunctionPrototypes                    SDK EVAL Led Private Function Prototypes
 * @{
 */

/**
 * @}
 */


/**
 * @defgroup SDK_EVAL_Led_Private_Functions                             SDK EVAL Led Private Functions
 * @{
 */


/**
 * @brief  Configures LED GPIO.
 * @param  xLed Specifies the Led to be configured.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 * @retval None.
 */
void SdkEvalLedInit(SdkEvalLed xLed)
{
  NUCLEO_LED1_GPIO_CLK();
  GPIO_InitTypeDef  GPIO_InitStructure;
  
  /* Configure the GPIO_LED pin */
  GPIO_InitStructure.Pin = NUCLEO_LED1_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_MEDIUM;
  HAL_GPIO_Init(NUCLEO_LED1_GPIO_PORT, &GPIO_InitStructure);
  
  HAL_GPIO_WritePin(NUCLEO_LED1_GPIO_PORT, NUCLEO_LED1_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Turns selected LED On.
 * @param  xLed Specifies the Led to be set on.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 * @retval None.
 */
void SdkEvalLedOn(SdkEvalLed xLed)
{
  
  HAL_GPIO_WritePin(NUCLEO_LED1_GPIO_PORT, NUCLEO_LED1_PIN, GPIO_PIN_SET);
}

/**
 * @brief  Turns selected LED Off.
 * @param  xLed Specifies the Led to be set off.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 * @retval None.
 */
void SdkEvalLedOff(SdkEvalLed xLed)
{
  HAL_GPIO_WritePin(NUCLEO_LED1_GPIO_PORT, NUCLEO_LED1_PIN, GPIO_PIN_RESET);
}

/**
 * @brief  Toggles the selected LED.
 * @param  xLed Specifies the Led to be toggled.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 * @retval None.
 */
void SdkEvalLedToggle(SdkEvalLed xLed)
{
  NUCLEO_LED1_GPIO_PORT->ODR ^= NUCLEO_LED1_PIN;
}

/**
 * @brief  Returns the status of a specified led.
 * @param  xLed Specifies the Led to be read.
 *         This parameter can be one of following parameters:
 *         @arg LED1
 * @retval FlagStatus return the status of the LED. This parameter can be:
 *         SET or RESET.
 */
FlagStatus SdkEvalLedGetState(SdkEvalLed xLed)
{
  if(NUCLEO_LED1_GPIO_PORT->IDR & NUCLEO_LED1_PIN)
    return RESET;
  else
    return SET;

}


/**
 * @}
 */

/**
 * @}
 */

/**
 * @}
 */



/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
