/**
* @file    SDK_EVAL_Button.c
* @author  VMA division - AMS
* @version 3.2.0
* @date    May 1, 2016
* @brief   This file provides all the low level API to manage SDK buttons.
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
#include "SDK_EVAL_Button.h"
#include "SDK_EVAL_Config.h"


/** @addtogroup SDK_EVAL_NUCLEO
* @{
*/


/** @addtogroup SDK_EVAL_Button
* @{
*/

/** @defgroup SDK_EVAL_Button_Private_TypesDefinitions          SDK EVAL Button Private Types Definitions
* @{
*/

/**
* @}
*/


/** @defgroup SDK_EVAL_Button_Private_Defines                   SDK EVAL Button Private Defines
* @{
*/

/**
* @}
*/


/** @defgroup SDK_EVAL_Button_Private_Macros                    SDK EVAL Button Private Macros
* @{
*/

/**
* @}
*/





/** @defgroup SDK_EVAL_Button_Private_FunctionPrototypes                        SDK EVAL Button Private Function Prototypes
* @{
*/

/**
* @}
*/

/** @defgroup SDK_EVAL_Button_Private_Functions                                 SDK EVAL Button Private Functions
* @{
*/


/**
* @brief  Configures Button GPIO and EXTI Line.
* @param  xButton Specifies the Button to be configured.
*         This parameter can be one of following parameters:
*         @arg BUTTON_SCM_PS: SCM Push Button
*         @arg BUTTON_KEY: Key Push Button
*         @arg BUTTON_RIGHT: Joystick Right Push Button
*         @arg BUTTON_LEFT: Joystick Left Push Button
*         @arg BUTTON_UP: Joystick Up Push Button
*         @arg BUTTON_DOWN: Joystick Down Push Button
*         @arg BUTTON_SEL: Joystick Sel Push Button
* @param  xButtonMode Specifies Button mode.
*         This parameter can be one of following parameters:
*         @arg BUTTON_MODE_GPIO: Button will be used as simple IO
*         @arg BUTTON_MODE_EXTI: Button will be connected to EXTI line with interrupt
*         generation capability
* @retval None.
*/ 
void SdkEvalPushButtonInit(SdkEvalButton xButton, SdkEvalButtonMode xButtonMode)
{  
  BUTTON1_GPIO_CLK();
     
  GPIO_InitTypeDef GPIO_InitStructure;
  GPIO_InitTypeDef EXTI_InitStructure;
  
  /* Configures Button pin as input */
  GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Pin = BUTTON1_PIN;
  HAL_GPIO_Init(BUTTON1_GPIO_PORT, &GPIO_InitStructure);
  
  if (xButtonMode == BUTTON_MODE_EXTI)
  {
    /* Configures Button EXTI line */
    EXTI_InitStructure.Mode = GPIO_MODE_IT_FALLING;
    EXTI_InitStructure.Pull = GPIO_NOPULL;
    EXTI_InitStructure.Pin = BUTTON1_PIN;
    HAL_GPIO_Init(BUTTON1_GPIO_PORT, &EXTI_InitStructure);
    
    HAL_NVIC_SetPriority(BUTTON1_EXTI_IRQn, BUTTON1_IRQ_PREEMPTION_PRIORITY, BUTTON1_IRQ_SUB_PRIORITY);
    HAL_NVIC_EnableIRQ(BUTTON1_EXTI_IRQn);
  }
}

/**
* @brief  Returns the selected Button state.
* @param  xButton Specifies the Button to be checked.
*         This parameter can be one of following parameters:
*         @arg BUTTON_WAKEUP: Wakeup Push Button
*         @arg BUTTON_TAMPER: Tamper Push Button
*         @arg BUTTON_KEY: Key Push Button
*         @arg BUTTON_RIGHT: Joystick Right Push Button
*         @arg BUTTON_LEFT: Joystick Left Push Button
*         @arg BUTTON_UP: Joystick Up Push Button
*         @arg BUTTON_DOWN: Joystick Down Push Button
*         @arg BUTTON_SEL: Joystick Sel Push Button
* @retval FlagStatus The Button GPIO pin value.
*/
FlagStatus SdkEvalPushButtonGetState(SdkEvalButton xButton)
{
  GPIO_PinState ret = HAL_GPIO_ReadPin(BUTTON1_GPIO_PORT, BUTTON1_PIN);
  return (FlagStatus)ret;    
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
