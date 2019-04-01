/**
* @file    SDK_EVAL_Button.h
* @author  AMG - RF Application team
* @version 3.2.1
* @date    26-September-2016
* @brief   This file contains definitions for Software Development Kit eval board push-buttons.
* @details
*
* This module exports functions used to configure and manage the SDK motherboard
* push-buttons.
*
* <b>Example:</b>
* @code
*
*   SdkEvalPushButtonInit(BUTTON_KEY, BUTTON_MODE_EXTI);
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
#ifndef __SDK_EVAL_BUTTON_H
#define __SDK_EVAL_BUTTON_H

/* Includes ------------------------------------------------------------------*/
#include "cube_hal.h"

#ifdef __cplusplus
extern "C" {
#endif
  
  
/** @addtogroup SDK_EVAL_NUCLEO
* @{
*/

/** @addtogroup SDK_EVAL_Button         SDK EVAL Button
* @brief Management of Software Development Kit eval board push-buttons.
* @details See the file <i>@ref SDK_EVAL_Button.h</i> for more details.
* @{
*/

/** @addtogroup SDK_EVAL_Button_Exported_Types          SDK EVAL Button Exported Types
* @{
*/

/**
* @brief  Buttons for SDK EVAL enumeration
*/
typedef enum
{
  BUTTON_1 = 0, /*!< BUTTON_1 */
    
} SdkEvalButton;

/**
* @brief All the old buttons are mapped in the BUTTON_1 (only one in the Nucleo Board)
*/
#define BUTTON_2        BUTTON_1
#define BUTTON_3        BUTTON_1
#define BUTTON_4        BUTTON_1
#define BUTTON_5        BUTTON_1
#define BUTTON_6        BUTTON_1
#define BUTTON_7        BUTTON_1

/**
* @brief All the old buttons are mapped in the BUTTON_1 (only one in the Nucleo Board)
*/
#define BUTTON_KEY			 BUTTON_1
#define BUTTON_RIGHT			 BUTTON_2
#define BUTTON_LEFT			 BUTTON_3
#define BUTTON_UP			 BUTTON_4
#define BUTTON_DOWN			 BUTTON_5
#define BUTTON_SEL			 BUTTON_6
#define BUTTON_SCM_PS			 BUTTON_7


/**
* @brief  Button Mode for SDK EVAL enumeration
*/
typedef enum
{
  BUTTON_MODE_GPIO = 0,
  BUTTON_MODE_EXTI = 1
    
} SdkEvalButtonMode;

/**
* @brief  Joystick for SDK EVAL enumeration
*/
typedef enum
{
  JOY_NONE = 0,
  JOY_SEL = 1,
  JOY_DOWN = 2,
  JOY_LEFT = 3,
  JOY_RIGHT = 4,
  JOY_UP = 5
    
} SdkEvalJoyState;

/**
* @}
*/


/** @defgroup SDK_EVAL_Button_Exported_Constants                SDK EVAL Button Exported Constants
* @{
*/

  
/**
* @}
*/


/** @defgroup SDK_EVAL_Button_Exported_Macros           SDK EVAL Button Exported Macros
* @{
*/

/**
* @}
*/


/** @defgroup SDK_EVAL_Button_Exported_Functions        SDK EVAL Button Exported Functions
* @{
*/
  
void SdkEvalPushButtonInit(SdkEvalButton xButton, SdkEvalButtonMode xButtonMode);
FlagStatus SdkEvalPushButtonGetState(SdkEvalButton xButton);

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
