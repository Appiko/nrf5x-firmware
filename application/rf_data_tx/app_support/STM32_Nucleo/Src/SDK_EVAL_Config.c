/**
* @file    SDK_EVAL_Config.c
* @author  VMA division - AMS
* @version 3.2.0
* @date    May 1, 2016
* @brief   This file provides all the low level API to manage SDK Version identification.
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
#include "SDK_EVAL_Config.h"
#include "cube_hal.h"

/** @addtogroup SDK_EVAL_NUCLEO                                 SDK EVAL NUCLEO
* @brief  This module contains the API to manage the <i>NUCLEO</i> motherboard.
* @note The pin and specific struct details are located in the file stm32lxxx_nucleo.h .
* @{
*/


/** @addtogroup SDK_EVAL_Config
* @{
*/

/** @defgroup SDK_EVAL_Config_Private_TypesDefinitions             SDK EVAL Config Private Types Definitions
* @{
*/

/**
* @}
*/


/** @defgroup SDK_EVAL_Config_Private_Defines                      SDK EVAL Config Private Defines
* @{
*/

/**
* @}
*/


/** @defgroup SDK_EVAL_Config_Private_Macros                       SDK EVAL Config Private Macros
* @{
*/

/**
* @}
*/


/** @defgroup SDK_EVAL_Config_Private_Variables                    SDK EVAL Config Private Variables
* @{
*/
/**
* @}
*/


/**
* @defgroup SDK_EVAL_Config_Private_FunctionPrototypes                    SDK EVAL Config Private Function Prototypes
* @{
*/

/**
* @}
*/


/**
* @defgroup SDK_EVAL_Config_Private_Functions                             SDK EVAL Config Private Functions
* @{
*/



static uint8_t s_SdkEvalVersion = SDK_EVAL_NUCLEO_VER;


/**
* @brief  Identifies the current motherboard.
* @param  None.
* @retval None.
*/
void SdkEvalIdentification(void)
{

}
/**
* @brief  Returns the version of the current motherboard.
* @param  None.
* @retval None.
*/
uint8_t SdkEvalGetVersion(void)
{
  return s_SdkEvalVersion;
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
