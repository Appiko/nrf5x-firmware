/**
 * @file    SPIRIT_SDK_Util.h
 * @author  LowPower RF BU - AMG
 * @version 1.1.0
 * @date    July 1, 2016
 * @brief   Identification functions for S2-LP DK.
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

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef S2LP_SDK_UTIL_H_
#define S2LP_SDK_UTIL_H_


/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Config.h"

#ifdef __cplusplus
  "C" {
#endif


/**
 * @addtogroup SPIRIT_DK
 * @{
 */


/**
 * @defgroup SDK_SPIRIT_MANAGEMENT
 * @{
 */

typedef enum
{
  RANGE_EXT_NONE = 0,
  RANGE_EXT_SKYWORKS_868,
} RangeExtType;

typedef enum
{
  S2LP_CUT_2_1 = 0x91,
  S2LP_CUT_2_0 = 0x81,
  S2LP_CUT_3_0 = 0xC1,
} S2LPCutType;

/**
 * @addgroup SDK_SPIRIT_MANAGEMENT_FUNCTIONS
 * @{
 */
void S2LPManagementEnableTcxo(void);
void S2LPManagementIdentificationRFBoard(void);
void S2LPManagementSetBand(uint8_t value);
uint8_t S2LPManagementGetBand(void);
void S2LPManagementSetOffset(int32_t value);
int32_t S2LPManagementGetOffset(void);
void S2LPManagementRcoCalibration(void);
void S2LPManagementRangeExtInit(void);
RangeExtType S2LPManagementGetRangeExtender(void);
S2LPCutType S2LPManagementGetCut(void);
uint8_t S2LPManagementGetTcxo(void);
void S2LPManagementTcxoInit(void);
uint32_t S2LPManagementGetXtalFrequency(void);
void S2LPManagementSetRangeExtender(RangeExtType xRangeType);
uint32_t S2LPManagementComputeRcoFrequency(void);
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
