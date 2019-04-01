/**
 * @file     SDK_EVAL_Gpio.c
 * @author  VMA division - AMS
 * @version 3.2.0
 * @date    May 1, 2016
 * @brief    This file provides all the low level API to manage SDK eval pin to drive GPIOs.
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
 *
 */

/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Gpio.h"
#include "SDK_EVAL_Config.h"

/** @addtogroup SDK_EVAL_NUCLEO
 * @{
 */


/** @addtogroup SDK_EVAL_Gpio
 * @{
 */




/** @defgroup SDK_EVAL_Gpio_Private_Variables                    SDK EVAL Gpio Private Variables
 * @{
 */

/**
 * @brief  M2S GPio Port array
 */

GPIO_TypeDef* vectpxM2SGpioPort[M2S_GPIO_NUMBER] = {M2S_GPIO_0_PORT_NUCLEO,M2S_GPIO_1_PORT_NUCLEO,M2S_GPIO_2_PORT_NUCLEO,M2S_GPIO_3_PORT_NUCLEO,M2S_GPIO_SDN_PORT};

/**
 * @brief  M2S GPio Pin array
 */
static const uint16_t s_vectnM2SGpioPin[M2S_GPIO_NUMBER] = {
        M2S_GPIO_0_PIN,
        M2S_GPIO_1_PIN,
        M2S_GPIO_2_PIN,
        M2S_GPIO_3_PIN,
        M2S_GPIO_SDN_PIN
};

/**
 * @brief  M2S GPio Speed array
 */
static const uint32_t s_vectxM2SGpioSpeed[M2S_GPIO_NUMBER] = {
        M2S_GPIO_0_SPEED,
        M2S_GPIO_1_SPEED,
        M2S_GPIO_2_SPEED,
        M2S_GPIO_3_SPEED,
        M2S_GPIO_SDN_SPEED
};

/**
 * @brief  M2S GPio PuPd array
 */
static const uint32_t s_vectxM2SGpioPuPd[M2S_GPIO_NUMBER] = {
        M2S_GPIO_0_PUPD,
        M2S_GPIO_1_PUPD,
        M2S_GPIO_2_PUPD,
        M2S_GPIO_3_PUPD,
        M2S_GPIO_SDN_PUPD
};

/**
 * @brief  M2S Exti Mode array
 */
static const uint32_t s_vectxM2sGpioExtiMode[M2S_GPIO_NUMBER-1] = {
        M2S_GPIO_0_EXTI_MODE,
        M2S_GPIO_1_EXTI_MODE,
        M2S_GPIO_2_EXTI_MODE,
        M2S_GPIO_3_EXTI_MODE
};


/**
 * @brief  M2S Exti IRQn array
 */
static const IRQn_Type s_vectcM2SGpioExtiIrqn[M2S_GPIO_NUMBER-1] = {
        M2S_GPIO_0_EXTI_IRQN,
        M2S_GPIO_1_EXTI_IRQN,
        M2S_GPIO_2_EXTI_IRQN,
        M2S_GPIO_3_EXTI_IRQN,
};


/**
 * @}
 */





/** @defgroup SDK_EVAL_Gpio_Private_Functions                            SDK EVAL Gpio Private Functions
 * @{
 */

/**
 * @brief  Configures MCU GPIO and EXTI Line for GPIOs.
 * @param  xGpio Specifies the GPIO to be configured.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 *         @arg M2S_GPIO_1: GPIO_1
 *         @arg M2S_GPIO_2: GPIO_2
 *         @arg M2S_GPIO_3: GPIO_3
 *         @arg M2S_GPIO_SDN: GPIO_SDN
 * @param  xGpioMode Specifies GPIO mode.
 *         This parameter can be one of following parameters:
 *         @arg M2S_MODE_GPIO_IN: MCU GPIO will be used as simple input.
 *         @arg M2S_MODE_EXTI_IN: MCU GPIO will be connected to EXTI line with interrupt
 *         generation capability.
 *         @arg M2S_MODE_GPIO_OUT: MCU GPIO will be used as simple output.
 * @retval None.
 */
void SdkEvalM2SGpioInit(M2SGpioPin xGpio, M2SGpioMode xGpioMode)
{
  GPIO_InitTypeDef GPIO_InitStructure, EXTI_InitStructure;
  
  /* Check the parameters */
  assert_param(IS_M2S_GPIO_PIN(xGpio));
  assert_param(IS_M2S_GPIO_MODE(xGpioMode));
  
  switch(xGpio)
  {
  case M2S_GPIO_0:
    M2S_GPIO_0_CLOCK_NUCLEO();
    break;
  case M2S_GPIO_1:
    M2S_GPIO_1_CLOCK_NUCLEO();
    break;
  case M2S_GPIO_2:
    M2S_GPIO_2_CLOCK_NUCLEO();
    break;
  case M2S_GPIO_3:
    M2S_GPIO_3_CLOCK_NUCLEO();
    break;
  case M2S_GPIO_SDN:
    M2S_GPIO_SDN_CLOCK_NUCLEO();
    break;
  }
  
  /* Configures MCU GPIO */
  if(xGpioMode == M2S_MODE_GPIO_OUT)
    GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  else
    GPIO_InitStructure.Mode = GPIO_MODE_INPUT;
  
  GPIO_InitStructure.Pull = s_vectxM2SGpioPuPd[xGpio];
  GPIO_InitStructure.Speed = s_vectxM2SGpioSpeed[xGpio];
  GPIO_InitStructure.Pin = s_vectnM2SGpioPin[xGpio];
  HAL_GPIO_Init(vectpxM2SGpioPort[xGpio], &GPIO_InitStructure);
  
  /* Configures MCU GPIO EXTI line */
  EXTI_InitStructure.Pull = s_vectxM2SGpioPuPd[xGpio];
  EXTI_InitStructure.Mode = s_vectxM2sGpioExtiMode[xGpio];
  EXTI_InitStructure.Pin = s_vectnM2SGpioPin[xGpio];
  EXTI_InitStructure.Speed = s_vectxM2SGpioSpeed[xGpio];
  
  if (xGpioMode == M2S_MODE_EXTI_IN) {
    HAL_GPIO_Init(vectpxM2SGpioPort[xGpio], &EXTI_InitStructure);

  }

}

/**
 * @brief  Enables or disables the interrupt on GPIO .
 * @param  xGpio Specifies the GPIO whose priority shall be changed.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 *         @arg M2S_GPIO_1: GPIO_1
 *         @arg M2S_GPIO_2: GPIO_2
 *         @arg M2S_GPIO_3: GPIO_3
 * @param  nPreemption Specifies Preemption Priority.
 * @param  nSubpriority Specifies Subgroup Priority.
 * @param  xNewState Specifies the State.
 *         This parameter can be one of following parameters:
 *         @arg ENABLE: Interrupt is enabled
 *         @arg DISABLE: Interrupt is disabled
 * @retval None.
 */
void SdkEvalM2SGpioInterruptCmd(M2SGpioPin xGpio, uint8_t nPreemption, uint8_t nSubpriority, FunctionalState xNewState)
{  
  if (xNewState == ENABLE)  {
    HAL_NVIC_EnableIRQ(s_vectcM2SGpioExtiIrqn[xGpio]);
  HAL_NVIC_SetPriority(s_vectcM2SGpioExtiIrqn[xGpio], nPreemption, nSubpriority);

  }
  else {
    HAL_NVIC_DisableIRQ(s_vectcM2SGpioExtiIrqn[xGpio]);
  }
}

/**
 * @brief  Enables or disables trigger on rising edge for that GPIO .
 * @param  xGpio Specifies the GPIO.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 *         @arg M2S_GPIO_1: GPIO_1
 *         @arg M2S_GPIO_2: GPIO_2
 *         @arg M2S_GPIO_3: GPIO_3
 * @param  xNewState Specifies the State.
 *         This parameter can be one of following parameters:
 *         @arg ENABLE: Rising trigger is enabled
 *         @arg DISABLE: Rising trigger is disabled
 * @retval None.
 */
void SdkEvalM2SGpioTriggerRising(M2SGpioPin xGpio, FunctionalState xNewState)
{
  if(xNewState)
    EXTI->RTSR |= (uint16_t)s_vectnM2SGpioPin[xGpio];
  else
    EXTI->RTSR &= ~(uint16_t)s_vectnM2SGpioPin[xGpio];
}

/**
 * @brief  To assert if the rising edge IRQ is enabled for that GPIO .
 * @param  xGpio Specifies the GPIO.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 *         @arg M2S_GPIO_1: GPIO_1
 *         @arg M2S_GPIO_2: GPIO_2
 *         @arg M2S_GPIO_3: GPIO_3
 * @retval  Specifies the State.
 *         @arg ENABLE: Rising trigger is enabled
 *         @arg DISABLE: Rising trigger is disabled
 */
FunctionalState SdkEvalM2SGpioGetTriggerRising(M2SGpioPin xGpio)
{
  if(EXTI->RTSR & (uint16_t)s_vectnM2SGpioPin[xGpio])
    return ENABLE;
  
  return DISABLE;
}

/**
 * @brief  Enables or disables trigger on falling edge for that GPIO .
 * @param  xGpio Specifies the GPIO.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 *         @arg M2S_GPIO_1: GPIO_1
 *         @arg M2S_GPIO_2: GPIO_2
 *         @arg M2S_GPIO_3: GPIO_3
 * @param  xNewState Specifies the State.
 *         This parameter can be one of following parameters:
 *         @arg ENABLE: Falling trigger is enabled
 *         @arg DISABLE: Falling trigger is disabled
 * @retval None.
 */
void SdkEvalM2SGpioTriggerFalling(M2SGpioPin xGpio, FunctionalState xNewState)
{
  if(xNewState)
    EXTI->FTSR |= (uint16_t)s_vectnM2SGpioPin[xGpio];
  else
    EXTI->FTSR &= ~(uint16_t)s_vectnM2SGpioPin[xGpio];
}

/**
 * @brief  To assert if the falling edge IRQ is enabled for that GPIO .
 * @param  xGpio Specifies the GPIO.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 *         @arg M2S_GPIO_1: GPIO_1
 *         @arg M2S_GPIO_2: GPIO_2
 *         @arg M2S_GPIO_3: GPIO_3
 * @retval  Specifies the State.
 *         @arg ENABLE: Falling trigger is enabled
 *         @arg DISABLE: Falling trigger is disabled
 */
FunctionalState SdkEvalM2SGpioGetTriggerFalling(M2SGpioPin xGpio)
{
  if(EXTI->FTSR & (uint16_t)s_vectnM2SGpioPin[xGpio])
    return ENABLE;
  
  return DISABLE;
}

/**
 * @brief  Returns the level of a specified GPIO.
 * @param  xGpio Specifies the GPIO to be read.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 *         @arg M2S_GPIO_1: GPIO_1
 *         @arg M2S_GPIO_2: GPIO_2
 *         @arg M2S_GPIO_3: GPIO_3
 * @retval FlagStatus Level of the GPIO. This parameter can be:
 *         SET or RESET.
 */
FlagStatus SdkEvalGpioGetLevel(M2SGpioPin xGpio)
{
  /* Gets the GPIO level */
  GPIO_PinState ret = HAL_GPIO_ReadPin(vectpxM2SGpioPort[xGpio], s_vectnM2SGpioPin[xGpio]);
  
  return (FlagStatus)ret;
}

/**
 * @brief  Sets the level of a specified GPIO.
 * @param  xGpio Specifies the GPIO to be set.
 *         This parameter can be one of following parameters:
 *         @arg M2S_GPIO_0: GPIO_0
 *         @arg M2S_GPIO_1: GPIO_1
 *         @arg M2S_GPIO_2: GPIO_2
 *         @arg M2S_GPIO_3: GPIO_3
 * @param  FlagStatus Level of the GPIO. This parameter can be:
 *         SET or RESET.
 * @retval None.
 */
void SdkEvalGpioSetLevel(M2SGpioPin xGpio, FlagStatus xLevel)
{
  /* Sets the GPIO level */
  HAL_GPIO_WritePin(vectpxM2SGpioPort[xGpio], s_vectnM2SGpioPin[xGpio], (GPIO_PinState)xLevel);
}

/**
 * @brief  Puts at logic 1 the SDN pin.
 * @param  None.
 * @retval None.
 */
void SdkEvalEnterShutdown(void)
{
  /* Puts high the GPIO connected to shutdown pin */
  HAL_GPIO_WritePin(M2S_GPIO_SDN_PORT, M2S_GPIO_SDN_PIN, GPIO_PIN_SET);
}


/**
 * @brief  Put at logic 0 the SDN pin.
 * @param  None.
 * @retval None.
 */
void SdkEvalExitShutdown(void)
{
  /* Puts low the GPIO connected to shutdown pin */
  HAL_GPIO_WritePin(M2S_GPIO_SDN_PORT, M2S_GPIO_SDN_PIN, GPIO_PIN_RESET);

  /* Delay to allow the circuit POR, about 700 us */
  for(volatile uint32_t i=0;i<0x1E00;i++);
}

/**
 * @brief  check the logic(0 or 1) at the SDN pin.
 * @param  None.
 * @retval FlagStatus.
 */
FlagStatus SdkEvalCheckShutdown(void)
{
  return  SdkEvalGpioGetLevel(M2S_GPIO_SDN);
}

/**
 * @brief  Gets the GPIO_PIN of the M2SGpioPin.
 * @param  xGpio: M2S GPIO.
 * @retval uint16_t GPIO_PIN value.
 */
uint16_t SdkEvalGpioGetPin(M2SGpioPin xGpio)
{
  return s_vectnM2SGpioPin[xGpio];
}


/**
 * @brief  Initialize the TCXO enable pin.
 * @param  None.
 * @retval None.
 */
void SdkEvalTcxoInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
    
  TCXO_EN_GPIO_CLK();
  
  /* Configures MCU GPIO */
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_NOPULL;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pin = TCXO_EN_PIN;
  HAL_GPIO_Init(TCXO_EN_PORT, &GPIO_InitStructure);
}

/**
 * @brief  Puts at logic 1 the TCXO pin.
 * @param  None.
 * @retval None.
 */
void SdkEvalTcxoOn(void)
{
  /* Puts high the GPIO connected to TCXO pin */
  HAL_GPIO_WritePin(TCXO_EN_PORT, TCXO_EN_PIN, GPIO_PIN_SET);
}


/**
 * @brief  Put at logic 0 the TCXO pin.
 * @param  None.
 * @retval None.
 */
void SdkEvalTcxoOff(void)
{
  /* Puts low the GPIO connected to TCXO pin */
  HAL_GPIO_WritePin(TCXO_EN_PORT, TCXO_EN_PIN, GPIO_PIN_RESET);
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
