/**
 * @file    SDK_BasicGeneric_B.c
 * @author  LowPower RF BU - AMG
 * @version 1.2.1
 * @date    16-April-2018
 * @brief   Example of reception of S2-LP Basic packets.
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
 * <h2><center>&copy; COPYRIGHT 2018 STMicroelectronics</center></h2>
 */


/* Includes ------------------------------------------------------------------*/
#include "SDK_EVAL_Config.h"
#include "S2LP_Config.h"
#include "SDK_Configuration_Common.h"
#include "S2LP_SDK_Util.h"


#define USE_VCOM





/**
 * @addtogroup SDK_Examples
 * @{
 */

/**
 * @addtogroup SDK_Basic_Generic        SDK Basic Generic
 * @{
 */

/**
 * @addtogroup SDK_Basic_Generic_B              SDK Basic Generic B
 * @brief Device B configured as a receiver.
 * @details This code explains how to configure a receiver for
 * basic packets.
 *
 * The user can change the Basic packet configuration parameters editing the defines
 * at the beginning of the file.
 * @{
 */


/**
 * @defgroup Basic_Generic_B_Private_Variables                          Basic Generic B Private Variables
 * @{
 */

 /**
  * @brief Radio structure fitting
  */
SRadioInit xRadioInit = {
  BASE_FREQUENCY,
  MODULATION_SELECT,
  DATARATE,
  FREQ_DEVIATION,
  BANDWIDTH
};


/**
 * @brief Packet Basic structure fitting
 */
PktBasicInit xBasicInit={
  PREAMBLE_LENGTH,
  SYNC_LENGTH,
  SYNC_WORD,
  VARIABLE_LENGTH,
  EXTENDED_LENGTH_FIELD,
  CRC_MODE,
  EN_ADDRESS,
  EN_FEC,
  EN_WHITENING
};



/**
 * @brief GPIO IRQ structure fitting
 */
SGpioInit xGpioIRQ={
  S2LP_GPIO_3,
  S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP,
  S2LP_GPIO_DIG_OUT_IRQ
};


/**
 * @brief IRQ status struct declaration
 */
S2LPIrqs xIrqStatus;


/**
 * @brief Rx buffer declaration: how to store the received data
 */
uint8_t vectcRxBuff[128], cRxData;

/**
* @brief Preemption priority IRQ
*/
#define IRQ_PREEMPTION_PRIORITY         0x03

/**
 *@}
 */


/**
 * @defgroup Basic_Generic_B_Private_Functions                                  Basic Generic B Private Functions
 * @{
 */

/**
 * @brief  This function handles External interrupt request. In this application it is used
 *         to manage the S2LP IRQ configured to be notified on the S2LP GPIO_3.
 * @param  None
 * @retval None
 */
static uint16_t M2S_GPIO_PIN_IRQ;

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
  if(GPIO_Pin==M2S_GPIO_PIN_IRQ)
  { 
    
    /* Get the IRQ status */
    S2LPGpioIrqGetStatus(&xIrqStatus);
    
    /* Check the S2LP RX_DATA_DISC IRQ flag */
    if(xIrqStatus.IRQ_RX_DATA_DISC) {
      /* toggle LED1 */
      SdkEvalLedToggle(LED1);
      
#ifdef USE_VCOM
      printf("DATA DISCARDED\n\r");
#endif
          
      /* RX command - to ensure the device will be ready for the next reception */
      S2LPCmdStrobeRx();
    }
    
    /* Check the S2LP RX_DATA_READY IRQ flag */
    if(xIrqStatus.IRQ_RX_DATA_READY) {
      /* Get the RX FIFO size */
      cRxData = S2LPFifoReadNumberBytesRxFifo();
      
      /* Read the RX FIFO */
      S2LPSpiReadFifo(cRxData, vectcRxBuff);
      
      /* Flush the RX FIFO */
      S2LPCmdStrobeFlushRxFifo();      
      
      /*  A simple way to check if the received data sequence is correct (in this case LED5 will toggle) */
      {
        SBool xCorrect=S_TRUE;
        
        for(uint8_t i=0 ; i<cRxData ; i++)
          if(vectcRxBuff[i] != i+1)
            xCorrect=S_FALSE;
        
        if(xCorrect) {
          /* toggle LED2 */
          SdkEvalLedToggle(LED2);
#ifdef USE_VCOM
          printf("DATA CORRECT, RSSI: %d dBm\r\n",S2LPRadioGetRssidBm());
#endif
        }
      }      
      /* RX command - to ensure the device will be ready for the next reception */
      S2LPCmdStrobeRx();
      
#ifdef USE_VCOM
      /* print the received data */
      printf("B data received: [");
      for(uint8_t i=0 ; i<cRxData ; i++)
        printf("%d ", vectcRxBuff[i]);
      printf("]\r\n");
#endif
    }    
    
  }
  
}


#ifdef USE_STM32L0XX_NUCLEO
/**
* @brief  Configure the System clock to work with HSI PLL'd to give a 32MHz system clock.
* @param  None
* @retval None
*/
static void SystemClock_Config(void)
{
  RCC_ClkInitTypeDef RCC_ClkInitStruct;
  RCC_OscInitTypeDef RCC_OscInitStruct;
  
  /* Enable Power Control clock */
  __PWR_CLK_ENABLE();
  
  /* The voltage scaling allows optimizing the power consumption when the device is 
  clocked below the maximum system frequency, to update the voltage scaling value 
  regarding system frequency refer to product datasheet.  */
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Enable HSI Oscillator and activate PLL with HSI as source */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSEState = RCC_HSE_OFF;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLLMUL_4;
  RCC_OscInitStruct.PLL.PLLDIV = RCC_PLLDIV_2;
  RCC_OscInitStruct.HSICalibrationValue = 0x10;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);  
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2 
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;  
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;  
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
}
#else
static void SystemClock_Config(void)
{
  
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  
  /* Enable HSE Oscillator and Activate PLL with HSE as source */
  RCC_OscInitStruct.OscillatorType      = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState            = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState        = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource       = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLMUL          = RCC_PLL_MUL6;
  RCC_OscInitStruct.PLL.PLLDIV          = RCC_PLL_DIV3;
  HAL_RCC_OscConfig(&RCC_OscInitStruct);
  
  /* Set Voltage scale1 as MCU will run at 32MHz */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);
  
  /* Poll VOSF bit of in PWR_CSR. Wait until it is reset to 0 */
  while (__HAL_PWR_GET_FLAG(PWR_FLAG_VOS) != RESET) {};
  
  /* Select PLL as system clock source and configure the HCLK, PCLK1 and PCLK2
  clocks dividers */
  RCC_ClkInitStruct.ClockType = (RCC_CLOCKTYPE_SYSCLK | RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_PCLK1 | RCC_CLOCKTYPE_PCLK2);
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;
  HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1);
  
}
#endif


/**
 * @brief  System main function.
 * @param  None
 * @retval None
 */
int main (void)
{
  HAL_Init();
  SystemClock_Config();
  
  SdkEvalIdentification();
  SdkEvalLedInit(LED1);
  SdkEvalM2SGpioInit(M2S_GPIO_SDN,M2S_MODE_GPIO_OUT);
  S2LPSpiInit();
  
#ifdef USE_VCOM
  SdkEvalComInit();
#endif
  
  /* S2LP ON */
  S2LPEnterShutdown();
  S2LPExitShutdown();
  
  /* EEPROM SPI if init + data retrieving */
  S2LPManagementIdentificationRFBoard();
  
  /* if the board has eeprom, we can compensate the offset calling S2LPManagementGetOffset
  (if eeprom is not present this fcn will return 0) */
  xRadioInit.lFrequencyBase = xRadioInit.lFrequencyBase + S2LPManagementGetOffset();

  /* if needed this will set the range extender pins */
  S2LPManagementRangeExtInit();
  
  /* if needed this will set the EXT_REF bit of the S2-LP */
  S2LPManagementTcxoInit();
  
  /* uC IRQ config */
  SdkEvalM2SGpioInit(M2S_GPIO_3,M2S_MODE_EXTI_IN);
  M2S_GPIO_PIN_IRQ=SdkEvalGpioGetPin(M2S_GPIO_3);
  
  /* S2LP IRQ config */
  S2LPGpioInit(&xGpioIRQ);  
  
  /* uC IRQ enable */
  SdkEvalM2SGpioInterruptCmd(M2S_GPIO_3,IRQ_PREEMPTION_PRIORITY,0,ENABLE);
  
  /* S2LP Radio config */
  S2LPRadioInit(&xRadioInit);
  
  /* S2LP Packet config */
  S2LPPktBasicInit(&xBasicInit);
  
  /* S2LP IRQs enable */
  S2LPGpioIrqDeInit(&xIrqStatus);
  S2LPGpioIrqConfig(RX_DATA_DISC,S_ENABLE);
  S2LPGpioIrqConfig(RX_DATA_READY,S_ENABLE);

  /* payload length config */
  S2LPPktBasicSetPayloadLength(20);

  /* RX timeout config */
  S2LPTimerSetRxTimerUs(700000);
  //SET_INFINITE_RX_TIMEOUT();

  /* IRQ registers blanking */
  S2LPGpioIrqClearStatus();
  
  /* RX command */
  S2LPCmdStrobeRx();

  /* infinite loop */
  while (1){
  }

}



#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param file: pointer to the source file name
  * @param line: assert_param error line source number
  * @retval : None
  */
void assert_failed(uint8_t* file, uint32_t line)
{
  /* User can add his own implementation to report the file name and line number */
     printf("Wrong parameters value: file %s on line %d\r\n", file, line);

  /* Infinite loop */
  while (1)
  {
  }
}
#endif


/**
 *@}
 */

/**
 *@}
 */

/**
 *@}
 */

/**
 *@}
 */


/******************* (C) COPYRIGHT 2018 STMicroelectronics *****END OF FILE****/
