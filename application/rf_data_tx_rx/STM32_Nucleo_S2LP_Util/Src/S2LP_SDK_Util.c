/**
 * @file    SPIRIT_SDK_Util.c
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


/* Includes ------------------------------------------------------------------*/
#include "S2LP_SDK_Util.h"
#include "MCU_Interface.h"


/**
* @addtogroup SPIRIT_DK                   SPIRIT DK
* @{
*/


/**
* @defgroup SDK_SPIRIT_MANAGEMENT              SDK SPIRIT Management
* @{
*/




/**
* @brief This flag is used to synchronize the TIM3 ISR with the XtalMeasurement routine.
*/
static volatile FlagStatus s_xTIMChCompareModeRaised = RESET;

/**
* @brief This flag is used to synchronize the TIM3 ISR with the XtalMeasurement routine.
*/
static volatile uint8_t s_RfModuleBand = 0, s_Tcxo=0;
static volatile int32_t s_RfModuleOffset=0;
static volatile RangeExtType s_RangeExtType = RANGE_EXT_NONE;
static volatile uint32_t s_XtalFrequency=50000000;
static volatile S2LPCutType s_S2LPCut = S2LP_CUT_2_1;
static TIM_HandleTypeDef  TimHandle;


/**
* @defgroup SDK_SPIRIT_MANAGEMENT_FUNCTIONS    SDK SPIRIT Management Functions
* @{
*/

/**
* @defgroup IDENTIFICATION_FUNCTIONS      SDK SPIRIT Management Identification Functions
* @{
*/
void S2LPRadioSetXtalFrequency(uint32_t lXtalFrequency);

/**
* @brief This function handles TIM3 global interrupt
* @param None.
* @retval None.
*/
void TIM3_IRQHandler(void)
{
  HAL_TIM_IRQHandler(&TimHandle);
}

void HAL_TIM_IC_CaptureCallback(TIM_HandleTypeDef *htim) 
{
  s_xTIMChCompareModeRaised = SET;
}


void S2LPManagementEnableTcxo(void)
{
  __GPIOC_CLK_ENABLE();
  HAL_GPIO_Init(GPIOC, &(GPIO_InitTypeDef){GPIO_PIN_7, GPIO_MODE_OUTPUT_PP, GPIO_NOPULL, GPIO_SPEED_HIGH });
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_7, GPIO_PIN_SET);
}

void S2LPManagementRangeExtInit(void)
{
  
  if(s_RangeExtType==RANGE_EXT_SKYWORKS_868)
  {
    uint8_t tmp[3]={0x92,0x52,0x2A};
    SdkEvalSpiWriteRegisters(0x00, 3, tmp);
  }
}

RangeExtType S2LPManagementGetRangeExtender(void)
{
  return s_RangeExtType;
}

S2LPCutType S2LPManagementGetCut(void)
{
  return s_S2LPCut;
}

static void S2LPManagementSetRangeExtender(RangeExtType xRangeType)
{
  s_RangeExtType = xRangeType;
}

static void S2LPManagementSetTcxo(uint8_t tcxo)
{
  s_Tcxo = tcxo;
}

uint8_t S2LPManagementGetTcxo(void)
{
  return s_Tcxo;
}

void S2LPManagementTcxoInit(void)
{
  if(s_Tcxo)
  {
    S2LPManagementEnableTcxo();
    
    uint8_t tmp;
    SdkEvalSpiReadRegisters(0x6D, 1, &tmp);
    tmp|=0x80;
    SdkEvalSpiWriteRegisters(0x6D, 1, &tmp);
  }
}

/**
* @brief  This function can be used to automatically measure the XTAL frequency making use of the
*         Spirit clock output to pin and an STM32L timer in compare mode.
* @param  None.
* @retval None.
*/
uint32_t S2LPManagementComputeXtalFrequency(void)
{   
  uint32_t lMeasuredXtalFrequency;
  
  
#if defined(USE_STM32L0XX_NUCLEO) || defined(USE_STM32F0XX_NUCLEO)
  lMeasuredXtalFrequency=50000000;
#else
  uint8_t tmp;
  
#define N_SAMPLES 20
#define SETTLING_PERIODS 40
  
  GPIO_TypeDef *pGpioPeriph;
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_IC_InitTypeDef     sICConfig;
  
  
  TimHandle.Instance = TIM3;
  pGpioPeriph=GPIOB;
  /* TIM3 clock enable */
  __HAL_RCC_TIM3_CLK_ENABLE();
  /* GPIOC clock enable */
  __GPIOB_CLK_ENABLE();
    
  /* Instance the variables used to compute the XTAL frequency */
  uint8_t CaptureNumber=0;
  uint16_t IC3ReadValue1=0,IC3ReadValue2=0,Capture=0;
  volatile uint16_t cWtchdg = 0;
  uint32_t TIMFreq=0,lXoFreq=0,lXoFreqRounded,j=0;
  
  /* TIM3 channel 2 pin (PC.07) configuration */
  GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull  = GPIO_PULLUP;
  GPIO_InitStructure.Pin   = GPIO_PIN_0;

  GPIO_InitStructure.Alternate   = GPIO_AF2_TIM3;
  
  HAL_GPIO_Init(pGpioPeriph, &GPIO_InitStructure);
    
  /* Configure the timer IRQ to be raised on the rising fronts */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  
  /* Input capture selection setting */
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
   
  /* Input capture prescaler setting. Setting it to TIM_ICPSC_DIV8 makes the IRQ are raised every 8 rising fronts detected by hardware.  */
  sICConfig.ICPrescaler = TIM_ICPSC_DIV8;
  
  /* Disable every kind of capture filter */
  sICConfig.ICFilter = 0x0;

  /* Timer initialization */
  HAL_TIM_IC_ConfigChannel(&TimHandle, &sICConfig, TIM_CHANNEL_3);
  
  /* TIM enable counter */
  HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_3);
        
  HAL_NVIC_SetPriority(TIM3_IRQn, 0x0F, 0x0F);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
 
  
  /* Disable the clock divider to measure the max frequency of the clock. */
  //S2LPRadioSetDigDiv(S_DISABLE);
  tmp=0x55;
  SdkEvalSpiWriteRegisters(0x6C, 1, &tmp);
    
  /* GPIO2 to the clock output */
  //S2LPGpioInit(&(SGpioInit){S2LP_GPIO_2, S2LP_GPIO_MODE_DIGITAL_OUTPUT_LP, S2LP_GPIO_DIG_OUT_MCU_CLOCK});
  tmp=0x88|0x02;
  SdkEvalSpiWriteRegisters(0x02, 1, &tmp);
  
  SdkEvalSpiReadRegisters(0xF1, 1, &tmp);
  if((tmp&0x80)==0)
  {/* CUT 1.x */
    tmp=0x9E;
  }
  else
  {
    tmp=0x90;
  }
  SdkEvalSpiWriteRegisters(0x04, 1, &tmp);

  //S2LPGpioClockOutput(S_ENABLE);
  
  
  /* measure the frequency and average it on N_SAMPLES. Moreover cycle to wait for same SETTLING_PERIODS */
  for(uint32_t i=0;i<2*(N_SAMPLES+SETTLING_PERIODS);i++) {
    /* block the routine until the TIM CCP2 IRQ is raised */
    while(!s_xTIMChCompareModeRaised && (cWtchdg!=0xFFFF)) {
      cWtchdg++;    
    }
    
    if(cWtchdg==0xFFFF) {
      break;
    }
    else {
      cWtchdg=0;
    }
    
    /* reset the IRQ raised flag */
    s_xTIMChCompareModeRaised = RESET;
    
    /* if the SETTLING PERIODS expired */
    if(i>=SETTLING_PERIODS*2) {
      /* First TIMER capture */
      if(CaptureNumber == 0)
      {
        /* Get the Input Capture value */
        IC3ReadValue1 = HAL_TIM_ReadCapturedValue(&TimHandle, TIM_CHANNEL_3);
        CaptureNumber = 1;
      }
      /* Second TIMER capture */
      else if(CaptureNumber == 1)
      {
        /* Get the Input Capture value */
        IC3ReadValue2 = HAL_TIM_ReadCapturedValue(&TimHandle, TIM_CHANNEL_3);
        
        /* Capture computation */
        if (IC3ReadValue2 > IC3ReadValue1)
        {
          /* If the TIMER didn't overflow between the first and the second capture. Compute it as the difference between the second and the first capture values. */
          Capture = (IC3ReadValue2 - IC3ReadValue1) - 1;
        }
        else
        {
          /* .. else, if overflowed 'roll' the first measure to be complementar of 0xFFFF */
          Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2) - 1;
        }
        
        /* Punctual frequency computation */
        TIMFreq = (uint32_t) SystemCoreClock / Capture;
        
        /* Averaged frequency computation */
        //lXoFreq =(uint32_t)(A*(float)lXoFreq+((float)1-A)*(float)TIMFreq);
        lXoFreq += TIMFreq;
        j++;
        
        CaptureNumber = 0;
      }
    }
  }
  lXoFreq /= j;
  
  /* Compute the real frequency in Hertz tanking in account the MCU and Spirit divisions */

  SdkEvalSpiReadRegisters(0xF1, 1, &tmp);
  /* cut 1.1 */
  if((tmp&0x80)==0)
  {
    lXoFreq *=(192*8);
  }
  /* cut 2.0 */
  else
  {
    lXoFreq *=(256*8);
  }
  
  /* Disable the output clock */
  tmp=0x00;
  SdkEvalSpiWriteRegisters(0x04, 1, &tmp);
  //S2LPGpioClockOutput(S_DISABLE);
  
  /* TIM disable counter */
  HAL_TIM_IC_Stop(&TimHandle, TIM_CHANNEL_3);
    
  SdkEvalM2SGpioInit(M2S_GPIO_2, M2S_MODE_GPIO_IN);
  
  /* S2LP GPIO 0 to the default configuration */
  tmp=0xA2;
  SdkEvalSpiWriteRegisters(0x02, 1, &tmp);
  //S2LPGpioSetLevel(S2LP_GPIO_2, LOW);

  //S2LPRadioSetDigDiv(S_ENABLE);
  tmp=0x45;
  SdkEvalSpiWriteRegisters(0x6C, 1, &tmp);
  
  lXoFreqRounded=(lXoFreq/1000000)*1000000;
  
  lMeasuredXtalFrequency=lXoFreqRounded;
  if(lXoFreq-lXoFreqRounded > (lXoFreqRounded+1000000)-lXoFreq)
  {
     lMeasuredXtalFrequency+=1000000;
  }
     
  
#endif  
  
  S2LPRadioSetXtalFrequency(lMeasuredXtalFrequency);
  s_XtalFrequency=lMeasuredXtalFrequency;
  
  return lMeasuredXtalFrequency;
}

uint32_t S2LPManagementGetXtalFrequency(void)
{
  return s_XtalFrequency;
}


uint32_t S2LPManagementComputeXtalFrequencyGpio2(void)
{
  return S2LPManagementComputeXtalFrequency();
}




uint32_t S2LPManagementComputeRcoFrequency(void)
{   
  uint32_t lMeasuredRcoFrequency;
  
#if defined(USE_STM32L0XX_NUCLEO) || defined(USE_STM32F0XX_NUCLEO)
  lMeasuredRcoFrequency=33330;
#else
  uint8_t tmp;
  
#define RCO_SETTLING_PERIODS            4000
#define RCO_N_SAMPLES                   10000
  
  HAL_NVIC_DisableIRQ(TIM2_IRQn);
  
  GPIO_TypeDef *pGpioPeriph;
  GPIO_InitTypeDef GPIO_InitStructure;
  TIM_IC_InitTypeDef     sICConfig;
  
 
  TimHandle.Instance = TIM3;
  pGpioPeriph=GPIOB;
  __HAL_RCC_TIM3_CLK_ENABLE();
  __GPIOB_CLK_ENABLE();
 

  /* Instance the variables used to compute the XTAL frequency */
  uint8_t CaptureNumber=0;
  uint16_t IC3ReadValue1=0,IC3ReadValue2=0,Capture=0;
  volatile uint16_t cWtchdg = 0;
  uint32_t TIMFreq=0,lXoFreq=0;

  
 
  GPIO_InitStructure.Mode  = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Pull  = GPIO_PULLUP;
  GPIO_InitStructure.Pin   = GPIO_PIN_0;

  GPIO_InitStructure.Alternate   = GPIO_AF2_TIM3;
 
  HAL_GPIO_Init(pGpioPeriph, &GPIO_InitStructure);
    
  /* Configure the timer IRQ to be raised on the rising fronts */
  sICConfig.ICPolarity  = TIM_ICPOLARITY_RISING;
  
  /* Input capture selection setting */
  sICConfig.ICSelection = TIM_ICSELECTION_DIRECTTI;
   
  /* Input capture prescaler setting. Setting it to TIM_ICPSC_DIV8 makes the IRQ are raised every 8 rising fronts detected by hardware.  */
  sICConfig.ICPrescaler = TIM_ICPSC_DIV1;
  
  /* Disable every kind of capture filter */
  sICConfig.ICFilter = 0x0;

  /* Timer initialization */
  HAL_TIM_IC_ConfigChannel(&TimHandle, &sICConfig, TIM_CHANNEL_3);
  
  /* TIM enable counter */
  HAL_TIM_IC_Start_IT(&TimHandle, TIM_CHANNEL_3);
        
  
  HAL_NVIC_SetPriority(TIM3_IRQn, 0x0F, 0x0F);
  HAL_NVIC_EnableIRQ(TIM3_IRQn);
  
  
  /* Disable the clock divider to measure the max frequency of the clock. */
  //S2LPRadioSetDigDiv(S_DISABLE);
  tmp=0x55;
  SdkEvalSpiWriteRegisters(0x6C, 1, &tmp);
  
  SdkEvalSpiReadRegisters(0xF1, 1, &tmp);
  tmp=0x88|0x02;
  SdkEvalSpiWriteRegisters(0x02, 1, &tmp);
  
  SdkEvalSpiReadRegisters(0xF1, 1, &tmp);
  if((tmp&0x80)==0)
  {/* CUT 1.x */
    tmp=0x9E;
  }
  else
  {
    tmp=0x90;
  }
  SdkEvalSpiWriteRegisters(0x04, 1, &tmp);
    
  // go to SLEEP
  SdkEvalSpiCommandStrobes(0x64);
  
  
  uint32_t n=0;
  
  /* measure the frequency and average it on N_SAMPLES. Moreover cycle to wait for same SETTLING_PERIODS */
  for(uint32_t i=0;i<2*(RCO_N_SAMPLES+RCO_SETTLING_PERIODS);i++) {
    /* block the routine until the TIM CCP2 IRQ is raised */
    while(!s_xTIMChCompareModeRaised && (cWtchdg!=0xFFFF)) {
      cWtchdg++;    
    }
    HAL_TIM_IRQHandler(&TimHandle);
    
    if(cWtchdg==0xFFFF) {
      break;
    }
    else {
      cWtchdg=0;
    }
    
    /* reset the IRQ raised flag */
    s_xTIMChCompareModeRaised = RESET;
    
    /* if the SETTLING PERIODS expired */
    if(i>=RCO_SETTLING_PERIODS*2) {
      /* First TIMER capture */
      if(CaptureNumber == 0)
      {
        /* Get the Input Capture value */
        IC3ReadValue1 = HAL_TIM_ReadCapturedValue(&TimHandle, TIM_CHANNEL_3);
        CaptureNumber = 1;
      }
      /* Second TIMER capture */
      else if(CaptureNumber == 1)
      {
        /* Get the Input Capture value */
        IC3ReadValue2 = HAL_TIM_ReadCapturedValue(&TimHandle, TIM_CHANNEL_3);
        
        /* Capture computation */
        if (IC3ReadValue2 > IC3ReadValue1)
        {
          /* If the TIMER didn't overflow between the first and the second capture. Compute it as the difference between the second and the first capture values. */
          Capture = (IC3ReadValue2 - IC3ReadValue1) - 1;
        }
        else
        {
          /* .. else, if overflowed 'roll' the first measure to be complementar of 0xFFFF */
          Capture = ((0xFFFF - IC3ReadValue1) + IC3ReadValue2) - 1;
        }
        
        /* Punctual frequency computation */
        TIMFreq = (uint32_t) SystemCoreClock / Capture;
        
        lXoFreq+=TIMFreq;
        n++;
        
        CaptureNumber = 0;
      }
    }
  }
  
  /* Compute the real frequency in Hertz tanking in account the MCU and Spirit divisions */
  
  lXoFreq/=n;
  
  
  /* Disable the output clock */
  tmp=0x00;
  SdkEvalSpiWriteRegisters(0x04, 1, &tmp);
  //S2LPGpioClockOutput(S_DISABLE);
  
  
  /* TIM disable counter */
  HAL_TIM_IC_Stop(&TimHandle, TIM_CHANNEL_3);
    
  SdkEvalM2SGpioInit(M2S_GPIO_2, M2S_MODE_GPIO_IN);
  
  /* S2LP GPIO2 to the default configuration */
  //S2LPGpioSetLevel(S2LP_GPIO_2, LOW);
  tmp=0xA2;
  SdkEvalSpiWriteRegisters(0x02, 1, &tmp);
  
  //S2LPRadioSetDigDiv(S_ENABLE);
  tmp=0x45;
  SdkEvalSpiWriteRegisters(0x6C, 1, &tmp);
  
  //uint8_t tmp= 0x21; S2LPSpiWriteRegisters(0xB4, 1, &tmp);
  
  lMeasuredRcoFrequency=lXoFreq;
  
  HAL_NVIC_EnableIRQ(TIM2_IRQn);
  //S2LPCmdStrobeReady();
  SdkEvalSpiCommandStrobes(0x62);
  
#endif
  
  return lMeasuredRcoFrequency;
}


/**
* @brief  Read the status register.
* @param  None
* @retval Status
*/
uint8_t EepromIdentification(void)
{
  uint8_t status=0;
  
  /*Configure EEPROM SPI with default configuration*/
  EepromSpiInitialization();
  
  /* Try this procedure for both FKIxxx and XNUCLEO -S2868 boards */
  for (int i=0; i<2; i++)   
  {
    /*Switch to new CS configuration only if previoisly conf failed */
    if (i==1  &&  status==0)
      EepromCsXnucleoPinInitialization();   
      
    /* try to get the status of the EEPROM */
    status = EepromStatus();
    if((status&0xF0) == EEPROM_STATUS_SRWD) {
      /* if it is EEPROM_STATUS_SRWD, ok the EEPROM is present and ready to work */
      status=1;
    }
    else
    {
      EepromWriteEnable();
      SdkDelayMs(10);
      /* else the bit may be not set (first time we see this EEPROM), try to set it*/
      status = EepromSetSrwd();
      SdkDelayMs(10);
      /*check again*/
      status = EepromStatus();

      if((status&0xF0) == EEPROM_STATUS_SRWD) { // 0xF0 mask [SRWD 0 0 0]
        /* if it is EEPROM_STATUS_SRWD, ok the EEPROM is present and ready to work */
        status=1;
      }
      else
      {
        /* else no EEPROM is present */
        status = 0;
      }
    }
  }

  return status;
}


void S2LPManagementSetBand(uint8_t value)
{
  s_RfModuleBand = value;
}

uint8_t S2LPManagementGetBand(void)
{
  return s_RfModuleBand;
}

void S2LPManagementSetOffset(int32_t value)
{
  s_RfModuleOffset=value;
}

int32_t S2LPManagementGetOffset(void)
{
  return s_RfModuleOffset;
}

void S2LPManagementIdentificationRFBoard(void)
{
  uint8_t tmp;
  StatusBytes status;
  
  do{
    /* Delay for state transition */
    for(volatile uint8_t i=0; i!=0xFF; i++);
    
    /* Reads the MC_STATUS register */
    status = SdkEvalSpiReadRegisters(0x8E, 1, &tmp);
  }while(status.MC_STATE!=MC_STATE_READY);
  
  SdkEvalSpiReadRegisters(0xF1, 1, &tmp);
  
    
  s_S2LPCut=(S2LPCutType)tmp;
  
  if((s_S2LPCut==S2LP_CUT_2_0) || (s_S2LPCut==S2LP_CUT_2_1))
  {
    SdkEvalEnterShutdown();
  }

  SdkEvalSetHasEeprom(EepromIdentification());
  
  if(!SdkEvalGetHasEeprom()) // EEPROM is not present
  {
    SdkEvalExitShutdown();
    if(S2LPManagementComputeXtalFrequency()==0)
    {
      /* if it fails force it to 50MHz */
      S2LPRadioSetXtalFrequency(50000000);
    }
  }
  else  // EEPROM found
  {
    //read the memory and set the variable
    uint8_t tmpBuffer[32];
    EepromRead(0x0000, 32, tmpBuffer);
    uint32_t xtal;
    float foffset=0;
    if(tmpBuffer[0]==0 || tmpBuffer[0]==0xFF) {
      /* this one happens in production where the E2PROM is here but blank */
      S2LPManagementEnableTcxo();
      if((s_S2LPCut==S2LP_CUT_2_0) || (s_S2LPCut==S2LP_CUT_2_1))
      {
        SdkEvalExitShutdown();
      }
      S2LPManagementComputeXtalFrequency();
      return;
    }
    switch(tmpBuffer[1]) {
    case 0:
      xtal = 24000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 1:
      xtal = 25000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 2:
      xtal = 26000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 3:
      xtal = 48000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 4:
      xtal = 50000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 5:
      xtal = 52000000;
      S2LPRadioSetXtalFrequency(xtal);
      break;
    case 0xff:
      /* XTAL freqeuncy is custom */
      for(uint8_t i=0;i<4;i++)
      {
        ((uint8_t*)&xtal)[i]=tmpBuffer[30-i];
      }
      S2LPRadioSetXtalFrequency(xtal);
      break;
    default:
      S2LPManagementComputeXtalFrequency();
      break;
    }
        
    /* TCXO field */
    if(tmpBuffer[31]==1)
    {
      S2LPManagementSetTcxo(1);
    }
    
    S2LPManagementSetBand(tmpBuffer[3]);
    
    RangeExtType range=RANGE_EXT_NONE;
    if(tmpBuffer[5]==2) 
    {
      range = RANGE_EXT_SKYWORKS_868;
    }
    S2LPManagementSetRangeExtender(range);
    
    EepromRead(0x0021,4,tmpBuffer);
    
    if(*(uint32_t*)tmpBuffer != 0xffffffff)
    { 
      for(uint8_t i=0;i<4;i++)
      {
        ((uint8_t*)&foffset)[i]=tmpBuffer[3-i];
      }
      
      if(foffset<-100000 || foffset>100000)
      {
        foffset=0;
      }
    }
    S2LPManagementSetOffset((int32_t)foffset);
  }
  
  if((s_S2LPCut==S2LP_CUT_2_0) || (s_S2LPCut==S2LP_CUT_2_1))
  {
    SdkEvalExitShutdown();
  }
  
  
  
  
}

void S2LPManagementRcoCalibration(void)
{
  uint8_t tmp[2],tmp2;
  
  //S2LPTimerCalibrationRco(S_ENABLE);
  SdkEvalSpiReadRegisters(0x6D, 1, &tmp2);
  tmp2 |= 0x01;
  SdkEvalSpiWriteRegisters(0x6D, 1, &tmp2);  

  //S2LPCmdStrobeStandby();
  SdkEvalSpiCommandStrobes(0x63);
  SdkDelayMs(100);
  //S2LPCmdStrobeReady();
  SdkEvalSpiCommandStrobes(0x62);

  do
  {
    SdkEvalSpiReadRegisters(0x8D, 1, tmp);
  }
  while((tmp[0]&0x10)==0);
  
  
  
  SdkEvalSpiReadRegisters(0x94, 2, tmp);
  SdkEvalSpiReadRegisters(0x6F, 1, &tmp2);
  tmp[1]=(tmp[1]&0x80)|(tmp2&0x7F);
  
  SdkEvalSpiWriteRegisters(0x6E, 2, tmp);
  
  
  //S2LPTimerCalibrationRco(S_DISABLE);
  SdkEvalSpiReadRegisters(0x6D, 1, &tmp2);
  tmp2 &= 0xFE;
  SdkEvalSpiWriteRegisters(0x6D, 1, &tmp2);

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

/**
* @}
*/


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
