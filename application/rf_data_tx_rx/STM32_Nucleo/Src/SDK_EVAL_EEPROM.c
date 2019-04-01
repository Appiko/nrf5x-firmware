/**
* @file    SPIRIT_SDK_EEPROM.c
* @author  VMA division - AMS
* @version 3.2.0
* @date    May 1, 2016
* @brief   SDK EVAL eeprom management
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
#include "SDK_EVAL_EEPROM.h"

/**
* @addtogroup SDK_EVAL_NUCLEO
* @{
*/


/**
* @defgroup SDK_EEPROM              SDK EEPROM Management
* @{
*/


/** @defgroup SPI_Private_Functions
* @{
*/

static SPI_HandleTypeDef EepromSpiHandle;
static uint8_t s_eeprom  = 0;

/*-----------------To Manage Board Selection-----------------------*/
static uint8_t BoardType = 0; //0 for SDK (default) 1 for XNUCLEO
/*      To Store the state of the main CS reg     */
static uint32_t MainCs_ModerGpioReg;
static uint32_t MainCs_OtyperGpioReg;
static uint32_t MainCs_OspeedrGpioReg;
static uint32_t MainCs_PupdrGpioReg;
static uint32_t MainCs_OdrGpioReg;
static uint32_t MainCs_AfrGpioReg[2];


void EepromSPICSLow()
{
  if (BoardType==1) HAL_GPIO_WritePin(EEPROM_SPI_PERIPH_XNUCLEO_CS_PORT, EEPROM_SPI_PERIPH_XNUCLEO_CS_PIN, GPIO_PIN_RESET);
  else HAL_GPIO_WritePin(EEPROM_SPI_PERIPH_CS_PORT, EEPROM_SPI_PERIPH_CS_PIN, GPIO_PIN_RESET);
}      

void EepromSPICSHigh()
{
  if (BoardType==1) HAL_GPIO_WritePin(EEPROM_SPI_PERIPH_XNUCLEO_CS_PORT, EEPROM_SPI_PERIPH_XNUCLEO_CS_PIN, GPIO_PIN_SET);
  else HAL_GPIO_WritePin(EEPROM_SPI_PERIPH_CS_PORT, EEPROM_SPI_PERIPH_CS_PIN, GPIO_PIN_SET);
}   
/**
* @brief  Initializes the SPI for the EEPROM.
*         SPI, MISO, MOSI and SCLK are the same used for the SPIRIT1.
*         This function can be replaced by EepromCsPinInitialization if
*         SpiritSpiInit is called.
* @param  None
* @retval None
*/
void EepromSpiInitialization(void)
{ 
  
  /*----------------------- GPIO Mode Configuration --------------------*/  
  
  GPIO_InitTypeDef GPIO_InitStructure;
  
  /* Enable SPI periph and SCLK, MOSI, MISO and CS GPIO clocks */
  EEPROM_SPI_PERIPH_RCC();
  EEPROM_SPI_PERIPH_MOSI_RCC();
  EEPROM_SPI_PERIPH_MISO_RCC();
  EEPROM_SPI_PERIPH_SCLK_RCC();
  EEPROM_SPI_PERIPH_CS_RCC();
  
  
  /* Configure the AF for MOSI, MISO and SCLK GPIO pins*/
  GPIO_InitStructure.Pin       = EEPROM_SPI_PERIPH_SCLK_PIN;
  GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull      = GPIO_PULLUP;
  GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
  GPIO_InitStructure.Alternate = EEPROM_SPI_PERIPH_SCLK_AF;
  HAL_GPIO_Init(EEPROM_SPI_PERIPH_SCLK_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = EEPROM_SPI_PERIPH_MISO_PIN;
  GPIO_InitStructure.Alternate = EEPROM_SPI_PERIPH_MISO_AF;
  HAL_GPIO_Init(EEPROM_SPI_PERIPH_MISO_PORT, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin = EEPROM_SPI_PERIPH_MOSI_PIN;
  GPIO_InitStructure.Alternate = EEPROM_SPI_PERIPH_MOSI_AF;
  HAL_GPIO_Init(EEPROM_SPI_PERIPH_MOSI_PORT, &GPIO_InitStructure);
  
  
  /*- Freeze the previous status of Main CS (Maybe it is used by the user) -*/
  
  GPIO_TypeDef *MainCsEepromPort;
  MainCsEepromPort = EEPROM_SPI_PERIPH_CS_PORT;
 
  /* Store IO Direction in Input Floting Mode */
  MainCs_ModerGpioReg = MainCsEepromPort->MODER;
           
  /* Store the previous Alternate Function in current IO */ 
  MainCs_AfrGpioReg[0] = MainCsEepromPort->AFR[0] ;
  MainCs_AfrGpioReg[1] = MainCsEepromPort->AFR[1] ;
      
  /* Store the previous value for IO Speed */
  MainCs_OspeedrGpioReg = MainCsEepromPort->OSPEEDR;
      
  /* Store the previous value IO Output Type */
  MainCs_OtyperGpioReg = MainCsEepromPort->OTYPER ;
      
  /* Store the previous Output Value  */
  MainCs_OdrGpioReg = MainCsEepromPort->ODR ;
      
  /* Store the previous Pull-up oand Pull-down value */
  MainCs_PupdrGpioReg = MainCsEepromPort->PUPDR;
  /*--------------------------- GPIO Mode Configuration --------------------*/ 
  
  //Chip Select (CS) GPIO Conf
  GPIO_InitStructure.Pin = EEPROM_SPI_PERIPH_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  HAL_GPIO_Init(EEPROM_SPI_PERIPH_CS_PORT, &GPIO_InitStructure);
  
  
  /* Configure SPI peripheral */
  if(HAL_SPI_GetState(&EepromSpiHandle) == HAL_SPI_STATE_RESET)
  {  
    /* Set the SPI parameters */
    EepromSpiHandle.Instance               = EEPROM_SPI_PERIPH_NB;
    EepromSpiHandle.Init.Mode              = SPI_MODE_MASTER;
    EepromSpiHandle.Init.BaudRatePrescaler = SDK_EVAL_SPI_PRESCALER;
    
    EepromSpiHandle.Init.Direction         = SPI_DIRECTION_2LINES;
    EepromSpiHandle.Init.CLKPhase          = SPI_PHASE_1EDGE;
    EepromSpiHandle.Init.CLKPolarity       = SPI_POLARITY_LOW;
    EepromSpiHandle.Init.CRCCalculation    = SPI_CRCCALCULATION_DISABLED;
    EepromSpiHandle.Init.CRCPolynomial     = 7;
    EepromSpiHandle.Init.DataSize          = SPI_DATASIZE_8BIT;
    EepromSpiHandle.Init.FirstBit          = SPI_FIRSTBIT_MSB;
    EepromSpiHandle.Init.NSS               = SPI_NSS_SOFT;
    EepromSpiHandle.Init.TIMode            = SPI_TIMODE_DISABLED;
    
    if(HAL_SPI_Init(&EepromSpiHandle) != HAL_OK) {
      return;
    } 
    __HAL_SPI_ENABLE(&EepromSpiHandle);    
  }
  
  EepromSPICSHigh();
}

/**
* @brief  Initialization of the CSn pin of the EEPROM.
*         This function is called internally by EepromCsPinInitialization.
* @param  None
* @retval None
*/

void EepromCsPinInitialization(void)
{
    
  
  BoardType = 0;
  
  GPIO_InitTypeDef GPIO_InitStructure;
  /* Configure SPI pin: CS */
  GPIO_InitStructure.Pin = EEPROM_SPI_PERIPH_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(EEPROM_SPI_PERIPH_CS_PORT, &GPIO_InitStructure);
  
  /* Enable CS GPIO clock */
  EEPROM_SPI_PERIPH_CS_RCC();
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
}

/**
* @brief  Initialization of the CSn pin of the EEPROM for XNUCLEO boards.
*     
* @param  None
* @retval None
*/

void EepromCsXnucleoPinInitialization(void)
{
  /*--------------- To Restore the previous statut of CS ----------------*/
  GPIO_TypeDef *MainCsEepromPort;
  MainCsEepromPort = EEPROM_SPI_PERIPH_CS_PORT;
 
  /* Store IO Direction in Input Floting Mode */
  MainCsEepromPort->MODER = MainCs_ModerGpioReg;
           
  /* Store the previous Alternate Function in current IO */ 
  MainCsEepromPort->AFR[0] = MainCs_AfrGpioReg[0];
  MainCsEepromPort->AFR[1]=  MainCs_AfrGpioReg[1];
      
  /* Store the previous value for IO Speed */
  MainCsEepromPort->OSPEEDR = MainCs_OspeedrGpioReg;
      
  /* Store the previous value IO Output Type */
  MainCsEepromPort->OTYPER = MainCs_OtyperGpioReg;
      
  /* Store the previous Output Value  */
  MainCsEepromPort->ODR = MainCs_OdrGpioReg;
      
  /* Store the previous Pull-up oand Pull-down value */
  MainCsEepromPort->PUPDR = MainCs_PupdrGpioReg;

    
  ///* It affects EepromSPICSHigh/Low functions 
  BoardType = 1;
  
  GPIO_InitTypeDef GPIO_InitStructure;
  ///* Configure SPI pin: CS 
  GPIO_InitStructure.Pin = EEPROM_SPI_PERIPH_XNUCLEO_CS_PIN;
  GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStructure.Pull = GPIO_PULLUP;
  GPIO_InitStructure.Speed = GPIO_SPEED_HIGH;
  HAL_GPIO_Init(EEPROM_SPI_PERIPH_XNUCLEO_CS_PORT, &GPIO_InitStructure);
  
  EEPROM_SPI_PERIPH_XNUCLEO_CS_RCC();
  
  //Put the SPI chip select high to end the transaction 
  EepromSPICSHigh();
}

/**
* @brief  Wait polling the SPI until the internal WIP flag is RESET.
*         The flag is SET when a write operation is running.
* @param  None
* @retval None
*/
void EepromWaitEndWriteOperation(void)
{
  uint8_t cmd = EEPROM_CMD_RDSR;
  uint8_t dummy = 0xFF;
  uint8_t status;
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  /* Send command */
  HAL_SPI_TransmitReceive(&EepromSpiHandle, &cmd, &status, 1, 1000);
  
  /* Polling on status register */
  do{
    HAL_SPI_TransmitReceive(&EepromSpiHandle, &dummy, &status, 1, 1000);
  }while(status&EEPROM_STATUS_WIP);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
}


/**
* @brief  Read the status register.
* @param  None
* @retval Status
*/
uint8_t EepromStatus(void)
{
  uint8_t status[2];
  uint8_t cmd[] = {EEPROM_CMD_RDSR, 0xFF};
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  /* Send command */
  HAL_SPI_TransmitReceive(&EepromSpiHandle, cmd, status, 2, 1000);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
  return status[1];

}

/**
* @brief  Set the ERSR status bit.
* @param  None
* @retval Status
*/
uint8_t EepromSetSrwd(void)
{
  uint8_t status[2];
  uint8_t cmd[] = {EEPROM_CMD_WRSR, EEPROM_STATUS_SRWD};
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  /* Send command */
  HAL_SPI_TransmitReceive(&EepromSpiHandle, cmd, status, 2, 1000);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
    
  return status[1];
}

/**
* @brief  Reset the ERSR status bit.
* @param  None
* @retval Status
*/
uint8_t EepromResetSrwd(void)
{
  uint8_t status[2];
  uint8_t cmd[] = {EEPROM_CMD_WRSR, 0};//EEPROM_STATUS_SRWD};
  
  EepromWriteEnable();
  EepromWaitEndWriteOperation();
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  /* Send command */
  HAL_SPI_TransmitReceive(&EepromSpiHandle, cmd, status, 2, 1000);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  EepromWaitEndWriteOperation();
  
  return status[1];
}

/**
* @brief  Set the internal WEL flag to allow write operation.
* @param  None
* @retval None
*/
void EepromWriteEnable(void)
{
  uint8_t cmd = EEPROM_CMD_WREN;
  uint8_t status;
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  /* Send command */
  HAL_SPI_TransmitReceive(&EepromSpiHandle, &cmd, &status, 1, 1000);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
}


/**
* @brief  Read a page of the EEPROM.
*         A page size is 32 bytes.
*         The pages are 256.
*         Page 0 address: 0x0000
*         Page 1 address: 0x0020
*         ...
*         Page 255 address: 0x1FE0
* @param  None
* @retval None
*/
void EepromRead(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t cmd[3];
  uint8_t dummy[255];
  cmd[0] = EEPROM_CMD_READ;
  
  for(uint8_t k=0; k<2; k++) {
    cmd[k+1] = (uint8_t)(nAddress>>((1-k)*8));
  }  
  
  /* Wait the end of a previous write operation */
  EepromWaitEndWriteOperation();
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  /* Write the header bytes and read the status bytes */
  HAL_SPI_TransmitReceive(&EepromSpiHandle, cmd, dummy, 3, 1000);
  
  /* Read the registers according to the number of bytes */
  HAL_SPI_TransmitReceive(&EepromSpiHandle, dummy, pcBuffer, cNbBytes, 1000);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
}


/**
* @brief  Write a page of the EEPROM.
*         A page size is 32 bytes.
*         The pages are 256.
*         Page 0 address: 0x0000
*         Page 1 address: 0x0020
*         ...
*         Page 255 address: 0x1FE0
*         It is allowed to write only a page for each operation. If the bytes
*         exceed the single page location, the other bytes are written at the 
*         beginning.
* @param  None
* @retval None
*/
void EepromWrite(uint16_t nAddress, uint8_t cNbBytes, uint8_t* pcBuffer)
{
  uint8_t cmd = EEPROM_CMD_WRITE, tmp[255];
  uint8_t address[2];
  
  /* Wait the end of a previous write operation */
  EepromWaitEndWriteOperation();
  
  /* SET the WREN flag */
  EepromWriteEnable();
  
  for(uint8_t k=0; k<2; k++) {
    address[k] = (uint8_t)(nAddress>>((1-k)*8));
  }
  //EepromWaitEndWriteOperation();
  
  /* Put the SPI chip select low to start the transaction */
  EepromSPICSLow();
  
  /* Write the header bytes and read the SPIRIT status bytes */
  HAL_SPI_TransmitReceive(&EepromSpiHandle, &cmd, tmp, 1, 1000);
  
  HAL_SPI_TransmitReceive(&EepromSpiHandle, address, tmp, 2, 1000);
  
  HAL_SPI_TransmitReceive(&EepromSpiHandle, pcBuffer, tmp, cNbBytes, 1000);
  
  /* Put the SPI chip select high to end the transaction */
  EepromSPICSHigh();
  
}

/**
* @brief  This function is to query if EEPROM is present or not.
* @param  None
* @retval 1 (yes) or 0 (no).
*/
uint8_t SdkEvalGetHasEeprom(void)
{
  return s_eeprom;
}


/**
* @brief  This function is to set if EEPROM is present or not.
* @param  1 (yes) or 0 (no).
* @retval None
*/
void SdkEvalSetHasEeprom(uint8_t eeprom)
{
  s_eeprom = eeprom;
}


/**
* @}
*/

/**
* @}
*/


/******************* (C) COPYRIGHT 2016 STMicroelectronics *****END OF FILE****/
