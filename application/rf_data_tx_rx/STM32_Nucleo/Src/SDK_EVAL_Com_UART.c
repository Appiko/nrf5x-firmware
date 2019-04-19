/**
* @file    SDK_EVAL_Com.c
* @author  VMA division - AMS
* @version 3.2.0
* @date    May 1, 2016
* @brief   This file provides all the low level API to manage SDK UART.
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
#include "SDK_EVAL_Com.h"
#include "SDK_EVAL_Config.h"

/** @addtogroup SDK_EVAL_NUCLEO
* @{
*/


/** @addtogroup SDK_EVAL_Com
* @{
*/


/** @defgroup SDK_EVAL_Com_Private_TypesDefinitions             SDK EVAL Com Private Types Definitions
* @{
*/

/**
* @}
*/


/** @defgroup SDK_EVAL_Com_Private_Defines                      SDK EVAL Com Private Defines
* @{
*/

/**
* @}
*/


/** @defgroup SDK_EVAL_Com_Private_Macros                       SDK EVAL Com Private Macros
* @{
*/

/**
* @}
*/

/** @defgroup SDK_EVAL_Com_Private_Variables                    SDK EVAL Com Private Variables
* @{
*/

#define RECEIVE_QUEUE_SIZE              NUCLEO_UARTx_RX_QUEUE_SIZE


static uint8_t rxQ[RECEIVE_QUEUE_SIZE];
static uint16_t rxHead = 0;
static uint16_t rxTail = 0;
volatile uint16_t rxUsed = 0;
static UART_HandleTypeDef huart;
static DMA_HandleTypeDef dma_handle;

/**
* @}
*/




/** @defgroup SDK_EVAL_Com_Private_Functions                            SDK EVAL Com Private Functions
* @{
*/

void prepareDmaTx(void){}

/**
* @brief  Configures UART port in RX DMA mode and TX blocking mode.
* @param  None
* @retval None.
*/
void SdkEvalComInit(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;
  
  NUCLEO_UARTx_GPIO_CLK_ENABLE();
  NUCLEO_UARTx_CLK_ENABLE();
  NUCLEO_UARTx_DMA_CLK_ENABLE();
  
  dma_handle.Instance=NUCLEO_UARTx_RX_DMA_CHANNEL;
#ifdef USE_STM32L0XX_NUCLEO
  dma_handle.Init.Request=NUCLEO_UARTx_RX_DMA_REQUEST;
#endif
  dma_handle.Init.Direction=DMA_PERIPH_TO_MEMORY;
  dma_handle.Init.PeriphInc=DMA_PINC_DISABLE;
  dma_handle.Init.MemInc=DMA_MINC_ENABLE;
  dma_handle.Init.PeriphDataAlignment=DMA_PDATAALIGN_BYTE;
  dma_handle.Init.MemDataAlignment=DMA_MDATAALIGN_BYTE;
  dma_handle.Init.Mode=DMA_CIRCULAR;
  dma_handle.Init.Priority=DMA_PRIORITY_LOW;
  HAL_DMA_Init(&dma_handle);
  
  GPIO_InitStructure.Mode      = GPIO_MODE_AF_PP;
  GPIO_InitStructure.Pull      = GPIO_PULLUP;
  GPIO_InitStructure.Speed     = GPIO_SPEED_HIGH;
  
  GPIO_InitStructure.Pin       = NUCLEO_UARTx_RX_PIN;
  GPIO_InitStructure.Alternate = NUCLEO_UARTx_AF;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  GPIO_InitStructure.Pin       = NUCLEO_UARTx_TX_PIN;
  GPIO_InitStructure.Alternate = NUCLEO_UARTx_AF;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  
  huart.Init.BaudRate=SDK_EVAL_UART_BAUDRATE;
  huart.Init.WordLength=UART_WORDLENGTH_8B;
  huart.Init.StopBits=UART_STOPBITS_1;
  huart.Init.Parity=UART_PARITY_NONE;
  huart.Init.HwFlowCtl=UART_HWCONTROL_NONE;
  huart.Init.Mode=UART_MODE_TX_RX;
  __HAL_LINKDMA(&huart, hdmarx, dma_handle);
    
  huart.Instance=NUCLEO_UARTx;
  
  
  HAL_UART_Init(&huart);
  
  __HAL_UART_ENABLE(&huart);
  
  
  
  HAL_UART_Receive_DMA(&huart,rxQ,RECEIVE_QUEUE_SIZE);
  
}


void SdkEvalComBaudrate(uint32_t baudrate)
{
  huart.Init.BaudRate=baudrate;
  __HAL_UART_DISABLE(&huart);
  HAL_UART_Init(&huart);
  __HAL_UART_ENABLE(&huart);
}


#ifdef __ICCARM__
#include <yfuns.h>
#include <stdint.h>
#define stdout _LLIO_STDOUT

// IAR Standard library hook for serial output
size_t __write(int handle, const unsigned char * buffer, size_t size)
{
  
  /* This template only writes to "standard out" and "standard err",
   * for all other file handles it returns failure. */
  if (handle != _LLIO_STDOUT && handle != _LLIO_STDERR) {
    return _LLIO_ERROR;
  }
  if (buffer == 0) {
    uint8_t c=0;
    HAL_UART_Transmit(&huart, &c, 1, 100);
    return 0;
  }
  
  
  HAL_UART_Transmit(&huart, (uint8_t*)buffer, size, 300);

  return size;
}

size_t fflush(int handle)
{
  return __write(_LLIO_STDOUT, NULL, 0);
}



// IAR Standard library hook for serial input
size_t __read(int handle, unsigned char * buffer, size_t size)
{
  int nChars = 0;
  
  /* This template only reads from "standard in", for all other file
  * handles it returns failure. */
  if (handle != _LLIO_STDIN)
  {
    return _LLIO_ERROR;
  }
  
  if(huart.hdmarx)
  {
    rxHead=(RECEIVE_QUEUE_SIZE-huart.hdmarx->Instance->CNDTR)%RECEIVE_QUEUE_SIZE;
    
    if(rxHead>=rxTail)
      rxUsed=rxHead-rxTail;
    else
      rxUsed=RECEIVE_QUEUE_SIZE-rxTail+rxHead;
    
  }
  
  for(nChars = 0; (rxUsed>0) && (nChars < size); nChars++) {
    *buffer++ = rxQ[rxTail];
    rxTail = (rxTail+1) % RECEIVE_QUEUE_SIZE;
    rxUsed--;
  }
  
  return nChars;
}

void enqueueRxChars(unsigned char * buffer, uint16_t size)
{
  while (( size > 0 ) && (rxUsed < (RECEIVE_QUEUE_SIZE-1))) {
      rxQ[rxHead] = *buffer++;
      rxHead = (rxHead+1) % RECEIVE_QUEUE_SIZE;
      rxUsed++;
      size--;
  }
}

uint8_t __io_getcharNonBlocking(uint8_t *data)
{
  if (__read(_LLIO_STDIN,data,1))
    return 1;
  else
    return 0;
}

void __io_putchar( char c )
{
  __write(_LLIO_STDOUT, (unsigned char *)&c, 1);
}

int __io_getchar()
{
  unsigned char c;
  __read(_LLIO_STDIN, &c, 1);
  return (int)(c);
}

void __io_flush( void )
{
  __write(_LLIO_STDOUT, NULL, 0);
}

#else
#ifdef __CC_ARM

#include <stdio.h>
/* keil debug port defines */
#define ITM_Port8(n)    (*((volatile unsigned char *)(0xE0000000+4*n)))
#define ITM_Port16(n)   (*((volatile unsigned short*)(0xE0000000+4*n)))
#define ITM_Port32(n)   (*((volatile unsigned long *)(0xE0000000+4*n)))
#define DEMCR           (*((volatile unsigned long *)(0xE000EDFC)))
#define TRCENA          0x01000000

struct __FILE { int handle; /* Add whatever needed */ };
FILE __stdout;
FILE __stdin;

/* KEIL fputc implementation template allowing to redirect printf output towards serial port (UART/USB) */
int fputc(int c, FILE *f) {

  HAL_UART_Transmit(&huart, (uint8_t*)&c, 1, 300);
  
  return 1;
  
}

uint8_t __io_getcharNonBlocking(uint8_t *data)
{
  int c = fgetc(&__stdin);
  
  if (c == EOF)
    return 0;
  else {
		*data = (uint8_t)c;
    return 1;
	}
}/* end serialReadByte() */

int fgetc (FILE *f) {
  int data = -1;
	int nChars = 0;
  
   
  if(huart.hdmarx)
  {
    rxHead=(RECEIVE_QUEUE_SIZE-huart.hdmarx->Instance->CNDTR)%RECEIVE_QUEUE_SIZE;
    
    if(rxHead>=rxTail)
      rxUsed=rxHead-rxTail;
    else
      rxUsed=RECEIVE_QUEUE_SIZE-rxTail+rxHead;
    
  }
  
  for(nChars = 0; (rxUsed>0) && (nChars < 1); nChars++) {
    data = rxQ[rxTail];
    rxTail = (rxTail+1) % RECEIVE_QUEUE_SIZE;
    rxUsed--;
  }
  
  return data;
}


#endif
#endif

uint8_t append_to_buf(void)
{
  uint8_t c;
  
  HAL_UART_Receive(&huart, &c, 1, 100);
  rxQ[rxHead] = c;
  rxHead=(rxHead+1)%RECEIVE_QUEUE_SIZE;
  rxUsed++;
  
  if(c=='\n' || c=='\r')
    return 1;
  
  return 0;
  
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
