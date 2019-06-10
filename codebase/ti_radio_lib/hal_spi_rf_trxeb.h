/******************************************************************************
*  Filename: hal_spi_rf_trxeb.h
*
*  Description: Common header file for spi access to the different tranceiver
*               radios. Supports CC1101/CC112X radios
*				 
*  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
*
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*    Redistributions of source code must retain the above copyright
*    notice, this list of conditions and the following disclaimer.
*
*    Redistributions in binary form must reproduce the above copyright
*    notice, this list of conditions and the following disclaimer in the
*    documentation and/or other materials provided with the distribution.
*
*    Neither the name of Texas Instruments Incorporated nor the names of
*    its contributors may be used to endorse or promote products derived
*    from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
*  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
*  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
*  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
*  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
*  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
*  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
*  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
*  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*
*******************************************************************************/

#ifndef HAL_SPI_RF_TRXEB_H
#define HAL_SPI_RF_TRXEB_H

#ifdef __cplusplus
extern "C" {
#endif

/******************************************************************************
 * INCLUDES
 */
#include <msp430.h>
#include "hal_types.h"
#include "hal_defs.h"


/******************************************************************************
 * DEFINE THE TRANSCEIVER TO USE
 */
#define USE_CC1101                     /* use the CC110x transciever commands */
//#define USE_CC112X                     /* use the CC112x transciever commands */
#define RF_XTAL 26000                  /* default is 26000 for CC1101 */
                                       /* 32000 for CC1120 */
									   /* 40000 for CC1125 */
//#define ENABLE_RANGE_EXTENDER          /* use external range extender */


/******************************************************************************
 * CONSTANTS
 */

/* Transceiver SPI signal */
#define     RF_PORT_SEL            P3SEL
#define     RF_PORT_OUT            P3OUT
#define     RF_PORT_DIR            P3DIR
#define     RF_PORT_IN             P3IN

#define     RF_MOSI_PIN            BIT1
#define     RF_MISO_PIN            BIT2
#define     RF_SCLK_PIN            BIT3

/* Transceiver chip select signal */
#define     RF_CS_N_PORT_SEL       P3SEL
#define     RF_CS_N_PORT_DIR       P3DIR
#define     RF_CS_N_PORT_OUT       P3OUT
#define     RF_CS_N_PIN            BIT0

/* Transciever optional reset signal */
#define     RF_RESET_N_PORT_SEL    P2SEL
#define     RF_RESET_N_PORT_DIR    P2DIR
#define     RF_RESET_N_PORT_OUT    P2OUT
#define     RF_RESET_N_PIN         BIT6

/* CC1190 Control signals */
#define    RF_LNA_EN_PxOUT         P1OUT
#define    RF_LNA_EN_PxDIR         P1DIR
#define    RF_LNA_EN_PIN           BIT6

#define    RF_PA_EN_PxOUT          P2OUT
#define    RF_PA_EN_PxDIR          P2DIR
#define    RF_PA_EN_PIN            BIT7

/* Transceiver interrupt configuration */
#define     RF_PORT_VECTOR         PORT1_VECTOR
#define     RF_GDO_OUT             P1OUT
#define     RF_GDO_DIR             P1DIR
#define     RF_GDO_IN              P1IN
#define     RF_GDO_SEL             P1SEL
#define     RF_GDO_PxIES           P1IES
#define     RF_GDO_PxIFG           P1IFG
#define     RF_GDO_PxIE            P1IE
#define     RF_GDO_PIN             BIT3

/* Optional button interrupt configuration */
#define     BUTTON_VECTOR          PORT2_VECTOR
#define     BUTTON_OUT             P2OUT
#define     BUTTON_DIR             P2DIR
#define     BUTTON_IN              P2IN
#define     BUTTON_SEL             P2SEL
#define     BUTTON_PxIES           P2IES
#define     BUTTON_PxIFG           P2IFG
#define     BUTTON_PxIE            P2IE
#define     BUTTON_PIN             BIT1
#define     BUTTON_REN             P2REN

/* Macro to enable LEDs */
#define     LED1_PxOUT      P4OUT
#define     LED1_PxDIR      P4DIR
#define     LED1_PIN        BIT0
#define     LED2_PxOUT      P4OUT
#define     LED2_PxDIR      P4DIR
#define     LED2_PIN        BIT1
#define     LED3_PxOUT      P4OUT
#define     LED3_PxDIR      P4DIR
#define     LED3_PIN        BIT2
#define     LED4_PxOUT      P4OUT
#define     LED4_PxDIR      P4DIR
#define     LED4_PIN        BIT3

#define     HAL_LED1_OFF()     LED1_PxOUT |= LED1_PIN
#define     HAL_LED2_OFF()     LED2_PxOUT |= LED2_PIN
#define     HAL_LED3_OFF()     LED3_PxOUT |= LED3_PIN
#define     HAL_LED4_OFF()     LED4_PxOUT |= LED4_PIN

#define     HAL_LED1_ON()      LED1_PxOUT &= ~LED1_PIN
#define     HAL_LED2_ON()      LED2_PxOUT &= ~LED2_PIN
#define     HAL_LED3_ON()      LED3_PxOUT &= ~LED3_PIN
#define     HAL_LED4_ON()      LED4_PxOUT &= ~LED4_PIN

#define     HAL_LED1_TOGGLE()  LED1_PxOUT ^= LED1_PIN
#define     HAL_LED2_TOGGLE()  LED2_PxOUT ^= LED2_PIN
#define     HAL_LED3_TOGGLE()  LED3_PxOUT ^= LED3_PIN
#define     HAL_LED4_TOGGLE()  LED4_PxOUT ^= LED4_PIN

#define     RADIO_BURST_ACCESS      0x40
#define     RADIO_SINGLE_ACCESS     0x00
#define     RADIO_READ_ACCESS       0x80
#define     RADIO_WRITE_ACCESS      0x00

/* Bit fields in the chip status byte */
#define STATUS_CHIP_RDYn_BM              0x80
#define STATUS_STATE_BM                  0x70
#define STATUS_FIFO_BYTES_AVAILABLE_BM   0x0F

/******************************************************************************
 * MACROS
 */

/* Macros for Tranceivers(TRX) */
#define RF_SPI_BEGIN()              st( RF_CS_N_PORT_OUT &= ~RF_CS_N_PIN; NOP(); )
#define RF_SPI_TX(x)                st( UCB0IFG &= ~UCRXIFG; UCB0TXBUF= (x); )
#define RF_SPI_WAIT_DONE()          st( while(!(UCB0IFG & UCRXIFG)); )
#define RF_SPI_WAIT_TX_DONE()       st( while(!(UCB0IFG & UCTXIFG)); )
#define RF_SPI_RX()                 UCB0RXBUF
#define RF_SPI_WAIT_MISO_LOW(x)     st( uint8 count = 200; \
                                           while(RF_PORT_IN & RF_SPI_MISO_PIN) \
                                           { \
                                              __delay_cycles(5000); \
                                              count--; \
                                              if (count == 0) break; \
                                           } \
                                           if(count>0) (x) = 1; \
                                           else (x) = 0; )

#define RF_SPI_END()                st( NOP(); RF_CS_N_PORT_OUT |= RF_CS_N_PIN; )


/******************************************************************************
 * TYPEDEFS
 */

typedef struct
{
  uint16  addr;
  uint8   data;
}registerSetting_t;

typedef uint8 rfStatus_t;

/******************************************************************************
 * PROTOTYPES
 */

void trxRfSpiInterfaceInit(uint8 prescalerValue);
rfStatus_t trx8BitRegAccess(uint8 accessType, uint8 addrByte, uint8 *pData, uint16 len);
rfStatus_t trxSpiCmdStrobe(uint8 cmd);

/* CC112X specific prototype function */
rfStatus_t trx16BitRegAccess(uint8 accessType, uint8 extAddr, uint8 regAddr, uint8 *pData, uint8 len);

#ifdef  __cplusplus
}
#endif

#endif
