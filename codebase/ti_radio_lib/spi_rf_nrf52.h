/*
 *  spi_rf_nrf52.h : <Write brief>
 *  Copyright (C) 2019  Appiko
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *  
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */


#include "hal_spim.h"

#ifndef SPI_RF_NRF52_H
#define SPI_RF_NRF52_H

#define USE_CC112X                     /* use the CC112x transciever commands */
#define RF_XTAL 32000                  /* default is 26000 for CC1101 */


//RX
//#define RF_MISO_PIN 3
//#define RF_MOSI_PIN 4
//#define RF_SCK_PIN  22
//#define RF_CS_PIN   23
//#define RF_RESET_PIN    24


#define RF_MISO_PIN 4
#define RF_MOSI_PIN 6
#define RF_SCK_PIN  5
#define RF_CS_PIN   2
#define RF_RESET_PIN   9

#define LED1
#define LED2
#define LED3
#define LED4

#define     RADIO_BURST_ACCESS      0x40
#define     RADIO_SINGLE_ACCESS     0x00
#define     RADIO_READ_ACCESS       0x80
#define     RADIO_WRITE_ACCESS      0x00

#define RF_SPI_BEGIN()              
#define RF_SPI_TX(x)                
#define RF_SPI_WAIT_DONE()          
#define RF_SPI_WAIT_TX_DONE()       
#define RF_SPI_RX()                 
#define RF_SPI_WAIT_MISO_LOW(x)     
#define RF_SPI_END()        


typedef struct
{
  uint16_t  addr;
  uint8_t   data;
}registerSetting_t;

typedef uint8_t rfStatus_t;

void trxRfSpiInterfaceInit(void);
rfStatus_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint16_t len);
rfStatus_t trxSpiCmdStrobe(uint8_t cmd);

/* CC112X specific prototype function */
rfStatus_t trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, uint8_t *pData, uint8_t len);


#endif /* SPI_RF_NRF52_H */

