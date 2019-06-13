/******************************************************************************
 *  Filename: hal_spi_rf.c
 *
 *  Description: Implementation file for common spi access with the CCxxxx
 *               transceiver radios using trxeb. Supports CC1101/CC112X radios
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


/******************************************************************************
 * INCLUDES
 */
#include "hal_spi_rf.h"
#include "hal_spim.h"

#include "stdint.h"
#include "string.h"
#include "hal_gpio.h"
#include "log.h"
#include "hal_nop_delay.h"


/******************************************************************************
 * LOCAL FUNCTIONS
 */
//static void trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t len) ;

void trxRfSpiInterfaceInit()
{
    log_printf("%s\n", __func__);
    hal_gpio_cfg_output (RF_CS_PIN, 1);
    hal_gpio_cfg_output (RF_SCK_PIN, 1);
    hal_gpio_cfg_input (RF_MISO_PIN, HAL_GPIO_PULL_UP);
    hal_gpio_cfg_output (RF_MOSI_PIN, 1);
    hal_spim_init_t default_spim_config = 
    {
        .csBar_pin = RF_CS_PIN,
        .miso_pin = RF_MISO_PIN,
        .mosi_pin = RF_MOSI_PIN,
        .sck_pin = RF_SCK_PIN,
        .freq = HAL_SPIM_FREQ_2M,
        .spi_mode = HAL_SPIM_SPI_MODE0,
        .byte_order = HAL_SPIM_MSB_FIRST,
        .en_intr = 0,
        .irq_priority = APP_IRQ_PRIORITY_HIGHEST,
        .tx_done_handler = NULL,
        .rx_done_handler = NULL,
        
    };
    hal_spim_init (&default_spim_config);
    
}


/*******************************************************************************
 * @fn          trx8BitRegAccess
 *
 * @brief       This function performs a read or write from/to a 8bit register
 *              address space. The function handles burst and single read/write
 *              as specfied in addrByte. Function assumes that chip is ready.
 *
 * input parameters
 *
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       addrByte - address byte of register.
 * @param       pData    - data array
 * @param       len      - Length of array to be read(TX)/written(RX)
 *
 * output parameters
 *
 * @return      chip status
 */
rfStatus_t trx8BitRegAccess(uint8_t accessType, uint8_t addrByte, uint8_t *pData, uint16_t len)
{
	uint8_t readValue;

//	/* Pull CS_N low and wait for SO to go low before communication starts */
//	RF_SPI_BEGIN();
//	while(RF_PORT_IN & RF_MISO_PIN);
//	/* send register address byte */
//	RF_SPI_TX(accessType|addrByte);
//	RF_SPI_WAIT_DONE();
//	/* Storing chip status */
//	readValue = RF_SPI_RX();
//	trxReadWriteBurstSingle(accessType|addrByte,pData,len);
//	RF_SPI_END();
//	/* return the status byte value */
  uint8_t tx_buff[257];
  tx_buff[0] = accessType|addrByte;
  uint8_t rx_buff[257];
  
  
  if((accessType&RADIO_READ_ACCESS) == RADIO_READ_ACCESS)  
  {
      memset (&tx_buff[1], 0x00, len);
      hal_spim_tx_rx (tx_buff, len+1, rx_buff, 1 + len);
      while(hal_spim_is_busy ());
      memcpy(pData,rx_buff  + 1, len); 
  }
  else if((accessType&RADIO_WRITE_ACCESS) == RADIO_WRITE_ACCESS)
  {
      memcpy (&tx_buff[1], pData, len);
      hal_spim_tx_rx (tx_buff, len+1, rx_buff, 2);
      while(hal_spim_is_busy ());
  }
  
//  ((uint8_t*)&status)[1]=rx_buff[0];
//  ((uint8_t*)&status)[0]=rx_buff[1];
  readValue = rx_buff[0];
  
	return(readValue);
}

/******************************************************************************
 * @fn          trx16BitRegAccess
 *
 * @brief       This function performs a read or write in the extended adress
 *              space of CC112X.
 *
 * input parameters
 *
 * @param       accessType - Specifies if this is a read or write and if it's
 *                           a single or burst access. Bitmask made up of
 *                           RADIO_BURST_ACCESS/RADIO_SINGLE_ACCESS/
 *                           RADIO_WRITE_ACCESS/RADIO_READ_ACCESS.
 * @param       extAddr - Extended register space address = 0x2F.
 * @param       regAddr - Register address in the extended address space.
 * @param       *pData  - Pointer to data array for communication
 * @param       len     - Length of bytes to be read/written from/to radio
 *
 * output parameters
 *
 * @return      rfStatus_t
 */
rfStatus_t trx16BitRegAccess(uint8_t accessType, uint8_t extAddr, uint8_t regAddr, uint8_t *pData, uint8_t len)
{
	uint8_t readValue = 0;

//	RF_SPI_BEGIN();
//	while(RF_PORT_IN & RF_MISO_PIN);
//	/* send extended address byte with access type bits set */
//	RF_SPI_TX(accessType|extAddr);
//	RF_SPI_WAIT_DONE();
//	/* Storing chip status */
//	readValue = RF_SPI_RX();
//	RF_SPI_TX(regAddr);
//	RF_SPI_WAIT_DONE();
//	/* Communicate len number of bytes */
//	trxReadWriteBurstSingle(accessType|extAddr,pData,len);
//	RF_SPI_END();
	/* return the status byte value */
    uint8_t tx_buff[257];
    tx_buff[0] = accessType|extAddr;
    tx_buff[1] = regAddr;
    uint8_t rx_buff[257];


  if((accessType&RADIO_READ_ACCESS) == RADIO_READ_ACCESS)  
  {
      memset (&tx_buff[2], 0x00, len);
      hal_spim_tx_rx (tx_buff, len+2, rx_buff, len+2);
      while(hal_spim_is_busy ());
      memcpy(pData, &rx_buff[2], len);   
  }
  else if((accessType&RADIO_WRITE_ACCESS) == RADIO_WRITE_ACCESS)
  {
      memcpy (&tx_buff[2], pData, len);
      hal_spim_tx_rx (tx_buff, len+2, rx_buff, 2);
      while(hal_spim_is_busy ());
  }
  
//  ((uint8_t*)&status)[1]=rx_buff[0];
//  ((uint8_t*)&status)[0]=rx_buff[1];
    readValue = rx_buff[0];
  
    return(readValue);
}

/*******************************************************************************
 * @fn          trxSpiCmdStrobe
 *
 * @brief       Send command strobe to the radio. Returns status byte read
 *              during transfer of command strobe. Validation of provided
 *              is not done. Function assumes chip is ready.
 *
 * input parameters
 *
 * @param       cmd - command strobe
 *
 * output parameters
 *
 * @return      status byte
 */
rfStatus_t trxSpiCmdStrobe(uint8_t cmd)
{
	uint8_t rc;
//	RF_SPI_BEGIN();
//	while(RF_PORT_IN & RF_MISO_PIN);
//	RF_SPI_TX(cmd);
//	RF_SPI_WAIT_DONE();
//	rc = RF_SPI_RX();
//	RF_SPI_END();
    hal_spim_tx_rx (&cmd, 1, NULL, 0);
    while(hal_spim_is_busy ());
    rc = 0;

    return(rc);
}

/*******************************************************************************
 * @fn          trxReadWriteBurstSingle
 *
 * @brief       When the address byte is sent to the SPI slave, the next byte
 *              communicated is the data to be written or read. The address
 *              byte that holds information about read/write -and single/
 *              burst-access is provided to this function.
 *
 *              Depending on these two bits this function will write len bytes to
 *              the radio in burst mode or read len bytes from the radio in burst
 *              mode if the burst bit is set. If the burst bit is not set, only
 *              one data byte is communicated.
 *
 *              NOTE: This function is used in the following way:
 *
 *              RF_SPI_BEGIN();
 *              while(RF_PORT_IN & RF_SPI_MISO_PIN);
 *              ...[Depending on type of register access]
 *              trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t len);
 *              RF_SPI_END();
 *
 * input parameters
 *
 * @param       none
 *
 * output parameters
 *
 * @return      void
 */
//static void trxReadWriteBurstSingle(uint8_t addr,uint8_t *pData,uint16_t len)
//{
//	uint16_t i;
//	/* Communicate len number of bytes: if RX - the procedure sends 0x00 to push bytes from slave*/
//	if(addr&RADIO_READ_ACCESS)
//	{
//		if(addr&RADIO_BURST_ACCESS)
//		{
//			for (i = 0; i < len; i++)
//			{
//				RF_SPI_TX(0);            /* Possible to combining read and write as one access type */
//				RF_SPI_WAIT_DONE();
//				*pData = RF_SPI_RX();     /* Store pData from last pData RX */
//				pData++;
//			}
//		}
//		else
//		{
//			RF_SPI_TX(0);
//			RF_SPI_WAIT_DONE();
//			*pData = RF_SPI_RX();
//		}
//	}
//	else
//	{
//		if(addr&RADIO_BURST_ACCESS)
//		{
//			/* Communicate len number of bytes: if TX - the procedure doesn't overwrite pData */
//			for (i = 0; i < len; i++)
//			{
//				RF_SPI_TX(*pData);
//				RF_SPI_WAIT_DONE();
//				pData++;
//			}
//		}
//		else
//		{
//			RF_SPI_TX(*pData);
//			RF_SPI_WAIT_DONE();
//		}
//	}
//	return;
//}
