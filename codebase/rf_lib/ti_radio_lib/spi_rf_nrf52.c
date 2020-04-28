/*
 *  spi_rf_nrf52.c : <Write brief>
 *  Copyright (C) 2013 Texas Instruments Incorporated - http://www.ti.com/
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


/******************************************************************************
 * INCLUDES
 */
#include "spi_rf_nrf52.h"
#include "hal_spim.h"

#include "stdint.h"
#include "string.h"
#include "hal_gpio.h"
#include "log.h"
#include "hal_nop_delay.h"

#include "stdint.h"
/******************************************************************************
 * LOCAL FUNCTIONS
 */

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
    hal_spim_tx_rx (&cmd, 1, &rc, 1);
    while(hal_spim_is_busy ());

    return(rc);
}
