/******************************************************************************
 *  Filename: cc1101_utils.c
 *
 *  Description: Implementation file for entering various test modes and
 *               support functions needed for the demostration software
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
#include "stdio.h"
#include "stdlib.h"
#include "msp430.h"
#include "cc1101_def.h"
#include "hal_spi_rf.h"

#ifdef USE_CC1101

/******************************************************************************
* @fn          set_tx_unmodulated_test_mode
*
* @brief       Configure transceiver for continuous unmodulated data test mode
*
* input parameters
*
* @param       void
*
* output parameters
*
* @return      void
*/
void set_tx_unmodulated_test_mode(void) {
  unsigned char reg_access;

  reg_access = 0x32;
  trx8BitRegAccess(RADIO_WRITE_ACCESS, PKTCTRL0, &reg_access, 1);

  reg_access = 0x33;
  trx8BitRegAccess(RADIO_WRITE_ACCESS, MDMCFG2, &reg_access, 1);

}

/******************************************************************************
* @fn          set_tx_modulated_test_mode
*
* @brief       Configure transceiver for continuous modulated data test mode
*
* input parameters
*
* @param       void
*
* output parameters
*
* @return      void
*/
void set_tx_modulated_test_mode(void) {
   unsigned char reg_access;

   reg_access = 0x26;
   trx8BitRegAccess(RADIO_WRITE_ACCESS, PKTCTRL0, &reg_access, 1);
  
}

/******************************************************************************
* @fn          radio_get_rssi
*
* @brief       Get and calculate the RSSI from unsigned char value
*
* input parameters
*
* @param       void
*
* output parameters
*
* @return      int - rssi value in dB
*/
int radio_get_rssi(void) {
  int rssi;
  unsigned char cc_rssi;
  
  trx8BitRegAccess(RADIO_READ_ACCESS | RADIO_BURST_ACCESS, RSSI, &cc_rssi, 1);
  
  if (cc_rssi >= 128) {
    rssi = ((cc_rssi-256)>>1) - 72;
  } else {
    rssi = (cc_rssi>>1) - 72;
  }
  return rssi; 
}

/******************************************************************************
* @fn          get_device_id
*
* @brief       Gets the device ID by reading the version and partnum registers
*
* input parameters
*
* @param       void
*
* output parameters
*
* @return      char - device id define
*/
char get_device_id(void) {
  unsigned char ret_partnum;
  unsigned char ret_version;
  
  trx8BitRegAccess(RADIO_READ_ACCESS+RADIO_BURST_ACCESS, VERSION, &ret_version, 1);
  trx8BitRegAccess(RADIO_READ_ACCESS+RADIO_BURST_ACCESS, PARTNUM, &ret_partnum, 1);

  switch (ret_partnum) {
  case 0:
    if(ret_version == 0x04) {
      return DEV_CC1101;
    } 
    if(ret_version == 0x07) {
      return DEV_CC1101;
    } 
    if(ret_version == 0x06) {
      return DEV_CC430x;
    }    
    if(ret_version == 0x00) {
      return DEV_CC1100;
    }
    break;
  case 128:
    if(ret_version == 0x03) {
      return DEV_CC2500;
    }
    break;
  default:
    break;
  }
  
  return DEV_UNKNOWN;
}

/******************************************************************************
* @fn          set_rf_packet_length
*
* @brief       Configure the radio to handle fixed packets with a certain
*              packet length.
*
* input parameters
*
* @param       unsigned char length - length of fixed payload
*
* output parameters
*
* @return      0 - no_error
*/
unsigned char set_rf_packet_length(unsigned char length) {
  unsigned char reg_value;

  /* make sure we are in fixed packet mode */
  reg_value = 0x04;
  trx8BitRegAccess(RADIO_WRITE_ACCESS, PKTCTRL0, &reg_value, 1);

  /* set the fixed packet length */
  trx8BitRegAccess(RADIO_WRITE_ACCESS, PKTLEN, &length, 1);

  return (0);
}

#endif
