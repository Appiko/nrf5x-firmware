/******************************************************************************
 *  Filename: cc112x_utils.c
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
#include "cc112x_def.h"
#include "hal_spi_rf.h"
#include "log.h"

#ifdef USE_CC112X


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
	uint8_t regs_uint8_t;

	/* disable FIFO mode*/
	regs_uint8_t = 0x06;
	trx8BitRegAccess(RADIO_WRITE_ACCESS , MDMCFG1, &regs_uint8_t, 1);

	/* configure continuous mode*/
	trx8BitRegAccess(RADIO_READ_ACCESS , PA_CFG2, &regs_uint8_t, 1);
	regs_uint8_t = regs_uint8_t | 0x40;
	trx8BitRegAccess(RADIO_WRITE_ACCESS , PA_CFG2, &regs_uint8_t, 1);

	/* disable the modulator */
	regs_uint8_t = 0x01;
	trx16BitRegAccess(RADIO_WRITE_ACCESS , 0x2F, (0xFF & CFM_DATA_CFG), &regs_uint8_t, 1);

	return;
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
	uint8_t regs_uint8_t;

	/* disable FIFO mode*/
	regs_uint8_t = 0x46;
	trx8BitRegAccess(RADIO_WRITE_ACCESS , MDMCFG1, &regs_uint8_t, 1);

	/* disable FIFO mode*/
	regs_uint8_t = 0x05;
	trx8BitRegAccess(RADIO_WRITE_ACCESS , MDMCFG0, &regs_uint8_t, 1);

	/* configure set_tx_modulated_test_mode */
	regs_uint8_t = 0x06;
	trx8BitRegAccess(RADIO_WRITE_ACCESS , PKT_CFG2, &regs_uint8_t, 1);

	regs_uint8_t = 0x40;
	trx8BitRegAccess(RADIO_WRITE_ACCESS , PKT_CFG0, &regs_uint8_t, 1);

	/* load a random byte into the FIFO, this causes the modulator to start */
	trx8BitRegAccess(RADIO_WRITE_ACCESS+RADIO_BURST_ACCESS, TXFIFO, &regs_uint8_t, 1);

	return;
}

/******************************************************************************
* @fn          radio_get_rssi
*
* @brief       Get and calculate the RSSI from uint8_t value
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
	uint8_t cc_rssi;

	trx16BitRegAccess(RADIO_READ_ACCESS , 0x2F, (0xFF & RSSI1), &cc_rssi, 1);

	rssi = cc_rssi;
	if (rssi >= 128) {
		rssi = rssi - 256;
	}
	rssi = rssi - 99;

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
	uint8_t ret_partnum;
	uint8_t ret_version;
	uint8_t ret;

	/*  Read the PARTNUM status register */
	trx16BitRegAccess(RADIO_READ_ACCESS , 0x2F, (0xFF & PARTNUMBER), &ret_partnum, 1);

	/*  Read the PARTNUMBER status register */
	trx16BitRegAccess(RADIO_READ_ACCESS , 0x2F, (0xFF & PARTVERSION), &ret_version, 1);

	switch (ret_partnum) {
	case 0x40:
		ret = DEV_CC1121;
		break;
	case 0x48:
		ret = DEV_CC1120;
		break;
	case 0x58:
		ret = DEV_CC1125;
		break;
	case 0x5A:
		ret = DEV_CC1175;
		break;
	default:
		ret = DEV_UNKNOWN;
		break;
	}

	return ret;
}

/******************************************************************************
* @fn          set_rf_packet_length
*
* @brief       Configure the radio to handle fixed packets with a certain
*              packet length.
*
* input parameters
*
* @param       uint8_t length - length of fixed payload
*
* output parameters
*
* @return      0 - no_error
*/
uint8_t set_rf_packet_length(uint8_t length) {
	uint8_t regs_uint8_t;

	/* configure set_rf_packet_lengthe */
	regs_uint8_t = 0x00;
	trx8BitRegAccess(RADIO_WRITE_ACCESS , PKT_CFG0, &regs_uint8_t, 1);

	/* configure set_rf_packet_length */
	regs_uint8_t = length;
	trx8BitRegAccess(RADIO_WRITE_ACCESS , PKT_LEN, &regs_uint8_t, 1);
    
    regs_uint8_t = 0;
	trx8BitRegAccess(RADIO_READ_ACCESS , PKT_LEN, &regs_uint8_t, 1);
//    log_printf("Pkt Len set to : %d\n", regs_uint8_t);


	return (0);
}

#endif
