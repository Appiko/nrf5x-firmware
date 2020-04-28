/******************************************************************************
 *  Filename: cc112x_drv.c
 *
 *  Description: Radio driver abstraction layer, this uses the same concept
 *               as found in Contiki OS.
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
#include "stdlib.h"
#include "cc112x_def.h"
#include "radio_drv.h"
#include "hal_spi_rf.h"
#include "hal_nop_delay.h"
#include "log.h"
#include "hal_gpio.h"
//#include "cc112x_drv.h"

#ifdef USE_CC112X

/******************************************************************************
 * DEFINES
 */
#define RF_XTAL_FREQ           RF_XTAL       /* XTAL frequency, given in 1kHz steps */
#define RF_LO_DIVIDER          4             /* there is a hardware LO divider CC112x */

/******************************************************************************
 * GLOBALS -  used by the driver
 */
uint8_t rf_end_packet = 0;

/******************************************************************************
 * Configuration extracted from SmartRF Studio version 7 Release 2.0.0
 */
// Address config = No address check
// Performance mode = High Performance
// Packet bit length = 0
// Symbol rate = 1.2
// Whitening = false
// Carrier frequency = 902.750000
// TX power = 15
// Manchester enable = false
// Packet length mode = Fixed
// Packet length = 20
// RX filter BW = 25.000000
// Deviation = 3.997803
// Device address = 0
// Bit rate = 1.2
// Modulation format = 2-FSK
// PA ramping = true
// Append packetinfo = false

const registerSetting_t preferredSettings_1200bps[]=
{
		{IOCFG3,            0xB0},
		{IOCFG2,            0x06},
		{IOCFG1,            0xB0},
		{IOCFG0,            0x06},
		{SYNC3,             0xD3},
		{SYNC2,             0x91},
		{SYNC1,             0xD3},
		{SYNC0,             0x91},
		{SYNC_CFG1,         0x0B},
		{DCFILT_CFG,        0x1C},
		{PREAMBLE_CFG1,     0x18},
		{IQIC,              0xC6},
		{CHAN_BW,           0x08},
		{MDMCFG0,           0x05},
		{SYMBOL_RATE2,      0x43},
		{SYMBOL_RATE1,      0xA9},
		{SYMBOL_RATE0,      0x2A},
		{AGC_REF,           0x20},
		{AGC_CS_THR,        0x19},
		{AGC_CFG1,          0xA9},
		{AGC_CFG0,          0xCF},
		{FIFO_CFG,          0x00},
		{SETTLING_CFG,      0x0B},
		{FS_CFG,            0x12},
		{PKT_CFG2,          0x04},
		{PKT_CFG1,          0x04},
		{PKT_CFG0,          0x00},
		{PA_CFG2,           0x7F},
		{PA_CFG1,           0x56},
		{PA_CFG0,           0x7C},
		{PKT_LEN,           0x14},
		{IF_MIX_CFG,        0x00},
		{FREQOFF_CFG,       0x22},
		{FREQ2,             0x70},
		{FREQ1,             0xD8},
		{FS_DIG1,           0x00},
		{FS_DIG0,           0x5F},
		{FS_CAL1,           0x40},
		{FS_CAL0,           0x0E},
		{FS_DIVTWO,         0x03},
		{FS_DSM0,           0x33},
		{FS_DVC0,           0x17},
		{FS_PFD,            0x50},
		{FS_PRE,            0x6E},
		{FS_REG_DIV_CML,    0x14},
		{FS_SPARE,          0xAC},
		{FS_VCO0,           0xB4},
		{XOSC5,             0x0E},
		{XOSC1,             0x03},
};

// RX filter BW = 100.000000
// Packet bit length = 0
// Deviation = 20.019531
// Packet length mode = Variable
// Packet length = 255
// Carrier frequency = 902.750000
// Manchester enable = false
// TX power = 15
// PA ramping = true
// Device address = 0
// Symbol rate = 38.4
// Address config = No address check
// Bit rate = 38.4
// Modulation format = 2-GFSK
// Whitening = false
// Performance mode = High Performance
// Append packetinfo = false

const registerSetting_t preferredSettings_38400bps[]=
{
		{IOCFG3,            0xB0},
		{IOCFG2,            0x06},
		{IOCFG1,            0xB0},
		{IOCFG0,            0x06},
		{SYNC3,             0xD3},
		{SYNC2,             0x91},
		{SYNC1,             0xD3},
		{SYNC0,             0x91},
		{SYNC_CFG1,         0x08},
		{DEVIATION_M,       0x48},
		{MODCFG_DEV_E,      0x0D},
		{DCFILT_CFG,        0x1C},
		{PREAMBLE_CFG1,     0x18},
		{IQIC,              0x00},
		{CHAN_BW,           0x02},
		{MDMCFG0,           0x05},
		{SYMBOL_RATE2,      0x93},
		{SYMBOL_RATE1,      0xA9},
		{SYMBOL_RATE0,      0x2A},
		{AGC_CS_THR,        0x19},
		{AGC_CFG1,          0xA9},
		{AGC_CFG0,          0xCF},
		{FIFO_CFG,          0x00},
		{SETTLING_CFG,      0x0B},
		{FS_CFG,            0x12},
		{PKT_CFG2,          0x04},
		{PKT_CFG1,          0x04},
		{PKT_CFG0,          0x20},
		{PA_CFG2,           0x7F},
		{PA_CFG1,           0x56},
		{PA_CFG0,           0x7B},
		{PKT_LEN,           0xFF},
		{IF_MIX_CFG,        0x00},
		{FREQOFF_CFG,       0x22},
		{FREQOFF1,          0x00},
		{FREQOFF0,          0x00},
		{FREQ2,             0x70},
		{FREQ1,             0xD8},
		{FREQ0,             0x00},
		{FS_DIG1,           0x00},
		{FS_DIG0,           0x5F},
		{FS_CAL1,           0x40},
		{FS_CAL0,           0x0E},
		{FS_DIVTWO,         0x03},
		{FS_DSM0,           0x33},
		{FS_DVC0,           0x17},
		{FS_PFD,            0x50},
		{FS_PRE,            0x6E},
		{FS_REG_DIV_CML,    0x14},
		{FS_SPARE,          0xAC},
		{FS_VCO0,           0xB4},
		{XOSC5,             0x0E},
		{XOSC1,             0x03},
};

// Device address = 0
// Performance mode = High Performance
// Symbol rate = 50
// RX filter BW = 100.000000
// TX power = 15
// Modulation format = 2-GFSK
// Whitening = false
// Address config = No address check
// Packet length = 255
// Deviation = 24.963379
// Bit rate = 50
// Packet bit length = 0
// PA ramping = true
// Carrier frequency = 902.750000
// Packet length mode = Variable
// Manchester enable = false
// Append packetinfo = false

const registerSetting_t preferredSettings_50kbps[]=
{
		{IOCFG3,            0xB0},
		{IOCFG2,            0x06},
		{IOCFG1,            0xB0},
		{IOCFG0,            0x06},
		{SYNC3,             0xD3},
		{SYNC2,             0x91},
		{SYNC1,             0xD3},
		{SYNC0,             0x91},
		{SYNC_CFG1,         0x08},
		{SYNC_CFG0,         0x17},
		{DEVIATION_M,       0x99},
		{MODCFG_DEV_E,      0x0D},
		{DCFILT_CFG,        0x15},
		{PREAMBLE_CFG1,     0x18},
		{PREAMBLE_CFG0,     0x2A},
		{FREQ_IF_CFG,       0x3A},
		{IQIC,              0x00},
		{CHAN_BW,           0x02},
		{MDMCFG1,           0x46},
		{MDMCFG0,           0x05},
		{SYMBOL_RATE2,      0x99},
		{SYMBOL_RATE1,      0x99},
		{SYMBOL_RATE0,      0x99},
		{AGC_REF,           0x3C},
		{AGC_CS_THR,        0xEF},
		{AGC_CFG1,          0xA9},
		{AGC_CFG0,          0xC0},
		{FIFO_CFG,          0x00},
		{SETTLING_CFG,      0x0B},
		{FS_CFG,            0x12},
		{PKT_CFG2,          0x04},
		{PKT_CFG1,          0x04},
		{PKT_CFG0,          0x20},
		{PA_CFG2,           0x7F},
		{PA_CFG1,           0x56},
		{PA_CFG0,           0x79},
		{PKT_LEN,           0xFF},
		{IF_MIX_CFG,        0x00},
		{FREQOFF_CFG,       0x20},
		{TOC_CFG,           0x0A},
		{FREQOFF1,          0x00},
		{FREQOFF0,          0x00},
		{FREQ2,             0x70},
		{FREQ1,             0xD8},
		{FREQ0,             0x00},
		{FS_DIG1,           0x00},
		{FS_DIG0,           0x5F},
		{FS_CAL1,           0x40},
		{FS_CAL0,           0x0E},
		{FS_DIVTWO,         0x03},
		{FS_DSM0,           0x33},
		{FS_DVC0,           0x17},
		{FS_PFD,            0x50},
		{FS_PRE,            0x6E},
		{FS_REG_DIV_CML,    0x14},
		{FS_SPARE,          0xAC},
		{FS_VCO0,           0xB4},
		{XOSC5,             0x0E},
		{XOSC1,             0x03},
};

// Address Config = No address check 
// Bit Rate = 0.3 
// Carrier Frequency = 915.000000 
// Deviation = 1.998901 
// Device Address = 0 
// Manchester Enable = false 
// Modulation Format = 2-FSK 
// PA Ramping = true 
// Packet Bit Length = 0 
// Packet Length = 255 
// Packet Length Mode = Variable 
// Performance Mode = High Performance 
// RX Filter BW = 8.000000 
// Symbol rate = 0.3 
// TX Power = 15 
// Whitening = false 

const registerSetting_t trial_Settings[]=
{
    {IOCFG3,            0xB0},
    {IOCFG2,            0x06},
    {IOCFG1,            0xB0},
    {IOCFG0,            0x40},
    {SYNC_CFG1,         0x0B},
    {MODCFG_DEV_E,      0x02},
    {DCFILT_CFG,        0x1C},
    {PREAMBLE_CFG1,     0x18},
    {IQIC,              0xC6},
    {CHAN_BW,           0x19},
    {MDMCFG0,           0x05},
    {SYMBOL_RATE2,      0x23},
    {AGC_REF,           0x20},
    {AGC_CS_THR,        0x19},
    {AGC_CFG1,          0xA9},
    {AGC_CFG0,          0xCF},
    {FIFO_CFG,          0x00},
    {FS_CFG,            0x12},
    {PKT_CFG0,          0x20},
    {PA_CFG0,           0x7E},
    {PKT_LEN,           0xFF},
    {IF_MIX_CFG,        0x00},
    {FREQOFF_CFG,       0x22},
    {FREQ2,             0x72},
    {FREQ1,             0x60},
    {FS_DIG1,           0x00},
    {FS_DIG0,           0x5F},
    {FS_CAL1,           0x40},
    {FS_CAL0,           0x0E},
    {FS_DIVTWO,         0x03},
    {FS_DSM0,           0x33},
    {FS_DVC0,           0x17},
    {FS_PFD,            0x50},
    {FS_PRE,            0x6E},
    {FS_REG_DIV_CML,    0x14},
    {FS_SPARE,          0xAC},
    {FS_VCO0,           0xB4},
    {XOSC5,             0x0E},
    {XOSC1,             0x03},
};



/******************************************************************************
 * @fn         radio_init
 *
 * @brief      Initialize the radio hardware
 *
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 */
int radio_init(radio_config_id_t config_select, radio_hw_config_t * hw_config) {

	uint8_t i, writeByte, preferredSettings_length;
	uint32_t bit_rate;
	registerSetting_t *preferredSettings;


	/* Instantiate transceiver RF SPI interface to SCLK ~ 4 MHz */
	/* Input parameter is clockDivider */
	/* SCLK frequency = SMCLK/clockDivider */
    hal_gpio_cfg_output (hw_config->reset_pin, 1);
    hal_gpio_pin_set (hw_config->reset_pin);
    hal_nop_delay_ms (1);
    hal_gpio_pin_clear (hw_config->reset_pin);
    hal_nop_delay_ms (1);
    hal_gpio_pin_set (hw_config->reset_pin);
	trxRfSpiInterfaceInit((rf_spi_hw_t * )hw_config);
    
//	/* remove the reset from the rf device */
//	RF_RESET_N_PORT_SEL &= ~RF_RESET_N_PIN;
//	RF_RESET_N_PORT_DIR |= RF_RESET_N_PIN;
//	RF_RESET_N_PORT_OUT |= RF_RESET_N_PIN;
//
	/* Reset radio */
	trxSpiCmdStrobe(SRES);

	/* give the tranciever time enough to complete reset cycle */
	hal_nop_delay_us (16000);


	switch (config_select) {
	case TI_1120_1K2:
		preferredSettings_length = sizeof(preferredSettings_1200bps)/sizeof(registerSetting_t);
		preferredSettings = (registerSetting_t *)preferredSettings_1200bps;
		bit_rate = 12;
		break;
	case TI_1120_38K4:
		preferredSettings_length = sizeof(preferredSettings_38400bps)/sizeof(registerSetting_t);
		preferredSettings = (registerSetting_t *)preferredSettings_38400bps;
		bit_rate = 384;
		break;
	case TI_1120_50K:
		preferredSettings_length = sizeof(preferredSettings_50kbps)/sizeof(registerSetting_t);
		preferredSettings = (registerSetting_t *)preferredSettings_50kbps;
		bit_rate = 500;
		break;
	case APPIKO_1120_0K3:
		preferredSettings_length = sizeof(trial_Settings)/sizeof(registerSetting_t);
		preferredSettings = (registerSetting_t *)trial_Settings;
		bit_rate = 02;
		break;
//	default:
//		preferredSettings_length = sizeof(preferredSettings_1200bps)/sizeof(registerSetting_t);
//		preferredSettings = (registerSetting_t *)preferredSettings_1200bps;
//		bit_rate = 12;
//		break;
	}

	/* Write registers to radio */
	for(i = 0; i < preferredSettings_length; i++) {

		if(preferredSettings[i].addr < 0x2F) {
			writeByte = preferredSettings[i].data;
			trx8BitRegAccess(RADIO_WRITE_ACCESS, preferredSettings[i].addr, &writeByte, 1);
		} else {
			writeByte = preferredSettings[i].data;
			trx16BitRegAccess(RADIO_WRITE_ACCESS, 0x2F , (0xFF & preferredSettings[i].addr),
					&writeByte, 1);
		}
	}

	/* enable range extender */
#ifdef ENABLE_RANGE_EXTENDER
	range_extender_init();
#endif
    log_printf("%s\n", __func__);

	return bit_rate;
}

/******************************************************************************
 * @fn         radio_prepare
 *
 * @brief      Prepare the radio with a packet to be sent, but do not send
 *
 * input parameters
 *
 * @param       uint8_t *payload     - pointer to payload
 *              uint16_t payload_len - payload length information
 *
 * output parameters
 *
 * @return      0
 *
 */
int radio_prepare(uint8_t *payload, uint16_t payload_len) {
	trx8BitRegAccess(RADIO_WRITE_ACCESS+RADIO_BURST_ACCESS, TXFIFO, payload, payload_len);

	return 0;
}

/******************************************************************************
 * @fn         radio_transmit
 *
 * @brief      Send the packet that has previously been prepared (used for
 *             exact timing)
 *
 * input parameters
 *
 * @param       uint8_t *payload     - pointer to payload
 *              uint16_t payload_len - payload length information
 *
 * output parameters
 *
 * @return      0
 *
 */
int radio_transmit(void) {

	/* Range extender in TX mode */
#ifdef ENABLE_RANGE_EXTENDER
	range_extender_txon();
#endif

	/* Change state to TX, initiating */
	trxSpiCmdStrobe(STX);

	return(0);
}

/******************************************************************************
 * @fn         radio_receive_on
 *
 * @brief      Initiate the RX chain
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 */
int radio_receive_on(void) {

		trxSpiCmdStrobe(SFRX);	                     // Flush RXFIFO
	/* Range extender in RX mode */
#ifdef ENABLE_RANGE_EXTENDER
	range_extender_rxon();
#endif

	/* Strobe RX to initiate the recieve chain */
	trxSpiCmdStrobe(SRX);

	return 0;
}

/******************************************************************************
 * @fn         radio_send
 *
 * @brief      Prepare & transmit a packet in same call
 *
 *
 * input parameters
 *
 * @param       uint8_t *payload
 *              uint16_t payload_len
 *
 * output parameters
 *
 * @return      void
 *
 *
 */
int radio_send(uint8_t *payload, uint16_t payload_len) {

    
    trxSpiCmdStrobe (SFTX);
    
	/* Write packet to TX FIFO */
	trx8BitRegAccess(RADIO_WRITE_ACCESS|RADIO_BURST_ACCESS, TXFIFO, payload, payload_len);
    
	/* Read number of bytes in TX FIFO */
	uint8_t pktLen;

	trx16BitRegAccess(RADIO_READ_ACCESS|RADIO_BURST_ACCESS, 0x2F, 0xff & NUM_TXBYTES, &pktLen, 1);
    log_printf("Pkt : %d, %d\n", payload_len, pktLen);

	/* Range extender in TX mode */
#ifdef ENABLE_RANGE_EXTENDER
	range_extender_txon();
#endif

	/* Strobe TX to send packet */
	trxSpiCmdStrobe(STX);               // Change state to TX, initiating

	return 0;
}

/******************************************************************************
 * @fn         radio_read
 *
 * @brief      Read a received packet into a buffer
 *
 *
 * input parameters
 *
 * @param       uint8_t *buf
 *              uint16_t buf_len
 *
 * output parameters
 *
 * @return      void
 *
 *
 */
int radio_read(uint8_t *buf, uint8_t *buf_len) {
	uint8_t status;
	uint8_t pktLen;

	/* Read number of bytes in RX FIFO */
	trx16BitRegAccess(RADIO_READ_ACCESS, 0x2F, 0xff & NUM_RXBYTES, &pktLen, 1);
//	pktLen = pktLen  & NUM_RXBYTES;    

    log_printf("Pkt :%d, %d\n",*buf_len, pktLen);
	/* make sure the packet size is appropriate, that is 1 -> buffer_size */
	if ((pktLen > 0) && (pktLen <= *buf_len)) {

		/* retrieve the FIFO content */
		trx8BitRegAccess(RADIO_READ_ACCESS|RADIO_BURST_ACCESS, RXFIFO, buf, pktLen);

		/* return the actual length of the FIFO */
		*buf_len = pktLen;

		/* retrieve the CRC status information */
		trx16BitRegAccess(RADIO_READ_ACCESS, 0x2F, 0xff & LQI_VAL, &status, 1);

		/* Return CRC_OK bit */
		status  = status & CRC_OK;
        trxSpiCmdStrobe(SFRX);	                     // Flush RXFIFO

	} else {

		/* if the length returned by the transciever does not make sense, flush it */

        log_printf("Wrong..!!\n");
		*buf_len = 0;                                // Return 0 indicating a failure
		status = 0;                                  // Return 0 indicating a failure
		trxSpiCmdStrobe(SFRX);	                     // Flush RXFIFO

	}

	/* return status information, CRC OK or NOT OK */
	return (status);
}

/******************************************************************************
 * @fn         radio_channel_clear
 *
 * @brief      Perform a Clear-Channel Assessment (CCA) to find out if
 *             channel is clear
 *
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      0  - no carrier found
 *              >0 - carrier found
 *
 */
int radio_channel_clear(void) {
	uint8_t status;

	/* get RSSI0, and return the carrier sense signal */
	trx16BitRegAccess(RADIO_READ_ACCESS, 0x2F, 0xff & RSSI0, &status, 1);

	/* return the carrier sense signal */
	return(status  & 0x04);
}


/******************************************************************************
 * @fn         radio_channel_clear
 *
 * @brief      Wait for end of packet interupt to happen
 *
 *             Timeout is controlled by TimerA running at 8MHz
 *             64000 = 128ms, 32000 = 64ms, 16000 = 32ms
 *             0 = no timeout.
 *
 * input parameters
 *
 * @param       max_hold   :  Watch dog timeout, no end of packet = 0;
 *
 * output parameters
 *
 * @return      timer value:  Interupt happened based on end_of_packet interupt
 *
 */

//COMPLETELY BROKEN
int radio_wait_for_idle(uint16_t max_hold) {

	uint32_t status;
	uint8_t reg_status;

	/* check that we are still in RX mode before entering wait for RX end */
	trx16BitRegAccess(RADIO_READ_ACCESS, 0x2F, 0xff & MARCSTATE, &reg_status, 1);

	/* filter out only the status section of the register values */
	reg_status  = (reg_status & 0x1F);

	/* check for not idle mode */
	if(!(reg_status == MARCSTATE_IDLE)) {

		rf_end_packet = 0;  // initialize global variable for use in this function

//		/* setup the interrupt */
//		RF_GDO_PxIES |= RF_GDO_PIN;       // Int on falling edge (end of pkt)
//		RF_GDO_PxIFG &= ~RF_GDO_PIN;      // Clear flag
//		RF_GDO_PxIE |= RF_GDO_PIN;        // Enable int on end of packet

		/* enabled timeout if requested */
		if(max_hold > 0) {
//			status = hal_nop_delay_ms ((uint)max_hold);   //CHECKOUT WHAT TO DO WITH THIS LINE    // this will timeout either with GDO or timer
		} else {
			// wait for radio to interupt us to continue processing
			status = 0;
//			_BIS_SR(LPM0_bits + GIE);             // Enter LPM0
		}

		/********  Setup the GDO ports to not interupts ****************************/
//		RF_GDO_PxIE &= ~RF_GDO_PIN;              // Disable int on end of packet

	}
	/* Get timer values, however if we did not get a packet in time use 0      */
	if(rf_end_packet == 0) {
		status = max_hold;
	}

#ifdef ENABLE_RANGE_EXTENDER
	range_extender_idle();
#endif

	return status;
}

/******************************************************************************
 * @fn         radio_is_busy
 *
 * @brief      Wait for radio to become idle
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 */
int radio_is_busy(void) {

	// Wait GDO0 to go hi -> sync TX'ed
//	while (!(RF_GDO_IN & RF_GDO_PIN));

	// Wait GDO0 to go low again -> sync TX'ed
//	while (RF_GDO_IN & RF_GDO_PIN);

	// Wait GDO0 to clear -> end of pkt
//	RF_GDO_PxIFG &= ~RF_GDO_PIN;          // After pkt TX, this flag is set.

	return(0);
}

/******************************************************************************
 * @fn         radio_pending_packet
 *
 * @brief      Check if the radio driver has just received a packet
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 */
int radio_pending_packet(void) {

//	RF_GDO_PxIES |= RF_GDO_PIN;       // Int on falling edge (end of pkt)
//	RF_GDO_PxIE |= RF_GDO_PIN;        // Enable int on end of packet

	return rf_end_packet;
}

/******************************************************************************
 * @fn         radio_clear_pending_packet
 *
 * @brief      Clear pending packet indicator
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 */
int radio_clear_pending_packet(void) {

//	RF_GDO_PxIES &= ~RF_GDO_PIN;       // Int on falling edge (end of pkt)
//	RF_GDO_PxIE &= ~RF_GDO_PIN;        // Enable int on end of packet

	rf_end_packet = 0;

	return 0;
}


/******************************************************************************
 * @fn         radio_set_pwr
 *
 * @brief      Set the output power of the CC112x by looking up in table
 *
 * input parameters
 *
 * @param       int tx_pwr
 *
 * output parameters
 *
 * @return      int tx_pwr
 *
 */
int radio_set_pwr(int tx_pwr) {

	return tx_pwr;
}


/******************************************************************************
 * @fn          radio_set_freq
 *
 * @brief       Calculate the required frequency registers and send the using
 *              serial connection to the RF tranceiver.
 *
 * input parameters
 *
 * @param       freq   -  frequency word provided in [kHz] resolution
 *
 * output parameters
 *
 * @return      void
 */
int radio_set_freq(uint64_t freq) {

	uint8_t freq_regs[3];
	uint32_t freq_regs_uint32;
	float f_vco;

	/* Radio frequency -> VCO frequency */
	f_vco = freq * RF_LO_DIVIDER;

	/* Divide by oscillator frequency */
	f_vco = f_vco * (1/(float)RF_XTAL_FREQ);

	/* Multiply by 2^16 */
	f_vco = f_vco * 65536;

	/* Convert value into uint32_t from float */
	freq_regs_uint32 = (uint32_t) f_vco;

	/* return the frequency word */
	freq_regs[2] = ((uint8_t*)&freq_regs_uint32)[0];
	freq_regs[1] = ((uint8_t*)&freq_regs_uint32)[1];
	freq_regs[0] = ((uint8_t*)&freq_regs_uint32)[2];

	/* write the frequency word to the transciever */
	trx16BitRegAccess(RADIO_WRITE_ACCESS | RADIO_BURST_ACCESS, 0x2F, (0xFF & FREQ2), freq_regs, 3);

    log_printf("Here\n");
	return 0;
}


/******************************************************************************
 * @fn         radio_idle
 *
 * @brief      Idle the radio, used when leaving low power modes (below)
 *
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 */
int radio_idle(void) {

#ifdef ENABLE_RANGE_EXTENDER
	range_extender_idle();
#endif

	/* Idle range extender */
//	range_extender_idle();

	/* Force transciever idle state */
	trxSpiCmdStrobe(SIDLE);

	/* Flush the FIFO's */
	trxSpiCmdStrobe(SFRX);
	trxSpiCmdStrobe(SFTX);

	return(0);
}

/******************************************************************************
 * @fn         radio_sleep
 *
 * @brief      Enter sleep mode
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 */
int radio_sleep(void) {

	/* Idle range extender */
#ifdef ENABLE_RANGE_EXTENDER
	range_extender_idle();
#endif

	/* Force transciever idle state */
	trxSpiCmdStrobe(SIDLE);

	/* Enter sleep state on exit */
	trxSpiCmdStrobe(SPWD);

	return(0);
}

/******************************************************************************
 * @fn         radio_wake
 *
 * @brief      Exit sleep mode
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 */
int radio_wake(void) {

	/* Force transciever idle state */
	trxSpiCmdStrobe(SIDLE);

	/* 1 ms delay for letting RX settle */
	hal_nop_delay_us (1000);

	return(0);
}


/******************************************************************************
 * @fn          radio_freq_error
 *
 * @brief       Estimate the frequency error from two complement coded
 *              data to frequency error in Hertz
 *
 * input parameters
 *
 * @param       freq_reg_error -  two complement formatted data from tranceiver
 *
 * output parameters
 *
 * @return      freq_error     - 32 bit signed integer value representing
 *                                frequency error in Hertz
 *
 */
int radio_freq_error(void) {

	uint64_t freq_error_est;
	long freq_error_est_int;
	uint8_t sign, regState, regState1;
	uint32_t freq_reg_error;

    /* Read marcstate to check for frequency error estimate */
	trx16BitRegAccess(RADIO_READ_ACCESS, 0x2F, (0xFF & FREQOFF_EST0), &regState, 1);
	trx16BitRegAccess(RADIO_READ_ACCESS, 0x2F, (0xFF & FREQOFF_EST1), &regState1, 1);

    /* Calculate the frequency error in Hz */
	freq_reg_error = ((uint32_t)regState1 << 8) + regState;

	/* the incoming data is 16 bit two complement format, separate "sign" */
	if (freq_reg_error > 32768) {
		freq_error_est = -(freq_reg_error - 65535);
		sign = 1;
	} else {
		freq_error_est = freq_reg_error;
		sign = 0;
	}

	/* convert the data to hertz format in two steps to avoid integer overuns */
	freq_error_est = (freq_error_est * (RF_XTAL_FREQ/RF_LO_DIVIDER)) >> 8;
	freq_error_est = (freq_error_est * 1000) >> 10;

	/* re-assign the "sign" */
	if(sign == 1) {
		freq_error_est_int = -freq_error_est;
	} else {
		freq_error_est_int = freq_error_est;
	}

	return freq_error_est_int;
}


int radio_check_status_flag (uint8_t status_bits)
{
    uint8_t marc_sts1 ;
	trx16BitRegAccess((RADIO_READ_ACCESS | RADIO_BURST_ACCESS), 0x2F,
                     (0x00FF & MARC_STATUS1), &marc_sts1, 1);
    
    if((status_bits & marc_sts1) == status_bits)
    {
        return 1;
    }
    else
    {
        return 0;
    }
    
}


int radio_get_rssi_val (void)
{
    uint8_t rssi_regs[2];
    uint8_t rssi_val;
    trx16BitRegAccess (RADIO_READ_ACCESS, 0x2F, (0xFF & RSSI1), rssi_regs, 
                       sizeof(rssi_regs));
    if(rssi_regs[1] & RSSI0_RSSI_VALID)
    {
//        rssi_val = ((rssi_regs[0]<<RSSI0_RSSI_3_0_POS) |
//            ((rssi_regs[1]&RSSI0_RSSI_3_0_MSK)>>RSSI0_RSSI_3_0_POS));
//        rssi_val--;
//        rssi_val = 0xFF - rssi_val;
        rssi_val = 0xFF - (rssi_regs[0] -1);
    }
    else
    {
        rssi_val = 0xFF;
    }
//    rssi_val = rssi_regs[0];
    return rssi_val;
}
/******************************************************************************
 * @fn         radio_ISR
 *
 * @brief      Interrupt service routine used by transciever
 *
 * input parameters
 *
 * @param       void
 *
 * output parameters
 *
 * @return      void
 *
 */
//#pragma vector=RF_PORT_VECTOR
//__interrupt void radio_isr(void) {
//
//	if(RF_GDO_PxIFG & RF_GDO_PIN) {
//
//		// Clear LPM0 bits from 0(SR)
//		__bic_SR_register_on_exit(LPM3_bits);
//
//		// clear the interrupt flag
//		RF_GDO_PxIFG &= ~RF_GDO_PIN;
//
//		// indicate that end of packet has been found
//		rf_end_packet = 1;
//	}
//}

#endif
