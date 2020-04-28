/******************************************************************************
 *  Filename: hal_spi_rf_trx.h
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
#include "stdint.h"

#if defined (__MSP430G2553__)
  #include "hal_spi_rf_exp430g2.h"
#endif
#if defined (__MSP430F5438A__)
  #include "hal_spi_rf_trxeb.h"
#endif
#if defined (__MSP430F5529__)
  #include "hal_spi_rf_exp5529.h"
#endif

#if defined NRF52810 || NRF52832 || NRF52840
  #include "spi_rf_nrf52.h" 
#endif
// CC Chip versions
#define DEV_UNKNOWN       10
#define DEV_CC1100        11
#define DEV_CC1101        12
#define DEV_CC2500        13
#define DEV_CC430x        14
#define DEV_CC1120        15
#define DEV_CC1121        16
#define DEV_CC1125        17
#define DEV_CC1200        18
#define DEV_CC1201        19
#define DEV_CC1175        20

#define RADIO_GENERAL_ERROR     0x00
#define RADIO_CRC_OK            0x80
#define RADIO_IDLE              0x81
#define RADIO_RX_MODE           0x82
#define RADIO_TX_MODE           0x83
#define RADIO_RX_ACTIVE         0x84
#define RADIO_TX_ACTIVE         0x85
#define RADIO_SLEEP             0x86
#define RADIO_TX_PACKET_RDY     0x87
#define RADIO_CHANNEL_NOT_CLR   0x88
#define RADIO_CHANNEL_IS_CLR    0x89

