/******************************************************************************
 *  Filename: radio_drv.h
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
#include "stdint.h"


/* Initialize the radio hardware */
int radio_init(uint8_t config_select);

/* Prepare the radio with a packet to be sent */
int radio_prepare(uint8_t *payload, uint16_t payload_len);

/* Send the packet that has previously been prepared (used for exact timing)*/
int radio_transmit(void);

/* Enter recieve mode */
int radio_receive_on(void);

/* Prepare & transmit a packet in same call (slightly worse timing jitter) */
int radio_send(uint8_t *payload, uint16_t payload_len);

/* Read a received packet into a buffer */
int radio_read(uint8_t * buf, uint8_t *buf_len);

/* Perform a Clear-Channel Assessment (CCA) to find out if channel is clear */
int radio_channel_clear(void);

/* Wait for radio to become idle (currently receiving or transmitting) */
int radio_wait_for_idle(uint16_t max_hold);

/* Check if the radio driver has just received a packet */
int radio_pending_packet(void);

/* Clear the flag that the driver has just received a packet */
int radio_clear_pending_packet(void);

/* Change rf transmit power of radio */
int radio_set_pwr(int tx_pwr);

/* Change channel of radio */
int radio_set_freq(uint64_t freq);

/* Idle the radio, used when leaving low power modes (below)*/
int radio_idle(void);

/* Put the radio into sleep mode */
int radio_sleep(void);

/* Wake the radio from sleep mode */
int radio_wakeup(void);

/* Force PLL calibration, used enabling manual calibration for ultra low power */
int radio_calibrate_on(void);

/* extract the frequency error estimate of the previous packet */
int radio_freq_error(void);

/* Function to check certain status flag */
int radio_check_status_flag (uint8_t status_bits);

/* Function to get RSSI Value */
int radio_get_rssi_val  ();
