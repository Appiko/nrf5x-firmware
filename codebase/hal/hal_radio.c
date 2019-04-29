/*
 *  hal_radio.c : Basic driver for Radio peripheral
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

#include <stddef.h>
#include "string.h"

#include "hal_radio.h"
#include "nrf.h"

/** Short between Ready event and Start task */
#define SHORT_READY_START			\
		(RADIO_SHORTS_READY_START_Enabled << RADIO_SHORTS_READY_START_Pos)
/** Short between End event and Disable task */
#define SHORT_END_DIS				\
		(RADIO_SHORTS_END_DISABLE_Enabled << RADIO_SHORTS_END_DISABLE_Pos)
/** Short between Disabled event and TX Enable task */
#define SHORT_DIS_TXEN				\
		(RADIO_SHORTS_DISABLED_TXEN_Enabled << RADIO_SHORTS_DISABLED_TXEN_Pos)
/** Short between Disabled event and RX Enable task */
#define SHORT_DIS_RXEN				\
		(RADIO_SHORTS_DISABLED_RXEN_Enabled << RADIO_SHORTS_DISABLED_RXEN_Pos)


/** Maximum number of Bytes a pkt can hold */
#define MAX_PAYLOAD_BYTES 255

/** Static radio address (used in first iteration) */
#define ADDR 0x8E89BE35

/** Position of length parameter in payload */
#define LEN_OFFSET 0

/** Position of data in payload */
#define DATA_OFFSET 1

/**
 * @brief Structure to store payload.
 */
typedef struct
{
    /** Length of payload */
    uint8_t payload_len;
    /** Payload buffer */
    uint8_t p_payload[MAX_PAYLOAD_BYTES - 1];
}payload_t;

/** Global variable to store payload */
static payload_t payload_buff;

/** Function pointer buffer for transmission done function pointer */
void (* pb_tx_done_handler) (void * buff, uint32_t len);
/** Function pointer buffer for reception done function pointer */
void (* pb_rx_done_handler) (void * buff, uint32_t len);

void hal_radio_init (hal_radio_config_t * radio_init_config)
{
    if(radio_init_config->tx_done_handler != NULL)
    {
        pb_tx_done_handler = radio_init_config->tx_done_handler;
    }
    if(radio_init_config->rx_done_handler != NULL)
    {
        pb_rx_done_handler = radio_init_config->rx_done_handler;
    }
    
    /**Enable HF Clock*/
    if(NRF_CLOCK->HFCLKSTAT !=
      ((CLOCK_HFCLKSTAT_STATE_Running << CLOCK_HFCLKSTAT_STATE_Pos) |
      (CLOCK_HFCLKSTAT_SRC_Xtal << CLOCK_HFCLKSTAT_SRC_Pos)))
    {
        NRF_CLOCK->TASKS_HFCLKSTART = 1;
    }
    
    //Power on Radio
    NRF_RADIO->POWER = RADIO_POWER_POWER_Enabled;
    
    //Set mode to BLE 2Mbps
    NRF_RADIO->MODE = (RADIO_MODE_MODE_Ble_2Mbit << RADIO_MODE_MODE_Pos)&RADIO_MODE_MODE_Msk;

    //Configure payload package    
    NRF_RADIO->PCNF0 = ((0UL << RADIO_PCNF0_S0LEN_Pos) & RADIO_PCNF0_S0LEN_Msk)
        | ((8UL << RADIO_PCNF0_LFLEN_Pos) & RADIO_PCNF0_LFLEN_Msk)
        | ((0UL << RADIO_PCNF0_S1LEN_Pos) & RADIO_PCNF0_S1LEN_Msk);

    NRF_RADIO->PCNF1 = ((RADIO_PCNF1_ENDIAN_Little << RADIO_PCNF1_ENDIAN_Pos) & RADIO_PCNF1_ENDIAN_Msk)
        | ((MAX_PAYLOAD_BYTES << RADIO_PCNF1_MAXLEN_Pos) & RADIO_PCNF1_MAXLEN_Msk)
        | ((RADIO_PCNF1_WHITEEN_Enabled << RADIO_PCNF1_WHITEEN_Pos) & RADIO_PCNF1_WHITEEN_Msk)
        | ((3UL << RADIO_PCNF1_BALEN_Pos) & RADIO_PCNF1_BALEN_Msk);

    //Configure Address
    NRF_RADIO->CRCCNF = ((RADIO_CRCCNF_LEN_Three << RADIO_CRCCNF_LEN_Pos) & RADIO_CRCCNF_LEN_Msk)
        | ((RADIO_CRCCNF_SKIPADDR_Skip << RADIO_CRCCNF_SKIPADDR_Pos) & RADIO_CRCCNF_SKIPADDR_Msk);
    
    NRF_RADIO->CRCPOLY = 0x100065B;
    
    
    //Configure freq  
    NRF_RADIO->DATAWHITEIV = 0x8888;
    NRF_RADIO->FREQUENCY = radio_init_config->freq; 



    NRF_RADIO->BASE0 = (ADDR << 8) & 0xFFFFFF00;
    NRF_RADIO->PREFIX0 = (ADDR >> 24) & 0xFF;
    NRF_RADIO->CRCINIT = 0x012345;

    NRF_RADIO->TXADDRESS =((0<<RADIO_TXADDRESS_TXADDRESS_Pos)&RADIO_TXADDRESS_TXADDRESS_Msk);    
    NRF_RADIO->TXPOWER = 4;
    
    NRF_RADIO->RXADDRESSES = ((RADIO_RXADDRESSES_ADDR0_Enabled << RADIO_RXADDRESSES_ADDR0_Pos)
         & RADIO_RXADDRESSES_ADDR0_Msk);

    //enable related interrupts and shorts
    NRF_RADIO->INTENSET |= ((RADIO_INTENSET_CRCERROR_Enabled 
        << RADIO_INTENSET_CRCERROR_Pos) & RADIO_INTENSET_CRCERROR_Msk)
        | ((RADIO_INTENSET_END_Enabled << RADIO_INTENSET_END_Pos) & RADIO_INTENSET_END_Msk)
        | ((RADIO_INTENSET_CRCOK_Enabled << RADIO_INTENSET_CRCOK_Pos) & RADIO_INTENSET_CRCOK_Msk);
    NRF_RADIO->SHORTS = SHORT_READY_START | SHORT_END_DIS;
    NRF_RADIO->PACKETPTR = (uint32_t) &payload_buff;
    NVIC_SetPriority (RADIO_IRQn, radio_init_config->irq_priority);
    NVIC_EnableIRQ (RADIO_IRQn);
}

void hal_radio_set_tx_payload_data (void * p_payload, uint32_t len)
{
    payload_buff.payload_len = len + 1; 
    memcpy (payload_buff.p_payload, p_payload, len);
}

void hal_radio_start_tx ()
{
    NRF_RADIO->TASKS_TXEN = 1;
    
}

void hal_radio_start_rx ()
{
    NRF_RADIO->TASKS_RXEN = 1;
}

void hal_radio_stop ()
{
    NRF_RADIO->TASKS_DISABLE = 1;
}

bool hal_radio_is_on ()
{
    return (NRF_RADIO->STATE != RADIO_STATE_STATE_Disabled) ? true : false;
}


void RADIO_IRQHandler ()
{
    if(NRF_RADIO->EVENTS_CRCOK == 1)
    {
        NRF_RADIO->EVENTS_CRCOK = 0;
        if(pb_rx_done_handler != NULL)
        {
            pb_rx_done_handler (payload_buff.p_payload, payload_buff.payload_len - 1);
        }
    }
    if(NRF_RADIO->EVENTS_CRCERROR == 1)
    {
        NRF_RADIO->EVENTS_CRCERROR = 0;
        
    }
    if(NRF_RADIO->EVENTS_END == 1)
    {
        NRF_RADIO->EVENTS_END = 0;
        if(pb_tx_done_handler != NULL)
        {
            pb_tx_done_handler (payload_buff.p_payload, payload_buff.payload_len - 1);
        }
    }
}
