/*
 *  mcp4012_x.c
 *  Creadted on: 28-Mar-2018
 *  Copyright (c) 2018 Appiko
 *  File Author: Tejas Vasekar (https://github.com/tejas-tj)
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice, this
 *  list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice, this
 *  list of conditions and the following disclaimer in the documentation and/or other 
 *  materials provided with the distribution.
 * 
 *  3. Neither the name of the copyright holder nor the names of its contributors may
 *  be used to endorse or promote products derived from this software without specific
 *  prior written permission.
 * 
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND ANY 
 *  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES
 *  OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT
 *  SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED
 *  TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR 
 *  BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 *  CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN 
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH 
 *  DAMAGE.
 */

/**
 * @file mcp4012_x.c
 * @brief This file contains definations for the funtions declared in mcp4012_x.h.
 * It also contains other supporting functions. These funtions are useful to drive
 * MCP4012 properly. 
 */

#include "mcp4012_x.h"
#include "boards.h"
#include "nrf.h"
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "string.h"
#include "stdint.h"
/**
 * @brief This function is used to define a simple SPI based two wire protocol.
 */
static void simple_spi_init();

/**
 * @brief This function is used to transfer value passed by mcp4012_set_value() to
 * mcp4012 using simple SPI based two wire protrocol.
 * @param data[] This parameter is to be transferred over SPI
 * @param len This parameter define length for the data stream which is to be transfered 
 */
static void simple_spi_transmit(uint8_t *data, uint32_t len);

#define MAX_SIZE 16 

#define NO_OF_STEPS 64

#define SIZE_OF_WORD (NO_OF_STEPS/MAX_SIZE)

#define ALL_AA 0xAA

#define ALL_00 0x00

static uint8_t data_to_be_sent[MAX_SIZE];
static uint32_t UD_bar;
static uint32_t CS_bar;
static uint32_t SCK_pin;

void mcp4012_init(uint32_t CS_bar_pin_no, uint32_t UD_bar_pin_no, uint32_t SCK_pin_no)
{
	hal_gpio_cfg_output(CS_bar_pin_no, 1);
	hal_gpio_cfg_output(UD_bar_pin_no, 1);
	hal_gpio_cfg_output(SCK_pin_no, 1);
	UD_bar = UD_bar_pin_no;
	CS_bar = CS_bar_pin_no;
	SCK_pin = SCK_pin_no;

}

void mcp4012_set_value(uint32_t value_sent_by_user)
{
	uint32_t i =0;
	uint32_t word_no = 0;
	uint32_t last_seq = 0;
	uint32_t last_word_no = 0; 
	uint32_t wiper_value = 0;
	/*Loop to set value of all the bytes in data_to_be_sent as 0b10101010*/   
//	memset(data_to_be_sent, 0xAAAAAAAA, sizeof(data_to_be_sent)/sizeof(data_to_be_sent[0]);

 /*   for(i=0; i < MAX_SIZE; i++)
	{
		data_to_be_sent[i] = 0xAA;	
	}*/
	memset(data_to_be_sent, ALL_AA, MAX_SIZE);
	/* Part to set wiper value to maximum */
	hal_gpio_pin_set(CS_bar);
	hal_nop_delay_us(1);
	hal_gpio_pin_set(UD_bar);
	hal_nop_delay_us(1);			//T(LCU)
	hal_gpio_pin_clear(CS_bar);
	hal_nop_delay_us(1);			//T(LCUF)
	hal_gpio_pin_clear(UD_bar);
	hal_nop_delay_us(2);			//T(LCUR)-T(LCUF)	  
	simple_spi_transmit(data_to_be_sent, 16);
	hal_gpio_pin_set(CS_bar);
	
	/* As we are setting wiper value to maximum, to set value to different value
	   we need to decrease it by (maximum_wiper_value - value_sent_by_user) */
	wiper_value = NO_OF_STEPS - value_sent_by_user;
	word_no = wiper_value/SIZE_OF_WORD;
	last_seq = wiper_value % SIZE_OF_WORD;
	last_word_no = word_no+1; 

  	/* To set data_to_be_sent[0 ... (byte-1)] as 0b10101010 */
	memset(data_to_be_sent, ALL_AA, word_no);
	/* To set data_to_be_sent[byte_no ... 15] as 0b0000000 */
	memset(&data_to_be_sent[word_no], ALL_00, (MAX_SIZE-word_no));
	/* Loop to set value for data_to_be_sent[byte_no] */
	for(i = 0; i < last_seq; i++)
	{
		data_to_be_sent[word_no] = 0x80 | data_to_be_sent[word_no] >> 2;
	}

	/* Part to decrease wiper value by (maximum - wiper_value_given_by_user) */
	hal_gpio_pin_set(UD_bar);
	hal_nop_delay_us(1);
	hal_gpio_pin_set(CS_bar);
	hal_gpio_pin_clear(UD_bar);
	hal_nop_delay_us(1);			//T(LCU)
	hal_gpio_pin_clear(CS_bar);
	hal_nop_delay_us(2);			//T(LCUR)
	simple_spi_transmit(data_to_be_sent, last_word_no);	
	hal_gpio_pin_set(CS_bar);
}


static void simple_spi_init()
{
	NRF_SPIM0->TASKS_SUSPEND  = 1;
	NRF_SPIM0->TASKS_STOP = 1;
   	NRF_SPIM0->PSEL.MOSI = UD_bar | (0 << 31);//PIN MOSI_Pin selected as MOSI
	NRF_SPIM0->PSEL.SCK = SCK_pin | (0 << 31);
	NRF_SPIM0->CONFIG = (SPI_CONFIG_ORDER_MsbFirst << SPI_CONFIG_ORDER_Pos)
						|(SPI_CONFIG_CPHA_Trailing << SPI_CONFIG_CPHA_Pos)
						|(SPI_CONFIG_CPOL_ActiveHigh << SPI_CONFIG_CPOL_Pos);
	NRF_SPIM0->FREQUENCY = SPI_FREQUENCY_FREQUENCY_M1 << SPI_FREQUENCY_FREQUENCY_Pos;
	NRF_SPIM0->TXD.LIST = 1 << 0;
}

static void simple_spi_transmit(uint8_t *data, uint32_t len)
{
	simple_spi_init();
	NRF_SPIM0->TXD.PTR = (uint32_t ) data;
	NRF_SPIM0->TXD.MAXCNT = len;
	NRF_SPIM0->ENABLE = 0x07 << 0x00;
	NRF_SPIM0->TASKS_START = 0x01; 
	while(!(NRF_SPIM0->EVENTS_ENDTX));   
	NRF_SPIM0->EVENTS_ENDTX = 0;
	NRF_SPIM0->ENABLE = 0x00 << 0x00; 
	NRF_SPIM0->TASKS_STOP = 0x01;
}
