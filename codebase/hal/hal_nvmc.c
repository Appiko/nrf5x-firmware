/* 
 * File:   hal_nvmc.c
 * Copyright (c) 2018 Appiko
 * Created on 11 October, 2018, 6:41 PM
 * Author:  Tejas Vasekar (https://github.com/tejas-tj)
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 * this list of conditions and the following disclaimer in the documentation
 * and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its contributors
 * may be used to endorse or promote products derived from this software without
 * specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR 
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF 
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN 
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) 
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE
 */

#include "hal_nvmc.h"

#include "common_util.h"
#include "log.h"

#define PAGE_START_ADDR_SUFFIX 0x1000

int hal_nvcm_erase_page (uint32_t page_start_address)
{
    if((page_start_address % PAGE_START_ADDR_SUFFIX) != 0)
    {
        log_printf("Invalid address\n");
        return 1;
    }
    else
    {
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Een;
        while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
        NRF_NVMC->ERASEPAGE = page_start_address;
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
        return 0;
    
    }
}

void hal_nvmc_write_data (void * p_destination, void * p_source, uint32_t size_of_data)
{
    uint32_t start_address = ((uint32_t)p_destination/4)*4;  // to make it to floor
    uint32_t length_in_words = CEIL_DIV(size_of_data, 4);
    uint32_t end_address = (CEIL_DIV(((uint32_t)p_destination + length_in_words*4), 4))*4; //
    uint32_t no_of_words = (end_address - start_address)/4;
    log_printf("Start Address : %x, End Address : %x, Length : %d, No of words : %d\n",
               start_address, end_address, length_in_words, no_of_words);
    //To write the first word. Before writing check if data is aligned with Word lengths
    //And if not add proper masking.
    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Wen;
    while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
    uint32_t word_to_write;
    uint32_t * loc_to_write = (uint32_t *)start_address;
    uint32_t head_offset = ((uint32_t)p_destination - start_address)%4;
    uint32_t * data_to_write = (uint32_t *)((uint32_t)p_source);
    word_to_write = *((uint32_t *)(data_to_write));
    for(uint32_t cnt = 0; cnt < head_offset; cnt++)
    {
        word_to_write = (word_to_write << 8) | 0x000000FF;
    }
    *loc_to_write = word_to_write;
    loc_to_write++;

    data_to_write = (uint32_t *)((uint32_t)data_to_write - head_offset);
     //words in middlle can be written directly.
    for(int32_t i = 1; i< (no_of_words - 1); i++)
    {
        data_to_write++;
        word_to_write = (*(uint32_t*)(data_to_write));
        *loc_to_write = word_to_write;
        loc_to_write++;
    }
    //Before writing last word add proper masking at the end if data is not 
    //ending at word length
    //for masking we will first remove unnecessary data. and then it'll be sent back 
    //to it's original desired location in word with mask
    uint32_t tail_offset = ((no_of_words*4) - size_of_data - head_offset)%4;
    data_to_write++;
    word_to_write = *(data_to_write);
    //Note: Please suggest any other logic if you come up with other than 2 loop logic
    for(uint32_t cnt = 0; cnt < tail_offset; cnt++)
    {
        word_to_write = (word_to_write << 8);
    }
    for(uint32_t cnt=0; cnt <tail_offset; cnt++)
    {
        word_to_write = (word_to_write >> 8) | 0xFF000000;
    }
    *loc_to_write = word_to_write;

    NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
    while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
}
