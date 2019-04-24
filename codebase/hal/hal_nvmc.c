/**
 *  hal_nvmc.c : NVMC HAL
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#include "hal_nvmc.h"

#include "common_util.h"
#include "log.h"

#define PAGE_START_ADDR_SUFFIX 0x1000

uint32_t hal_nvmc_erase_page (uint32_t page_start_address)
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
    uint32_t end_address = (CEIL_DIV(((uint32_t)p_destination + size_of_data), 4))*4; //
    uint32_t no_of_words = (end_address - start_address)/4;
    log_printf("Start Address : %x, End Address : %x, Size in bytes : %d, No of words : %d\n",
               start_address, end_address, size_of_data, no_of_words);
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
    
    if(no_of_words == 1)
    {
        //special case.
        NRF_NVMC->CONFIG = NVMC_CONFIG_WEN_Ren;
        while(NRF_NVMC->READY != NVMC_READY_READY_Ready);
        return;
    }
    
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
