/*
 *  hal_ppi.c : Basic HAL driver to provide abstraction for PPI peripheral
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
#include "hal_ppi.h"
#include "nrf.h"


#define MAX_PPI_CH 20

#define PERIPHERAL_VALIDATION 0xF0000000
#define VALID_PERIPHERAL 0x40000000
#define TASK_EVT_VALIDATION_BIT_POS 8
#define TASK_VALIDATION_VAL 0
#define EVT_VALIDATION_VAL 0x100

ppi_setup_status_t hal_ppi_set (hal_ppi_setup_t * setup)
{
    ppi_setup_status_t ppi_status = PPI_SETUP_SUCCESSFUL;
    if(setup->ppi_id > MAX_PPI_CH)
    {
        ppi_status |= PPI_INVALID_CH;
    }
    if(((setup->event & PERIPHERAL_VALIDATION) != VALID_PERIPHERAL) ||
       ((setup->task & PERIPHERAL_VALIDATION) != VALID_PERIPHERAL) ||
       (((setup->fork & PERIPHERAL_VALIDATION) != VALID_PERIPHERAL) &&
        (setup->fork != 0)) )
    {
        ppi_status |= PPI_INVALID_PERIPHERAL;
    }
    if((setup->event & (1 << TASK_EVT_VALIDATION_BIT_POS)) != EVT_VALIDATION_VAL)
    {
        ppi_status |= PPI_INVALID_EVENT;
    }
    if((setup->task & (1 << TASK_EVT_VALIDATION_BIT_POS)) != TASK_VALIDATION_VAL)
    {
        ppi_status |= PPI_INVALID_TASK;
    }
    if((setup->fork & (1 << TASK_EVT_VALIDATION_BIT_POS)) != TASK_VALIDATION_VAL)
    {
        ppi_status |= PPI_INVALID_FORK;
    }
    if(ppi_status == PPI_SETUP_SUCCESSFUL)
    {
        NRF_PPI->CH[setup->ppi_id].EEP = setup->event;
        NRF_PPI->CH[setup->ppi_id].TEP = setup->task;
        if(setup->fork != 0)
        {
            NRF_PPI->FORK[setup->ppi_id].TEP = setup->fork;
        }
    }
    return ppi_status;
}

void hal_ppi_en_ch (uint32_t ppi_id)
{
    NRF_PPI->CHENSET |= 1 << ppi_id;
}

void hal_ppi_dis_ch (uint32_t ppi_id)
{
    NRF_PPI->CHENCLR |= 1 << ppi_id;
}

void hal_ppi_set_event (uint32_t ppi_id, uint32_t new_event)
{
    NRF_PPI->CH[ppi_id].EEP = new_event;
}

void hal_ppi_set_task (uint32_t ppi_id, uint32_t new_task)
{
    NRF_PPI->CH[ppi_id].TEP = new_task;
}

void hal_ppi_set_fork (uint32_t ppi_id, uint32_t new_fork)
{
    NRF_PPI->FORK[ppi_id].TEP = new_fork;
}
