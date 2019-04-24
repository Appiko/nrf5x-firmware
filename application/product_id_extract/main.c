/*
 *  main.c : Application to read Device ID from UICR
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

#include "nrf.h"
#include "log.h"
#include "boards.h"

int main()
{
	log_init();
	log_printf("\nSTART\n");
	log_printf("UICR_0 : %8x\n", NRF_UICR->CUSTOMER[0]);
	log_printf("UICR_1 : %8x\n", NRF_UICR->CUSTOMER[1]);
	log_printf("UICR_2 : %8x\n", NRF_UICR->CUSTOMER[2]);
	log_printf("UICR_3 : %8x\n", NRF_UICR->CUSTOMER[3]);
	log_printf("END\n");
	while(1);
} 
