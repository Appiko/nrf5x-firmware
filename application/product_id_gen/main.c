/*
 *  main.c : Application to write Device ID in UICR
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

#include "boards.h"
#include "nrf.h"
#include "board_no.h"

int main()
{
	NRF_NVMC->CONFIG = 1;
	NRF_UICR->CUSTOMER[0] = BOARD_NO_1;
	NRF_UICR->CUSTOMER[1] = BOARD_NO_2;
	NRF_UICR->CUSTOMER[2] = BOARD_NO_3;
	NRF_UICR->CUSTOMER[3] = BOARD_NO_4;
	NRF_NVMC->CONFIG = 0;
	while(1);
}
