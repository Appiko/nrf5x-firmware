/**
 *  nrf_util.c : nRF specific utilities
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

#include "nrf_util.h"
#include "nrf.h"
#if defined(SOFTDEVICE_PRESENT)
#include "nrf_nvic.h"
#include "nrf_sdm.h"
/** The variable referenced by nrf_nvic.h header file */
nrf_nvic_state_t nrf_nvic_state = {0};
#endif

void nrf_util_critical_region_enter(uint8_t * is_critical_entered)
{
#if defined(SOFTDEVICE_PRESENT)
    (void) sd_nvic_critical_region_enter(is_critical_entered);
#else
    __disable_irq();
#endif
}

void nrf_util_critical_region_exit(uint8_t critical_entered)
{
#if defined(SOFTDEVICE_PRESENT)
    (void) sd_nvic_critical_region_exit(critical_entered);
#else
    __enable_irq();
#endif
}


