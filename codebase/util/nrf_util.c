/*
 *  nrf_util.c
 *
 *  Created on: 31-Jan-2018
 *
 *  Copyright (c) 2018, Appiko
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software without
 *  specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#include "nrf_util.h"
#include "nrf.h"
#if defined(SOFTDEVICE_PRESENT)
#include "nrf_nvic.h"
/** The variable referenced by nrf_nvic.h header file */
nrf_nvic_state_t nrf_nvic_state = {0};
#endif

void nrf_util_critical_region_enter(uint8_t * is_critical_entered)
{
#if defined(SOFTDEVICE_PRESENT)
    uint8_t is_sd_enabled;
    sd_softdevice_is_enabled(&is_sd_enabled);
    // Would be coming from the SENSING mode
    if(is_sd_enabled)
    {
        (void) sd_nvic_critical_region_enter(is_critical_entered);
    }
    else
    {
        __disable_irq();
    }
#else
    __disable_irq();
#endif
}

void nrf_util_critical_region_exit(uint8_t critical_entered)
{
#if defined(SOFTDEVICE_PRESENT)
    uint8_t is_sd_enabled;
    sd_softdevice_is_enabled(&is_sd_enabled);
    // Would be coming from the SENSING mode
    if(is_sd_enabled)
    {
        (void) sd_nvic_critical_region_exit(critical_entered);
    }
    else
    {
        __enable_irq();
    }
#else
    __enable_irq();
#endif
}


