/*
 *  sd_evt_handler.c
 *
 *  Created on: 05-Apr-2018
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

#include "stddef.h"
#include "stdbool.h"
#include "nrf_sdm.h"
#include "app_error.h"
#include "common_util.h"
#include "stdint.h"

#include "evt_sd_handler.h"
#include "nrf_assert.h"
#include "nrf_nvic.h"

///The buffer where the ble event data is stored by sd_evt_get
uint32_t ble_evt_buffer[
          CEIL_DIV(BLE_EVT_LEN_MAX(BLE_GATT_ATT_MTU_DEFAULT)
          ,sizeof(uint32_t))];

void (*ble_handler)(ble_evt_t * evt);
void (*soc_handler)(uint32_t evt);

void evt_sd_handler_init
            (void (* ble_evt_handler)(ble_evt_t * ble_evt),
            void (* soc_evt_handler)(uint32_t soc_evt_id))
{
    ASSERT(ble_evt_handler != NULL);
    ASSERT(soc_evt_handler != NULL);
    ble_handler = ble_evt_handler;
    soc_handler = soc_evt_handler;

    uint32_t err_code = sd_nvic_EnableIRQ(SWI2_IRQn);
    APP_ERROR_CHECK(err_code);
}

void SWI2_IRQHandler(void)
{
    bool soc_evts_handled = false;
    while(soc_evts_handled == false)
    {
        uint32_t err_code, evt_id;
        err_code = sd_evt_get(&evt_id);
        if (err_code == NRF_ERROR_NOT_FOUND)
        {
            //No more events
            soc_evts_handled = true;
        }
        else if (err_code != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            // Call the SoC event handler.
            soc_handler(evt_id);
        }
    }

    bool ble_evts_handled = false;
    while(ble_evts_handled == false)
    {
        // Pull event from stack
        uint16_t evt_len = sizeof(ble_evt_buffer);

        uint32_t err_code = sd_ble_evt_get((uint8_t*)ble_evt_buffer, &evt_len);
        if (err_code == NRF_ERROR_NOT_FOUND)
        {
            ble_evts_handled = true;
        }
        else if (err_code != NRF_SUCCESS)
        {
            APP_ERROR_CHECK(err_code);
        }
        else
        {
            // Call the BLE stack event handler.
            ble_handler((ble_evt_t *) ble_evt_buffer);
        }
    }
}
