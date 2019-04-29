/*
 *  hal_radio.h : Basic driver for Radio peripheral
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

#ifndef HAL_RADIO_H
#define HAL_RADIO_H

#include "stdbool.h"
#include "nrf_util.h"
#include "stdint.h"

typedef struct
{
    uint32_t freq;
    app_irq_priority_t irq_priority;
    void (* tx_done_handler) (void * p_buff, uint32_t len);
    void (* rx_done_handler) (void * p_buff, uint32_t len);
    
}hal_radio_config_t;

/***/
void hal_radio_init (hal_radio_config_t * radio_init_config);

/***/
void hal_radio_set_payload_data (void * p_payload, uint32_t len);

/***/
void hal_radio_start_tx ();

/***/
void hal_radio_start_rx ();

void hal_radio_stop ();


//For future development

/***/
//void hal_radio_update_freq (uint32_t new_freq);

#endif /* HAL_RADIO_H */
