/**
 *  evt_sd_handler.h : Softdevice event handler
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


/**
 * @addtogroup group_sd_assist
 * @{
 *
 * @defgroup group_evt_sd_handler Softdevice event handler
 *
 * @brief Module responsible for making sure that all the events
 *  generated by the Softdevice's SoC and BLE sections are received
 *  and passed on to handlers provided by the application.
 *
 * @{
 */


#ifndef CODEBASE_SD_ASSIST_EVT_SD_HANDLER_H_
#define CODEBASE_SD_ASSIST_SD_EVT_HANDLER_H_

#include "ble.h"

/**
 * @brief Initializes the SWI2 interrupt routine and stores the
 *  handlers for the BLE and SoC events.
 * @param ble_evt_handler
 * @param soc_evt_handler
 */
void evt_sd_handler_init(void (* ble_evt_handler)(ble_evt_t * ble_evt),
                         void (* soc_evt_handler)(uint32_t soc_evt_id));

#endif /* CODEBASE_SD_ASSIST_EVT_SD_HANDLER_H_ */

/**
 * @}
 * @}
 */
