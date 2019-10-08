/*
 *  slot_manage.h : Module to keep track of operation conditions for a device
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

#ifndef CODEBASE_PERIPHERAL_MODULES_SLOT_MANAGE_H_
#define CODEBASE_PERIPHERAL_MODULES_SLOT_MANAGE_H_
/**
 * @addtogroup group_peripheral_modules
 * @{
 *
 * @defgroup group_slot_manage Operational condition management
 *
 * @brief Module to keep track of operational conditions (i.e. Time of day, 
 * ambient light) of Appiko device.
 * @{
 */


#include "time_tracker.h"

/** Maximum number of slots which can be managed by the module */
#define SLOT_MANAGE_MAX_SLOTS 8

/** If all slots are active then module is not working properly */
#define SLOT_MANAGE_INVALID_SLOTS 0xFF

/** List of operation conditions which are being managed by the module */
typedef enum
{
    /** Operation Condition : Ambient light cut-off */
    SLOT_MANAGE_AMBI_LIGHT,
    /** Operation Condition : Time of day */
    SLOT_MANAGE_TIME_OF_DAY,
}slot_list_t;

/** Structure to store Condition slots information */
typedef struct
{
    /** Operation condition selection @ref slot_list_t */
    slot_list_t slot_sel;
    /** Slot number for given condition. Should be less than @ref SLOT_MANAGE_MAX_SLOTS */
    uint32_t slot_no;
    /** Starting condition for slot */
    uint32_t start_cond;
    /** Ending condition for slot */
    uint32_t end_cond;
}slot_manage_slot_t;

/**
 * @brief Function to initiate new operation condition slot
 * @param new_slot Structure pointer to data type @ref slot_manage_slot_t
 * storing new slots information
 */
void slot_manage_set_slot (slot_manage_slot_t * new_slot);

/**
 * @breif Function to get status of slots
 * @return bitwise status of all the slots. 
 * @retval if [i]th bit is 1 then [i]th condition slot is active.
 */
uint8_t slot_manage_get_active_slots ();

/**
 * @brief Function to check if there are any changes in active slots.
 * @return bitwise status of all slots.
 * @retval if [i]th bit is 1 then [i]th condition slot is active.
 */
uint8_t slot_manage_check_update ();

/**
 * @brief Function to store ADC pin number
 * @param light_sense_adc ADC pin number. @ref simple_adc_input_t 
 */
void slot_manage_set_light_sense_pin (uint32_t light_sense_adc);

#endif /* CODEBASE_PERIPHERAL_MODULES_SLOT_MANAGE_H_ */

/**
 * @}
 * @}
 */
