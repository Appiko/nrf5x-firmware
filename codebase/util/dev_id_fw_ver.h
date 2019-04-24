/**
 *  dev_id_fw_ver.h : Retrieve the device ID and firmware version
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
 * @addtogroup group_util
 * @{
 *
 * @defgroup group_dev_id_fw_ver Retrieve the device ID and firmware version
 * @brief For a product this module retrieves the device ID programmed in the board
 *  level test at factory and the firmware version provided at compile time.
 * @{
 */


#ifndef CODEBASE_UTIL_DEV_ID_FW_VER_H_
#define CODEBASE_UTIL_DEV_ID_FW_VER_H_

#include "stdint.h"

/**
 * Device ID format specification. All values are in ASCII.
 */
typedef struct
{
    /// Two characters
    /// "SP" for SensePi
    /// "SB" for SenseBe
    uint8_t prod_code[2];
    /// Two numbers (0 to 99) for hardware revision
    uint8_t prod_rev[2];
    /// Factory code (0 to 99)
    /// "00" for Jaaga
    /// "01" for Lion Circuits
    uint8_t factory_code[2];
    /// Last two digits of the year of manufacturing
    uint8_t year[2];
    /// Month of manufacturing
    uint8_t month[2];
    /// Day of manufacturing
    uint8_t day[2];
    /// Serial number of the board being manufactured on that particular day
    /// (0000 to 9999)
    uint8_t serial_no[4];
}__attribute__ ((packed)) dev_id_t ;

/**
 * Firmware version number format as per specification at semver.org
 */
typedef struct
{
    uint8_t major;
    uint8_t minor;
    uint8_t build;
}__attribute__ ((packed)) fw_ver_t ;

/**
 * @brief Gets a pointer to the location where the Device ID is stored
 * @return The said pointer
 * @note The firmware must be compiled by passing the firmware version as
 *  a macro FW_VER at compile time.
 */
dev_id_t * dev_id_get(void);

/**
 * @brief Gets a pointer to the location where the Device ID is stored
 * @return The said pointer
 * @note The device must be programmed with the device ID in UICR 0 to 3
 *  during manufacturing
 */
fw_ver_t * fw_ver_get(void);

#endif /* CODEBASE_UTIL_DEV_ID_FW_VER_H_ */

/**
 * @}
 * @}
 */
