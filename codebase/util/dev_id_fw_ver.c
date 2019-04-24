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


#include "dev_id_fw_ver.h"
#include "nrf.h"

static const fw_ver_t fw_ver =
{
        .major = (uint8_t) (FW_VER/10000),
        .minor = (uint8_t) ((FW_VER%10000)/100),
        .build = (uint8_t) (FW_VER%100)
};

dev_id_t * dev_id_get(void)
{
    return (dev_id_t *)&NRF_UICR->CUSTOMER[0];
}

fw_ver_t * fw_ver_get(void)
{
    return (fw_ver_t *)(&fw_ver);
}
