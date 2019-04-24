/**
 *  nrf_assert.c : static and runtime assertion
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

#include "nrf_assert.h"

#ifdef DEBUG

#include "log.h"
#include <stdnoreturn.h>

noreturn void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{
    log_printf("Assertion at line %d in file %s.\n", line_num,
            file_name);
    while (1)
    {
    }
}

#else

void assert_nrf_callback(uint16_t line_num, const uint8_t * file_name)
{

}

#endif /* DEBUG flag as compiler flag */
