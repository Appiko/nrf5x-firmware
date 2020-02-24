/*
 *  rf_spi_hw.h : <Write brief>
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

#ifndef RF_SPI_HW_H
#define RF_SPI_HW_H

#include "stdint.h"

typedef struct
{
    uint32_t mosi_pin;
    uint32_t miso_pin;
    uint32_t sclk_pin;
    uint32_t csn_pin;
    app_irq_priority_t irq_priority;
}rf_spi_init_t;

/**
 * @param Function to initialize SPI communication with chip
 * @param p_spi_init Pointer to sturcture of type @refer rf_spi_init_t
 * @return Status
 */
uint32_t rf_spi_init (rf_spi_init_t * p_spi_init);

#endif /* RF_SPI_HW_H */
