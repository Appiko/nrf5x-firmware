/*
 *  random_num.h : <Write brief>
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


#ifndef RANDOM_NUM_H
#define RANDOM_NUM_H

#include "stdint.h"

/**
 * @brief Function to generate random number with RNG hw peripheral in given range.
 * @param min Minimum value of range
 * @param max Maximum value of range
 * @return Random number
 * @note Random number generated is in the form of steps of (max - min)/256
 */
uint32_t random_num_generate (uint32_t min, uint32_t max);


#endif /* RANDOM_NUM_H */
