/*
 *  byte_frame.c : <Write brief>
 *  Copyright (C) 2020  Appiko
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

#ifndef BYTE_FRAME_H
#define BYTE_FRAME_H

#include "stdint.h"
#include "stdbool.h"

bool encodeFrame(const uint8_t *bytesToEncode, uint16_t len,
        void (*encode_done)(const uint8_t * encoded_data,uint16_t len));

void decodeFrame(const uint8_t *bytesToDecode, uint16_t len,
        void (*decode_done)(const uint8_t * decoded_data,uint16_t len));



#endif /* BYTE_FRAME_H */
