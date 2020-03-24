/**
 *  display_lib.h : APIs for Display peripheral device
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
 
 *  You should have received a copy of the GNU General Public License
 *  along with this program.  If not, see <https://www.gnu.org/licenses/>.
 */

#ifndef DISPLAY_LIB_H
#define DISPLAY_LIB_H

#include "DEV_Config.h"
#include "ugui.h"
//#include "ugui_config.h"
#include "stdint.h"
#include "stdbool.h"

#ifdef WSEPD_1IN54_BW_V1
#define X_DIM_PIX   (200)
#define Y_DIM_PIX   (200)
#define X_DIM_BYTE  (200)
#define Y_DIM_BYTE  (25)
#define EP1B_BLACK  (0)
#define EP1B_WHITE  (1)
#endif

typedef DEV_Config_t disp_lib_hw_t;

typedef UG_FONT disp_lib_font_t;

typedef enum
{
    E_PAPER_NO_ERR,
    E_PAPER_LINE_EXC,
    E_PAPER_SPACE_EXC,
}disp_lib_status_t;


void disp_lib_init (disp_lib_hw_t * hw_init, disp_lib_font_t * font);

void disp_lib_refresh (void);

void disp_lib_clear (void);

void disp_lib_set_bg_color (UG_COLOR color);

void disp_lib_set_fg_color (UG_COLOR color);

bool disp_lib_is_busy (void);

void disp_lib_set_cursor_loc (uint32_t line_no, uint32_t space);

uint32_t disp_lib_get_current_line (void);

uint32_t disp_lib_get_current_col (void);

disp_lib_status_t disp_lib_putS (char * str, uint32_t len);

disp_lib_status_t disp_lib_putS_at (char * str, uint32_t len,
                                    uint32_t col, uint32_t line);

disp_lib_status_t disp_lib_putC (uint8_t c);

disp_lib_status_t disp_lib_putC_at (uint8_t c, uint32_t col, uint32_t line);

#endif //DISPLAY_LIB_H