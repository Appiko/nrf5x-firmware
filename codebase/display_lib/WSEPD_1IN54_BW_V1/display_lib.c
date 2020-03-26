/**
 *  display_lib.c : APIs for Display peripheral device
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

#include "display_lib.h"

#include "hal_nop_delay.h"
#include "string.h"

#include "EPD_1in54.h"

#define OFFSET_PER_LINE 4

struct disp_lib_txt_t
{
    disp_lib_font_t font;
    uint32_t total_lines;
    uint32_t col_per_lines;
    uint32_t current_col_no;
    uint32_t current_line_no;
}disp_lib_txt;

UG_GUI g_gui;
static uint8_t g_arr_img [(X_DIM_BYTE * Y_DIM_BYTE)];

UG_COLOR g_backg, g_foreg;

uint32_t g_disp_busy;

uint16_t line_to_pix (uint32_t line)
{
    return ((line) ? (line*(disp_lib_txt.font.char_height + 4)) : 4);
}

uint16_t col_to_pix (uint32_t col)
{
    return (col * disp_lib_txt.font.char_width);
}

void set_pix_array (UG_S16 H_cmp,UG_S16 W_cmp,UG_COLOR pixel)
{
    UDOUBLE Addr = (H_cmp / 8) + (W_cmp * Y_DIM_BYTE);
    UBYTE Rdata = g_arr_img[Addr];
    if(pixel)
    {
        g_arr_img[Addr] = Rdata | (0x80 >> (H_cmp % 8));
    }
    else
    {
        g_arr_img[Addr] = Rdata & (~(0x80 >> (H_cmp % 8)));
    }
}

void disp_lib_init (disp_lib_hw_t * hw_init, disp_lib_font_t * font)
{
    memcpy (&disp_lib_txt.font, font, sizeof(disp_lib_font_t));
    g_backg = EP1B_WHITE;
    g_foreg = EP1B_BLACK;
    g_disp_busy = hw_init->busy_pin;
    disp_lib_txt.col_per_lines = (Y_DIM_PIX / (font->char_width));
    disp_lib_txt.total_lines = (X_DIM_PIX / (font->char_height + 4));
    disp_lib_txt.current_line_no = 0;
    disp_lib_txt.current_col_no = 0;

    DEV_Module_Init ((DEV_Config_t * ) hw_init);
    EPD_1IN54_Init (0);
    EPD_1IN54_Clear ();
    EPD_1IN54_Display (g_arr_img);
    
    hal_nop_delay_ms(1000);
    
    EPD_1IN54_Init (1);
    EPD_1IN54_Clear ();
    UG_Init (&g_gui, set_pix_array, X_DIM_PIX, Y_DIM_PIX);
    UG_FontSelect ((UG_FONT *)font);
    UG_SetForecolor (g_foreg);
    UG_SetBackcolor (g_backg);
    
    UG_FillFrame (0, 0, X_DIM_PIX-1, X_DIM_PIX-1, g_backg);
    EPD_1IN54_Display (g_arr_img);
}

void disp_lib_refresh (void)
{
    EPD_1IN54_Init (0);
    EPD_1IN54_Clear ();
    EPD_1IN54_Display (g_arr_img);

    EPD_1IN54_Init (1);
    EPD_1IN54_Display (g_arr_img);
}

void disp_lib_clear (void)
{
    EPD_1IN54_Init (0);
    EPD_1IN54_Clear ();
}

void disp_lib_set_bg_color (UG_COLOR color)
{
    UG_SetBackcolor (color);
    UG_FillFrame (0, 0, (X_DIM_PIX - 1), (Y_DIM_PIX - 1), color);
}

void disp_lib_set_fg_color (UG_COLOR color)
{
    UG_SetForecolor (color);
}

bool disp_lib_is_busy (void)
{
    return hal_gpio_pin_read (g_disp_busy);
}

void disp_lib_set_cursor_loc (uint32_t line_no, uint32_t space)
{
    disp_lib_txt.current_line_no = line_no;
    disp_lib_txt.current_col_no = space;
}

uint32_t disp_lib_get_current_line (void)
{
    return disp_lib_txt.current_line_no;
}


uint32_t r_paper_get_current_col (void)
{
    return disp_lib_txt.current_col_no;
}

disp_lib_status_t disp_lib_putS (char * str, uint32_t len)
{
    uint16_t x_loc, y_loc;
    x_loc = col_to_pix (disp_lib_txt.current_col_no);
    y_loc = line_to_pix (disp_lib_txt.current_line_no);
    UG_PutString (x_loc, y_loc, str);
    EPD_1IN54_Display (g_arr_img);
    
    uint32_t cnt = 0;
    while (cnt < len)
    {
        if(str[cnt] == '\n')
        {
            disp_lib_txt.current_col_no = 0;
            disp_lib_txt.current_line_no++;
        }
        else if ((disp_lib_txt.current_col_no!=0) &&
                 ((disp_lib_txt.current_col_no%disp_lib_txt.col_per_lines) == 0))
        {
            disp_lib_txt.current_col_no = 0;
            disp_lib_txt.current_line_no++;
        }
        else if (str[cnt] != '\n')
        {
            disp_lib_txt.current_col_no++;
        }
        cnt++;
        
    }
        
    return E_PAPER_NO_ERR;
}

disp_lib_status_t disp_lib_putS_at (char * str, uint32_t len,
                                    uint32_t col, uint32_t line)
{
    uint16_t x_loc, y_loc;
    log_printf ("%s, %d\n",__func__, len );
    x_loc = col_to_pix (col);
    y_loc = line_to_pix (line);
    
    log_printf ("params %d %d\n", x_loc, y_loc);
    UG_PutString (x_loc, y_loc, str);
    EPD_1IN54_Display (g_arr_img);
    
    return E_PAPER_NO_ERR;
}

disp_lib_status_t disp_lib_putC (uint8_t c)
{
    uint32_t x_loc, y_loc;
    x_loc = col_to_pix (disp_lib_txt.current_col_no);
    y_loc = line_to_pix (disp_lib_txt.current_line_no);

    UG_PutChar ((char)c, x_loc, y_loc, g_foreg, g_backg);
    EPD_1IN54_Display (g_arr_img);

    if(c == '\n')
    {
        disp_lib_txt.current_col_no = 0;
        disp_lib_txt.current_line_no++;
    }
    else if ((disp_lib_txt.current_col_no!=0) &&
             ((disp_lib_txt.current_col_no%disp_lib_txt.col_per_lines) == 0))
    {
        disp_lib_txt.current_col_no = 0;
        disp_lib_txt.current_line_no++;
    }
    else if (c != '\n')
    {
        disp_lib_txt.current_col_no++;
    }

    return E_PAPER_NO_ERR;
}

disp_lib_status_t disp_lib_putC_at (uint8_t c, uint32_t col, uint32_t line)
{
    uint32_t x_loc, y_loc;
    x_loc = col_to_pix (col);
    y_loc = line_to_pix (line);

    UG_PutChar ((char)c, x_loc, y_loc, g_foreg, g_backg);
    EPD_1IN54_Display (g_arr_img);

    return E_PAPER_NO_ERR;
}