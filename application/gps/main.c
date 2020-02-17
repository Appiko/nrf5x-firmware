/*
 *  main.c : Application to blink LED
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


#include <stdbool.h>
#include <stdint.h>
#include "hal_nop_delay.h"
#include "hal_gpio.h"
#include "hal_uart.h"
#include "boards.h"
#include "minmea.h"
#include "log.h"

int main(void) {
    log_init();
    log_printf("Hello world, GPS!\n");


    hal_gpio_cfg_output(22, 1);

    char lines[3][100] = {"$GPGLL,4916.450,N,12311.120,W,225444,A*31",
        "$GPRMC,123519,A,4807.038,N,01131.000,E,022.4,084.4,230394,003.1,W*6A",
        "$GPGGA,123519,4807.038,N,01131.000,E,1,08,0.9,545.4,M,46.9,M,,*47"};


    for (int i = 0; i < 3; i++) {
        switch (minmea_sentence_id(lines[i], false)) {
            case MINMEA_SENTENCE_GLL:
            {
                struct minmea_sentence_gll frame;
                if (minmea_parse_gll(&frame, lines[i])) {
                    log_printf("GLL Coordinates: (%d/%d,%d/%d)\n\n",
                            frame.latitude.value, frame.latitude.scale,
                            frame.longitude.value, frame.longitude.scale
                            );
                } else {
                    log_printf(" GLL Err. \n");
                }
            }

            case MINMEA_SENTENCE_GGA:
            {
                struct minmea_sentence_gga frame;
                if (minmea_parse_gga(&frame, lines[i])) {
                    log_printf("$GGA: LatLng: (%d/%d,%d/%d)\n Accuracy: (%d/%d) \n\n",
                            
                            frame.latitude.value, frame.latitude.scale,
                            frame.longitude.value,frame.longitude.scale,
                            frame.hdop.value, frame.hdop.scale
                            
                            );
                } else {
                    log_printf("GGA Err.\n");
                }
            }
                break;
            case MINMEA_SENTENCE_RMC:
            {
                struct minmea_sentence_rmc frame;
                if (minmea_parse_rmc(&frame, lines[i])) {
                    log_printf("$RMC: LatLng: (%d/%d, %d/%d)\n\n",
                            frame.latitude.value, frame.latitude.scale,
                            frame.longitude.value, frame.longitude.scale
                            );
                }
                else{
                    log_printf("RMC Err.\n");
                }
                
            }
                break;
            default:
                break;

        }
    }


    //    while (1){
    //        log_printf(".\n");
    //        hal_gpio_pin_set(22);
    //        hal_nop_delay_ms(10000);
    //        hal_gpio_pin_clear(22);
    //        hal_nop_delay_ms(500);
    //    }



}

/** @} */
/** @} */
