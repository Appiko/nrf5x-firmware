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
#include "hal_uarte.h"
#include "boards.h"
#include "minmea.h"
#include "log.h"
#include "nrf_util.h"
#include "ms_timer.h"
#include "hal_clocks.h"
#include "string.h"

#define MAX_SENTENCE_LENGTH 81

struct gps_data {
    struct minmea_float lat;
    struct minmea_float lng;
    struct minmea_float hdop;
};

void rx_data_parser(uint8_t byte);

void print_gps_data();

// nmea sentence limit to 79 chars + $ + [CR][LF]
char line[MAX_SENTENCE_LENGTH] = {};
uint8_t line_length = 81;
struct gps_data last_recored_gps = {
    .hdop.scale = 0,
    .hdop.value = 0,
    .lat.scale = 0,
    .lat.value = 0,
    .lng.value = 0,
    .lng.scale = 0,
};

int main(void) {
    hal_gpio_cfg_output(22, 0);
    lfclk_init(LFCLK_SRC_Xtal);

    hal_uarte_init(HAL_UARTE_BAUD_9600, APP_IRQ_PRIORITY_LOW);
    ms_timer_init(APP_IRQ_PRIORITY_LOWEST);
    log_printf("Hello world, GPS!\n");

    hal_uarte_start_rx(rx_data_parser);

    ms_timer_start(MS_TIMER0, MS_REPEATED_CALL, MS_TIMER_TICKS_MS(2000), print_gps_data);
    while (true) {
        hal_uarte_process();
    }

}

void print_gps_data() {
    //        hal_uarte_stop_rx();
    log_printf("Lat: %d/%d\nLng: %d/%d\nAcc: %d/%d\n",
            last_recored_gps.lat.value, last_recored_gps.lat.scale,
            last_recored_gps.lng.value, last_recored_gps.lng.scale,
            last_recored_gps.hdop.value, last_recored_gps.hdop.scale
            );
    //        hal_uarte_start_rx(rx_data_parser);
}

void get_coord_with_accuracy(char * line, struct gps_data *data) {
    switch (minmea_sentence_id(line, false)) {
        case MINMEA_SENTENCE_GLL:
        {
            struct minmea_sentence_gll frame;
            if (minmea_parse_gll(&frame, line)) {
                //                    log_printf("GLL Coordinates: (%d/%d,%d/%d)\n\n",
                //                            frame.latitude.value, frame.latitude.scale,
                //                            frame.longitude.value, frame.longitude.scale
                //                            );
                data->lat = frame.latitude;
                data->lng = frame.longitude;

            } else {
//                                    log_printf(" GLL Err. \n");
            }
        }

        case MINMEA_SENTENCE_GGA:
        {
            struct minmea_sentence_gga frame;
            if (minmea_parse_gga(&frame, line)) {
                //                    log_printf("$GGA: LatLng: (%d/%d,%d/%d)\n Accuracy: (%d/%d) \n\n",
                //
                //                            frame.latitude.value, frame.latitude.scale,
                //                            frame.longitude.value, frame.longitude.scale,
                //                            frame.hdop.value, frame.hdop.scale
                //
                //                            );

                data->lat = frame.latitude;
                data->lng = frame.longitude;
                data->hdop = frame.hdop;
            } else {
                //                    log_printf("GGA Err.\n");
            }
        }
            break;
        case MINMEA_SENTENCE_RMC:
        {
            struct minmea_sentence_rmc frame;
            if (minmea_parse_rmc(&frame, line)) {
                //                    log_printf("$RMC: LatLng: (%d/%d, %d/%d)\n\n",
                //                            frame.latitude.value, frame.latitude.scale,
                //                            frame.longitude.value, frame.longitude.scale
                //                            );

                data->lat = frame.latitude;
                data->lng = frame.longitude;
            } else {
//                                    log_printf("RMC Err.\n");
            }

        }
            break;
        default:
            break;

    }


}

void rx_data_parser(uint8_t byte) {
//    log_printf("%c\n", byte);
//    line[line_length++] = byte;
//    if (line_length > MAX_SENTENCE_LENGTH || byte == '\n') {
//        line_length = 0;
//        memset(line, '\0', MAX_SENTENCE_LENGTH);
//    }
        if (byte == '\n' || byte == '\r') {
            get_coord_with_accuracy(line, &last_recored_gps);
            line_length = 0;
            //        line = {};
        } else if (line_length == 0 && byte == '$') {
            // push G to line at index 0
            memset(line,'\0',MAX_SENTENCE_LENGTH);
            line[line_length++] = '$';
        } else if (line_length != 0) {
            //push to line
            line[line_length++] = byte;
        }
}
