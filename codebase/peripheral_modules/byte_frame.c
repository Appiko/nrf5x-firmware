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
#include "byte_frame.h"

#define START_FLAG 0x12
#define END_FLAG 0x13
#define ESCAPE_FLAG 0x7D

#define MAX_FRAME_SIZE 32
#define MAX_ENCODED_FRAME_SIZE (MAX_FRAME_SIZE*2+2)

typedef enum {WAIT_HEADER, IN_MSG, AFTER_ESCAPE} DecodeState;

typedef struct{
    DecodeState state;
    uint8_t decodedOutput[MAX_FRAME_SIZE];
    uint8_t *currentOutput;
}DecodeData;

DecodeData data = 
{
    .state = WAIT_HEADER,

};


bool encodeFrame(const uint8_t *bytesToEncode, uint16_t len,
        void (*encode_done)(const uint8_t * encoded_data,uint16_t len))
{
    uint8_t encodedOutput[MAX_ENCODED_FRAME_SIZE];
    uint8_t *currentOutput = encodedOutput;
    const uint8_t *currentInput = bytesToEncode;

    if (len>MAX_FRAME_SIZE)
    {
        return false;
    }

    *currentOutput = START_FLAG;
    currentOutput++;

    while(currentInput < bytesToEncode+len)
    {
        if (*currentInput==START_FLAG || *currentInput==END_FLAG || *currentInput==ESCAPE_FLAG)
        {
            *currentOutput = ESCAPE_FLAG;
            currentOutput++;
        }

        *currentOutput = *currentInput;
        currentOutput++;
        currentInput++;
    }

    *currentOutput = END_FLAG;
    currentOutput++;

    (*encode_done)(encodedOutput, currentOutput-encodedOutput);
    return true;
}


void decodeFrame(const uint8_t *bytesToDecode, uint16_t len,
        void (*decode_done)(const uint8_t * decoded_data,uint16_t len))
{
    /* Decode data start values */
    
    const uint8_t *currentInput = bytesToDecode;

    while(currentInput < bytesToDecode+len){

        if (data.state==WAIT_HEADER)
        {
            if (*currentInput==START_FLAG)
            {
                data.currentOutput = data.decodedOutput;
                data.state=IN_MSG;
            }
            else
            {
                data.state=WAIT_HEADER;
            }
        }
        else if (data.state==IN_MSG)
        {
            if (*currentInput==ESCAPE_FLAG)
            {
                data.state=AFTER_ESCAPE;
            }
            else if (*currentInput==END_FLAG)
            {
                (*decode_done) (data.decodedOutput, 
                    (data.currentOutput - data.decodedOutput));
                data.state=WAIT_HEADER;
                data.currentOutput = data.decodedOutput;
            }
            else if (*currentInput==START_FLAG)
            {
                /* Something wrong happened!, restarting.. */
                data.state=WAIT_HEADER;
                data.currentOutput = data.decodedOutput;
                /* Skip increment currentInput. Maybe currentInput is START_FLAG from next packet */
                continue;
            }
            else
            {
                if (data.currentOutput-data.decodedOutput<MAX_FRAME_SIZE)
                {
                    data.state=IN_MSG;
                    *data.currentOutput = *currentInput;
                    data.currentOutput++;
                }
                else
                {
                    /* Something wrong happened!, restarting.. */
                    data.state=WAIT_HEADER;
                    data.currentOutput = data.decodedOutput;
                    /* Skip increment currentInput. Maybe currentInput is START_FLAG from next packet */
                    continue;
                }
            }
        }
        else if (data.state==AFTER_ESCAPE)
        {
            if (*currentInput==START_FLAG || *currentInput==END_FLAG || *currentInput==ESCAPE_FLAG)
            {
                if ((data.currentOutput-data.decodedOutput)<MAX_FRAME_SIZE)
                {
//                    printf("State:AFTER_ESCAPE, appending byte 0x%X to decoded frame\n",*currentInput);
                    data.state=IN_MSG;
                    *data.currentOutput = *currentInput;
                    data.currentOutput++;
                }
                else
                {
                    /* Something wrong happened!, restarting.. */
                    data.state=WAIT_HEADER;
                    data.currentOutput = data.decodedOutput;
                    /* Skip increment currentInput. Maybe currentInput is START_FLAG from next packet */
                    continue;
                }
            }
            else
            {
                /* Something wrong happened!, restarting.. */
                data.state=WAIT_HEADER;
                data.currentOutput = data.decodedOutput;
                /* Skip increment currentInput. Maybe currentInput is START_FLAG from next packet */
                continue;
            }
        }
        else
        {
            data.state=WAIT_HEADER;
            data.currentOutput = data.decodedOutput;
        }

        /* Next input uint8_t */
        currentInput++;
    }
}

