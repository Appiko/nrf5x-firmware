/*
 *  button_ui.h
 *
 *  Created on: 25-Apr-2018
 *
 *  Copyright (c) 2018, Appiko
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without modification,
 *  are permitted provided that the following conditions are met:
 *
 *  1. Redistributions of source code must retain the above copyright notice,
 *  this list of conditions and the following disclaimer.
 *
 *  2. Redistributions in binary form must reproduce the above copyright notice,
 *  this list of conditions and the following disclaimer in the documentation
 *  and/or other materials provided with the distribution.
 *
 *  3. Neither the name of the copyright holder nor the names of its contributors
 *  may be used to endorse or promote products derived from this software without
 *  specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 *  ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 *  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 *  IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 *  INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 *  OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY,
 *  WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 *  ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 */

#ifndef CODEBASE_PERIPHERAL_MODULES_BUTTON_UI_H_
#define CODEBASE_PERIPHERAL_MODULES_BUTTON_UI_H_

#include "stdint.h"
#include "stdbool.h"

typedef enum
{
    BUTTON_UI_CROSS,
    BUTTON_UI_RELEASE
}button_ui_action;

/**
 *
 */
typedef enum
{
    BUTTON_UI_PRESS,//!< BUTTON_UI_PRESS
    BUTTON_UI_LONG,  //!< BUTTON_UI_LONG
    BUTTON_UI_WAKE, //!< BUTTON_UI_WAKE
}button_ui_steps;

/**
 *
 */
const static uint32_t press_duration[] = {
    100,
    5000,
    0xFFFFFF
};


void button_ui_init(uint32_t button_pin, uint32_t irq_priority,
        void (*button_ui_handler)(button_ui_steps step, button_ui_action act));

bool button_ui_add_tick(uint32_t ui_ticks);

void button_ui_config_wake(bool set_wake_on);

#endif /* CODEBASE_PERIPHERAL_MODULES_BUTTON_UI_H_ */
