/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2021 by Solomon Peachy
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License
 * as published by the Free Software Foundation; either version 2
 * of the License, or (at your option) any later version.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/


#include <SDL.h>
#include "button.h"
#include "buttonmap.h"

int key_to_button(int keyboard_button)
{
    // touchscreen mapping for sim overrides some standard buttons to touch events
    // so we need to remap available physical buttons 
    int new_btn = BUTTON_NONE;
    switch (keyboard_button)
    {
        case SDLK_KP_ENTER:
        case SDLK_RETURN:
        case SDLK_SPACE:
        case SDLK_KP_5:
            new_btn = BUTTON_POWER;
            break;
        case SDLK_UP:
        case SDLK_KP_8:
            new_btn = BUTTON_UP;
            break;
        case SDLK_DOWN:
        case SDLK_KP_2:
            new_btn = BUTTON_DOWN;
            break;
        case SDLK_LEFT:
        case SDLK_KP_4:
            new_btn = BUTTON_LEFT;
            break;
        case SDLK_RIGHT:
        case SDLK_KP_6:
            new_btn = BUTTON_RIGHT;
            break;
    }
    return new_btn;
}

struct button_map bm[] = {
    { 0, 0, 0, 0, "None" }
};
