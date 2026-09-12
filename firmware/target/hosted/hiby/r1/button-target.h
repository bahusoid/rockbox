/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2017 by Marcin Bukat
 * Copyright (C) 2025 by Melissa Autumn
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
#ifndef _BUTTON_TARGET_H_
#define _BUTTON_TARGET_H_

#include <stdbool.h>
#include "config.h"

/* Main unit's buttons */
#define BUTTON_POWER                1 << 0
#define BUTTON_RIGHT                 1 << 1 //Next
#define BUTTON_LEFT                 1 << 2 //Play/Pause
#define BUTTON_UP               1 << 3 //Volume Up
#define BUTTON_DOWN             1 << 4 //Volume Down

//TODO: It's from bluetooth, should we define it as BUTTON_REMOTE?
#define BUTTON_PREV                 1 << 5
#define BUTTON_NEXT                 1 << 6
#define BUTTON_PLAY                 1 << 7
#define BUTTON_VOL_UP               1 << 8
#define BUTTON_VOL_DOWN             1 << 9

//Max up to 22 buttons (10 are reserved for touchscreen virtual buttons)
#define  BUTTON_LAST_MAIN    BUTTON_VOL_DOWN

#define BUTTON_MAIN                ((BUTTON_POWER|BUTTON_RIGHT|BUTTON_LEFT|BUTTON_UP|BUTTON_DOWN)|(BUTTON_PREV|BUTTON_NEXT|BUTTON_PLAY|BUTTON_VOL_UP|BUTTON_VOL_DOWN))


/* Touchscreen virtual buttons */
#define BUTTON_TOPLEFT      BUTTON_LAST_MAIN << 1
#define BUTTON_TOPMIDDLE    BUTTON_LAST_MAIN << 2
#define BUTTON_TOPRIGHT     BUTTON_LAST_MAIN << 3
#define BUTTON_MIDLEFT      BUTTON_LAST_MAIN << 4
#define BUTTON_CENTER       BUTTON_LAST_MAIN << 5
#define BUTTON_MIDRIGHT     BUTTON_LAST_MAIN << 6
#define BUTTON_BOTTOMLEFT   BUTTON_LAST_MAIN << 7
#define BUTTON_BOTTOMMIDDLE BUTTON_LAST_MAIN << 8
#define BUTTON_BOTTOMRIGHT  BUTTON_LAST_MAIN << 9

#define BUTTON_TOUCH        BUTTON_LAST_MAIN << 10


/* Software power-off */
#define POWEROFF_BUTTON BUTTON_POWER
#define POWEROFF_COUNT 25

int button_map_with_id(int keycode, int id);

#endif /* _BUTTON_TARGET_H_ */
