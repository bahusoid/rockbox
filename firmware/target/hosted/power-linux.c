/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
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
#include <sys/types.h>
#include <fcntl.h>
#include <string.h>
#include <unistd.h>
#include <stdio.h>

#include "system.h"
#include "power.h"
#include "panic.h"
#include "sysfs.h"

#include "tick.h"
#ifdef FIIO_M3K_LINUX
#include "usb.h"
#endif

#ifdef BATTERY_DEV_NAME
# define BATTERY_SYSFS_PATH     "/sys/class/power_supply/" BATTERY_DEV_NAME
# define BATTERY_STATUS_PATH    BATTERY_SYSFS_PATH "/status"
# define BATTERY_VOLTAGE_PATH   BATTERY_SYSFS_PATH "/voltage_now"
# define BATTERY_CURRENT_PATH   BATTERY_SYSFS_PATH "/current_now"
# define BATTERY_LEVEL_PATH     BATTERY_SYSFS_PATH "/capacity"
# define BATTERY_TTE_PATH       BATTERY_SYSFS_PATH "/time_to_empty_now"
#endif

/* Voltage is normally in microvolts */
#ifndef BATTERY_VOLTAGE_SCALE_MUL
# define BATTERY_VOLTAGE_SCALE_MUL 1
#endif

#ifndef BATTERY_VOLTAGE_SCALE_DIV
# define BATTERY_VOLTAGE_SCALE_DIV 1000
#endif

/* Current is normally in microamps */
#ifndef BATTERY_CURRENT_SCALE_MUL
# define BATTERY_CURRENT_SCALE_MUL 1
#endif

#ifndef BATTERY_CURRENT_SCALE_DIV
# define BATTERY_CURRENT_SCALE_DIV 1000
#endif

/* Level is normally in whole percentage points (0-100) */
#ifndef BATTERY_LEVEL_SCALE_MUL
# define BATTERY_LEVEL_SCALE_MUL 1
#endif

#ifndef BATTERY_LEVEL_SCALE_DIV
# define BATTERY_LEVEL_SCALE_DIV 1
#endif

#define POWER_STATUS_PATH "/sys/class/power_supply/" POWER_DEV_NAME "/online"

//#undef HAVE_HOSTED_NETLINK_MONITOR

#ifdef HAVE_HOSTED_NETLINK_MONITOR
long get_last_battery_event(void);

#define GET_CACHED(result_var) \
static int result_var = -1; \
static long last_event_tick = -1; \
long last_battery_event = get_last_battery_event(); \
if (last_event_tick == last_battery_event) \
    return result_var; \
last_event_tick = last_battery_event
#else
#define GET_CACHED(result_var) \
static int result_var = 0; \
static long last_tick = 0; \
if (last_tick && (current_tick - last_tick) < HZ/2) \
    return result_var; \
last_tick = current_tick
#endif

#ifdef BATTERY_DEV_NAME

bool charging_state(void)
{
    GET_CACHED(last_power);

    char buf[12] = {0};
    sysfs_get_string(BATTERY_STATUS_PATH, buf, sizeof(buf));
    last_power = (strncmp(buf, "Charging", 8) == 0);
    return last_power;
}

#if (CONFIG_BATTERY_MEASURE & VOLTAGE_MEASURE)
int _battery_voltage(void)
{
    GET_CACHED(result);
    int voltage = 0;
    sysfs_get_int(BATTERY_VOLTAGE_PATH, &voltage);

    result = (voltage * BATTERY_VOLTAGE_SCALE_MUL) / BATTERY_VOLTAGE_SCALE_DIV;
    return result;
}
#endif

#if (CONFIG_BATTERY_MEASURE & CURRENT_MEASURE)
int _battery_current(void)
{
    GET_CACHED(result);
    int current = 0;
    sysfs_get_int(BATTERY_CURRENT_PATH, &current);

    result = (current * BATTERY_CURRENT_SCALE_MUL) / BATTERY_CURRENT_SCALE_DIV;
    return result;
}
#endif

#if (CONFIG_BATTERY_MEASURE & PERCENTAGE_MEASURE)
int _battery_level(void)
{
    GET_CACHED(result);
    int level = 0;
    sysfs_get_int(BATTERY_LEVEL_PATH, &level);

    result = (level * BATTERY_LEVEL_SCALE_MUL) / BATTERY_LEVEL_SCALE_DIV;
    return result;
}
#endif

#if (CONFIG_BATTERY_MEASURE & TIME_MEASURE)
int _battery_time(void)
{
    GET_CACHED(battery_tte);
    sysfs_get_int(BATTERY_TTE_PATH, &battery_tte);

    return battery_tte;
}
#endif
#endif

unsigned int power_input_status(void)
{
    GET_CACHED(result);
    int present = 0;
    sysfs_get_int(POWER_STATUS_PATH, &present);

#ifdef FIIO_M3K_LINUX
    usb_enable(present ? true : false);
#endif

    result = present ? POWER_INPUT_USB_CHARGER : POWER_INPUT_NONE;
    return result;
}
