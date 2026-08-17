/***************************************************************************
*             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2026 by Roman Artiukhin
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
#include <stdbool.h>
#include <fcntl.h>
#include <unistd.h>
#include <string.h>
#include <sys/socket.h>
#include <linux/netlink.h>
#include <poll.h>

#include "tick.h"

static int netlink_fd = -1;
static bool state_hs = false;
static bool state_bal = false;
static bool state_lin = false;
static long last_device_event = 0;
static long last_battery_event = 0;
static bool initialized = false;

static int read_sysfs_state(const char *path)
{
    int fd = open(path, O_RDONLY | O_CLOEXEC );
    if (fd < 0) return false;
    
    char buf[16] = {0};
    ssize_t n = read(fd, buf, sizeof(buf) - 1);
    close(fd);
    
    if (n > 0) {
        return (buf[0] != '0');
    }
    return false;
}

static void update_switch_states(void)
{
    state_hs = read_sysfs_state("/sys/class/switch/headset/state");
    state_bal = read_sysfs_state("/sys/class/switch/balance/state");
    state_lin = read_sysfs_state("/sys/class/switch/lineout/state");
}

void init_netlink_monitor(void)
{
    if (initialized)
        return;

    struct sockaddr_nl nls;
    memset(&nls, 0, sizeof(struct sockaddr_nl));
    nls.nl_family = AF_NETLINK;
    nls.nl_pid = getpid();
    nls.nl_groups = -1; 

    netlink_fd = socket(PF_NETLINK, SOCK_DGRAM | SOCK_NONBLOCK, NETLINK_KOBJECT_UEVENT);
    if (netlink_fd >= 0) {
        if (bind(netlink_fd, (void *)&nls, sizeof(struct sockaddr_nl)) < 0) {
            close(netlink_fd);
            netlink_fd = -1;
        }
    }

    update_switch_states();
    initialized = true;
}

void drain_netlink_events(void)
{
    if (netlink_fd < 0)
        return;

    struct pollfd pfd;
    pfd.fd = netlink_fd;
    pfd.events = POLLIN;

    bool refresh_switch = false;
    bool battery = false;
    
    /* Poll with timeout 0 makes this non-blocking */
    while (poll(&pfd, 1, 0) > 0) {
        if (pfd.revents & POLLIN) {
            char buf[128];
            ssize_t len = recv(netlink_fd, buf, sizeof(buf) - 1, 0);
            if (len > 0) {
                buf[len] = '\0';

                /* If any switch uevent comes through, refresh our state cache */
                if (buf[0] == 'c')
                {
                    //change@/devices/virtual/switch/headset
                    if (!refresh_switch && strstr(buf, "switch")) {
                        refresh_switch = true;
                    }
                    //change@/devices/i2c-0/0-0062/power_supply/battery
                    if (!battery && strstr(buf, "battery")) {
                        last_battery_event = current_tick;
                        battery = true;
                    }
                }
                //remove@/devices/virtual/input/input6
                //add@/devices/virtual/input/input7
                else if (buf[0] == 'r' || buf[0] == 'a')
                {
                    last_device_event = current_tick;
                }
            } else {
                break;
            }
        } else {
            break;
        }
    }
    if (refresh_switch)
        update_switch_states();
}

int hiby_has_valid_output(void) {
    int ps = 0;
    if (state_hs > 0)
        ps = 2; // headset

    if (state_bal > 0)
        ps = 3; // balanced output

    return ps;
}

bool headphones_inserted(void)
{
    return hiby_has_valid_output();
}

bool lineout_inserted(void)
{
    return state_lin;
}

long get_last_devices_event(void)
{
    return last_device_event;
}

long get_last_battery_event(void)
{
    return last_battery_event;
}