/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 *
 * Copyright (c) 2018 Marcin Bukat
 * Copyright (c) 2025 Solomon Peachy
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

//#define LOGF_ENABLE

#include "config.h"
#include "audio.h"
#include "audiohw.h"
#include "button.h"
#include "system.h"
#include "kernel.h"
#include "panic.h"
#include "sysfs.h"
#include "alsa-controls.h"
#include "pcm-alsa-hiby.h"
#include "sound.h"
#include "settings.h"

#include "logf.h"

#include <stdio.h>
#include <string.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <unistd.h>

int hiby_has_valid_output(void);

#define HIBY_SYS_SERVER_SOCKET "/var/run/sys_server"
#define HIBY_ABSVOL_MAX 127

static int hw_init = 0;

static long int vol_l_hw = 255;
static long int vol_r_hw = 255;
static long int last_ps = -1;

static int muted = -1;
static int bt_absvol_last_step = -1;
static char bt_absvol_last_mac[18];

static int hwvolume_to_percent(int volume_cb)
{
    int min_vol = sound_min(SOUND_VOLUME);
    int max_vol = sound_max(SOUND_VOLUME);

    int span = max_vol - min_vol;
    if (span <= 0)
        return 0;

    int i_from_span = volume_cb - min_vol;
    int pct = (i_from_span * 100 + span / 2) / span;

    return pct;
}

static int hiby_volume_to_absvol_step(int volume_cb)
{
    int pct = hwvolume_to_percent(volume_cb);

    int step;
    
    step = (pct * HIBY_ABSVOL_MAX + 50) / 100;
    if (step < 0)
        step = 0;
    if (step > HIBY_ABSVOL_MAX)
        step = HIBY_ABSVOL_MAX;

    return step;
}

static int hiby_sys_server_command(const char *command, char *reply, size_t reply_size)
{
    struct sockaddr_un addr;
    struct timeval tv = { .tv_sec = 0, .tv_usec = 300000 };
    int fd = -1;
    int rc = -1;
    ssize_t n;

    if (reply && reply_size > 0)
        reply[0] = '\0';

    if (!command || !*command)
        return -1;

    fd = socket(AF_UNIX, SOCK_STREAM, 0);
    if (fd < 0)
        return -1;

    setsockopt(fd, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    setsockopt(fd, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));

    memset(&addr, 0, sizeof(addr));
    addr.sun_family = AF_UNIX;
    snprintf(addr.sun_path, sizeof(addr.sun_path), "%s", HIBY_SYS_SERVER_SOCKET);

    if (connect(fd, (struct sockaddr *)&addr, sizeof(addr)) < 0)
        goto out;
    if (send(fd, command, strlen(command), 0) < 0)
        goto out;

    if (reply && reply_size > 1)
    {
        n = recv(fd, reply, reply_size - 1, 0);
        if (n < 0)
            goto out;
        reply[n] = '\0';
    }

    rc = 0;

out:
    if (fd >= 0)
        close(fd);
    return rc;
}

static void hiby_notify_bt_absvol(int volume_cb)
{
    const char *mac_u = hiby_pcm_get_bt_mac();
    char cmd[96];
    char reply[128];
    int step;

    if (!mac_u || !mac_u[0])
    {
        bt_absvol_last_step = -1;
        bt_absvol_last_mac[0] = '\0';
        return;
    }

    step = hiby_volume_to_absvol_step(volume_cb);
    if (step == bt_absvol_last_step && !strcmp(mac_u, bt_absvol_last_mac))
        return;

    snprintf(cmd, sizeof(cmd), "BT:ABSVOL:%s %d", mac_u, step);
    if (hiby_sys_server_command(cmd, reply, sizeof(reply)) < 0)
    {
        logf("bt absvol send fail: %s", cmd);
        return;
    }

    if (!strstr(reply, "OK"))
    {
        logf("bt absvol not-ok: %s", reply);
        return;
    }

    bt_absvol_last_step = step;
    snprintf(bt_absvol_last_mac, sizeof(bt_absvol_last_mac), "%s", mac_u);
}

void audiohw_mute(int mute)
{
    if (hw_init < 0 || muted == mute)
        return;

    muted = mute;

    alsa_controls_set_bool("Mute Output", !!mute);
}

int hiby_has_valid_output(void) {
    long int ps = 0; // Muted, if nothing is plugged in!

    int status = 0;

    if (!hw_init) return ps;

    const char * const sysfs_hs_switch = "/sys/class/switch/headset/state";
    const char * const sysfs_bal_switch = "/sys/class/switch/balance/state";

    if (sysfs_get_int(sysfs_hs_switch, &status) && status)
        ps = 2; // headset

    if (sysfs_get_int(sysfs_bal_switch, &status) && status)
        ps = 3; // balanced output

    return ps;
}

int hiby_get_outputs(void){
    long int ps = hiby_has_valid_output();

    hiby_set_output(ps);

    return ps;
}

void hiby_set_output(int ps)
{
    if (!hw_init || muted) return;

    // Default to headset if nothing was ever inserted; otherwise, R3 Pro II crashes on playback
    if (ps == 0)
    {
        ps = last_ps > 0 ? last_ps : 2;
    }

    if (last_ps != ps)
    {
        logf("set out %d/%d", ps, last_ps);
        /* Output port switch */
        last_ps = ps;
        alsa_controls_set_ints("Output Port Switch", 1, &last_ps);
        audiohw_set_volume(vol_l_hw, vol_r_hw);
    }
}

void audiohw_preinit(void)
{
    logf("hw preinit");
    alsa_controls_init("default");
    hw_init = 1;

    audiohw_mute(false);  /* No need ? */
    alsa_controls_set_bool("DOP_EN", 0); //isDSD
}

void audiohw_postinit(void)
{
    logf("hw postinit");
}

void audiohw_close(void)
{
    logf("hw close");
    hw_init = 0;
    alsa_controls_close();
}

void audiohw_set_frequency(int fsel)
{
    (void)fsel;
}
void bt_bluealsa_change_volume(int l, int r, char* mac);

void audiohw_set_volume(int vol_l, int vol_r)
{
    logf("hw vol %d %d", vol_l, vol_r);

    long l,r;

    vol_l_hw = vol_l;
    vol_r_hw = vol_r;

    l = -vol_l/5;
    r = -vol_r/5;

    if (!hw_init)
        return;

    alsa_controls_set_ints("Left Playback Volume", 1, &l);
    alsa_controls_set_ints("Right Playback Volume", 1, &r);

    int vol_avg = (vol_l + vol_r) / 2;
    hiby_notify_bt_absvol(vol_avg);

    // It won't work as is, too many system calls will cause lag and audio dropouts
    //int vol_percents = hwvolume_to_percent(vol_avg);
    //bt_bluealsa_change_volume(vol_percents, vol_percents, hiby_pcm_get_bt_mac());
}

void audiohw_set_filter_roll_off(int value)
{
    logf("rolloff %d", value);
    /* 0 = Sharp;
     *       1 = Slow;
     *       2 = Short Sharp
     *       3 = Short Slow
     *       4 = Super Slow */
    long int value_hw = value;
    alsa_controls_set_ints("Digital Filter", 1, &value_hw);

}
