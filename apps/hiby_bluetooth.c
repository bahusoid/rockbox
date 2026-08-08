/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/_ \ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2026
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

#include "config.h"
#include "hiby_bluetooth.h"

#if defined(HIBY_LINUX) && !defined(SIMULATOR)


#include <ctype.h>
#include <dirent.h>
#include <fcntl.h>
#include <stdio.h>
#include <stdarg.h>
#include <unistd.h>

#include "kernel.h"
#include "audio.h"
#include "action.h"
#include "button-devinput.h"
#include "menu.h"
#include "misc.h"
#include "splash.h"
#include "gui/list.h"
#include "pcm-alsa.h"
#include "yesno.h"

/* HiBy hosted build provides dynamic output routing helper in its
 * target-specific PCM implementation. */
int pcm_alsa_switch_playback_device(const char *device);
void pcm_alsa_close_device(const char *device);
void hiby_pcm_set_bt_mac(const char *mac);
static bool bt_ctl_run(const char *arg1, const char *arg2, const char *success_str);
static bool bt_get_active_mac(char *mac_out, size_t mac_out_len);

#define BT_MAX_DEVICES 32
#define BT_NAME_LEN 80
#define BT_LOCAL_PLAYBACK_DEVICE "plughw:0,0"
#define BT_DEVICE_PICK_CANCEL (-1)
#define BT_DEVICE_PICK_SCAN (-2)
#define BT_SCAN_MENU_LABEL "Scan for new devices"
#define BT_MAX_CODECS 8
#define BT_CODEC_NAME_LEN 16
#define BOOT_SETTING_FILE ROCKBOX_DIR"/rb_bt_on.txt"
#define BT_SYS_PATH "/sys/class/bluetooth"
#define BT_DEBUG_LOG_FILE "/data/mnt/sd_0/rockbox-bt-debug.log"

const int BT_REMOTE_INPUT_IDX = 4;


struct bt_device
{
    char mac[18];
    char name[BT_NAME_LEN];
    bool paired;
};

struct bt_device_menu_data
{
    struct bt_device *devices;
    int count;
};

struct bt_strlist_data
{
    char (*items)[BT_CODEC_NAME_LEN];
    int count;
};

static char bt_selected_mac[18];
static char bt_selected_name[BT_NAME_LEN];
static const char *bt_playback_dev = BT_LOCAL_PLAYBACK_DEVICE;
static char bt_bt_playback_dev[2][96];
static unsigned int bt_bt_playback_dev_next = 0;
static char bt_active_codec[16];

static bool bt_wait_for_bluealsa_pcm(const char *mac, int timeout_ticks);
static void bt_set_active_codec(const char *mac);
static bool bt_bluealsa_pcm_ready(const char *mac);
static bool bt_is_connected(const char *mac);
static bool bt_prepare_stack(void);
static void bt_connect_device(const struct bt_device *device);
static void bt_disconnect(void);
static bool is_busy = false;

void hiby_debug_log(const char *format, ...)
{
    return;
    char line[512];
    va_list ap;
    int fd;
    int len;

    va_start(ap, format);
    len = vsnprintf(line, sizeof(line) - 1, format, ap);
    va_end(ap);

    if (len < 0)
        return;
    if (len >= (int)sizeof(line) - 1)
        len = sizeof(line) - 2;
    line[len++] = '\n';

    fd = open(BT_DEBUG_LOG_FILE, O_WRONLY | O_CREAT | O_APPEND, 0666);
    if (fd < 0)
        return;

    write(fd, line, len);
    fsync(fd);
    close(fd);
}

int count_items(const char *path, int max_count){
    
    DIR *dir = opendir(path);
    if (!dir) return -1;

    int count = 0;
    struct dirent *entry;

    while ((entry = readdir(dir)) != NULL) {
        if (entry->d_name[0] == '.') {
            if (entry->d_name[1] == '\0' || 
               (entry->d_name[1] == '.' && entry->d_name[2] == '\0')) {
                continue;
               }
        }
        if (++count >= max_count) {
            break;
        }
    }

    closedir(dir);
    return count;
}

bool bt_is_suspended_fast(void)
{
    return access(BT_SYS_PATH"/hci0", F_OK) != 0 
    && access(PIVOT_ROOT BOOT_SETTING_FILE, F_OK) != 0;
}

bool bt_is_connected_fast(void)
{
    return count_items(BT_SYS_PATH, 2) > 1;
}

static void bt_suspend(void)
{
    system("/usr/bin/bt_suspend");
}

static const char *bt_make_bt_playback_dev(const char *mac, const char *codec)
{
    char *route = bt_bt_playback_dev[bt_bt_playback_dev_next];

    bt_bt_playback_dev_next ^= 1;

    char* codec_str = codec ? ",CODEC=": "";
    codec = codec ? codec : "";
    snprintf(route, sizeof(bt_bt_playback_dev[0]),
             "bluealsa:DEV=%s,PROFILE=a2dp%s%s", mac, codec_str, codec);
    return route;
}

static int bt_simplelist_ok_cancel(int action, struct gui_synclist *lists)
{
    (void)lists;
    if (action == ACTION_STD_OK)
        return ACTION_STD_CANCEL;
    return action;
}

static int bt_simplelist_ok_cancel_return_action(int action, struct gui_synclist *lists)
{
    if (lists->data)
        *((int*)lists->data) = action;

    if (action == ACTION_STD_CONTEXT)
        return ACTION_STD_OK;
    if (action == ACTION_STD_OK)
        return ACTION_STD_CANCEL;

    return action;
}

static int bt_devicelist_callback(int action, struct gui_synclist *lists)
{
    (void)lists;
    if (action == ACTION_STD_OK)
        return ACTION_STD_CANCEL;
    if (action == ACTION_STD_CONTEXT)
    {
        struct bt_device_menu_data* ctx = lists->data;
        struct bt_device* bt_device = &ctx->devices[lists->selected_item - 1];
        if (bt_device->paired && confirm_delete_yesno(bt_device->name) == YESNO_YES)
        {
            bt_enable();
            if (*bt_active_codec && strcmp(bt_selected_mac, bt_device->mac) == 0)
                bt_disconnect();

            if (bt_ctl_run("remove", bt_device->mac, "has been removed"))
                bt_device->paired = false;
            return ACTION_REDRAW;
        }
    }

    return action;
}

static const char *bt_action_name_cb(int selected_item, void *data,
    char *buffer, size_t buffer_len)
{
    const char **items = data;
    if (selected_item < 0 || selected_item >= 3)
    {
        buffer[0] = '\0';
        return buffer;
    }
    snprintf(buffer, buffer_len, "%s", items[selected_item]);
    return buffer;
}

static const char *bt_device_name_cb(int selected_item, void *data,
    char *buffer, size_t buffer_len)
{
    struct bt_device_menu_data *ctx = data;
    if (selected_item == 0)
    {
        snprintf(buffer, buffer_len, "%s", BT_SCAN_MENU_LABEL);
        return buffer;
    }
    selected_item--;
    if (selected_item < 0 || selected_item >= ctx->count)
    {
        buffer[0] = '\0';
        return buffer;
    }

    snprintf(buffer, buffer_len, "%s%s (%s)",
    ctx->devices[selected_item].paired ? " [P]" : "",
        ctx->devices[selected_item].name,
        ctx->devices[selected_item].mac
        );
    return buffer;
}

static const char *bt_strlist_name_cb(int selected_item, void *data,
    char *buffer, size_t buffer_len)
{
    struct bt_strlist_data *ctx = data;
    if (selected_item < 0 || selected_item >= ctx->count)
    {
        buffer[0] = '\0';
        return buffer;
    }
    snprintf(buffer, buffer_len, "%s", ctx->items[selected_item]);
    return buffer;
}

static void bt_trim(char *s)
{
    size_t len;

    if (!s)
        return;

    len = strlen(s);
    while (len > 0 && (s[len - 1] == '\n' || s[len - 1] == '\r' || isspace((unsigned char)s[len - 1])))
        s[--len] = '\0';
}

static bool bt_has_mac_pattern(const char *p, char sep)
{
    int i;

    for (i = 0; i < 17; i++)
    {
        if ((i % 3) == 2)
        {
            if (p[i] != sep)
                return false;
        }
        else if (!isxdigit((unsigned char)p[i]))
        {
            return false;
        }
    }

    return true;
}

static bool bt_extract_mac_from_line(const char *line, char *mac_out, size_t mac_out_len)
{
    size_t i, len;

    if (!line || mac_out_len < 18)
        return false;

    len = strlen(line);
    if (len < 17)
        return false;

    for (i = 0; i + 17 <= len; i++)
    {
        char sep = line[i + 2];
        if (sep != ':' && sep != '_')
            continue;
        if (!bt_has_mac_pattern(&line[i], sep))
            continue;

        snprintf(mac_out, mac_out_len, "%c%c:%c%c:%c%c:%c%c:%c%c:%c%c",
            toupper((unsigned char)line[i + 0]), toupper((unsigned char)line[i + 1]),
            toupper((unsigned char)line[i + 3]), toupper((unsigned char)line[i + 4]),
            toupper((unsigned char)line[i + 6]), toupper((unsigned char)line[i + 7]),
            toupper((unsigned char)line[i + 9]), toupper((unsigned char)line[i + 10]),
            toupper((unsigned char)line[i + 12]), toupper((unsigned char)line[i + 13]),
            toupper((unsigned char)line[i + 15]), toupper((unsigned char)line[i + 16]));
        return true;
    }

    return false;
}

static void bt_mac_to_underscore(const char *mac, char *out, size_t out_len)
{
    size_t i, n = 0;

    for (i = 0; mac[i] != '\0' && n + 1 < out_len; i++)
    {
        out[n++] = (mac[i] == ':') ? '_' : mac[i];
    }
    out[n] = '\0';
}

static int bt_add_device_unique_ex(struct bt_device *devices, int count, int max_devices,
    const char *mac, const char *name, bool paired)
{
    int i;

    if (!mac || !mac[0] || count >= max_devices)
        return count;

    for (i = 0; i < count; i++)
    {
        if (!strcasecmp(devices[i].mac, mac))
        {
            if (paired)
                devices[i].paired = true;
            if (name && name[0] &&
                (!devices[i].name[0] || !strcasecmp(devices[i].name, devices[i].mac)))
            {
                snprintf(devices[i].name, sizeof(devices[i].name), "%s", name);
            }
            return count;
        }
    }

    snprintf(devices[count].mac, sizeof(devices[count].mac), "%s", mac);
    if (name && name[0])
        snprintf(devices[count].name, sizeof(devices[count].name), "%s", name);
    else
        snprintf(devices[count].name, sizeof(devices[count].name), "%s", mac);
    devices[count].paired = paired;

    return count + 1;
}

static int bt_device_sort_cmp(const void *a, const void *b)
{
    const struct bt_device *da = a;
    const struct bt_device *db = b;

    if (da->paired != db->paired)
        return da->paired ? -1 : 1;

    return strcasecmp(da->name, db->name);
}

/* Run "bluetoothctl <subcmd> <mac>" and return true if success_str appears in output.
 * Pass NULL for success_str to skip output checking. */
static bool bt_ctl_run(const char *arg1, const char *arg2, const char *success_str)
{
    char cmd[256];
    snprintf(cmd, sizeof(cmd), "bluetoothctl %s %s | grep -q '%s'", arg1, arg2, success_str);
    int status = system(cmd);
    if (status == 0)
        return true;
    return false;
}

/* Parse "Device XX:XX:XX:XX:XX:XX Name" lines from a bluetoothctl command. */
static int  bt_parse_ctl_devices(const char *ctl_cmd, struct bt_device *devices,
                                 int count, int max_devices, bool paired)
{
    FILE *fp;
    char line[256];

    fp = popen(ctl_cmd, "r");
    if (!fp)
        return count;

    while (fgets(line, sizeof(line), fp))
    {
        const char *p = line;
        char mac[18] = "";
        char name[BT_NAME_LEN] = "";

        /* Strip leading junk (bluetoothctl can emit color codes / prompts) */
        while (*p && *p != 'D') p++;
        if (strncmp(p, "Device ", 7) != 0)
            continue;

        p += 7;
        if (strlen(p) < 17 || p[2] != ':' || p[5] != ':')
            continue;

        memcpy(mac, p, 17);
        mac[17] = '\0';
        p += 18;
        snprintf(name, sizeof(name), "%s", p);
        bt_trim(name);

        count = bt_add_device_unique_ex(devices, count, max_devices, mac, name, paired);
    }

    pclose(fp);
    return count;
}

/* Load paired devices instantly via bluetoothctl paired-devices. */
static int bt_load_devices_via_bluetoothctl(struct bt_device *devices, int max_devices)
{
    int count = bt_parse_ctl_devices("bluetoothctl paired-devices 2>/dev/null",
                                     devices, 0, max_devices, true);
    count = bt_parse_ctl_devices("bluetoothctl devices 2>/dev/null",
                             devices, count, max_devices, false);
    if (count > 1)
        qsort(devices, count, sizeof(devices[0]), bt_device_sort_cmp);
    return count;
}

static int bt_scan_devices(struct bt_device *devices, int count, int max_devices)
{
    int waited = 0;
    int action;
    FILE *fp;

    fp = popen("bluetoothctl", "w");
    if (!fp)
        return count;

    fprintf(fp, "scan on\n");
    fflush(fp);

    const int timeout = 15;
    while (waited < timeout)
    {
        splashf(0, "Scanning for %d/%d secs... Press any key to stop", waited, timeout);
        action = get_action(CONTEXT_STD, HZ);
        if (action != ACTION_NONE)
            break;
        waited++;
    }

    fprintf(fp, "scan off\n");
    fflush(fp);
    fprintf(fp, "exit\n");
    fflush(fp);
    pclose(fp);

    count = bt_parse_ctl_devices("bluetoothctl devices 2>/dev/null",
                                 devices, count, max_devices, false);
    if (count > 1)
        qsort(devices, count, sizeof(devices[0]), bt_device_sort_cmp);
    return count;
}

static int bt_choose_device(const char *title, struct bt_device *devices, int count)
{
    struct bt_device_menu_data data;
    struct simplelist_info info;
    int total_count = count + 1;

    if (total_count <= 0)
    {
        splash(HZ, "No devices");
        return BT_DEVICE_PICK_CANCEL;
    }

    data.devices = devices;
    data.count = count;

    simplelist_info_init(&info, (char *)title, total_count, &data);
    info.get_name = bt_device_name_cb;
    info.action_callback = bt_devicelist_callback;
    info.selection = -1;
    info.title_icon = Icon_Submenu;

    simplelist_show_list(&info);
    if (info.selection < 0 || info.selection >= total_count)
        return BT_DEVICE_PICK_CANCEL;

    if (info.selection == 0)
        return BT_DEVICE_PICK_SCAN;

    return info.selection - 1;
}

static void bt_set_selected(const struct bt_device *device)
{
    if (device)
    {
        snprintf(bt_selected_mac, sizeof(bt_selected_mac), "%s", device->mac);
        snprintf(bt_selected_name, sizeof(bt_selected_name), "%s", device->name);
    }
    else
    {
        bt_selected_mac[0] = '\0';
        bt_selected_name[0] = '\0';
        bt_active_codec[0] = '\0';
    }
}

static void bt_kick_audio_if_playing(void)
{
    int status = audio_status();
    if ((status & AUDIO_STATUS_PLAY) && !(status & AUDIO_STATUS_PAUSE))
    {
        audio_pause();
        sleep(HZ / 4);
        audio_resume();
    }
}

void bt_route_to_local(void)
{
    bt_playback_dev = BT_LOCAL_PLAYBACK_DEVICE;
    bt_active_codec[0] = '\0';
    hiby_pcm_set_bt_mac(NULL);
    pcm_alsa_switch_playback_device(bt_playback_dev);
    bt_kick_audio_if_playing();
}

static bool bt_route_to_bluetooth(const char *mac, const char* codec)
{
    int rc;

    if (!mac || !mac[0])
        return false;

    bt_playback_dev = bt_make_bt_playback_dev(mac, codec);

    if (!bt_wait_for_bluealsa_pcm(mac, HZ * 6))
    {
        bt_route_to_local();
        return false;
    }

    if (!codec)
        bt_set_active_codec(mac);
    else
    {
        //bt_set_active_codec hangs if called after codec switching. So just believe...
        strcpy(bt_active_codec, codec);
    }

    rc = -1;
    if (*bt_active_codec)
        rc = pcm_alsa_switch_playback_device(bt_playback_dev);
    if (rc == 0)
    {
        hiby_pcm_set_bt_mac(mac);
        bt_kick_audio_if_playing();
        button_add_input_device(BT_REMOTE_INPUT_IDX);
        return true;
    }

    bt_route_to_local();
    return false;
}

static bool bt_is_connected(const char *mac)
{
    if (!mac || !*mac)
        return false;

    return bt_bluealsa_pcm_ready(mac);
}

static bool bt_bluealsa_pcm_ready(const char *mac)
{
    char line[256];
    char mac_u[18];
    FILE *fp;

    if (!mac || !*mac)
        return false;

    bt_mac_to_underscore(mac, mac_u, sizeof(mac_u));
    fp = popen("bluealsa-cli list-pcms 2>/dev/null", "r");
    if (!fp)
        return false;

    while (fgets(line, sizeof(line), fp))
    {
        if (strstr(line, mac_u) && strstr(line, "/a2dpsrc/sink"))
        {
            pclose(fp);
            return true;
        }
    }

    pclose(fp);
    return false;
}

static void bt_get_device_name(char * mac, char *name_out)
{
    //TODO: reuse code from bt_ctl_run

    char cmd[128];
    char line[256];
    FILE *fp;

    if (!mac || !name_out)
        return;

    name_out[0] = '\0';
    snprintf(cmd, sizeof(cmd), "bluetoothctl info %s 2>/dev/null", mac);
    fp = popen(cmd, "r");
    if (!fp)
        return;

    while (fgets(line, sizeof(line), fp))
    {
        char *p = line;

        while (*p == ' ' || *p == '\t')
            p++;

        if (strncmp(p, "Name:", 5) != 0)
            continue;

        p += 5;
        while (*p == ' ' || *p == '\t')
            p++;

        bt_trim(p);
        strcpy(name_out, p);
        break;
    }

    pclose(fp);
}

static bool bt_get_active_mac(char *mac_out, size_t mac_out_len)
{
    FILE *fp;
    char line[256];

    if (!mac_out || mac_out_len < 18)
        return false;

    mac_out[0] = '\0';
    fp = popen("bluealsa-cli list-pcms 2>/dev/null", "r");
    if (fp)
    {
        while (fgets(line, sizeof(line), fp))
        {
            if (!strstr(line, "/a2dpsrc/sink"))
                continue;
            if (bt_extract_mac_from_line(line, mac_out, mac_out_len))
            {
                pclose(fp);
                return true;
            }
        }
        pclose(fp);
    }

    return false;
}

static bool bt_wait_for_bluealsa_pcm(const char *mac, int timeout_ticks)
{
    int ticks = 0;

    if (timeout_ticks < HZ / 2)
        timeout_ticks = HZ / 2;

    while (ticks < timeout_ticks)
    {
        if (bt_bluealsa_pcm_ready(mac))
            return true;

        sleep(HZ / 5);
        ticks += HZ / 5;
    }

    return false;
}

static bool bt_try_set_codec(const char *pcm_path, const char *codec)
{
    char cmd[256];
    int rc;

    snprintf(cmd, sizeof(cmd),
             "bluealsa-cli codec '%s' %s >/dev/null 2>&1",
             pcm_path, codec);
    rc = system(cmd);
    return (rc == 0);
}

static void bt_build_pcm_path(const char *mac, char *path, size_t path_len)
{
    char mac_u[18];
    bt_mac_to_underscore(mac, mac_u, sizeof(mac_u));
    snprintf(path, path_len, "/org/bluealsa/hci0/dev_%s/a2dpsrc/sink", mac_u);
}

void bt_bluealsa_change_volume(int l, int r, const char* mac)
{
    if (!mac || !mac[0])
        return;

    //With hardware volume, smaller value is used as volume, so keep that in mind
    char pcm_path[96];
    bt_build_pcm_path(mac, pcm_path, sizeof(pcm_path));
    char cmd[256];

    snprintf(cmd, sizeof(cmd), "bluealsa-cli volume '%s' %d %d >/dev/null 2>&1", pcm_path, l, r);
    system(cmd);
}

static void bt_set_active_codec(const char *mac)
{
    char pcm_path[96];
    char cmd[256];
    char line[256];
    FILE *fp;

    if (!mac || !*mac)
        return;

    bt_active_codec[0] = '\0';
    bt_build_pcm_path(mac, pcm_path, sizeof(pcm_path));
    //snprintf(cmd, sizeof(cmd), "bluealsa-cli codec '%s' 2>/dev/null", pcm_path);
    snprintf(cmd, sizeof(cmd), "bluealsa-cli info '%s' 2>/dev/null", pcm_path);
    fp = popen(cmd, "r");
    if (!fp)
        return;

    while (fgets(line, sizeof(line), fp))
    {
        if (strncmp(line, "Selected codec:", 15) == 0)
        {
            const char *p = line + 15;
            while (*p == ' ' || *p == '\t') p++;
            snprintf(bt_active_codec, sizeof(bt_active_codec), "%s", p);
            bt_trim(bt_active_codec);
            break;
        }
    }

    pclose(fp);
}

bool bt_disable(void)
{
    return bt_ctl_run("power", "off", "power off succeeded");
}

void wait_for_bt_init(void)
{
    static bool initialized = false;
    if (initialized)
        return;

    initialized = true;

    while (system("pgrep -f 'bt_init' > /dev/null 2>&1") == 0) 
    {
        splash(HZ/4,"BT initializing...");
    }
}

bool bt_enable(void)
{
     return bt_ctl_run("power", "on", "power on succeeded");
    //return system("/usr/bin/bt_enable | grep 'Powered: 1'", "r") == 0;
    //return system("bt-adapter --set \"Powered\" \"On\" | grep 'Powered: 1'") == 0;
}

static bool bt_prepare_stack(void)
{
    if (bt_enable())
        return true;

    splash(0, "Bluetooth is suspended. Resuming may take some time...");
    system("/usr/bin/bt_resume");
    splash(0, "Done.");
    int fd = open(BOOT_SETTING_FILE, O_RDWR | O_CREAT | O_TRUNC);
    close(fd);

    return bt_enable();
}

static void bt_show_devices(void)
{
    wait_for_bt_init();

    static struct bt_device devices[BT_MAX_DEVICES];
    int count;
    int idx;

    if (!bt_enable())
    {
        static const char *lines[] = {"Bluetooth is suspended.",
                              "Enable it?"};
        static const struct text_message message = {lines, 2};

        if (gui_syncyesno_run(&message, NULL, NULL) != YESNO_YES)
            return;

        if (!bt_prepare_stack())
        {
            //try to suspend again to avoid leaving it in a weird state
            bt_suspend();
            splash(HZ * 2, "BT unavailable");
            return;
        }
    }

    count = bt_load_devices_via_bluetoothctl(devices, BT_MAX_DEVICES);

    while (1)
    {
        idx = bt_choose_device("Devices", devices, count);
        if (idx == BT_DEVICE_PICK_SCAN)
        {
            count = bt_scan_devices(devices, count, BT_MAX_DEVICES);
            if (count <= 0)
                splash(HZ, "No devices found");
            continue;
        }

        if (idx >= 0 && idx < count)
            bt_connect_device(&devices[idx]);
        return;
    }
}

static void bt_connect_device(const struct bt_device *device)
{
    const char *mac;

    if (!device || !device->mac[0])
        return;

    mac = device->mac;

    // disallow for now multiple connections.
    if (bt_active_codec[0] && bt_selected_mac[0] && strcmp(bt_selected_mac, mac) != 0)
        bt_disconnect();

    splash(0, "Connecting...");

    if (!bt_prepare_stack())
    {
        splash(HZ * 2, "BT unavailable");
        return;
    }
    is_busy = true;

    if (!device->paired)
    {
        bt_ctl_run("trust", mac, NULL);
        if (!bt_ctl_run("pair", mac, "Pairing successful"))
        {
            splash(HZ * 2, "BT pair failed");
            is_busy = false;
            return;
        }
        //sleep(HZ / 2);
    }

    if (!bt_ctl_run("connect", mac, "Connection successful"))
    {
        char active_mac[18];
        // Make sure it's not a race with firmware auto-connection
        sleep(HZ/2);
        bt_get_active_mac(active_mac, sizeof(active_mac));
        if (strcmp(active_mac, mac) != 0)
        {
            splash(HZ * 2, "BT connect failed");
            is_busy = false;
            return;
        }
    }

    bt_set_selected(device);

    if (bt_route_to_bluetooth(mac, NULL))
        splash(HZ, "BT connected");
    else
        splash(HZ * 2, "BT connected, no audio route");

    is_busy = false;
}

static void bt_disconnect(void)
{
    is_busy = true;
    button_remove_input_device(BT_REMOTE_INPUT_IDX);

    //bt_set_selected(NULL);
    bt_active_codec[0] = 0;
    bt_route_to_local();

    char mac[18];
    char cmd[96];

    if (bt_get_active_mac(mac, sizeof(mac)))
    {
        splash(0, "Disconnecting...");
        snprintf(cmd, sizeof(cmd), "bluetoothctl disconnect %s >/dev/null 2>&1", mac);
        system(cmd);
    }

    is_busy = false;
    splash(HZ/4, "Disconnected");
}

static bool bt_is_enabled(void)
{
    FILE *fp;

    fp = popen("bluetoothctl show 2>/dev/null | grep -q 'Powered: yes'", "r");
    //fp = popen("bt-adapter -i 2>/dev/null | grep 'Powered: 1'", "r");

    if (fp && pclose(fp) == 0)
    {
        return true;
    }
    //splash(0,"NOT ENABLED!");
    return false;
}

static int bt_get_available_codecs(const char *mac,
                                   char codecs[][BT_CODEC_NAME_LEN],
                                   int max_codecs)
{
    char pcm_path[96];
    char cmd[256];
    char line[256];
    FILE *fp;
    int count = 0;

    if (!mac || !*mac)
        return 0;

    bt_build_pcm_path(mac, pcm_path, sizeof(pcm_path));
    snprintf(cmd, sizeof(cmd), "bluealsa-cli codec '%s' 2>/dev/null", pcm_path);
    fp = popen(cmd, "r");
    if (!fp)
        return 0;

    while (fgets(line, sizeof(line), fp))
    {
        if (strncmp(line, "Available codecs:", 17) == 0)
        {
            char *p = line + 17;
            while (*p && count < max_codecs)
            {
                int i = 0;
                while (*p == ' ' || *p == '\t') p++;
                if (*p == '\0' || *p == '\n' || *p == '\r') break;
                while (*p && *p != ' ' && *p != '\t' &&
                       *p != '\n' && *p != '\r' &&
                       i < BT_CODEC_NAME_LEN - 1)
                    codecs[count][i++] = *p++;
                codecs[count][i] = '\0';
                if (i > 0)
                    count++;
            }
            break;
        }
    }

    pclose(fp);
    return count;
}

static void bt_show_codec_picker(const char *mac)
{
    static char codecs[BT_MAX_CODECS][BT_CODEC_NAME_LEN];
    struct bt_strlist_data data;
    struct simplelist_info info;
    int count;

    count = bt_get_available_codecs(mac, codecs, BT_MAX_CODECS);
    if (count <= 0)
    {
        splash(HZ, "No codecs available");
        return;
    }

    data.items = codecs;
    data.count = count;

    simplelist_info_init(&info, "Select Codec", count, &data);
    info.get_name = bt_strlist_name_cb;
    info.action_callback = bt_simplelist_ok_cancel;
    info.selection = -1;
    info.title_icon = Icon_Submenu;

    simplelist_show_list(&info);

    if (info.selection >= 0 && info.selection < count)
    {
        //pcm_alsa_close_device(bt_playback_dev);
        bt_route_to_local();
        char pcm_path[96];
        bt_build_pcm_path(mac, pcm_path, sizeof(pcm_path));
        //This seems to work more reliable with active playback, but setting seems doesn't stick through reboots (?)
        //if (bt_route_to_bluetooth(mac, codecs[info.selection]))

        if (bt_try_set_codec(pcm_path, codecs[info.selection]) && bt_route_to_bluetooth(mac, NULL))
        {
            //bt_set_active_codec(mac);
            splashf(HZ, "Codec: %s", bt_active_codec );
        }
        else
            splash(HZ, "Codec change failed.");
    }
}

bool bt_can_autoconnect(void)
{
    return !is_busy && pcm_is_initialized();
}

bool bt_autoconnection_route_to_bluetooth(char* active_mac, bool bt_on)
{
    if (is_busy)
        return false;

    is_busy = true;
    bool bt_connected = bt_on && bt_get_active_mac(active_mac, 18);
    bool active_mac_changed = bt_connected && (strcmp(bt_selected_mac, active_mac) != 0);
    /* Auto-route to BT if headphone is connected but output is still local (connected by Hiby OS) */
    if (bt_connected && 
        (active_mac_changed || strcmp(bt_playback_dev, BT_LOCAL_PLAYBACK_DEVICE) == 0 ))
    {
       //splash(0, "Bluetooth connection detected.\nRouting audio to Bluetooth...");
        //TODO: Auto connection seems to ignore codec preference
        // Should we do full reconnection?
        if (active_mac_changed)
        {
            bt_get_device_name(active_mac, bt_selected_name);
            strcpy(bt_selected_mac, active_mac);
        }
        bt_route_to_bluetooth(active_mac, NULL);
        //splash(HZ/5, "Done.");
    }
    is_busy = false;
    return bt_connected;
}

static void bt_show_status(void)
{
    struct simplelist_info info;
    char active_mac[18];
    bool bt_on = false;
    int sel;

    bool suspended = bt_is_suspended_fast();
    if (!suspended)
    {
        wait_for_bt_init();
        bt_on = bt_is_enabled();
    }
    while (1)
    {
        bool bt_connected = bt_autoconnection_route_to_bluetooth(active_mac, bt_on);

        int line_idx = 0;
        int bt_toggle_line;
        int codec_line = -1;
        int device_line = -1;
        int executed_action = -1;

        simplelist_info_init(&info, "Status", 0, &executed_action);
        info.action_callback = bt_simplelist_ok_cancel_return_action;
        info.selection = -1;
        simplelist_reset_lines();

        simplelist_addline("Bluetooth: %s", bt_on ? "Enabled" : (suspended ? "Suspended" : "Disabled"));
        bt_toggle_line = line_idx++;

        if (bt_selected_mac[0])
        {
            simplelist_addline("Device: %s", bt_selected_name[0] ? bt_selected_name :  "Bluetooth");
            device_line = line_idx++;

            if (bt_connected)
            {
                simplelist_addline("Codec: %s",
                       bt_active_codec[0] ? bt_active_codec : "Unknown");
                codec_line = line_idx++;
            }
            simplelist_addline("MAC: %s", bt_selected_mac);
            line_idx++;
            simplelist_addline("Connected: %s",
                               bt_connected ? "Yes" : "No");
            line_idx++;

        }
        else
        {
            simplelist_addline("Device: Local");
            line_idx++;
        }

        simplelist_addline("Output: %s", bt_playback_dev);
        line_idx++;

        info.count = simplelist_get_line_count();
        simplelist_show_list(&info);

        sel = info.selection;
        if (sel < 0)
            break;

        if (sel == bt_toggle_line)
        {
            static const char *const toggle_items[] = { "On", "Off", "Suspend" };
            struct simplelist_info t_info;
            simplelist_info_init(&t_info, "Bluetooth", 3, (void *)toggle_items);
            t_info.get_name = bt_action_name_cb;
            t_info.action_callback = bt_simplelist_ok_cancel;
            t_info.selection = -1;
            simplelist_show_list(&t_info);
            bool suspend = t_info.selection == 2; 

            if (t_info.selection == 0)
            {
                bt_on = bt_prepare_stack();
            }
            else if (t_info.selection > 0)
            {
                button_remove_input_device(BT_REMOTE_INPUT_IDX);
                bt_route_to_local();
                if (suspend)
                {
                    bt_suspend();
                    remove(BOOT_SETTING_FILE);
                }
                else
                {
                    bt_disable();
                }
                
                bt_on = false;
            }
            suspended = !bt_on && suspend;
        }
        else if (sel == codec_line && active_mac[0])
        {
            bt_show_codec_picker(active_mac);
        }
        else if (sel == device_line && bt_on && bt_selected_mac[0])
        {
            if (bt_connected && executed_action == ACTION_STD_CONTEXT)
            {
                bt_disconnect();
                continue;
            }
            struct bt_device device;
            strcpy(device.mac, bt_selected_mac);
            strcpy(device.name, bt_selected_name);
            device.paired = true;
            bt_connect_device(&device);
        }
        /* other lines: just re-show status */
    }
}

int hiby_bluetooth_menu(void)
{
    static const char *const action_items[] =
    {
        "Status",
        "Devices",
        "Disconnect",
    };

    int action = -1;

    while (true)
    {
        struct simplelist_info info;

        simplelist_info_init(&info, "Bluetooth",
            (int)(sizeof(action_items) / sizeof(action_items[0])),
            (void *)action_items);
        info.get_name = bt_action_name_cb;
        info.action_callback = bt_simplelist_ok_cancel;
        info.selection = -1;
        info.title_icon = Icon_Submenu;

        simplelist_show_list(&info);
        action = info.selection;
        if (action < 0)
            break;

        switch (action)
        {
            case 0:
                bt_show_status();
                break;
            case 1:
                bt_show_devices();
                break;
            case 2:
                bt_disconnect();
                break;
            default:
                break;
        }
    }
    return 0;
}

#endif /* HIBY_LINUX */
