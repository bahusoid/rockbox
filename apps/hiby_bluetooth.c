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

#ifdef HIBY_LINUX


#include <ctype.h>

#include "kernel.h"
#include "audio.h"
#include "action.h"
#include "button-devinput.h"
#include "menu.h"
#include "misc.h"
#include "splash.h"
#include "gui/list.h"
#include "pcm-alsa.h"

/* HiBy hosted build provides dynamic output routing helper in its
 * target-specific PCM implementation. */
int pcm_alsa_switch_playback_device(const char *device);
void pcm_alsa_close_device(const char *device);
void hiby_pcm_set_bt_mac(const char *mac);
static bool bt_ctl_run(const char *subcmd, const char *mac, const char *success_str);
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
static int bt_devicelist_callback(int action, struct gui_synclist *lists)
{
    (void)lists;
    if (action == ACTION_STD_OK)
        return ACTION_STD_CANCEL;
    if (action == ACTION_STD_CONTEXT)
    {
        struct bt_device_menu_data* ctx = lists->data;
        struct bt_device bt_device = ctx->devices[lists->selected_item - 1];
        if (bt_device.paired && confirm_delete_yesno(bt_device.name) == 0)
        {
            if (*bt_active_codec && strcmp(bt_selected_mac, bt_device.mac) == 0)
                bt_disconnect();

            bt_ctl_run("remove", bt_device.mac, NULL);
            bt_device.paired = false;
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
static bool bt_ctl_run(const char *subcmd, const char *mac, const char *success_str)
{
    char cmd[128];
    char line[256];
    bool success = false;
    FILE *fp;

    snprintf(cmd, sizeof(cmd), "bluetoothctl %s %s 2>&1", subcmd, mac);
    fp = popen(cmd, "r");
    if (!fp)
        return false;

    while (fgets(line, sizeof(line), fp))
    {
        if (success_str && strstr(line, success_str))
            success = true;
    }

    pclose(fp);
    return success;
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

static void bt_set_selected_mac(const char *mac)
{
    if (mac && mac[0])
        snprintf(bt_selected_mac, sizeof(bt_selected_mac), "%s", mac);
    else
        bt_selected_mac[0] = '\0';
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

static void bt_route_to_local(bool show_message)
{
    bt_playback_dev = BT_LOCAL_PLAYBACK_DEVICE;
    bt_active_codec[0] = '\0';
    hiby_pcm_set_bt_mac(NULL);
    pcm_alsa_switch_playback_device(bt_playback_dev);
    bt_kick_audio_if_playing();
    if (show_message)
        splash(HZ, "Output: Local");
}

static bool bt_route_to_bluetooth(const char *mac, const char* codec)
{
    int rc;

    if (!mac || !mac[0])
        return false;

    bt_playback_dev = bt_make_bt_playback_dev(mac, codec);

    if (!bt_wait_for_bluealsa_pcm(mac, HZ * 6))
    {
        bt_route_to_local(false);
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

    bt_route_to_local(false);
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

    if (bt_selected_mac[0] && bt_is_connected(bt_selected_mac))
    {
        snprintf(mac_out, mac_out_len, "%s", bt_selected_mac);
        return true;
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

static bool bt_enable(void)
{
    FILE* fp;

    //fp = popen("/usr/bin/bt_enable | grep 'Powered: 1'", "r");
    fp = popen("bluetoothctl power on | grep -q 'power on succeeded'", "r");

    if (fp && pclose(fp) == 0)
        return true;

    //splash(0, "FAILED TO POWER ON");
    return false;
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
    static struct bt_device devices[BT_MAX_DEVICES];
    int count;
    int idx;

    if (!bt_prepare_stack())
    {
        splash(HZ * 2, "BT unavailable");
        return;
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
    splash(0, "Connecting...");

    if (!bt_prepare_stack())
    {
        splash(HZ * 2, "BT unavailable");
        return;
    }

    if (!device->paired)
    {
        bt_ctl_run("trust", mac, NULL);
        if (!bt_ctl_run("pair", mac, "Pairing successful"))
        {
            splash(HZ * 2, "BT pair failed");
            return;
        }
        //sleep(HZ / 2);
    }

    if (!bt_ctl_run("connect", mac, "Connection successful"))
    {
        splash(HZ * 2, "BT connect failed");
        return;
    }

    bt_set_selected_mac(mac);

    if (bt_route_to_bluetooth(mac, NULL))
        splash(HZ, "BT connected");
    else
        splash(HZ * 2, "BT connected, no audio route");
}

static void bt_disconnect(void)
{
    char mac[18];
    char cmd[96];

    button_remove_input_device(BT_REMOTE_INPUT_IDX);
    mac[0] = '\0';
    if (!bt_get_active_mac(mac, sizeof(mac)) && bt_selected_mac[0])
        snprintf(mac, sizeof(mac), "%s", bt_selected_mac);

    if (mac[0])
    {
        snprintf(cmd, sizeof(cmd), "bluetoothctl disconnect %s >/dev/null 2>&1", mac);
        system(cmd);
    }

    bt_set_selected_mac(NULL);
    bt_active_codec[0] = '\0';
    bt_route_to_local(false);
    splash(HZ, "Disconnected");
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
    char pcm_path[96];
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
        //bt_build_pcm_path(mac, pcm_path, sizeof(pcm_path));
        pcm_alsa_close_device(bt_playback_dev);
        if (bt_route_to_bluetooth(mac, codecs[info.selection]))
        {
            //bt_set_active_codec(mac);
            splashf(HZ, "Codec: %s", bt_active_codec );
        }
        else
            splash(HZ, "Codec change failed.");
    }
}

static void bt_show_status(void)
{
    struct simplelist_info info;
    char active_mac[18];
    bool bt_on = false;
    int sel;

    bt_on = bt_is_enabled();
    // /* Auto-route to BT if headphone is connected but output is still local */
    if (bt_on == 1 && bt_get_active_mac(active_mac, sizeof(active_mac)) && strcmp(bt_playback_dev, BT_LOCAL_PLAYBACK_DEVICE) == 0)
    {
        bt_route_to_bluetooth(active_mac, NULL);
    }

    while (1)
    {
        int line_idx = 0;
        int bt_toggle_line;
        int codec_line = -1;

        active_mac[0] = '\0';

        simplelist_info_init(&info, "Status", 0, NULL);
        info.action_callback = bt_simplelist_ok_cancel;
        info.selection = -1;
        simplelist_reset_lines();

        simplelist_addline("Bluetooth: %s", bt_on ? "Enabled" : "Disabled");
        bt_toggle_line = line_idx++;

        if (bt_on && bt_get_active_mac(active_mac, sizeof(active_mac)))
        {
            simplelist_addline("Device: Bluetooth");
            line_idx++;
            simplelist_addline("MAC: %s", active_mac);
            line_idx++;
            simplelist_addline("Connected: %s",
                               bt_is_connected(active_mac) ? "Yes" : "No");
            line_idx++;
            simplelist_addline("Codec: %s",
                               bt_active_codec[0] ? bt_active_codec : "Unknown");
            codec_line = line_idx++;
        }
        else if (bt_selected_mac[0])
        {
            simplelist_addline("Device: Last selected");
            line_idx++;
            simplelist_addline("MAC: %s", bt_selected_mac);
            line_idx++;
            simplelist_addline("Connected: No");
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
            if (bt_on)
            {
                button_remove_input_device(BT_REMOTE_INPUT_IDX);
                bt_route_to_local(false);
                system("/usr/bin/bt_suspend");
                remove(BOOT_SETTING_FILE);
                bt_on = false;
            }
            else
            {
                bt_on = bt_prepare_stack();
            }
        }
        else if (sel == codec_line && active_mac[0])
        {
            bt_show_codec_picker(active_mac);
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
