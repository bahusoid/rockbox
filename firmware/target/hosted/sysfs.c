/***************************************************************************
 *             __________               __   ___
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * Copyright (C) 2014 by Ilia Sergachev: Initial Rockbox port to iBasso DX50
 * Copyright (C) 2014 by Mario Basister: iBasso DX90 port
 * Copyright (C) 2014 by Simon Rothen: Initial Rockbox repository submission, additional features
 * Copyright (C) 2014 by Udo Schläpfer: Code clean up, additional features
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


#include <stdio.h>
#include <string.h>

#include "config.h"
#include "debug.h"
#include "sysfs.h"
//#define COLLECT_SYSFS_ACCESS_LOG
#ifdef COLLECT_SYSFS_ACCESS_LOG
#include "file.h"

#define SYSFS_ACCESS_LOG_LIMIT 100

struct sysfs_access_record {
    char name[128];
    unsigned int writes;
    unsigned int reads;
};

static struct sysfs_access_record sysfs_access_log[SYSFS_ACCESS_LOG_LIMIT];
static size_t sysfs_access_log_count = 0;
static size_t sysfs_access_log_next_slot = 0;

static void sysfs_record_access(const char *file_name, bool is_write)
{
    if (file_name == NULL || file_name[0] == '\0')
    {
        return;
    }

    for (size_t i = 0; i < sysfs_access_log_count; ++i)
    {
        if (strcmp(sysfs_access_log[i].name, file_name) == 0)
        {
            if (is_write)
                sysfs_access_log[i].writes++;
            else
                sysfs_access_log[i].reads++;
            return;
        }
    }

    if (sysfs_access_log_count < SYSFS_ACCESS_LOG_LIMIT)
    {
        struct sysfs_access_record *record = &sysfs_access_log[sysfs_access_log_count++];
        memset(record, 0, sizeof(*record));
        snprintf(record->name, sizeof(record->name), "%s", file_name);
        if (is_write)
            record->writes = 1;
        else
            record->reads = 1;
        return;
    }

    struct sysfs_access_record *record = &sysfs_access_log[sysfs_access_log_next_slot];
    memset(record, 0, sizeof(*record));
    snprintf(record->name, sizeof(record->name), "%s", file_name);
    if (is_write)
        record->writes = 1;
    else
        record->reads = 1;
    sysfs_access_log_next_slot = (sysfs_access_log_next_slot + 1) % SYSFS_ACCESS_LOG_LIMIT;
}

void sysfs_debug_save_access_log(const char *path)
{
    if (path == NULL || path[0] == '\0')
    {
        return;
    }

    int f = open(ROCKBOX_DIR "/logf.txt", O_CREAT|O_WRONLY|O_TRUNC, 0666);
    if (f < 0)
    {
        DEBUGF("ERROR %s: Can not open %s for writing.", __func__, path);
        return;
    }

    fdprintf(f, "name\treads\twrites\n");
    for (size_t i = 0; i < SYSFS_ACCESS_LOG_LIMIT; ++i)
    {
        if (sysfs_access_log[i].name[0] == '\0')
            continue;

        fdprintf(f, "%s\t%u\t%u\n",
                sysfs_access_log[i].name,
                sysfs_access_log[i].reads,
                sysfs_access_log[i].writes);
    }

    close(f);
}

void sysfs_debug_reset_access_log(void)
{
    memset(sysfs_access_log, 0, sizeof(sysfs_access_log));
    sysfs_access_log_count = 0;
    sysfs_access_log_next_slot = 0;
}
#else
#define sysfs_record_access(file_name, is_write) do {} while(0)
#endif

static FILE* open_read(const char *file_name)
{
    sysfs_record_access(file_name, false);

    FILE *f = fopen(file_name, "re");
    if(f == NULL)
    {
        DEBUGF("ERROR %s: Can not open %s for reading.", __func__, file_name);
    }

    return f;
}


static FILE* open_write(const char* file_name)
{
    sysfs_record_access(file_name, true);

    FILE *f = fopen(file_name, "we");
    if(f == NULL)
    {
        DEBUGF("ERROR %s: Can not open %s for writing.", __func__, file_name);
    }

    return f;
}


bool sysfs_get_int(const char *path, int *value)
{
    *value = -1;

    FILE *f = open_read(path);
    if(f == NULL)
    {
        return false;
    }

    bool success = true;
    if(fscanf(f, "%d", value) == EOF)
    {
        DEBUGF("ERROR %s: Read failed for %s.", __func__, path);
        success = false;
    }

    fclose(f);
    return success;
}


bool sysfs_set_int(const char *path, int value)
{
    FILE *f = open_write(path);
    if(f == NULL)
    {
        return false;
    }

    bool success = true;
    if(fprintf(f, "%d", value) < 0)
    {
        DEBUGF("ERROR %s: Write failed for %s.", __func__, path);
        success = false;
    }

    fclose(f);
    return success;
}


bool sysfs_get_char(const char *path, char *value)
{
    int c;
    FILE *f = open_read(path);
    if(f == NULL)
    {
        return false;
    }

    bool success = true;
    c = fgetc(f);

    if(c == EOF)
    {
        DEBUGF("ERROR %s: Read failed for %s.", __func__, path);
        success = false;
    }
    else
    {
        *value = c;
    }

    fclose(f);
    return success;
}


bool sysfs_set_char(const char *path, char value)
{
    FILE *f = open_write(path);
    if(f == NULL)
    {
        return false;
    }

    bool success = true;
    if(fprintf(f, "%c", value) < 1)
    {
        DEBUGF("ERROR %s: Write failed for %s.", __func__, path);
        success = false;
    }

    fclose(f);
    return success;
}


bool sysfs_get_string(const char *path, char *value, int size)
{
    value[0] = '\0';
    FILE *f = open_read(path);
    if(f == NULL)
    {
        return false;
    }

    bool success = true;

    /* fgets returns NULL if en error occured OR
     * when EOF occurs while no characters have been read.
     *
     * Empty string is not an error for us.
     */
    if(fgets(value, size, f) == NULL && value[0] != '\0')
    {
        DEBUGF("ERROR %s: Read failed for %s.", __func__, path);
        success = false;
    }
    else
    {
        size_t length = strlen(value);
        if((length > 0) && value[length - 1] == '\n')
        {
            value[length - 1] = '\0';
        }
    }

    fclose(f);
    return success;
}


bool sysfs_set_string(const char *path, char *value)
{
    FILE *f = open_write(path);
    if(f == NULL)
    {
        return false;
    }

    bool success = true;

    /* If an output error is encountered, a negative value is returned */
    if(fprintf(f, "%s", value) < 0)
    {
        DEBUGF("ERROR %s: Write failed for %s.", __func__, path);
        success = false;
    }

    fclose(f);
    return success;
}
