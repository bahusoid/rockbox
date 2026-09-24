/***************************************************************************
*             __________               __   ___.
*   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
*   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
*   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
*   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
*                     \/            \/     \/    \/            \/
* $Id$
*
* JPEG image viewer
* Common structs and defines for plugin and core JPEG decoders
*
* File scrolling addition (C) 2005 Alexander Spyridakis
* Copyright (C) 2004 Jörg Hohensohn aka [IDC]Dragon
* Heavily borrowed from the IJG implementation (C) Thomas G. Lane
* Small & fast downscaling IDCT (C) 2002 by Guido Vollbeding  JPEGclub.org
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

#ifndef _ALBUMART_LOAD_COMMON_H
#define _ALBUMART_LOAD_COMMON_H
#include "albumart_load_common.h"
#include <stddef.h>
#include <stdbool.h>
#include "plugin.h"
#include "debug.h"
#include "jpeg_load.h"
#include "metadata_common.h"

/* 
 * 1. Define the common fields in a macro.
 * Note: The function pointers still explicitly expect `struct file_buffer*`. 
 * This is correct, as the underlying API will pass the base type.
 */
#define FILE_BUFFER_FIELDS                                         \
    int fd;                                                        \
    int buf_left;                                                  \
    int buf_index;                                                 \
    int (*read_buf)(struct file_buffer* p_jpeg, size_t count);     \
    bool (*skip_bytes_seek)(struct file_buffer* p_jpeg);           \
    void* custom_param;                                            \
    unsigned long len;                                             \
    unsigned char buf[JPEG_READ_BUF_SIZE];


/* 2. Define the base struct */
struct file_buffer {
    FILE_BUFFER_FIELDS
};

int read_buf_id3_unsync(struct file_buffer* p_jpeg, size_t count)
{
    count = read(p_jpeg->fd, p_jpeg->buf, count);
    return id3_unsynchronize(p_jpeg->buf, count, (bool*) &p_jpeg->custom_param);
}

int read_buf_vorbis_base64(struct file_buffer* p_jpeg, size_t count)
{
    struct ogg_file* ogg = p_jpeg->custom_param;
    unsigned char* buf = p_jpeg->buf;
    count = ogg_file_read(ogg, buf, count);
    if (count == (size_t) -1)
        return 0;

    return base64_decode(buf, count, buf);
}

static inline void fill_buf(struct file_buffer* p_jpeg)
{
    p_jpeg->buf_left = p_jpeg->read_buf(p_jpeg, MIN(JPEG_READ_BUF_SIZE, p_jpeg->len));
    p_jpeg->buf_index = 0;
    if (p_jpeg->buf_left > 0)
        p_jpeg->len -= p_jpeg->buf_left;
}

unsigned char *filebuf_getc(struct file_buffer* p_jpeg)
{
    if (UNLIKELY(p_jpeg->buf_left < 1))
        fill_buf(p_jpeg);
    if (UNLIKELY(p_jpeg->buf_left < 1))
        return NULL;
    p_jpeg->buf_left--;
    return (p_jpeg->buf_index++) + p_jpeg->buf;
}

/* when pjpeg->read_buf involves additional data processing (like base64 decoding)
 * we can't use lseek and have to call pjpeg->read_buf for proper seek */
bool skip_bytes_read_buf(struct file_buffer* p_jpeg)
{
    do
    {
        int count = -p_jpeg->buf_left;
        fill_buf(p_jpeg);
        if (p_jpeg->buf_left < 0)
            return false;
        p_jpeg->buf_left -= count;
        p_jpeg->buf_index += count;
    } while (p_jpeg->buf_left < 0);
    return true;
}
static int read_buf(struct file_buffer* p_jpeg, size_t count)
{
    return read(p_jpeg->fd, p_jpeg->buf, count);
}
static bool skip_bytes_seek(struct file_buffer* p_jpeg)
{
    if (UNLIKELY(lseek(p_jpeg->fd, -p_jpeg->buf_left, SEEK_CUR) < 0))
        return false;
    p_jpeg->buf_left = 0;
    return true;
}

bool skip_bytes(struct file_buffer* p_jpeg, int count)
{
    p_jpeg->buf_left -= count;
    p_jpeg->buf_index += count;
    return p_jpeg->buf_left >= 0 || p_jpeg->skip_bytes_seek(p_jpeg);
}

int init_file_buffer(struct file_buffer *p_jpeg, int fd, int flags,
                          unsigned char *buf_format, struct ogg_file *ogg)
{
    p_jpeg->fd = fd;
    if (p_jpeg->len == 0)
        p_jpeg->len = filesize(p_jpeg->fd);

    p_jpeg->read_buf = read_buf;
    p_jpeg->skip_bytes_seek = skip_bytes_seek;

#ifdef HAVE_ALBUMART
    if (flags & AA_FLAG_ID3_UNSYNC)
    {
        p_jpeg->read_buf = read_buf_id3_unsync;
        p_jpeg->custom_param = false;
    }
    else if (flags & AA_FLAG_VORBIS_BASE64)
    {
        off_t pic_pos = lseek(fd, 0, SEEK_CUR);
        int type = get_ogg_format_and_move_to_comments(fd, buf_format);

        ogg_file_init(ogg, fd, type, 0);
        bool packet_found;
        do
        {
            int seek_from_cur_pos = pic_pos - lseek(fd, 0, SEEK_CUR);
            packet_found = seek_from_cur_pos <= ogg->packet_remaining;
            if (ogg_file_read(ogg, NULL, packet_found ? seek_from_cur_pos : ogg->packet_remaining) < 0)
                return -1;
        }
        while (!packet_found);

        p_jpeg->read_buf = read_buf_vorbis_base64;
        p_jpeg->skip_bytes_seek = skip_bytes_read_buf;
        p_jpeg->custom_param = ogg;
    }
#else
    (void)flags;
#endif /* HAVE_ALBUMART */

    return 0;
}

#endif /* _ALBUMART_LOAD_COMMON_H */
