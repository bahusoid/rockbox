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
#include <stddef.h>
#include <stdbool.h>
#include "jpeg_common.h"


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
int read_buf_id3_unsync(struct file_buffer* p_jpeg, size_t count);
int read_buf_vorbis_base64(struct file_buffer* p_jpeg, size_t count);
bool skip_bytes_read_buf(struct file_buffer* p_jpeg);
unsigned char *filebuf_getc(struct file_buffer* p_jpeg);
int init_file_buffer(struct file_buffer *p_jpeg, int fd, int flags,
                          unsigned char *buf_format, struct ogg_file *ogg);
bool skip_bytes(struct file_buffer* p_jpeg, int count);

#endif /* _ALBUMART_LOAD_COMMON_H */
