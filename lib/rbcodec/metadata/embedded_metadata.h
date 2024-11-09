/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2005 Dave Chapman
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
//#include <stdio.h>
//#include <stdlib.h>
//#include <ctype.h>
//#include <inttypes.h>
#include "platform.h"


int get_ogg_format_and_move_to_comments(int fd, unsigned char *buf);

int b64_decode(const char *in, size_t in_len, unsigned char *out,  size_t *outlen);

struct file
{
    int fd;
    bool packet_ended;
    long packet_remaining;
};

bool file_init(struct file* file, int fd, int type, int remaining);
ssize_t file_read(struct file* file, void* buffer, size_t buffer_size);
bool file_read_page_header(struct file* file);
int id3_unsynchronize(char* tag, int len, bool *ff_found);
