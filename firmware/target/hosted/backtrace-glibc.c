/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (c) 2017 Amaury Pouly
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
#include "system.h"
#include "font.h"
#include "lcd.h"
#include <execinfo.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define HOSTED_BT_LINE_CHARS ((LCD_WIDTH / SYSFONT_WIDTH) - 2)

static void hosted_backtrace_line(unsigned *line, const char *text)
{
    size_t len = strlen(text);
    size_t pos = 0;

    while (pos < len)
    {
        size_t chunk = len - pos;
        if (chunk > HOSTED_BT_LINE_CHARS)
            chunk = HOSTED_BT_LINE_CHARS;

        static char buf[HOSTED_BT_LINE_CHARS + 1];
        memcpy(buf, text + pos, chunk);
        buf[chunk] = '\0';

        lcd_puts(0, (*line)++, (unsigned char *)buf);
        pos += chunk;
    }
}

/* backtrace from the call-site of this function */
void rb_backtrace(int pc, int sp, unsigned *line)
{
    /* ignore SP and PC */
    (void) pc;
    (void) sp;

    /* backtrace */
    #define BT_BUF_SIZE 100
    void *buffer[BT_BUF_SIZE];
    int count = backtrace(buffer, BT_BUF_SIZE);
    /* print symbols to stdout for debug */
    fprintf(stdout, "backtrace:\n");
    fflush(stdout);
    backtrace_symbols_fd(buffer, count, STDOUT_FILENO);
    /* print on screen */
    char **strings;
    strings = backtrace_symbols(buffer, count);
    if(strings == NULL)
    {
        perror("backtrace_symbols");
        return;
    }

    for(int i = 0; i < count; i++)
    {
        fprintf(stderr, "bt[%d]: %s\n", i, strings[i]);
        hosted_backtrace_line(line, strings[i]);
        lcd_update();
    }

    free(strings);
}
