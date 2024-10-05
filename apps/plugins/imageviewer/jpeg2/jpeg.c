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
* (This is a real mess if it has to be coded in one single C file)
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
#include "jpeg81.h"
#include "GETC.h"
#include "plugin.h"
#include "tlsf.h"

#include "../imageviewer.h"

#ifdef HAVE_LCD_COLOR
#include "yuv2rgb.h"
#endif

/**************** begin Application ********************/

/************************* Types ***************************/

struct t_disp
{
#ifdef HAVE_LCD_COLOR
    unsigned char* bitmap[3]; /* Y, Cr, Cb */
    int csub_x, csub_y;
#else
    unsigned char* bitmap[1]; /* Y only */
#endif
    int stride;
};

/************************* Globals ***************************/

/* decompressed image in the possible sizes (1,2,4,8), wasting the other */
static struct t_disp disp[9];

/* my memory pool (from the mp3 buffer) */
static char print[32]; /* use a common snprintf() buffer */

/* the root of the images, hereafter are decompresed ones */
static unsigned char* buf_root;
static int root_size;


static struct JPEGD jpg; /* too large for stack */

/************************* Implementation ***************************/

static void draw_image_rect(struct image_info *info,
                            int x, int y, int width, int height)
{
    struct t_disp* pdisp = (struct t_disp*)info->data;
#ifdef HAVE_LCD_COLOR
    yuv_bitmap_part(
        pdisp->bitmap, pdisp->csub_x, pdisp->csub_y,
        info->x + x, info->y + y, pdisp->stride,
        x + MAX(0, (LCD_WIDTH - info->width) / 2),
        y + MAX(0, (LCD_HEIGHT - info->height) / 2),
        width, height,
        iv->settings->jpeg_colour_mode, iv->settings->jpeg_dither_mode);
#else
    mylcd_ub_gray_bitmap_part(
        pdisp->bitmap[0], info->x + x, info->y + y, pdisp->stride,
        x + MAX(0, (LCD_WIDTH-info->width)/2),
        y + MAX(0, (LCD_HEIGHT-info->height)/2),
        width, height);
#endif
}

static int get_x_phys()
{
    return jpg.X + jpg.P;
}

static int get_y_phys()
{
    return jpg.Y + jpg.P;
}

//#define x_phys X + p_jpg->P
//#define y_phys Y + p_jpg->P
static int img_mem(int ds)
{
    int size;
    struct JPEGD *p_jpg = &jpg;
    int x_phys = p_jpg->X + p_jpg->P;
    int y_phys = p_jpg->Y + p_jpg->P;
    int h0= p_jpg->Hmax / p_jpg->Components[0].Hi;
    int v0= p_jpg->Vmax / p_jpg->Components[0].Vi;
    int h1= p_jpg->Hmax / p_jpg->Components[1].Hi;
    int v1= p_jpg->Vmax / p_jpg->Components[1].Vi;
    int h2= p_jpg->Hmax / p_jpg->Components[2].Hi;
    int v2= p_jpg->Vmax / p_jpg->Components[2].Vi;
    size = (x_phys  /ds/h0)
         * (y_phys/ds/v0);
#ifdef HAVE_LCD_COLOR
    //if (p_jpg->blocks > 1) /* colour, add requirements for chroma */
    if (p_jpg->Nf == 3) //TODO???? 
    {
        size += (x_phys/ds/h1)
              * (y_phys/ds/v1);
        size += (x_phys/ds/h2)
              * (y_phys/ds/v2);
    }
#endif
    return size;
}

static int load_image(char *filename, struct image_info *info,
                      unsigned char *buf, ssize_t *buf_size)
{
    int status;
    struct JPEGD *p_jpg = &jpg;

    rb->memset(&disp, 0, sizeof(disp));
    rb->memset(&jpg, 0, sizeof(jpg));

    unsigned char *memory, *memory_max;
    size_t memory_size, img_size, disp_size;

    /* align buffer */
    memory = (unsigned char *)((intptr_t)(buf + 3) & ~3);
    memory_max = (unsigned char *)((intptr_t)(memory + *buf_size) & ~3);
    memory_size = memory_max - memory;
    init_memory_pool(memory_size, memory);
    
    OPEN(filename);
    if (!OPEN(filename))
    {
        return PLUGIN_ERROR;
    }


    if(!iv->running_slideshow)
    {
        rb->lcd_puts(0, 0, rb->strrchr(filename,'/')+1);
        rb->lcd_update();
    }


    if(!iv->running_slideshow)
    {
        rb->lcd_puts(0, 2, "decoding markers");
        rb->lcd_update();
    }
#ifdef DISK_SPINDOWN
    else if(iv->immediate_ata_off)
    {
        /* running slideshow and time is long enough: power down disk */
        rb->storage_sleep();
    }
#endif

    status = JPEGDecode(p_jpg);

    if (status < 0)
    {   /* bad format or minimum components not contained */
        rb->splashf(HZ, "unsupported %d", status);
        return PLUGIN_ERROR;
    }
    
    if(!iv->running_slideshow)
    {
        rb->lcd_putsf(0, 2, "image %dx%d", p_jpg->X, p_jpg->Y);
        rb->lcd_update();
    }

    info->x_size = p_jpg->X;
    info->y_size = p_jpg->Y;
    return PLUGIN_OK;
}

static int get_image(struct image_info *info, int frame, int ds)
{
    //TODO;
    return 0;
}

const struct image_decoder image_decoder = {
    false,
    img_mem,
    load_image,
    get_image,
    draw_image_rect,
};

IMGDEC_HEADER
