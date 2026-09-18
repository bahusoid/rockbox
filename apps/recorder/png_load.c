/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 *
 * PNG album art.
 *
 * Rockbox could only ever decode JPEG album art. An embedded PNG - and a
 * cover.png next to the tracks - was skipped without a word, which on a
 * library that uses them is a player that simply has no artwork.
 *
 * There was no PNG decoder in the core, but there was everything needed to
 * write one: firmware/common/inflate.c is a streaming, callback-driven
 * DEFLATE, already built in. What was missing is the thin part - the chunk
 * walk, the scanline filters, and the scaling.
 *
 * Two decisions worth stating:
 *
 * - **Alpha is dropped, not composited.** The skin engine has no alpha for
 *   album art, and there is nothing sensible to composite against: the
 *   picture is drawn over a backdrop the decoder cannot see. Album art with
 *   a meaningful alpha channel is rare enough that a wrong guess about the
 *   background would be worse than none.
 * - **It does not go through resize_on_load().** That scaler *pulls* rows
 *   and inflate *pushes* them, and bridging the two needs either a
 *   coroutine or the whole picture in memory - 12 MB for a 2000x2000
 *   cover. A box filter that consumes rows as they arrive needs one source
 *   row and one destination row of accumulators, and is what a downscale of
 *   a photograph wants anyway.
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

#include <stdlib.h>
#include <stdint.h>
#include <string.h>
#include "file.h"
#include "system.h"
#include "lcd.h"
#include "bmp.h"
#include "resize.h"
#include "inflate.h"
#include "png_load.h"

#define PNG_MAX_DIM 4096

enum png_colour {
    PNG_GREY       = 0,
    PNG_RGB        = 2,
    PNG_PALETTE    = 3,
    PNG_GREY_ALPHA = 4,
    PNG_RGBA       = 6,
};

struct png_ctx
{
    /* ---- the file ---------------------------------------------------- */
    int      fd;
    off_t    limit;         /* one past the last byte we may read         */
    uint32_t chunk_left;    /* bytes left in the IDAT being read          */
    bool     ended;         /* IEND seen, or no more IDATs                */

    /* ---- the image --------------------------------------------------- */
    int  width, height;
    int  depth, colour;
    int  channels;          /* samples per pixel in the stream            */
    int  bpp;               /* bytes per pixel, for the filter's "left"   */
    int  rowbytes;
    unsigned char palette[256 * 3];
    int  palette_n;

    /* ---- the scanline in progress ------------------------------------ */
    unsigned char *cur, *prev;
    int  fill;              /* bytes of (filter + row) assembled so far   */
    int  src_y;

    /* ---- the box filter ---------------------------------------------- */
    struct bitmap *bm;
    uint32_t *acc;          /* 3 per destination column                   */
    uint32_t *cnt;          /* 1 per destination column                   */
    int  dst_rows_done;
    bool failed;
};

/* ------------------------------------------------------------------ file */

static bool rd_exact(struct png_ctx *p, void *buf, size_t n)
{
    if (p->limit > 0 && lseek(p->fd, 0, SEEK_CUR) + (off_t)n > p->limit)
        return false;
    return read(p->fd, buf, n) == (ssize_t)n;
}

static bool rd_u32(struct png_ctx *p, uint32_t *out)
{
    unsigned char b[4];

    if (!rd_exact(p, b, 4))
        return false;

    *out = ((uint32_t)b[0] << 24) | ((uint32_t)b[1] << 16) |
           ((uint32_t)b[2] << 8)  |  (uint32_t)b[3];
    return true;
}

/* Walk to the next chunk of interest. Returns false at the end of the
 * stream, which for our purposes is IEND or anything unreadable. */
static bool next_chunk(struct png_ctx *p, uint32_t *len, char type[5])
{
    unsigned char t[4];

    if (!rd_u32(p, len) || !rd_exact(p, t, 4))
        return false;

    memcpy(type, t, 4);
    type[4] = '\0';
    return true;
}

static bool skip_chunk(struct png_ctx *p, uint32_t len)
{
    /* +4 for the CRC, which we do not check: a corrupt cover shows as a
     * corrupt cover, and the alternative is buffering the whole chunk. */
    if (lseek(p->fd, len + 4, SEEK_CUR) < 0)
        return false;
    return true;
}

/* The reader inflate pulls from: the concatenation of every IDAT. */
static uint32_t png_reader(void *block, uint32_t block_size, void *ctx)
{
    struct png_ctx *p = ctx;
    uint32_t got = 0;

    while (got < block_size)
    {
        ssize_t n;

        if (p->chunk_left == 0)
        {
            uint32_t len;
            char type[5];

            if (p->ended)
                break;

            /* step over the finished chunk's CRC */
            if (lseek(p->fd, 4, SEEK_CUR) < 0)
                break;

            for (;;)
            {
                if (!next_chunk(p, &len, type))
                {
                    p->ended = true;
                    break;
                }
                if (!memcmp(type, "IDAT", 4))
                    break;
                if (!memcmp(type, "IEND", 4))
                {
                    p->ended = true;
                    break;
                }
                if (!skip_chunk(p, len))
                {
                    p->ended = true;
                    break;
                }
            }

            if (p->ended)
                break;

            p->chunk_left = len;
            if (len == 0)
                continue;
        }

        n = block_size - got;
        if ((uint32_t)n > p->chunk_left)
            n = p->chunk_left;

        n = read(p->fd, (unsigned char *)block + got, n);
        if (n <= 0)
        {
            p->ended = true;
            break;
        }

        p->chunk_left -= n;
        got += n;
    }

    return got;
}

/* ------------------------------------------------------------- unfilter */

static void unfilter(struct png_ctx *p, int filter)
{
    unsigned char *cur = p->cur, *prev = p->prev;
    int bpp = p->bpp, n = p->rowbytes, i;

    switch (filter)
    {
        case 0:
            break;

        case 1:                                     /* Sub */
            for (i = bpp; i < n; i++)
                cur[i] = (unsigned char)(cur[i] + cur[i - bpp]);
            break;

        case 2:                                     /* Up */
            for (i = 0; i < n; i++)
                cur[i] = (unsigned char)(cur[i] + prev[i]);
            break;

        case 3:                                     /* Average */
            for (i = 0; i < bpp && i < n; i++)
                cur[i] = (unsigned char)(cur[i] + (prev[i] >> 1));
            for (; i < n; i++)
                cur[i] = (unsigned char)(cur[i] +
                                         ((cur[i - bpp] + prev[i]) >> 1));
            break;

        case 4:                                     /* Paeth */
            for (i = 0; i < bpp && i < n; i++)
                cur[i] = (unsigned char)(cur[i] + prev[i]);
            for (; i < n; i++)
            {
                int a = cur[i - bpp], b = prev[i], c = prev[i - bpp];
                int q = a + b - c;
                int pa = q > a ? q - a : a - q;
                int pb = q > b ? q - b : b - q;
                int pc = q > c ? q - c : c - q;
                int pred = (pa <= pb && pa <= pc) ? a : (pb <= pc ? b : c);

                cur[i] = (unsigned char)(cur[i] + pred);
            }
            break;

        default:
            p->failed = true;
            break;
    }
}

/* ------------------------------------------------------------- to pixels */

/* One source pixel as 8-bit r,g,b. Alpha, where there is any, is dropped -
 * see the note at the top. */
static void pixel_at(struct png_ctx *p, int x, unsigned *r, unsigned *g,
                     unsigned *b)
{
    const unsigned char *row = p->cur;

    if (p->depth == 16)
    {
        int step = p->channels * 2;
        const unsigned char *s = row + (size_t)x * step;

        if (p->colour == PNG_RGB || p->colour == PNG_RGBA)
        {
            *r = s[0]; *g = s[2]; *b = s[4];
        }
        else
        {
            *r = *g = *b = s[0];
        }
        return;
    }

    if (p->depth == 8)
    {
        const unsigned char *s = row + (size_t)x * p->channels;

        switch (p->colour)
        {
            case PNG_RGB:
            case PNG_RGBA:
                *r = s[0]; *g = s[1]; *b = s[2];
                return;
            case PNG_PALETTE:
            {
                int i = s[0] < p->palette_n ? s[0] : 0;
                *r = p->palette[i * 3];
                *g = p->palette[i * 3 + 1];
                *b = p->palette[i * 3 + 2];
                return;
            }
            default:
                *r = *g = *b = s[0];
                return;
        }
    }

    /* depth 1, 2 or 4: grey or palette, packed big-endian within the byte */
    {
        int bits = p->depth;
        int per_byte = 8 / bits;
        int byte = x / per_byte;
        int shift = 8 - bits * (x % per_byte) - bits;
        unsigned v = (row[byte] >> shift) & ((1u << bits) - 1);

        if (p->colour == PNG_PALETTE)
        {
            int i = (int)v < p->palette_n ? (int)v : 0;
            *r = p->palette[i * 3];
            *g = p->palette[i * 3 + 1];
            *b = p->palette[i * 3 + 2];
            return;
        }

        /* grey: 1 bit is 0 or 255, 2 bits 0/85/170/255, and so on */
        v = v * 255u / ((1u << bits) - 1);
        *r = *g = *b = v;
    }
}

/* ------------------------------------------------------------ box filter */

static void emit_row(struct png_ctx *p, int dy)
{
    struct bitmap *bm = p->bm;
    fb_data *dst = (fb_data *)bm->data +
                   (size_t)dy * STRIDE_MAIN(bm->width, bm->height);
    int dx;

    for (dx = 0; dx < bm->width; dx++)
    {
        uint32_t n = p->cnt[dx];
        unsigned r, g, b;

        if (n == 0)
        {
            /* A destination column no source pixel reached, which happens
             * only on an upscale of a one-pixel-wide picture. Repeat the
             * one to its left rather than leaving a hole. */
            dst[dx] = dx > 0 ? dst[dx - 1] : 0;
            continue;
        }

        r = p->acc[dx * 3]     / n;
        g = p->acc[dx * 3 + 1] / n;
        b = p->acc[dx * 3 + 2] / n;
        dst[dx] = LCD_RGBPACK(r, g, b);
    }
}

static void accumulate_row(struct png_ctx *p)
{
    struct bitmap *bm = p->bm;
    int sw = p->width, dw = bm->width;
    int sh = p->height, dh = bm->height;
    int y = p->src_y;
    int dy0 = (int)((int64_t)y * dh / sh);
    int dy1 = (int)((int64_t)(y + 1) * dh / sh);
    int x, dy;

    for (x = 0; x < sw; x++)
    {
        unsigned r, g, b;
        int dx0 = (int)((int64_t)x * dw / sw);
        int dx1 = (int)((int64_t)(x + 1) * dw / sw);
        int dx;

        pixel_at(p, x, &r, &g, &b);

        if (dx1 <= dx0)
            dx1 = dx0 + 1;
        if (dx1 > dw)
            dx1 = dw;

        for (dx = dx0; dx < dx1; dx++)
        {
            p->acc[dx * 3]     += r;
            p->acc[dx * 3 + 1] += g;
            p->acc[dx * 3 + 2] += b;
            p->cnt[dx]++;
        }
    }

    /* This source row is the last one feeding destination row dy0 exactly
     * when the next source row starts a new one. On an upscale dy1 runs
     * ahead by more than one and the same averaged row is written to each
     * of them, which is a nearest-neighbour stretch - right for the rare
     * case of a cover smaller than its box. */
    if (dy1 <= dy0)
        return;

    if (dy1 > dh)
        dy1 = dh;

    for (dy = dy0; dy < dy1; dy++)
    {
        if (dy < 0 || dy >= dh)
            continue;
        emit_row(p, dy);
        p->dst_rows_done++;
    }

    memset(p->acc, 0, sizeof(uint32_t) * 3 * dw);
    memset(p->cnt, 0, sizeof(uint32_t) * dw);
}

/* The writer inflate pushes to: raw scanlines, filter byte and all. */
static uint32_t png_writer(const void *block, uint32_t block_size, void *ctx)
{
    struct png_ctx *p = ctx;
    const unsigned char *in = block;
    uint32_t left = block_size;
    int stride = p->rowbytes + 1;

    if (p->failed)
        return 0;

    while (left > 0)
    {
        int want = stride - p->fill;
        int take = (int)left < want ? (int)left : want;

        if (p->src_y >= p->height)
            return block_size;      /* trailing data: done, swallow it */

        /* cur[-1] is the filter byte and cur[0..] the row, so one memcpy
         * takes the scanline however it happens to be split across the
         * blocks inflate hands us. */
        memcpy(p->cur - 1 + p->fill, in, take);
        p->fill += take;
        in      += take;
        left    -= take;

        if (p->fill < stride)
            continue;

        unfilter(p, p->cur[-1]);
        if (p->failed)
            return 0;

        accumulate_row(p);

        {
            unsigned char *t = p->prev;
            p->prev = p->cur;
            p->cur  = t;
        }
        p->fill = 0;
        p->src_y++;
    }

    return block_size;
}

/* ------------------------------------------------------------------ entry */

static int png_decode(int fd, off_t limit, struct bitmap *bm, int maxsize,
                      int format)
{
    struct png_ctx p;
    unsigned char sig[8];
    struct dim src_dim, dst_dim;
    struct inflate *it;
    unsigned char *tail;
    int bm_size, avail, rowalloc, bits;
    uint32_t len;
    char type[5];
    bool got_ihdr = false;

    memset(&p, 0, sizeof(p));
    p.fd = fd;
    p.limit = limit;

    if (!rd_exact(&p, sig, 8) ||
        memcmp(sig, "\x89PNG\r\n\x1a\n", 8))
        return -1;

    /* ---- the header chunks, up to the first IDAT --------------------- */
    for (;;)
    {
        if (!next_chunk(&p, &len, type))
            return -1;

        if (!memcmp(type, "IHDR", 4))
        {
            unsigned char h[13];

            if (len < 13 || !rd_exact(&p, h, 13))
                return -1;

            p.width  = (h[0] << 24) | (h[1] << 16) | (h[2] << 8) | h[3];
            p.height = (h[4] << 24) | (h[5] << 16) | (h[6] << 8) | h[7];
            p.depth  = h[8];
            p.colour = h[9];
            /* h[10] compression, h[11] filter, h[12] interlace */
            if (h[12] != 0)
                return -1;      /* Adam7: rare for cover art, not handled */
            if (h[10] != 0 || h[11] != 0)
                return -1;
            if (p.width <= 0 || p.height <= 0 ||
                p.width > PNG_MAX_DIM || p.height > PNG_MAX_DIM)
                return -1;

            if (len > 13 && lseek(fd, len - 13, SEEK_CUR) < 0)
                return -1;
            if (lseek(fd, 4, SEEK_CUR) < 0)     /* CRC */
                return -1;
            got_ihdr = true;
            continue;
        }

        if (!got_ihdr)
            return -1;

        if (!memcmp(type, "PLTE", 4))
        {
            uint32_t n = len > sizeof(p.palette) ? sizeof(p.palette) : len;

            if (!rd_exact(&p, p.palette, n))
                return -1;
            p.palette_n = n / 3;
            if (len > n && lseek(fd, len - n, SEEK_CUR) < 0)
                return -1;
            if (lseek(fd, 4, SEEK_CUR) < 0)
                return -1;
            continue;
        }

        if (!memcmp(type, "IDAT", 4))
        {
            p.chunk_left = len;
            break;
        }

        if (!memcmp(type, "IEND", 4))
            return -1;

        if (!skip_chunk(&p, len))
            return -1;
    }

    switch (p.colour)
    {
        case PNG_GREY:       p.channels = 1; break;
        case PNG_RGB:        p.channels = 3; break;
        case PNG_PALETTE:    p.channels = 1; break;
        case PNG_GREY_ALPHA: p.channels = 2; break;
        case PNG_RGBA:       p.channels = 4; break;
        default:             return -1;
    }

    if (p.depth != 1 && p.depth != 2 && p.depth != 4 &&
        p.depth != 8 && p.depth != 16)
        return -1;
    if (p.depth != 8 && p.depth != 16 &&
        p.colour != PNG_GREY && p.colour != PNG_PALETTE)
        return -1;
    if (p.colour == PNG_PALETTE && p.palette_n == 0)
        return -1;

    bits = p.depth * p.channels;
    p.rowbytes = ((size_t)p.width * bits + 7) / 8;
    p.bpp = bits >= 8 ? bits / 8 : 1;

    /* ---- how big the result is --------------------------------------- */
    src_dim.width  = p.width;
    src_dim.height = p.height;

    if (format & FORMAT_RESIZE)
    {
        dst_dim.width  = bm->width;
        dst_dim.height = bm->height;
        if (format & FORMAT_KEEP_ASPECT)
            recalc_dimension(&dst_dim, &src_dim);
        bm->width  = dst_dim.width;
        bm->height = dst_dim.height;
    }
    else
    {
        bm->width  = p.width;
        bm->height = p.height;
    }

    if (bm->width <= 0 || bm->height <= 0)
        return -1;

    bm_size = BM_SIZE(bm->width, bm->height, FORMAT_NATIVE, false);
    if (bm_size > maxsize)
        return -1;

    /* ---- the workspace, out of the tail of the same buffer ----------- */
    tail  = (unsigned char *)bm->data + bm_size;
    avail = maxsize - bm_size;

    {
        uintptr_t a = (uintptr_t)tail;
        uintptr_t pad = (-a) & (inflate_align - 1);

        if ((int)pad + (int)inflate_size > avail)
            return -1;
        it = (struct inflate *)(tail + pad);
        tail  += pad + inflate_size;
        avail -= (int)pad + (int)inflate_size;
    }

    {
        uintptr_t a = (uintptr_t)tail;
        uintptr_t pad = (-a) & (sizeof(uint32_t) - 1);
        size_t need = (size_t)bm->width * 4 * sizeof(uint32_t);

        if ((int)pad + (int)need > avail)
            return -1;
        p.acc = (uint32_t *)(tail + pad);
        p.cnt = p.acc + (size_t)bm->width * 3;
        tail  += pad + need;
        avail -= (int)pad + (int)need;
    }

    /* Two scanlines, each with one byte in front of it for the filter, so
     * the two can be swapped without copying. */
    rowalloc = p.rowbytes + 1;
    if (2 * rowalloc > avail)
        return -1;

    memset(tail, 0, 2 * rowalloc);
    p.cur  = tail + 1;
    p.prev = tail + rowalloc + 1;

    memset(p.acc, 0, sizeof(uint32_t) * 3 * bm->width);
    memset(p.cnt, 0, sizeof(uint32_t) * bm->width);

    p.bm = bm;

    /* ---- go ---------------------------------------------------------- */
    if (inflate(it, INFLATE_ZLIB, png_reader, &p, png_writer, &p) < 0 &&
        p.dst_rows_done < bm->height)
        return -1;

    if (p.failed)
        return -1;

    /* A truncated picture is still a picture: fill what never arrived with
     * the last row that did rather than returning nothing. */
    while (p.dst_rows_done < bm->height)
    {
        fb_data *dst = (fb_data *)bm->data +
                       (size_t)p.dst_rows_done *
                       STRIDE_MAIN(bm->width, bm->height);

        if (p.dst_rows_done == 0)
        {
            memset(dst, 0, sizeof(fb_data) * bm->width);
        }
        else
        {
            memcpy(dst, dst - STRIDE_MAIN(bm->width, bm->height),
                   sizeof(fb_data) * bm->width);
        }
        p.dst_rows_done++;
    }

    return bm_size;
}

int read_png_fd(int fd, struct bitmap *bm, int maxsize, int format)
{
    return png_decode(fd, 0, bm, maxsize, format);
}

int clip_png_fd(int fd, int pos, int size, struct bitmap *bm, int maxsize,
                int format)
{
    if (lseek(fd, pos, SEEK_SET) < 0)
        return -1;

    return png_decode(fd, size > 0 ? (off_t)pos + size : 0, bm, maxsize,
                      format);
}
