/***************************************************************************
 * PNG album art: see png_load.c.
 ****************************************************************************/
#ifndef _PNG_LOAD_H_
#define _PNG_LOAD_H_

#include <stdbool.h>
#include "bmp.h"

/* Decode a PNG into bm, scaled to bm->width x bm->height when
 * FORMAT_RESIZE is set (and to the source's aspect ratio inside that box
 * when FORMAT_KEEP_ASPECT is). Returns the number of bytes of bm->data
 * used, or a negative number.
 *
 * maxsize is the whole of the buffer at bm->data: the decoder takes its
 * workspace - the inflate state, the scaling accumulators and two source
 * scanlines - out of whatever is left past the finished picture. That is
 * at most ~105 KiB for a 4096 px source, which the JPEG reservation in
 * buffering.c already covers. */
int read_png_fd(int fd, int flags,
                 struct bitmap *bm,
                 int maxsize,
                 int format,
                 const struct custom_format *cformat,
                 bool (*cb_progress)(int current, int total));

/* The same, for art embedded in a tag: pos and size bound it. */
int clip_png_fd(int fd, int pos, int size, struct bitmap *bm, int maxsize,
                int format);

#endif /* _PNG_LOAD_H_ */
