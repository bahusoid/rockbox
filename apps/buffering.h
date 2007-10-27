/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2007 Nicolas Pennequin
 *
 * All files in this archive are subject to the GNU General Public License.
 * See the file COPYING in the source tree root for full license agreement.
 *
 * This software is distributed on an "AS IS" basis, WITHOUT WARRANTY OF ANY
 * KIND, either express or implied.
 *
 ****************************************************************************/

#ifndef _BUFFERING_H_
#define _BUFFERING_H_

#include <sys/types.h>
#include <stdbool.h>


enum data_type {
    TYPE_CODEC,
    TYPE_AUDIO,
    TYPE_STREAM,
    TYPE_ID3,
    TYPE_CUESHEET,
    TYPE_IMAGE,
    TYPE_BUFFER,
    TYPE_UNKNOWN,
};


/* Initialise the buffering subsystem */
void buffering_init(void);

/* Reset the buffering system */
bool buffering_reset(char *buf, size_t buflen);


/***************************************************************************
 * MAIN BUFFERING API CALLS
 * ========================
 *
 * bufopen   : Reserve space in the buffer for a given file
 * bufalloc  : Open a new handle from data that needs to be copied from memory
 * bufclose  : Close an open handle
 * bufseek   : Set handle reading index, relatively to the start of the file
 * bufadvance: Move handle reading index, relatively to current position
 * bufread   : Copy data from a handle to a buffer
 * bufgetdata: Obtain a pointer for linear access to a "size" amount of data
 ****************************************************************************/

int bufopen(const char *file, size_t offset, enum data_type type);
int bufalloc(const void *src, size_t size, enum data_type type);
bool bufclose(int handle_id);
int bufseek(int handle_id, size_t newpos);
int bufadvance(int handle_id, off_t offset);
ssize_t bufread(int handle_id, size_t size, void *dest);
ssize_t bufgetdata(int handle_id, size_t size, void **data);


/***************************************************************************
 * SECONDARY FUNCTIONS
 * ===================
 *
 * buf_get_offset: Get a handle offset from a pointer
 * buf_handle_offset: Get the offset of the first buffered byte from the file
 * buf_request_buffer_handle: Request buffering of a handle
 * buf_set_base_handle: Tell the buffering thread which handle is currently read
 * buf_used: Total amount of buffer space used (including allocated space)
 ****************************************************************************/

ssize_t buf_get_offset(int handle_id, void *ptr);
ssize_t buf_handle_offset(int handle_id);
void buf_request_buffer_handle(int handle_id);
void buf_set_base_handle(int handle_id);
size_t buf_used(void);


/***************************************************************************
 * CALLBACK UTILITIES
 * ==================
 *
 * register_buffer_low_callback, unregister_buffer_low_callback:
 *
 * Register/Unregister callback functions that will get executed when the buffer
 * goes below the low watermark. They are executed once, then forgotten.
 *
 * NOTE: The callbacks are called from the buffering thread, so don't make them
 * do too much. Ideally they should just post an event to a queue and return.
 ****************************************************************************/

#define MAX_BUF_CALLBACKS 4
typedef void (*buffer_low_callback)(void);
bool register_buffer_low_callback(buffer_low_callback func);
void unregister_buffer_low_callback(buffer_low_callback func);

/* Settings */
enum {
    BUFFERING_SET_WATERMARK = 1,
    BUFFERING_SET_CHUNKSIZE,
    BUFFERING_SET_PRESEEK,
};
void buf_set_conf(int setting, size_t value);


/* Debugging */
struct buffering_debug {
    int num_handles;
    size_t buffered_data;
    size_t wasted_space;
    size_t data_rem;
    size_t useful_data;
};
void buffering_get_debugdata(struct buffering_debug *dbgdata);

#endif
