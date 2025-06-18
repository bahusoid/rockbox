/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2007 Nicolas Pennequin, Jonathan Gordon
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

#ifndef _CUESHEET_H_
#define _CUESHEET_H_

#include <stdbool.h>
#include "screens.h"
#include "file.h"
#include "metadata.h"

#define MAX_NAME 80    /* Max length of information strings */
#define MAX_TRACKS 256  /* Max number of tracks in a cuesheet */

struct cue_track_info {
    int title_idx;
    int performer_idx;
    int songwriter_idx;
    int file_idx;
    unsigned long offset; /* ms from start of track */
};

#define MAX_LIST  (64000/sizeof(struct cue_track_info))

struct cuesheet {
    char path[MAX_PATH];
    char file[MAX_PATH];
    char title[MAX_NAME*3+1];
    char performer[MAX_NAME*3+1];
    char songwriter[MAX_NAME*3+1];
    bool multi_file;

    int track_count;

    union
    {
        struct cue_track_info tracks[MAX_LIST];
        char buffer[sizeof(struct cue_track_info) * MAX_LIST];
    };
    int curr_track_idx;
};

static FORCE_INLINE struct cue_track_info* get_cue_track(struct cuesheet *cue, int index)
{
    return &cue->tracks[MAX_LIST - index - 1];
}

static FORCE_INLINE struct cue_track_info* get_cue_curr_track(struct cuesheet *cue)
{
    return get_cue_track(cue, cue->curr_track_idx);
}

static FORCE_INLINE char* get_cue_track_performer(struct cuesheet *cue, struct cue_track_info* track)
{
    return track->performer_idx ? &cue->buffer[track->performer_idx] : cue->performer;
}

static FORCE_INLINE char* get_cue_track_songwriter(struct cuesheet *cue, struct cue_track_info* track)
{
    return track->songwriter_idx ? &cue->buffer[track->songwriter_idx] : cue->songwriter;
}

static FORCE_INLINE char* get_cue_track_file(struct cuesheet *cue, struct cue_track_info* track)
{
    return track->file_idx ? &cue->buffer[track->file_idx] : cue->file;
}

static FORCE_INLINE char* get_cue_track_title(struct cuesheet *cue, struct cue_track_info* track)
{
    return &cue->buffer[track->title_idx];
}

struct cuesheet_file {
    char path[MAX_PATH];
    int size;
    off_t pos;
    enum character_encoding encoding;
};

/* looks if there is a cuesheet file with a name matching path of "track_id3" */
bool look_for_cuesheet_file(struct mp3entry *track_id3, struct cuesheet_file *cue_file);

/* parse cuesheet_file "cue_file" and store the information in "cue" */
bool parse_cuesheet(struct cuesheet_file *cue_file, struct cuesheet *cue);

/* reads a cuesheet to find the audio track associated to it */
bool get_trackname_from_cuesheet(char *filename, char *buf);

/* display a cuesheet struct */
bool browse_cuesheet(struct cuesheet *cue);

/* display a cuesheet file after parsing and loading it to the plugin buffer */
bool display_cuesheet_content(char* filename);

/* finds the index of the current track played within a cuesheet */
int cue_find_current_track(struct cuesheet *cue, unsigned long curpos);

/* skip to next track in the cuesheet towards "direction" (which is 1 or -1) */
bool curr_cuesheet_skip(struct cuesheet *cue, int direction, unsigned long curr_pos);

/* draw track markers on the progressbar */
void cue_draw_markers(struct screen *screen, struct cuesheet *cue,
                      unsigned long tracklen,
                      int x, int y, int w, int h);

/* check if the subtrack has changed */
bool cuesheet_subtrack_changed(struct mp3entry *id3);

#endif
