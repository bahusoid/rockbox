/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2025 Roman Artiukhin
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

#include "plugin.h"

/* Global buffer management */
static char *g_buffer = NULL;
static size_t g_buffer_size = 0;
static size_t g_buffer_used = 0;

/* Initialize buffer on first use */
static bool init_buffer(void) {
    if (g_buffer == NULL) {
        g_buffer = rb->plugin_get_buffer(&g_buffer_size);
        g_buffer_used = 0;
    }
    return g_buffer != NULL;
}


/* Allocate from global buffer */
static void* buffer_alloc(size_t size) {
    if (!init_buffer()) {
        return NULL;
    }
    
    /* Align to 8-byte boundary for better performance */
    size = (size + 7) & ~7;
    
    if (g_buffer_used + size > g_buffer_size) {
        return NULL;
    }
    
    void *ptr = g_buffer + g_buffer_used;
    g_buffer_used += size;
    return ptr;
}

/* Reset buffer usage */
static void buffer_reset(void) {
    g_buffer_used = 0;
}

/* MP4 atom identifiers */
#define MP4_chpl 0x6368706C /* 'chpl' - Chapter List (Nero) */
#define MP4_CHAP 0x43484150 /* 'CHAP' - Apple chapter atom */
#define MP4_moov 0x6D6F6F76 /* 'moov' - Movie atom */
#define MP4_trak 0x7472616B /* 'trak' - Track atom */
#define MP4_tref 0x74726566 /* 'tref' - Track reference atom */
#define MP4_chap_ref 0x63686170 /* 'chap' - Chapter reference */
#define MP4_mdia 0x6D646961 /* 'mdia' - Media atom */
#define MP4_minf 0x6D696E66 /* 'minf' - Media information atom */
#define MP4_stbl 0x7374626C /* 'stbl' - Sample table atom */
#define MP4_stts 0x73747473 /* 'stts' - Time-to-sample atom */
#define MP4_stsc 0x73747363 /* 'stsc' - Sample-to-chunk atom */
#define MP4_stco 0x7374636F /* 'stco' - Chunk offset atom */
#define MP4_co64 0x636F3634 /* 'co64' - 64-bit chunk offset atom */
#define MP4_tkhd 0x746B6864 /* 'tkhd' - Track header atom */
#define MP4_hdlr 0x68646C72 /* 'hdlr' - Handler reference atom */
#define MP4_TEXT 0x74657874 /* 'text' - Text handler */
#define MP4_tx3g 0x74783367 /* 'tx3g' - 3GPP timed text */
#define MP4_stsz 0x7374737A /* 'stsz' - Sample size atom */
#define MP4_mdhd 0x6D646864 /* 'mdhd' - Media header atom */

/* Structure to hold chapter information */
struct chapter_info {
    uint64_t timestamp;     /* Chapter start time in milliseconds */
    char title[256];        /* Chapter title */
};

/* MP4 utility functions */
static uint32_t read_uint32be(int fd) {
    uint8_t buf[4];
    if (rb->read(fd, buf, 4) != 4) return 0;
    return (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
}

static uint64_t read_uint64be(int fd) {
    uint8_t buf[8];
    if (rb->read(fd, buf, 8) != 8) return 0;
    return ((uint64_t)buf[0] << 56) | ((uint64_t)buf[1] << 48) | 
           ((uint64_t)buf[2] << 40) | ((uint64_t)buf[3] << 32) |
           ((uint64_t)buf[4] << 24) | ((uint64_t)buf[5] << 16) | 
           ((uint64_t)buf[6] << 8) | (uint64_t)buf[7];
}

static uint8_t read_uint8(int fd) {
    uint8_t buf;
    if (rb->read(fd, &buf, 1) != 1) return 0;
    return buf;
}

/* Convert timestamp (milliseconds) to CUE format MM:SS:FF */
static void timestamp_to_cue_time(uint64_t timestamp_ms, char *time_str, size_t size) {
    uint32_t minutes = timestamp_ms / 60000;
    uint32_t seconds = (timestamp_ms % 60000) / 1000;
    uint32_t frames = ((timestamp_ms % 1000) * 75) / 1000; /* 75 frames per second for CUE */
    
    rb->snprintf(time_str, size, "%02u:%02u:%02u", minutes, seconds, frames);
}

/* Recursively search for atoms in MP4 structure */
static int search_for_atom(int fd, off_t start_pos, off_t end_pos, uint32_t target_atom,
                          off_t *found_pos, off_t *found_size) {
    off_t current_pos = start_pos;
    
    while (current_pos < end_pos) {
        if (rb->lseek(fd, current_pos, SEEK_SET) < 0) break;
        
        uint32_t atom_size = read_uint32be(fd);
        uint32_t atom_type = read_uint32be(fd);
        
        if (atom_size < 8) break;
        
        if (atom_type == target_atom) {
            *found_pos = current_pos;
            *found_size = atom_size;
            return 1;
        }
        
        /* Recursively search container atoms */
        if (atom_type == MP4_moov || atom_type == MP4_trak || 
            atom_type == MP4_mdia || atom_type == MP4_minf || atom_type == MP4_stbl) {
            if (search_for_atom(fd, current_pos + 8, current_pos + atom_size,
                               target_atom, found_pos, found_size)) {
                return 1;
            }
        }
        
        current_pos += atom_size;
    }
    
    return 0;
}

/* Parse Apple chapter track - look for text track with chapter data */
static int parse_apple_chapter_track(int fd, off_t track_start, off_t track_size, 
                                   struct chapter_info *chapters) {
    off_t track_end = track_start + track_size;
    int chapter_count = 0;
    uint32_t track_timescale = 1000; /* Default timescale */
    
    /* Look for track header to check if this is a text track */
    off_t tkhd_pos, tkhd_size;
    if (!search_for_atom(fd, track_start, track_end, MP4_tkhd, &tkhd_pos, &tkhd_size)) {
        DEBUGF("No tkhd found in track");
        return 0;
    }
    
    /* Look for media atom */
    off_t mdia_pos, mdia_size;
    if (!search_for_atom(fd, track_start, track_end, MP4_mdia, &mdia_pos, &mdia_size)) {
        DEBUGF("No mdia found in track");
        return 0;
    }
    
    /* Look for media header to get correct timescale */
    off_t mdhd_pos, mdhd_size;
    if (search_for_atom(fd, mdia_pos, mdia_pos + mdia_size, MP4_mdhd, &mdhd_pos, &mdhd_size)) {
        rb->lseek(fd, mdhd_pos + 8, SEEK_SET); /* Skip atom header */
        uint8_t version = read_uint8(fd);
        rb->lseek(fd, 3, SEEK_CUR); /* Skip flags */
        
        if (version == 1) {
            /* 64-bit version */
            rb->lseek(fd, 16, SEEK_CUR); /* Skip creation_time and modification_time */
            track_timescale = read_uint32be(fd);
        } else {
            /* 32-bit version */
            rb->lseek(fd, 8, SEEK_CUR); /* Skip creation_time and modification_time */
            track_timescale = read_uint32be(fd);
        }
        
        DEBUGF("Found track timescale: %u", track_timescale);
    } else {
        DEBUGF("No mdhd found, using default timescale");
    }
    
    /* Look for media handler to check track type */
    off_t hdlr_pos, hdlr_size;
    if (!search_for_atom(fd, mdia_pos, mdia_pos + mdia_size, MP4_hdlr, &hdlr_pos, &hdlr_size)) {
        DEBUGF("No hdlr found in track");
        return 0;
    }
    
    /* Check if this is a text track */
    rb->lseek(fd, hdlr_pos + 8 + 8, SEEK_SET); /* Skip atom header + version/flags */
    uint32_t handler_type = read_uint32be(fd);
    
    DEBUGF("Track handler type: %c%c%c%c", 
           (char)(handler_type >> 24), (char)(handler_type >> 16), 
           (char)(handler_type >> 8), (char)handler_type);
    
    if (handler_type != MP4_TEXT && handler_type != MP4_tx3g) {
        DEBUGF("Not a text track, skipping");
        return 0; /* Not a text track */
    }
    
    DEBUGF("Found text track!");
    
    /* Look for media information atom first */
    off_t minf_pos, minf_size;
    if (!search_for_atom(fd, mdia_pos + 8, mdia_pos + mdia_size, MP4_minf, &minf_pos, &minf_size)) {
        DEBUGF("No minf found in text track");
        return 0;
    }
    
    DEBUGF("Found minf in text track\n");
    
    /* Look for sample table */
    off_t stbl_pos, stbl_size;
    if (!search_for_atom(fd, minf_pos + 8, minf_pos + minf_size, MP4_stbl, &stbl_pos, &stbl_size)) {
        DEBUGF("No stbl found in text track");
        return 0;
    }
    
    DEBUGF("Found sample table in text track at pos %ld, size %ld\n", (long)stbl_pos, (long)stbl_size);
    
    /* Get time-to-sample information */
    off_t stts_pos, stts_size;
    if (!search_for_atom(fd, stbl_pos + 8, stbl_pos + stbl_size, MP4_stts, &stts_pos, &stts_size)) {
        DEBUGF("No stts found");
        return 0;
    }
    
    /* Get sample size information to know how many samples we have */
    off_t stsz_pos, stsz_size;
    if (!search_for_atom(fd, stbl_pos + 8, stbl_pos + stbl_size, MP4_stsz, &stsz_pos, &stsz_size)) {
        DEBUGF("No stsz found");
        return 0;
    }
    
    /* Get chunk offset table to find sample data */
    off_t stco_pos, stco_size;
    bool has_stco = search_for_atom(fd, stbl_pos + 8, stbl_pos + stbl_size, MP4_stco, &stco_pos, &stco_size);
    off_t co64_pos, co64_size;
    bool has_co64 = false;
    if (!has_stco) {
        has_co64 = search_for_atom(fd, stbl_pos + 8, stbl_pos + stbl_size, MP4_co64, &co64_pos, &co64_size);
    }
    
    if (!has_stco && !has_co64) {
        DEBUGF("No chunk offset table found\n");
        return 0;
    }
    
    DEBUGF("Found all required sample tables\n");
    
    /* Read sample size table to get number of samples */
    rb->lseek(fd, stsz_pos + 8 + 4, SEEK_SET); /* Skip atom header + version/flags */
    uint32_t default_sample_size = read_uint32be(fd);
    uint32_t sample_count = read_uint32be(fd);
    
    DEBUGF("Sample count: %u, default size: %u, timescale: %u\n", sample_count, default_sample_size, track_timescale);
    
    if (sample_count == 0 || sample_count > 1000) {
        return 0; /* Sanity check */
    }

    // Allocate memory for chapters
    buffer_alloc(sizeof(struct chapter_info) * sample_count);
    
    /* Calculate memory requirements */
    size_t sample_sizes_bytes = (default_sample_size == 0) ? sample_count * sizeof(uint32_t) : 0;
    size_t chunk_offsets_bytes = 1000 * sizeof(uint64_t); /* Maximum chunks */
    
    /* Allocate memory from global buffer */
    uint32_t *sample_sizes = NULL;
    if (default_sample_size == 0) {
        sample_sizes = (uint32_t*)buffer_alloc(sample_sizes_bytes);
        if (sample_sizes == NULL) {
            DEBUGF("Not enough memory for sample sizes\n");
            return 0;
        }
        
        for (uint32_t i = 0; i < sample_count; i++) {
            sample_sizes[i] = read_uint32be(fd);
        }
    }
    
    /* Allocate memory for chunk offsets */
    uint64_t *chunk_offsets = (uint64_t*)buffer_alloc(chunk_offsets_bytes);
    if (chunk_offsets == NULL) {
        DEBUGF("Not enough memory for chunk offsets\n");
        return 0;
    }
    
    /* Read chunk offset table */
    uint32_t chunk_count = 0;
    
    if (has_stco) {
        rb->lseek(fd, stco_pos + 8 + 4, SEEK_SET); /* Skip atom header + version/flags */
        chunk_count = read_uint32be(fd);
        
        if (chunk_count > 0 && chunk_count <= 1000) {
            for (uint32_t i = 0; i < chunk_count; i++) {
                chunk_offsets[i] = read_uint32be(fd);
            }
        }
    } else if (has_co64) {
        rb->lseek(fd, co64_pos + 8 + 4, SEEK_SET); /* Skip atom header + version/flags */
        chunk_count = read_uint32be(fd);
        
        if (chunk_count > 0 && chunk_count <= 1000) {
            for (uint32_t i = 0; i < chunk_count; i++) {
                chunk_offsets[i] = read_uint64be(fd);
            }
        }
    }
    
    DEBUGF("Found %u chunks\n", chunk_count);
    
    /* Parse time-to-sample table to build timeline */
    rb->lseek(fd, stts_pos + 8 + 4, SEEK_SET); /* Skip atom header + version/flags */
    uint32_t stts_entry_count = read_uint32be(fd);

    DEBUGF("STTS entries: %u\n", stts_entry_count);

    if (stts_entry_count == 0 || stts_entry_count > 1000) {
        return 0; /* Sanity check */
    }
    
    /* Build timestamp array from time-to-sample data */
    uint64_t current_time = 0;
    uint32_t sample_index = 0;
    uint32_t current_chunk = 0;
    
    /* Helper buffer for reading sample data */
    char sample_buffer[512];
    
    for (uint32_t i = 0; i < stts_entry_count; i++) {
        rb->lseek(fd, stts_pos + 8 + 8 + (i * 8), SEEK_SET);
        uint32_t samples_in_entry = read_uint32be(fd);
        uint32_t sample_duration = read_uint32be(fd);
        
        DEBUGF("STTS entry %u: %u samples, duration %u\n", i, samples_in_entry, sample_duration);
        
        /* Each sample represents a chapter */
        for (uint32_t j = 0; j < samples_in_entry && sample_index < sample_count; j++) {
            /* Convert from track timescale to milliseconds */
            chapters[chapter_count].timestamp = (current_time * 1000) / track_timescale;
            chapters[chapter_count].title[0] = 0;
            DEBUGF("Chapter %d: time=%lu units, timestamp=%lu ms (timescale=%u)\n", 
                   chapter_count, (unsigned long)current_time, (unsigned long)chapters[chapter_count].timestamp, track_timescale);
            
            /* Try to read the actual chapter title from sample data */
            if (chunk_offsets != NULL && current_chunk < chunk_count) {
                /* Get sample size */
                uint32_t sample_size;
                if (default_sample_size > 0) {
                    sample_size = default_sample_size;
                } else if (sample_sizes != NULL) {
                    sample_size = sample_sizes[sample_index];
                } else {
                    sample_size = 0;
                }
                
                if (sample_size > 0 && sample_size < sizeof(sample_buffer)) {
                    /* Read sample data */
                    rb->lseek(fd, chunk_offsets[current_chunk], SEEK_SET);
                    int bytes_read = rb->read(fd, sample_buffer, sample_size);
                    
                    if (bytes_read > 0) {
                        /* Parse text sample - format varies but often starts with length */
                        const char *title_text = NULL;
                        
                        /* Try different text sample formats */
                        if (sample_size >= 2) {
                            /* Check for length-prefixed string (common format) */
                            uint16_t text_len = (sample_buffer[0] << 8) | sample_buffer[1];
                            if (text_len > 0 && text_len < sample_size - 2 && text_len < 200) {
                                title_text = &sample_buffer[2];
                                /* Ensure null termination */
                                size_t copy_len = text_len;
                                if (copy_len >= sizeof(chapters[chapter_count].title)) {
                                    copy_len = sizeof(chapters[chapter_count].title) - 1;
                                }
                                rb->strlcpy(chapters[chapter_count].title, title_text, copy_len + 1);
                            }
                        }
                        
                        /* If that didn't work, try looking for plain text */
                        if (title_text == NULL) {
                            /* Look for readable text in the sample */
                            for (uint32_t k = 0; k < sample_size - 1; k++) {
                                if (sample_buffer[k] >= 32 && sample_buffer[k] <= 126) {
                                    /* Found start of readable text */
                                    uint32_t text_end = k;
                                    while (text_end < sample_size && 
                                           sample_buffer[text_end] >= 32 && 
                                           sample_buffer[text_end] <= 126) {
                                        text_end++;
                                    }
                                    
                                    if (text_end - k > 3) { /* At least 4 characters */
                                        size_t copy_len = text_end - k;
                                        if (copy_len >= sizeof(chapters[chapter_count].title)) {
                                            copy_len = sizeof(chapters[chapter_count].title) - 1;
                                        }
                                        rb->strlcpy(chapters[chapter_count].title, &sample_buffer[k], copy_len + 1);
                                        title_text = chapters[chapter_count].title;
                                        break;
                                    }
                                }
                            }
                        }
                        
                        DEBUGF("Sample %u: size=%u, title='%s'\n", sample_index, sample_size, 
                               title_text ? chapters[chapter_count].title : "failed");
                    }
                }
                
                /* Move to next chunk for next sample (simplified - assumes one sample per chunk) */
                current_chunk++;
            }
            
            /* Fallback to generic title if we couldn't read the text */
            if (chapters[chapter_count].title[0] == 0) {
                rb->snprintf(chapters[chapter_count].title, sizeof(chapters[chapter_count].title), 
                           "Chapter %d", chapter_count + 1);
            }

            chapter_count++;
            sample_index++;
            current_time += sample_duration;
        }
    }
    
    DEBUGF("Extracted %d chapters from text track", chapter_count);
    
    return chapter_count;
}

/* Find and parse MP4 chapters (both Nero and Apple formats) */
static int find_mp4_chapters(const char *mp4_path, struct chapter_info *chapters, int max_chapters) {
    int fd = rb->open(mp4_path, O_RDONLY);
    if (fd < 0) return -1;
    
    int chapter_count = 0;
    uint32_t atom_size, atom_type;
    off_t file_pos = 0;
    
    /* First pass: Look for Nero chpl atoms (simpler format) */
    while (chapter_count == 0) {
        if (rb->lseek(fd, file_pos, SEEK_SET) < 0) break;
        if (rb->read(fd, &atom_size, 4) != 4) break;
        if (rb->read(fd, &atom_type, 4) != 4) break;
        
        /* Convert to host byte order */
        atom_size = (atom_size >> 24) | ((atom_size >> 8) & 0xFF00) | 
                   ((atom_size << 8) & 0xFF0000) | (atom_size << 24);
        atom_type = (atom_type >> 24) | ((atom_type >> 8) & 0xFF00) | 
                   ((atom_type << 8) & 0xFF0000) | (atom_type << 24);
        
        if (atom_size < 8) break;
        
        if (atom_type == MP4_chpl) {
            /* Found Nero chapter list atom */
            rb->lseek(fd, 8, SEEK_CUR); /* Skip version and flags */
            uint8_t num_chapters = read_uint8(fd);
            
            for (int i = 0; i < num_chapters && chapter_count < max_chapters; i++) {
                uint64_t timestamp_100ns = read_uint64be(fd);
                chapters[chapter_count].timestamp = timestamp_100ns / 10000; /* Convert from 100ns to milliseconds */
                
                /* Read chapter title (Pascal string) */
                uint8_t title_len = read_uint8(fd);
                if (title_len > 0 && title_len < 255) {
                    rb->read(fd, chapters[chapter_count].title, title_len);
                    chapters[chapter_count].title[title_len] = '\0';
                } else {
                    rb->snprintf(chapters[chapter_count].title, sizeof(chapters[chapter_count].title), 
                               "Chapter %d", chapter_count + 1);
                }

                chapter_count++;
            }
            break;
        }

        file_pos += atom_size;
    }
    
    /* If no Nero chapters found, look for Apple chapters in moov atom */
    if (chapter_count == 0) {
        file_pos = 0;
        
        while (1) {
            if (rb->lseek(fd, file_pos, SEEK_SET) < 0) break;
            if (rb->read(fd, &atom_size, 4) != 4) break;
            if (rb->read(fd, &atom_type, 4) != 4) break;
            
            /* Convert to host byte order */
            atom_size = (atom_size >> 24) | ((atom_size >> 8) & 0xFF00) | 
                       ((atom_size << 8) & 0xFF0000) | (atom_size << 24);
            atom_type = (atom_type >> 24) | ((atom_type >> 8) & 0xFF00) | 
                       ((atom_type << 8) & 0xFF0000) | (atom_type << 24);
            
            if (atom_size < 8) break;
            
            if (atom_type == MP4_moov) {
                /* Found movie atom - look for chapter tracks */
                DEBUGF("Found moov atom, searching for tracks...");
                off_t moov_end = file_pos + atom_size;
                off_t moov_pos = file_pos + 8;
                int track_num = 0;
                
                /* Look for tracks that might contain chapters */
                while (moov_pos < moov_end && chapter_count == 0) {
                    rb->lseek(fd, moov_pos, SEEK_SET);
                    uint32_t sub_size = read_uint32be(fd);
                    uint32_t sub_type = read_uint32be(fd);
                    
                    if (sub_size < 8) break;
                    
                    if (sub_type == MP4_trak) {
                        track_num++;
                        DEBUGF("Checking track %d for chapters...", track_num);
                        /* Check if this track contains chapters */
                        chapter_count = parse_apple_chapter_track(fd, moov_pos + 8, sub_size - 8, 
                                                                chapters);
                        if (chapter_count > 0) {
                            DEBUGF("Found %d chapters in track %d!", chapter_count, track_num);
                            break;
                        } else {
                            DEBUGF("Track %d: no chapters found", track_num);
                        }
                    }
                    
                    moov_pos += sub_size;
                }
                break;
            }
            
            file_pos += atom_size;
        }
    }
    
    rb->close(fd);
    return chapter_count;
}

/* Generate CUE file content */
static bool generate_cue_file(const char *mp4_path, struct chapter_info *chapters, int chapter_count) {
    char cue_path[MAX_PATH];
    char *dot;
    
    /* Create CUE filename */
    rb->strlcpy(cue_path, mp4_path, MAX_PATH);
    dot = rb->strrchr(cue_path, '.');
    if (dot) {
        rb->strcpy(dot, ".cue");
    } else {
        rb->strlcat(cue_path, ".cue", MAX_PATH);
    }
    
    int fd = rb->open(cue_path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) return false;
    
    /* Get MP4 metadata */
    struct mp3entry id3;
    bool have_metadata = rb->get_metadata(&id3, -1, mp4_path);
    
    /* Write CUE header */
    rb->fdprintf(fd, "REM Generated by Rockbox MP4 Chapters to CUE plugin\n");
    
    if (have_metadata && id3.title) {
        rb->fdprintf(fd, "TITLE \"%s\"\n", id3.title);
    } else {
        char *basename = rb->strrchr(mp4_path, '/');
        basename = basename ? basename + 1 : (char*)mp4_path;
        rb->fdprintf(fd, "TITLE \"%s\"\n", basename);
    }
    
    if (have_metadata && id3.artist) {
        rb->fdprintf(fd, "PERFORMER \"%s\"\n", id3.artist);
    }
    
    /* Extract filename for FILE line */
    char *filename = rb->strrchr(mp4_path, '/');
    filename = filename ? filename + 1 : (char*)mp4_path;
    rb->fdprintf(fd, "FILE \"%s\" MP3\n", filename);
    
    /* Write chapters as tracks */
    for (int i = 0; i < chapter_count; i++) {
        char time_str[16];
        timestamp_to_cue_time(chapters[i].timestamp, time_str, sizeof(time_str));
        
        rb->fdprintf(fd, "  TRACK %02d AUDIO\n", i + 1);
        rb->fdprintf(fd, "    TITLE \"%s\"\n", chapters[i].title);

        rb->fdprintf(fd, "    INDEX 01 %s\n", time_str);
    }
    
    rb->close(fd);
    return true;
}

/* Main plugin entry point */
enum plugin_status plugin_start(const void* parameter) {
    char* mp4_path = (char*)parameter;
    
    if (!mp4_path || !mp4_path[0]) {
        rb->splash(HZ*2, "No file specified");
        return PLUGIN_ERROR;
    }
    
    /* Check if file exists and is readable */
    if (!rb->file_exists(mp4_path)) {
        rb->splash(HZ*2, "File not found");
        return PLUGIN_ERROR;
    }
    
    /* Initialize buffer management */
    if (!init_buffer()) {
        rb->splash(HZ*2, "Out of memory");
        return PLUGIN_ERROR;
    }
    
    /* Reset buffer before use */
    buffer_reset();
    
    /* HACK. GET RAW BUFFER. DO ALLOCATION WHEN SIZE IS KNONW. Make sure no other allocations are done BEFORE*/
    struct chapter_info *chapters = (struct chapter_info*)buffer_alloc(0);
    if (!chapters) {
        rb->splash(HZ*2, "Out of memory for chapters");
        return PLUGIN_ERROR;
    }
    
    /* Find MP4 chapters */
    int chapter_count = find_mp4_chapters(mp4_path, chapters, 1000);
    
    if (chapter_count <= 0) {
        rb->splash(HZ*2, "No chapters found");
        return PLUGIN_OK;
    }
    
    rb->splashf(HZ, "Found %d chapters", chapter_count);
    
    /* Generate CUE file */
    if (generate_cue_file(mp4_path, chapters, chapter_count)) {
        rb->splashf(HZ*2, "CUE file created with %d tracks", chapter_count);
        return PLUGIN_OK;
    } else {
        rb->splash(HZ*2, "Failed to create CUE file");
        return PLUGIN_ERROR;
    }
}
