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

/* Enable this define when file logging is needed for troubleshooting. */
/* #define MP4CHAPTERS_DEBUG */

#ifdef MP4CHAPTERS_DEBUG
static int debug_fd = -1;

static void mp4_debug_init(const char *audio_path) {
    if (debug_fd >= 0) return; /* Already initialized */

    char log_path[MAX_PATH];
    char *dot;

    /* Create log filename based on audio file */
    rb->strlcpy(log_path, audio_path, MAX_PATH);
    dot = rb->strrchr(log_path, '.');
    if (dot) {
        rb->strcpy(dot, ".log");
    } else {
        rb->strlcat(log_path, ".log", MAX_PATH);
    }

    debug_fd = rb->open(log_path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
}

static void debug_log(const char *format, ...) {
    if (debug_fd < 0) return;

    va_list args;
    va_start(args, format);
    char buffer[512];
    rb->vsnprintf(buffer, sizeof(buffer), format, args);
    va_end(args);

    rb->write(debug_fd, buffer, rb->strlen(buffer));
    rb->write(debug_fd, "\n", 1);
}

static void mp4_debug_close(void) {
    if (debug_fd >= 0) {
        rb->close(debug_fd);
        debug_fd = -1;
    }
}
#else
#define debug_log(...) do { } while (0)
static void mp4_debug_init(const char *audio_path) { (void)audio_path; }
static void mp4_debug_close(void) { }
#endif

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

#define MAX_LEN 256
/* Structure to hold chapter information */
struct chapter_info {
    uint64_t timestamp;     /* Chapter start time in milliseconds */
    char title[MAX_LEN];        /* Chapter title */
};

/* MP4 utility functions */
static uint32_t read_uint32be(int fd) {
    uint8_t buf[4];
    ssize_t bytes_read = rb->read(fd, buf, 4);
    if (bytes_read != 4) {
        debug_log("read_uint32be failed: got %d bytes instead of 4", (int)bytes_read);
        return 0;
    }
    uint32_t result = (buf[0] << 24) | (buf[1] << 16) | (buf[2] << 8) | buf[3];
    DEBUGF("read_uint32be: bytes=%02X %02X %02X %02X -> 0x%08X\n", buf[0], buf[1], buf[2], buf[3], result);
    return result;
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
    int minutes = timestamp_ms / 60000;
    int seconds = (timestamp_ms % 60000) / 1000;
    int frames = ((timestamp_ms % 1000) * 75) / 1000; /* 75 frames per second for CUE */
    rb->snprintf(time_str, size, "%02d:%02d:%02d", minutes, seconds, frames);
}

/* Escape special characters in CUE strings */
static void escape_cue_string(const char *input, char *output, size_t output_size) {
    size_t in_pos = 0, out_pos = 0;
    
    while (input[in_pos] && out_pos < output_size - 1) {
        char c = input[in_pos];
        
        /* Replace or escape problematic characters */
        if (c == '"') {
            /* Replace quote with single quote */
            if (out_pos < output_size - 1) {
                output[out_pos++] = '\'';
            }
        } else if (c == '\r' || c == '\n') {
            /* Replace newlines with space */
            if (out_pos < output_size - 1) {
                output[out_pos++] = ' ';
            }
        } else if (c >= 32 && c <= 126) {
            /* Keep printable ASCII characters */
            output[out_pos++] = c;
        } else if ((unsigned char)c >= 128) {
            /* Keep UTF-8 characters as-is */
            output[out_pos++] = c;
        } else {
            /* Replace other control characters with space */
            if (out_pos < output_size - 1) {
                output[out_pos++] = ' ';
            }
        }
        in_pos++;
    }
    
    output[out_pos] = '\0';
}

/* Recursively search for atoms in MP4 structure */
static int search_for_atom(int fd, off_t start_pos, off_t end_pos, uint32_t target_atom,
                          off_t *found_pos, off_t *found_size) {
    off_t current_pos = start_pos;
    int search_count = 0;
    
    debug_log("search_for_atom: looking for %c%c%c%c in range %ld-%ld", 
               (char)(target_atom >> 24), (char)(target_atom >> 16), 
               (char)(target_atom >> 8), (char)target_atom, 
               (long)start_pos, (long)end_pos);
    
    while (current_pos < end_pos) {
        search_count++;
        if (search_count > 100) {
            debug_log("search_for_atom: too many atoms, breaking");
            break;
        }
        
        if (rb->lseek(fd, current_pos, SEEK_SET) < 0) {
            debug_log("search_for_atom: lseek failed at %ld", (long)current_pos);
            break;
        }
        
        uint32_t atom_size = read_uint32be(fd);
        uint32_t atom_type = read_uint32be(fd);
        
        if (atom_size < 8) {
            debug_log("search_for_atom: bad atom size %lu at %ld", (unsigned long)atom_size, (long)current_pos);
            break;
        }
        
        debug_log("search_for_atom: found %c%c%c%c size=%lu at %ld", 
                   (char)(atom_type >> 24), (char)(atom_type >> 16), 
                   (char)(atom_type >> 8), (char)atom_type, 
                   (unsigned long)atom_size, (long)current_pos);
        
        if (atom_type == target_atom) {
            *found_pos = current_pos;
            *found_size = atom_size;
            debug_log("search_for_atom: FOUND target atom at %ld", (long)current_pos);
            return 1;
        }
        
        /* Recursively search container atoms */
        if (atom_type == MP4_moov || atom_type == MP4_trak || 
            atom_type == MP4_mdia || atom_type == MP4_minf || atom_type == MP4_stbl) {
            debug_log("search_for_atom: recursing into container %c%c%c%c", 
                       (char)(atom_type >> 24), (char)(atom_type >> 16), 
                       (char)(atom_type >> 8), (char)atom_type);
            if (search_for_atom(fd, current_pos + 8, current_pos + atom_size,
                               target_atom, found_pos, found_size)) {
                return 1;
            }
        }
        
        current_pos += atom_size;
    }
    
    debug_log("search_for_atom: target not found after %d atoms", search_count);
    return 0;
}

/* Parse Apple chapter track - look for text track with chapter data */
static struct chapter_info* parse_apple_chapter_track(int fd, off_t track_start, off_t track_size, int* num_chapters)
{
    *num_chapters = 0;
    off_t track_end = track_start + track_size;
    int chapter_count = 0;
    uint32_t track_timescale = 1000; /* Default timescale */
    
    debug_log("Parsing Apple track...");
    
    /* Look for track header to check if this is a text track */
    off_t tkhd_pos, tkhd_size;
    debug_log("Searching for tkhd in track from %ld to %ld", (long)track_start, (long)track_end);
    if (!search_for_atom(fd, track_start, track_end, MP4_tkhd, &tkhd_pos, &tkhd_size)) {
        DEBUGF("No tkhd found in track\n");
        debug_log("No tkhd found in track range %ld-%ld", (long)track_start, (long)track_end);
        return 0;
    }
    
    debug_log("Found tkhd at %ld", (long)tkhd_pos);
    
    /* Look for media atom */
    off_t mdia_pos, mdia_size;
    if (!search_for_atom(fd, track_start, track_end, MP4_mdia, &mdia_pos, &mdia_size)) {
        DEBUGF("No mdia found in track\n");
        debug_log("No mdia found");
        return 0;
    }
    
    debug_log("Found mdia, checking timescale...");
    
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
        
        DEBUGF("Found track timescale: %u\n", track_timescale);
        debug_log("Found track timescale: %u", track_timescale);
    } else {
        DEBUGF("No mdhd found, using default timescale\n");
        debug_log("No mdhd, using default timescale");
    }
    
    /* Look for media handler to check track type */
    off_t hdlr_pos, hdlr_size;
    if (!search_for_atom(fd, mdia_pos, mdia_pos + mdia_size, MP4_hdlr, &hdlr_pos, &hdlr_size)) {
        DEBUGF("No hdlr found in track\n");
        debug_log("No hdlr found");
        return 0;
    }
    
    /* Check if this is a text track */
    rb->lseek(fd, hdlr_pos + 8 + 8, SEEK_SET); /* Skip atom header + version/flags */
    uint32_t handler_type = read_uint32be(fd);
    
    DEBUGF("Track handler type: %c%c%c%c\n", 
           (char)(handler_type >> 24), (char)(handler_type >> 16), 
           (char)(handler_type >> 8), (char)handler_type);
    
    debug_log("Handler: %c%c%c%c", 
               (char)(handler_type >> 24), (char)(handler_type >> 16), 
               (char)(handler_type >> 8), (char)handler_type);
    
    if (handler_type != MP4_TEXT && handler_type != MP4_tx3g) {
        DEBUGF("Not a text track, skipping\n");
        debug_log("Not text track: %c%c%c%c", 
                   (char)(handler_type >> 24), (char)(handler_type >> 16), 
                   (char)(handler_type >> 8), (char)handler_type);
        return 0; /* Not a text track */
    }
    
    DEBUGF("Found text track!\n");
    debug_log("Found text track!");
    
    /* Look for media information atom first */
    off_t minf_pos, minf_size;
    if (!search_for_atom(fd, mdia_pos + 8, mdia_pos + mdia_size, MP4_minf, &minf_pos, &minf_size)) {
        DEBUGF("No minf found in text track\n");
        debug_log("No minf found in text track");
        return 0;
    }
    
    DEBUGF("Found minf in text track\n");
    debug_log("Found minf in text track");
    
    /* Look for sample table */
    off_t stbl_pos, stbl_size;
    if (!search_for_atom(fd, minf_pos + 8, minf_pos + minf_size, MP4_stbl, &stbl_pos, &stbl_size)) {
        DEBUGF("No stbl found in text track\n");
        debug_log("No stbl found in text track");
        return 0;
    }
    
    DEBUGF("Found sample table in text track at pos %ld, size %ld\n", (long)stbl_pos, (long)stbl_size);
    debug_log("Found sample table in text track at pos %ld, size %ld", (long)stbl_pos, (long)stbl_size);
    
    /* Get time-to-sample information */
    off_t stts_pos, stts_size;
    if (!search_for_atom(fd, stbl_pos + 8, stbl_pos + stbl_size, MP4_stts, &stts_pos, &stts_size)) {
        DEBUGF("No stts found\n");
        debug_log("No stts found");
        return 0;
    }
    
    /* Get sample size information to know how many samples we have */
    off_t stsz_pos, stsz_size;
    if (!search_for_atom(fd, stbl_pos + 8, stbl_pos + stbl_size, MP4_stsz, &stsz_pos, &stsz_size)) {
        DEBUGF("No stsz found\n");
        debug_log("No stsz found");
        return 0;
    }
    
    /* Get sample-to-chunk table */
    off_t stsc_pos, stsc_size;
    if (!search_for_atom(fd, stbl_pos + 8, stbl_pos + stbl_size, MP4_stsc, &stsc_pos, &stsc_size)) {
        DEBUGF("No stsc found\n");
        debug_log("No stsc found");
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
        debug_log("No chunk offset table found");
        return 0;
    }
    
    DEBUGF("Found all required sample tables\n");
    debug_log("Found all required sample tables");
    
    /* Read sample size table to get number of samples */
    rb->lseek(fd, stsz_pos + 8 + 4, SEEK_SET); /* Skip atom header + version/flags */
    uint32_t default_sample_size = read_uint32be(fd);
    uint32_t sample_count = read_uint32be(fd);
    
    DEBUGF("Sample count: %u, default size: %u, timescale: %u\n", sample_count, default_sample_size, track_timescale);
    
    debug_log("Sample count: %u, default size: %u, timescale: %u", sample_count, default_sample_size, track_timescale);
    
    if (sample_count == 0) {
        debug_log("Sample count is zero, returning");
        return 0; /* Sanity check */
    }

    // Allocate memory for chapters
    struct chapter_info *chapters = buffer_alloc(sizeof(struct chapter_info) * sample_count);
    if (chapters == NULL) {
        DEBUGF("Failed to allocate memory for chapters\n");
        debug_log("Failed to allocate memory for chapters");
        return 0;
    }
    *num_chapters = sample_count;
    
    debug_log("Allocated memory for %u chapters", sample_count);
    
    
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
    
    /* Read sample-to-chunk table first */
    rb->lseek(fd, stsc_pos + 8 + 4, SEEK_SET); /* Skip atom header + version/flags */
    uint32_t stsc_entry_count = read_uint32be(fd);
    debug_log("Sample-to-chunk entries: %u", stsc_entry_count);
    
    /* For text tracks, usually each sample is in its own chunk, but let's check */
    uint32_t samples_per_chunk = 1; /* Default assumption */
    if (stsc_entry_count > 0) {
        /* Read first entry to see samples per chunk */
        uint32_t first_chunk = read_uint32be(fd);
        samples_per_chunk = read_uint32be(fd);
        debug_log("First chunk: %u, samples per chunk: %u", first_chunk, samples_per_chunk);
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
            
            DEBUGF("Chapter %d: time=%lu units, timestamp=%lu ms (timescale=%u)\n", 
                   chapter_count, (unsigned long)current_time, (unsigned long)chapters[chapter_count].timestamp, track_timescale);
            
            /* Calculate sample position in file */
            /* For text tracks, typically each sample is in its own chunk */
            uint64_t sample_pos = 0;
            if (sample_index < chunk_count) {
                sample_pos = chunk_offsets[sample_index];
            } else if (chunk_count > 0) {
                /* If we have fewer chunks than samples, assume samples are sequential within chunks */
                uint32_t chunk_index = sample_index / samples_per_chunk;
                uint32_t sample_in_chunk = sample_index % samples_per_chunk;
                
                if (chunk_index < chunk_count) {
                    sample_pos = chunk_offsets[chunk_index];
                    /* Add offset for samples within chunk */
                    if (default_sample_size > 0) {
                        sample_pos += sample_in_chunk * default_sample_size;
                    }
                }
            }
            
            /* Try to read the actual chapter title from sample data */
            if (sample_pos > 0) {
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
                    /* Read sample data from calculated position */
                    rb->lseek(fd, sample_pos, SEEK_SET);
                    int bytes_read = rb->read(fd, sample_buffer, sample_size);
                    
                    if (bytes_read > 0) {
                        /* Add debug logging for sample data */
                        debug_log("Sample %u: pos=%lu size=%u bytes_read=%d", 
                                  sample_index, (unsigned long)sample_pos, sample_size, bytes_read);
                        
                        /* Debug: show first few bytes of sample */
                        if (bytes_read >= 4) {
                            debug_log("Sample %u data: %02X %02X %02X %02X...", 
                                      sample_index, sample_buffer[0], sample_buffer[1], 
                                      sample_buffer[2], sample_buffer[3]);
                        }
                        
                        /* Parse text sample - format varies but often starts with length */
                        const char *title_text = NULL;
                        
                        /* Try different text sample formats */
                        if (sample_size >= 2) {
                            /* Check for length-prefixed string (common format) */
                            uint16_t text_len = (sample_buffer[0] << 8) | sample_buffer[1];
                            debug_log("Sample %u: trying length-prefixed, text_len=%u", sample_index, text_len);
                            if (text_len > 0 && text_len < sample_size - 2 && text_len < 200) {
                                title_text = &sample_buffer[2];
                                rb->strlcpy(chapters[chapter_count].title, title_text, MIN(text_len  + 1, MAX_LEN));
                                debug_log("Sample %u: extracted title '%s'", sample_index, chapters[chapter_count].title);
                            }
                        }
                        
                        /* If that didn't work, try looking for plain text */
                        if (title_text == NULL) {
                            debug_log("Sample %u: trying plain text search", sample_index);
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
                                        size_t copy_len = text_end - k + 1;
                                        rb->strlcpy(chapters[chapter_count].title, &sample_buffer[k], MIN(copy_len, MAX_LEN));
                                        title_text = chapters[chapter_count].title;
                                        debug_log("Sample %u: found plain text '%s' at offset %u", 
                                                  sample_index, chapters[chapter_count].title, k);
                                        break;
                                    }
                                }
                            }
                        }
                        
                        if (title_text == NULL) {
                            debug_log("Sample %u: no readable text found", sample_index);
                        }
                        
                        DEBUGF("Sample %u: size=%u, title='%s'\n", sample_index, sample_size, 
                               title_text ? chapters[chapter_count].title : "failed");
                    }
                }
            }
            
            /* Fallback to generic title if we couldn't read the text */
            if (rb->strlen(chapters[chapter_count].title) == 0) {
                rb->snprintf(chapters[chapter_count].title, sizeof(chapters[chapter_count].title), 
                           "Chapter %d", chapter_count + 1);
            }

            chapter_count++;
            sample_index++;
            current_time += sample_duration;
        }
    }
    
    DEBUGF("Extracted %d chapters from text track\n", chapter_count);
    
    return chapters;
}

/* Find and parse MP4 chapters (both Nero and Apple formats) */
static struct chapter_info* find_mp4_chapters(const char *mp4_path, int* chapter_count) {
    int fd = rb->open(mp4_path, O_RDONLY);
    if (fd < 0) {
        debug_log("Failed to open file: %s", mp4_path);
        return NULL;
    }
    
    debug_log("Starting MP4 chapter search...");
    
    uint32_t atom_size, atom_type;
    off_t file_pos = 0;
    
    /* First pass: Look for Nero chpl atoms (simpler format) */
    debug_log("Looking for Nero chapters...");
    int atom_count = 0;
    while (1) {
        if (rb->lseek(fd, file_pos, SEEK_SET) < 0) {
            debug_log("lseek failed at pos %ld", (long)file_pos);
            break;
        }
        
        atom_size = read_uint32be(fd);
        atom_type = read_uint32be(fd);
        atom_count++;
        
        if (atom_size == 0 || atom_type == 0) {
            debug_log("NULL atom %d: size=%lu type=%lu", atom_count, (unsigned long)atom_size, (unsigned long)atom_type);
            break;
        }
        
        if (atom_size < 8) {
            debug_log("Bad atom size: %lu", (unsigned long)atom_size);
            break;
        }
        
        debug_log("Atom %d: %c%c%c%c size=%lu", atom_count,
                   (char)(atom_type >> 24), (char)(atom_type >> 16), 
                   (char)(atom_type >> 8), (char)atom_type, (unsigned long)atom_size);
        
        if (atom_type == MP4_chpl) {
            debug_log("Found Nero chpl atom!");
            /* Found Nero chapter list atom */
            rb->lseek(fd, 8, SEEK_CUR); /* Skip version and flags */
            uint8_t num_chapters = read_uint8(fd);
            debug_log("Nero chapters count: %d", num_chapters);
            struct chapter_info *chapters = buffer_alloc( sizeof(*chapters) * num_chapters);
            for (int i = 0; i < num_chapters; i++) {
                uint64_t timestamp_100ns = read_uint64be(fd);
                chapters[i].timestamp = timestamp_100ns / 10000; /* Convert from 100ns to milliseconds */

                /* Read chapter title (Pascal string) */
                ssize_t title_len = read_uint8(fd);
                title_len = rb->read(fd, chapters[i].title, MIN(title_len, MAX_LEN -1));
                if (title_len > 0)
                    chapters[i].title[title_len] = '\0';
                else
                    rb->snprintf(chapters[i].title, sizeof(chapters[i].title), 
                               "Chapter %d", i + 1);
            }

            *chapter_count = num_chapters;
            rb->close(fd);
            return chapters; /* Found Nero chapters */
        }

        file_pos += atom_size;
    }
    
    /* If no Nero chapters found, look for Apple chapters in moov atom */
    struct chapter_info *chapters = NULL;
    {
        file_pos = 0;
        debug_log("Looking for Apple chapters...");
        
        while (1) {
            if (rb->lseek(fd, file_pos, SEEK_SET) < 0) break;
            
            atom_size = read_uint32be(fd);
            atom_type = read_uint32be(fd);
            
            if (atom_size < 8) break;
            
            debug_log("Apple search: %c%c%c%c size=%lu", 
                       (char)(atom_type >> 24), (char)(atom_type >> 16), 
                       (char)(atom_type >> 8), (char)atom_type, (unsigned long)atom_size);
            
            if (atom_type == MP4_moov) {
                debug_log("Found moov atom!");
                /* Found movie atom - look for chapter tracks */
                DEBUGF("Found moov atom, searching for tracks...\n");
                off_t moov_end = file_pos + atom_size;
                off_t moov_pos = file_pos + 8;
                int track_num = 0;
                
                debug_log("moov: pos=%ld end=%ld size=%lu", 
                           (long)moov_pos, (long)moov_end, (unsigned long)atom_size);
                
                /* Look for tracks that might contain chapters */
                int loop_count = 0;
                while (moov_pos < moov_end && *chapter_count == 0) {
                    loop_count++;
                    debug_log("Loop %d: seeking to pos %ld", loop_count, (long)moov_pos);
                    
                    if (rb->lseek(fd, moov_pos, SEEK_SET) < 0) {
                        debug_log("lseek failed at pos %ld", (long)moov_pos);
                        break;
                    }
                    
                    /* Verify current position */
                    off_t current_pos = rb->lseek(fd, 0, SEEK_CUR);
                    debug_log("Loop %d: current file pos is %ld", loop_count, (long)current_pos);
                    
                    debug_log("Loop %d: reading sub atom", loop_count);
                    uint32_t sub_size = read_uint32be(fd);
                    uint32_t sub_type = read_uint32be(fd);
                    debug_log("Loop %d: read sub_size=%lu sub_type=0x%08X", loop_count, (unsigned long)sub_size, sub_type);
                    
                    debug_log("sub atom: %c%c%c%c size=%lu", 
                               (char)(sub_type >> 24), (char)(sub_type >> 16), 
                               (char)(sub_type >> 8), (char)sub_type, (unsigned long)sub_size);
                    
                    if (sub_size < 8) {
                        debug_log("Bad sub_size %lu, breaking", (unsigned long)sub_size);
                        break;
                    }
                    
                    /* Safety check for reasonable atom size */
                    if (sub_size > 100000000) { /* 100MB limit */
                        debug_log("Suspiciously large sub_size %lu, skipping", (unsigned long)sub_size);
                        moov_pos += 8; /* Skip this atom header and try next */
                        continue;
                    }
                    
                    if (sub_type == MP4_trak) {
                        track_num++;
                        debug_log("Found track %d, checking...", track_num);
                        debug_log("Track %d: pos=%ld size=%lu", track_num, (long)moov_pos, (unsigned long)sub_size);
                        DEBUGF("Checking track %d for chapters...\n", track_num);
                        /* Check if this track contains chapters */
                        debug_log("Calling parse_apple_chapter_track for track %d", track_num);
                        chapters = parse_apple_chapter_track(fd, moov_pos + 8, sub_size - 8, chapter_count);
                        debug_log("parse_apple_chapter_track returned, chapters=%d", *chapter_count);

                        if (*chapter_count > 0) {
                            debug_log("SUCCESS: %d chapters in track %d!", *chapter_count, track_num);
                            DEBUGF("Found %d chapters in track %d!\n", *chapter_count, track_num);
                            break;
                        } else {
                            debug_log("Track %d: no chapters", track_num);
                            DEBUGF("Track %d: no chapters found\n", track_num);
                        }
                    } else {
                        debug_log("Skipping non-track atom: %c%c%c%c", 
                                   (char)(sub_type >> 24), (char)(sub_type >> 16), 
                                   (char)(sub_type >> 8), (char)sub_type);
                    }
                    
                    debug_log("Loop %d: advancing moov_pos by %lu", loop_count, (unsigned long)sub_size);
                    moov_pos += sub_size;
                    
                    /* Safety check to prevent infinite loop */
                    if (sub_size == 0) {
                        debug_log("Zero size atom, breaking");
                        break;
                    }
                    
                    /* Additional safety check for loop count */
                    if (loop_count > 1000) {
                        debug_log("Too many loops (%d), breaking", loop_count);
                        break;
                    }
                }
                
                if (*chapter_count == 0) {
                    debug_log("No chapters found in %d tracks", track_num);
                }
                break;
            }
            
            file_pos += atom_size;
        }
    }
    
    rb->close(fd);
    return chapters; /* No chapters found */
}

/* Generate CUE file content */
static bool generate_cue_file(const char *mp4_path, struct chapter_info *chapters, int chapter_count) {
    char cue_path[MAX_PATH];
    char *dot;
    
    debug_log("Starting CUE generation...");
    
    /* Create CUE filename */
    rb->strlcpy(cue_path, mp4_path, MAX_PATH);
    dot = rb->strrchr(cue_path, '.');
    if (dot) {
        rb->strcpy(dot, ".cue");
    } else {
        rb->strlcat(cue_path, ".cue", MAX_PATH);
    }
    
    debug_log("Creating CUE file: %s", cue_path);
    
    int fd = rb->open(cue_path, O_WRONLY | O_CREAT | O_TRUNC, 0666);
    if (fd < 0) {
        debug_log("Failed to create CUE file");
        return false;
    }
    
    debug_log("CUE file opened, writing header...");
    
    /* Write UTF-8 BOM for better compatibility */
    const char utf8_bom[] = "\xEF\xBB\xBF";
    rb->write(fd, utf8_bom, 3);
    
    /* Get MP4 metadata */
    struct mp3entry id3;
    bool have_metadata = rb->get_metadata(&id3, -1, mp4_path);
    
    /* Write CUE header */
    rb->fdprintf(fd, "REM Generated by Rockbox MP4 Chapters to CUE plugin (UTF-8)\n");
    
    if (have_metadata && id3.title) {
        char escaped_title[MAX_LEN];
        escape_cue_string(id3.title, escaped_title, sizeof(escaped_title));
        rb->fdprintf(fd, "TITLE \"%s\"\n", escaped_title);
    } else {
        char *basename = rb->strrchr(mp4_path, '/');
        basename = basename ? basename + 1 : (char*)mp4_path;
        char escaped_basename[MAX_LEN];
        escape_cue_string(basename, escaped_basename, sizeof(escaped_basename));
        rb->fdprintf(fd, "TITLE \"%s\"\n", escaped_basename);
    }
    
    if (have_metadata && id3.artist) {
        char escaped_artist[MAX_LEN];
        escape_cue_string(id3.artist, escaped_artist, sizeof(escaped_artist));
        rb->fdprintf(fd, "PERFORMER \"%s\"\n", escaped_artist);
    }
    
    /* Extract filename for FILE line */
    char *filename = rb->strrchr(mp4_path, '/');
    filename = filename ? filename + 1 : (char*)mp4_path;
    rb->fdprintf(fd, "FILE \"%s\" MP3\n", filename);
    
    debug_log("Writing %d chapters...", chapter_count);
    
    /* Write chapters as tracks  */
    for (int i = 0; i < chapter_count; i++) {
        char time_str[16];
        char escaped_title[MAX_LEN];
        
        timestamp_to_cue_time(chapters[i].timestamp, time_str, sizeof(time_str));
        escape_cue_string(chapters[i].title, escaped_title, sizeof(escaped_title));
        
        rb->fdprintf(fd, "  TRACK %02d AUDIO\n", i + 1);
        rb->fdprintf(fd, "    TITLE \"%s\"\n", escaped_title);
        rb->fdprintf(fd, "    INDEX 01 %s\n", time_str);
    }
    
    rb->close(fd);
    debug_log("CUE file generation complete!");
    return true;
}

/* Main plugin entry point */
enum plugin_status plugin_start(const void* parameter) {
    char* mp4_path = (char*)parameter;
    
    /* Initialize debug logging first */
    if (mp4_path && mp4_path[0]) {
        mp4_debug_init(mp4_path);
    }
    
    debug_log("MP4 Chapters plugin starting...");
    
    if (!mp4_path || !mp4_path[0]) {
        rb->splash(HZ*2, "No file specified");
        mp4_debug_close();
        return PLUGIN_ERROR;
    }
    
    debug_log("Processing file: %s", mp4_path);
    
    /* Check if file exists and is readable */
    if (!rb->file_exists(mp4_path)) {
        rb->splash(HZ*2, "File not found");
        mp4_debug_close();
        return PLUGIN_ERROR;
    }
    
    debug_log("File exists, initializing...");
    
    /* Initialize buffer management */
    if (!init_buffer()) {
        rb->splash(HZ*2, "Out of memory");
        mp4_debug_close();
        return PLUGIN_ERROR;
    }
    
    debug_log("Buffer initialized: %d bytes", (int)g_buffer_size);
    
    /* Reset buffer before use */
    buffer_reset();
    

    /* Find MP4 chapters */
    int chapter_count = 0;  /* Initialize to 0! */
    debug_log("Starting chapter search...");
    struct chapter_info *chapters = find_mp4_chapters(mp4_path, &chapter_count);
    
    if (chapter_count <= 0) {
        rb->splash(HZ*2, "No chapters found");
        mp4_debug_close();
        return PLUGIN_OK;
    }

    debug_log("Found %d chapters, generating CUE...", chapter_count);
    
    /* Generate CUE file */
    bool success = generate_cue_file(mp4_path, chapters, chapter_count);
    
    if (success) {
        rb->splashf(HZ*2, "CUE file created with %d tracks", chapter_count);
        debug_log("SUCCESS: CUE file created with %d tracks", chapter_count);
    } else {
        rb->splashf(HZ*2, "Failed to create CUE file");
        debug_log("ERROR: Failed to create CUE file");
    }
    
    mp4_debug_close();
    return success ? PLUGIN_OK : PLUGIN_ERROR;
}
