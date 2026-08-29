/***************************************************************************
 *             __________               __   ___.
 *   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
 *   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
 *   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
 *   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
 *                     \/            \/     \/    \/            \/
 * $Id$
 *
 * Copyright (C) 2005 Magnus Holmgren
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

#include <ctype.h>
#include <math.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "platform.h"
#include "string-extra.h"
#include "strcasecmp.h"
#include "metadata.h"
#include "debug.h"
#include "replaygain.h"
#include "fixedpoint.h"
#include "metadata_common.h"

#define FP_BITS         (12)
#define FP_ONE          (1 << FP_BITS)
#define FP_MIN          (-48 * FP_ONE)
#define FP_MAX          ( 17 * FP_ONE)
#define FP_OPUS_BOOST   (  5 * FP_ONE)
#define OPUS_FP_BITS    (8)
#define FP_OPUS_SCALE   (FP_BITS - OPUS_FP_BITS)

void replaygain_itoa(char* buffer, int length, long int_gain)
{
    /* int_gain uses Q19.12 format. */
    int one  = abs(int_gain) >> FP_BITS;
    int cent = ((abs(int_gain) & 0x0fff) * 100 + (FP_ONE/2)) >> FP_BITS;
    snprintf(buffer, length, "%s%d.%02d dB", (int_gain<0) ? "-":"+", one, cent);
}

static long fp_atof(const char* s, int precision)
{
    long int_part = 0;
    long int_one = BIT_N(precision);
    long frac_part = 0;
    long frac_count = 0;
    long frac_max = ((precision * 4) + 12) / 13;
    long frac_max_int = 1;
    long sign = 1;
    bool point = false;

    while ((*s != '\0') && isspace(*s))
    {    
        s++;     
    }

    if (*s == '-')
    {
        sign = -1;
        s++;
    }
    else if (*s == '+')
    {
        s++;
    }

    while (*s != '\0')
    {
        if (*s == '.')
        {
            if (point)
            {
                break;
            }

            point = true;
        }
        else if (isdigit(*s))
        {
            if (point)
            {
                if (frac_count < frac_max)
                {
                    frac_part = frac_part * 10 + (*s - '0');
                    frac_count++;
                    frac_max_int *= 10;
                }
            }
            else
            {
                int_part = int_part * 10 + (*s - '0');
            }
        }
        else
        {
            break;
        }

        s++;
    }

    while (frac_count < frac_max)
    {
      frac_part *= 10;
      frac_count++;
      frac_max_int *= 10;
    }

    return sign * ((int_part * int_one)
        + (((int64_t) frac_part * int_one) / frac_max_int));
}

/* Attempts to convert a string to an int16_t.
 * Returns 0 and sets the result to 0 if the conversion failed.
 *
 *  str       The string to attempt to convert to an int16_t.
 *  result    A pointer to the result. Set to 0 on failure to convert.
 */
static int atoi16_chk(const char* str, int16_t* result)
{
    /* Use int32_t for the intermediate value to avoid overflow. */
    int32_t value = 0;
    int sign = 1;
    char cur_char = '\0';

    *result = 0;

    if (!str)
        return 0;

    cur_char = *str++;
    while (isspace(cur_char))
        cur_char = *str++;

    switch (cur_char)
    {
        case '-':
            sign = -1;
            /* Intentional fallthrough. */
        case '+':
            cur_char = *str++;
            break;
        default:
            if (!isdigit(cur_char))
                return 0;
    }

    while (isdigit(cur_char) && value < INT16_MAX)
    {
        value = (value * 10) + (cur_char - '0');
        cur_char = *str++;
    }

    if (value > INT16_MAX || (sign == -1 && value > -INT16_MIN))
        return 0;
    
    *result = (int16_t)(value * sign);
    return 1;
}

static long convert_gain(long gain)
{
    /* Don't allow unreasonably low or high gain changes.
     * Our math code can't handle it properly anyway. :) */
    gain = MAX(gain, FP_MIN);
    gain = MIN(gain, FP_MAX);

    return fp_factor(gain, FP_BITS) << (24 - FP_BITS);
}

/* Get the sample scale factor in Q19.12 format from a gain value. Returns 0
 * for no gain.
 *
 * str  Gain in dB as a string. E.g., "-3.45 dB"; the "dB" part is ignored.
 */
static long get_replaygain(const char* str)
{
    return fp_atof(str, FP_BITS);
}

/* Get the sample scale factor in Q19.12 format from a gain value as used
 * in the Ogg Opus format. Also, compensate for the -23 dB target used in
 * Opus, adjusting it to match the -18 dB target used in ReplayGain.
 *
 * str  Gain as a Q7.8 decimal string. E.g., "-883"; approximatly -3.45 dB.
 */
static bool get_replaygain_opus(const char* str, long* result)
{
    int16_t file_gain;
    long fp_gain;
    int success = atoi16_chk(str, &file_gain);

    if (!success)
        return false;
    
    fp_gain = ((long)file_gain) << FP_OPUS_SCALE;
    *result = fp_gain + FP_OPUS_BOOST;
    return true;
}

/* Get the peak volume in Q7.24 format.
 *
 * str  Peak volume. Full scale is specified as "1.0". Returns 0 for no peak.
 */
static long get_replaypeak(const char* str)
{
    return fp_atof(str, 24);
}

/* Get a sample scale factor in Q7.24 format from a gain value.
 *
 * int_gain  Gain in dB, multiplied by 100.
 */
long get_replaygain_int(long int_gain)
{
    return convert_gain(int_gain * FP_ONE / 100);
}

/* Parse a ReplayGain tag conforming to the "VorbisGain standard". If a
 * valid tag is found, update mp3entry struct accordingly. Existing values 
 * are not overwritten.
 *
 * key     Name of the tag.
 * value   Value of the tag.
 * entry   mp3entry struct to update.
 */
void parse_replaygain(const char* key, const char* value, 
                      struct mp3entry* entry)
{
    static const char *rg_options[] = {"replaygain_track_gain", "rg_radio",
                                       "replaygain_album_gain", "rg_audiophile",
                                       "replaygain_track_peak", "rg_peak",
                                       "replaygain_album_peak",
                                       "R128_TRACK_GAIN", "R128_ALBUM_GAIN",
                                        NULL};

    int rg_op = string_option(key, rg_options, true);

    if ((rg_op == 0 || rg_op == 1) && !entry->track_gain)
    {  /*replaygain_track_gain||rg_radio*/
        entry->track_level = get_replaygain(value);
        entry->track_gain  = convert_gain(entry->track_level);
    }
    else if ((rg_op == 2 || rg_op == 3) && !entry->album_gain)
    {  /*replaygain_album_gain||rg_audiophile*/
        entry->album_level = get_replaygain(value);
        entry->album_gain  = convert_gain(entry->album_level);
    }
    else if ((rg_op == 4 || rg_op == 5) && !entry->track_peak)
    { /*replaygain_track_peak||rg_peak*/
        entry->track_peak = get_replaypeak(value);
    }
    else if ((rg_op == 6) && !entry->album_peak)
    { /*replaygain_album_peak*/
        entry->album_peak = get_replaypeak(value);
    }
    else if (rg_op == 7)
    {
        long level;
        if (get_replaygain_opus(value, &level))
        {
            entry->track_level = level;
            entry->track_gain  = convert_gain(entry->track_level);
        }
    }
    else if (rg_op == 8)
    {
        long level;
        if (get_replaygain_opus(value, &level))
        {
            entry->album_level = level;
            entry->album_gain  = convert_gain(entry->album_level);
        }
    }
}

/* Parse Ogg Opus's R128_TRACK_GAIN and R128_ALBUM_GAIN tags. If a valid
 * tag is found, update mp3entry struct accordingly. Existing values are
 * not overwritten. Note that RFC7845 specifies no peak tag is included.
 *
 * key     Name of the tag.
 * value   Value of the tag.
 * entry   mp3entry struct to update.
 */
void parse_replaygain_opus(const char* key, const char* value, 
                      struct mp3entry* entry)
{
   
}

/* Set ReplayGain values from integers. Existing values are not overwritten. 
 *
 * album   If true, set album values, otherwise set track values.
 * gain    Gain value in dB, multiplied by 512. 0 for no gain.
 * peak    Peak volume in Q7.24 format, where 1.0 is full scale. 0 for no 
 *         peak volume.
 * entry   mp3entry struct to update.
 */
void parse_replaygain_int(bool album, long gain, long peak,
                          struct mp3entry* entry)
{
    gain = gain * FP_ONE / 512;

    if (album)
    {
        entry->album_level = gain;
        entry->album_gain  = convert_gain(gain);
        entry->album_peak  = peak;
    }
    else
    {
        entry->track_level = gain;
        entry->track_gain  = convert_gain(gain);
        entry->track_peak  = peak;
    }
}
