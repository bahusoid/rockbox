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

#include "codeclib.h"
#include "libasf/asf.h"
#include <codecs/libmad/mad.h>
#include <inttypes.h>

CODEC_HEADER

#if NUM_CORES > 1 && !defined(MPEGPLAYER)
#define MPA_SYNTH_ON_COP
#endif

static struct mad_stream stream IBSS_ATTR;
static struct mad_frame frame IBSS_ATTR;
static struct mad_synth synth IBSS_ATTR;

#ifdef MPA_SYNTH_ON_COP
static volatile short die IBSS_ATTR = 0;          /*thread should die*/

#if (CONFIG_CPU == PP5024) || (CONFIG_CPU == PP5022)
static mad_fixed_t sbsample_prev[2][36][32] IBSS_ATTR;
#else
static mad_fixed_t sbsample_prev[2][36][32] SHAREDBSS_ATTR; 
#endif

static struct semaphore synth_done_sem IBSS_ATTR;
static struct semaphore synth_pending_sem IBSS_ATTR;
#endif

#define INPUT_CHUNK_SIZE   8192

static mad_fixed_t mad_frame_overlap[2][32][18] IBSS_ATTR;
static mad_fixed_t sbsample[2][36][32] IBSS_ATTR;

static unsigned char mad_main_data[MAD_BUFFER_MDLEN] IBSS_ATTR;
/* TODO: what latency does layer 1 have? */
static int mpeg_latency[3] = { 0, 481, 529 };
static int mpeg_framesize[3] = {384, 1152, 1152};

static unsigned char stream_buffer[INPUT_CHUNK_SIZE] IBSS_ATTR;
static unsigned char *stream_data_start;
static unsigned char *stream_data_end;
static unsigned char *packetdata;
static int stream_data_available;
static int packetlength;
static int packetdatasize;
static int packetrest;
static unsigned char *lastpacketpos;

static void reset_stream_buffer(void)
{
    stream_data_start = stream_buffer;
    stream_data_end = stream_buffer;
    stream_data_available = 1;
    packetdatasize = 0;
    packetrest = 0;
    lastpacketpos = stream_buffer;
}

static inline unsigned char *get_stream_data(size_t *realsize, size_t reqsize)
{
    static int errcount = 0;
    size_t datasize = stream_data_end - stream_data_start;
    if (!ci->id3->is_asf_stream)
        return ci->request_buffer(realsize, reqsize);
    else if (datasize < INPUT_CHUNK_SIZE / 2)
    {
        if (stream_data_start < stream_data_end && stream_data_start > stream_buffer)
        {
            lastpacketpos -= stream_data_start - stream_buffer;
            memmove(stream_buffer, stream_data_start, datasize);
            stream_data_start = stream_buffer;
            stream_data_end = stream_buffer + datasize;
        }
        while (datasize < INPUT_CHUNK_SIZE && (packetrest || stream_data_available > 0))
        {
            if (packetrest && packetdata)
            {
                datasize = INPUT_CHUNK_SIZE - datasize;
                if (datasize > (size_t)packetrest)
                    datasize = packetrest;
                memcpy(stream_data_end, packetdata, datasize);
                packetrest -= datasize;
                stream_data_end += datasize;
                if (packetrest)
                    packetdata += datasize;
                else
                {
                    ci->advance_buffer(packetlength);
                    lastpacketpos = stream_data_end;
                }
                datasize = stream_data_end - stream_data_start;
            }
            else
            {
                asf_waveformatex_t *wfx = (asf_waveformatex_t *)(ci->id3->toc);
                int res = asf_read_packet(&packetdata, &packetdatasize, &packetlength, wfx);
                if (res < 0)
                {
                    if (res == ASF_ERROR_EOF)
                        stream_data_available = 0;
                    else if (++errcount > 5)
                        stream_data_available = -1;
                }
                else errcount = 0;
                packetrest = packetdatasize;
            }
        }
    }
    if (packetdatasize && lastpacketpos > stream_data_start)
      ci->set_offset(ci->curpos - ((lastpacketpos - stream_data_start) / packetdatasize + 1) * packetlength);
    *realsize = (datasize > reqsize) ? reqsize : datasize;
    return stream_data_start;
}

static void advance_stream_buffer(size_t size)
{
    if (!ci->id3->is_asf_stream)
        ci->advance_buffer(size);
    else if (stream_data_start + size > stream_data_end)
        stream_data_start = stream_data_end;
    else stream_data_start += size;
}

static void init_mad(void)
{
    ci->memset(&stream, 0, sizeof(struct mad_stream));
    ci->memset(&frame , 0, sizeof(struct mad_frame));
    ci->memset(&synth , 0, sizeof(struct mad_synth));

#ifdef MPA_SYNTH_ON_COP
    frame.sbsample_prev = &sbsample_prev;
    frame.sbsample      = &sbsample;
#else
    frame.sbsample_prev = &sbsample;
    frame.sbsample      = &sbsample;
#endif

    /* We do this so libmad doesn't try to call codec_calloc(). This needs to
     * be called before mad_stream_init(), mad_frame_inti() and 
     * mad_synth_init(). */
    frame.overlap    = &mad_frame_overlap;
    stream.main_data = &mad_main_data;
    
    /* Call mad initialization. Those will zero the arrays frame.overlap,
     * frame.sbsample and frame.sbsample_prev. Therefore there is no need to 
     * zero them here. */
    mad_stream_init(&stream);
    mad_frame_init(&frame);
    mad_synth_init(&synth);
}

/*
 * Compute the elapsed percentage of the file based on elapsed time.
 */
static unsigned int toc_time_to_percent(unsigned long time)
{
    unsigned int percent = (uint64_t)time * 100 / ci->id3->length;

    return MIN(percent, 100);
}

/*
 * Locate the percent interval that contains the file position "pos".
 */
static unsigned int toc_pos_to_percent(unsigned long pos)
{
    if (pos >= ci->id3->filesize)
        return 100;

    /* Do a binary search comparing fractions pos/fsize and toc[i]/256 */
    uint64_t xpos = (uint64_t)pos * 256;
    unsigned int a = 0, b = 99;
    while (a != b) {
        unsigned int i = (a + b + 1) / 2;
        uint64_t cmp = (uint64_t)ci->id3->toc[i] * ci->id3->filesize;
        if (xpos < cmp)
            b = i - 1;
        else
            a = i;
    }

    return a;
}

/*
 * Get start and end points for linear interpolation within the given
 * percent interval. "percent" must be from 0 to 99.
 */
static void
toc_get_interpolation_points(unsigned int percent,
                             unsigned long *pos_a, unsigned long *pos_b,
                             unsigned long *time_a, unsigned long *time_b)
{
    /* Start point */
    *pos_a = (uint64_t)ci->id3->toc[percent] * ci->id3->filesize / 256;
    *time_a = (uint64_t)percent * ci->id3->length / 100;

    /* End point is either the next TOC mark or end of file */
    percent++;
    if (percent < 100) {
        *pos_b = (uint64_t)ci->id3->toc[percent] * ci->id3->filesize / 256;
        *time_b = (uint64_t)percent * ci->id3->length / 100;
    } else {
        *pos_b = ci->id3->filesize;
        *time_b = ci->id3->length;
    }
}

/*
 * Return an output value in the range [out_a, out_b] with the
 * same relative position as the input w.r.t. the range [in_a, in_b]
 */
static unsigned long toc_interpolate(unsigned long input,
                                     unsigned long in_a, unsigned long in_b,
                                     unsigned long out_a, unsigned long out_b)
{
    if (input <= in_a)
        return out_a;

    if (input >= in_b)
        return out_b;

    unsigned long in_scale = in_b - in_a;
    unsigned long out_scale = out_b - out_a;

    unsigned long tmp = (uint64_t)out_scale * (input - in_a) / in_scale;
    return tmp + out_a;
}

/*
 * Compute the file position corresponding to an elapsed time using
 * the TOC, using linear interpolation. Fast but inaccurate.
 */
static unsigned long toc_get_pos_interpolated(unsigned long time)
{
    unsigned int percent = toc_time_to_percent(time);
    if (percent == 100)
        return ci->id3->length;

    unsigned long pos_a, pos_b;
    unsigned long time_a, time_b;
    toc_get_interpolation_points(percent, &pos_a, &pos_b, &time_a, &time_b);

    return toc_interpolate(time, time_a, time_b, pos_a, pos_b);
}

/*
 * Compute the elapsed time at file position "pos" using the TOC,
 * using linear interpolation. Fast but inaccurate.
 */
static unsigned long toc_get_time_interpolated(unsigned long pos)
{
    unsigned int percent = toc_pos_to_percent(pos);
    if (percent == 100)
        return ci->id3->length;

    unsigned long pos_a, pos_b;
    unsigned long time_a, time_b;
    toc_get_interpolation_points(percent, &pos_a, &pos_b, &time_a, &time_b);

    return toc_interpolate(pos, pos_a, pos_b, time_a, time_b);
}

static int get_file_pos(int newtime)
{
    int pos = -1;
    struct mp3entry *id3 = ci->id3;

    if (id3->vbr) {
        if (id3->has_toc) {
            pos = toc_get_pos_interpolated(newtime);
        } else {
            /* No TOC exists, estimate the new position */
            pos = (uint64_t)newtime * id3->filesize / id3->length;
        }
        // VBR seek might be very inaccurate in long files 
        // So make sure that seeking actually happened in the intended direction 
        // Fix jumps in the wrong direction by seeking relative to the current position
        long delta = id3->elapsed - newtime;        
        int curpos = ci->curpos - id3->first_frame_offset;
        if ((delta >= 0 && pos > curpos) || (delta < 0 && pos < curpos))
        {
            pos = curpos - delta * id3->filesize / id3->length;
        }
    } else if (id3->bitrate) {
        pos = newtime * (id3->bitrate / 8);
    } else {
        return -1;
    }

    /* Don't seek right to the end of the file so that we can
       transition properly to the next song */
    if (pos >= (int)(id3->filesize - id3->id3v1len))
        pos = id3->filesize - id3->id3v1len - 1;

    /* id3->filesize excludes id3->first_frame_offset, so add it now */
    pos += id3->first_frame_offset;

    return pos;
}

static void set_elapsed(struct mp3entry* id3)
{
    unsigned long offset = id3->offset > id3->first_frame_offset ?
        id3->offset - id3->first_frame_offset : 0;
    unsigned long elapsed = id3->elapsed;

    if ( id3->vbr ) {
        if ( id3->has_toc ) {
            elapsed = toc_get_time_interpolated(offset);
        } else {
            /* No TOC, use an approximation (this'll be wildly inaccurate) */
            uint64_t data_size = id3->filesize -
                                 id3->first_frame_offset - id3->id3v1len;
            elapsed = (uint64_t)id3->length * offset / data_size;
        }
    }
    else
    {
        /* constant bitrate, use exact calculation */
        if (id3->bitrate != 0)
            elapsed = offset / (id3->bitrate / 8);
    }

    ci->set_elapsed(elapsed);
}

#ifdef MPA_SYNTH_ON_COP

/*
 * Run the synthesis filter on the COProcessor 
 */

static int mad_synth_thread_stack[DEFAULT_STACK_SIZE/sizeof(int)] IBSS_ATTR;

static const unsigned char * const mad_synth_thread_name = "mp3dec";
static unsigned int mad_synth_thread_id = 0;


static void mad_synth_thread(void)
{
    while(1) {
        ci->semaphore_release(&synth_done_sem);
        ci->semaphore_wait(&synth_pending_sem, TIMEOUT_BLOCK);
        
        if(die)
            break;

        mad_synth_frame(&synth, &frame);
    }    
}

/* wait for the synth thread to go idle which indicates a PCM frame has been
 * synthesized */
static inline void mad_synth_thread_wait_pcm(void)
{
    ci->semaphore_wait(&synth_done_sem, TIMEOUT_BLOCK);
}

/* increment the done semaphore - used after a wait for idle to preserve the
 * semaphore count */
static inline void mad_synth_thread_unwait_pcm(void)
{
    ci->semaphore_release(&synth_done_sem);
}

/* after synth thread has gone idle - switch decoded frames and commence
 * synthesis on it */
static void mad_synth_thread_ready(void)
{
    mad_fixed_t (*temp)[2][36][32];

    /*circular buffer that holds 2 frames' samples*/
    temp=frame.sbsample;
    frame.sbsample = frame.sbsample_prev;
    frame.sbsample_prev=temp;

    ci->semaphore_release(&synth_pending_sem);
}

static bool mad_synth_thread_create(void)
{
    ci->semaphore_init(&synth_done_sem, 1, 0);
    ci->semaphore_init(&synth_pending_sem, 1, 0);
       
    mad_synth_thread_id = ci->create_thread(mad_synth_thread, 
                            mad_synth_thread_stack,
                            sizeof(mad_synth_thread_stack), 0,
                            mad_synth_thread_name 
                            IF_PRIO(, PRIORITY_PLAYBACK)
                            IF_COP(, COP));
    
    if (mad_synth_thread_id == 0)
        return false;

    return true;
}

static void mad_synth_thread_quit(void)
{
    /* mop up COP thread */
    die = 1;
    ci->semaphore_release(&synth_pending_sem);
    ci->thread_wait(mad_synth_thread_id);
    ci->commit_discard_dcache();
}
#else
static inline void mad_synth_thread_ready(void)
{
     mad_synth_frame(&synth, &frame);
}

static inline bool mad_synth_thread_create(void)
{
    return true;
}

static inline void mad_synth_thread_quit(void)
{
}

static inline void mad_synth_thread_wait_pcm(void)
{
}

static inline void mad_synth_thread_unwait_pcm(void)
{
}
#endif /* MPA_SYNTH_ON_COP */

/* this is the codec entry point */
enum codec_status codec_main(enum codec_entry_call_reason reason)
{
    if (reason == CODEC_LOAD) {
        /* Create a decoder instance */
        if (codec_init())
            return CODEC_ERROR;

        ci->configure(DSP_SET_SAMPLE_DEPTH, MAD_F_FRACBITS);

        /* does nothing on 1 processor systems except return true */
        if(!mad_synth_thread_create())
            return CODEC_ERROR;
    }
    else if (reason == CODEC_UNLOAD) {
        /* mop up COP thread - MT only */
        mad_synth_thread_quit();
    }

    return CODEC_OK;
}

bool seek_by_time(int64_t* samplesdone, unsigned long current_frequency, unsigned long elapsed_ms)
{
    if (ci->id3->is_asf_stream) {
        asf_waveformatex_t *wfx = (asf_waveformatex_t *)(ci->id3->toc);
        int elapsedtime = asf_seek(elapsed_ms, wfx);

        *samplesdone = (elapsedtime > 0) ?
                       (((int64_t)elapsedtime)*current_frequency/1000) : 0;

        if (elapsedtime < 1) {
            ci->set_elapsed(0);
            return false;
        } else {
            ci->set_elapsed(elapsedtime);
            reset_stream_buffer();
        }
    } else {
        int newpos = elapsed_ms ? get_file_pos(elapsed_ms) : (int)(ci->id3->first_frame_offset);

        *samplesdone = ((int64_t)elapsed_ms) * current_frequency / 1000;

        if (!ci->seek_buffer(newpos))
            return false;

        ci->set_elapsed((*samplesdone * 1000LL) / current_frequency);
    }
    return true;
}

/* this is called for each file to process */
enum codec_status codec_run(void)
{
    size_t size;
    int file_end;
    int samples_to_skip; /* samples to skip in total for this file (at start) */
    char *inputbuffer;
    int64_t samplesdone;
    int stop_skip, start_skip;
    int current_stereo_mode = -1;
    unsigned long current_frequency = 0;
    int framelength;
    int padding = MAD_BUFFER_GUARD; /* to help mad decode the last frame */
    intptr_t param;

    /* Reinitializing seems to be necessary to avoid playback quircks when seeking. */
    init_mad();

    file_end = 0;

    ci->configure(DSP_SET_FREQUENCY, ci->id3->frequency);
    current_frequency = ci->id3->frequency;
    codec_set_replaygain(ci->id3);

     if (ci->id3->offset) {

        if (ci->id3->is_asf_stream) {
            asf_waveformatex_t *wfx = (asf_waveformatex_t *)(ci->id3->toc);
            int packet_offset = ((ci->id3->offset > ci->id3->first_frame_offset) ?
                                 (ci->id3->offset - ci->id3->first_frame_offset) : 0)
              % wfx->packet_size;
            ci->seek_buffer(ci->id3->offset - packet_offset);
            ci->id3->elapsed = asf_get_timestamp(&packet_offset);
            ci->set_elapsed(ci->id3->elapsed);
        }
        else {
            ci->seek_buffer(ci->id3->offset);
            set_elapsed(ci->id3);
        }
    }
    else if (ci->id3->elapsed)
         /* Have elapsed time but not offset */
        seek_by_time(&samplesdone, current_frequency, ci->id3->elapsed);
    else
        ci->seek_buffer(ci->id3->first_frame_offset);

    if (ci->id3->lead_trim >= 0 && ci->id3->tail_trim >= 0) {
        stop_skip = ci->id3->tail_trim - mpeg_latency[ci->id3->layer];
        if (stop_skip < 0) stop_skip = 0;
        start_skip = ci->id3->lead_trim + mpeg_latency[ci->id3->layer];
    } else {
        stop_skip = 0;
        /* We want to skip this amount anyway */
        start_skip = mpeg_latency[ci->id3->layer];
    }

    /* Libmad will not decode the last frame without 8 bytes of extra padding
       in the buffer. So, we can trick libmad into not decoding the last frame
       if we are to skip it entirely and then cut the appropriate samples from
       final frame that we did decode. Note, if all tags (ID3, APE) are not
       properly stripped from the end of the file, this trick will not work. */
    if (stop_skip >= mpeg_framesize[ci->id3->layer]) {
        padding = 0;
        stop_skip -= mpeg_framesize[ci->id3->layer];
    } else {
        padding = MAD_BUFFER_GUARD;
    }

    samplesdone = ((int64_t)ci->id3->elapsed) * current_frequency / 1000;

    /* Don't skip any samples unless we start at the beginning. */
    if (samplesdone > 0)
        samples_to_skip = 0;
    else
        samples_to_skip = start_skip;

    if (ci->id3->is_asf_stream)
        reset_stream_buffer();
    framelength = 0;

    /* This is the decoding loop. */
    while (1) {
        long action = ci->get_command(&param);

        if (action == CODEC_ACTION_HALT)
            break;

        if (action == CODEC_ACTION_SEEK_TIME) {
            /*make sure the synth thread is idle before seeking - MT only*/
            mad_synth_thread_wait_pcm();
            mad_synth_thread_unwait_pcm();

            if (param == 0) {
                samples_to_skip = start_skip;
            } else {
                samples_to_skip = 0;
            }

            bool success = seek_by_time(&samplesdone, current_frequency, param);
            ci->seek_complete();
            if (!success)
                break;

            init_mad();
            framelength = 0;
        }

        /* Lock buffers */
        if (stream.error == 0) {
            inputbuffer = get_stream_data(&size, INPUT_CHUNK_SIZE);
            if (size == 0 || inputbuffer == NULL) {
                if (ci->id3->is_asf_stream && stream_data_available < 0)
                    return CODEC_ERROR;
                break;
            }
            mad_stream_buffer(&stream, (unsigned char *)inputbuffer,
                              size + padding);
        }

        if (mad_frame_decode(&frame, &stream)) {
            if (stream.error == MAD_ERROR_BUFLEN) {
                /* This makes the codec support partially corrupted files */
                if (file_end == 30)
                    break;

                /* Fill the buffer */
                if (stream.next_frame)
                    advance_stream_buffer(stream.next_frame - stream.buffer);
                else
                    advance_stream_buffer(size);
                stream.error = 0; /* Must get new inputbuffer next time */
                file_end++;
                continue;
            } else if (MAD_RECOVERABLE(stream.error)) {
                /* Probably syncing after a seek */
                continue;
            } else {
                /* Some other unrecoverable error */
                return CODEC_ERROR;
            }
        }

        /* Do the pcmbuf insert here. Note, this is the PREVIOUS frame's pcm
           data (not the one just decoded above). When we exit the decoding
           loop we will need to process the final frame that was decoded. */
        mad_synth_thread_wait_pcm();

        if (framelength > 0) {
            
            /* In case of a mono file, the second array will be ignored. */
            ci->pcmbuf_insert(&synth.pcm.samples[0][samples_to_skip],
                              &synth.pcm.samples[1][samples_to_skip],
                              framelength);

            /* Only skip samples for the first frame added. */
            samples_to_skip = 0;
        }

        /* Initiate PCM synthesis on the COP (MT) or perform it here (ST) */
        mad_synth_thread_ready();

        /* Check if sample rate and stereo settings changed in this frame. */
        if (frame.header.samplerate != current_frequency) {
            current_frequency = frame.header.samplerate;
            ci->configure(DSP_SET_FREQUENCY, current_frequency);
        }
        if (MAD_NCHANNELS(&frame.header) == 2) {
            if (current_stereo_mode != STEREO_NONINTERLEAVED) {
                ci->configure(DSP_SET_STEREO_MODE, STEREO_NONINTERLEAVED);
                current_stereo_mode = STEREO_NONINTERLEAVED;
            }
        } else {
            if (current_stereo_mode != STEREO_MONO) {
                ci->configure(DSP_SET_STEREO_MODE, STEREO_MONO);
                current_stereo_mode = STEREO_MONO;
            }
        }

        if (stream.next_frame)
            advance_stream_buffer(stream.next_frame - stream.buffer);
        else
            advance_stream_buffer(size);
        stream.error = 0; /* Must get new inputbuffer next time */
        file_end = 0;

        framelength = synth.pcm.length - samples_to_skip;
        if (framelength < 0) {
            framelength = 0;
            samples_to_skip -= synth.pcm.length;
        }

        samplesdone += framelength;
        ci->set_elapsed((samplesdone * 1000LL) / current_frequency);
    }

    /* wait for synth idle - MT only*/
    mad_synth_thread_wait_pcm();
    mad_synth_thread_unwait_pcm();

    /* Finish the remaining decoded frame.
       Cut the required samples from the end. */
    if (framelength > stop_skip){
        ci->pcmbuf_insert(synth.pcm.samples[0], synth.pcm.samples[1],
                          framelength - stop_skip);
    }

    return CODEC_OK;
}
