#include <fcntl.h>
#include <unistd.h>
#include <poll.h>
#include <stdio.h>
#include <linux/input.h>
#include <stdlib.h>

// Keep file descriptors open globally or within the thread struct
static int fd_hs = -1;
static int fd_bal = -1;
static int fd_lin = -1;
static struct pollfd poll_fds[3];
enum OUTPUT_IDX {
    OUTPUT_HEADSET,
    OUTPUT_BALANCED,
    OUTPUT_LINEOUT,
};

void init_headphone_detection(void) {
    // Open once and keep open
    fd_hs = open("/sys/class/switch/headset/state", O_RDONLY | O_NONBLOCK);
    fd_bal = open("/sys/class/switch/balance/state", O_RDONLY | O_NONBLOCK);
    fd_lin = open("/sys/class/switch/lineout/state", O_RDONLY | O_NONBLOCK);
  
    for (int i = 0; i < sizeof(poll_fds) / sizeof(poll_fds[0]); i++) {
        if (poll_fds[i].fd >= 0) 
            poll_fds[i].events = POLLPRI | POLLERR;
    }
    poll_fds[OUTPUT_HEADSET].fd = fd_hs;
    poll_fds[OUTPUT_HEADSET].events = POLLPRI | POLLERR; 
    
    poll_fds[OUTPUT_BALANCED].fd = fd_bal;
    poll_fds[OUTPUT_BALANCED].events = POLLPRI | POLLERR;

    poll_fds[OUTPUT_LINEOUT].fd = fd_lin;
    poll_fds[OUTPUT_LINEOUT].events = POLLPRI | POLLERR;
    // Read them once initially to clear any pending events
    // (Implementation of a read_switch() helper is in Approach 2)
}

// Run this inside your dedicated hardware monitoring thread
void headphone_monitor_loop(void) {

    while (1) {
        // Block indefinitely until the kernel signals a state change (-1 timeout)
        int ret = poll(poll_fds, 2, -1);
        
        if (ret > 0) {
            // A change happened! 
            // You MUST lseek to 0 and read the file to clear the POLLPRI state.
            
            if (poll_fds[0].revents & POLLPRI) {
                // Headset changed. Seek to start, read value, update state.
                // handle_state_change(fd_hs);
            }
            if (poll_fds[1].revents & POLLPRI) {
                // Balanced output changed.
                // handle_state_change(fd_bal);
            }
            
            // Trigger your UI update or audio routing logic here
        }
    }
}
int button_read_device()
{
    struct input_event event;
    /* check if there are any events pending and process them */
    while(poll(poll_fds, num_devices, 0)) {
        for(int i = 0; i < num_devices; i++) {
            /* read only if non-blocking */
            if(poll_fds[i].revents & POLLIN) {
                int size = read(poll_fds[i].fd, &event, sizeof(event));
                if(size == (int)sizeof(event)) {
                    switch(event.type) {
                    case EV_KEY: {
                        /* map linux event code to rockbox button bitmap */
                        int bmap = button_map(event.code);

                        /* event.value == 0x10000 means press
                         * event.value == 0 means release
                         */
                        if(event.value) {
#ifdef HAVE_SCROLLWHEEL
                            /* Filter out wheel ticks */
                            if (bmap & BUTTON_SCROLL_BACK)
                                wheel_ticks--;
                            else if (bmap & BUTTON_SCROLL_FWD)
                                wheel_ticks++;
                            bmap &= ~(BUTTON_SCROLL_BACK|BUTTON_SCROLL_FWD);
#endif
#ifdef BUTTON_DELAY_RELEASE
                            bmap &= ~BUTTON_DELAY_RELEASE;
#endif
#if defined(HAVE_TOUCHSCREEN) && defined(BUTTON_TOUCH)
                            /* Some touchscreens give us actual touch/untouch as a "key" */
                            if (bmap & BUTTON_TOUCH) {
                                handle_touchscreen_event(ABS_FAKE_PRESSED, EVENT_VALUE_TOUCHSCREEN_PRESS);
                                bmap &= ~BUTTON_TOUCH;
                            }
#endif
                            button_bitmap |= bmap;
                        } else {
#if defined(HAVE_TOUCHSCREEN) && defined(BUTTON_TOUCH)
                            /* Some touchscreens give us actual touch/untouch as a "key" */
                            if (bmap & BUTTON_TOUCH) {
                                handle_touchscreen_event(ABS_FAKE_PRESSED, EVENT_VALUE_TOUCHSCREEN_RELEASE);
                                bmap &= ~BUTTON_TOUCH;
                            }
#endif
#ifdef BUTTON_DELAY_RELEASE
                            /* Delay the release of any requested buttons */
                            if (bmap & BUTTON_DELAY_RELEASE) {
                                button_delay_release |= bmap & ~BUTTON_DELAY_RELEASE;
                                delay_tick = current_tick + HZ/20;
                                bmap = 0;
                            }
#endif
#ifdef HAVE_SCROLLWHEEL
                            /* Wheel gives us press+release back to back; ignore the release */
                            bmap &= ~(BUTTON_SCROLL_BACK|BUTTON_SCROLL_FWD);
#endif
                            button_bitmap &= ~bmap;
                        }
                        break;
                    }
#ifdef HAVE_TOUCHSCREEN
                    case EV_ABS: {
                        if (ts_enabled) {
                            handle_touchscreen_event(event.code, event.value);
                        } else {
                             /* If disabled... ignore */
                            _last_touch_state = TOUCHSCREEN_STATE_UNKNOWN;
                        }
                        break;
                    }
#endif
                    default:
                        /* Ignore other event types */
                        break;
                    }
                }
            }
            /* device was removed/disconnected — close it to stop poll returning POLLHUP forever */
            else if (poll_fds[i].revents & (POLLERR | POLLHUP)) {
                button_remove_input_device(i);
            }
        }
    }

#ifdef HAVE_SCROLLWHEEL
    /* Reset backlight and poweroff timers */
    if (wheel_ticks) {
#ifdef HAVE_BACKLIGHT
        backlight_on();
#endif
#ifdef HAVE_BUTTON_LIGHT
        buttonlight_on();
#endif
        reset_poweroff_timer();
    }

    if (wheel_ticks > 0)
    {
        while (wheel_ticks-- > 0)
        {
            button_queue_post(BUTTON_SCROLL_FWD, 0);
        }
    }
    else if (wheel_ticks < 0)
    {
        while (wheel_ticks++ < 0)
        {
            button_queue_post(BUTTON_SCROLL_BACK, 0);
        }
    }
#endif /* HAVE_SCROLLWHEEL */

#ifdef HAVE_TOUCHSCREEN
    int touch = touchscreen_to_pixels(_last_x, _last_y, data);

    if(_last_touch_state == TOUCHSCREEN_STATE_DOWN)
    {
        return button_bitmap | touch;
    }
#endif

    return button_bitmap;
}