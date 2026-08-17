
#define HIBY_LINUX_X1600

#if !defined(BOOTLOADER) && !defined(SIMULATOR)
#define HAVE_HOSTED_NETLINK_MONITOR
#endif
/* USB */
#define HAVE_USB_ADB
#define HAVE_HOST_USB_AUDIO
#define HAVE_USB_POWER


#include "hibylinux.h"