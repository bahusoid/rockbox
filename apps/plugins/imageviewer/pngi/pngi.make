#             __________               __   ___.
#   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
#   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
#   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
#   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
#                     \/            \/     \/    \/            \/
# $Id$
#

PNGISRCDIR := $(IMGVSRCDIR)/pngi
PNGIBUILDDIR := $(IMGVBUILDDIR)/pngi

PNGI_SRC := $(call preprocess, $(PNGISRCDIR)/SOURCES)
PNGI_OBJ := $(call c2obj, $(PNGI_SRC))

OTHER_SRC += $(PNGI_SRC)

ROCKS += $(PNGIBUILDDIR)/pngi.ovl

$(PNGIBUILDDIR)/pngi.refmap: $(PNGI_OBJ)
$(PNGIBUILDDIR)/pngi.link: $(PLUGIN_LDS) $(PNGIBUILDDIR)/pngi.refmap
$(PNGIBUILDDIR)/pngi.ovl: $(PNGI_OBJ)

PNGIFLAGS = $(IMGDECFLAGS)
ifndef DEBUG
PNGIFLAGS += -Os
endif

# Compile plugin with extra flags (adapted from ZXBox)
$(PNGIBUILDDIR)/%.o: $(PNGISRCDIR)/%.c $(PNGISRCDIR)/pngi.make
	$(SILENT)mkdir -p $(dir $@)
	$(call PRINTS,CC $(subst $(ROOTDIR)/,,$<))$(CC) -I$(dir $<) $(PNGIFLAGS) -c $< -o $@
