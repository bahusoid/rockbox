#             __________               __   ___.
#   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
#   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
#   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
#   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
#                     \/            \/     \/    \/            \/
# $Id$
#

JPEGISRCDIR := $(IMGVSRCDIR)/jpegi
JPEGIBUILDDIR := $(IMGVBUILDDIR)/jpegi

JPEGI_SRC := $(call preprocess, $(JPEGISRCDIR)/SOURCES)
JPEGI_OBJ := $(call c2obj, $(JPEGI_SRC))

OTHER_SRC += $(JPEGI_SRC)

ROCKS += $(JPEGIBUILDDIR)/jpegi.ovl

$(JPEGIBUILDDIR)/jpegi.refmap: $(JPEGI_OBJ)
$(JPEGIBUILDDIR)/jpegi.link: $(PLUGIN_LDS) $(JPEGIBUILDDIR)/jpegi.refmap
$(JPEGIBUILDDIR)/jpegi.ovl: $(JPEGI_OBJ)

JPEGIFLAGS = $(IMGDECFLAGS)
ifndef DEBUG
JPEGIFLAGS += -Os
endif

# Compile plugin with extra flags (adapted from ZXBox)
$(JPEGIBUILDDIR)/%.o: $(JPEGISRCDIR)/%.c $(JPEGISRCDIR)/jpegi.make
	$(SILENT)mkdir -p $(dir $@)
	$(call PRINTS,CC $(subst $(ROOTDIR)/,,$<))$(CC) -I$(dir $<) $(JPEGIFLAGS) -c $< -o $@
