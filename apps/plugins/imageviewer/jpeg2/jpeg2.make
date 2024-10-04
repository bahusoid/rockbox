#             __________               __   ___.
#   Open      \______   \ ____   ____ |  | _\_ |__   _______  ___
#   Source     |       _//  _ \_/ ___\|  |/ /| __ \ /  _ \  \/  /
#   Jukebox    |    |   (  <_> )  \___|    < | \_\ (  <_> > <  <
#   Firmware   |____|_  /\____/ \___  >__|_ \|___  /\____/__/\_ \
#                     \/            \/     \/    \/            \/
# $Id$
#

JPEG2SRCDIR := $(IMGVSRCDIR)/jpeg2
JPEG2BUILDDIR := $(IMGVBUILDDIR)/jpeg2

JPEG2_SRC := $(call preprocess, $(JPEG2SRCDIR)/SOURCES)
JPEG2_OBJ := $(call c2obj, $(JPEG2_SRC))

OTHER_SRC += $(JPEG2_SRC)

ROCKS += $(JPEG2BUILDDIR)/jpeg2.ovl

$(JPEG2BUILDDIR)/jpeg2.refmap: $(JPEG2_OBJ) $(TLSFLIB)
$(JPEG2BUILDDIR)/jpeg2.link: $(PLUGIN_LDS) $(JPEG2BUILDDIR)/jpeg2.refmap
$(JPEG2BUILDDIR)/jpeg2.ovl: $(JPEG2_OBJ) $(TLSFLIB)

#-Os breaks decoder - dunno why
JPEG2FLAGS = $(IMGDECFLAGS) 
#-O2

# Compile PNG plugin with extra flags (adapted from ZXBox)
$(JPEG2BUILDDIR)/%.o: $(JPEG2SRCDIR)/%.c $(JPEG2SRCDIR)/jpeg2.make
	$(SILENT)mkdir -p $(dir $@)
	$(call PRINTS,CC $(subst $(ROOTDIR)/,,$<))$(CC) -I$(dir $<) $(JPEG2FLAGS) -c $< -o $@
