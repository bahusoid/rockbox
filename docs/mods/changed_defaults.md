# Changed default settings

This document details the default setting changes introduced by commit (`settings: change defaults`).
To revert to the official Rockbox default settings, use [restore_official_rb_settings.cfg](configs/restore_official_rb_settings.cfg).

## Settings -> Theme Settings -> Line Separator
Changed from Off to Auto on touchscreen builds.

Old setting value:
list separator height: off

New setting value:
list separator height: auto

## Settings -> Theme Settings -> Status-/Scrollbar -> Battery Display
Changed from Graphic to Numeric.

Old setting value:
battery display: graphic

New setting value:
battery display: numeric

## Settings -> General Settings -> System -> Advanced Key Lock -> Enabled
Changed from No to Yes.

Old setting value:
No Screen Lock For Selected Actions: off

New setting value:
No Screen Lock For Selected Actions: on

## Settings -> General Settings -> System -> Advanced Key Lock -> Settings
Changed from no actions selected to Exempt Volume, Exempt Play, Exempt Seek, Exempt Skip, Start/Stop, Disable Locked Reminders, and Disable All Lock Notifications.

Old setting value:
Selective Screen Lock Actions: 0

New setting value:
Selective Screen Lock Actions: 4655

## Settings -> General Settings -> File View -> Show Filename Extensions
Changed from 'Only When Viewing All Types' to 'On'.

Old setting value:
show filename exts: view all

New setting value:
show filename exts: on

## Settings -> General Settings -> File View -> Follow Playlist
Changed from No to Yes.

Old setting value:
follow playlist: off

New setting value:
follow playlist: on

## Settings -> General Settings -> Bookmarking -> Maintain a List of Recent Bookmarks
Changed from 'No' to 'One per playlist'.

Old setting value:
use most-recent-bookmarks: off

New setting value:
use most-recent-bookmarks: unique only

## Settings -> General Settings -> Alt Settings -> Alt Bookmark On Stop
Changed from No to Yes.

Old setting value:
alt autocreate bookmarks: off

New setting value:
alt autocreate bookmarks: on

## Settings -> General Settings -> Alt Settings
Changed from No to Yes. Path is set to '/ABooks:/Audiobooks'.

Old setting value:
alt settings enable: off
alt settings paths: ""

New setting value:
alt settings enable: on
alt settings paths: /ABooks:/Audiobooks

## Settings -> General Settings -> Alt Settings -> Reset Pitch
Changed from No to Yes.

Old setting value:
alt reset pitch: off

New setting value:
alt reset pitch: on

## Settings -> General Settings -> Alt Settings -> Alt Skip Length
Changed from 'Skip Track' to '30 s'.

Old setting value:
alt skip length: 0

New setting value:
alt skip length: 30

## Settings -> Sound Settings -> Timestretch Enabled
Changed from No to Yes.

Old setting value:
timestretch enabled: off

New setting value:
timestretch enabled: on

## Settings -> General Settings -> Display -> LCD -> Backlight On Button Hold
Changed from Normal to Off.

Old setting value:
backlight on button hold: normal

New setting value:
backlight on button hold: off

## Settings -> Playback Settings -> Pause on Headphone Unplug
Changed from Off to Pause.

Old setting value:
pause on headphone unplug: off

New setting value:
pause on headphone unplug: pause

## Settings -> Playback Settings -> Disable resume on startup if phones unplugged
Changed from No to Yes.

Old setting value:
disable autoresume if phones not present: off

New setting value:
disable autoresume if phones not present: on

## Settings -> Playback Settings -> Cuesheet Support
Changed from No to Yes.

Old setting value:
cuesheet support: off

New setting value:
cuesheet support: on

## Settings -> Playback Settings -> Rewind Across Tracks
Changed from No to Yes.

Old setting value:
rewind across tracks: off

New setting value:
rewind across tracks: on

## Settings -> General Settings -> Startup/Shutdown -> Keypress Restarts Sleeptimer
Changed from No to Yes.

Old setting value:
keypress restarts sleeptimer: off

New setting value:
keypress restarts sleeptimer: on

## Quickscreen Top item
Changed from unset to Alt Skip Length.

Old setting value:
qs top: -

New setting value:
qs top: alt skip length

## Quickscreen Bottom item
Changed from unset to Alt Skip Length.

Old setting value:
qs bottom: -

New setting value:
qs bottom: alt skip length

## Pitch screen (Show screen with Speed by default)
Changed from Off to On.

Old setting value:
Timestretch mode: off

New setting value:
Timestretch mode: on