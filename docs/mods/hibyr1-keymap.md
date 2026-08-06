### Keymap changes

The idea is to make Rockbox fully functional without a touchscreen (didn’t touch most plugins, only a few) and to add the Rewind button.

Essentially, we have 4 navigation keys + the Power button. Pretty limited.  
So consider the following mapping:

**In Lists/Menus:**
- Power → Playback resume button. Works as Cancel in menus
- Long Power → Stops playback
- Vol Up/Down → Up/Down in list
- Play/Next → Left/Right. Play (Left) works as Back, Next (Right) as Select
- Long Play → Main menu (see it as long Back)
- Long Next → Context menu (see it as long Select)

**Combos:**
- Power + Vol Up → Lock
- Power + Vol Down → Quickscreen
- Next + Vol Up/Down → Tree scrolling (to see what is it: try Next + Vol Down on long filename in File/DB Browser)

**In WPS (While Playing Screen):**
- Power → Play/Pause
- Long Power → WPS context menu (Stops playback in locked state)
- Vol Up → Browse (Vol+ in locked state)
- Long Vol Up → Volume Up
- Vol Down → WPS Hotkey (View Playlist by default) (Vol– in locked state)
- Long Vol Down → Volume Down
- Play → Prev track/Rewind
- Next → Next track/ffwd

**Combos:**
- Power + Vol Up → Lock
- Power + Vol Down → Quickscreen (Long Power in Quickscreen opens Shortcuts)
- Power + Play → Main menu
- Power + Next → Pitchscreen

**Notes:**
- I had to sacrifice single presses of the volume buttons in WPS (but they still work after Lock). So use long presses for volume changes.
- Quickest way to stop playback — long press Power twice (the first opens the Context Menu, the second stops playback).
- If you just want to turn on the screen, press Power + Vol Up (Lock combo).
- In Pitchscreen, use Long Power to change mode (or tap right OK on screen).
- In most non‑WPS contexts, OK = Next and Cancel = Play. This takes some getting used to. If Left/Right buttons are required in such contexts, try Power + Play/Next (one example - keyboard).
