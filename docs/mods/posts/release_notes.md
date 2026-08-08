Hiby R1/R3 PRO II Rockbox Mod 26.08.08

## Features and Improvements
* **[Hiby R1 Touchless Navigation](https://github.com/bahusoid/rockbox/tree/Mod25.12.07/docs/mods/hibyr1-keymap.md):** Implemented a custom keymap enabling full Rockbox navigation without the touchscreen, added **Prev/Rewind** button.
* **[Audiobooks](https://github.com/bahusoid/rockbox/tree/Mod25.12.07/docs/mods/audiobook-mod.md):** Included a specialized audiobook settings profile (`audiobook-mod-config.cfg`) with 30-second skips, auto-bookmarking, and M4B chapter support (via CUE generation).
* **[Bluetooth](https://github.com/bahusoid/rockbox/tree/Mod25.12.07/docs/mods/bluetooth-hiby-x1600.md):** Integrated bidhata's [patches](https://github.com/bidhata/hiby-r1-rockbox-bt) with major improvements:
  * Replaced `sys_server` with `bluetoothctl` for more reliable connection handling.
  * Added support for Bluetooth headset media buttons.
  * Added direct Bluetooth management in the Status menu (Enable/Disable, Codec switching).
* **[Touch Controls](https://github.com/bahusoid/rockbox/tree/Mod25.12.07/docs/mods/touch-controls.md):** Included gestures, swipes, and kinetic scrolling by [amachronic](https://github.com/amachronic).
* **[Bootloader](https://github.com/bahusoid/rockbox/tree/Mod25.12.07/docs/mods/hiby-bootloader-changes.md):** Modified Rockbox bootloader to persist Bluetooth state across reboots and adjusted button mappings for Hiby R1.
* **PictureFlow:** Added support for embedded album art.
* **ImageViewer:** Tries to display JPG images in fullscreen and supports viewing embedded album art in ogg/opus files.
* Includes other useful community patches like [USB DAC](https://gerrit.rockbox.org/r/c/rockbox/+/7674) and [Touchscreen keyboard](https://gerrit.rockbox.org/r/c/rockbox/+/7695) by [Michael McAllister](https://github.com/michaelmcallister).
* Big bunch of other fixes and improvements (see full list of changes [here](https://github.com/Rockbox/rockbox/compare/master...bahusoid:rockbox:Mod25.12.07)).


**⚠️ NOTE:** The HiBy R3 Pro II build is currently untested. Please install and use it at your own risk.