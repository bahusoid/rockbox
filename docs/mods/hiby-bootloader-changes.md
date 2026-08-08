# Bootloader Changes

## 1. Modified Keymap for Hiby R1
Button controls are aligned with my [Rockbox keymap](hibyr1-keymap.md).
* **Vol Up/Down** - Move Up / Down
* **Play** - Go Back
* **Next/Power** - Select

## 2. Bluetooth State Persistence
Bluetooth state is preserved across Rockbox reboots (checks for `.rockbox/rb_bt_on.txt` file presence).
