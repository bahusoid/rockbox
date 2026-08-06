Bluetooth support for hosted hiby Rockbox build

Includes bidhata patches (https://github.com/bidhata/hiby-r1-rockbox-bt) for bluetooth support. With the following improvements:
- Removed slow and glitchy best codec selection code (let firmware handle it);
- Replaced 'sys_server' (hiby's own service) bt control with standard 'bluetoothctl' utility. 
Makes connection and devices discovery much more reliable for me.
- Added BT buttons support. If buttons doesn't work - just reconnect the device (can be done without disconnection. Just press on the device in Devices list or in Status menu)
- Ability to control BT in Status menu:
Bluetooth: On/Off/Suspend Bluetooth (Suspend is useful for saving battery when not using BT)
Device:  Reconnection (Long press to disconnect)
Codec: Switching codec (Note: doesn't work reliable with active playback)
- Handles autoconnection (If paired device is connected automatically - no need to connect it again from Devices list)
- Some other minor improvements and fixes.

IMPORTANT NOTES: 
* Default rockbox bootloader kills all BT services and unloads BT drivers on boot (via bt_suspend script).
So if you want to avoid waiting/seeing "Bluetooth is suspended. Resuming may take some time..." you need to install modified bootloader that keeps BT on.
My bootloader supports both modes (keeps BT susppended/enabled across reboots depending on settings).

* If BT glitches try to Suspend and enable it again.

* Rockbox boots pretty fast and bluetooth is not yet fully initialized and needs some time (so opening BT menus might feel laggy right after boot)

* Switching off bt headphones during active playback will crash rockbox (so don't do it). PCM threading code needs some looking into. I've not touched it yet.

* Long press on device in Devices list to remove it from paired list.

* Quickest way to connect BT go right to Bluetooth -> Devices. If device is already paired - in most cases it will connect automatically.
If not - just select the device and press OK to connect.


