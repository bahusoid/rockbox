# Bluetooth Support for Hosted Hiby Rockbox Build

This build integrates [bidhata's patches](https://github.com/bidhata/hiby-r1-rockbox-bt) to enable Bluetooth functionality, alongside several stability and usability improvements.

## 🛠️ Key Improvements

*   **Standardized Bluetooth Control:** Replaced Hiby's proprietary `sys_server` control with the standard `bluetoothctl` utility. This makes device discovery and connection significantly more reliable.
*   Removed the slow and glitchy "best codec" selection logic, offloading this responsibility directly to the firmware.
*   **Media Button Support:** Added support for Bluetooth hardware buttons.
    *   *Tip:* If the buttons don't work, simply re-initialize the connection by selecting the device in the **Devices** list or **Status** menu (no need to fully disconnect first).
*   **Bluetooth Control:** You can now manage Bluetooth directly from the Status menu:
    *   **Bluetooth:** Toggle On / Off / Suspend *(Suspend kills all BT services, while Off just powers off the BT adapter)*.
    *   **Device:** Press to reconnect headphones; long-press to disconnect.
    *   **Codec:** Manually switch audio codecs *(Note: This is currently unstable during active playback).*
*   Various under-the-hood fixes and improvements.

---

## ⚠️ Important Notes & Known Issues

*   **Custom Bootloader Recommended:** The default Rockbox bootloader runs a `bt_suspend` script on boot, killing all BT services and unloading the drivers. This results in a *"Bluetooth is suspended. Resuming may take some time..."* waiting screen. To avoid this, install my modified bootloader, which persists your chosen BT state (enabled or suspended) across reboots.
*   **CRASH WARNING:** Do not switch off your Bluetooth headphones during active playback. This will currently crash Rockbox. The PCM threading code needs some looking into.
*   **Troubleshooting Glitches:** If Bluetooth begins to behave erratically, simply Suspend and re-enable it.
*   **Unpairing Devices:** To remove a paired device, long-press its name in the **Devices** list.
*   **Quick Connect:** The fastest way to connect is to navigate straight to `Bluetooth -> Devices`. There if the device is already ON and paired, it will usually connect automatically. 