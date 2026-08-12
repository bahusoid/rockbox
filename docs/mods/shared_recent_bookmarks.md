### Cross-Device Playback Resume (via shared Recent Bookmarks)

Eject the SD card from one player, insert it into another, and seamlessly resume playback exactly from where you left off.

**Prerequisites**
Two or more players running Rockbox from the same SD card. This usually requires at least one of the players to support multiboot (the ability to place the `.rockbox` installation in a subfolder). For more details, see the [Multiboot Bootloader Wiki](https://www.rockbox.org/wiki/MultibootBootloader.html).

**Limitations**
* As with bookmarks, this feature does not support dynamic playlists or other non-bookmarkable states.

**Setup Instructions**
To make this work, all players must be configured to read and write recent bookmarks to the same location.

1. **Synchronize the Playlists Folder**
   On each player, ensure your Playlists folder (accessed via **Main Menu > Playlists**) targets the exact same directory.
   * If you don't actively use playlists, create a dedicated folder on the SD card to share recent bookmarks.
   * On all of your devices, navigate to this folder, open the context menu, and select **Set as... > Playlists Folder**.
   * *Note: This feature reuses the configured Playlists directory to store recent bookmarks, avoiding the need for an additional dedicated setting in the UI.*

2. **Configure Bookmark Settings**
   On each player, navigate to **Settings > General Settings > Bookmarks** and configure the following options:
   * **Maintain a List of Recent Bookmarks:** Set to anything except *No*. (Setting this to *One per playlist* will use less storage data).
   * **Bookmark on Stop:** Set to anything except *No*. To prevent the creation of standard folder bookmarks and only use this cross-device feature, select *Yes - Recent only*.
   * **Resume Recent Bookmark:** Set to *Yes*.