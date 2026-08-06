### Keymap Changes: Touchless Rockbox Navigation

The goal of this keymap is to make Rockbox fully functional without a touchscreen (applying to the core and a few key plugins) and add a **Rewind** button.

Essentially, we have 4 navigation keys + the Power button. Pretty limited.  
So consider the following mapping:

#### 1. In Lists and Menus

| Button / Combo | Action |
| :--- | :--- |
| **Power** | Resume Playback *(Acts as **Cancel** in menus)* |
| **Long Power** | Stop playback |
| **Vol Up / Vol Down** | Move Up / Down in list |
| **Play** (Left) | Go Back |
| **Long Play** | Main Menu *(Think of it as a "Long Back")* |
| **Next** (Right) | Select |
| **Long Next** | Context Menu *(Think of it as a "Long Select")* |
| `Power` + `Vol Up` | **Lock** device |
| `Power` + `Vol Down` | **Quickscreen** |
| `Next` + `Vol Up/Down` | **Tree scrolling** *(To test: try Next + Vol Down on a long filename in the File/DB Browser)* |

---

#### 2. While Playing Screen (WPS)
This keymap restores standard media controls while keeping essential Rockbox menus accessible.

| Button / Combo | Action                                                                               |
| :--- |:-------------------------------------------------------------------------------------|
| **Power** | Play / Pause                                                                         |
| **Long Power** | WPS Context Menu *(Stops playback if the device is locked)*                          |
| **Play** | Previous Track / Rewind                                                              |
| **Next** | Next Track / Fast Forward                                                            |
| **Vol Up** | Browse *(Acts as Volume Up if the device is locked)*                                 |
| **Long Vol Up** | Volume Up                                                                            |
| **Vol Down** | WPS Hotkey (View current Playlist/Cue by default ) *(Acts as Volume Down if locked)* |
| **Long Vol Down** | Volume Down                                                                          |
| `Power` + `Vol Up` | **Lock** device                                                                      |
| `Power` + `Vol Down` | **Quickscreen** *(Note: Long Power inside Quickscreen opens Shortcuts)*              |
| `Power` + `Play` | **Main Menu**                                                                        |
| `Power` + `Next` | **Pitchscreen**                                                                      |

---

#### Notes 

- I had to sacrifice single presses of the volume buttons in WPS (but they still work after Lock). So use long presses for volume changes.
- Quickest way to stop playback — long press Power twice (the first opens the Context Menu, the second stops playback).
- If you just want to turn on the screen, press Power + Vol Up (Lock combo).
- In Pitchscreen, use Long Power to change mode (or tap right OK on screen).
- In most non‑WPS contexts, OK = Next and Cancel = Play. This takes some getting used to. If Left/Right buttons are required in such contexts, try Power + Play/Next (one example - keyboard).
