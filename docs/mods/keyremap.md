# Key remapping on the Hiby R1

Rockbox supports per-device key remapping through the built-in `keyremap` plugin.

## Quick setup

1. Place a prepared keymap text file onto the player's SD card (for example, in `/keymaps`).
2. In Rockbox, navigate to `Plugins -> Applications -> keyremap`.
3. Choose `Import Text Keymap`, select your file, and then select `Set Core Remap`.

*Note: To remove a remap, use `Remove Core Remap` within the same plugin.*

## Sample keymaps

A few example remaps are included in the [`keyremaps`](keyremaps/) folder:

- [hibyr1-official-rockbox-wps.txt](keyremaps/hibyr1-official-rockbox-wps.txt)  
  Restores the official Rockbox Hiby R1 keymap for the WPS (While Playing Screen).

- [hibyr1-short-power-lock.txt](keyremaps/hibyr1-short-power-lock.txt)  
  A short press of `Power` locks the device; a long press of `Power` handles play/pause in the WPS and stops playback in lists/menus.

- [hibyr1-long-power-lock.txt](keyremaps/hibyr1-long-power-lock.txt)  
  A long press of `Power` locks the device.

The built-in source keymap can be found at [`apps/keymaps/keymap-hibyr1.c`](../../apps/keymaps/keymap-hibyr1.c).

## Manual remapping format

The file uses a simplified C-like syntax. Each context block contains one or more mappings:

```c
CONTEXT_WPS = {
    {ACTION_WPS_PLAY, BUTTON_LEFT | BUTTON_REL, BUTTON_LEFT},
    {ACTION_WPS_STOP, BUTTON_POWER | BUTTON_REPEAT, BUTTON_POWER},
    {ACTION_WPS_VOLUP, BUTTON_UP | BUTTON_REL, BUTTON_UP},
    {ACTION_WPS_VOLDOWN, BUTTON_DOWN | BUTTON_REL, BUTTON_DOWN},
    {ACTION_WPS_SKIPNEXT, BUTTON_RIGHT | BUTTON_REL, BUTTON_RIGHT},
    {ACTION_WPS_SEEKFWD, BUTTON_RIGHT | BUTTON_REPEAT, BUTTON_NONE},
}
```

Each line follows this exact structure:

```c
{ACTION_NAME, BUTTON_COMBO, PRE_BUTTON_CONDITION},
```

- `ACTION_NAME`: The Rockbox action to trigger (e.g., `ACTION_STD_OK`, `ACTION_WPS_PLAY`).
- `BUTTON_COMBO`: The button or combination of buttons that triggers the action.
- `PRE_BUTTON_CONDITION`: The previous button state requirement. Use `BUTTON_NONE` if there is no prerequisite. (See the lifecycle section below for why this is necessary).

The parser ignores whitespace and accepts `#` for comments:

```c
# Lock device on power release
CONTEXT_STD = {
    {ACTION_STD_KEYLOCK, BUTTON_POWER | BUTTON_REL, BUTTON_POWER},
}
```

## Available buttons

For the Hiby R1, the button names are defined in `firmware/target/hosted/hiby/r1/button-target.h`:

| Button Name | Physical Key on Hiby R1 |
| :--- | :--- |
| `BUTTON_POWER` | Power button |
| `BUTTON_RIGHT` | Next button |
| `BUTTON_LEFT` | Play / Pause button |
| `BUTTON_UP` | Volume Up button |
| `BUTTON_DOWN` | Volume Down button |

These can be combined with event modifier flags using a pipe `|`:

- `BUTTON_REL`: Released event
- `BUTTON_REPEAT`: Long-press hold event

Examples:
- `BUTTON_POWER | BUTTON_REL`: Power button released.
- `BUTTON_POWER | BUTTON_REPEAT`: Power button held down.
- `BUTTON_POWER | BUTTON_UP`: Power and Volume Up pressed simultaneously.

### Button press lifecycle & PRE_BUTTON_CONDITION

To understand when to use `PRE_BUTTON_CONDITION`, look at the event sequence Rockbox sees when a generic button (`BUTTON_X`) is pressed:

**Short press sequence:**
1. `BUTTON_X` (Initial press)
2. `BUTTON_X | BUTTON_REL` (Release)

**Long press sequence:**
1. `BUTTON_X` (Initial press)
2. `BUTTON_X | BUTTON_REPEAT` (Hold threshold reached)
3. `BUTTON_X | BUTTON_REL` (Release after hold)

Because the `BUTTON_X | BUTTON_REL` event occurs at the end of *both* short and long presses, you must use `PRE_BUTTON_CONDITION` to differentiate them:

- To trigger an action **only on a short press release**:
  `{ACTION_STD_OK, BUTTON_POWER | BUTTON_REL, BUTTON_POWER},`

- To trigger an action **only on a long press release**:
  `{ACTION_STD_KEYLOCK, BUTTON_POWER | BUTTON_REL, BUTTON_POWER | BUTTON_REPEAT},`

- Multi-button combinations typically ignore previous conditions:
  `{ACTION_STD_KEYLOCK, BUTTON_POWER | BUTTON_UP, BUTTON_NONE},`

## Available contexts

The main contexts used for remapping are defined in `apps/action.h`:

- `CONTEXT_STD`: Standard / fallback context
- `CONTEXT_WPS`: While Playing Screen
- `CONTEXT_TREE`: File and Database browser
- `CONTEXT_MAINMENU`: Main menu
- `CONTEXT_LIST`: Lists and menus

The plugin also exposes locked variants so keys can behave differently when the device is locked:

- `CONTEXT_WPS_LOCKED`
- `CONTEXT_STD_LOCKED`
- `CONTEXT_TREE_LOCKED`

## Available actions

The full list is in `apps/action.h`. The most commonly remapped actions are:

### Standard actions
`ACTION_STD_PREV`, `ACTION_STD_PREVREPEAT`, `ACTION_STD_NEXT`, `ACTION_STD_NEXTREPEAT`, `ACTION_STD_OK`, `ACTION_STD_CANCEL`, `ACTION_STD_CONTEXT`, `ACTION_STD_MENU`, `ACTION_STD_QUICKSCREEN`, `ACTION_STD_KEYLOCK`, `ACTION_STD_HOTKEY`

### WPS actions
`ACTION_WPS_PLAY`, `ACTION_WPS_STOP`, `ACTION_WPS_BROWSE`, `ACTION_WPS_SKIPNEXT`, `ACTION_WPS_SKIPPREV`, `ACTION_WPS_SEEKFWD`, `ACTION_WPS_SEEKBACK`, `ACTION_WPS_STOPSEEK`, `ACTION_WPS_VOLUP`, `ACTION_WPS_VOLDOWN`, `ACTION_WPS_CONTEXT`, `ACTION_WPS_MENU`, `ACTION_WPS_QUICKSCREEN`, `ACTION_WPS_HOTKEY`

### List / Tree actions
`ACTION_LIST_VOLUP`, `ACTION_LIST_VOLDOWN`, `ACTION_TREE_STOP`, `ACTION_TREE_WPS`

### Special action
`ACTION_NONE`: Does nothing (useful for disabling a button in a specific context).

## Example: Changing volume in lists instead of scrolling

In most cases, you do not need to invent a custom keymap from scratch. Start from the original mapping, find the action you do not want, and replace it with the one you want.

For instance, to use Volume Up/Down for volume control in lists instead of scrolling up/down, first find the `CONTEXT_STD` (or `CONTEXT_LIST`/`CONTEXT_TREE`) mapping in the source file (`apps/keymaps/keymap-hibyr1.c`) that currently uses `BUTTON_UP` and `BUTTON_DOWN` for navigation:

```c
// Original mapping in apps/keymaps/keymap-hibyr1.c
static const struct button_mapping button_context_standard[] = {
    { ACTION_STD_PREV,       BUTTON_UP,                 BUTTON_NONE },
    { ACTION_STD_PREVREPEAT, BUTTON_UP|BUTTON_REPEAT,   BUTTON_NONE },
    // ...
    { ACTION_STD_NEXT,       BUTTON_DOWN,               BUTTON_NONE },
    { ACTION_STD_NEXTREPEAT, BUTTON_DOWN|BUTTON_REPEAT, BUTTON_NONE },
    // ...
};
```

In your custom text remap, you simply copy these lines, change the header to `CONTEXT_STD = {`, and replace the navigation actions (`ACTION_STD_PREV` / `ACTION_STD_NEXT`) with the volume actions (`ACTION_LIST_VOLUP` / `ACTION_LIST_VOLDOWN`):

```c
CONTEXT_STD = {
    {ACTION_LIST_VOLUP, BUTTON_UP, BUTTON_NONE},
    {ACTION_LIST_VOLUP, BUTTON_UP | BUTTON_REPEAT, BUTTON_NONE},
    {ACTION_LIST_VOLDOWN, BUTTON_DOWN, BUTTON_NONE},
    {ACTION_LIST_VOLDOWN, BUTTON_DOWN | BUTTON_REPEAT, BUTTON_NONE},
}
```

## Generating Keymaps with AI

You can easily ask an AI to create a custom remap for you. Since modern AI models can browse the internet, you can simply give them the link to this guide.

Copy and paste the template below into your AI chat (this box has a convenient copy button in the top right):

```text
Please write a custom Rockbox keymap for my Hiby R1. 
First, read the documentation for the syntax, buttons, and contexts here: 
[https://github.com/bahusoid/rockbox/blob/Mod25.12.07/docs/mods/keyremap.md](https://github.com/bahusoid/rockbox/blob/Mod25.12.07/docs/mods/keyremap.md)

Here is what I want the buttons to do:
[Describe what you want here, e.g., "long press on power locks the device", or "use volume buttons for list scrolling instead of navigation"]

Apply these strict constraints based on the documentation:
1. Output ONLY valid keymap text block syntax.
2. Use exact button names (BUTTON_POWER, BUTTON_RIGHT, BUTTON_LEFT, BUTTON_UP, BUTTON_DOWN).
3. Use exact context headers (e.g., CONTEXT_STD = { ... }).
4. Account for PRE_BUTTON_CONDITION if differentiating between short and long presses on the same button.
```

*(If you are using an older AI without internet access, simply copy and paste this entire markdown page into the chat alongside your prompt).*

## Further reference

- `apps/action.h`: all action and context names
- `firmware/target/hosted/hiby/r1/button-target.h`: Hiby R1 button names
- `apps/keymaps/keymap-hibyr1.c`: the built-in target keymap
- `docs/mods/hibyr1-keymap.md`: description of this mod keymap
- `docs/mods/keyremaps/`: ready-to-use text remap files for keyremap plugin

This should be enough to build and tweak your own custom Hiby R1 remaps without guessing the syntax or button names.
