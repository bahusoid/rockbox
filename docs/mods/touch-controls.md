# Touch Controls

If you have trouble tapping the right element in lists, you can increase the line height using the `Line Padding in Lists` setting (found in *Settings -> General Settings -> Display -> Touchscreen Settings*).
This build also enables the `Line Separator` by default to make element tap zones easier to see (found in *Settings -> Theme Settings*).

Touch controls (gestures, swipes, and kinetic scrolling) developed by [**amachronic**](https://gerrit.rockbox.org/r/c/rockbox/+/5393).

**Development Build Changes:**
* I modified the kinetic scrolling to remove "inertia" when stopping - it now stops immediately on a tap or at the end of a list, which feels snappier.
* I removed the right-half header tap shortcut for the **Quickscreen** (it caused too many accidental activations when I expected to go **Back**).

## 1. In Lists and Menus

**Header Taps**
* Tap the left half of the title bar to go **Back** or **Cancel**.
* Tap the right half to ~~bring up the **Quickscreen**~~ go **Back** or **Cancel**.
* Long-press the left half to return to the **Root Menu**.
* Long-press the right half to go to the **While Playing Screen (WPS)**.

> **Note:** For right-to-left languages, the left and right shortcuts are swapped.

> **Note:** Header taps do not work with the *Snappy* theme.

**Edge Swipes**
*(Note: Swipes must start from the very edge of the screen. Longer swipes are more reliably recognized.)*
* Top to bottom: **Quickscreen**
* Right to left: **While Playing Screen (WPS)**
* Left to right: **Back**

## 2. While Playing Screen (WPS)

**Edge Swipes**
* Top to bottom: **Quickscreen**
* Right to left: **Current Playlist / Cuesheet**
* Left to right: **File Browser**
* Bottom to top: **Context Menu**