# Keyboard and Mouse Reference

Every control in one place. "Viewport focus" means you have clicked in the 3D view since you last used another panel; the other shortcuts work whichever panel has focus (a text field you are typing in keeps its own keys).

## Mouse in the viewport

| Gesture | Where | What it does |
| --- | --- | --- |
| Left-drag | on a joint | Full-body IK: the point you grabbed follows the cursor, the body follows anatomically. |
| Left-drag | on empty space or a model's body | Orbit the camera. |
| Ctrl + left-drag | on a joint | Rotate that joint alone (sideways: about its vertical axis; up and down: about its sideways axis). |
| Left-click | on a model | Select it (blue outline); on empty space, deselect. |
| Left-click | on a joint | Select the joint (and make its figure the active one). |
| Middle-drag | anywhere | Pan. |
| Wheel | normally | Zoom. |
| Wheel | during a left-drag of a joint | Move the grabbed point toward (scroll down) or away from (scroll up) the camera. |
| Wheel | while holding `X`, `Y` or `Z` with a joint selected | Rotate the joint about that axis, 5° a notch. |
| Right-click | on a model or joint | The context menu: pin / unpin, reset joint or limb, mirror limb, delete. |

## Camera

| Key | Action |
| --- | --- |
| `1` / `Ctrl+1` | Front / Back view |
| `3` / `Ctrl+3` | Right / Left view |
| `7` / `Ctrl+7` | Top / Bottom view |
| `9` | Flip the view 180° |
| `5` | Home view (the default framing) |
| `.` | Frame the selected model |

The keys work on the number row and on the numeric keypad. With NumLock off, the keypad's `End` / `PageDown` / `Home` / `PageUp` / `Clear` (5) / `Del` stand in for `1` / `3` / `7` / `9` / `5` / `.`. The axis views are orthographic; orbiting returns to perspective.

## Posing

| Key | Needs | Action |
| --- | --- | --- |
| `P` | viewport focus, a joint selected | Pin or unpin the selected joint |
| `X`, `Y`, `Z` (hold) + wheel | viewport focus, a joint selected | Rotate the joint about that axis |
| `Delete` or `Backspace` | viewport focus, a model selected | Delete the selected model (clears the undo history) |
| `Ctrl+Z` | | Undo |
| `Ctrl+Y` | | Redo |

Edit → Reset Selected Joint, Reset Limb, Reset Pose, Mirror Pose and Mirror Limb to Other Side have no keys; they are on the Edit menu and (the per-joint ones) on the joint's context menu.

## Application

| Key | Action |
| --- | --- |
| `F1` | Open this manual |
| `Ctrl+N` | File → New — reset the scene to a fresh launch |
| `Ctrl+O` | File → Open — load a `.pss` project |
| `Ctrl+S` | File → Save — save the scene to its `.pss` file (asks for a place first if it has none) |
| `Ctrl+Shift+S` | File → Save As… — save the scene under a new `.pss` name |
| `Ctrl+Q` | Quit |

## Asset Manager

| Key | Action |
| --- | --- |
| `Enter` in the search field | Search |
| `F2` on a collection | Rename it |
| Double-click an asset | Import it (`.obj`, `.duf`, `.dsf`) or open it with its application |
| Double-click a folder | Open the folder |
| Drag a thumbnail | Reorder (in Favorites and Collections), or drop onto Favorites / a Collection in the tree |

## Environment tab

| Gesture | Action |
| --- | --- |
| Drag across a value | Scrub it |
| Click a value | Type it; `Enter` confirms, `Esc` cancels |
| The small button beside a row | Restore that setting's default |

## This manual

| Key | Action |
| --- | --- |
| `Ctrl+F` | Search |
| `Enter` / `F3` | Next match; `Shift+F3` previous |
| `Alt+Left` / `Alt+Right` | Back / Forward |
| `Esc` | Close |
