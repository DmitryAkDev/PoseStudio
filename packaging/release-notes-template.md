## 📥 Download & Install

**Most people want this file → [`PoseStudio-{{VERSION}}-Windows-Setup.exe`](https://github.com/PoseStudio/PoseStudio/releases/download/v{{VERSION}}/PoseStudio-{{VERSION}}-Windows-Setup.exe)**

1. **Download** `PoseStudio-{{VERSION}}-Windows-Setup.exe` (from the **Assets** list below).
2. **Run it.** If Windows shows a blue *"Windows protected your PC"* screen, click **More info → Run anyway**. That warning appears because we're a new open-source project that hasn't bought a code-signing certificate yet — the installer is built in public by [GitHub Actions](https://github.com/PoseStudio/PoseStudio/actions) straight from this repository's source code.
3. **Click through the installer** (no administrator password needed) and launch PoseStudio from the Start Menu.

Full step-by-step instructions, including screenshots of the SmartScreen warning and troubleshooting help: **[Installation Guide](https://github.com/PoseStudio/PoseStudio/blob/main/docs/INSTALL.md)**.

| You want… | Download |
|---|---|
| **The app, installed normally** (recommended) | `PoseStudio-{{VERSION}}-Windows-Setup.exe` |
| A no-install folder you can put anywhere | `PoseStudio-{{VERSION}}-Windows-Portable.zip` |
| To verify your download | `SHA256SUMS.txt` |

**System requirements:** 64-bit Windows 10 or 11, and a Vulkan-capable GPU (virtually any graphics card from 2016 onward) with reasonably current drivers.

## 💡 Quick Tips — the stuff that isn't obvious

- 🖱️ **Camera** — left-drag empty space to orbit, scroll to zoom, middle-drag to pan. The 🏠 button (top-right of the viewport) resets the view. Blender-style view keys: **1** / **3** / **7** for the front / right / top view (**Ctrl** for the opposite side; these are flat orthographic views — orbit to return to perspective), **9** flips the view, **5** is the Home view, **.** frames the selected object.
- 🔵 **Selection** — the selected object wears a blue outline. Click an object (or one of a figure's joints) to select it; click empty space to clear the selection. Orbiting never changes it. **Delete** removes the selected object.
- 🦴 **Posing — the magic one:** just grab the figure's joints directly (hand, elbow, knee, head…) and drag. There's no visible skeleton by default, but every joint is grabbable — the part you grab lights up blue — and the whole body follows naturally: feet stay planted, balance is automatic. Keep dragging sideways and the figure *steps and walks* to follow. Pull a hand beyond reach and the figure lifts off and dangles. Drag the hip downward for a crouch. **Ctrl+drag** rotates just that one joint, or hold **X**, **Y**, or **Z** and roll the mouse wheel to dial it about that axis (a badge shows the axis while you hold).
- 📌 **Pin joints** — select a joint and press **P** (or right-click → Pin Joint). Pinned joints stay exactly put while you drag everything else — plant a hand on a table and crouch under it. Pins save with poses.
- 🎭 **Importing figures** — File → Import → .DUF, or double-click one in the Asset Manager. If it asks, point it at your content folder once — it's remembered.
- ⬇️ **Ground button** — drops a floating figure onto the floor. In actual free fall.
- 💾 **Poses** — File → Save Pose / Load Pose. **Ctrl+Z / Ctrl+Y** undo and redo — poses *and* lighting edits.
- 🌅 **Lighting** — Environment tab (right side) → click the environment name for the thumbnail picker. Drop your own `.hdr`/`.exr` files into `Documents\My PoseStudio Library\hdri` — subfolders become categories. And any number field: drag left/right to scrub, click to type.
- 🎨 **Shade modes** — the picker at the viewport's top-right: photoreal PBR, textured, cartoon, matcap, clay, lighting-only, silhouette, wireframes (including a hidden-line one), and data views for albedo, ambient occlusion, roughness, specular, normals, and UVs. View → Show Skeleton overlays the bone cage.

## 💬 We want your feedback!

This release exists so you can try PoseStudio and tell us what's confusing, broken, or missing:

- 🗣️ [Leave usability feedback](https://github.com/PoseStudio/PoseStudio/issues/new?template=usability_feedback.yml) — even "I couldn't figure out how to X" is gold
- 💬 [Join our Discord](https://discord.gg/SaKvt9aYCM) or [GitHub Discussions](https://github.com/PoseStudio/PoseStudio/discussions)
- 🐛 [Report a bug](https://github.com/PoseStudio/PoseStudio/issues)

See the [CHANGELOG](https://github.com/PoseStudio/PoseStudio/blob/main/CHANGELOG.md) for the detailed list of what's new in this version.

---
