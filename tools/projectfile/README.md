# .pss project file tools

Two standalone tools for the `.pss` project file codec (`src/viewport/scene/projectfile.cpp`) —
one a guard that runs in CTest, the other a headless diagnostic that reproduces the
window-level save/load sequence.

## PoseStudioProjectFileCheck (`roundtrip.cpp`)

Round-trip and rejection checks for the file-format contract. It writes a fully-populated
`ProjectDocument` to the OS temp dir (`pss-rt/`, created up front), reads it back and compares
every field bit-for-bit, then feeds the reader its failure cases — bad JSON, wrong format
marker, higher version, missing file — plus a forward-compatibility document with unknown
fields at every level.

It links only `scene/projectfile.cpp` — no Qt, no Vulkan — so it runs anywhere the app builds.
Registered with CTest:

```
ctest -R project-file-check
```

Exit code = number of failed checks (0 = green).

## PoseStudioPssProbe (`probe.cpp`)

Headless diagnostic for save/load bugs: imports a real figure through `FigureImportService`,
poses a few bones, applies a non-identity model transform, saves to `.pss` — then re-imports
fresh and restores exactly the way `VulkanWindow::loadProjectFile()` does. It prints captures
at each stage (after import, after the codec round-trip, after restore) plus the constructed
and actual model matrices, so a figure that comes back wrong from a `.pss` file can be
isolated to the codec, the import or the restore.

```
cmake --build build --target PoseStudioPssProbe -j4
POSESTUDIO_NO_UPDATE_CHECK=1 ./build/PoseStudioPssProbe <figure.duf> <out.pss>
```

Exit code 0 = the restore matches the original (3 = a divergence, printed with `DIFF` lines).
Needs a Vulkan device; a debug import of a Genesis 9 figure takes ~20 s.
