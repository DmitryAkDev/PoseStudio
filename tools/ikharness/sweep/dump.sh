#!/bin/bash
# Dumps the skeleton (and its body-mesh sidecar) of every figure preset listed on stdin, one path a
# line, into a folder — the input of run.sh. Each preset is imported by the real application (so
# the dump is the rig the app would pose: morphed joint centres, figure scale, floor grounding) and
# the app quits again; nothing is clicked, so it is safe on a desktop in use.
#   find <content folder> -name "*.duf" | tools/ikharness/sweep/dump.sh <dump folder>
# POSESTUDIO_EXE overrides the executable (default: build/Release/PoseStudio.exe).
# The dumps are vendor rig data: keep the folder OUTSIDE the repository.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
exe="${POSESTUDIO_EXE:-$here/../../../build/Release/PoseStudio.exe}"
out="$1"
mkdir -p "$out"
quit="$out/.quit_only.txt"
printf 'quit\n' > "$quit"
while IFS= read -r preset; do
    [ -z "$preset" ] && continue
    slug="$(basename "$preset" | sed 's/\.[^.]*$//' | tr -c 'A-Za-z0-9\n' '_' | tr 'A-Z' 'a-z')"
    dump="$out/skeleton_$slug.txt"
    rm -f "$dump" "$dump.mesh"
    POSESTUDIO_DUMP_SKELETON="$dump" POSESTUDIO_IK_SCRIPT="$quit" POSESTUDIO_NO_PING=1 \
        POSESTUDIO_NO_UPDATE_CHECK=1 QT_LOGGING_TO_CONSOLE=1 timeout 300 "$exe" "$preset" > "$out/dump_$slug.log" 2>&1
    if [ -f "$dump" ]; then
        echo "$slug: $(grep -c '' "$dump") bones$(grep -h -o 'rests .* as authored' "$out/dump_$slug.log" | head -1 | sed 's/^/, /')"
    else
        echo "$slug: NO DUMP (not a figure preset, or its content is not on a known root) - see $out/dump_$slug.log"
    fi
done
rm -f "$quit"
