#!/bin/bash
# Runs one in-app posing script and builds a contact sheet of its shots.
#   POSESTUDIO_TEST_FIGURE=<figure preset> tools/ikscripts/run.sh <script.txt> <outdir> [NAME=value ...]
# Extra NAME=value arguments are passed to the app as environment (IK_… probes for an A/B).
# POSESTUDIO_EXE overrides the executable (default: build/Release/PoseStudio.exe).
# The exit code is the app's: the number of `expect` lines that failed.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
exe="${POSESTUDIO_EXE:-$here/../../build/Release/PoseStudio.exe}"
if [ -z "${POSESTUDIO_TEST_FIGURE:-}" ]; then
    echo "set POSESTUDIO_TEST_FIGURE to a figure preset" >&2
    exit 2
fi
script="$1"; out="$2"; shift 2
mkdir -p "$out"
rm -f "$out"/*.png
env "$@" POSESTUDIO_IK_SCRIPT="$script" POSESTUDIO_IK_SCRIPT_OUT="$out" POSESTUDIO_NO_PING=1 \
    POSESTUDIO_NO_UPDATE_CHECK=1 QT_LOGGING_TO_CONSOLE=1 "$exe" "$POSESTUDIO_TEST_FIGURE" > "$out/raw.txt" 2>&1
code=$?
grep -E "\[ikscript\]" "$out/raw.txt" > "$out/log.txt"
if grep -q "\[ikscript\] done" "$out/log.txt"; then
    rm -f "$out/raw.txt" "$out/unfinished.txt"
else
    # The app went away mid-script (a crash, a lost device, a window someone closed): keep what it
    # said, and how it ended, for the post-mortem.
    mv "$out/raw.txt" "$out/unfinished.txt"
    echo "DID NOT FINISH (exit code $code): the app's whole output is in $out/unfinished.txt"
fi
grep -E "\]   \[|EXPECT|picked|began no drag|unknown|NOT SAVED|\] done" "$out/log.txt"
python "$here/montage.py" "$out" "$out/sheet.png" "${SHEET_COLS:-4}" "${SHEET_CROP:-0.42}"
exit $code
