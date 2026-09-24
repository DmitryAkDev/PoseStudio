#!/bin/bash
# Runs every in-app posing script in this folder as a TEST and reports what changed.
#   POSESTUDIO_TEST_FIGURE=<figure preset> tools/ikscripts/run_all.sh <outdir> [--accept] [NAME=value ...]
#
# Per script: the app runs it (run.sh), its `expect` lines pass or fail (the app's exit code is the
# number that failed), a contact sheet is built, and — when POSESTUDIO_IKSHOTS_REF names a folder of
# reference shots — every shot is compared with its reference (compare.py): the ones that LOOK
# different are listed and laid out beside their references in <outdir>/<script>/changes.png.
# --accept makes this run's shots the new references (after you have looked at the changes).
# Extra NAME=value arguments reach the app as environment (IK_… probes for an A/B run).
# The exit code is the number of scripts with a failed expectation.
set -u
here="$(cd "$(dirname "$0")" && pwd)"
out="$1"; shift
accept=0
probes=()
for a in "$@"; do
    if [ "$a" = "--accept" ]; then accept=1; else probes+=("$a"); fi
done
ref="${POSESTUDIO_IKSHOTS_REF:-}"
failed=0
summary=()
for script in "$here"/*.txt; do
    name="$(basename "$script" .txt)"
    echo "=== $name"
    bash "$here/run.sh" "$script" "$out/$name" ${probes[@]+"${probes[@]}"} | grep -E "EXPECT FAILED|DID NOT FINISH|began no drag|unknown|NOT SAVED|^sheet"
    verdict="$(grep -E "\[ikscript\] done" "$out/$name/log.txt" | sed 's/.*done: //')"
    if [ -z "$verdict" ]; then
        verdict="DID NOT FINISH"
        failed=$((failed + 1))
    elif ! echo "$verdict" | grep -q " 0 FAILED"; then
        failed=$((failed + 1))
    fi
    summary+=("$name: $verdict")
    if [ -n "$ref" ]; then
        if [ "$accept" = 1 ]; then
            python "$here/compare.py" "$out/$name" "$ref/$name" --accept
        elif [ -d "$ref/$name" ]; then
            python "$here/compare.py" "$out/$name" "$ref/$name"
        else
            echo "  (no reference shots yet: run with --accept to keep these)"
        fi
    fi
done
echo "=== summary"
for line in "${summary[@]}"; do echo "  $line"; done
exit $failed
