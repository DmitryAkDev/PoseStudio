#!/bin/bash
# Runs the whole IK harness on every skeleton dump in a folder, several at a time, and tabulates
# the gates that failed (summarize.py). The harness's real-figure gates are calibrated on the
# reference rigs; a sweep over many CHARACTER rigs — other proportions, joint centres, scales — is
# the robustness report: which gates are rig-calibrated numbers a few percent over, and which are
# behaviour that breaks on some body.
#   tools/ikharness/sweep/run.sh <dump folder> <results folder> [NAME=value ...]
# Extra NAME=value arguments reach the harness as environment (IK_… probes for an A/B sweep).
# POSESTUDIO_HARNESS overrides the executable; SWEEP_JOBS the parallelism (default 4).
set -u
here="$(cd "$(dirname "$0")" && pwd)"
exe="${POSESTUDIO_HARNESS:-$here/../../../build/Release/PoseStudioIkHarness.exe}"
dumps="$1"; results="$2"; shift 2
mkdir -p "$results"
export SWEEP_EXE="$exe" SWEEP_RESULTS="$results" SWEEP_ENV="$*"
ls "$dumps"/skeleton_*.txt | xargs -P "${SWEEP_JOBS:-4}" -I{} bash -c \
    'slug="$(basename "{}" .txt | sed "s/^skeleton_//")"; env $SWEEP_ENV "$SWEEP_EXE" "{}" --verbose > "$SWEEP_RESULTS/$slug.txt" 2>&1'
python "$here/summarize.py" "$results"
