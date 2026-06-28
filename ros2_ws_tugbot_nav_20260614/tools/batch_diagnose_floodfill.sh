#!/usr/bin/env bash
# Controlled-batch diagnostic for the flood-fill solver.
# Runs run_flood_fill_maze.sh N times with the SAME config (current main),
# collects ONLY this batch's artifact dirs, then runs the failure-mode
# classifier over them so the result is a controlled distribution, not a
# single-run anecdote.
#
# Usage: tools/batch_diagnose_floodfill.sh [N] [MAX_SECONDS] [HEADLESS] [USE_RVIZ] [POSE_SOURCE] [SENSE_DEBUG]
#   defaults: N=8 MAX_SECONDS=1200 HEADLESS=true USE_RVIZ=false POSE_SOURCE=odom_locked SENSE_DEBUG=true
#
# Notes:
#   - HEADLESS=true / USE_RVIZ=false are deliberate batch defaults (speed +
#     reproducibility across the N runs). To match GUI runs, pass: ... false true ...
#     (and `export DISPLAY=:1` first).
#   - Each inner run self-cleans sims before/after, so runs are independent.
#   - Worst case wall-clock ~= N * MAX_SECONDS (most failing runs sit stuck
#     until the cap). N=8, MAX=1200 -> up to ~2.7h.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"

N="${1:-8}"
MAX_SECONDS="${2:-1200}"
HEADLESS="${3:-true}"
USE_RVIZ="${4:-false}"
POSE_SOURCE="${5:-odom_locked}"
SENSE_DEBUG="${6:-true}"

STAMP="$(date +%Y%m%d_%H%M%S)"
BATCH="log/batch_diag_${STAMP}"
mkdir -p "$BATCH"
MANIFEST="$BATCH/manifest.txt"
: > "$MANIFEST"

echo "[BATCH] N=$N max=${MAX_SECONDS}s headless=$HEADLESS rviz=$USE_RVIZ pose=$POSE_SOURCE sense=$SENSE_DEBUG"
echo "[BATCH] artifacts -> $BATCH"

for i in $(seq 1 "$N"); do
    echo "[BATCH] === run $i / $N ==="
    OUT="$(bash tools/run_flood_fill_maze.sh "$MAX_SECONDS" "$HEADLESS" "$USE_RVIZ" "$POSE_SOURCE" "$SENSE_DEBUG")"
    echo "$OUT" | tail -3
    ART="$(echo "$OUT" | sed -n 's/^ARTIFACT_DIR=//p' | tail -1)"
    if [ -n "$ART" ]; then
        echo "$WS/$ART" >> "$MANIFEST"
        echo "[BATCH] run $i artifact: $ART"
    else
        echo "[BATCH] run $i produced no ARTIFACT_DIR (infra fail?)"
    fi
done

echo "[BATCH] all runs done; classifying $(wc -l < "$MANIFEST") collected runs"
# Feed the exact collected dirs to the classifier (one batch, no history mixed in).
mapfile -t DIRS < "$MANIFEST"
python3 tools/diagnose_floodfill_runs.py "${DIRS[@]}" | tee "$BATCH/report.txt"

echo "[BATCH] report written -> $BATCH/report.txt"
echo "BATCH_DIR=$BATCH"
