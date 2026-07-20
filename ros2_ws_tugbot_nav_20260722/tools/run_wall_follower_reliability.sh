#!/usr/bin/env bash
# Reliability harness for the reactive wall-following solver: run N full headless
# runs and tally how many reach the exit. Target: >= 4/5 EXIT_REACHED.
#
# Usage: tools/run_wall_follower_reliability.sh [N_RUNS] [MAX_SECONDS_PER_RUN] [FOLLOW_SIDE]
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"

N="${1:-5}"
SECS="${2:-1500}"
FOLLOW_SIDE="${3:-left}"
PASS=0
SUMMARY="log/wall_follower_reliability_$(date +%Y%m%d_%H%M%S).txt"
echo "[RELIABILITY] N=$N secs=$SECS side=$FOLLOW_SIDE" | tee "$SUMMARY"

for i in $(seq 1 "$N"); do
    echo "=== wall-follower run $i/$N ===" | tee -a "$SUMMARY"
    OUT=$(./tools/run_wall_follower_maze.sh "$SECS" true false "$FOLLOW_SIDE")
    ART=$(echo "$OUT" | grep -aoE "ARTIFACT_DIR=.*" | tail -1 | cut -d= -f2)
    R="UNKNOWN"
    [ -n "$ART" ] && R=$(cat "$ART/result.txt" 2>/dev/null)
    [ "$R" = "EXIT_REACHED" ] && PASS=$((PASS + 1))
    echo "run $i: result=$R artifact=$ART (cumulative pass=$PASS/$i)" | tee -a "$SUMMARY"
done

echo "RELIABILITY: $PASS/$N reached EXIT" | tee -a "$SUMMARY"
