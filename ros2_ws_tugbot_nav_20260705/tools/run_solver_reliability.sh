#!/usr/bin/env bash
# Reliability harness for the Trémaux autonomous solver: run N full headless
# runs and tally how many reach the exit. Target: >= 4/5 EXIT_REACHED.
#
# Usage: tools/run_solver_reliability.sh [N_RUNS] [MAX_SECONDS_PER_RUN] [MAX_GOALS]
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"

N="${1:-5}"
SECS="${2:-900}"
MAX_GOALS="${3:-400}"
PASS=0
SUMMARY="log/solver_reliability_$(date +%Y%m%d_%H%M%S).txt"
echo "[RELIABILITY] N=$N secs=$SECS max_goals=$MAX_GOALS" | tee "$SUMMARY"

for i in $(seq 1 "$N"); do
    echo "=== solver run $i/$N ===" | tee -a "$SUMMARY"
    OUT=$(./tools/run_solver_maze.sh "$SECS" true false "$MAX_GOALS")
    ART=$(echo "$OUT" | grep -aoE "ARTIFACT_DIR=.*" | tail -1 | cut -d= -f2)
    R="UNKNOWN"
    [ -n "$ART" ] && R=$(cat "$ART/result.txt" 2>/dev/null)
    [ "$R" = "EXIT_REACHED" ] && PASS=$((PASS + 1))
    echo "run $i: result=$R artifact=$ART (cumulative pass=$PASS/$i)" | tee -a "$SUMMARY"
done

echo "RELIABILITY: $PASS/$N reached EXIT" | tee -a "$SUMMARY"
