#!/usr/bin/env bash
# Run the GCN maze traversal N times sequentially and tally EXIT_REACHED vs not.
# Usage: tools/run_gcn_reliability.sh [N] [PER_RUN_SECONDS]
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"
cd "$WS"
N="${1:-3}"
SECS="${2:-540}"
STAMP="$(date +%Y%m%d_%H%M%S)"
SUMMARY="log/gcn_reliability_${STAMP}.txt"
echo "GCN reliability: N=$N per_run=${SECS}s  -> $SUMMARY" | tee "$SUMMARY"

pass=0
for i in $(seq 1 "$N"); do
    echo "=== RUN $i/$N ($(date +%H:%M:%S)) ===" | tee -a "$SUMMARY"
    # run_gcn_maze.sh self-cleans processes at the end
    bash tools/run_gcn_maze.sh "$SECS" true >/dev/null 2>&1
    ART=$(ls -td log/gcn_run_* | head -1)
    RES=$(cat "$ART/result.txt" 2>/dev/null || echo "NO_RESULT")
    NADV=$(grep -ac "GCN: reached target" "$ART/launch.log" 2>/dev/null || echo 0)
    LASTC=$(grep -aoE "now on C[0-9]+" "$ART/launch.log" 2>/dev/null | tail -1)
    echo "RUN $i: result=$RES corridor_advances=$NADV last=[$LASTC] art=$ART" | tee -a "$SUMMARY"
    [ "$RES" = "EXIT_REACHED" ] && pass=$((pass+1))
done
echo "=== RELIABILITY: $pass/$N reached exit ===" | tee -a "$SUMMARY"
echo "RELIABILITY_DONE pass=$pass of $N"
