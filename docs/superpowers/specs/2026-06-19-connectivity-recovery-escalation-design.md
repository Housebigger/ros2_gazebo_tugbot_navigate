# Connectivity-Recovery Escalation (`_unstick`) — Design

**Date:** 2026-06-19
**Workspace:** `ros2_ws_tugbot_nav_20260614` · **Branch:** `tighter-corridor-controller` (local only)
**Status:** Approved (ladder approved by user), ready for plan.

## Problem

The hardened `_center` fix eliminated the (3,4) churn and drove a clean 20-cell path to (7,3), then
hit terminal `stuck`: the exit was **unreachable in the known map**, and the current `_unstick` only
re-opens *locomotion-failure* walls — so a false/poor **sensed** wall sealing the exit path has
nothing to blame → `stuck`. In a perfect maze the exit is always reachable, so a frontier sensed
WALL is false-and-disconnecting (typically a non-committed poor first-read the robot can't return to
re-sense once cut off). The recovery surface is too narrow.

## Goal

Replace the locomotion-only `_unstick` with a **cut-based escalating recovery**: re-open the WALL
edges that actually separate the robot's reachable component from the rest of the maze, in ascending
order of trust (locomotion → non-committed sensed → committed), forcing a fresh good-gated re-sense,
bounded so it always terminates.

## Design (one method + one helper + state; `_center`/`_drive`/primitives unchanged)

**New state** (in `__init__`): `self.reopened = set()` — edges `(cell, dir)` already re-opened by
`_unstick` (monotonic; the termination bound). **Remove** `self.evicted` (superseded by `reopened`).

**`_reachable_component(start)`** — cells reachable from `start` over **non-WALL** edges (OPEN or
UNKNOWN are traversable; only WALL blocks), BFS:
```
seen = {start}; stack = [start]
while stack:
    c = stack.pop()
    for d,(dx,dy) in DIRS.items():
        nb = (c[0]+dx, c[1]+dy)
        if in_grid(nb) and not self.brain.is_wall(c,d) and nb not in seen:
            seen.add(nb); stack.append(nb)
return seen
```

**`_unstick(t)`** — replaces the current method. Called by `_route` when `next_cell is None` OR the
exit is flood-unreachable from `self.cell` (both reduce to "self.cell's component excludes the exit"):
```
R = self._reachable_component(self.cell)
# Cut = not-yet-reopened WALL edges from a cell in R to an in-grid neighbour OUTSIDE R.
cut = [(c,d) for c in R for d,(dx,dy) in DIRS.items()
       if self.brain.is_wall(c,d) and (c,d) not in self.reopened
       and in_grid((c[0]+dx,c[1]+dy)) and (c[0]+dx,c[1]+dy) not in R]
# Escalate by ASCENDING trust. locomotion walls are hop-failure (most untrusted);
# non-committed sensed walls are poor first-reads; committed walls are 2x-corroborated (trust last).
def is_loco(e):  return e in self.locomotion_walls
def is_comm(e):  return e[0] in self.committed
for pick in (lambda e: is_loco(e),
             lambda e: (not is_loco(e)) and (not is_comm(e)),
             lambda e: is_comm(e)):
    cand = [e for e in cut if pick(e)]
    if cand:
        for (c,d) in cand:
            self.brain.mark(c, d, is_wall=False)        # re-open optimistically; a good re-sense re-confirms
            self.reopened.add((c,d))
            self.locomotion_walls.discard((c,d))
            self.committed.discard(c); self.sensed.discard(c); self.corrob.pop(c, None)
        self.center_start = None
        self.align_start = None; self.latched_cardinal = None
        self.settle_until = t + self.settle_s
        return (0.0, 0.0, False)                         # stay in center -> re-route/re-sense next tick
self.phase = 'stuck'
return (0.0, 0.0, False)
```

**Why this is correct + bounded:**
- *Surgical:* only cut edges (walls that actually separate R from the rest) are re-opened — not the
  whole map. Re-opening a cut edge strictly grows R (toward the exit's region).
- *Trust order:* the most-likely-false walls (locomotion hop-failures, then poor non-committed
  reads) are tried before 2x-corroborated committed walls, which stay trusted longest.
- *Self-confirming:* re-open is optimistic; the robot drives to the re-opened cell and re-senses it
  (good-gated, reliable) — a genuine wall is re-marked WALL, a false one stays OPEN and reconnects.
- *Termination:* every re-opened edge enters `reopened` and is never re-opened again; `cut` excludes
  `reopened` edges, so each `_unstick` consumes ≥1 of the ≤~360 WALL edges. When no un-reopened cut
  edge remains → `stuck`. Monotonic, hard bound.

`_route` is unchanged from the prior commit (`nxt is None or not reachable -> self._unstick(t)`); only
`_unstick`'s body and the helper change.

## Tests (update existing + add)

- Update `test_committed_dead_end_is_evicted_not_stuck` and `test_disconnecting_false_wall_triggers_unstick`:
  assert recovery via `self.reopened` (the cut edges were re-opened, `phase != 'stuck'`, walls OPEN)
  instead of the removed `self.evicted`.
- Add `test_unstick_escalates_to_noncommitted_then_committed`: a cut where the only cut edge is a
  NON-COMMITTED sensed wall (no locomotion wall) → `_unstick` re-opens it (tier 2), not `stuck`; and a
  variant where the only cut edge is COMMITTED → re-opened only at tier 3.
- Add `test_unstick_terminates_when_all_cut_edges_reopened`: after every cut edge is in `reopened`,
  `_unstick` declares `stuck` (termination).
- Offline regression `test_maze_motion_sim` stays green (the end-to-end solve is unaffected: it never
  disconnects, so `_unstick` never fires there).

## Validation
99→~101 offline tests green; then a Gazebo run watching whether (7,3)-class disconnections now
recover (re-open the false cut, re-sense, continue) and the robot pushes past dist 13.36 toward the
exit. Multiple seeded runs per the standing spec. Clear stray sims via the `!` pkill first.

## Constraints
Local only, never push. Trailer `Co-Authored-By: Claude Opus 4.8 (1M context) <noreply@anthropic.com>`.
Agent pkill hook-blocked (user clears sims). Foreground sleep blocked. Absolute paths / `git -C`.
