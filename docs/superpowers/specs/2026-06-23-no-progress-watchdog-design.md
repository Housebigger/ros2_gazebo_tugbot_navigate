# No-Progress Watchdog + Escalating Escape — Design

**Status:** approved architecture, hardened by adversarial review (8 must-fixes + should-fixes folded — see §11).
**Supersedes:** the per-cell stall watchdog (`docs/superpowers/specs/2026-06-22-decision-cell-stall-watchdog-design.md`), whose premise ("the robot thrashes in `self.cell` for > `cell_stall_s`") was falsified by Gazebo run `flood_fill_run_20260622_224432` (§1). This redesign keeps that branch's one good part (arrival-time `mark_traversal`) and replaces the rest.

---

## 1. Motivation (what the Gazebo run proved)

Run `flood_fill_run_20260622_224432` (1800 s GUI, odom_locked, per-cell stall watchdog active): `RESULT=TIMEOUT`, best `dist_to_exit` only **18.29 m** at (3,4) @ t=160 s (prior bests 1.65 m / 12.84 m), 11 distinct cells, ended **`phase=stuck` at (1,4) for the final 725 s**. Timeline: clean climb (1,0)→(3,4) by t=160 s, then ~15 min of **2–4-cell ping-pong** across (1,3)/(2,3)/(1,4)/(1,5), then terminal stuck.

Three root findings, all confirmed against code + log:

- **F1 — the per-cell clock is defeated by multi-cell ping-pong.** `_track_cell` reset `cell_since_t` on every `self.cell` change; the real churn is a 2–4-cell oscillation, so 120 s never accumulated in any one cell.
- **F2 — `phase=='stuck'` is a hard terminal freeze** (`maze_motion.py:136-137` returns zero velocity, never re-enters `_route`). The watchdog check lived at the top of `_route`, so once stuck it could never fire → 725 s dead.
- **F3 — failed forward hops no longer deprioritize their edge.** Hop-timeout failures deliberately stopped stamping walls (a past false-wall fix); combined with arrival-time `mark_traversal` (a *bounced* hop no longer counts toward the Trémaux ≤2 cap), an unpassable forward edge can be re-selected indefinitely.

Caveat: documented run-to-run non-determinism (this run drew the SW/entrance path; the 1.65 m run drew NE). F1/F2 are structural (path-independent); F3 is mechanistic.

## 2. Goal

Bound both the multi-cell churn and the terminal freeze with a **no-progress watchdog** at the top of `step()` (covering every phase, incl. `stuck`), gated by a **confinement predicate** (so long legitimate backtracks don't false-fire), driving an **escalating escape** (decisive re-approach → give-up-edge + reverse + flood-reroute). Add a conservative **failed-hop deprioritizer** as the fast local loop-breaker, keeping the now-correct arrival-time `mark_traversal`. Success = no multi-minute dwell, no terminal freeze while reachable frontier remains, offline regression green with the watchdog silent on the calibrated solve.

## 3. What is kept / replaced / reused

- **KEEP** — arrival-time `mark_traversal` (branch commit `024bd87`: count a Trémaux traversal in `_drive` on hop arrival, before `self.cell = self.hop_target`) and its two unit tests.
- **REPLACE** — the per-cell stall machinery: the `_route`-top stall check, `_escape_stall`, the `cell_stall_s`/`stall_escapes`/`stall_escape_count` state, and `_track_cell`'s stall-clock (`cell_since_t`). `_track_cell` is retained and repurposed.
- **REUSE** — `backout` phase + `_backout` (the one-cell reverse), `_unstick` + its `self.reopened`/`locomotion_walls` ladder, `committed`/`sensed`/`reopened` sets, `_dir_name`/`DIRS`/`OPP`/`in_grid`.
  - **Critical reuse note (MF1):** `_stamp_loco_wall(cell,d)` only adds to `self.locomotion_walls` — a **trust-tier provenance tag** so `_unstick` reopens loco-walls *first* (`maze_motion.py:337`). It does **not** change routing: `flood`/`next_cell`/`is_wall` and the `_unstick` cut (`maze_motion.py:335`) key off `self.brain._edge` (i.e. `brain.mark(cell,d,is_wall=True)`). **Every edge give-up / deprioritize MUST call `brain.mark(cell,d,is_wall=True)` AND `_stamp_loco_wall(cell,d)` AND `committed.discard(cell)` together** (mirroring the existing give-up at `maze_motion.py:445-448` / `467-470`). A loco-wall alone is a no-op.

**Branch strategy (for the plan):** fresh branch `no-progress-watchdog` off `main` (`cedb2d1`); cherry-pick `024bd87` (the `mark_traversal` fix + its tests); build this design on top. Do **not** carry `c237b45`/`e3c490c` (the replaced watchdog) — and migrate/delete its now-obsolete tests (§9, MF7). Never push; merge is a separate user step after a good Gazebo run.

## 4. Components

### C1 — Exploration-progress clock + confinement footprint
New monotonic progress signal, distinct from the intra-cell wedge detector (`progress_t`/`progress_pose`/`wedge_detect_s`, untouched).

State (`__init__`):
```
self.visited = set()                 # cells ever occupied (monotonic; never shrinks)
self.explore_t = None                # sim-time of last visited-growth (None = unseeded)
self.recent = []                     # [(t, cell)] appended on cell-change; rolling confinement window
self.escape_tier = 0                 # current escalation level (0 = none)
self.escape_count = 0                # MONOTONIC observable (calibration/regression gate, MF7)
self._escape_backout = False         # tags an escape reverse (vs a dead-end back-out) [renamed _stall_backout]
self.no_progress_s = <calibrated>    # window; calibrated across ALL 5 (drift,latency) cases (§7)
self.confine_k = 6                   # max distinct cells in-window to count as "confined" (calibrated)
self.max_escape_tier = 2
self.failed_hops = {}                # (cell,dir) -> consecutive cross-occupation failed-hop count (C5)
self.failed_hop_limit = 3
```
`last_seen_cell`/`prev_cell` are retained from the replaced `_track_cell`.

`_track_cell(t)` — **the seed and visited-update blocks run UNCONDITIONALLY (MF2)**, idempotent across its two calls (step() and top of `_route`):
```
def _track_cell(self, t):
    if self.explore_t is None:               # seed (unconditional; like the old cell_since_t==0.0 seed)
        self.explore_t = t
    if self.cell not in self.visited:        # NEW GROUND (unconditional: entrance enters on tick 1,
        self.visited.add(self.cell)          #   since cell == last_seen_cell == ENTRANCE_CELL initially,
        self.explore_t = t                   #   so a change-gated block would never seed it -> F2 again)
        self.escape_tier = 0                 # real progress clears any in-flight escalation
    if self.cell != self.last_seen_cell:     # cell-change bookkeeping
        self.prev_cell = self.last_seen_cell
        self.last_seen_cell = self.cell
        self.recent.append((t, self.cell))
```
**Confinement** — `_confined(t)`: prune `self.recent` to entries with `t' >= t - self.no_progress_s`; `footprint = {c for (_, c) in self.recent} | {self.cell}`; return `len(footprint) <= self.confine_k`. A tight ping-pong keeps a small rolling footprint (fires); a long cross-maze backtrack sweeps many distinct cells (not confined → no fire); a frozen single cell → footprint `{cell}` size 1 (fires).

### C2 — Top-level watchdog (the F2 fix)
Insert **immediately after `self._track_cell(t)` (≈`maze_motion.py:132`) and BEFORE the done/exit early-return at :133** (so the `cell != EXIT_CELL` guard is non-vacuous, NH13):
```
if (self.phase != 'done' and self.cell != EXIT_CELL
        and not self._escape_backout                     # don't re-fire mid-escape-reverse (MF4)
        and self.explore_t is not None
        and (t - self.explore_t) > self.no_progress_s
        and self._confined(t)):
    return self._escape(pose, t)
```
Runs every tick in every phase → a frozen `stuck` trips it. `not self._escape_backout` suppresses re-firing while the one-cell escape reverse executes (the only multi-tick escape action, since the trail-retreat is dropped — MF3).

### C3 — Escalating escape (`_escape`, replaces `_escape_stall`)
`escape_tier` (reset to 0 on new ground by C1) escalates per fire; **`explore_t` is reset on EVERY entry (MF4)** so a no-op escape can't busy-re-fire:
```
def _escape(self, pose, t):
    x, y, yaw = pose
    self.escape_count += 1
    self.escape_tier = min(self.escape_tier + 1, self.max_escape_tier)   # MF11: pinned cap
    self.explore_t = t                                                   # MF4: reset on every entry
    pc = self.prev_cell
    man = None if pc is None else abs(pc[0]-self.cell[0]) + abs(pc[1]-self.cell[1])
    d_prev = (_dir_name((pc[0]-self.cell[0], pc[1]-self.cell[1]))
              if (pc is not None and man == 1) else None)
    can_reverse = d_prev is not None and not self.brain.is_wall(self.cell, d_prev)
    # Tier >=2: GIVE UP the blocked forward edge (real wall + provenance + uncommit), reopenable by _unstick
    if self.escape_tier >= 2 and self.hop_dir is not None:               # blocked_dir = hop_dir at fire time
        dirn = _dir_name(self.hop_dir)                                   #   (if hop_dir stale/None: skip give-up)
        self.brain.mark(self.cell, dirn, is_wall=True)                   # MF1: the real routing change
        self._stamp_loco_wall(self.cell, dirn)                          # provenance: _unstick reopens loco first
        self.committed.discard(self.cell)
        self.failed_hops.pop((self.cell, dirn), None)
    # Tier 1 & 2: decisive ONE-CELL reverse to the adjacent known-open prev_cell (MF3: no trail-retreat)
    if can_reverse:
        dx, dy = DIRS[d_prev]
        self.backout_target = pc
        self.backout_cardinal = math.atan2(-dy, -dx)                     # face away from prev -> reverse into it
        self.backout_start = (x, y)
        self.backout_deadline = t + self.backout_timeout_s
        self.center_start = None
        self._escape_backout = True
        self.phase = 'backout'
        return (0.0, 0.0, False)
    # No valid reverse -> hand to _unstick AT MOST ONCE this tick (MF5: terminal within the tick)
    return self._unstick(t)
```
- **Tier 1** (first fire): reverse to `prev_cell`, edge left OPEN → on arrival, normal `_route` re-approaches the same hop from a fresh pose (clears a position-dependent false `front_block`). New ground → `escape_tier=0` (success).
- **Tier 2** (fires again, no new ground): give up the blocked edge (MF1) + reverse. Then `flood`/`next_cell` routes from `prev_cell` to the nearest optimistic-UNKNOWN frontier over the known map — i.e. **flood already *is* "retreat to nearest unexplored junction"** (MF3); no explicit multi-hop machinery.
- **No reverse possible** (null/non-adjacent/walled `prev_cell`): hand to `_unstick` once.

`_backout`'s timeout branch keeps the `_escape_backout` guard (an escape-reverse timeout must NOT charge the dead-end `backout_attempts` budget), and clears `_escape_backout` on leaving `backout` (arrival or timeout). `escape_count`/the escape reverse never touch `backout_count` (the dead-end metric) — `_escape` sets `phase='backout'` directly, not via `_route`'s dead-end branch.

### C4 — `stuck` is no longer terminal (other half of F2; resolves MF5)
No `_unstick`→`_escape` routing (that mutual recursion has no in-tick base case — MF5). Instead: `_unstick` is **unchanged** (sets `phase='stuck'` on `self.reopened` exhaustion and returns). Non-terminality comes purely from **C2 re-checking every tick**: a `stuck` robot trips the watchdog after `no_progress_s` (it is trivially confined — footprint `{cell}`) and `_escape` fires next tick. `escape_tier` is also reset to 0 whenever `_unstick` **reopens** an edge (a map mutation is a fresh situation, MF11) so a post-stuck revival retries the cheap Tier-1 first.
- **Latched permanent stop (NH14):** if (every reachable cell is `visited`) AND (no un-reopened cut edge remains) AND (exit unreachable), set a latched terminal flag that issues a quiet permanent stop and is **exempt from the C2 re-fire** — avoids a benign forever-oscillation in a genuinely unsolvable/fully-explored maze. (Does not arise in the real solvable maze, but bounds the pathological case.)

### C5 — Failed-hop deprioritizer (the F3 fix; the `mark_traversal` companion)
`self.failed_hops: {(cell,dir): consecutive_count}`, distinct from the existing per-dwell `hop_attempts` (which is wholesale-cleared on arrival and already hard-walls after `max_hop_attempts`). `failed_hops` is the **cross-occupation accumulator** that persists across cell changes (MF6).
- In `_drive`'s hop-failure branch (deadline/attempts exceeded → bounced to `center` without arrival): `self.failed_hops[(self.cell,dirn)] += 1`; if it reaches `failed_hop_limit`, give up the edge the MF1 way (`brain.mark(..,True)` + `_stamp_loco_wall` + `committed.discard`) and pop that edge's count.
- On **successful arrival**: capture `prev = self.cell` **before** `self.cell = self.hop_target` (≈:424) and clear `failed_hops` for `(prev, dirn)` **both reps** — never the wholesale `hop_attempts.clear()` (else every ping-pong arrival wipes the cross-occupation count and C5 no-ops on exactly the F3 case — MF6).
- Wherever `_unstick` **reopens** an edge, clear `failed_hops` for it (both reps), else it re-walls after a single post-reopen fail.

Conservative by construction (N *consecutive* fails, lowest-trust reopenable wall, cleared on success/reopen) to avoid the false-wall regression that motivated removing per-failure stamping.

## 5. Data structures & tunables
- C1 state above. `recent` is the only window structure (rolling, pruned in `_confined`). The DFS `trail`/`retreat_path` of the prior draft are **deleted** (MF3).
- `prev_cell` (from `_track_cell`) is the sole reverse target — always adjacent & known-open when `can_reverse`.
- Tunables: `no_progress_s` (§7), `confine_k = 6`, `max_escape_tier = 2`, `failed_hop_limit = 3`.

## 6. Data flow (one tick)
1. `step()`: yaw-rate, then `_track_cell(t)` → seed/visited/explore_t/escape_tier-on-new-ground, prev_cell, recent.
2. **C2 check** (after `_track_cell`, before done/exit return): if not done/at-exit, not mid-escape-reverse, `t-explore_t > no_progress_s`, and `_confined(t)` → `_escape(pose,t)`; return.
3. Else done/exit early-return (`:133-135`), then dispatch by phase (`center`/`turn`/`drive`/`recover`/`backout`/`stuck`).
4. `_escape`: `escape_count += 1`, `escape_tier = min(+1, cap)`, `explore_t = t`; Tier≥2 gives up the blocked edge (MF1); reverse to `prev_cell` via `backout`, or `_unstick` once if no reverse.
5. `_drive` arrival → `mark_traversal` (kept) → clear `failed_hops[(prev,dirn)]` (MF6) → advance `self.cell` (→ `_track_cell` adds it to `visited` next call). `_drive` failure → bump `failed_hops`, maybe give up edge (C5).
6. `_unstick` reopen → reset `escape_tier=0`, clear reopened edges' `failed_hops`.

## 7. Calibration (`no_progress_s`, `confine_k`)
Measure, don't guess, and **across all 5 (drift, latency) cases (MF12)** — the clean (drift=0) case has the fewest recovery cycles, so calibrating only on it risks a false-fire on (0.05, 3). Instrument `_run` to record, per case: (a) the max no-new-cell stretch (`t` between successive `visited` growths) and (b) the rolling distinct-cell footprint during those stretches. Set `no_progress_s` comfortably above the worst-case (a) over all 5; set `confine_k` above the ping-pong footprint but below a legitimate backtrack's footprint (the (a)-stretches on the clean solve are legitimate backtracks — their footprint is the lower bound for "not confined"). The regression then asserts `escape_count == 0` on all 5 cases. If any fires, raise `no_progress_s` (or lower `confine_k` only if it's a footprint false-positive) — never relax the silent assertion. **Worst-case escalation bound:** with the confinement gate a confined churn fires at `no_progress_s` and reaches Tier-2 at `2·no_progress_s`; assert this bound in the calibration so it stays under "no multi-minute dwell" (§2).

## 8. Error handling / edge cases
- **Startup:** `explore_t` seeds on tick 1 (unconditional, MF2); the entrance enters `visited` immediately; rapid new-ground keeps resetting → no false-fire.
- **R3 termination invariant (SF10):** the C5-loco-wall ↔ `_unstick`-reopen oscillation is bounded by **`self.reopened` being monotonic** (each cut edge reopened ≤ once; exhaustion → `stuck`, `maze_motion.py:354`), so total reopens are O(#edges) and the loop provably halts. It is **NOT** bounded by the no-progress backstop (which resets on any new ground). **Forbid** any future change that clears `self.reopened` on progress — it would reintroduce unbounded livelock. (Tested in §9.)
- **Genuine dead-maze:** the NH14 latched stop handles "fully explored + disconnected"; otherwise C2 keeps motion recoverable across map changes.
- **Bounded escalation:** `escape_tier` capped at `max_escape_tier`; no unbounded escalation.

## 9. Testing
ROS-free unit + offline-regression, mirroring the existing suite.

**Test migration (MF7) — do FIRST so the suite collects:** delete/replace the obsolete stall-watchdog tests (`test_maze_motion.py:360-422`) and rewire `test_maze_motion_sim.py` (`_run` returns `m.escape_count` as the 6th field; lines 38/46/57/61 updated; assertion becomes `esc == 0`).

**Unit (new):**
- **C1:** `visited`/`explore_t` grow only on new ground; revisits don't reset `explore_t`; `escape_tier` resets to 0 on new ground; **startup test** — after tick 1 with NO cell change, `explore_t` is non-None and `ENTRANCE_CELL ∈ visited` (MF2). `_confined`: ≤K distinct in-window → True; a swept >K footprint → False; single frozen cell → True.
- **C2:** no-progress + confined > `no_progress_s` triggers `_escape` from `center` AND from `stuck` (the F2 regression) — these tests must seed `visited` to INCLUDE the stuck/current cell + a stale `explore_t` (realistic state; MF2). Never fires at `EXIT_CELL`/`done` (NH13). Startup large-absolute-clock tick-1 does not fire. Not-confined (large footprint) does not fire even past the window.
- **C3 Tier-1:** fire with adjacent open `prev_cell` → `backout` toward `prev_cell`, `_escape_backout=True`, **edge still OPEN**, `explore_t` reset, `escape_count` incremented, `escape_tier==1`.
- **C3 Tier-2:** second fire before progress (`escape_tier` already 1) → blocked edge **`brain.is_wall` True** (MF1) + reverse; `escape_tier==2`. Cap: third fire stays `==2` (MF11).
- **C3 fallbacks:** null/non-adjacent/walled `prev_cell` → `_unstick` (once, MF5).
- **C4:** a `stuck` robot escapes within `no_progress_s` (C2 re-fires; no permanent freeze); `_unstick` reopen resets `escape_tier` to 0 (MF11). MF5: in one tick, `_escape`→`_unstick` does not re-enter `_escape`. SF10: an impassable-but-map-critical edge ends in bounded `stuck` (reopens ≤ #edges). NH14: fully-explored+disconnected latches a permanent stop exempt from re-fire.
- **C5:** `failed_hop_limit` consecutive *cross-occupation* failed hops → `brain.is_wall` True (test sets `max_hop_attempts` high so only `failed_hops` is under test, per `test_failed_hop_does_not_inflate_traversal`); a successful hop clears only that edge's count keyed on the pre-advance cell (MF6); one/two fails don't wall; an `_unstick` reopen clears the edge's `failed_hops`.
- **`_backout` guard:** an escape-reverse timeout does not charge `backout_attempts` and does not inflate `backout_count`; `_escape_backout` clears on leaving `backout`.
- **NH15 (Tier-1 accounting):** a Tier-1 cycle leaves the forward edge's traversal count unchanged (reverse leg intentionally uncounted) and does not disconnect the map.

**Offline regression (`test_maze_motion_sim.py`):**
- All 5 (drift, latency) cases reach the exit, no collision, `max_desync ≤ 1`, **and `escape_count == 0`** (calibration gate, §7).
- `test_backout_is_exercised_end_to_end` stays green.
- **Churn fixture (MF8) — replaces the unbuildable "drift induces ping-pong" test.** MazeSim raycasts ground truth (no false `front_block`), so use either/both: (a) an **FSM-synthetic harness** (no physics) — seed `visited`/`last_seen_cell`/`prev_cell`/`recent`/`explore_t`, script revisits of already-visited cells with `t` advancing past `no_progress_s`, assert `_escape` escalates Tier-1→Tier-2 then **liveness**: `len(visited)` strictly grows past the region (or the latch fires); and (b) a **brain-seeded** scenario on the real maze that pre-marks a false WALL at a junction so `flood`/`next_cell` oscillates between two already-visited reachable cells, then asserts the watchdog fires, `escape_count` increments, and within a bounded step budget `len(m.visited)` strictly increases. Liveness predicate stated explicitly: visited strictly grows after the escape; the map mutates within N ticks of any `stuck`.

## 10. Risks / open questions (resolved by review unless noted)
- **R1 (Tier-2 complexity):** RESOLVED — dropped the trail-retreat; Tier-2 = give-up-edge + one-cell reverse + flood-reroute (MF3).
- **R2 (slow-dribble / long-backtrack):** RESOLVED — confinement predicate added (SF9 → adopted; §4 C1).
- **R3 (loco-wall ↔ reopen livelock):** RESOLVED — bounded by monotonic `self.reopened` (SF10, §8).
- **R4 (loco-walls × arrival-time `mark_traversal`):** benign — the Trémaux cap is soft (`pool = legal if legal else cands`), no new disconnection (NH15 documents Tier-1's asymmetric accounting).
- **Open (monitor in Gazebo):** `confine_k`/`no_progress_s` are calibrated offline but the real churn footprint may differ; if a benign fire or a missed churn is seen in Gazebo, retune (the watchdog escape is safe/bounded regardless).

## 11. Revisions (folded from the 2026-06-23 adversarial review)
**Must-fixes (8):**
- **MF1** — every edge give-up/deprioritize pairs `brain.mark(..,is_wall=True)` with `_stamp_loco_wall` + `committed.discard` (a loco-wall alone is a routing no-op). §3, §4 C3/C5.
- **MF2** — `_track_cell`'s seed + visited-update run UNCONDITIONALLY (else entrance never seeds `explore_t` → F2 returns). §4 C1; startup tests added.
- **MF3** — Tier-2 trail-retreat was unsound (`trail` is a DFS preorder, non-adjacent → teleports tracker / off-cardinal collisions). Replaced with give-up-edge + one-cell reverse + flood-reroute; `trail`/`retreat_path` deleted. §4 C3, §5.
- **MF4** — `_escape` resets `explore_t` on every entry; C2 guarded by `not self._escape_backout` (no busy-re-fire / mid-escape abort). `blocked_dir` source pinned to `hop_dir` at fire time with a stale/None fallback. §4 C2/C3.
- **MF5** — `_unstick`↔Tier-2 mutual recursion removed; `_escape` calls `_unstick` at most once/tick, `_unstick`-exhaustion sets `stuck` and returns (C2 revives next tick). §4 C3/C4, §6.
- **MF6** — `failed_hops` cleared per-edge with the pre-advance key (not the wholesale `hop_attempts.clear()`); distinguished from `hop_attempts`; cleared on `_unstick` reopen. §4 C5.
- **MF7** — added monotonic `escape_count` observable (distinct from `escape_tier`, not reusing `backout_count`); plumbed through `_run`; obsolete stall tests migrated so the suite collects. §4 C1, §9.
- **MF8** — replaced the non-constructible offline churn test (MazeSim has no false-`front_block`) with an FSM-synthetic harness + a brain-seeded false-wall fixture, with an explicit liveness predicate. §9.

**Should-fixes (folded):** SF9 — confinement predicate adopted (user-approved; §4 C1, §7). SF10 — `self.reopened` documented as the real R3 termination invariant, clearing-on-progress forbidden (§8). SF11 — `escape_tier` cap pinned (`min(+1, max_escape_tier)`) + reset on `_unstick` reopen for post-stuck Tier-1 retry (§4 C3/C4). SF12 — calibrate `no_progress_s`/`confine_k` across all 5 cases (§7).

**Nice-to-haves (folded):** NH13 — exact C2 insertion point pinned (after `_track_cell`, before done/exit return) so the guard is non-vacuous. NH14 — latched permanent-terminal predicate for the fully-explored/unsolvable maze. NH15 — documented Tier-1's asymmetric (reverse-leg uncounted) traversal accounting + test.

**Overall review verdict:** SOUND to proceed after folding the must-fixes — no architectural rethink; all four lenses independently confirmed C1/C2/C4 close F1/F2 path-independently and the `escape_tier` escalation sequences correctly.
