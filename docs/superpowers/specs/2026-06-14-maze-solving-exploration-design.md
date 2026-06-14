# Trémaux Maze-Solving Exploration — Design Spec

**Date:** 2026-06-14
**Workspace:** `ros2_ws_tugbot_nav_20260614` (new, forked from `ros2_ws_tugbot_nav_20260522`)
**Status:** Approved design — ready for implementation planning
**Goal:** Reliable **autonomous** maze completion (通关) — `guided_corridor_mode:=false` — held to the GCN bar of **≥ 4/5 clean runs**, on the same 20260522 maze, with GCN kept as a fallback/baseline.

---

## 1. Problem & diagnosis

The `20260522` autonomous DFS explorer gets the robot to within ~7.8 m of the exit, then traps it in a south-east pocket (~`(19,9)`), oscillating 7.8–10.5 m from the exit, never finding the route north. The previous session concluded this needed an "algorithm redesign."

A fresh read of the code refutes that framing. The exploration **brain is already sound**:

- `maze_topology.py` is a clean, ROS-free junction graph with per-branch states, a DFS `visit_stack`, `next_backtrack_target` (stack pop to a junction with untried branches), **and** `dijkstra_nearest_unexplored` / `dijkstra_farthest_unexplored` for global rerouting. The robot is **not** trapped for lack of a reroute algorithm.

The trap is caused by the **layer beneath the brain**:

1. **Unreliable back-out / reroute execution.** Backtrack and reroute are dispatched as a *single long Nav2 goal* (`maze_explorer.py:2266`, `:2394`). In the 1.76 m corridors, the MPPI controller times out on long goals — the exact failure GCN beat with short centerline goals + reactive drive. The robot frequently cannot physically return to the unexplored junction.
2. **False dead-ends.** A real corridor that Nav2 times out on twice gets **blacklisted** (`maze_topology.py:500`, `record_branch_failure` → `BLOCKED` + `blacklist_goal`), permanently deleting the true route to the exit from the "untried" set.

**Conclusion:** "稳定地从死胡同里面回退出去并转入另一条未探索通路" (stably back out of dead ends and turn into another unexplored passage) is precisely the broken part — an **execution + classification reliability** problem, not a missing-theory problem. GCN already proved the locomotion fix; we apply it to autonomy and add a coverage-complete decision discipline.

## 2. Success criteria

- **Primary:** robot reaches the exit fully autonomously (`explorer_type:=tremaux`, `guided_corridor_mode:=false`), **≥ 4/5 clean runs** via `tools/run_solver_maze.sh`.
- No false dead-ends: a real corridor is never permanently discarded because of a Nav2 timeout.
- Reliable dead-end back-out: at a confirmed dead end, the robot reverses out and re-enters a genuinely unexplored passage.
- The maze-solving correctness is provable in **ROS-free unit tests** (milliseconds), including a regression test for the SE-pocket scenario — not only in 15-minute sim runs.

## 3. Approach (chosen: B — clean solver node + Trémaux)

Considered three approaches; all three require the same mandatory reliability layer (short centerline goals + reactive drive; geometry-confirmed dead-ends that never blacklist). They differ in the brain and how much we rebuild:

- **A — Reliability retrofit** on the existing 6530-line `maze_explorer.py`. Fastest, but piles onto a brittle, near-untestable file (the trap that stalled last session). Rejected.
- **B — Clean solver node + Trémaux (CHOSEN).** New, focused, unit-testable node; reuse the proven low-level stack; replace brain+execution with a clean, completeness-guaranteed design.
- **C — Frontier-based coverage explorer.** Coverage-complete but leans entirely on Nav2 in narrow corridors, discards the topology infrastructure, needs the same reliability layer anyway. Rejected as a bigger departure with a similar risk surface.

## 4. Architecture

### 4.1 New workspace

Fork **`src/` only** from `20260522` into `ros2_ws_tugbot_nav_20260614/` (NOT the 363 build + 223 install + 5437 log files currently tracked). Add a real `.gitignore`:

```
ros2_ws_tugbot_nav_20260614/build/
ros2_ws_tugbot_nav_20260614/install/
ros2_ws_tugbot_nav_20260614/log/
ros2_ws_tugbot_nav_20260614/.pytest_cache/
```

Keep both fallbacks runnable: GCN (`guided_corridor_mode:=true`) and legacy `maze_dfs`. The new solver is a third selectable `explorer_type:=tremaux`.

### 4.2 Module decomposition

New code is a self-contained "solver" subsystem, cleanly separate from the legacy explorer. Each unit — *what it does / interface / depends on*:

| Unit | Responsibility | Interface | Depends on |
|---|---|---|---|
| **`maze_solver.py`** (node, thin) | ROS orchestration + control loop. Owns `/map`, TF, `/scan`, Nav2 action client, `/cmd_vel_nav`, goal-event pub. Delegates all decisions to the brain, all motion to the pilot. | ROS topics/actions; `brain.update()`, `pilot.execute()` | rclpy, brain, pilot, perception |
| **`tremaux_solver.py`** (brain, **ROS-free**) | Maze graph with Trémaux edge visit-counts; decides next action. | `update(node, local_topology) → Action` | `maze_topology` (extended), pure Python |
| **`reactive_pilot.py`** (pilot) | Reliable locomotion: short centerline Nav2 goals + 10 Hz reactive fallback + watchdog/unwedge + back-out. | `drive_to(wp)`, `follow_path(wps)`, `back_out(d)` | Nav2 client, `/scan`, perception |
| **`dead_end_classifier`** (in brain or small module) | True wall vs transient Nav2 failure | `is_true_dead_end(local, scan) → bool` | perception |
| `maze_topology.py` (reused, **+`visit_count` on `TopoEdge`**) | Junction graph, Dijkstra reroute | existing API | pure Python |
| `maze_perception.py` (reused core, lines 1–236) | `classify_local_topology`, branch/junction geometry | existing funcs | numpy |
| `corridor_navigator.py` (reused) | GCN fallback path | existing | — |

**Key principle:** the node stays thin and the brain is ROS-free, so maze-solving correctness is proven in millisecond unit tests, not sim runs.

### 4.3 Integration

- `setup.py`: add `maze_solver = tugbot_maze.maze_solver:main`.
- `tugbot_maze_explore.launch.py`: add a `Node` under `IfCondition(PythonExpression(["'", explorer_type, "' == 'tremaux'"]))`, mirroring the `maze_dfs` block (same 13 s `TimerAction`, same readiness params).
- `maze_goal_monitor` (EXIT_REACHED detection) reused unchanged.

## 5. The algorithm

### 5.1 Brain — Trémaux edge-counting on the topology graph

Extend `TopoEdge` with `traversals ∈ {0,1,2}` (once per direction). This **replaces** the failure-driven `BLOCKED`/`BLACKLISTED` states that cause false dead-ends. At each junction, having arrived via edge `e`:

- **Rule A — explore:** if any incident edge has `traversals == 0`, take the best one (exit-bias / novelty as a *tiebreaker only*); it becomes `1`.
- **Rule B — back out:** if none, this junction is locally exhausted.
  - *Primary:* Dijkstra to the nearest junction that still has a `traversals == 0` edge, executed **waypoint-by-waypoint** along known edges (retraced edges → `2`).
  - *Fallback* (graph disconnected / perception gap): strict Trémaux — retrace entry edge `e`, marking it `2`. Always makes progress.
- **Never** traverse a `traversals == 2` edge.
- **Dead end:** geometry-confirmed wall, no other opening → immediately Rule B.
- **Done:** no reachable edge has `traversals == 0` and no undetected openings remain → full coverage.

**Why strictly more powerful than current DFS+Dijkstra:** a corridor is never permanently discarded because Nav2 timed out — only when physically traversed twice or walled. That dissolves the SE-pocket trap.

**Completeness:** in a finite maze every edge is walked ≤ 2×, so traversal terminates having visited every reachable node — including the exit — *modulo* SLAM actually mapping each corridor, which the reliability layer guarantees by physically driving in. Edge-counting also guarantees we never permanently discard a real corridor.

**Implementation note:** add `visit_count` to `TopoEdge` and a thin Trémaux interpretation layer in `tremaux_solver.py`; reuse the existing Dijkstra. The legacy `BLOCKED`/`BLACKLIST` code paths are simply unused by the new brain (keeps `maze_topology` usable by both explorers).

### 5.2 Reliability layer — the two root-cause fixes

**`reactive_pilot.py`** (extracted/generalized from the proven GCN reactive drive):

- `drive_to(wp, min_clearance)` — centerline-snapped **short (~1.5 m)** Nav2 goal; on Nav2 timeout/abort fall back to 10 Hz reactive `/cmd_vel_nav` (aim at centerline-snapped heading, laser safety cone, watchdog 12 s + no-progress/no-rotation 3 s → unwedge back 0.4 m). Returns `SUCCESS / WALL_AHEAD / WEDGED`.
- `follow_path([wp…])` — drive each consecutive graph hop; **this is the waypoint-by-waypoint reroute** that fixes the "one long Nav2 goal times out" bug.
- `back_out(d)` — dedicated **reverse** maneuver out of a dead-end pocket (rear/side laser safety), so the robot never has to U-turn-plan in a 1.76 m corridor; then rotate to the open heading.

**Dead-end classifier (kills false dead-ends):**

- A node is a **true dead end** only when perception reports `DEAD_END` **and** the laser scan corroborates a wall within ~corridor-length.
- A **Nav2 goal failure (timeout/abort/preempt) is NOT a dead-end signal.** It triggers a pilot reactive retry of the same branch. Only if the reactive attempt *also* reaches a geometry-confirmed wall is the edge marked `traversals=2`. No timeout ever blacklists a corridor.

### 5.3 Worked example — escaping the SE pocket (the actual v8 failure)

Robot drives into the pocket via edge `E12` (`traversals=1`), hits the wall at `(19,9)`.

- **Old system:** the northward branch was blacklisted after two MPPI timeouts → no untried branch reachable → oscillates 7.8–10.5 m forever.
- **New system:** wall geometry-confirmed → `back_out()` reverses to the pocket-mouth junction (`E12`→`2`) → Dijkstra finds junction `J7` still has a `traversals==0` edge heading north (never condemned, just timed out before) → `follow_path` drives the few short hops to `J7` reliably → takes the north passage → continues toward the exit.

## 6. Data flow & error handling

### 6.1 Control loop

```
/map (OccupancyGrid) ─┐
TF map→base_link ─────┼─▶ perception.classify_local_topology(map_view, pose)
/scan (LaserScan) ────┘        → LocalTopology{kind, open_directions}
                               ▼
        tremaux_solver.update(node, local_topology) → Action
        { explore_branch(wp) | reroute(path) | back_out | done }
                               ▼
        reactive_pilot.execute(action) ──▶ Nav2 short goal / /cmd_vel_nav
                               → SUCCESS | WALL_AHEAD | WEDGED
                               ▼
        brain updates edge.traversals (success→+1, wall→2) ; publish goal_event
                               ▼
        maze_goal_monitor → EXIT_REACHED (reused, unchanged)
```

Decision tick ~2 Hz; pilot reactive control at 10 Hz (the proven GCN cadence that beats `velocity_timeout`).

### 6.2 Error handling

| Failure | Response |
|---|---|
| Nav2 goal timeout/abort | Pilot reactive retry — never a dead-end signal, never blacklists |
| Reactive wedge (watchdog) | `back_out(0.4 m)` + retry; repeated wedge → soft "blocked-this-visit" (Trémaux retries later), not permanent |
| Geometry-confirmed wall | True dead end → edge `=2` → `back_out` → reroute |
| Reroute hop impassable | Reactive fallback for the hop; else re-run Dijkstra excluding that edge |
| Brain "done" but exit not reached | Full-coverage-without-exit (perception gap) → log loudly + GCN fallback |
| SLAM/TF not ready | Reuse existing 13 s warmup + dispatch-readiness gates; pilot waits |
| max_goals / time budget | Graceful stop with diagnostics |

## 7. Validation strategy

1. **ROS-free unit tests for `tremaux_solver`** (milliseconds) — synthetic graphs assert: traversals ≤ 2, never picks a `2` edge, always routes to a `0`-frontier when one exists, terminates at full coverage. **Includes the SE-pocket regression test** (encode the pocket-with-timed-out-corridors topology; assert the brain routes north instead of oscillating). Proves the core fix without a sim run.
2. **Dead-end classifier unit tests** — wall-geometry → dead-end; timeout → retry.
3. **Pilot tests** — reactive angle/centerline math + back-out geometry (ROS-free where possible).
4. **Bounded sim smoke** (phase137-style) — N consecutive reliable explore/backtrack actions, zero false-blacklists, back-out works at a real dead end.
5. **Reliability harness** `tools/run_solver_maze.sh` (fork of `run_dfs_maze.sh`) — N full runs, tally EXIT_REACHED, **target ≥ 4/5**; closest-approach/oscillation tracker to quantify each iteration.

## 8. Milestones (build order → implementation plan)

- **M0** Workspace `20260614`: fork `src/`, add `.gitignore`, build green, GCN + `maze_dfs` still run.
- **M1** Brain: `tremaux_solver.py` + `TopoEdge.traversals` + full unit suite incl. **pocket-escape regression**. ← correctness gate, all fast/ROS-free.
- **M2** Pilot: `reactive_pilot.py` (drive_to / follow_path / back_out) extracted from GCN + dead-end classifier + tests.
- **M3** Node: thin `maze_solver.py` wiring brain+pilot+perception; `explorer_type:=tremaux` launch; bounded sim smoke gate.
- **M4** Full-maze runs + tune to ≥ 4/5 with the reliability harness.
- **M5** Document + commit + update memory.

## 9. Out of scope

- A new/harder maze (this effort keeps the 20260522 maze constant to isolate the algorithm improvement).
- Re-architecting the Nav2/SLAM/Gazebo stack (reused unchanged).
- Removing GCN or the legacy `maze_dfs` explorer (both retained).

## 10. Open risks

- **Perception completeness:** the completeness guarantee holds modulo SLAM mapping each corridor; narrow-corridor SLAM gaps could still hide an opening. Mitigation: the pilot physically drives into corridors (mapping them); a coverage re-sweep on "done-without-exit."
- **Pilot extraction fidelity:** the GCN reactive drive is embedded in the 6530-line explorer; extraction must preserve the 10 Hz cadence, centerline aiming, and watchdog/unwedge behaviors that made GCN reliable.
- **Sim cost:** ≥ 4/5 reliability validation needs many 15-minute runs; front-load correctness into fast unit tests (M1) to minimize sim iterations.
