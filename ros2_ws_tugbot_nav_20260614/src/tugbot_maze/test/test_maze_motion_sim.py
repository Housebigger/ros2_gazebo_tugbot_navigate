"""End-to-end validation: drive MazeMotion through the inertia+collision maze_sim to the exit.

This is the strong, fast (offline) validator the motion layer needs -- it exercises the real
diff-drive accel limits, the LIDAR raycaster, COLLISIONS, and injectable odom drift, so gains
can be tuned without Gazebo. Asserts: reaches the exit cell; never collides (anti-wedge); and
the discrete cell tracker stays synced to the physical cell.
"""
import math
import pytest
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.flood_fill_brain import (
    ENTRANCE_CELL, EXIT_CELL, cell_center, pose_to_cell)
from tugbot_maze.cell_walls import cell_wall_perp_dist
from tugbot_maze.hop_controller import side_distances, corridor_follow_command, grid_cross_track
from tugbot_maze.scan_match_localizer import ScanMatchLocalizer
from tugbot_maze.pose_tracking import odom_prior


def _run(drift, latency=0, dt=0.1, max_steps=30000):
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0,
                  inertia=True, odom_drift_per_m=drift, cmd_latency_steps=latency)
    m = MazeMotion()
    t = 0.0
    collided = False
    max_desync = 0
    prev_phase = m.phase
    backout_cell = None
    backout_advances = 0
    for _ in range(max_steps):
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        v, w, done = m.step(sim.reported_pose, scan, t)   # navigate on (drifting) reported pose
        if prev_phase != 'backout' and m.phase == 'backout':
            backout_cell = m.cell                         # cell where this back-out began
        elif prev_phase == 'backout' and m.phase != 'backout':
            if backout_cell is not None and m.cell != backout_cell:
                backout_advances += 1                     # a back-out moved us to a new (parent) cell
        prev_phase = m.phase
        if done:
            return True, collided, max_desync, m.backout_count, backout_advances, m.escape_count
        sim.step(v, w, dt)
        if sim.collides(sim.x, sim.y, sim.yaw):                    # body entered a wall margin (true pose)
            collided = True
        if m.phase == 'center':                           # tracker-vs-physical sync at rest
            tc = pose_to_cell(sim.x, sim.y)
            max_desync = max(max_desync, abs(tc[0] - m.cell[0]) + abs(tc[1] - m.cell[1]))
        t += dt
    return (m.cell == EXIT_CELL), collided, max_desync, m.backout_count, backout_advances, m.escape_count


# Gazebo-relevant operating regime: moderate wheel-odom drift (Gazebo's is ~0.25 m total,
# i.e. a few %/m; 0.05 = 5%/m is already a stress) crossed with control-pipeline LATENCY
# (the velocity_smoother + plugin delay that induces rotate-in-place overshoot). The
# discrete cell is re-anchored to the (accurate, at this drift) odom cell each cell, so it
# stays synced under both. Extreme drift (>=10%/m) is out of scope -- it is not
# representative of this Gazebo and conflicts with odom re-anchoring.
@pytest.mark.parametrize("drift,latency", [
    (0.0, 0),
    (0.03, 0),
    (0.05, 0),
    (0.05, 2),
    (0.05, 3),
])
def test_reaches_exit_without_collision_or_desync(drift, latency):
    reached, collided, max_desync, _, _, esc = _run(drift, latency)
    assert reached, f"did not reach the exit cell (drift={drift}, latency={latency})"
    assert not collided, f"robot body collided with a wall (drift={drift}, latency={latency})"
    assert max_desync <= 1, f"dcell desynced by {max_desync} (drift={drift}, latency={latency})"
    assert esc == 0, f"no-progress watchdog fired ({esc}x) on the CLEAN solve (drift={drift}, latency={latency})"


def test_backout_is_exercised_end_to_end():
    """The decisive dead-end back-out must actually fire during the offline solve and advance the
    robot to the parent cell -- guards against a silently-disabled/regressed back-out that still
    happens to reach the exit via the fallback path."""
    reached, collided, _, backout_count, backout_advances, _ = _run(0.0, 0)
    assert reached and not collided
    assert backout_count > 0, "dead-end back-out never fired in the offline solve"
    assert backout_advances > 0, "no back-out advanced the robot to the parent cell"


def test_symmetric_following_converges_to_centerline():
    """Straight N-S corridor (side walls at x=+/-1.0, faces ~0.88 m from centre); start 0.4 m
    off-centre and drive N with symmetric wall-following -> converges onto the centerline
    (|x| shrinks well below the start) without colliding. The offline, deterministic form of the
    centerline-envelope guarantee the Gazebo physical-wedge needs."""
    walls = [(-1.0, -5.0, -1.0, 5.0), (1.0, -5.0, 1.0, 5.0)]   # two parallel side walls
    sim = MazeSim(walls, (0.4, 0.0), math.pi / 2, inertia=True)
    for _ in range(120):                                       # 12 s at dt=0.1
        ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        perp = cell_wall_perp_dist(ranges, amin, ainc, sim.yaw)
        d_left, d_right = side_distances(perp, (0, 1))
        v, w = corridor_follow_command(sim.yaw, math.pi / 2, d_left, d_right)
        sim.step(v, w, 0.1)
    assert abs(sim.x) < 0.2, f"did not converge to centerline: x={sim.x:.3f}"
    assert not sim.collides(sim.x, sim.y, sim.yaw)


def test_grid_centerline_holds_through_open_junction():
    """Open zone: side walls at x=+-2.0 are beyond sensing range (face ~1.5-1.9 m > wall_seen_m=1.3,
    so both UNSEEN -> the fallback path runs) yet collidable (a wrong-sign fallback diverges into a
    wall). Start 0.4 m right of the grid centerline (x = 2*0 = 0) and drive N: the odom-grid fallback
    must re-centre. With the old fallback=0.0 the robot holds the 0.4 m offset and the convergence
    assertion fails."""
    walls = [(-2.0, -5.0, -2.0, 8.0), (2.0, -5.0, 2.0, 8.0)]
    sim = MazeSim(walls, (0.4, 0.0), math.pi / 2, inertia=True)
    collided = False
    for _ in range(120):
        ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        perp = cell_wall_perp_dist(ranges, amin, ainc, sim.yaw)
        d_left, d_right = side_distances(perp, (0, 1))
        fallback = max(-0.40, min(0.40, grid_cross_track(sim.x, sim.y, (0, 0), (0, 1))))
        v, w = corridor_follow_command(sim.yaw, math.pi / 2, d_left, d_right, None, fallback_cross=fallback)
        sim.step(v, w, 0.1)
        if sim.collides(sim.x, sim.y, sim.yaw):
            collided = True
    assert abs(sim.x) < 0.15, f"did not converge via grid fallback: x={sim.x:.3f}"
    assert not collided


def test_grid_centerline_recovers_from_large_offset():
    """0.55 m offset (beyond the 0.40 clamp): must re-centre without stalling/wedging or hitting the
    x=+-2.0 walls -- characterises the clamp's creep-and-steer regime."""
    walls = [(-2.0, -5.0, -2.0, 8.0), (2.0, -5.0, 2.0, 8.0)]
    sim = MazeSim(walls, (0.55, 0.0), math.pi / 2, inertia=True)
    collided = False
    for _ in range(160):
        ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        perp = cell_wall_perp_dist(ranges, amin, ainc, sim.yaw)
        d_left, d_right = side_distances(perp, (0, 1))
        fallback = max(-0.40, min(0.40, grid_cross_track(sim.x, sim.y, (0, 0), (0, 1))))
        v, w = corridor_follow_command(sim.yaw, math.pi / 2, d_left, d_right, None, fallback_cross=fallback)
        sim.step(v, w, 0.1)
        if sim.collides(sim.x, sim.y, sim.yaw):
            collided = True
    assert not collided
    assert abs(sim.x) < 0.2, f"did not recover from large offset: x={sim.x:.3f}"


def test_perimeter_reanchor_recenters_offcenter_in_boundary_cell():
    """Robot 0.35 m off-center in boundary cell (10,5): the perimeter re-anchor's local offset lets
    _center recenter to true x.  Note: x=19.4 collides (rear gripper at -0.468 m hits the W cell wall);
    x=19.65 is the nearest valid off-center start from the west side."""
    import math
    sim = MazeSim(load_segments(), (19.65, 10.0), 0.0, inertia=True)   # (10,5) center (20,10), 0.35 m off in x
    m = MazeMotion(); m.cell = (10, 5); m.phase = 'center'
    t = 0.0
    for _ in range(120):
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        v, w, _ = m.step(sim.reported_pose, scan, t); sim.step(v, w, 0.1); t += 0.1
    assert abs(sim.x - 20.0) < 0.20, f"did not recenter: x={sim.x:.3f}"
    assert not sim.collides(sim.x, sim.y, sim.yaw)


def _run_scan_match(drift, dt=0.1, max_steps=30000):
    segs = load_segments()
    sim = MazeSim(segs, cell_center(ENTRANCE_CELL), 0.0, inertia=True, odom_drift_per_m=drift)
    loc = ScanMatchLocalizer(segs, scan_offset_x=0.0)         # sim scans from body center
    m = MazeMotion()
    t = 0.0
    corrected = sim.pose                                       # known entrance (truth) seed
    last_odom = sim.reported_pose                             # == truth at t0 (drift starts at 0)
    max_pose_err = 0.0
    max_desync = 0
    collided = False
    for _ in range(max_steps):
        ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        cur_odom = sim.reported_pose
        prior = odom_prior(corrected, last_odom, cur_odom)
        corrected, _info = loc.correct(prior, ranges, amin, ainc)
        last_odom = cur_odom
        max_pose_err = max(max_pose_err, math.hypot(corrected[0] - sim.x, corrected[1] - sim.y))
        v, w, done = m.step(corrected, (ranges, amin, ainc), t)
        if done:
            return True, collided, max_desync, max_pose_err
        sim.step(v, w, dt)
        if sim.collides(sim.x, sim.y, sim.yaw):
            collided = True
        if m.phase == 'center':
            tc = pose_to_cell(sim.x, sim.y)
            max_desync = max(max_desync, abs(tc[0] - m.cell[0]) + abs(tc[1] - m.cell[1]))
        t += dt
    return (m.cell == EXIT_CELL), collided, max_desync, max_pose_err


def test_scan_match_pose_tracks_truth_under_high_drift():
    """At 10%/m odom drift (raw odom would be off by >1 cell mid-maze), the corrected
    pose stays locked to truth because it re-anchors to the known map every tick."""
    _reached, _collided, _desync, max_pose_err = _run_scan_match(0.10, max_steps=4000)
    assert max_pose_err < 0.15, f"scan-match pose drifted from truth: {max_pose_err:.3f} m"


def test_scan_match_solves_under_drift_that_desyncs_raw_odom():
    """Closed-loop solve on the corrected pose at 5%/m drift: reaches the exit, the cell
    tracker never desyncs, and the body never collides (accurate pose -> correct centering)."""
    reached, collided, max_desync, max_pose_err = _run_scan_match(0.05)
    assert max_pose_err < 0.15, f"pose drifted from truth: {max_pose_err:.3f} m"
    assert reached, "scan-match solve did not reach the exit"
    assert max_desync <= 1, f"cell tracker desynced by {max_desync}"
    assert not collided, "robot collided during the scan-match solve"


