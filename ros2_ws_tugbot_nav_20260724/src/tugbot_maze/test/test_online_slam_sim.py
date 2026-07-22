"""End-to-end CRUX validation: drive MazeMotion through maze_sim to the exit with the
interior wall map WITHHELD from the localizer.

The sim raycasts/collides against the TRUE maze (load_segments()), but the localizer is
seeded with ONLY the perimeter (outer_segments()) plus the brain's online-committed interior
walls -- never the full map. This closes the online-localization loop exactly as the solver
node does. Asserts: reaches exit, no collision, bounded localization error.
"""
import math
from tugbot_maze.maze_motion import MazeMotion
from tugbot_maze.maze_sim import MazeSim, load_segments, outer_segments
from tugbot_maze.flood_fill_brain import ENTRANCE_CELL, EXIT_CELL, cell_center
from tugbot_maze.pose_tracking import odom_prior
from tugbot_maze.online_scan_match_localizer import (
    OnlineScanMatchLocalizer, confirmed_wall_segments, local_reference_cells)


def _run_online_slam(drift, latency=0, dt=0.1, max_steps=30000):
    """Drive MazeMotion on the ONLINE-localized pose. The sim knows the true walls; the
    localizer is seeded with the perimeter only and grows its interior reference from the
    brain's committed walls. Returns (reached, collided, max_loc_err_m)."""
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0,
                  inertia=True, odom_drift_per_m=drift, cmd_latency_steps=latency)
    m = MazeMotion()
    loc = OnlineScanMatchLocalizer(outer_segments(), scan_offset_x=0.0)   # sim scans from body centre
    corrected = (sim.x, sim.y, sim.yaw)              # cold-start bootstrap = true start pose
    last_odom = sim.reported_pose
    t, collided, max_loc_err = 0.0, False, 0.0
    for _ in range(max_steps):
        scan = sim.scan(n_beams=360, fov_rad=2 * math.pi)
        cur_odom = sim.reported_pose
        prior = odom_prior(corrected, last_odom, cur_odom)
        ref_cells = local_reference_cells(m.committed, m.cell, m.sensed)
        interior = confirmed_wall_segments(m.brain, ref_cells)
        corrected, _info = loc.correct(prior, scan[0], scan[1], scan[2], interior)
        last_odom = cur_odom
        max_loc_err = max(max_loc_err, math.hypot(corrected[0] - sim.x, corrected[1] - sim.y))
        v, w, done = m.step(corrected, scan, t)
        if done:
            return True, collided, max_loc_err
        sim.step(v, w, dt)
        if sim.collides(sim.x, sim.y, sim.yaw):
            collided = True
        t += dt
    return (m.cell == EXIT_CELL), collided, max_loc_err


def test_online_slam_reaches_exit_with_map_withheld():
    reached, collided, max_loc_err = _run_online_slam(drift=0.0)
    assert reached, "did not reach the exit on the online self-built map"
    assert not collided, "robot body collided with a wall"
    assert max_loc_err < 0.5, f"online localization error unbounded ({max_loc_err:.2f} m)"


def test_online_slam_under_moderate_drift():
    reached, collided, max_loc_err = _run_online_slam(drift=0.03)
    assert reached and not collided
    assert max_loc_err < 0.5, f"drift=0.03 localization error {max_loc_err:.2f} m (want < 0.5)"
