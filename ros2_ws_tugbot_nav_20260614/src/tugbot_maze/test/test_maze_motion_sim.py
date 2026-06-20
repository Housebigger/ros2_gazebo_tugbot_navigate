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
from tugbot_maze.hop_controller import side_distances, corridor_follow_command


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
            return True, collided, max_desync, m.backout_count, backout_advances
        sim.step(v, w, dt)
        if sim.collides(sim.x, sim.y):                    # body entered a wall margin (true pose)
            collided = True
        if m.phase == 'center':                           # tracker-vs-physical sync at rest
            tc = pose_to_cell(sim.x, sim.y)
            max_desync = max(max_desync, abs(tc[0] - m.cell[0]) + abs(tc[1] - m.cell[1]))
        t += dt
    return (m.cell == EXIT_CELL), collided, max_desync, m.backout_count, backout_advances


# Gazebo-relevant operating regime: moderate wheel-odom drift (Gazebo's is ~0.25 m total,
# i.e. a few %/m; 0.05 = 5%/m is already a stress) crossed with control-pipeline LATENCY
# (the velocity_smoother + plugin delay that induces rotate-in-place overshoot). The
# discrete cell is re-anchored to the (accurate, at this drift) odom cell each cell, so it
# stays synced under both. Extreme drift (>=10%/m) is out of scope -- it is not
# representative of this Gazebo and conflicts with odom re-anchoring.
@pytest.mark.parametrize("drift,latency", [(0.0, 0), (0.03, 0), (0.05, 0), (0.05, 2), (0.05, 3)])
def test_reaches_exit_without_collision_or_desync(drift, latency):
    reached, collided, max_desync, _, _ = _run(drift, latency)
    assert reached, f"did not reach the exit cell (drift={drift}, latency={latency})"
    assert not collided, f"robot body collided with a wall (drift={drift}, latency={latency})"
    assert max_desync <= 1, f"dcell desynced by {max_desync} (drift={drift}, latency={latency})"


def test_backout_is_exercised_end_to_end():
    """The decisive dead-end back-out must actually fire during the offline solve and advance the
    robot to the parent cell -- guards against a silently-disabled/regressed back-out that still
    happens to reach the exit via the fallback path."""
    reached, collided, _, backout_count, backout_advances = _run(0.0, 0)
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
    assert not sim.collides(sim.x, sim.y)
