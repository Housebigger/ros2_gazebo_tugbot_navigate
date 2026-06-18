"""Offline guarantees for the drift-immune (discrete cell tracking + wall-referenced
re-centering) design. Two properties:

  A. Robust to REPORTED (odom) drift: relative-forward hops + discrete tracking reach
     the exit even when the odom pose diverges heavily — the design never navigates by
     absolute odom, so reported drift can't pull it off course.
  B. Wall-referenced re-centering corrects PHYSICAL off-centering: when the TRUE pose is
     pushed laterally each hop (drive/yaw imperfection), re-centering pulls it back to
     the corridor centre each cell so sensing stays correct and it solves; WITHOUT
     re-centering the lateral error accumulates and it fails.
"""
import math
from tugbot_maze.maze_sim import MazeSim, load_segments
from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, cell_center)
from tugbot_maze.cell_walls import sense_cell_walls
from tugbot_maze.wall_localize import cell_center_offset


def _recenter(sim):
    ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
    ox, oy = cell_center_offset(ranges, amin, ainc, sim.yaw)
    if ox is not None:
        sim.x -= ox
    if oy is not None:
        sim.y -= oy


def _sense(sim, brain, cell):
    ranges, amin, ainc = sim.scan(n_beams=360, fov_rad=2 * math.pi)
    for d, is_wall in sense_cell_walls(ranges, amin, ainc, sim.yaw).items():
        brain.mark(cell, d, is_wall)


def _drive_loop(sim, brain, *, perturb=0.0, drive_reported_2m=False, recenter=True):
    cell = ENTRANCE_CELL
    for _hop in range(400):
        if recenter:
            _recenter(sim)
        if cell == EXIT_CELL:
            return True
        _sense(sim, brain, cell)
        nxt = brain.next_cell(cell)
        if nxt is None:
            return False
        brain.mark_traversal(cell, nxt)
        dx, dy = nxt[0] - cell[0], nxt[1] - cell[1]      # unit cardinal
        sim.yaw = math.atan2(dy, dx)
        if drive_reported_2m:                            # drive until REPORTED odom = ~2 m
            rx0, ry0, _ = sim.reported_pose
            for _ in range(400):
                sim.step(0.5, 0.0, 0.1)
                rx, ry, _ = sim.reported_pose
                if math.hypot(rx - rx0, ry - ry0) >= 2.0:
                    break
        else:                                            # kinematic 2 m forward hop
            sim.x += 2.0 * dx
            sim.y += 2.0 * dy
        if perturb:                                      # lateral TRUE-pose push (perp to travel)
            sim.x += perturb * (-dy)
            sim.y += perturb * dx
        cell = nxt
    return cell == EXIT_CELL


def test_robust_to_reported_odom_drift():
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0, odom_drift_per_m=0.15)
    assert _drive_loop(sim, FloodFillBrain(), drive_reported_2m=True, recenter=True), \
        "relative-hop + discrete design should solve despite heavy reported-odom drift"


def test_recentering_corrects_physical_off_centering():
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    assert _drive_loop(sim, FloodFillBrain(), perturb=0.20, recenter=True), \
        "wall-referenced re-centering should keep it solving under 0.2 m/hop lateral push"


def test_without_recentering_physical_drift_breaks_it():
    sim = MazeSim(load_segments(), cell_center(ENTRANCE_CELL), 0.0)
    assert not _drive_loop(sim, FloodFillBrain(), perturb=0.20, recenter=False), \
        "without re-centering, accumulated lateral drift should break the solve"
