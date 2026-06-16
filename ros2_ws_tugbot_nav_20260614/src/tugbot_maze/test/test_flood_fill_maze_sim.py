import math
from tugbot_maze.maze_sim import MazeSim, load_segments, outer_boundary_box, ground_truth_edge_open
from tugbot_maze.flood_fill_brain import (
    FloodFillBrain, ENTRANCE_CELL, EXIT_CELL, DIRS, in_grid, cell_center, pose_to_cell)
from tugbot_maze.hop_controller import hop_command

OUTSIDE_TOL_M = 0.4


def _sense(sim, brain, cell):
    for d, (dx, dy) in DIRS.items():
        nb = (cell[0] + dx, cell[1] + dy)
        brain.mark(cell, d, is_wall=not (in_grid(nb) and ground_truth_edge_open(sim, cell, nb)))


def run(max_hops=400, dt=0.1, max_ticks_per_hop=400):
    segs = load_segments()
    bx0, bx1, by0, by1 = outer_boundary_box()
    start = cell_center(ENTRANCE_CELL)
    sim = MazeSim(segs, start, 0.0, inertia=True)
    brain = FloodFillBrain()
    outside = 0
    for hop in range(max_hops):
        cur = pose_to_cell(sim.x, sim.y)
        _sense(sim, brain, cur)
        if brain.is_done(cur):
            return True, hop, outside
        nxt = brain.next_cell(cur)
        if nxt is None:
            return False, hop, outside
        brain.mark_traversal(cur, nxt)
        target = cell_center(nxt)
        for _ in range(max_ticks_per_hop):
            v, w, arrived = hop_command((sim.x, sim.y, sim.yaw), target)
            sim.step(v, w, dt)
            if max(bx0 - sim.x, sim.x - bx1, by0 - sim.y, sim.y - by1) > OUTSIDE_TOL_M:
                outside += 1
            if arrived:
                break
    return pose_to_cell(sim.x, sim.y) == EXIT_CELL, max_hops, outside


def test_flood_fill_reaches_exit_staying_inside():
    reached, hops, outside = run()
    assert reached, f"flood-fill did not reach the exit cell (hops={hops})"
    assert outside == 0, f"left the maze {outside} times"
