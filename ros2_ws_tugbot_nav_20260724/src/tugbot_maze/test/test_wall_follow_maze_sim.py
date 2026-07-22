import math
import pytest
from tugbot_maze.wall_follower import WallFollower, sectorize
from tugbot_maze.maze_sim import MazeSim, load_segments, outer_boundary_box
from tugbot_maze.wall_follow_control import entrance_seal_segment

EXIT_XY = (21.072562, 18.083566)
EXIT_RADIUS = 1.2
START_XY = (2.0, 0.0)         # post-ENTRY_DIRECT: ~2 m inside the entrance mouth
START_YAW = 0.0               # facing +x (east), into the maze
# Entrance: 2.072 m gap in the west outer wall, centered map (0.95, 0).
ENTRANCE_SEAL = entrance_seal_segment((0.95, 0.0), 2.072423, 'left')
# Threshold separating a legitimate interior solve from exterior cheating: sealed
# runs stay ~0.5 m INSIDE the box (outside==0 for any tol>0), while the historical
# unsealed cheat ran ~1 m OUTSIDE it -- so any tol in ~(0, 1.0) works; 0.4 sits in the gap.
# (Since the legged 0.49/0.37 footprint, the unsealed cheat is geometrically closed --
# see test_unsealed_left_hand_exterior_cheat_closed_by_legged_footprint below.)
# Coupled to MazeSim wall_half_thickness_m=0.12 AND the footprint constants
# (FOOT_X_FRONT/FOOT_X_REAR +-0.49, FOOT_HALF_W 0.37, which set the collision envelope;
# robot_radius_m=0.35 is unused by the oracle): revisit if any of those change.
OUTSIDE_TOL_M = 0.4


def run_to_exit(follow_side, *, sealed=True, max_steps=20000, dt=0.1, n_beams=72,
                start_xy=START_XY, start_yaw=START_YAW):
    """Run the follower in the sim. Returns (steps_to_exit | None, closest, outside).

    `sealed` closes the entrance gap with ENTRANCE_SEAL. `outside` counts samples
    where the robot center left the outer-wall box by more than OUTSIDE_TOL_M --
    i.e. cheating by following the maze exterior.
    """
    segs = load_segments()
    if sealed:
        segs = segs + [ENTRANCE_SEAL]
    bx0, bx1, by0, by1 = outer_boundary_box()
    sim = MazeSim(segs, start_xy=start_xy, start_yaw=start_yaw)
    assert not sim.collides(start_xy[0], start_xy[1]), "start must be collision-free"
    wf = WallFollower(follow_side=follow_side)
    closest = float('inf')
    outside = 0
    for step in range(max_steps):
        ranges, amin, ainc = sim.scan(n_beams=n_beams)
        sectors = sectorize(ranges, amin, ainc, follow_side)
        cmd = wf.update(sectors)
        sim.step(cmd.v, cmd.w, dt)
        x, y, _ = sim.pose
        if max(bx0 - x, x - bx1, by0 - y, y - by1) > OUTSIDE_TOL_M:
            outside += 1
        d = math.hypot(x - EXIT_XY[0], y - EXIT_XY[1])
        closest = min(closest, d)
        if d <= EXIT_RADIUS:
            return step + 1, closest, outside
    return None, closest, outside


def test_sealed_left_hand_solves_interior():
    steps, closest, outside = run_to_exit('left', sealed=True)
    assert steps is not None, f"sealed left failed to reach exit; closest={closest:.2f} m"
    assert outside == 0, f"sealed left left the maze {outside} times (cheating)"


def test_sealed_right_hand_solves_interior():
    steps, closest, outside = run_to_exit('right', sealed=True)
    assert steps is not None, f"sealed right failed to reach exit; closest={closest:.2f} m"
    assert outside == 0, f"sealed right left the maze {outside} times (cheating)"


def test_unsealed_left_hand_exterior_cheat_closed_by_legged_footprint():
    # HISTORICAL RECORD: pre-2026-07-17, with the 0.39/0.32 footprint, this test locked in WHY
    # the entrance seal was required — the legacy WallFollower squeezed OUTSIDE the unsealed
    # perimeter and "solved" via the exterior (the '5/5 cheat'; see project history) and this
    # asserted `outside > 0`.
    # NEW REALITY: the honest legged dynamic envelope (0.49/0.37) no longer fits through that
    # corridor — the exterior cheat is closed by geometry alone. Characterize that: same
    # unsealed scenario, but the robot now stays inside. The sealed-entrance tests above remain
    # the primary interior-solve guard.
    _, _, outside = run_to_exit('left', sealed=False)
    assert outside == 0, "expected the legged 0.49/0.37 footprint to geometrically close the exterior cheat"


def test_report_faster_legitimate_hand(capsys):
    steps_l, _, out_l = run_to_exit('left', sealed=True)
    steps_r, _, out_r = run_to_exit('right', sealed=True)
    assert steps_l is not None and steps_r is not None
    assert out_l == 0 and out_r == 0
    faster = 'left' if steps_l <= steps_r else 'right'
    with capsys.disabled():
        print(f"\n[GUARANTEE sealed] left={steps_l} right={steps_r} -> FASTER_LEGIT_HAND={faster}")
    # Intentional spec lock on the hand-selection decision — update only after
    # re-verifying BOTH sides still stay inside. left ~5609 < right ~9320 (sealed).
    assert faster == 'left'


@pytest.mark.parametrize("dx,dy,dyaw", [(0.2, 0.2, 0.1), (-0.2, -0.2, -0.1), (0.2, -0.2, 0.1)])
def test_sealed_left_hand_robust_to_start_perturbation(dx, dy, dyaw):
    steps, closest, outside = run_to_exit('left', sealed=True,
                                          start_xy=(START_XY[0] + dx, START_XY[1] + dy),
                                          start_yaw=START_YAW + dyaw)
    assert steps is not None, (
        f"perturbed sealed left failed to reach exit; closest={closest:.2f} m "
        f"(dx={dx}, dy={dy}, dyaw={dyaw})")
    assert outside == 0, f"perturbed sealed left cheated {outside} times"
