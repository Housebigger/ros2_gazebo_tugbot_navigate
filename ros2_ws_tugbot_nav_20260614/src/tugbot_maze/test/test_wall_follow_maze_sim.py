import math
import pytest
from tugbot_maze.wall_follower import WallFollower, sectorize
from tugbot_maze.maze_sim import MazeSim, load_segments

EXIT_XY = (21.072562, 18.083566)
EXIT_RADIUS = 1.2
START_XY = (2.0, 0.0)        # post-ENTRY_DIRECT: ~2 m inside the entrance mouth
START_YAW = 0.0              # facing +x (east), into the maze


def run_to_exit(follow_side, max_steps=20000, dt=0.1, n_beams=72,
                start_xy=START_XY, start_yaw=START_YAW):
    segs = load_segments()
    sim = MazeSim(segs, start_xy=start_xy, start_yaw=start_yaw)
    assert not sim.collides(START_XY[0], START_XY[1]), "start must be collision-free"
    wf = WallFollower(follow_side=follow_side)
    closest = float('inf')
    for step in range(max_steps):
        ranges, amin, ainc = sim.scan(n_beams=n_beams)
        sectors = sectorize(ranges, amin, ainc, follow_side)
        cmd = wf.update(sectors)
        sim.step(cmd.v, cmd.w, dt)
        x, y, _ = sim.pose
        d = math.hypot(x - EXIT_XY[0], y - EXIT_XY[1])
        closest = min(closest, d)
        if d <= EXIT_RADIUS:
            return step + 1, closest
    return None, closest


def test_right_hand_reaches_exit():
    steps, closest = run_to_exit('right')
    assert steps is not None, f"right-hand failed to reach exit; closest={closest:.2f} m"


def test_left_hand_reaches_exit():
    steps, closest = run_to_exit('left')
    assert steps is not None, f"left-hand failed to reach exit; closest={closest:.2f} m"


def test_report_faster_hand(capsys):
    steps_r, _ = run_to_exit('right')
    steps_l, _ = run_to_exit('left')
    assert steps_r is not None and steps_l is not None
    faster = 'right' if steps_r <= steps_l else 'left'
    with capsys.disabled():
        print(f"\n[GUARANTEE] right={steps_r} steps, left={steps_l} steps -> "
              f"FASTER_HAND={faster}")
    # By construction the faster hand has the smaller (or equal) step count.
    assert min(steps_r, steps_l) == (steps_r if faster == 'right' else steps_l)


@pytest.mark.parametrize("dx,dy,dyaw", [(0.2, 0.2, 0.1), (-0.2, -0.2, -0.1), (0.2, -0.2, 0.1)])
def test_left_hand_robust_to_start_perturbation(dx, dy, dyaw):
    # Bakes the validated start-perturbation robustness into the regression suite,
    # so a future tuning change that breaks robustness can't pass silently.
    steps, closest = run_to_exit('left',
                                 start_xy=(START_XY[0] + dx, START_XY[1] + dy),
                                 start_yaw=START_YAW + dyaw)
    assert steps is not None, (
        f"perturbed left-hand failed to reach exit; closest={closest:.2f} m "
        f"(dx={dx}, dy={dy}, dyaw={dyaw})")
