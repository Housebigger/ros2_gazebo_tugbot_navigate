"""Low-speed final approach using the learned navigation pose and live scan."""
import math


def exit_approach_command(pose, exit_xy, target_radius, front_clearance):
    """Return (vx, wz, reached) without access to Gazebo ground truth or walls.

    A discrete cell arrival is only the start of this approach. The target
    radius is smaller than the formal exit region to allow localization and
    gait stopping error. A NaN clearance represents a missing/stale scan.
    """
    if not all(math.isfinite(value) for value in (*pose, *exit_xy, target_radius)):
        return 0.0, 0.0, False
    if math.isnan(front_clearance) or front_clearance <= 0.15:
        return 0.0, 0.0, False
    dx, dy = exit_xy[0]-pose[0], exit_xy[1]-pose[1]
    if math.hypot(dx, dy) <= target_radius:
        return 0.0, 0.0, True
    error = math.remainder(math.atan2(dy, dx)-pose[2], math.tau)
    angular = max(-0.35, min(0.35, 1.2*error))
    linear = 0.12*max(0.0, math.cos(error)) if abs(error) < 0.3 else 0.0
    return linear, angular, False
