"""ROS-free point-to-point drive command for a single cell hop, within the real
motion envelope (v,w <= 0.5). P-control on heading; forward speed is throttled by
heading error so the robot turns toward the target before driving (no overshoot)."""
from __future__ import annotations
import math
from typing import Tuple


def _norm(a: float) -> float:
    return math.atan2(math.sin(a), math.cos(a))


def hop_command(pose, target_xy, *, v_max: float = 0.5, w_max: float = 0.5,
                kp_ang: float = 1.5, slow_angle: float = 0.6,
                arrive_m: float = 0.25) -> Tuple[float, float, bool]:
    px, py, yaw = pose
    dx, dy = target_xy[0] - px, target_xy[1] - py
    dist = math.hypot(dx, dy)
    if dist <= arrive_m:
        return (0.0, 0.0, True)
    err = _norm(math.atan2(dy, dx) - yaw)
    w = max(-w_max, min(w_max, kp_ang * err))
    throttle = max(0.0, 1.0 - abs(err) / slow_angle)   # 0 when |err|>=slow_angle
    v = max(0.0, min(v_max, v_max * throttle))
    return (v, w, False)
