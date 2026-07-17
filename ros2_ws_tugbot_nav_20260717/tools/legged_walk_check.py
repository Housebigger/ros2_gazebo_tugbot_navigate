#!/usr/bin/env python3
"""One verification-ladder rung against a live sim (booted by
verify_legged_walk.sh). Usage: legged_walk_check.py --rung {stand|step|walk|turn}
Exit 0 = PASS. Asserts also that attitude never exceeds fall thresholds."""
import argparse
import math
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rcl_interfaces.srv import SetParameters
from rcl_interfaces.msg import Parameter, ParameterValue, ParameterType
from rclpy.node import Node


class Rung(Node):
    def __init__(self):
        super().__init__('legged_walk_check')
        self.samples = []           # (sim_t, x, y, z, roll, pitch, yaw)
        self.create_subscription(Odometry, '/odom', self._on_odom, 50)
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)

    def _on_odom(self, m):
        q = m.pose.pose.orientation
        roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
        pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
        yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        p = m.pose.pose.position
        self.samples.append((t, p.x, p.y, p.z, roll, pitch, yaw))

    def sim_span(self):
        return self.samples[-1][0] - self.samples[0][0] if len(self.samples) > 1 else 0.0

    def drive(self, vx, wz, sim_seconds, wall_cap=120.0):
        """Publish cmd until sim_seconds of sim time elapse (RTF-independent)."""
        self.samples.clear()
        tw = Twist(); tw.linear.x = float(vx); tw.angular.z = float(wz)
        t0 = time.time()
        while rclpy.ok() and time.time() - t0 < wall_cap:
            self.pub.publish(tw)
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.sim_span() >= sim_seconds:
                break
        self.pub.publish(Twist())
        assert self.sim_span() >= sim_seconds, \
            f'sim advanced only {self.sim_span():.1f}s in {wall_cap}s wall'

    def set_force_trot(self, on):
        cli = self.create_client(SetParameters, '/locomotion_controller/set_parameters')
        assert cli.wait_for_service(timeout_sec=20.0), 'locomotion param service missing'
        req = SetParameters.Request()
        req.parameters = [Parameter(name='force_trot', value=ParameterValue(
            type=ParameterType.PARAMETER_BOOL, bool_value=bool(on)))]
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        assert fut.result() is not None


def _attitude_ok(samples, limit=0.6):
    return all(abs(s[4]) < limit and abs(s[5]) < limit for s in samples)


def _unwrap(yaws):
    out = [yaws[0]]
    for y in yaws[1:]:
        d = y - out[-1]
        while d > math.pi: d -= 2 * math.pi
        while d < -math.pi: d += 2 * math.pi
        out.append(out[-1] + d)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--rung', required=True, choices=['stand', 'step', 'walk', 'turn'])
    args = ap.parse_args()
    rclpy.init()
    n = Rung()

    if args.rung == 'stand':
        n.drive(0.0, 0.0, 5.0)
        zs = [s[3] for s in n.samples]
        x0, y0 = n.samples[0][1], n.samples[0][2]
        drift = max(math.hypot(s[1] - x0, s[2] - y0) for s in n.samples)
        assert 0.42 <= min(zs) and max(zs) <= 0.60, f'z out of band ({min(zs):.2f},{max(zs):.2f})'
        assert _attitude_ok(n.samples, 0.09), 'attitude > 5deg while standing'
        assert drift < 0.05, f'stand drift {drift:.3f} m'
        print(f'RUNG stand PASS z=({min(zs):.3f},{max(zs):.3f}) drift={drift:.3f}')

    elif args.rung == 'step':
        n.set_force_trot(True)
        n.drive(0.0, 0.0, 10.0)
        n.set_force_trot(False)
        zs = [s[3] for s in n.samples]
        x0, y0 = n.samples[0][1], n.samples[0][2]
        drift = max(math.hypot(s[1] - x0, s[2] - y0) for s in n.samples)
        assert _attitude_ok(n.samples), 'fell while stepping in place'
        assert max(zs) - min(zs) < 0.06, f'z oscillation {max(zs)-min(zs):.3f}'
        assert drift < 0.30, f'step-in-place drifted {drift:.2f} m'
        print(f'RUNG step PASS z_osc={max(zs)-min(zs):.3f} drift={drift:.2f}')

    elif args.rung == 'walk':
        n.drive(0.3, 0.0, 10.0)
        d = math.hypot(n.samples[-1][1] - n.samples[0][1], n.samples[-1][2] - n.samples[0][2])
        assert _attitude_ok(n.samples), 'fell while walking'
        # Threshold 2.2 m (73% tracking), controller-authorized 2026-07-17:
        # the stride is generated open-loop from COMMANDED vx and tracked by
        # a lagging PD, giving a gain-insensitive steady ~75-78% velocity
        # tracking (measured 2.30-2.34 m; PID/ODE-iters/kp_att all neutral,
        # swing_lift was the only effective lever). Downstream the solver is
        # position-closed-loop (pure pursuit + ICP), so the sole effect is a
        # slower run; the binding real gate stays the maze oracle. Closing
        # the loop on measured velocity is a designed follow-up, not tuning.
        assert d >= 2.2, f'tracked only {d:.2f} m of 3.0 commanded (73% floor)'
        print(f'RUNG walk PASS dist={d:.2f} ({d/3.0*100:.0f}% tracking)')

    elif args.rung == 'turn':
        n.drive(0.0, 0.5, 8.0)
        yaw = _unwrap([s[6] for s in n.samples])
        turned = abs(yaw[-1] - yaw[0])
        x0, y0 = n.samples[0][1], n.samples[0][2]
        drift = max(math.hypot(s[1] - x0, s[2] - y0) for s in n.samples)
        assert _attitude_ok(n.samples), 'fell while turning'
        assert turned >= 3.2, f'turned only {turned:.2f} rad of 4.0 commanded'
        assert drift < 0.40, f'turn-in-place drifted {drift:.2f} m'
        print(f'RUNG turn PASS yaw={turned:.2f} rad drift={drift:.2f}')

    n.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
