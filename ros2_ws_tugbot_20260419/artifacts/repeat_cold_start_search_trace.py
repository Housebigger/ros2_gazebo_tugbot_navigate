import json
import os
import re
import select
import signal
import subprocess
import time
from pathlib import Path

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

WORKDIR = Path('/home/hyh/Desktop/playground_hermes/tugbot_ros2_gazebo/ros2_ws_tugbot_20260419')
WORLD = WORKDIR / 'install' / 'tugbot_gazebo' / 'share' / 'tugbot_gazebo' / 'worlds' / 'tugbot_lane_world_zeroerr_outer.sdf'
OUT_JSON = WORKDIR / 'artifacts' / 'repeat_cold_start_search_summary.json'
ATTEMPTS = 4
MAX_SECONDS = 42.0
POST_REACQUIRE_SECONDS = 4.0
LANE_RE = re.compile(r'lane_center=(?P<center>None|\d+) error=(?P<error>missing|[-+0-9.eE]+)')


def classify_cmd(sample: dict) -> str:
    linear_x = float(sample['linear_x'])
    angular_z = float(sample['angular_z'])
    if abs(linear_x) <= 0.02 and 0.30 <= angular_z <= 0.40:
        return 'spin'
    if abs(linear_x - 0.08) <= 0.03 and 0.15 <= angular_z <= 0.25:
        return 'arc_pos'
    if abs(linear_x - 0.08) <= 0.03 and -0.25 <= angular_z <= -0.15:
        return 'arc_neg'
    return 'pid_or_other'


def compress_phases(cmd_samples: list[dict]) -> list[dict]:
    phases = []
    for sample in cmd_samples:
        label = classify_cmd(sample)
        if not phases or phases[-1]['label'] != label:
            phases.append({
                'label': label,
                'start_t': sample['t'],
                'end_t': sample['t'],
                'count': 1,
                'first_sample': sample,
                'last_sample': sample,
            })
        else:
            phases[-1]['end_t'] = sample['t']
            phases[-1]['count'] += 1
            phases[-1]['last_sample'] = sample
    return phases


def cleanup_ros() -> None:
    patterns = [
        'ros2 launch tugbot_bringup full_system.launch.py',
        'tugbot_perception/.*/lane_detector_node',
        'tugbot_control/.*/lane_controller_node',
        'ros_gz_bridge/parameter_bridge',
        'ros_gz_image/image_bridge',
        'gz sim',
    ]
    pattern = '|'.join(patterns)
    cmd = f"""
    pids=$(ps -eo pid,cmd | grep -E '{pattern}' | grep -v grep | awk '{{print $1}}' || true)
    if [ -n "$pids" ]; then
      echo "$pids" | xargs -r kill -TERM || true
      sleep 2
      rem=$(ps -eo pid,cmd | grep -E '{pattern}' | grep -v grep | awk '{{print $1}}' || true)
      if [ -n "$rem" ]; then
        echo "$rem" | xargs -r kill -KILL || true
      fi
    fi
    source /opt/ros/jazzy/setup.bash
    if [ -f install/setup.bash ]; then source install/setup.bash; fi
    ros2 daemon stop || true
    sleep 1
    ros2 daemon start || true
    sleep 1
    ros2 node list || true
    """
    subprocess.run(['/bin/bash', '-lc', cmd], cwd=str(WORKDIR), capture_output=True, text=True, timeout=40)


class Probe(Node):
    def __init__(self, start_time: float) -> None:
        super().__init__('repeat_cold_start_search_probe')
        self.start_time = start_time
        self.cmd_samples: list[dict] = []
        self.err_samples: list[dict] = []
        self.create_subscription(Twist, '/cmd_vel', self.cmd_cb, 50)
        self.create_subscription(Float32, '/lane_tracking/error', self.err_cb, 50)

    def rel_t(self) -> float:
        return round(time.time() - self.start_time, 3)

    def cmd_cb(self, msg: Twist) -> None:
        self.cmd_samples.append({
            't': self.rel_t(),
            'linear_x': round(float(msg.linear.x), 6),
            'angular_z': round(float(msg.angular.z), 6),
        })

    def err_cb(self, msg: Float32) -> None:
        self.err_samples.append({
            't': self.rel_t(),
            'error': round(float(msg.data), 6),
        })


def run_attempt(idx: int) -> dict:
    cleanup_ros()
    start = time.time()
    launch_cmd = (
        'source /opt/ros/jazzy/setup.bash && '
        'source install/setup.bash && '
        f'ros2 launch tugbot_bringup full_system.launch.py world_sdf:={WORLD}'
    )
    proc = subprocess.Popen(
        ['/bin/bash', '-lc', launch_cmd],
        cwd=str(WORKDIR),
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        bufsize=1,
        preexec_fn=os.setsid,
    )

    node = Probe(start)
    log_lines = []
    bridge_seen = False
    reacquire_time = None
    stop_after = None

    try:
        while time.time() - start < MAX_SECONDS:
            rclpy.spin_once(node, timeout_sec=0.05)
            if proc.stdout is not None:
                while True:
                    ready, _, _ = select.select([proc.stdout], [], [], 0)
                    if not ready:
                        break
                    line = proc.stdout.readline()
                    if not line:
                        break
                    rel_t = round(time.time() - start, 3)
                    stripped = line.rstrip('\n')
                    log_lines.append({'t': rel_t, 'line': stripped})
                    if 'Creating ROS->GZ Bridge' in stripped:
                        bridge_seen = True
            if proc.poll() is not None and time.time() - start > 3.0:
                break
            if reacquire_time is None and node.err_samples:
                reacquire_time = node.err_samples[0]['t']
                stop_after = reacquire_time + POST_REACQUIRE_SECONDS
            if stop_after is not None and (time.time() - start) >= stop_after:
                break
        phases = compress_phases(node.cmd_samples)
        result = {
            'attempt': idx,
            'bridge_seen': bridge_seen,
            'launch_exit_code_before_cleanup': proc.poll(),
            'cmd_sample_count': len(node.cmd_samples),
            'error_sample_count': len(node.err_samples),
            'first_reacquire_t': reacquire_time,
            'spin_seen': any(p['label'] == 'spin' for p in phases),
            'arc_seen': any(p['label'] in ('arc_pos', 'arc_neg') for p in phases),
            'phase_sequence': phases,
            'cmd_head': node.cmd_samples[:25],
            'cmd_tail': node.cmd_samples[-25:],
            'err_head': node.err_samples[:25],
            'err_tail': node.err_samples[-25:],
            'log_tail': log_lines[-80:],
        }
        attempt_path = WORKDIR / 'artifacts' / f'repeat_cold_start_attempt_{idx}.json'
        attempt_path.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding='utf-8')
        result['artifact'] = str(attempt_path)
        return result
    finally:
        try:
            if proc.poll() is None:
                os.killpg(os.getpgid(proc.pid), signal.SIGTERM)
                for _ in range(40):
                    if proc.poll() is not None:
                        break
                    time.sleep(0.1)
                if proc.poll() is None:
                    os.killpg(os.getpgid(proc.pid), signal.SIGKILL)
        except ProcessLookupError:
            pass
        cleanup_ros()
        node.destroy_node()


def main() -> None:
    OUT_JSON.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    results = []
    try:
        for idx in range(1, ATTEMPTS + 1):
            result = run_attempt(idx)
            results.append(result)
            if result['arc_seen'] and result['first_reacquire_t'] is not None:
                break
    finally:
        rclpy.shutdown()
    summary = {
        'world': str(WORLD),
        'attempt_count': len(results),
        'results': results,
    }
    OUT_JSON.write_text(json.dumps(summary, ensure_ascii=False, indent=2), encoding='utf-8')
    print(json.dumps(summary, ensure_ascii=False))


if __name__ == '__main__':
    main()
