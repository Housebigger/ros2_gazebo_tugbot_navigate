#!/usr/bin/env python3
"""Record /maze/explorer_state or /maze/goal_events String JSON payloads as JSONL.

Each output line is a JSON object with wall-clock time, sequence index, and parsed
state/event payload. The script exits when one of these happens:
- max samples reached
- timeout reached
- terminal explorer mode observed after min samples

The default topic remains /maze/explorer_state. For Phase 14 per-goal diagnostics,
use --topic /maze/goal_events; the summary then reports last_event in addition to
last_mode when available.
"""

from __future__ import annotations
import argparse
import json
import sys
import time
from pathlib import Path

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

TERMINAL_MODES = {'EXIT_REACHED', 'FAILED_EXHAUSTED'}


class ExplorerStateRecorder(Node):
    def __init__(self, args: argparse.Namespace) -> None:
        super().__init__('maze_explorer_state_recorder')
        self.args = args
        self.output_path = Path(args.output).expanduser().resolve()
        self.output_path.parent.mkdir(parents=True, exist_ok=True)
        self.started_at = time.time()
        self.count = 0
        self.last_state = None
        self.last_mode = None
        self.last_event = None
        self.terminal_first_seen_at = None
        self.done = False
        self.fp = self.output_path.open('w', encoding='utf-8')
        self.sub = self.create_subscription(String, args.topic, self._on_msg, 10)
        self.timer = self.create_timer(0.2, self._on_timer)
        self.get_logger().info(f'recording {args.topic} to {self.output_path}')

    def _on_msg(self, msg: String) -> None:
        now = time.time()
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError as exc:
            payload = {'_parse_error': str(exc), 'raw': msg.data}
        self.count += 1
        self.last_state = payload
        self.last_mode = payload.get('mode')
        self.last_event = payload.get('event')
        row = {
            'seq': self.count,
            'wall_time': now,
            'elapsed_sec': now - self.started_at,
            'state': payload,
        }
        self.fp.write(json.dumps(row, sort_keys=True) + '\n')
        self.fp.flush()
        if self.count >= self.args.max_samples:
            self.done = True
        if (
            self.args.stop_on_terminal
            and self.count >= self.args.min_samples
            and self.last_mode in TERMINAL_MODES
        ):
            if self.terminal_first_seen_at is None:
                self.terminal_first_seen_at = now
            if now - self.terminal_first_seen_at >= self.args.terminal_linger_sec:
                self.done = True

    def _on_timer(self) -> None:
        if time.time() - self.started_at >= self.args.timeout_sec:
            self.done = True

    def close(self) -> None:
        self.fp.close()


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/maze/explorer_state', help='String JSON topic to record, e.g. /maze/explorer_state or /maze/goal_events')
    parser.add_argument('--output', required=True)
    parser.add_argument('--max-samples', type=int, default=240)
    parser.add_argument('--min-samples', type=int, default=12)
    parser.add_argument('--timeout-sec', type=float, default=240.0)
    parser.add_argument('--stop-on-terminal', action='store_true')
    parser.add_argument('--terminal-linger-sec', type=float, default=0.0)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    rclpy.init()
    node = ExplorerStateRecorder(args)
    interrupted_by_external_shutdown = False
    try:
        while rclpy.ok() and not node.done:
            try:
                rclpy.spin_once(node, timeout_sec=0.2)
            except rclpy.executors.ExternalShutdownException:
                interrupted_by_external_shutdown = True
                break
    finally:
        last_mode = node.last_mode
        last_event = node.last_event
        count = node.count
        out = node.output_path
        node.close()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    print(json.dumps({
        'output': str(out),
        'samples': count,
        'last_mode': last_mode,
        'last_event': last_event,
        'external_shutdown': interrupted_by_external_shutdown,
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    sys.exit(main())
