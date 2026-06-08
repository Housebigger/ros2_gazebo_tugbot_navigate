#!/usr/bin/env python3
"""Phase44 manual Nav2 baseline evidence recorder.

Collects bounded, non-interactive readiness evidence for the manual RViz Nav2
baseline. It does not publish goals and does not start any explorer node.
"""

from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path
from typing import Any

import rclpy
from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformException, TransformListener


FRAME_PAIRS = [("map", "base_link"), ("odom", "base_link"), ("map", "odom")]
CLASSIFICATION_INCONCLUSIVE = "MANUAL_NAV2_INCONCLUSIVE_NEEDS_HUMAN_RVIZ_CHECK"


def quat_to_yaw(q: Any) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def summarize_grid(msg: OccupancyGrid | None, radius_m: float = 1.0) -> dict[str, Any]:
    if msg is None:
        return {"available": False}
    width = int(msg.info.width)
    height = int(msg.info.height)
    res = float(msg.info.resolution)
    data = list(msg.data)
    total = len(data)
    unknown = sum(1 for v in data if int(v) < 0)
    occupied = sum(1 for v in data if int(v) >= 50)
    free = total - unknown - occupied
    summary: dict[str, Any] = {
        "available": True,
        "frame_id": msg.header.frame_id,
        "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        "width": width,
        "height": height,
        "resolution": res,
        "origin": {
            "x": float(msg.info.origin.position.x),
            "y": float(msg.info.origin.position.y),
            "yaw": quat_to_yaw(msg.info.origin.orientation),
        },
        "total_cells": total,
        "known_ratio": (total - unknown) / total if total else None,
        "free_ratio": free / total if total else None,
        "occupied_ratio": occupied / total if total else None,
        "unknown_ratio": unknown / total if total else None,
    }
    # Near map origin / entrance summary, sufficient for baseline startup evidence.
    cx = int(round((0.0 - msg.info.origin.position.x) / res)) if res else 0
    cy = int(round((0.0 - msg.info.origin.position.y) / res)) if res else 0
    cells_radius = int(math.ceil(radius_m / res)) if res else 0
    near: list[int] = []
    for y in range(max(0, cy - cells_radius), min(height, cy + cells_radius + 1)):
        for x in range(max(0, cx - cells_radius), min(width, cx + cells_radius + 1)):
            near.append(int(data[y * width + x]))
    if near:
        n_total = len(near)
        n_unknown = sum(1 for v in near if v < 0)
        n_occupied = sum(1 for v in near if v >= 50)
        n_free = n_total - n_unknown - n_occupied
        summary["near_origin"] = {
            "radius_m": radius_m,
            "sampled_cells": n_total,
            "known_ratio": (n_total - n_unknown) / n_total,
            "free_ratio": n_free / n_total,
            "occupied_ratio": n_occupied / n_total,
            "unknown_ratio": n_unknown / n_total,
        }
    else:
        summary["near_origin"] = {"radius_m": radius_m, "sampled_cells": 0}
    return summary


def summarize_scan(msg: LaserScan | None) -> dict[str, Any]:
    if msg is None:
        return {"available": False}
    finite = [float(v) for v in msg.ranges if math.isfinite(float(v))]
    return {
        "available": True,
        "frame_id": msg.header.frame_id,
        "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        "range_count": len(msg.ranges),
        "finite_count": len(finite),
        "nearest_obstacle_m": min(finite) if finite else None,
        "range_min": float(msg.range_min),
        "range_max": float(msg.range_max),
    }


def summarize_odom(msg: Odometry | None) -> dict[str, Any]:
    if msg is None:
        return {"available": False}
    pose = msg.pose.pose
    twist = msg.twist.twist
    return {
        "available": True,
        "frame_id": msg.header.frame_id,
        "child_frame_id": msg.child_frame_id,
        "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        "pose": {
            "x": float(pose.position.x),
            "y": float(pose.position.y),
            "yaw": quat_to_yaw(pose.orientation),
        },
        "twist": {
            "linear_x": float(twist.linear.x),
            "angular_z": float(twist.angular.z),
        },
    }


def summarize_transform(msg: TransformStamped) -> dict[str, Any]:
    t = msg.transform.translation
    r = msg.transform.rotation
    return {
        "available": True,
        "parent": msg.header.frame_id,
        "child": msg.child_frame_id,
        "stamp_sec": msg.header.stamp.sec + msg.header.stamp.nanosec / 1e9,
        "translation": {"x": float(t.x), "y": float(t.y), "z": float(t.z)},
        "yaw": quat_to_yaw(r),
    }


class Phase44Recorder(Node):
    def __init__(self, duration: float) -> None:
        super().__init__("phase44_manual_nav2_baseline_evidence_recorder")
        self.duration = duration
        self.started = time.time()
        self.map_msg: OccupancyGrid | None = None
        self.local_costmap_msg: OccupancyGrid | None = None
        self.global_costmap_msg: OccupancyGrid | None = None
        self.scan_msg: LaserScan | None = None
        self.odom_msg: Odometry | None = None
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.nav_action = ActionClient(self, NavigateToPose, "navigate_to_pose")
        self.create_subscription(OccupancyGrid, "/map", self._on_map, 10)
        self.create_subscription(OccupancyGrid, "/local_costmap/costmap", self._on_local_costmap, 10)
        self.create_subscription(OccupancyGrid, "/global_costmap/costmap", self._on_global_costmap, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, 10)
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)

    def _on_map(self, msg: OccupancyGrid) -> None:
        self.map_msg = msg

    def _on_local_costmap(self, msg: OccupancyGrid) -> None:
        self.local_costmap_msg = msg

    def _on_global_costmap(self, msg: OccupancyGrid) -> None:
        self.global_costmap_msg = msg

    def _on_scan(self, msg: LaserScan) -> None:
        self.scan_msg = msg

    def _on_odom(self, msg: Odometry) -> None:
        self.odom_msg = msg

    def snapshot(self) -> dict[str, Any]:
        transforms: dict[str, Any] = {}
        for parent, child in FRAME_PAIRS:
            key = f"{parent}->{child}"
            try:
                transform = self.tf_buffer.lookup_transform(parent, child, rclpy.time.Time())
                transforms[key] = summarize_transform(transform)
            except TransformException as exc:
                transforms[key] = {"available": False, "error": str(exc)}
        action_available = bool(self.nav_action.wait_for_server(timeout_sec=0.5))
        forbidden_nodes = []
        for name in self.get_node_names():
            if any(token in name for token in ("maze_explorer", "maze_goal_monitor", "frontier_explorer")):
                forbidden_nodes.append(name)
        return {
            "run_id": "phase44_manual_nav2_baseline_recovery",
            "classification": CLASSIFICATION_INCONCLUSIVE,
            "classification_reason": "bounded readiness evidence collected; final manual Nav2 Goal acceptance requires human RViz operation",
            "elapsed_sec": time.time() - self.started,
            "map": summarize_grid(self.map_msg),
            "local_costmap": summarize_grid(self.local_costmap_msg),
            "global_costmap": summarize_grid(self.global_costmap_msg),
            "scan": summarize_scan(self.scan_msg),
            "odom": summarize_odom(self.odom_msg),
            "tf": transforms,
            "navigate_to_pose_action_available": action_available,
            "forbidden_explorer_nodes_seen": sorted(set(forbidden_nodes)),
            "manual_goal_sent_by_recorder": False,
        }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--duration", type=float, default=30.0)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()

    rclpy.init()
    node = Phase44Recorder(args.duration)
    end = time.time() + args.duration
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    evidence = node.snapshot()
    Path(args.output).parent.mkdir(parents=True, exist_ok=True)
    Path(args.output).write_text(json.dumps(evidence, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
