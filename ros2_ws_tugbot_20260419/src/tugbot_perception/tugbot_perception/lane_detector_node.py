from __future__ import annotations

from typing import Sequence

try:
    import numpy as np
except ImportError:  # pragma: no cover
    np = None

try:  # pragma: no cover
    import rclpy
    from rclpy.node import Node
    from sensor_msgs.msg import Image
    from std_msgs.msg import Float32
except ImportError:  # pragma: no cover
    rclpy = None
    Image = None
    Float32 = None

    class Node:  # type: ignore[override]
        pass


def _is_blue_pixel(pixel: Sequence[int], blue_threshold: int, blue_margin: int) -> bool:
    red = int(pixel[0])
    green = int(pixel[1])
    blue = int(pixel[2])
    return blue > blue_threshold and blue > red + blue_margin and blue > green + blue_margin


def detect_blue_lane_center(image, blue_threshold: int = 100, blue_margin: int = 40, crop_top_ratio: float = 0.5):
    if np is not None and hasattr(image, 'shape'):
        if len(image.shape) < 3 or image.shape[2] < 3:
            raise ValueError('image array must have at least 3 channels')
        height = int(image.shape[0])
        width = int(image.shape[1])
        start_row = min(height - 1, max(0, int(height * crop_top_ratio)))
        roi = image[start_row:, :, :3]
        roi_int = roi.astype(np.int16, copy=False)
        blue_mask = (
            (roi_int[:, :, 2] > blue_threshold)
            & (roi_int[:, :, 2] > roi_int[:, :, 0] + blue_margin)
            & (roi_int[:, :, 2] > roi_int[:, :, 1] + blue_margin)
        )
        counts = blue_mask.sum(axis=0).astype(int).tolist()
    else:
        height = len(image)
        if height == 0:
            raise ValueError('image array must not be empty')
        width = len(image[0])
        start_row = min(height - 1, max(0, int(height * crop_top_ratio)))
        counts = [0] * width
        for row in image[start_row:]:
            for column_index, pixel in enumerate(row):
                if _is_blue_pixel(pixel, blue_threshold=blue_threshold, blue_margin=blue_margin):
                    counts[column_index] += 1

    if not counts or max(counts) == 0:
        return None

    total = sum(counts)
    return round(sum(index * value for index, value in enumerate(counts)) / total)


def update_detection_state(
    detected_lane_center: int | None,
    prev_lane_center: int | None,
    prev_missing_frames: int,
    detection_hold_frames: int,
) -> tuple[int | None, int]:
    if detected_lane_center is not None:
        return detected_lane_center, 0

    missing_frames = int(prev_missing_frames) + 1
    if prev_lane_center is not None and missing_frames <= int(detection_hold_frames):
        return prev_lane_center, missing_frames

    return None, missing_frames


def compute_lane_error(image_width: int, lane_center: int | None):
    if image_width <= 0:
        raise ValueError('image_width must be positive')
    if lane_center is None:
        return None
    image_center = (image_width - 1) / 2.0
    return (image_center - float(lane_center)) / max(image_center, 1.0)


def image_message_to_array(msg: Image):  # type: ignore[valid-type]
    if np is not None:
        channels = max(len(msg.data) // max(msg.height * msg.width, 1), 1)
        return np.frombuffer(msg.data, dtype=np.uint8).reshape((msg.height, msg.width, channels))

    bytes_per_pixel = max(int(getattr(msg, 'step', 0) / max(msg.width, 1)), 3)
    pixels = []
    offset = 0
    for _ in range(msg.height):
        row = []
        for _ in range(msg.width):
            row.append([msg.data[offset], msg.data[offset + 1], msg.data[offset + 2]])
            offset += bytes_per_pixel
        pixels.append(row)
    return pixels


if rclpy is not None:
    class LaneDetectorNode(Node):
        def __init__(self) -> None:
            super().__init__('lane_detector_node')
            self.declare_parameter('input_topic', '/camera/image_raw')
            self.declare_parameter('error_topic', '/lane_tracking/error')
            self.declare_parameter('debug_image_topic', '/lane_tracking/debug_image')
            self.declare_parameter('blue_threshold', 100)
            self.declare_parameter('blue_margin', 40)
            self.declare_parameter('crop_top_ratio', 0.55)
            self.declare_parameter('detection_hold_frames', 0)
            self.declare_parameter('debug_image_enabled', True)

            input_topic = self.get_parameter('input_topic').value
            error_topic = self.get_parameter('error_topic').value
            self.debug_image_topic = self.get_parameter('debug_image_topic').value

            self.error_publisher = self.create_publisher(Float32, error_topic, 10)
            self.debug_image_publisher = self.create_publisher(Image, self.debug_image_topic, 10)
            self.image_subscription = self.create_subscription(Image, input_topic, self.image_cb, 10)
            self.frames_seen = 0
            self.prev_lane_center: int | None = None
            self.prev_missing_frames = 0

        def image_cb(self, msg: Image) -> None:
            image = image_message_to_array(msg)
            detected_lane_center = detect_blue_lane_center(
                image,
                blue_threshold=int(self.get_parameter('blue_threshold').value),
                blue_margin=int(self.get_parameter('blue_margin').value),
                crop_top_ratio=float(self.get_parameter('crop_top_ratio').value),
            )
            lane_center, self.prev_missing_frames = update_detection_state(
                detected_lane_center=detected_lane_center,
                prev_lane_center=self.prev_lane_center,
                prev_missing_frames=self.prev_missing_frames,
                detection_hold_frames=int(self.get_parameter('detection_hold_frames').value),
            )
            self.prev_lane_center = lane_center
            error = compute_lane_error(msg.width, lane_center)
            if error is not None:
                error_msg = Float32()
                error_msg.data = float(error)
                self.error_publisher.publish(error_msg)
            if bool(self.get_parameter('debug_image_enabled').value):
                self.debug_image_publisher.publish(msg)

            self.frames_seen += 1
            if self.frames_seen <= 3 or self.frames_seen % 20 == 0:
                self.get_logger().info(
                    f'lane_center={lane_center} error={error if error is not None else "missing"}'
                )


def main() -> None:
    if rclpy is None:
        raise RuntimeError('ROS 2 Python dependencies are unavailable; source the ROS environment first')
    rclpy.init()
    node = LaneDetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()
