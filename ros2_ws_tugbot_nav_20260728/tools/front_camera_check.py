#!/usr/bin/env python3
"""Live check: /camera/front/image must publish ~20Hz 720x540 frames whose
content is an actual rendered scene (not a constant fill). Dumps the last
frame to /tmp/front_camera_frame.ppm for eyeballing."""
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image


class Check(Node):
    def __init__(self):
        super().__init__('front_camera_check')
        self.msgs = []
        self.create_subscription(Image, '/camera/front/image', self.msgs_cb, 10)

    def msgs_cb(self, msg):
        self.msgs.append((time.monotonic(), msg))


def main():
    rclpy.init()
    node = Check()
    t0 = time.monotonic()
    # 8.0s window = ~160 frames at the nominal 20Hz: plenty for a stable rate
    # estimate while keeping the gate fast.
    while time.monotonic() - t0 < 8.0:
        rclpy.spin_once(node, timeout_sec=0.5)
    msgs = node.msgs
    assert len(msgs) >= 2, f'only {len(msgs)} image msgs in 8s - bridge or sensor dead'
    span = msgs[-1][0] - msgs[0][0]
    hz = (len(msgs) - 1) / span if span > 0 else 0.0
    m = msgs[-1][1]
    print(f'frames={len(msgs)} hz={hz:.1f} {m.width}x{m.height} '
          f'encoding={m.encoding} frame_id={m.header.frame_id}')
    assert m.width == 720 and m.height == 540, (
        f'wrong resolution {m.width}x{m.height} (want 720x540)')
    assert 10.0 <= hz <= 30.0, f'rate {hz:.1f}Hz outside 10-30 (nominal 20)'
    # The static TF in tugbot_gazebo.launch.py publishes base_link ->
    # anymal_c/base/camera_front; a silent rename in model.sdf would detach
    # the camera from TF, so the gate pins the frame id.
    assert m.header.frame_id == 'anymal_c/base/camera_front', (
        f'frame_id drifted: {m.header.frame_id!r} '
        f"(want 'anymal_c/base/camera_front'; the launch static TF must match)")
    data = bytes(m.data)
    # Sample evenly across the WHOLE frame (not just the leading bytes / top
    # rows): the maze has no roof and this lens is very wide-angle (~126 deg
    # horizontal, ~112 deg vertical), so the top ~50 of 540 rows are a
    # genuinely near-flat sky band (observed: 4 unique byte values there)
    # even when the rest of the frame clearly shows walls/floor (observed:
    # 145-146 unique byte values in the middle/lower rows). Sampling only
    # data[:30000] would false-fail on real, correctly-rendered frames.
    sample = data[::39]
    assert len(set(sample)) > 8, 'frame nearly constant - camera not rendering the scene?'
    if m.encoding in ('rgb8', 'bgr8'):
        with open('/tmp/front_camera_frame.ppm', 'wb') as f:
            f.write(b'P6\n%d %d\n255\n' % (m.width, m.height))
            if m.encoding == 'rgb8':
                f.write(data)
            else:
                swapped = bytearray(data)
                swapped[0::3], swapped[2::3] = data[2::3], data[0::3]
                f.write(bytes(swapped))
        print('frame saved: /tmp/front_camera_frame.ppm')
    rclpy.shutdown()
    print('FRONT CAMERA CHECK PASS')


if __name__ == '__main__':
    main()
