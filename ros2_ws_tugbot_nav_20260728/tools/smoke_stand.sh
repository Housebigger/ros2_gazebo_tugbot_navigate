#!/usr/bin/env bash
# Rung-1 smoke: spawn in the test world, settle 8 sim-s, assert standing.
set +u
WS="$(cd "$(dirname "$0")/.." && pwd)"; cd "$WS"
source /opt/ros/jazzy/setup.bash; source install/setup.bash
export GZ_SIM_RESOURCE_PATH="${WS}/install/tugbot_description/share/tugbot_description/models:${GZ_SIM_RESOURCE_PATH:-}"
pkill -9 -f "gz sim" 2>/dev/null; sleep 1
gz sim -s -r src/tugbot_gazebo/worlds/anymal_test_world.sdf > /tmp/smoke_stand_gz.log 2>&1 &
GZ_PID=$!
sleep 12   # ~8 sim-s at RTF<=1 plus startup
# NOTE: model.sdf's OdometryPublisher publishes to gz topic /odom (not the
# /model/<name>/odometry default) per <odom_topic>/odom</odom_topic>; confirmed
# via `gz topic -l | grep -i odom` while the sim was running.
POSE=$(timeout 10 gz topic -e -n 1 -t /odom)
kill -9 $GZ_PID 2>/dev/null; pkill -9 -f "gz sim" 2>/dev/null
echo "$POSE"
python3 - "$POSE" <<'EOF'
import math, re, sys

def field(block, name, default=0.0):
    # protobuf text format omits fields at their default value (0.0), so a
    # missing x/y/z/w means 0.0, not a parse error.
    m = re.search(rf'{name}:\s*([-\d.e]+)', block)
    return float(m.group(1)) if m else default

t = sys.argv[1]
pos_block = re.search(r'position\s*{([^}]*)}', t, re.S).group(1)
z = field(pos_block, 'z')
ori_block = re.search(r'orientation\s*{([^}]*)}', t, re.S)
ori_block = ori_block.group(1) if ori_block else ''
x = field(ori_block, 'x')
y = field(ori_block, 'y')
zz = field(ori_block, 'z')
w = field(ori_block, 'w', default=1.0)
roll = math.atan2(2*(w*x + y*zz), 1 - 2*(x*x + y*y))
pitch = math.asin(max(-1, min(1, 2*(w*y - zz*x))))
print(f'z={z:.3f} roll={math.degrees(roll):.1f}deg pitch={math.degrees(pitch):.1f}deg')
assert 0.42 <= z <= 0.60, f'not standing: z={z}'
assert abs(math.degrees(roll)) < 5 and abs(math.degrees(pitch)) < 5, 'tilted'
print('STAND SMOKE PASS')
EOF
