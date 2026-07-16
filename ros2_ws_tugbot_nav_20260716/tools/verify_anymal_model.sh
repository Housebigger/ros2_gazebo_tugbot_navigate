#!/usr/bin/env bash
# Headless verification & calibration of the anymal_c model (Task 4).
#
# Starts `gz sim -s -r` on anymal_test_world.sdf and checks, in order:
#   2. the 12 per-joint cmd_pos topics exist
#   3. /scan has no self-occlusion returns (< 8.0 m) on the empty plane
#   6. X-stance foot-bottom calibration (pristine joints, run before item 5!)
#   4. VelocityControl is body-frame (not world-frame)
#   5. joint control actually moves the model (perturbs LF_KFE -- run LAST)
# then kills gz and verifies no orphan process remains.
#
# Items 6 and 5 are executed out of their numbered order on purpose: item 6
# requires the joints to still be sitting untouched at their initial_position
# X-stance ("do NOT command anything first"), but item 5 must command a
# joint to prove control works. So 6 runs before 5 here; the PASS/FAIL table
# is still reported using the original item numbers 2/3/4/5/6.
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
cd "${REPO_ROOT}"

# NOTE: the ROS setup.bash scripts reference unset variables internally, so
# `set -u` must stay OFF while sourcing them.
# shellcheck disable=SC1091
source /opt/ros/jazzy/setup.bash
# shellcheck disable=SC1091
source install/setup.bash
export GZ_SIM_RESOURCE_PATH="${REPO_ROOT}/install/tugbot_description/share/tugbot_description/models:${GZ_SIM_RESOURCE_PATH:-}"

set -u

WORLD_FILE="src/tugbot_gazebo/worlds/anymal_test_world.sdf"
WORLD_NAME="anymal_test"
MODEL_NAME="anymal_c"
JOINTS=(LF_HAA LF_HFE LF_KFE RF_HAA RF_HFE RF_KFE LH_HAA LH_HFE LH_KFE RH_HAA RH_HFE RH_KFE)

LOGDIR="$(mktemp -d /tmp/anymal_verify.XXXXXX)"
GZLOG="${LOGDIR}/gz_sim.log"
echo "Log directory: ${LOGDIR}"

declare -a REPORT
PASS_COUNT=0
FAIL_COUNT=0

record() {
  local item="$1" status="$2" detail="$3"
  REPORT+=("${status}|${item}|${detail}")
  if [[ "${status}" == "PASS" ]]; then
    PASS_COUNT=$((PASS_COUNT + 1))
  else
    FAIL_COUNT=$((FAIL_COUNT + 1))
  fi
  echo "[${status}] ${item} -- ${detail}"
}

GZ_PID=""
cleanup() {
  if [[ -n "${GZ_PID}" ]]; then
    kill "${GZ_PID}" 2>/dev/null || true
  fi
  pkill -f "gz sim -s -r ${WORLD_FILE}" 2>/dev/null || true
  sleep 2
  pkill -9 -f "gz sim -s -r ${WORLD_FILE}" 2>/dev/null || true
  sleep 1
  echo "--- post-cleanup pgrep 'gz sim' ---"
  pgrep -fa "gz sim" || echo "(none -- clean)"
}
trap cleanup EXIT

# ---------------------------------------------------------------------------
# Start gz sim headless
# ---------------------------------------------------------------------------
echo "Starting: gz sim -s -r ${WORLD_FILE}"
nohup gz sim -s -r "${WORLD_FILE}" > "${GZLOG}" 2>&1 &
GZ_PID=$!
sleep 8

if ! kill -0 "${GZ_PID}" 2>/dev/null; then
  record "gz_startup" "FAIL" "gz sim process (pid ${GZ_PID}) died during startup; see ${GZLOG}"
  echo "----- tail of gz log -----"
  tail -60 "${GZLOG}"
  exit 1
fi
record "gz_startup" "PASS" "gz sim running (pid ${GZ_PID})"

TOPIC_LIST="$(timeout 10 gz topic -l 2>/dev/null || true)"

# ---------------------------------------------------------------------------
# Item 2: joint cmd_pos topics
# ---------------------------------------------------------------------------
CMDPOS_COUNT="$(echo "${TOPIC_LIST}" | grep -c 'joint/.*/cmd_pos' || true)"
missing=()
found_pattern=""
for j in "${JOINTS[@]}"; do
  match="$(echo "${TOPIC_LIST}" | grep -F "/model/${MODEL_NAME}/joint/${j}/" | grep 'cmd_pos' || true)"
  if [[ -z "${match}" ]]; then
    missing+=("${j}")
  elif [[ -z "${found_pattern}" ]]; then
    found_pattern="${match}"
  fi
done
if [[ "${CMDPOS_COUNT}" -eq 12 && ${#missing[@]} -eq 0 ]]; then
  record "2_joint_topics" "PASS" "12/12 cmd_pos topics present. NOTE: real topic pattern is /model/${MODEL_NAME}/joint/<J>/0/cmd_pos (gz-sim JointPositionController inserts a joint-index '0' segment; the task text's assumed pattern '/model/${MODEL_NAME}/joint/<J>/cmd_pos' omits it -- example: ${found_pattern})"
else
  record "2_joint_topics" "FAIL" "count=${CMDPOS_COUNT} (want 12), missing=${missing[*]:-none}"
fi

# ---------------------------------------------------------------------------
# Item 3: /scan self-occlusion
# ---------------------------------------------------------------------------
SCAN_MSG="${LOGDIR}/scan.txt"
timeout 10 gz topic -e -t /scan -n 1 > "${SCAN_MSG}" 2>&1
SCAN_FRAME="$(grep -m1 '^frame:' "${SCAN_MSG}" | sed -E 's/frame:\s*"(.*)"/\1/' || true)"
SCAN_RESULT="$(python3 - "${SCAN_MSG}" <<'PYEOF'
import sys
minv = None
n = 0
with open(sys.argv[1]) as f:
    for line in f:
        line = line.strip()
        if line.startswith("ranges:"):
            n += 1
            v = line.split(":", 1)[1].strip()
            if v == "inf":
                continue
            try:
                fv = float(v)
            except ValueError:
                continue
            if minv is None or fv < minv:
                minv = fv
print(f"{n}|{minv if minv is not None else 'inf'}")
PYEOF
)"
SCAN_COUNT="${SCAN_RESULT%%|*}"
SCAN_MIN="${SCAN_RESULT##*|}"
if [[ -z "${SCAN_FRAME}" || "${SCAN_COUNT}" -eq 0 ]]; then
  record "3_scan_self_occlusion" "FAIL" "no /scan message captured (see ${SCAN_MSG})"
elif [[ "${SCAN_MIN}" == "inf" ]]; then
  record "3_scan_self_occlusion" "PASS" "frame=${SCAN_FRAME} beams=${SCAN_COUNT} min_range=inf (no returns at all on the empty plane -- sensor clears own body/mounts; note frame uses '::' separators, e.g. anymal_c::base::scan_omni, not '/')"
else
  BAD="$(python3 -c "print(1 if float('${SCAN_MIN}') < 8.0 else 0)")"
  if [[ "${BAD}" == "1" ]]; then
    record "3_scan_self_occlusion" "FAIL" "frame=${SCAN_FRAME} min_range=${SCAN_MIN} m < 8.0 m -- self-occlusion, sensor z must be raised"
  else
    record "3_scan_self_occlusion" "PASS" "frame=${SCAN_FRAME} min_range=${SCAN_MIN} m (>= 8.0 m)"
  fi
fi

# ---------------------------------------------------------------------------
# Shared pose-parsing helper (used by items 6 and 5)
# Parses `gz topic -e -t /world/<world>/dynamic_pose/info -n 1` text dump.
# Reported link position/orientation are LOCAL, i.e. relative to the MODEL
# frame (verified empirically: the "base" link entry is always exactly
# 0,0,0/identity, and a link's position field only changes across a joint
# whose own axis does NOT pass through that link's origin -- e.g. LF_THIGH's
# position stays bit-identical whether HFE is 0 or 0.4 rad, since the HFE
# joint axis passes through the THIGH origin; only orientation encodes that
# joint's angle. A grandchild's position off that origin does move because
# the ancestor's orientation rotates the fixed offset to it.)
# ---------------------------------------------------------------------------
cat > "${LOGDIR}/geom.py" <<'PYEOF'
import re, sys, math

def parse_dynpose(path):
    text = open(path).read()
    blocks = re.findall(r'pose \{(.*?)\n\}\n', text, re.S)
    poses = {}
    for b in blocks:
        m = re.search(r'name:\s*"([^"]+)"', b)
        if not m:
            continue
        name = m.group(1)
        def block(tag):
            mm = re.search(tag + r' \{(.*?)\}', b, re.S)
            return mm.group(1) if mm else ""
        def field(f, blk, default=0.0):
            mm = re.search(f + r':\s*(-?[0-9.eE+-]+)', blk)
            return float(mm.group(1)) if mm else default
        posb = block('position')
        orib = block('orientation')
        pos = (field('x', posb), field('y', posb), field('z', posb))
        quat = (field('x', orib), field('y', orib), field('z', orib), field('w', orib, 1.0))
        poses[name] = {'pos': pos, 'quat': quat}
    return poses

def quat_to_matrix(q):
    x, y, z, w = q
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]

def mat_vec(M, v):
    return [sum(M[i][j] * v[j] for j in range(3)) for i in range(3)]

def add(a, b):
    return [a[i] + b[i] for i in range(3)]

def world_point(model_pose, link_pose, local_offset):
    """local_offset is expressed in the link's own frame; link_pose is
    relative to the model; model_pose is relative to world."""
    Rlink = quat_to_matrix(link_pose['quat'])
    p_rel_model = add(link_pose['pos'], mat_vec(Rlink, local_offset))
    Rmodel = quat_to_matrix(model_pose['quat'])
    p_world = add(model_pose['pos'], mat_vec(Rmodel, p_rel_model))
    return p_world

def world_z_of_link_origin(model_pose, link_pose):
    Rmodel = quat_to_matrix(model_pose['quat'])
    p_world = add(model_pose['pos'], mat_vec(Rmodel, link_pose['pos']))
    return p_world[2]
PYEOF

# ---------------------------------------------------------------------------
# Item 6: X-stance foot-bottom calibration (MUST run before item 5's
# joint command, so the stance is still pristine / untouched).
# ---------------------------------------------------------------------------
DPOSE_STANCE="${LOGDIR}/dpose_stance.txt"
timeout 10 gz topic -e -t "/world/${WORLD_NAME}/dynamic_pose/info" -n 1 > "${DPOSE_STANCE}" 2>&1

STANCE_OUT="$(PYTHONPATH="${LOGDIR}" python3 - "${DPOSE_STANCE}" "${MODEL_NAME}" <<'PYEOF'
import sys, math
import geom

dpose_path, model_name = sys.argv[1], sys.argv[2]
poses = geom.parse_dynpose(dpose_path)

if model_name not in poses:
    print("ERROR|no model pose entry found")
    sys.exit(0)
model_pose = poses[model_name]

legs = ["LF", "RF", "LH", "RH"]
theta = 0.4  # |HFE| = |KFE relative tilt| in the X-stance, see task geometry note
thigh_len = 0.285
shank_len = 0.345

missing = [l for l in legs if f"{l}_SHANK" not in poses]
if missing:
    print(f"ERROR|missing SHANK link poses in dynamic_pose/info: {missing}")
    sys.exit(0)

shank_world_z = {}
foot_bottom_z = {}
for l in legs:
    shank_pose = poses[f"{l}_SHANK"]
    wz = geom.world_z_of_link_origin(model_pose, shank_pose)
    shank_world_z[l] = wz
    foot_bottom_z[l] = wz - shank_len * math.cos(theta)

vals = list(foot_bottom_z.values())
mean_fb = sum(vals) / len(vals)
max_dev = max(abs(v - mean_fb) for v in vals)
spawn_z_used = model_pose['pos'][2]
recommended_spawn_z = round(spawn_z_used - mean_fb, 2)

symmetric_ok = max_dev <= 0.03
within_ground = all(abs(v) <= 0.08 for v in vals)

status = "PASS" if (symmetric_ok and within_ground) else "FAIL"
detail = (
    f"model_world_z={spawn_z_used:.4f} | "
    + " ".join(f"{l}:shank_z={shank_world_z[l]:.4f},foot_bottom_z={foot_bottom_z[l]:.4f}" for l in legs)
    + f" | mean_foot_bottom_z={mean_fb:.4f} max_pairwise_dev={max_dev:.4f} "
    + f"(symmetric<=0.03:{symmetric_ok}, |z|<=0.08:{within_ground}) | "
    + f"RECOMMENDED_SPAWN_Z={recommended_spawn_z}"
)
print(f"{status}|{detail}")
PYEOF
)"

if [[ "${STANCE_OUT}" == ERROR* ]]; then
  record "6_stance_calibration" "FAIL" "${STANCE_OUT#ERROR|}"
else
  ST_STATUS="${STANCE_OUT%%|*}"
  ST_DETAIL="${STANCE_OUT#*|}"
  record "6_stance_calibration" "${ST_STATUS}" "${ST_DETAIL}"
fi
RECOMMENDED_SPAWN_Z="$(echo "${STANCE_OUT}" | grep -oE 'RECOMMENDED_SPAWN_Z=[0-9.-]+' | cut -d= -f2)"

# ---------------------------------------------------------------------------
# Item 4: VelocityControl body-frame semantics
# ---------------------------------------------------------------------------
read_odom_xy() {
  local outfile="${LOGDIR}/odom_$$_${RANDOM}.txt"
  timeout 10 gz topic -e -t /odom -n 1 > "${outfile}" 2>/dev/null
  python3 - "${outfile}" <<'PYEOF'
import re, sys
text = open(sys.argv[1]).read()
m = re.search(r'pose \{(.*?)\n\}\n', text, re.S)
if not m:
    print("0.0 0.0")
else:
    blk = m.group(1)
    xm = re.search(r'x:\s*(-?[0-9.eE+-]+)', blk)
    ym = re.search(r'y:\s*(-?[0-9.eE+-]+)', blk)
    x = float(xm.group(1)) if xm else 0.0
    y = float(ym.group(1)) if ym else 0.0
    print(f"{x} {y}")
PYEOF
}

read -r X0 Y0 <<< "$(read_odom_xy)"

gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.5}" >/dev/null 2>&1
sleep 3
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.0}" >/dev/null 2>&1
sleep 1

read -r X1 Y1 <<< "$(read_odom_xy)"

DX1="$(python3 -c "print(${X1} - ${X0})")"
DY1_ABS="$(python3 -c "print(abs(${Y1} - ${Y0}))")"
STRAIGHT_OK="$(python3 -c "print(1 if (${DX1} >= 1.0 and ${DY1_ABS} < 0.3) else 0)")"

if [[ "${STRAIGHT_OK}" != "1" ]]; then
  record "4_velocity_body_frame" "FAIL" "straight-drive phase failed: dx=${DX1} (want >=1.0) dy_abs=${DY1_ABS} (want <0.3); x0=${X0} y0=${Y0} x1=${X1} y1=${Y1}. BLOCKED per instructions -- stopping."
  echo "BLOCKED: VelocityControl straight-line phase did not behave as expected. See detail above."
  # print summary then exit (cleanup runs via trap)
  echo "=== SUMMARY (early exit) ==="
  for r in "${REPORT[@]}"; do
    IFS='|' read -r st it det <<< "${r}"
    printf '%-6s %-28s %s\n' "${st}" "${it}" "${det}"
  done
  exit 1
fi

gz topic -t /cmd_vel -m gz.msgs.Twist -p "angular: {z: 0.7854}" >/dev/null 2>&1
sleep 2
gz topic -t /cmd_vel -m gz.msgs.Twist -p "angular: {z: 0.0}" >/dev/null 2>&1
sleep 1

gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.5}" >/dev/null 2>&1
sleep 3
gz topic -t /cmd_vel -m gz.msgs.Twist -p "linear: {x: 0.0}" >/dev/null 2>&1
sleep 1

read -r X2 Y2 <<< "$(read_odom_xy)"
DY2_ABS="$(python3 -c "print(abs(${Y2} - ${Y1}))")"
ROTATED_OK="$(python3 -c "print(1 if ${DY2_ABS} >= 0.5 else 0)")"

if [[ "${ROTATED_OK}" == "1" ]]; then
  record "4_velocity_body_frame" "PASS" "straight: dx=${DX1} dy_abs=${DY1_ABS} (x0=${X0},y0=${Y0} -> x1=${X1},y1=${Y1}); after ~45deg+ turn + drive: dy_abs=${DY2_ABS} >=0.5 (y1=${Y1} -> y2=${Y2}) => body-frame confirmed"
else
  record "4_velocity_body_frame" "FAIL" "WORLD-FRAME SEMANTICS SUSPECTED: after rotating and driving linear.x again, |y| only moved ${DY2_ABS} (want >=0.5). y1=${Y1} y2=${Y2}. Do NOT auto-fix -- fallback (DiffDrive rework) is a controller decision."
  echo "BLOCKED: item 4 (VelocityControl body-frame semantics) FAILED. See detail above."
  echo "=== SUMMARY (early exit) ==="
  for r in "${REPORT[@]}"; do
    IFS='|' read -r st it det <<< "${r}"
    printf '%-6s %-28s %s\n' "${st}" "${it}" "${det}"
  done
  exit 1
fi

# ---------------------------------------------------------------------------
# Item 5: joint control test (perturbs LF_KFE -- run LAST, after item 6)
# Reads LF_SHANK pose before/after and projects the true foot-bottom world
# point through the shank's actual orientation (the exact foot-visual mesh
# offset baked in model.sdf), because the raw LF_SHANK link ORIGIN barely
# moves under KFE (KFE's axis passes through the SHANK's own origin --
# confirmed empirically: origin position is bit-identical, only orientation
# changes). Reading raw origin z would incorrectly show ~0 change and falsely
# fail a working controller, so this is the "reality check" adaptation the
# task instructions explicitly allow.
# ---------------------------------------------------------------------------
DPOSE_BEFORE="${LOGDIR}/dpose_before_kfe.txt"
timeout 10 gz topic -e -t "/world/${WORLD_NAME}/dynamic_pose/info" -n 1 > "${DPOSE_BEFORE}" 2>&1

gz topic -t "/model/${MODEL_NAME}/joint/LF_KFE/0/cmd_pos" -m gz.msgs.Double -p "data: -1.6" >/dev/null 2>&1
sleep 2

DPOSE_AFTER="${LOGDIR}/dpose_after_kfe.txt"
timeout 10 gz topic -e -t "/world/${WORLD_NAME}/dynamic_pose/info" -n 1 > "${DPOSE_AFTER}" 2>&1

JOINT_OUT="$(PYTHONPATH="${LOGDIR}" python3 - "${DPOSE_BEFORE}" "${DPOSE_AFTER}" "${MODEL_NAME}" <<'PYEOF'
import sys
import geom

before_path, after_path, model_name = sys.argv[1], sys.argv[2], sys.argv[3]
poses_before = geom.parse_dynpose(before_path)
poses_after = geom.parse_dynpose(after_path)

if model_name not in poses_before or "LF_SHANK" not in poses_before:
    print("ERROR|missing pose entries in BEFORE dump")
    sys.exit(0)
if model_name not in poses_after or "LF_SHANK" not in poses_after:
    print("ERROR|missing pose entries in AFTER dump")
    sys.exit(0)

# exact foot-visual local offset from model.sdf (LF_SHANK_fixed_joint_lump__LF_FOOT_visual_1)
foot_local_offset = (0.01305, -0.08795, -0.33797)

raw_z_before = poses_before["LF_SHANK"]["pos"][2]
raw_z_after = poses_after["LF_SHANK"]["pos"][2]

foot_before = geom.world_point(poses_before[model_name], poses_before["LF_SHANK"], foot_local_offset)
foot_after = geom.world_point(poses_after[model_name], poses_after["LF_SHANK"], foot_local_offset)

dz_foot = foot_after[2] - foot_before[2]
dz_raw_origin = raw_z_after - raw_z_before

status = "PASS" if abs(dz_foot) > 0.02 else "FAIL"
detail = (
    f"raw LF_SHANK ORIGIN z barely moves (KFE axis passes through it): "
    f"before={raw_z_before:.6f} after={raw_z_after:.6f} delta={dz_raw_origin:.6f} -- "
    f"used instead: projected foot-bottom world z via LF_SHANK pose + real foot-mesh offset: "
    f"before={foot_before[2]:.4f} after={foot_after[2]:.4f} delta={dz_foot:.4f} (want |delta|>0.02)"
)
print(f"{status}|{detail}")
PYEOF
)"

if [[ "${JOINT_OUT}" == ERROR* ]]; then
  record "5_joint_control" "FAIL" "${JOINT_OUT#ERROR|}"
else
  JC_STATUS="${JOINT_OUT%%|*}"
  JC_DETAIL="${JOINT_OUT#*|}"
  record "5_joint_control" "${JC_STATUS}" "${JC_DETAIL}"
fi

# ---------------------------------------------------------------------------
# Summary
# ---------------------------------------------------------------------------
echo ""
echo "=== SUMMARY ==="
for r in "${REPORT[@]}"; do
  IFS='|' read -r st it det <<< "${r}"
  printf '%-6s %-28s %s\n' "${st}" "${it}" "${det}"
done
echo ""
echo "Recommended maze spawn z: ${RECOMMENDED_SPAWN_Z:-unknown}"
echo "PASS=${PASS_COUNT} FAIL=${FAIL_COUNT}"

if [[ "${FAIL_COUNT}" -gt 0 ]]; then
  exit 1
fi
exit 0
