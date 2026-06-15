#!/usr/bin/env python3
"""Phase53 static entrance ingress waypoint geometry validation.

This script is intentionally offline/static. It reads the active scaled2x maze
metadata, active SDF wall geometry, and the accepted Phase34 SLAM map snapshot.
It does not launch ROS, Gazebo, Nav2, or the autonomous explorer.
"""

from __future__ import annotations

import json
import math
import struct
import xml.etree.ElementTree as ET
from dataclasses import dataclass, asdict
from pathlib import Path
from typing import Any

try:
    import yaml
except Exception as exc:  # pragma: no cover
    raise SystemExit(f"PyYAML is required: {exc}")

ROOT = Path(__file__).resolve().parents[1]
ACTIVE_CONFIG = ROOT / "src/tugbot_maze/config/maze_20260528_scaled_instance.yaml"
ACTIVE_WORLD = ROOT / "src/tugbot_gazebo/worlds/tugbot_maze_world_20260528_clean_scaled2x.sdf"
PHASE34_MAP_YAML = ROOT / "log/phase34_scaled_clean_world_slam_smoke/phase34_scaled_clean_world_slam_smoke_map_snapshot.yaml"
ARTIFACT_DIR = ROOT / "log/phase53_entrance_ingress_waypoint_design"
ARTIFACT_JSON = ARTIFACT_DIR / "phase53_entrance_ingress_waypoint_design.json"
OVERLAY_PNG = ARTIFACT_DIR / "phase53_entrance_ingress_waypoint_overlay.png"

CLASS_VALIDATED = "INGRESS_WAYPOINT_GEOMETRY_VALIDATED"
CLASS_BLOCKED = "INGRESS_WAYPOINT_BLOCKED"
CLASS_INCONCLUSIVE = "INCONCLUSIVE_NEEDS_RUNTIME_CHECK"

CANDIDATE_DISTANCES_M = [0.8, 1.0, 1.2, 1.5]
MIN_WALL_CLEARANCE_M = 0.65
LOCAL_FREE_RADIUS_M = 0.35
LINE_SAMPLE_STEP_M = 0.05
PGM_FREE_MIN_VALUE = 250
PGM_UNKNOWN_VALUE = 205
PGM_OCCUPIED_MAX_VALUE = 10


@dataclass
class WallBox:
    name: str
    x: float
    y: float
    sx: float
    sy: float
    yaw: float


@dataclass
class Candidate:
    name: str
    distance_from_entrance_m: float
    point_map: dict[str, float]
    point_world: dict[str, float]
    inside_map_boundary: bool
    map_cell: list[int] | None
    map_pixel_value: int | None
    occupied_or_unknown: bool
    local_free_ratio_radius_0_35m: float | None
    local_unknown_ratio_radius_0_35m: float | None
    local_occupied_ratio_radius_0_35m: float | None
    min_clearance_to_wall_m: float
    nearest_wall: str
    short_straight_line_reachable: bool
    line_min_clearance_to_wall_m: float
    line_map_free_ratio: float | None
    line_occupied_or_unknown_count: int
    validation_passed: bool
    rejection_reasons: list[str]


def _load_yaml(path: Path) -> dict[str, Any]:
    with path.open("r", encoding="utf-8") as f:
        return yaml.safe_load(f)


def _load_pgm(path: Path) -> tuple[int, int, int, bytes]:
    with path.open("rb") as f:
        magic = f.readline().strip()
        if magic not in {b"P5", b"P2"}:
            raise ValueError(f"unsupported PGM magic {magic!r} in {path}")
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        width, height = [int(v) for v in line.split()]
        max_value = int(f.readline())
        if magic == b"P5":
            data = f.read(width * height)
        else:
            nums = []
            for token in f.read().split():
                nums.append(int(token))
            data = bytes(nums)
        if len(data) != width * height:
            raise ValueError(f"PGM data length mismatch: {len(data)} vs {width * height}")
        return width, height, max_value, data


def _map_to_cell(x: float, y: float, origin: tuple[float, float], resolution: float) -> tuple[int, int]:
    return int(math.floor((x - origin[0]) / resolution)), int(math.floor((y - origin[1]) / resolution))


def _cell_value(cell_x: int, cell_y: int, width: int, height: int, data: bytes) -> int | None:
    if not (0 <= cell_x < width and 0 <= cell_y < height):
        return None
    row = height - 1 - cell_y
    return data[row * width + cell_x]


def _is_free_value(value: int | None) -> bool:
    return value is not None and value >= PGM_FREE_MIN_VALUE


def _parse_wall_boxes(world_path: Path) -> list[WallBox]:
    root = ET.parse(world_path).getroot()
    walls: list[WallBox] = []
    for model in root.findall(".//model"):
        name = model.get("name", "")
        if "wall" not in name:
            continue
        pose_text = model.findtext("pose") or "0 0 0 0 0 0"
        pose_values = [float(v) for v in pose_text.split()]
        x, y, _, _, _, yaw = pose_values[:6]
        size_text = model.findtext(".//collision/geometry/box/size")
        if not size_text:
            continue
        sx, sy, _ = [float(v) for v in size_text.split()]
        walls.append(WallBox(name=name, x=x, y=y, sx=sx, sy=sy, yaw=yaw))
    return walls


def _point_to_axis_aligned_box_clearance(px: float, py: float, wall: WallBox) -> float:
    # The active scaled2x wall boxes are axis-aligned; keep a conservative fallback
    # for tiny yaw noise by treating the stored SDF dimensions as world-aligned.
    dx = max(abs(px - wall.x) - wall.sx / 2.0, 0.0)
    dy = max(abs(py - wall.y) - wall.sy / 2.0, 0.0)
    return math.hypot(dx, dy)


def _nearest_wall_clearance(px: float, py: float, walls: list[WallBox]) -> tuple[float, str]:
    distances = [(_point_to_axis_aligned_box_clearance(px, py, wall), wall.name) for wall in walls]
    return min(distances, key=lambda item: item[0])


def _local_map_ratios(
    x: float,
    y: float,
    radius_m: float,
    origin: tuple[float, float],
    resolution: float,
    width: int,
    height: int,
    data: bytes,
) -> tuple[float, float, float, int, int, int]:
    cx, cy = _map_to_cell(x, y, origin, resolution)
    radius_cells = int(math.ceil(radius_m / resolution))
    total = free = unknown = occupied = 0
    for dy in range(-radius_cells, radius_cells + 1):
        for dx in range(-radius_cells, radius_cells + 1):
            if dx * dx + dy * dy > radius_cells * radius_cells:
                continue
            total += 1
            value = _cell_value(cx + dx, cy + dy, width, height, data)
            if value is None or value == PGM_UNKNOWN_VALUE:
                unknown += 1
            elif value <= PGM_OCCUPIED_MAX_VALUE:
                occupied += 1
            elif _is_free_value(value):
                free += 1
            else:
                unknown += 1
    if total == 0:
        return 0.0, 1.0, 0.0, free, unknown, occupied
    return free / total, unknown / total, occupied / total, free, unknown, occupied


def _line_check(
    start: tuple[float, float],
    end: tuple[float, float],
    origin: tuple[float, float],
    resolution: float,
    width: int,
    height: int,
    data: bytes,
    walls: list[WallBox],
) -> tuple[bool, float, float, int]:
    sx, sy = start
    ex, ey = end
    distance = math.hypot(ex - sx, ey - sy)
    steps = max(2, int(math.ceil(distance / LINE_SAMPLE_STEP_M)) + 1)
    free_count = 0
    bad_count = 0
    min_clearance = float("inf")
    for i in range(steps):
        t = i / (steps - 1)
        x = sx + (ex - sx) * t
        y = sy + (ey - sy) * t
        cx, cy = _map_to_cell(x, y, origin, resolution)
        value = _cell_value(cx, cy, width, height, data)
        if _is_free_value(value):
            free_count += 1
        else:
            bad_count += 1
        clearance, _ = _nearest_wall_clearance(x + WORLD_ENTRANCE_X, y + WORLD_ENTRANCE_Y, walls)
        min_clearance = min(min_clearance, clearance)
    free_ratio = free_count / steps if steps else 0.0
    return bad_count == 0 and min_clearance >= MIN_WALL_CLEARANCE_M, min_clearance, free_ratio, bad_count


# Set by main after metadata load; only used by _line_check to convert map->world.
WORLD_ENTRANCE_X = 0.0
WORLD_ENTRANCE_Y = 0.0


def _make_overlay_png(
    width: int,
    height: int,
    data: bytes,
    origin: tuple[float, float],
    resolution: float,
    candidates: list[Candidate],
    selected: Candidate | None,
    output: Path,
) -> None:
    # Generate a simple RGB PNG with the Phase34 occupancy snapshot and candidate markers.
    import zlib

    scale = 4
    out_w, out_h = width * scale, height * scale
    rgb = bytearray(out_w * out_h * 3)

    def set_px(x: int, y: int, color: tuple[int, int, int]) -> None:
        if 0 <= x < out_w and 0 <= y < out_h:
            idx = (y * out_w + x) * 3
            rgb[idx:idx + 3] = bytes(color)

    for cy in range(height):
        for cx in range(width):
            value = _cell_value(cx, cy, width, height, data)
            if value is None or value == PGM_UNKNOWN_VALUE:
                color = (165, 165, 165)
            elif value <= PGM_OCCUPIED_MAX_VALUE:
                color = (20, 20, 20)
            else:
                color = (245, 245, 245)
            # Convert map cell y-up to image top-left y-down for overlay.
            img_y = height - 1 - cy
            for yy in range(img_y * scale, img_y * scale + scale):
                for xx in range(cx * scale, cx * scale + scale):
                    set_px(xx, yy, color)

    def draw_cross(map_x: float, map_y: float, color: tuple[int, int, int], radius: int = 8) -> None:
        cx, cy = _map_to_cell(map_x, map_y, origin, resolution)
        px = cx * scale + scale // 2
        py = (height - 1 - cy) * scale + scale // 2
        for d in range(-radius, radius + 1):
            for thickness in range(-1, 2):
                set_px(px + d, py + thickness, color)
                set_px(px + thickness, py + d, color)

    def draw_line(a: tuple[float, float], b: tuple[float, float], color: tuple[int, int, int]) -> None:
        ax, ay = _map_to_cell(a[0], a[1], origin, resolution)
        bx, by = _map_to_cell(b[0], b[1], origin, resolution)
        ax = ax * scale + scale // 2
        ay = (height - 1 - ay) * scale + scale // 2
        bx = bx * scale + scale // 2
        by = (height - 1 - by) * scale + scale // 2
        steps = max(abs(bx - ax), abs(by - ay), 1)
        for i in range(steps + 1):
            t = i / steps
            x = round(ax + (bx - ax) * t)
            y = round(ay + (by - ay) * t)
            for ox in range(-1, 2):
                for oy in range(-1, 2):
                    set_px(x + ox, y + oy, color)

    draw_cross(0.0, 0.0, (0, 80, 255), radius=10)
    for candidate in candidates:
        p = candidate.point_map
        color = (255, 165, 0)
        if candidate.validation_passed:
            color = (0, 180, 0)
        draw_cross(p["x_m"], p["y_m"], color, radius=7)
        draw_line((0.0, 0.0), (p["x_m"], p["y_m"]), (0, 120, 255))
    if selected is not None:
        p = selected.point_map
        draw_cross(p["x_m"], p["y_m"], (255, 0, 0), radius=12)

    # Minimal PNG writer, no external plotting dependency.
    raw = bytearray()
    for y in range(out_h):
        raw.append(0)
        row_start = y * out_w * 3
        raw.extend(rgb[row_start:row_start + out_w * 3])

    def chunk(kind: bytes, payload: bytes) -> bytes:
        crc = zlib.crc32(kind + payload) & 0xFFFFFFFF
        return struct.pack("!I", len(payload)) + kind + payload + struct.pack("!I", crc)

    png = b"\x89PNG\r\n\x1a\n"
    png += chunk(b"IHDR", struct.pack("!IIBBBBB", out_w, out_h, 8, 2, 0, 0, 0))
    png += chunk(b"IDAT", zlib.compress(bytes(raw), 9))
    png += chunk(b"IEND", b"")
    output.write_bytes(png)


def main() -> int:
    ARTIFACT_DIR.mkdir(parents=True, exist_ok=True)
    config = _load_yaml(ACTIVE_CONFIG)
    map_meta = _load_yaml(PHASE34_MAP_YAML)
    pgm_path = PHASE34_MAP_YAML.parent / map_meta["image"]
    width, height, max_value, pgm_data = _load_pgm(pgm_path)
    resolution = float(map_meta["resolution"])
    origin = (float(map_meta["origin"][0]), float(map_meta["origin"][1]))
    walls = _parse_wall_boxes(ACTIVE_WORLD)

    global WORLD_ENTRANCE_X, WORLD_ENTRANCE_Y
    map_truth = config["map_frame_truth"]
    world_truth = config["world_frame_truth"]
    entrance_map = map_truth["entrance"]
    exit_map = map_truth["exit"]
    entrance_world = world_truth["entrance"]
    WORLD_ENTRANCE_X = float(entrance_world["x_m"])
    WORLD_ENTRANCE_Y = float(entrance_world["y_m"])
    yaw = float(entrance_map["yaw_rad"])
    direction = (math.cos(yaw), math.sin(yaw))

    candidates: list[Candidate] = []
    for distance_m in CANDIDATE_DISTANCES_M:
        px_map = float(entrance_map["x_m"]) + direction[0] * distance_m
        py_map = float(entrance_map["y_m"]) + direction[1] * distance_m
        px_world = WORLD_ENTRANCE_X + px_map
        py_world = WORLD_ENTRANCE_Y + py_map
        cell = _map_to_cell(px_map, py_map, origin, resolution)
        inside = 0 <= cell[0] < width and 0 <= cell[1] < height
        value = _cell_value(cell[0], cell[1], width, height, pgm_data)
        local_free, local_unknown, local_occupied, _, _, _ = _local_map_ratios(
            px_map, py_map, LOCAL_FREE_RADIUS_M, origin, resolution, width, height, pgm_data
        )
        clearance, nearest = _nearest_wall_clearance(px_world, py_world, walls)
        line_ok, line_clearance, line_free, line_bad_count = _line_check(
            (float(entrance_map["x_m"]), float(entrance_map["y_m"])),
            (px_map, py_map),
            origin,
            resolution,
            width,
            height,
            pgm_data,
            walls,
        )
        reasons: list[str] = []
        if not inside:
            reasons.append("candidate_outside_phase34_map_boundary")
        if not _is_free_value(value):
            reasons.append("candidate_cell_occupied_or_unknown")
        if local_unknown > 0.0 or local_occupied > 0.0:
            reasons.append("local_0_35m_radius_contains_occupied_or_unknown")
        if clearance < MIN_WALL_CLEARANCE_M:
            reasons.append("wall_clearance_below_minimum")
        if not line_ok:
            reasons.append("entrance_to_candidate_line_not_cleanly_reachable")
        passed = not reasons
        candidates.append(Candidate(
            name=f"ingress_{distance_m:.1f}m",
            distance_from_entrance_m=distance_m,
            point_map={"x_m": round(px_map, 6), "y_m": round(py_map, 6), "yaw_rad": yaw},
            point_world={"x_m": round(px_world, 6), "y_m": round(py_world, 6), "yaw_rad": yaw},
            inside_map_boundary=inside,
            map_cell=list(cell) if inside else None,
            map_pixel_value=value,
            occupied_or_unknown=not _is_free_value(value),
            local_free_ratio_radius_0_35m=local_free,
            local_unknown_ratio_radius_0_35m=local_unknown,
            local_occupied_ratio_radius_0_35m=local_occupied,
            min_clearance_to_wall_m=clearance,
            nearest_wall=nearest,
            short_straight_line_reachable=line_ok,
            line_min_clearance_to_wall_m=line_clearance,
            line_map_free_ratio=line_free,
            line_occupied_or_unknown_count=line_bad_count,
            validation_passed=passed,
            rejection_reasons=reasons,
        ))

    valid = [c for c in candidates if c.validation_passed]
    selected = None
    if valid:
        # Choose 1.0m if valid: safely inside the Phase52 1.0m gate boundary while
        # staying near the entrance; otherwise pick the nearest valid candidate.
        selected = min(valid, key=lambda c: (abs(c.distance_from_entrance_m - 1.0), c.distance_from_entrance_m))
        classification = CLASS_VALIDATED
    elif any(c.inside_map_boundary and not c.occupied_or_unknown for c in candidates):
        classification = CLASS_INCONCLUSIVE
    else:
        classification = CLASS_BLOCKED

    _make_overlay_png(width, height, pgm_data, origin, resolution, candidates, selected, OVERLAY_PNG)

    artifact = {
        "phase": "Phase53",
        "title": "Entrance Ingress Waypoint Design and Geometry Validation",
        "classification": classification,
        "active_world": str(ACTIVE_WORLD.relative_to(ROOT)),
        "active_config": str(ACTIVE_CONFIG.relative_to(ROOT)),
        "phase34_map_snapshot": str(PHASE34_MAP_YAML.relative_to(ROOT)),
        "overlay_png": str(OVERLAY_PNG.relative_to(ROOT)),
        "guardrails": {
            "nav2_mppi_controller_params_changed": False,
            "clearance_radius_changed": False,
            "map_sufficiency_threshold_changed": False,
            "maze_explorer_started": False,
            "dispatch_goal_sent": False,
            "old_scaffold_world_or_map_used": False,
            "autonomous_exploration_success_claimed": False,
        },
        "criteria": {
            "candidate_distance_range_m": [0.8, 1.5],
            "min_wall_clearance_m": MIN_WALL_CLEARANCE_M,
            "local_free_radius_m": LOCAL_FREE_RADIUS_M,
            "line_sample_step_m": LINE_SAMPLE_STEP_M,
            "pgm_free_min_value": PGM_FREE_MIN_VALUE,
        },
        "map_frame_truth": {
            "entrance": {
                "x_m": float(entrance_map["x_m"]),
                "y_m": float(entrance_map["y_m"]),
                "yaw_rad": float(entrance_map["yaw_rad"]),
            },
            "exit": {
                "x_m": float(exit_map["x_m"]),
                "y_m": float(exit_map["y_m"]),
                "radius_m": float(exit_map["radius_m"]),
            },
        },
        "world_frame_truth": {
            "entrance": {"x_m": WORLD_ENTRANCE_X, "y_m": WORLD_ENTRANCE_Y, "yaw_rad": float(entrance_world["yaw_rad"])},
            "exit": {"x_m": float(world_truth["exit"]["x_m"]), "y_m": float(world_truth["exit"]["y_m"]), "radius_m": float(world_truth["exit"]["radius_m"])},
        },
        "entrance_geometry": {
            "opening_side": config["entrance"]["opening_side"],
            "opening_center_world": {"x_m": float(config["entrance"]["opening_center_x_m"]), "y_m": float(config["entrance"]["opening_center_y_m"])},
            "opening_width_m": float(config["entrance"]["opening_width_m"]),
            "inward_direction": config["entrance"]["inward_direction"],
            "yaw_rad": yaw,
        },
        "slam_map_snapshot": {
            "width": width,
            "height": height,
            "resolution": resolution,
            "origin": {"x_m": origin[0], "y_m": origin[1], "yaw_rad": float(map_meta["origin"][2])},
            "bounds": {"min_x_m": origin[0], "max_x_m": origin[0] + width * resolution, "min_y_m": origin[1], "max_y_m": origin[1] + height * resolution},
        },
        "candidates": [asdict(c) for c in candidates],
        "selected_candidate": asdict(selected) if selected else None,
        "recommendation": "Use selected ingress_waypoint_map only as a future entrance-guide candidate; Phase53 does not run or validate autonomous exploration.",
    }
    ARTIFACT_JSON.write_text(json.dumps(artifact, indent=2, sort_keys=True), encoding="utf-8")
    print(json.dumps({"classification": classification, "selected_candidate": artifact["selected_candidate"], "artifact": str(ARTIFACT_JSON)}, indent=2))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
