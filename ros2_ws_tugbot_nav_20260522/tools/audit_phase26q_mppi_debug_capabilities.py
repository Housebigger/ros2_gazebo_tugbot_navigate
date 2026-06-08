#!/usr/bin/env python3
"""Phase26Q audit of installed Nav2 MPPI debug evidence capabilities.

Analysis-only: inspect the installed ROS prefix for MPPI critic-stat message,
parameter, and symbol availability. This avoids blocking future diagnostics on a
nonexistent topic in the current Nav2 Jazzy build.
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any


def read_text(path: Path) -> str:
    try:
        return path.read_text(encoding='utf-8', errors='ignore')
    except (FileNotFoundError, IsADirectoryError, PermissionError, UnicodeDecodeError):
        return ''


def search_tree(root: Path, needles: list[str], limit: int = 200) -> dict[str, list[str]]:
    hits: dict[str, list[str]] = {needle: [] for needle in needles}
    if not root.exists():
        return hits
    for path in root.rglob('*'):
        if not path.is_file():
            continue
        text = read_text(path)
        if not text:
            continue
        for needle in needles:
            if needle in text and len(hits[needle]) < limit:
                hits[needle].append(str(path))
    return hits


def strings_for_library(path: Path) -> str:
    if not path.exists():
        return ''
    try:
        result = subprocess.run(['strings', str(path)], text=True, capture_output=True, check=False, timeout=20)
    except (OSError, subprocess.TimeoutExpired):
        return ''
    return result.stdout


def build_audit(ros_prefix: Path) -> dict[str, Any]:
    include_root = ros_prefix / 'include' / 'nav2_mppi_controller'
    share_root = ros_prefix / 'share' / 'nav2_mppi_controller'
    msg_roots = [ros_prefix / 'share' / 'nav2_msgs', ros_prefix / 'include' / 'nav2_msgs']
    library = ros_prefix / 'lib' / 'libmppi_controller.so'

    needles = ['publish_critics_stats', 'critics_stats', 'CriticsStats']
    source_hits = search_tree(include_root, needles)
    source_hits_share = search_tree(share_root, needles)
    msg_hits = {needle: [] for needle in needles}
    for root in msg_roots:
        partial = search_tree(root, needles)
        for needle, values in partial.items():
            msg_hits[needle].extend(values)
    lib_strings = strings_for_library(library)

    parameter_name_supported = bool(source_hits['publish_critics_stats'] or source_hits_share['publish_critics_stats'])
    topic_symbol_available = 'critics_stats' in lib_strings
    message_type_available = bool(msg_hits['CriticsStats'])
    status = 'available' if parameter_name_supported and topic_symbol_available and message_type_available else 'unavailable_in_installed_jazzy_mppi'

    return {
        'phase': '26Q',
        'analysis_only': True,
        'ros_prefix': str(ros_prefix),
        'critics_stats': {
            'status': status,
            'parameter_name_supported': parameter_name_supported,
            'library_symbol_available': topic_symbol_available,
            'message_type_available': message_type_available,
            'source_hits': source_hits,
            'share_hits': source_hits_share,
            'message_hits': msg_hits,
            'library': str(library),
        },
        'known_available_mppi_debug_topics': {
            'library_strings': sorted(
                token for token in ('/trajectories', 'optimal_trajectory', 'transformed_global_plan')
                if token in lib_strings
            ),
        },
        'recommendation': (
            'critics_stats_available_continue_with_real_topic_collection'
            if status == 'available'
            else 'do_not_block_on_critics_stats_use_summarized_trajectories_and_existing_logs'
        ),
    }


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--ros-prefix', type=Path, default=Path('/opt/ros/jazzy'))
    parser.add_argument('--output-json', type=Path, required=True)
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    report = build_audit(args.ros_prefix)
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'critics_stats': report['critics_stats'], 'recommendation': report['recommendation']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
