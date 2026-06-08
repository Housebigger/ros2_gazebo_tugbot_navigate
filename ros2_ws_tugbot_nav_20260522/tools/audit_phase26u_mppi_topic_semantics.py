#!/usr/bin/env python3
"""Phase26U MPPI topic semantics audit.

Diagnostics-only. Audits installed Nav2 MPPI headers/library strings and compact
evidence rows to explain what `/trajectories`, `/optimal_trajectory`, and
`/transformed_global_plan` can and cannot prove.
"""

from __future__ import annotations

import argparse
import json
import subprocess
from pathlib import Path
from typing import Any


def read_text(path: Path) -> str:
    if not path.exists():
        return ''
    try:
        return path.read_text(encoding='utf-8', errors='replace')
    except TypeError:
        return path.read_text(encoding='utf-8')


def strings_text(path: Path) -> str:
    if not path.exists():
        return ''
    try:
        result = subprocess.run(['strings', str(path)], text=True, capture_output=True, check=False, timeout=20)
        return result.stdout
    except Exception:
        return read_text(path)


def iter_jsonl(path: Path):
    if not path.exists():
        return
    with path.open(encoding='utf-8') as f:
        for line in f:
            if line.strip():
                try:
                    yield json.loads(line)
                except json.JSONDecodeError:
                    continue


def sample_topic_summary(path: Path, topic: str) -> dict[str, Any]:
    for row in iter_jsonl(path) or []:
        if row.get('topic') == topic and isinstance(row.get('data_summary'), dict):
            return row['data_summary']
    return {}


def topic_report(ros_prefix: Path, sample_evidence: Path) -> dict[str, Any]:
    header_path = ros_prefix / 'include/nav2_mppi_controller/tools/trajectory_visualizer.hpp'
    controller_path = ros_prefix / 'include/nav2_mppi_controller/controller.hpp'
    lib_candidates = [ros_prefix / 'lib/libmppi_controller.so', ros_prefix / 'lib/libnav2_mppi_controller.so']
    header = read_text(header_path)
    controller = read_text(controller_path)
    lib_text = '\n'.join(strings_text(p) for p in lib_candidates if p.exists())

    trajectories_sample = sample_topic_summary(sample_evidence, '/trajectories')
    optimal_sample = sample_topic_summary(sample_evidence, '/optimal_trajectory')
    transformed_sample = sample_topic_summary(sample_evidence, '/transformed_global_plan')

    trajectories_message_type = 'visualization_msgs/msg/MarkerArray' if 'MarkerArray' in header or '/trajectories' in lib_text else 'unknown'
    optimal_message_type = 'nav_msgs/msg/Path' if 'optimal_path_pub_' in header or 'optimal_trajectory' in lib_text else 'unknown'
    transformed_message_type = 'nav_msgs/msg/Path' if 'transformed_path_pub_' in header or 'transformed_global_plan' in lib_text else 'unknown'

    sample_point_count = trajectories_sample.get('point_count')
    geometry_limit = 'not_observed'
    if sample_point_count == 0:
        geometry_limit = 'markerarray_points_empty_in_compact_summary'
    elif sample_point_count is None:
        geometry_limit = 'no_markerarray_sample'

    return {
        '/trajectories': {
            'message_type': trajectories_message_type,
            'publisher_evidence': {
                'header': str(header_path),
                'has_trajectories_publisher_member': 'trajectories_publisher_' in header,
                'has_candidate_trajectories_add_api': 'models::Trajectories' in header,
                'library_string_available': '/trajectories' in lib_text,
            },
            'sample_marker_count': trajectories_sample.get('marker_count'),
            'sample_point_count': sample_point_count,
            'sample_representative_path_length': trajectories_sample.get('representative_path_length'),
            'geometry_limit': geometry_limit,
            'interpretation': 'candidate trajectory visualization marker array; compact summary cannot reconstruct geometry when markers contain no points',
        },
        '/optimal_trajectory': {
            'message_type': optimal_message_type,
            'publisher_evidence': {
                'header': str(header_path),
                'has_optimal_path_publisher_member': 'optimal_path_pub_' in header,
                'has_optimal_trajectory_add_api': 'xt::xtensor<float, 2>' in header,
                'library_string_available': 'optimal_trajectory' in lib_text,
            },
            'sample_point_count': optimal_sample.get('point_count'),
            'sample_path_displacement': optimal_sample.get('path_displacement'),
            'sample_path_length': optimal_sample.get('path_length'),
            'interpretation': 'nav_msgs/Path for optimal trajectory geometry, but compact summary only keeps counts/length/displacement and not raw poses',
        },
        '/transformed_global_plan': {
            'message_type': transformed_message_type,
            'publisher_evidence': {
                'header': str(header_path),
                'has_transformed_path_publisher_member': 'transformed_path_pub_' in header,
                'controller_visualize_method': 'void visualize' in controller or 'visualize(' in controller,
                'library_string_available': 'transformed_global_plan' in lib_text,
            },
            'sample_point_count': transformed_sample.get('point_count'),
            'sample_path_displacement': transformed_sample.get('path_displacement'),
            'sample_path_length': transformed_sample.get('path_length'),
            'interpretation': 'nav_msgs/Path for transformed plan; compact summary lacks raw poses for spatial costmap join',
        },
    }


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument('--ros-prefix', type=Path, default=Path('/opt/ros/jazzy'))
    p.add_argument('--sample-evidence', type=Path, default=Path('log/phase26r_candidate_summary_run1_mppi_evidence_summary.jsonl'))
    p.add_argument('--output-json', type=Path, required=True)
    return p.parse_args()


def main() -> int:
    args = parse_args()
    topics = topic_report(args.ros_prefix, args.sample_evidence)
    report = {
        'phase': '26U',
        'analysis_only': True,
        'ros_prefix': str(args.ros_prefix),
        'sample_evidence': str(args.sample_evidence),
        'topics': topics,
        'summary': {
            'trajectories_markerarray_point_count': topics['/trajectories']['sample_point_count'],
            'trajectories_geometry_reconstructable_from_summary': bool((topics['/trajectories']['sample_point_count'] or 0) > 0),
            'optimal_raw_poses_available_in_summary': False,
            'transformed_plan_raw_poses_available_in_summary': False,
        },
        'decision': {
            'phase27_candidate_signal': 'not_supported',
            'intervention_allowed': False,
            'missing_evidence': ['raw_or_sampled_optimal_trajectory_poses', 'raw_or_sampled_transformed_plan_poses', 'markerarray_marker_types_and_points_or_selected_trajectory_id'],
        },
    }
    args.output_json.parent.mkdir(parents=True, exist_ok=True)
    args.output_json.write_text(json.dumps(report, indent=2, sort_keys=True) + '\n', encoding='utf-8')
    print(json.dumps({'summary': report['summary'], 'decision': report['decision']}, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
