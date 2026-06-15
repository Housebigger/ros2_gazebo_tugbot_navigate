#!/usr/bin/env python3
"""Phase138 static contract analyzer for second-step serialization instrumentation.

This is intentionally static/design-time only. It validates that maze_explorer.py
contains the minimal artifact/serialization hooks required for future Phase139
bounded runtime diagnosis without launching ROS/Gazebo/Nav2 or sending goals.
"""

from __future__ import annotations

import json
import sys
from pathlib import Path
from typing import Dict

REQUIRED_PHRASES = {
    'pending_snapshot_helper': 'def _serialize_pending_corridor_alignment_second_step',
    'pending_exists_runtime_serialized': "'runtime_serialized': True",
    'pending_origin_fields': "'original_goal_kind': pending.get('original_goal_kind')",
    'pending_branch_payload': "'active_branch': self._branch_option_to_payload",
    'pending_staging_result_status': "'staging_result_status_label': pending.get('staging_result_status_label')",
    'fresh_scan_timestamp': "pending['fresh_scan_sample_time_sec'] = self._now_wall_time_sec()",
    'fresh_local_costmap_timestamp': "pending['fresh_local_costmap_sample_time_sec'] = self._now_wall_time_sec()",
    'fresh_tf_timestamp': "pending['fresh_tf_sample_time_sec'] = second_step_generation_wall_time_sec",
    'second_step_forward_goal_helper': 'def _serialize_second_step_forward_goal',
    'second_step_validity': "'valid': self._second_step_forward_goal_valid(second_step)",
    'second_step_map_frame': "'map_frame_id': self.map_frame",
    'second_step_candidate_target': "'selected_candidate_target': second_step.get('selected_candidate_target')",
    'second_step_candidate_yaw': "'selected_candidate_yaw': second_step.get('selected_candidate_yaw')",
    'second_step_candidate_counts': "'hard_safety_pass_candidate_count': second_step.get('hard_safety_pass_candidate_count')",
    'front_wedge_payload': "'front_wedge_risk_after_staging': self._front_wedge_risk_payload(second_step)",
    'lateral_residual_payload': "'lateral_residual_after': self._second_step_lateral_residual_after(second_step)",
    'generated_after_fresh_evidence': "'generated_after_fresh_evidence': bool(second_step.get('generated_after_fresh_evidence', False))",
    'generation_time_ordering': "'generation_wall_time_sec': second_step_generation_wall_time_sec",
    'pending_context_serialized': "'pending_corridor_alignment_second_step': serialized_pending",
    'serialized_second_step_context': "'second_step_forward_goal': serialized_second_step",
    'skip_two_step_staging_context': "'skip_two_step_staging': bool(skip_two_step_staging)",
    'phase138_recursion_guard': "'phase138_recursion_guard': bool(skip_two_step_staging)",
    'phase136_recursion_guard': "'phase136_recursion_guard': bool(skip_two_step_staging)",
    'generic_recursion_guard': "'recursion_guard': bool(skip_two_step_staging)",
    'no_second_staging_request': "'two_step_stage_dispatch_requested': False",
    'no_second_staging_applied': "'staging_applied': False",
    'prior_staging_applied': "'prior_staging_applied': True",
    'dispatch_guard_call': "self._send_goal((float(selected[0]), float(selected[1])), float(yaw if isinstance(yaw, (int, float)) else direction_rad), 'explore', skip_two_step_staging=True)",
}


def analyze_text(text: str) -> dict[str, object]:
    checks: Dict[str, bool] = {
        name: phrase in text for name, phrase in REQUIRED_PHRASES.items()
    }
    valid = all(checks.values())
    return {
        'phase': 'Phase138',
        'valid': valid,
        'classification': (
            'PHASE138_SECOND_STEP_SERIALIZATION_CONTRACT_PRESENT'
            if valid else 'PHASE138_SECOND_STEP_SERIALIZATION_CONTRACT_INCOMPLETE'
        ),
        'checks': checks,
        'missing': [name for name, ok in checks.items() if not ok],
        'scope': 'static_contract_only_no_runtime_no_goal_dispatch',
    }


def analyze_source(path: str | Path) -> dict[str, object]:
    source_path = Path(path)
    return analyze_text(source_path.read_text(encoding='utf-8'))


def main(argv: list[str]) -> int:
    if len(argv) != 2:
        print('usage: analyze_phase138_second_step_contract_serialization_static.py <maze_explorer.py>', file=sys.stderr)
        return 2
    result = analyze_source(argv[1])
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result['valid'] else 1


if __name__ == '__main__':
    raise SystemExit(main(sys.argv))
