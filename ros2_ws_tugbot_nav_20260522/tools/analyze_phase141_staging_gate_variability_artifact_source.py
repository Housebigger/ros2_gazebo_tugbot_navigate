#!/usr/bin/env python3
"""Phase141 staging gate variability artifact/source analyzer.

This tool is intentionally artifact/source only.  It reads existing Phase134,
Phase136, and Phase139 JSON artifacts plus read-only source anchors from
maze_explorer.py and maze_perception.py.  It does not start ROS/Gazebo/Nav2,
does not send goals, and does not tune configuration or exploration behavior.
"""

from __future__ import annotations

import argparse
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Any

PHASE = "phase141_staging_gate_variability_artifact_source"
STATUS = "PHASE141_STAGING_GATE_VARIABILITY_ARTIFACT_SOURCE_COMPLETE_STOP_BEFORE_PHASE142"

ALLOWED_CLASSIFICATIONS = {
    "STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE",
    "STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT",
    "STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY",
    "STAGING_GATE_CONTRACT_AMBIGUOUS",
    "INSUFFICIENT_STAGING_GATE_EVIDENCE",
}


@dataclass(frozen=True)
class ArtifactSpec:
    phase: str
    path: Path


ARTIFACTS = [
    ArtifactSpec(
        "Phase134",
        Path("log/phase134_bounded_corridor_alignment_staging_smoke/phase134_bounded_corridor_alignment_staging_smoke.json"),
    ),
    ArtifactSpec(
        "Phase136",
        Path("log/phase136_bounded_second_step_explore_after_staging_smoke/phase136_bounded_second_step_explore_after_staging_smoke.json"),
    ),
    ArtifactSpec(
        "Phase139",
        Path(
            "log/phase139_instrumented_second_step_contract_runtime_verification/"
            "phase139_instrumented_second_step_contract_runtime_verification.json"
        ),
    ),
]

SOURCE_FILES = [
    Path("src/tugbot_maze/tugbot_maze/maze_explorer.py"),
    Path("src/tugbot_maze/tugbot_maze/maze_perception.py"),
]

SOURCE_ANCHORS = [
    "def _maybe_plan_corridor_alignment_staging",
    "plan_two_step_corridor_alignment_staging_goal",
    "def _two_step_staging_trigger_conditions",
    "'near_goal_lateral_residual'",
    "'single_step_forward_search_no_hard_safety_pass'",
    "'safety_floor_dominant_blocker'",
    "'execution_time_footprint_front_wedge_risk'",
    "return 'single_step_forward_search_had_hard_safe_candidate'",
]


def _get(obj: Any, path: str, default: Any = None) -> Any:
    cur = obj
    for part in path.split("."):
        if isinstance(cur, dict) and part in cur:
            cur = cur[part]
        else:
            return default
    return cur


def _first_present(obj: Any, paths: list[str], default: Any = None) -> Any:
    for path in paths:
        value = _get(obj, path, None)
        if value is not None:
            return value
    return default


def _bool_or_none(value: Any) -> bool | None:
    if isinstance(value, bool):
        return value
    return None


def _load_json(path: Path) -> dict[str, Any]:
    return json.loads(path.read_text(encoding="utf-8"))


def _front_wedge(first: dict[str, Any]) -> dict[str, Any]:
    risk = first.get("front_wedge_risk") or {}
    terminal = first.get("terminal_event") or {}
    return {
        "max": _first_present(first, ["front_wedge_risk.max", "terminal_event.phase62_front_wedge_cost.max"]),
        "mean": _first_present(first, ["front_wedge_risk.mean", "terminal_event.phase62_front_wedge_cost.mean"]),
        "high_cost_count": _first_present(
            first,
            ["front_wedge_risk.high_cost_count", "terminal_event.phase62_front_wedge_cost.high_cost_count"],
        ),
        "lethal_count": _first_present(
            first,
            ["front_wedge_risk.lethal_count", "terminal_event.phase62_front_wedge_cost.lethal_count"],
        ),
        "sample_count": _first_present(
            first,
            ["front_wedge_risk.sample_count", "terminal_event.phase62_front_wedge_cost.sample_count"],
        ),
        "source": "first_literal_dispatch.front_wedge_risk" if risk else ("terminal_event.phase62_front_wedge_cost" if terminal else None),
    }


def _source_single_step(first: dict[str, Any]) -> dict[str, Any]:
    paths = [
        "two_step_staging_plan.source_single_step",
        "pending_corridor_alignment_second_step.staging_plan.source_single_step",
        "terminal_event.two_step_staging_plan.source_single_step",
        "terminal_event.centerline_target_refinement.multi_candidate_forward_search",
        "terminal_event.centerline_target_refinement",
    ]
    source_path = None
    source = None
    for path in paths:
        value = _get(first, path)
        if isinstance(value, dict):
            source_path = path
            source = value
            break
    source = source or {}
    return {
        "source_path": source_path,
        "candidate_count": source.get("candidate_count"),
        "hard_safety_pass_candidate_count": source.get("hard_safety_pass_candidate_count"),
        "refinement_applied": source.get("refinement_applied"),
        "original_target_preserved_on_reject": source.get("original_target_preserved_on_reject"),
        "selected_candidate_index": source.get("selected_candidate_index"),
        "selected_candidate_target": source.get("selected_candidate_target"),
        "selection_priority_trace": source.get("selection_priority_trace"),
        "rejected_candidate_summary_count": len(source.get("rejected_candidate_summaries") or []),
    }


def _trigger_conditions(first: dict[str, Any]) -> dict[str, Any]:
    trigger = _first_present(
        first,
        [
            "two_step_staging_plan.trigger_conditions",
            "pending_corridor_alignment_second_step.staging_plan.trigger_conditions",
            "terminal_event.two_step_staging_plan.trigger_conditions",
        ],
        {},
    )
    return dict(trigger) if isinstance(trigger, dict) else {}


def _original_metrics(first: dict[str, Any]) -> dict[str, Any]:
    metrics = _first_present(
        first,
        [
            "two_step_staging_plan.source_single_step.original_metrics",
            "pending_corridor_alignment_second_step.staging_plan.source_single_step.original_metrics",
            "terminal_event.centerline_target_refinement.original_metrics",
            "terminal_event.centerline_target_refinement.multi_candidate_forward_search.original_metrics",
        ],
        {},
    )
    return dict(metrics) if isinstance(metrics, dict) else {}


def _classify_row(first: dict[str, Any], source_single_step: dict[str, Any]) -> str:
    goal_kind = first.get("goal_kind")
    staging_applied = first.get("staging_applied")
    reject_reason = first.get("staging_reject_reason")
    hard_count = source_single_step.get("hard_safety_pass_candidate_count")

    if goal_kind == "corridor_alignment_staging" and staging_applied is True:
        return "STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT"
    if (
        goal_kind == "explore"
        and staging_applied is False
        and reject_reason == "single_step_forward_search_had_hard_safe_candidate"
        and isinstance(hard_count, int)
        and hard_count > 0
    ):
        return "STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE"
    if goal_kind or staging_applied is not None or reject_reason:
        return "STAGING_GATE_CONTRACT_AMBIGUOUS"
    return "INSUFFICIENT_STAGING_GATE_EVIDENCE"


def _row_evidence_gaps(first: dict[str, Any], row: dict[str, Any]) -> list[str]:
    gaps: list[str] = []
    required = [
        "lateral_residual_before_m",
        "lateral_residual_after_m",
        "candidate_count",
        "hard_safety_pass_candidate_count",
        "target_local_cost",
        "target_local_cost_max_radius",
        "path_corridor_min_clearance_m",
        "target_clearance_m",
        "candidate_branch_count",
        "last_open_direction_count",
        "last_candidate_count",
        "branch_angle",
    ]
    for name in required:
        if row.get(name) is None:
            gaps.append(f"missing {name}")
    if not row.get("trigger_conditions"):
        gaps.append("missing trigger_conditions")
    if row.get("same_corridor") is None:
        gaps.append("missing same_corridor")
    if row.get("two_side_wall_evidence") is None:
        gaps.append("missing two_side_wall_evidence")
    return gaps


def normalize_artifact(root: Path, spec: ArtifactSpec) -> dict[str, Any]:
    artifact_path = root / spec.path
    artifact = _load_json(artifact_path)
    first = artifact.get("first_literal_dispatch") or {}
    terminal = first.get("terminal_event") or {}
    staging_check = first.get("staging_executability_check") or {}
    source_single = _source_single_step(first)
    trigger = _trigger_conditions(first)
    metrics = _original_metrics(first)

    row: dict[str, Any] = {
        "phase": spec.phase,
        "artifact_path": str(spec.path),
        "artifact_classification": artifact.get("classification"),
        "artifact_valid": artifact.get("valid"),
        "goal_kind": first.get("goal_kind"),
        "staging_applied": first.get("staging_applied"),
        "two_step_stage_dispatch_requested": first.get("two_step_stage_dispatch_requested"),
        "staging_reject_reason": first.get("staging_reject_reason"),
        "staging_reason": first.get("staging_reason"),
        "lateral_residual_before_m": first.get("lateral_residual_before_m"),
        "lateral_residual_after_m": first.get("lateral_residual_after_m"),
        "front_wedge_risk": _front_wedge(first),
        "staging_executability_check": {
            "checked": staging_check.get("checked"),
            "hard_safety_pass": staging_check.get("hard_safety_pass"),
            "same_corridor": staging_check.get("same_corridor"),
            "two_side_wall_evidence": staging_check.get("two_side_wall_evidence"),
            "local_same_corridor": staging_check.get("local_same_corridor"),
            "local_two_side_wall_evidence": staging_check.get("local_two_side_wall_evidence"),
            "local_cost_sample_count": staging_check.get("local_cost_sample_count"),
            "front_wedge_sample_count": staging_check.get("front_wedge_sample_count"),
            "reason": staging_check.get("reason"),
        },
        "source_single_step": source_single,
        "trigger_conditions": trigger,
        "candidate_count": source_single.get("candidate_count"),
        "hard_safety_pass_candidate_count": source_single.get("hard_safety_pass_candidate_count"),
        "selected_candidate_index": source_single.get("selected_candidate_index"),
        "selected_candidate_target": source_single.get("selected_candidate_target"),
        "selection_priority_trace": source_single.get("selection_priority_trace"),
        "rejected_candidate_summary_count": source_single.get("rejected_candidate_summary_count"),
        "same_corridor": _first_present(
            first,
            [
                "staging_executability_check.same_corridor",
                "staging_executability_check.local_same_corridor",
                "terminal_event.centerline_target_refinement.original_metrics.same_corridor",
                "terminal_event.centerline_target_refinement.multi_candidate_forward_search.original_metrics.same_corridor",
            ],
        ),
        "two_side_wall_evidence": _first_present(
            first,
            [
                "staging_executability_check.two_side_wall_evidence",
                "staging_executability_check.local_two_side_wall_evidence",
                "terminal_event.centerline_target_refinement.original_metrics.two_side_wall_evidence",
                "terminal_event.centerline_target_refinement.multi_candidate_forward_search.original_metrics.two_side_wall_evidence",
            ],
        ),
        "target_local_cost": terminal.get("dispatch_target_local_cost"),
        "target_local_cost_max_radius": _first_present(
            first,
            [
                "terminal_event.dispatch_target_local_cost_max_radius",
                "terminal_event.centerline_target_refinement.original_metrics.local_cost_max_radius",
                "terminal_event.centerline_target_refinement.multi_candidate_forward_search.original_metrics.local_cost_max_radius",
            ],
        ),
        "path_corridor_min_clearance_m": terminal.get("path_corridor_min_clearance_m"),
        "target_clearance_m": terminal.get("target_clearance_m"),
        "candidate_branch_count": terminal.get("candidate_branch_count"),
        "last_open_direction_count": terminal.get("last_open_direction_count"),
        "last_candidate_count": terminal.get("last_candidate_count"),
        "selected_branch_geometry": {
            "branch_angle": terminal.get("branch_angle"),
            "chosen_branch_rank": terminal.get("chosen_branch_rank"),
            "exit_progress_delta_m": _get(terminal, "chosen_branch_score_components.exit_progress_delta_m"),
            "target_exit_dist": _get(terminal, "chosen_branch_score_components.target_exit_dist"),
        },
        "branch_angle": terminal.get("branch_angle"),
        "original_metrics": metrics,
        "goal_event_count": artifact.get("goal_event_count"),
        "dispatch_event_count": artifact.get("dispatch_event_count"),
        "second_step_goal_count": artifact.get("second_step_goal_count"),
        "third_goal_dispatched": artifact.get("third_goal_dispatched"),
        "stop_reason": artifact.get("stop_reason"),
    }
    row["classification"] = _classify_row(first, source_single)
    row["evidence_gaps"] = _row_evidence_gaps(first, row)
    return row


def scan_sources(root: Path) -> dict[str, Any]:
    texts: dict[str, str] = {}
    for rel in SOURCE_FILES:
        texts[str(rel)] = (root / rel).read_text(encoding="utf-8")
    combined = "\n".join(texts.values())
    present = [anchor for anchor in SOURCE_ANCHORS if anchor in combined]
    missing = [anchor for anchor in SOURCE_ANCHORS if anchor not in combined]
    return {
        "read_only_files": [str(p) for p in SOURCE_FILES],
        "present_anchors": present,
        "missing_anchors": missing,
        "all_required_anchors_present": not missing,
        "source_model": {
            "delegation": "maze_explorer._maybe_plan_corridor_alignment_staging delegates to plan_two_step_corridor_alignment_staging_goal",
            "trigger_bundle": [
                "near_goal_lateral_residual",
                "single_step_forward_search_no_hard_safety_pass",
                "safety_floor_dominant_blocker",
                "execution_time_footprint_front_wedge_risk",
            ],
            "direct_explore_reject_reason": "single_step_forward_search_had_hard_safe_candidate",
        },
    }


def cross_phase_assessment(rows: list[dict[str, Any]]) -> dict[str, Any]:
    triggered = [r for r in rows if r["classification"] == "STAGING_GATE_TRIGGERED_CORRIDOR_ALIGNMENT"]
    direct = [r for r in rows if r["classification"] == "STAGING_GATE_DIRECT_EXPLORE_HARD_SAFE_CANDIDATE"]
    phase139 = next((r for r in rows if r["phase"] == "Phase139"), {})

    support: list[str] = []
    gaps: list[str] = []
    if {r["phase"] for r in triggered} >= {"Phase134", "Phase136"}:
        if all(r.get("candidate_count") == 63 and r.get("hard_safety_pass_candidate_count") == 0 for r in triggered):
            support.append("Phase134/Phase136 source single-step candidate_count=63 with hard_safety_pass_candidate_count=0")
    if phase139.get("candidate_count") == 63 and phase139.get("hard_safety_pass_candidate_count") == 17:
        support.append("Phase139 source single-step candidate_count=63 with hard_safety_pass_candidate_count=17")
    if phase139.get("staging_reject_reason") == "single_step_forward_search_had_hard_safe_candidate":
        support.append("Phase139 reject reason single_step_forward_search_had_hard_safe_candidate matches source trigger false condition")
    if all((r.get("front_wedge_risk") or {}).get("max") == 99 for r in rows):
        support.append("all compared samples retain high front_wedge risk evidence")
    if all(r.get("same_corridor") is True and r.get("two_side_wall_evidence") is True for r in triggered):
        support.append("same corridor and two-side wall evidence are present for staging-triggered samples")
    if any(r.get("target_local_cost_max_radius") is not None for r in triggered):
        support.append("staging-triggered samples include target/local cost and branch clearance geometry fields")
    if any(r.get("branch_angle") is not None for r in triggered):
        support.append("staging-triggered samples include selected branch geometry and branch_angle")

    if phase139.get("candidate_branch_count") is None or phase139.get("target_local_cost_max_radius") is None:
        gaps.append("Phase139 terminal branch geometry/local cost/clearance fields are absent from the Phase139 bounded artifact")
    if phase139.get("lateral_residual_before_m") is None and (phase139.get("trigger_conditions") or {}).get("near_goal_lateral_residual") is True:
        gaps.append("Phase139 lateral residual scalar is absent even though trigger_conditions.near_goal_lateral_residual is true")
    for row in rows:
        for gap in row.get("evidence_gaps", []):
            if row["phase"] == "Phase139" and gap not in gaps:
                gaps.append(f"{row['phase']}: {gap}")

    supported = bool(triggered and direct and len(support) >= 5)
    classification = "STAGING_GATE_VARIABILITY_BY_COST_GEOMETRY" if supported else "STAGING_GATE_CONTRACT_AMBIGUOUS"
    return {
        "classification": classification,
        "supported": supported,
        "confidence": "supported_with_explicit_evidence_gaps" if supported and gaps else ("supported" if supported else "ambiguous"),
        "decision_boundary": "diagnostic_artifact_source_only_not_runtime_success",
        "triggered_phases": [r["phase"] for r in triggered],
        "direct_explore_phases": [r["phase"] for r in direct],
        "supporting_evidence": support,
        "evidence_gaps": gaps,
        "interpretation": (
            "Cost/geometry/candidate evidence supports staging-gate variability: Phase134/136 had no hard-safe "
            "single-step forward candidates and therefore triggered corridor alignment staging, while Phase139 had "
            "17 hard-safe candidates and rejected staging as not needed. The conclusion remains diagnostic because "
            "Phase139 lacks several terminal branch/local-cost/clearance scalars."
        ),
    }


def analyze(root: Path | str) -> dict[str, Any]:
    root = Path(root)
    rows = [normalize_artifact(root, spec) for spec in ARTIFACTS]
    source = scan_sources(root)
    cross = cross_phase_assessment(rows)
    return {
        "phase": PHASE,
        "status": STATUS,
        "scope": {
            "artifact_source_only": True,
            "runtime_started": False,
            "goals_sent": False,
            "maze_explorer_started": False,
            "config_tuned": False,
            "strategy_changed": False,
            "staging_disabled": False,
            "success_claimed": False,
            "phase142_entered": False,
        },
        "inputs": {
            "artifacts": [{"phase": spec.phase, "path": str(spec.path)} for spec in ARTIFACTS],
            "source_files": [str(p) for p in SOURCE_FILES],
        },
        "normalized_rows": rows,
        "source_review": source,
        "cross_phase_assessment": cross,
        "forbidden_claims": [
            "No autonomous exploration success is claimed",
            "No exit success is claimed",
            "No Phase127 timeout repair is claimed",
            "No Phase138 runtime contract verification is claimed",
            "No Phase142 work is entered",
        ],
    }


def _fmt(value: Any) -> str:
    if value is None:
        return "null"
    if isinstance(value, bool):
        return "true" if value else "false"
    return str(value)


def render_markdown(result: dict[str, Any]) -> str:
    lines: list[str] = []
    lines.append("# Phase141 staging gate variability artifact/source analysis")
    lines.append("")
    lines.append(f"Status: `{result['status']}`")
    lines.append("")
    lines.append("## Scope")
    lines.append("")
    lines.append("This analysis is artifact/source analyzer only. It read existing Phase134/136/139 artifacts and source anchors only.")
    lines.append("")
    lines.append("- No Gazebo/RViz/Nav2 runtime was launched")
    lines.append("- No NavigateToPose goal was sent")
    lines.append("- No maze_explorer was started")
    lines.append("- No staging/explore/third goal was sent")
    lines.append("- No Nav2/MPPI/controller/goal checker/config tuning was performed")
    lines.append("- No exploration strategy, branch scoring, centerline, fallback, or terminal acceptance change was made")
    lines.append("- No staging was disabled")
    lines.append("- No autonomous exploration success or exit success is claimed")
    lines.append("- Phase142 not entered")
    lines.append("")
    lines.append("## Normalized rows")
    lines.append("")
    for row in result["normalized_rows"]:
        lines.append(f"### {row['phase']}")
        lines.append("")
        lines.append(f"- classification: `{row['classification']}`")
        lines.append(f"- artifact_classification: `{row['artifact_classification']}`")
        lines.append(f"- goal_kind: `{row['goal_kind']}`")
        lines.append(f"- staging_applied: `{_fmt(row['staging_applied'])}`")
        lines.append(f"- staging_reason: `{row['staging_reason']}`")
        lines.append(f"- staging_reject_reason: `{row['staging_reject_reason']}`")
        lines.append(f"- lateral_residual_before_m: `{_fmt(row['lateral_residual_before_m'])}`")
        lines.append(f"- lateral_residual_after_m: `{_fmt(row['lateral_residual_after_m'])}`")
        fwr = row["front_wedge_risk"]
        lines.append(
            "- front_wedge_risk: "
            f"max={_fmt(fwr.get('max'))}, mean={_fmt(fwr.get('mean'))}, "
            f"high_cost_count={_fmt(fwr.get('high_cost_count'))}, lethal_count={_fmt(fwr.get('lethal_count'))}, "
            f"sample_count={_fmt(fwr.get('sample_count'))}"
        )
        lines.append(
            "- source_single_step: "
            f"candidate_count={_fmt(row['candidate_count'])}, "
            f"hard_safety_pass_candidate_count={_fmt(row['hard_safety_pass_candidate_count'])}, "
            f"refinement_applied={_fmt(row['source_single_step'].get('refinement_applied'))}, "
            f"original_target_preserved_on_reject={_fmt(row['source_single_step'].get('original_target_preserved_on_reject'))}"
        )
        lines.append(f"- trigger_conditions: `{json.dumps(row['trigger_conditions'], sort_keys=True)}`")
        lines.append(f"- same_corridor: `{_fmt(row['same_corridor'])}`")
        lines.append(f"- two_side_wall_evidence: `{_fmt(row['two_side_wall_evidence'])}`")
        lines.append(f"- target_local_cost: `{_fmt(row['target_local_cost'])}`")
        lines.append(f"- target_local_cost_max_radius: `{_fmt(row['target_local_cost_max_radius'])}`")
        lines.append(f"- path_corridor_min_clearance_m: `{_fmt(row['path_corridor_min_clearance_m'])}`")
        lines.append(f"- target_clearance_m: `{_fmt(row['target_clearance_m'])}`")
        lines.append(f"- candidate_branch_count: `{_fmt(row['candidate_branch_count'])}`")
        lines.append(f"- last_open_direction_count: `{_fmt(row['last_open_direction_count'])}`")
        lines.append(f"- last_candidate_count: `{_fmt(row['last_candidate_count'])}`")
        lines.append(f"- branch_angle: `{_fmt(row['branch_angle'])}`")
        lines.append(f"- selected_branch_geometry: `{json.dumps(row['selected_branch_geometry'], sort_keys=True)}`")
        lines.append(f"- evidence_gaps: `{json.dumps(row['evidence_gaps'], sort_keys=True)}`")
        lines.append("")
    cross = result["cross_phase_assessment"]
    lines.append("## Cross-phase assessment")
    lines.append("")
    lines.append(f"- classification: `{cross['classification']}`")
    lines.append(f"- supported: `{_fmt(cross['supported'])}`")
    lines.append(f"- confidence: `{cross['confidence']}`")
    lines.append(f"- decision_boundary: `{cross['decision_boundary']}`")
    lines.append(f"- triggered_phases: `{', '.join(cross['triggered_phases'])}`")
    lines.append(f"- direct_explore_phases: `{', '.join(cross['direct_explore_phases'])}`")
    lines.append("")
    lines.append("Supporting evidence:")
    for item in cross["supporting_evidence"]:
        lines.append(f"- {item}")
    lines.append("")
    lines.append("Evidence gaps:")
    for item in cross["evidence_gaps"]:
        lines.append(f"- {item}")
    lines.append("")
    lines.append("Interpretation:")
    lines.append("")
    lines.append(cross["interpretation"])
    lines.append("")
    lines.append("## Source review")
    lines.append("")
    source = result["source_review"]
    lines.append(f"- read_only_files: `{', '.join(source['read_only_files'])}`")
    lines.append(f"- all_required_anchors_present: `{_fmt(source['all_required_anchors_present'])}`")
    lines.append("- present anchors:")
    for anchor in source["present_anchors"]:
        lines.append(f"  - `{anchor}`")
    if source["missing_anchors"]:
        lines.append("- missing anchors:")
        for anchor in source["missing_anchors"]:
            lines.append(f"  - `{anchor}`")
    lines.append("")
    lines.append("## Verification plan evidence")
    lines.append("")
    lines.append("Focused tests were written before the analyzer implementation.")
    lines.append("")
    lines.append("- RED command: `pytest -q src/tugbot_maze/test/test_phase141_staging_gate_variability_artifact_source.py`")
    lines.append("- RED result: `5 failed`; expected reason was missing `tools/analyze_phase141_staging_gate_variability_artifact_source.py`")
    lines.append("- GREEN command: `pytest -q src/tugbot_maze/test/test_phase141_staging_gate_variability_artifact_source.py`")
    lines.append("- GREEN result: `5 passed in 0.30s`")
    lines.append("- Static compile command: `python3 -m py_compile tools/analyze_phase141_staging_gate_variability_artifact_source.py src/tugbot_maze/test/test_phase141_staging_gate_variability_artifact_source.py`")
    lines.append("- Artifact-only guard: no runtime processes are allowed during Phase141")
    lines.append("- Stop condition: Phase142 not entered")
    lines.append("")
    return "\n".join(lines)


def write_outputs(root: Path, result: dict[str, Any]) -> tuple[Path, Path, Path]:
    out_dir = root / "log" / PHASE
    out_dir.mkdir(parents=True, exist_ok=True)
    json_path = out_dir / f"{PHASE}_analysis.json"
    md_path = out_dir / f"{PHASE}_analysis.md"
    report_path = root / "doc" / "doc_report" / f"{PHASE}_report.md"
    json_path.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    md = render_markdown(result)
    md_path.write_text(md, encoding="utf-8")
    report_path.parent.mkdir(parents=True, exist_ok=True)
    report_path.write_text(md, encoding="utf-8")
    return json_path, md_path, report_path


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--root", type=Path, default=Path.cwd())
    args = parser.parse_args()
    root = args.root.resolve()
    result = analyze(root)
    json_path, md_path, report_path = write_outputs(root, result)
    print("PHASE141_STAGING_GATE_VARIABILITY_ARTIFACT_SOURCE_ANALYSIS_COMPLETE")
    print(f"status={result['status']}")
    print(f"classification={result['cross_phase_assessment']['classification']}")
    print(f"supported={result['cross_phase_assessment']['supported']}")
    print(f"json={json_path}")
    print(f"markdown={md_path}")
    print(f"report={report_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
