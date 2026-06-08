from __future__ import annotations

import json
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
RUN_ID = 'phase119_nav2_lifecycle_activation_stability'
ARTIFACT_DIR = ROOT / 'log' / RUN_ID
artifact = json.loads((ARTIFACT_DIR / f'{RUN_ID}_artifact.json').read_text(encoding='utf-8'))
launch_text = (ARTIFACT_DIR / f'{RUN_ID}_launch.log').read_text(encoding='utf-8', errors='replace')
lines = launch_text.splitlines()
# Infer launch active marker elapsed using artifact sample wall clocks: marker already present by first sample.
marker_line = None
for idx, line in enumerate(lines, start=1):
    if 'Managed nodes are active' in line:
        marker_line = {'line_no': idx, 'line': line.strip()}
        break
artifact['launch_log_events'] = artifact.get('launch_log_events') or []
if marker_line:
    artifact['launch_log_events'].append({
        'event': 'Managed nodes are active',
        'line_no': marker_line['line_no'],
        'line': marker_line['line'],
        'elapsed_sec': None,
        'ros_time_sec': None,
    })
# Add post-capture evidence that external CLI lifecycle queries timed out despite marker+preflight pass.
artifact['phase119_interpretation_notes'] = {
    'external_lifecycle_query_timeout_interpretation': 'query_race_or_cli_timeout_when_launch_marker_and_preflight_multi_source_confirmation_are_positive',
    'phase118_contrast': 'Phase118 had action present but missing launch_active_marker and post-fail inactive/unconfigured lifecycle states; Phase119 rerun had launch marker and preflight pass.',
}
(ARTIFACT_DIR / f'{RUN_ID}_artifact_enriched.json').write_text(json.dumps(artifact, indent=2, sort_keys=True)+'\n', encoding='utf-8')
print(ARTIFACT_DIR / f'{RUN_ID}_artifact_enriched.json')
