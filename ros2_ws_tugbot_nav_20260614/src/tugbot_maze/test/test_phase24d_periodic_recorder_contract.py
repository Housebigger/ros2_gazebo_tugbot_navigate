from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
RECORDER = ROOT / 'tools' / 'record_post_recovery_snapshots.py'


def test_phase24d_recorder_emits_periodic_active_goal_snapshots():
    source = RECORDER.read_text(encoding='utf-8')
    assert 'periodic_active_goal' in source
    assert '--periodic-snapshot-sec' in source
    assert 'last_periodic_snapshot_time' in source
    assert "self._emit_snapshot(seq, 'periodic_active_goal'" in source
