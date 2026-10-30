"""Real Qt dispatch/RViz tools against a fake local supervisor socket."""
import json
import os
from pathlib import Path
import subprocess

import pytest

APP = Path(__file__).resolve().parents[1]
CHECKS = [
    'pose_escape_sends_no_stop', 'pose_escape_returns_to_default_tool',
    'pose_escape_keeps_session_and_stop_button', 'pose_escape_does_not_enter_stopping',
    'goal_escape_sends_no_stop', 'goal_escape_returns_to_default_tool',
    'goal_escape_keeps_session_and_stop_button', 'goal_escape_does_not_enter_stopping',
    'escape_on_default_keeps_session', 'q_still_sends_stop_from_rviz',
    'stop_button_still_sends_stop', 'stop_label_only_q',
    'keyboard_escape_no_stop_and_stays_open', 'keyboard_q_still_stops',
    'save_dialog_escape_cancels_without_stop', 'mapping_child_escape_sends_no_stop',
    'mapping_child_q_still_sends_stop',
    'goal_toolbar_button_found', 'goal_toolbar_click_activates_goal',
    'goal_toolbar_escape_sends_no_stop', 'goal_toolbar_escape_returns_to_default_tool',
]


@pytest.fixture(scope='module')
def key_result():
    assert os.environ.get('DISPLAY')
    assert os.environ.get('ROS_DOMAIN_ID') == '211' and os.environ.get('ROS_LOCALHOST_ONLY') == '1'
    output = Path(os.environ['FIRE_ROBOT_QA_EVIDENCE'])
    result = subprocess.run(['/tmp/fire_robot_pre_sync_ui/key_policy_probe',
        str(APP/'native_rviz/operational.rviz')], capture_output=True, text=True, timeout=20)
    (output/'key_policy.log').write_text(result.stdout+result.stderr)
    lines = [line.removeprefix('QA_KEY_POLICY ') for line in result.stdout.splitlines()
             if line.startswith('QA_KEY_POLICY ')]
    assert len(lines) == 1, result.stdout+result.stderr
    row = json.loads(lines[0])
    (output/'key_policy.json').write_text(json.dumps(row,indent=2)+'\n')
    assert result.returncode == 0, row
    assert set(row) == set(CHECKS)
    return row


@pytest.mark.parametrize('key', CHECKS)
def test_escape_is_tool_cancel_and_only_q_is_stop(key_result, key):
    assert key_result[key] is True, (key,key_result)
