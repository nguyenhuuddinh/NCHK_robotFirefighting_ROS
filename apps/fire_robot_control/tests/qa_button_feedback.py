"""Real Qt mouse/keyboard/rendering, no robot I/O."""
import json
import os
from pathlib import Path
import subprocess
import pytest


@pytest.fixture(scope='module')
def feedback():
    binary = Path('/tmp/fire_robot_pre_sync_ui/button_feedback_probe')
    result = subprocess.run([str(binary), '/tmp/fire_robot_button_feedback.png'],
        env=dict(os.environ, QT_QPA_PLATFORM='offscreen'), capture_output=True, text=True, timeout=10)
    assert result.returncode == 0, result.stdout + result.stderr
    rows = [s.removeprefix('BUTTON_FEEDBACK_QA ') for s in result.stdout.splitlines() if s.startswith('BUTTON_FEEDBACK_QA ')]
    assert len(rows) == 1
    return json.loads(rows[0])


@pytest.mark.parametrize('key', ['held_down', 'pressed_changes_pixels', 'no_action_before_release',
    'stable_size', 'one_action_on_release', 'fast_click_visible', 'feedback_cleared',
    'event_loop_responsive', 'disabled_ignores_input', 'keyboard_one_action'])
def test_button_feedback_and_input(feedback, key):
    assert feedback[key] is True, key
