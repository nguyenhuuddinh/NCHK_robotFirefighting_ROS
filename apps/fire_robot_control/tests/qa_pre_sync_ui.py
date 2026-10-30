"""Native SessionUi regression probes; run only in ROS localhost domain 211.

Build tests/pre_sync_ui_review into /tmp/fire_robot_pre_sync_ui first. The
probe only inspects RViz tools and never publishes a Pose or Goal.
"""
import json
import os
from pathlib import Path
import subprocess

import pytest

APP = Path(__file__).resolve().parents[1]
BINARY = Path("/tmp/fire_robot_pre_sync_ui/review_probe")


@pytest.fixture(scope="module")
def ui_result():
    assert BINARY.is_file(), "Build tests/pre_sync_ui_review first"
    result = subprocess.run([str(BINARY), str(APP / "native_rviz/operational.rviz")],
        env=dict(os.environ, ROS_DOMAIN_ID="211", ROS_LOCALHOST_ONLY="1"),
        capture_output=True, text=True, timeout=15)
    assert result.returncode == 0, result.stdout + result.stderr
    rows = [line.removeprefix("UI_REVIEW ") for line in result.stdout.splitlines() if line.startswith("UI_REVIEW ")]
    assert len(rows) == 1
    row = json.loads(rows[0])
    assert row["pose_before"] is True and row["goal_before"] is True
    assert row["goal_on_lost"] is False
    assert row["pose_after_explicit_restart"] is True
    assert row["pose_after_second_restart"] is True
    return row


@pytest.mark.parametrize("key", ["pose_after_stop", "pose_after_lost", "goal_after_refresh",
    "pose_after_late_status", "goal_after_stop_refresh", "pose_after_unsolicited_session",
    "pose_for_foreign_status", "pose_in_mapping", "old_stop_label_in_new_session",
    "pose_after_stop_before_ack", "pose_after_lost_late_status", "pose_after_lost_new_session"])
def test_command_tool_remains_locked_after_stop_or_disconnect(ui_result, key):
    assert ui_result[key] is False, "Stale command tool/receipt must remain disabled: " + key


@pytest.mark.parametrize("key", ["retry_button_enabled", "retry_one_check_no_start",
    "checking_all_commands_locked", "recovered_idle_no_tools_or_old_stop",
    "held_retry_survives_status_refresh", "retry_failure_visible_with_reason",
    "retry_fast_failure_count_persists", "retry_success_visible"])
def test_retry_button_checks_cleanup_before_a_separate_start(ui_result, key):
    assert ui_result[key] is True, key


@pytest.mark.parametrize("key", ["start_feedback_immediate", "stop_feedback_immediate",
    "save_busy_label", "stop_busy_label"])
def test_request_feedback_is_visible_immediately_and_while_busy(ui_result, key):
    assert ui_result[key] is True, key
