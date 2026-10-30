"""Actual Qt popup/mouse regression with an inert socket and localhost ROS."""
import json
import os
from pathlib import Path
import subprocess
import pytest

APP = Path(__file__).resolve().parents[1]
BINARY = Path("/tmp/fire_robot_pre_sync_ui/map_selection_probe")


@pytest.fixture(scope="module")
def result():
    assert BINARY.is_file()
    process = subprocess.run([str(BINARY), str(APP / "native_rviz/operational.rviz")],
        env=dict(os.environ, ROS_DOMAIN_ID="211", ROS_LOCALHOST_ONLY="1"),
        text=True, capture_output=True, timeout=15)
    assert process.returncode == 0, process.stdout + process.stderr
    rows = [line.removeprefix("MAP_SELECTION_QA ") for line in process.stdout.splitlines()
            if line.startswith("MAP_SELECTION_QA ")]
    assert len(rows) == 1
    return json.loads(rows[0])


def test_idle_popup_preserves_highlight_during_status_updates(result):
    assert result["idle_enabled"] is True
    assert result["count"] == 3
    assert result["unchanged_model_resets"] == 0
    assert result["popup_visible"] is True
    assert result["highlight_after_refresh"] == 2


def test_mouse_selection_sends_one_select_without_starting_a_session(result):
    assert result["mouse_selected_path"] == "/tmp/map_two.yaml"
    assert result["mouse_change_count"] == 1
    assert result["select_commands"] == 1
    assert result["select_command_path"] == "/tmp/map_two.yaml"
    assert result["start_commands"] == 0


def test_ack_and_new_catalog_preserve_or_update_selection_without_feedback(result):
    assert result["ack_selected_path"] == "/tmp/map_two.yaml"
    assert result["unchanged_model_resets_after_ack"] == 0
    assert result["new_catalog_count"] == 4
    assert result["new_catalog_selected_path"] == "/tmp/map_new.yaml"
