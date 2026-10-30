"""Regression cases found by the second pre-sync review; no Pi or ROS I/O.

The original six pre-sync regressions failed on e05. These cases also cover
failed thread starts and marker persistence; they never start a real worker.
"""
import errno
from pathlib import Path
import tempfile
import uuid
from unittest.mock import patch

import pytest

from services.supervisor import SessionController


class InertSession:
    def __init__(self, **kwargs):
        self.session = str(uuid.uuid4())
        self.emit = kwargs["emit"]
        self.started = False

    def start(self):
        self.started = True

    def send(self, *args):
        pass

    def heartbeat(self, *args):
        pass


@pytest.fixture
def controller():
    with tempfile.TemporaryDirectory(prefix="fire-review-fault-") as tmp:
        base = Path(tmp)
        for name in ("state", "runtime", "maps"):
            (base / name).mkdir(mode=0o700)
        yield SessionController(candidate=base, packet="a" * 64,
            state_dir=base / "state", runtime=base / "runtime",
            map_root=base / "maps", map_roots=[base / "maps"],
            session_factory=InertSession)


def finish(controller):
    controller.session.emit(dict(event="FINISHED", session=controller.session.session,
        robot_stop_confirmed=True, cleanup_clean=True, session_was_started=True,
        log_dir=str(controller.state_dir)))


def test_pending_write_failure_does_not_retain_an_unstarted_worker(controller):
    with patch("services.supervisor.atomic", side_effect=OSError(errno.ENOSPC, "QA full disk")):
        controller.action("SCAN")
    assert controller.session is None, "An unstarted worker cannot emit FINISHED or complete CLOSE"
    controller.action("CLOSE")
    assert controller.status()["close_ready"]


def test_pending_remove_failure_still_retires_the_finished_worker(controller):
    controller.action("SCAN")
    finish(controller)
    original = Path.unlink

    def fail_pending(path, *args, **kwargs):
        if path.name == "pending.json":
            raise PermissionError(errno.EACCES, "QA unlink denied")
        return original(path, *args, **kwargs)

    with patch.object(Path, "unlink", fail_pending):
        controller.tick()
    assert controller.session is None
    assert controller.phase == "BLOCKED"
    assert (controller.state_dir / "pending.json").exists()
    controller.action("CLOSE")
    assert controller.status()["close_ready"]


def test_new_session_does_not_report_the_previous_sessions_stop_proof(controller):
    controller.action("SCAN")
    finish(controller)
    controller.tick()
    assert controller.last_stop["robot_stop_confirmed"] is True
    controller.action("SCAN")
    assert controller.phase == "STARTING"
    assert controller.status()["last_stop"] is None, "Old STOP proof must not be rendered as current STOP"


@pytest.mark.parametrize("error", [RuntimeError("QA no thread slots"), OSError(errno.EAGAIN, "QA thread unavailable")])
def test_failed_worker_start_is_retired_and_app_can_close(controller, error):
    with patch.object(InertSession, "start", side_effect=error):
        controller.action("SCAN")
    assert controller.session is None
    assert controller.phase == controller.stage == "IDLE"
    assert not (controller.state_dir / "pending.json").exists()
    assert not any(row["event"] == "NEW_SESSION" for row in controller.tick())
    controller.action("CLOSE")
    assert controller.status()["close_ready"]


def test_failed_start_and_marker_removal_block_reuse_without_hanging_close(controller):
    with patch.object(InertSession, "start", side_effect=RuntimeError("QA thread unavailable")), \
         patch.object(Path, "unlink", side_effect=PermissionError(errno.EACCES, "QA unlink denied")):
        controller.action("SCAN")
    assert controller.session is None
    assert controller.phase == controller.stage == "BLOCKED"
    assert (controller.state_dir / "pending.json").is_file()
    assert "QA thread unavailable" in controller.error and "QA unlink denied" in controller.error
    controller.action("SCAN")
    assert controller.session is None
    controller.action("CLOSE")
    assert controller.status()["close_ready"]


def test_marker_left_by_a_failed_write_is_preserved_and_blocks_restart(controller):
    marker = controller.state_dir / "pending.json"

    def partial_write(path, value):
        path.write_text('{"qa_partial":true}')
        raise OSError(errno.ENOSPC, "QA late disk failure")

    with patch("services.supervisor.atomic", side_effect=partial_write):
        controller.action("SCAN")
    assert controller.session is None
    assert controller.phase == controller.stage == "BLOCKED"
    assert marker.read_text() == '{"qa_partial":true}'
    controller.action("CLOSE")
    assert controller.status()["close_ready"]
    reopened = SessionController(candidate=controller.candidate, packet=controller.packet,
        state_dir=controller.state_dir, runtime=controller.runtime, map_root=controller.map_root,
        map_roots=controller.map_roots, session_factory=InertSession)
    reopened.action("SCAN")
    assert reopened.phase == "BLOCKED" and reopened.session is None
