"""Supervisor wiring with fake session, used only by the isolated GUI test."""
import os
from pathlib import Path
import sys
import threading
import uuid
from dataclasses import asdict
from services.map_catalog import validate_map
from services.supervisor import main


class FakeSession:
    def __init__(self, **kwargs):
        self.session = str(uuid.uuid4())
        self.emit = kwargs["emit"]
        self.kind = kwargs["kind"]
        self.root = Path(kwargs["map_root"])
        self.out = kwargs["out"]

    def heartbeat(self, now): pass

    def begin_view_init(self):
        import time
        self.view_init_until = time.monotonic() + 3.
        return self.view_init_until

    def begin_async_mapping_view(self):
        assert self.kind == "mapping"

    def start(self):
        self.emit(dict(event="SESSION", session=self.session,
                       stage="WAIT_SLAM" if self.kind == "mapping" else "POSE"))

    def send(self, event, arg=None):
        if event == "VIEW_READY" and self.kind == "mapping":
            self.emit(dict(event="SESSION", session=self.session, stage="MAPPING"))
        elif event == "VIEW_READY" and self.kind == "fire":
            threading.Timer(.5, lambda: self.emit(dict(event="SESSION", session=self.session, stage="READY_GATE2"))).start()
        elif event == "SAVE":
            ident, name = arg
            (self.root / (name + ".pgm")).write_bytes(b"P5\n2 2\n255\n\0\0\xff\xff")
            (self.root / (name + ".yaml")).write_text("image: " + name + ".pgm\nresolution: 0.05\n"
                "origin: [0,0,0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
            record = validate_map(self.root / (name + ".yaml"), [self.root])
            self.emit(dict(event="SAVED", session=self.session, id=ident, map=asdict(record)))
        elif event == "STOP":
            self.emit(dict(event="FINISHED", session=self.session, robot_stop_confirmed=True,
                cleanup_clean=True, session_was_started=True, log_dir=str(self.out)))


if __name__ == "__main__":
    if os.environ.get("ROS_DOMAIN_ID") != "211" or os.environ.get("ROS_LOCALHOST_ONLY") != "1":
        raise SystemExit("Fake supervisor requires isolated ROS domain 211")
    import tempfile
    fd, state_path = tempfile.mkstemp(prefix="fake-gui-state-")
    os.close(fd)
    os.environ["FIRE_ROBOT_GUI_QA_STATE"] = state_path
    try:
        raise SystemExit(main(session_factory=FakeSession))
    finally:
        Path(state_path).unlink(missing_ok=True)
