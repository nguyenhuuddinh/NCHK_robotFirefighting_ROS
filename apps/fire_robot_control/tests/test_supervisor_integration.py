"""Offline button/session QA; fake drivers never connect to robot or ROS."""
import json
import os
from pathlib import Path
import subprocess
import sys
from types import SimpleNamespace
import tempfile
import unittest
import uuid
from unittest.mock import patch

from services.candidate_build import build
from services.desktop_hooks import DesktopHooks, stop_peer_after_local
from services.live_session import receipt_result
from proof_fixture import clean_local, clean_remote
from services.map_catalog import validate_map
from services.map_snapshot import snapshot_map
from services.operator_runner import OperatorBootstrap
from services.pi_receipt import validate_pi_receipt
from services import live_preflight
from services.supervisor import ControlProtocol, SessionController, verify_packet

ROOT = Path(__file__).resolve().parents[3]
APP = ROOT / "apps/fire_robot_control"
SEALED = ROOT / "releases/r5_20261029/ros"
PACKET = "a" * 64


def make_map(root, name):
    (root / (name + ".pgm")).write_bytes(b"P5\n2 2\n255\n\x00\x00\xff\xff")
    (root / (name + ".yaml")).write_text("image: " + name + ".pgm\nresolution: 0.05\n"
        "origin: [0, 0, 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")


class FakeSession:
    def __init__(self, **kw):
        self.session = str(uuid.uuid4())
        self.args, self.emit = kw, kw["emit"]
        self.commands, self.started, self.last_heartbeat = [], False, None

    def start(self): self.started = True
    def send(self, kind, argument=None): self.commands.append((kind, argument))
    def heartbeat(self, at): self.last_heartbeat = at
    def stage(self, stage): self.emit(dict(event="SESSION", session=self.session, stage=stage))
    def finish(self, stop=True, clean=True):
        self.emit(dict(event="FINISHED", session=self.session, robot_stop_confirmed=stop,
                       cleanup_clean=clean, log_dir=str(self.args["out"]), error=None))


class ControllerTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.base = Path(self.tmp.name)
        self.maps = self.base / "maps"
        self.state = self.base / "state"
        self.run = self.base / "run"
        for p in (self.maps, self.state, self.run): p.mkdir(mode=0o700)
        make_map(self.maps, "original")
        self.ctrl = self.create()

    def tearDown(self): self.tmp.cleanup()

    def create(self):
        return SessionController(candidate=self.base, packet=PACKET, state_dir=self.state,
            runtime=self.run, map_roots=[self.maps], map_root=self.maps, session_factory=FakeSession,
            clock=lambda: 123.0)

    def test_fire_operator_announcement_forwarded_only_for_bound_live_session(self):
        c = self.ctrl
        c.action("SELECT", str(self.maps / "original.yaml"))
        c.action("FIRE")
        worker = c.session
        worker.kind = "fire"
        worker.binding = OperatorBootstrap(worker.session, "b" * 32, "c" * 32, PACKET, self.run / "operator.sock")
        b = worker.binding
        row = dict(event="OPERATOR", session=b.session, generation=b.generation,
                   nonce=b.nonce, socket_path=str(b.socket_path))
        c.tick()
        worker.emit(row)
        self.assertEqual([e for e in c.tick() if e.get("event") == "OPERATOR"], [row])
        for key, value in (("session", str(uuid.uuid4())), ("generation", "d" * 32),
                           ("nonce", "d" * 32), ("socket_path", str(self.run / "other.sock"))):
            worker.emit(dict(row, **{key: value}))
            self.assertFalse(any(e.get("event") == "OPERATOR" for e in c.tick()))
        worker.kind = "mapping"
        worker.emit(row)
        self.assertFalse(any(e.get("event") == "OPERATOR" for e in c.tick()))
        worker.kind = "fire"
        c.action("STOP")
        worker.emit(row)
        self.assertFalse(any(e.get("event") == "OPERATOR" for e in c.tick()))

    def test_operator_announcement_is_removed_if_worker_finishes_in_same_batch(self):
        c = self.ctrl
        c.action("SELECT", str(self.maps / "original.yaml")); c.action("FIRE")
        worker = c.session; worker.kind = "fire"
        worker.binding = OperatorBootstrap(worker.session, "b" * 32, "c" * 32, PACKET, self.run / "operator.sock")
        b = worker.binding
        worker.emit(dict(event="OPERATOR", session=b.session, generation=b.generation,
                         nonce=b.nonce, socket_path=str(b.socket_path)))
        worker.finish(False, False)
        self.assertFalse(any(e.get("event") == "OPERATOR" for e in c.tick()))
        self.assertEqual(c.phase, "BLOCKED")

    def test_scan_save_stop_select_and_fire_new_or_existing_map(self):
        c = self.ctrl
        c.action("SCAN")
        mapping = c.session
        self.assertTrue(mapping.started)
        mapping.stage("MAPPING")
        c.tick()
        c.action("SAVE", "new")
        save = c.save_id
        c.action("SAVE", "new2")
        self.assertEqual(len([x for x in mapping.commands if x[0] == "SAVE"]), 1)
        make_map(self.maps, "new")
        mapping.emit(dict(event="SAVED", session=mapping.session, id=save,
            map={"yaml_path": str(self.maps / "new.yaml")}))
        c.tick()
        self.assertEqual(c.selected.name, "new")
        c.action("FIRE")
        self.assertIs(c.session, mapping)
        self.assertIn("dừng", c.error)
        c.action("STOP")
        mapping.finish()
        c.tick()
        self.assertEqual(c.phase, "IDLE")
        c.action("FIRE")
        self.assertEqual(c.session.args["selected_map"].name, "new")
        c.session.finish()
        c.tick()
        c.action("SELECT", str(self.maps / "original.yaml"))
        c.action("FIRE")
        self.assertEqual(c.session.args["selected_map"].name, "original")

    def test_stop_during_startup_wins_and_late_callbacks_cannot_reopen(self):
        c = self.ctrl
        c.action("SCAN")
        first = c.session
        c.action("SCAN")
        self.assertIs(c.session, first)
        c.action("STOP")
        first.stage("MAPPING")
        c.tick()
        self.assertEqual(c.phase, "STOPPING")
        first.finish()
        c.tick()
        c.action("SCAN")
        second = c.session
        first.stage("MAPPING")
        first.finish(False, False)
        c.tick()
        self.assertIs(c.session, second)
        self.assertEqual(c.phase, "STARTING")

    def test_unconfirmed_stop_or_unclean_exit_blocks_restarts_across_app_restart(self):
        for stopped, clean in ((False, True), (True, False), (False, False)):
            with self.subTest(stopped=stopped, clean=clean):
                c = self.create()
                c.action("SCAN")
                c.session.finish(stopped, clean)
                c.tick()
                self.assertEqual(c.phase, "BLOCKED")
                c.action("SCAN")
                self.assertIsNone(c.session)
                self.assertEqual(self.create().phase, "BLOCKED")
                (self.state / "pending.json").unlink()

    def test_map_modified_after_selection_never_starts(self):
        c = self.ctrl
        c.action("SELECT", str(self.maps / "original.yaml"))
        (self.maps / "original.pgm").write_bytes(b"P5\n2 2\n255\n\xff\xff\xff\xff")
        c.action("FIRE")
        self.assertIsNone(c.session)
        self.assertEqual(c.phase, "IDLE")

    def test_invalid_save_names_existing_map_and_close_while_saving(self):
        c = self.ctrl
        c.action("SCAN")
        session = c.session
        session.stage("MAPPING")
        c.tick()
        for name in ("", "../escape", "x;touch", "original", "x" * 65, None):
            c.action("SAVE", name)
            self.assertIsNone(c.save_id)
        c.action("SAVE", "test")
        operation = c.save_id
        c.action("CLOSE")
        self.assertEqual(c.phase, "STOPPING")
        self.assertFalse(c.status()["close_ready"])
        session.emit(dict(event="SAVED", session=session.session, id=operation, error="canceled"))
        session.finish(False, False)
        c.tick()
        self.assertTrue(c.status()["close_ready"])
        self.assertEqual(c.phase, "BLOCKED")

    def test_foreign_save_result_is_ignored(self):
        c = self.ctrl
        c.action("SCAN")
        c.session.stage("MAPPING")
        c.tick()
        c.action("SAVE", "test")
        pending = c.save_id
        c.session.emit(dict(event="SAVED", session=c.session.session, id="old", error="fake"))
        c.tick()
        self.assertEqual(c.save_id, pending)


class ProtocolTests(unittest.TestCase):
    def setUp(self):
        self.boot = OperatorBootstrap(str(uuid.uuid4()), "b" * 32, "c" * 32, PACKET, Path("/tmp/control.sock"))
        self.p = ControlProtocol(self.boot, 10.0)

    def msg(self, seq=1, event="HEARTBEAT", now=10.0, **overrides):
        row = dict(version=1, session=self.boot.session, generation=self.boot.generation,
            nonce=self.boot.nonce, seq=seq, sent_mono=now, event=event, argument=None)
        row.update(overrides)
        return json.dumps(row).encode()

    def test_replay_wrong_identity_uid_future_and_expiry_fail_closed(self):
        self.assertIsNone(self.p.accept(self.msg(nonce="d" * 32), 10., os.getuid()))
        self.assertIsNone(self.p.accept(self.msg(), 10., os.getuid() + 1))
        self.assertIsNone(self.p.accept(self.msg(now=10.1), 10., os.getuid()))
        self.assertEqual(self.p.accept(self.msg(), 10., os.getuid()), ("HEARTBEAT", None))
        self.assertIsNone(self.p.accept(self.msg(), 10., os.getuid()))
        self.assertTrue(self.p.expired(10.36))
        self.assertIsNone(self.p.accept(self.msg(seq=2, now=10.36), 10.36, os.getuid()))

    def test_invalid_event_duplicate_key_and_flood_never_become_actions(self):
        for raw in (self.msg(event="HACK"), b'{"seq":1,"seq":2}', b"x" * 4097,
                    self.msg(seq=True), self.msg(event="FIRE", argument=float("nan"))):
            with self.subTest(raw=raw[:80]):
                self.assertIsNone(self.p.accept(raw, 10., os.getuid()))


class ReceiptTests(unittest.TestCase):
    def test_strict_current_session_packet_and_remote_binding(self):
        binding = dict(session="p", secret="s", packet=PACKET, boot_id="b", runner_pid=7,
                       runner_start_ticks=123, root="/tmp/pi")
        row = dict(session="l", packet=PACKET, rc=0, profile="DESKTOP_MAPPING",
            local=clean_local(), remote=clean_remote(
                {k:v for k,v in binding.items() if k != "secret"}))
        self.assertEqual(receipt_result(row, "l", PACKET, remote_binding=binding), (True, True))
        for broken in (dict(row, session="old"), dict(row, packet="b" * 64),
                       dict(row, remote=dict(confirmed=True, binding={})),
                       dict(row, local=dict(stop_confirmed=False, clean=True))):
            self.assertNotEqual(receipt_result(broken, "l", PACKET, remote_binding=binding), (True, True))

    def test_pi_stop_is_deferred_until_all_owned_local_groups_are_proven_gone(self):
        class Process:
            def __init__(self, exited): self.exited = exited
            def report(self): return {"group_exit_confirmed": self.exited}
        class Local:
            def __init__(self): self.processes = {"operator": Process(True), "nav": Process(False)}
        class Peer:
            bind = {"session": "s", "secret": "not-in-result"}
            calls = 0
            def stop(self):
                self.calls += 1
                return {"confirmed": True}
        local, peer = Local(), Peer()
        result = stop_peer_after_local(local, peer)
        self.assertEqual(peer.calls, 0)
        self.assertEqual(result["reason"], "LOCAL_GROUP_EXIT_UNCONFIRMED")
        self.assertNotIn("secret", result["binding"])
        local.processes["nav"].exited = True
        self.assertTrue(stop_peer_after_local(local, peer)["confirmed"])
        self.assertEqual(peer.calls, 1)

    def test_preflight_rejection_is_not_reported_as_a_confirmed_robot_stop(self):
        # No process/SSH was started; the interface may return to IDLE without
        # presenting this as telemetry confirmation from the physical robot.
        with tempfile.TemporaryDirectory() as temp:
            base = Path(temp)
            maps, state, run = (base / name for name in ("maps", "state", "run"))
            for path in (maps, state, run): path.mkdir(mode=0o700)
            c = SessionController(candidate=base, packet=PACKET, state_dir=state, runtime=run,
                map_roots=[maps], map_root=maps, session_factory=FakeSession)
            c.action("SCAN")
            c.session.emit(dict(event="FINISHED", session=c.session.session, robot_stop_confirmed=None,
                cleanup_clean=True, session_was_started=False, log_dir=str(base), error="Preflight failed"))
            c.tick()
            self.assertEqual(c.phase, "IDLE")
            self.assertIsNone(c.last_stop["robot_stop_confirmed"])

    def test_stop_monitor_only_starts_if_pi_started_and_normal_operator_is_absent(self):
        class Process:
            done = False
            def tick(self, now): pass
        class Local:
            def __init__(self, stream):
                self.processes = {}
                self.registry = type("Registry", (), {"logs": {"operator": type("Log", (), {"file": stream})()}})()
                self.calls = 0
            def start(self, role, command):
                self.calls += 1
                self.processes[role] = Process()
                return self.processes[role]
        hooks = object.__new__(DesktopHooks)
        hooks.fallback = False
        with tempfile.TemporaryFile() as stream, patch("services.monitor_log.start_monitor", side_effect=lambda local, argv:local.start("operator",argv)):
            stream.write(b"MAPPING_MONITOR_READY\n")
            stream.flush()
            local = Local(stream)
            self.assertFalse(hooks.prepare_stop_monitor(local, remote_started=False))
            self.assertEqual(local.calls, 0)
            self.assertTrue(hooks.prepare_stop_monitor(local, remote_started=True))
            self.assertEqual(local.calls, 1)
            self.assertFalse(hooks.prepare_stop_monitor(local, remote_started=True))
            self.assertEqual(local.calls, 1)


class CandidateTests(unittest.TestCase):
    def test_live_preflight_requires_clean_laptop_static_inventory(self):
        modules = {
            "operational_entry": SimpleNamespace(packet_digest=lambda root: PACKET),
            "operational_preflight": SimpleNamespace(preflight=lambda root, role: None,
                                                       validate_gate2_profile=lambda root: None),
            "r5_install_check": SimpleNamespace(inventory=lambda root, role: {
                "status": "STATIC_INSTALL_BLOCKED", "errors": [{"check": "yolo_model"}]}),
        }
        with patch.dict(sys.modules, modules), patch.object(sys, "argv", ["preflight", "/tmp", PACKET]):
            with self.assertRaisesRegex(ValueError, "LAPTOP_STATIC_CHECK_FAILED:yolo_model"):
                live_preflight.main()

    def test_candidate_is_deterministic_separate_and_tamper_evident(self):
        with tempfile.TemporaryDirectory() as tmp:
            parent_before = (SEALED / "operational_packet.sha256").read_bytes()
            one, two = Path(tmp) / "one", Path(tmp) / "two"
            first = build(SEALED, APP, one)
            second = build(SEALED, APP, two)
            self.assertEqual(first, second)
            self.assertEqual(verify_packet(one), first)
            from services.packet_archive import export_packet, verify_archive
            archive = Path(tmp) / "candidate.tar.gz"
            self.assertEqual(export_packet(one, archive)["packet_sha256"], first)
            self.assertEqual(verify_archive(archive, first)["packet_sha256"], first)
            self.assertEqual(parent_before, (SEALED / "operational_packet.sha256").read_bytes())
            runner = one / "scripts/r5_laptop_session.py"
            compile(runner.read_text(), str(runner), "exec")
            runner.write_text(runner.read_text() + "\n# tampered\n")
            with self.assertRaises(ValueError): verify_packet(one)

    def test_snapshot_freezes_selected_pixels_and_checks_source_changes(self):
        with tempfile.TemporaryDirectory() as temp:
            root = Path(temp)
            maps = root / "maps"
            maps.mkdir()
            make_map(maps, "original")
            record = validate_map(maps / "original.yaml", [maps])
            path = snapshot_map(record, [maps], root / "snapshot")
            original_pixels = (root / "snapshot/image.pgm").read_bytes()
            (maps / "original.pgm").write_bytes(b"P5\n2 2\n255\n\xff\xff\xff\xff")
            self.assertEqual((root / "snapshot/image.pgm").read_bytes(), original_pixels)
            self.assertTrue(path.is_file())
            with self.assertRaises(ValueError):
                snapshot_map(record, [maps], root / "another")

    def test_pi_receipt_requires_matching_packet_source_paths_and_static_scope(self):
        with tempfile.TemporaryDirectory() as temp:
            path = Path(temp) / "receipt.json"
            root = "/home/pi/fire_robot_desktop/" + PACKET[:12] + "/ros/ros2_ws/install/pkg/"
            sources = [dict(path=root + str(i), sha256="b" * 64) for i in range(103)]
            row = dict(schema_version=1, role="pi", host_machine="aarch64", live=False,
                runtime_verified=False, status="STATIC_INSTALL_PASS_RUNTIME_NOT_VERIFIED",
                errors=[], checks=dict(source_packet_sha256=PACKET, installed_sources=sources))
            path.write_text(json.dumps(row))
            self.assertEqual(validate_pi_receipt(path, PACKET)["role"], "pi")
            for altered in (dict(row, live=True), dict(row, host_machine="x86_64"),
                            dict(row, checks=dict(row["checks"], source_packet_sha256="c" * 64)),
                            dict(row, checks=dict(row["checks"], installed_sources=sources[:-1])),
                            dict(row, checks=dict(row["checks"], installed_sources=sources[:102] + [sources[0]]))):
                path.write_text(json.dumps(altered))
                with self.assertRaises(ValueError): validate_pi_receipt(path, PACKET)


if __name__ == "__main__":
    unittest.main()
