"""Exercise the app's inherited Pi transport without SSH or hardware."""
from unittest.mock import Mock
import fcntl
import json
from pathlib import Path

import pytest

from operational_peer import PeerTransportError
from services.app_peer import AppPeer


@pytest.mark.parametrize(
    "command,confirmed",
    [
        ("START", {"phase": "IO_STARTING", "retries": 0}),
        ("RETRY_PREARM", {"phase": "IO_STARTING", "retries": 1}),
        ("SEAL", {"phase": "RUNNING_OBSERVE", "sealed": True}),
    ],
)
@pytest.mark.parametrize("applied", [True, False])
def test_lost_mutation_ack_is_checked_without_replaying(command, confirmed, applied):
    peer = AppPeer({}, "/unused")
    state = confirmed if applied else {"phase": "WAIT_LAPTOP"}
    peer.request = Mock(side_effect=[PeerTransportError("lost ack"), state])

    if applied:
        assert peer.startup(command, sleep=lambda _: None) == confirmed
    else:
        with pytest.raises(PeerTransportError, match="PI_MUTATION_ACK_LOST_NOT_PROVEN"):
            peer.startup(command, sleep=lambda _: None)

    assert [call.args[0] for call in peer.request.call_args_list] == [command, "STATE"]


def test_state_transport_retries_are_bounded():
    peer = AppPeer({}, "/unused")
    peer.request = Mock(side_effect=PeerTransportError("unavailable"))
    waits = []
    with pytest.raises(PeerTransportError):
        peer.startup("STATE", sleep=waits.append)
    assert [call.args[0] for call in peer.request.call_args_list] == ["STATE"] * 3
    assert waits == [1.0, 1.0]


def test_r5_cli_refuses_when_app_holds_the_shared_laptop_lock(tmp_path, monkeypatch):
    """Run the original R5 entry with inert dependencies and a held lock."""
    import r5_laptop_session as laptop

    (tmp_path / ".cache").mkdir()
    lock_path = tmp_path / ".cache/fire_robot_r5_laptop.lock"
    pairing = tmp_path / "pairing.json"
    pairing.write_text(json.dumps({"packet": "a" * 64}))
    map_file = tmp_path / "map.yaml"
    view_file = tmp_path / "view.rviz"
    map_file.touch()
    view_file.touch()
    monkeypatch.setattr(laptop.Path, "home", lambda: tmp_path)
    monkeypatch.setattr(laptop, "packet_digest", lambda: "a" * 64)
    monkeypatch.setattr(laptop, "preflight", lambda *args: None)
    network = Mock(side_effect=AssertionError("no remote call while locked"))
    monkeypatch.setattr(laptop, "SshPi", lambda *args: network)
    args = ["--run-observe", "--pairing", str(pairing), "--remote-script", "/unused",
            "--out", str(tmp_path / "out"), "--map", str(map_file), "--rviz", str(view_file)]

    with lock_path.open("a") as owner:
        fcntl.flock(owner, fcntl.LOCK_EX | fcntl.LOCK_NB)
        with pytest.raises(BlockingIOError):
            laptop.main(args)

    network.assert_not_called()
    network.startup.assert_not_called()
    assert Path(lock_path).is_file()
