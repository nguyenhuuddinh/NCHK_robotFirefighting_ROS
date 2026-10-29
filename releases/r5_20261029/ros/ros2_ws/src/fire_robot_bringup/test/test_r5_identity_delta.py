"""Only the new BOOT/current-connection telemetry boundary; no devices/ROS init."""
import json
import threading
from types import SimpleNamespace as N
import pytest

from fire_robot_bringup.serial_session_identity import SerialSessionIdentity
from fire_robot_bringup.serial_bridge_node import SerialBridgeNode

ELF='92ffd8e2190230d0550bae7da744e100e6880b08085d7495319445064d32e415'


def boot(ms=100,elf=ELF):return dict(type='BOOT',seq=1,esp_ms=ms,reset_reason=1,elf_sha256=elf)
def state(ms=120,seq=2):return dict(type='STATE',seq=seq,esp_ms=ms)


def identity():
    x=SerialSessionIdentity('ab'*16);x.observe(boot(),1,1.);x.observe(state(),1,1.01)
    return x


def test_BOOT_never_substitutes_for_STATE_or_bridge_readiness():
    x=SerialSessionIdentity('ab'*16);x.observe(boot(),1,1.)
    assert not x.snapshot(1,1.,True,True)['valid']
    x.observe(state(),1,1.01)
    assert not x.snapshot(1,1.02,False,True)['valid']
    assert not x.snapshot(1,1.02,True,False)['valid']
    report=x.snapshot(1,1.02,True,True)
    assert report['valid'] and report['elf_sha256']==ELF
    assert report['state_age_ms']==10. and report['updates_bridge_health'] is False


def test_new_connection_cannot_reuse_old_BOOT_or_STATE():
    x=identity();assert x.snapshot(1,1.1,True,True)['valid']
    assert not x.snapshot(2,1.1,True,True)['valid']
    x.observe(state(200),2,1.11)
    assert not x.snapshot(2,1.12,True,True)['valid']
    x.observe(boot(210),2,1.13)
    assert not x.snapshot(2,1.14,True,True)['valid']
    x.observe(state(220,3),2,1.15)
    assert x.snapshot(2,1.16,True,True)['valid']


def test_stale_and_repeated_STATE_cannot_be_refreshed_by_a_timer():
    x=identity()
    x.observe(state(120,3),1,1.3)
    r=x.snapshot(1,1.31,True,True)
    assert r['state_age_ms']==300. and not r['valid']
    x.observe(state(140,4),1,1.32)
    assert x.snapshot(1,1.33,True,True)['valid']


@pytest.mark.parametrize('fault',['unknown','bad_hash','bool_ms','negative_seq','ELF_changed','MCU_reboot','clock'])
def test_invalid_identity_latches_until_new_transport_epoch(fault):
    x=identity();frame=boot(130)
    if fault=='unknown':frame['elf_sha256']='unknown'
    if fault=='bad_hash':frame['elf_sha256']='g'*64
    if fault=='bool_ms':frame['esp_ms']=True
    if fault=='negative_seq':frame['seq']=-1
    if fault=='ELF_changed':frame['elf_sha256']='e'*64
    if fault=='MCU_reboot':frame=state(5)
    at=.9 if fault=='clock' else 1.02
    x.observe(frame,1,at)
    x.observe(state(150),1,1.03)
    assert not x.snapshot(1,1.04,True,True)['valid']
    x.observe(boot(1),2,1.05);x.observe(state(5),2,1.06)
    assert x.snapshot(2,1.07,True,True)['valid']


def test_uint32_ESP_wrap_is_not_a_reboot():
    x=SerialSessionIdentity('ab'*16)
    x.observe(boot(2**32-50),1,1.)
    x.observe(state(2**32-20),1,1.01);x.observe(state(5),1,1.03)
    assert x.snapshot(1,1.04,True,True)['valid']


def test_bridge_actual_frame_and_publish_callbacks_add_only_read_only_identity(monkeypatch):
    import fire_robot_bringup.serial_bridge_node as bridge
    n=object.__new__(SerialBridgeNode);n.session_identity=SerialSessionIdentity('ab'*16)
    n.state_lock=threading.Lock();n.publish_lock=threading.Lock()
    n.running=n.session_ready=True;n.telemetry_healthy=False;n.session_epoch=1
    n.latest_state=n.latest_env=n.latest_ir=n.latest_actuator=None
    n.node_destroyed=False;n._recovery_diagnostics=None;n.last_identity_publish_mono=None
    n._transition_state=lambda *a:None;n.identity_pub=N(publish=lambda msg:sent.append(json.loads(msg.data)))
    now=[1.];sent=[];monkeypatch.setattr(bridge.time,'monotonic',lambda:now[0])
    n.handle_frame(boot());assert not n.telemetry_healthy
    n.telemetry_publish_callback();assert not sent[-1]['valid']
    now[0]=1.02;n.handle_frame(state())
    # Odom publication is outside this delta; the identity has its own copy.
    n.latest_state=None
    now[0]=1.21;n.telemetry_publish_callback()
    assert sent[-1]['valid'] and sent[-1]['connection_epoch']==1
    count=len(sent);now[0]=1.22;n.telemetry_publish_callback();assert len(sent)==count
    n.session_epoch=2;now[0]=1.42;n.telemetry_publish_callback()
    assert not sent[-1]['valid'] and sent[-1]['elf_sha256'] is None
