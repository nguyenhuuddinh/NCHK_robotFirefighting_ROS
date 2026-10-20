"""Offline-only IPC tests. Hardware is replaced by fake objects or a PTY.

Written for user execution; do not treat their presence as a passing result.
"""
import ctypes
import json
import os
from pathlib import Path
import pty
import signal
import socket
import subprocess
import sys
import time
from types import SimpleNamespace

import pytest
from fire_robot_bringup import serial_process as ipc
from fire_robot_bringup import serial_transport_core as core


CONFIG = dict(port='/tmp/qa_no_hardware', baudrate=115200,
              first_state_timeout_s=8., pump_command_timeout_s=.7)


@pytest.fixture
def transport(monkeypatch):
    clock = SimpleNamespace(now=100.)
    fake = SimpleNamespace(monotonic=lambda: clock.now)
    monkeypatch.setattr(ipc, 'time', fake)
    monkeypatch.setattr(core, 'time', fake)
    worker = ipc.ProcessTransport(CONFIG, lambda *a, **k: None)
    worker.session_ready = worker.telemetry_healthy = True
    worker.session_epoch = 4
    worker._session_state = 'HEALTHY'
    worker.ser = SimpleNamespace(write=lambda b: len(b))
    return worker, clock


def packet(kind='CMD', values=None, stamp=100., epoch=4, seq=1):
    return dict(type='command', kind=kind, values=[.1, 0.] if values is None else values,
                mono=stamp, epoch=epoch, id=seq)


@pytest.mark.parametrize('stamp,epoch', [(99.7,4),(100.1,4),(100.,3),(float('nan'),4)])
def test_stale_future_epoch_commands_rejected(transport, stamp, epoch):
    worker, _ = transport
    assert not worker.accept_command(packet(stamp=stamp, epoch=epoch))
    assert worker.pending_cmd is None


def test_latest_command_and_duplicate_fence(transport):
    worker, _ = transport
    assert worker.accept_command(packet())
    assert not worker.accept_command(packet(seq=1, values=[.9,0.]))
    assert worker.accept_command(packet(seq=2, values=[.2,0.]))
    assert worker.pending_cmd[0] == (.2,0.)


@pytest.mark.parametrize('kind,values', [('CMD',[.2,0.]),('PUMP',[1]),('FIRE',[.2,.3])])
def test_deadline_checked_at_actual_write(transport, kind, values):
    worker, clock = transport
    writes=[];worker.ser=SimpleNamespace(write=lambda b: writes.append(b) or len(b))
    assert worker.accept_command(packet(kind,values))
    items=[i for i in worker._pending_snapshot() if i[0]==kind]
    clock.now+=.21
    for item in items: worker._write_pending_item(item,worker.ser)
    assert not writes


def test_lease_latches_off_and_cannot_auto_rearm(transport):
    worker, clock = transport
    assert worker.accept_command(packet('PUMP',[1]))
    clock.now+=.51
    worker.heartbeat(clock.now)
    assert worker.ipc_fault=='parent_lease_expired'
    assert not worker.accept_command(packet(stamp=clock.now,seq=2))
    assert worker.accept_command(packet(values=[0.,0.],stamp=clock.now,seq=3))
    assert worker.pending_fire is None
    assert worker.pending_pump[0]==0
    assert worker.ipc_fault


def test_off_allowed_even_when_telemetry_unhealthy(transport):
    worker, _ = transport
    worker.telemetry_healthy=False
    worker.fail_closed('test')
    items=worker._pending_snapshot()
    assert any(i[0]=='CMD' and i[1]==(0.,0.) for i in items)
    assert any(i[0]=='PUMP' and i[1]==0 for i in items)


def test_old_snapshot_cannot_write_after_new_safe_command(transport):
    worker, _ = transport
    assert worker.accept_command(packet())
    old=worker._pending_snapshot()[0]
    assert worker.accept_command(packet(values=[0.,0.],seq=2))
    writes=[];worker.ser=SimpleNamespace(write=lambda b:writes.append(b) or len(b))
    worker._write_pending_item(old,worker.ser)
    assert writes==[]


@pytest.mark.parametrize('values', [[float('inf'),0],[float('nan'),0],[True,0],[],['1',0]])
def test_invalid_command_fails_closed(transport, values):
    worker,_=transport
    with pytest.raises(ValueError):worker.accept_command(packet(values=values))


def test_ipc_buffer_full_is_nonblocking_and_eof_detected():
    left,right=ipc.channel_pair()
    try:
        for _ in range(10000):
            if not ipc.send_packet(left,dict(payload='x'*1000)):break
        else:pytest.fail('socket did not enforce bounded buffering')
        left.close()
        with pytest.raises(EOFError):
            for _ in range(100):ipc.receive_packets(right)
    finally:left.close();right.close()


@pytest.mark.parametrize('payload', [b'[]',b'{bad',b'x'*(ipc.PACKET_LIMIT+1)])
def test_malformed_or_oversized_ipc_rejected(payload):
    left,right=ipc.channel_pair()
    try:
        left.send(payload)
        with pytest.raises((ValueError,EOFError)):ipc.receive_packets(right)
    finally:left.close();right.close()


# Emulator is a separate interpreter: it continues generating bytes even when
# the test parent's native call holds the Python GIL. It opens only a PTY master.
EMULATOR = r'''
import os,sys,time,select,json
from fire_robot_bringup.serial_protocol import crc16_ccitt_false
fd=int(sys.argv[1]);out=open(sys.argv[2],'w',buffering=1);seq=0;buf=b'';next_state=0
os.set_blocking(fd,False)
while True:
 now=time.monotonic()
 if now>=next_state:
  seq+=1;payload=f'STATE,2,{seq},{seq*100},0.000,0.000,0.000,0.000,0.000,0.000'
  data=f'@{payload}*{crc16_ccitt_false(payload.encode()):04X}\n'.encode()
  try:os.write(fd,data)
  except (BlockingIOError,OSError):pass
  next_state=now+.1
 try:
  data=os.read(fd,4096);buf+=data
  while b'\n' in buf:
   line,buf=buf.split(b'\n',1);out.write(json.dumps(dict(mono=time.monotonic(),line=line.decode(errors='replace')))+'\n')
 except (BlockingIOError,OSError):pass
 time.sleep(.002)
'''


@pytest.fixture
def rig(tmp_path):
    master,slave=pty.openpty();path=os.ttyname(slave)
    log=tmp_path/'wire.jsonl'
    emulator=subprocess.Popen([sys.executable,'-c',EMULATOR,str(master),str(log)],pass_fds=(master,))
    client=None
    try:
        client=ipc.SerialProcessClient(dict(CONFIG,port=path))
        deadline=time.monotonic()+5
        while time.monotonic()<deadline:
            client.heartbeat();status=client.poll()
            if status and status['healthy']:break
            time.sleep(.02)
        else:pytest.fail('PTY worker not healthy')
        yield client,log
    finally:
        if client is not None:client.close()
        emulator.terminate()
        try:emulator.wait(timeout=2)
        except subprocess.TimeoutExpired:emulator.kill();emulator.wait(timeout=2)
        os.close(master);os.close(slave)


def drive(client, seconds):
    deadline=time.monotonic()+seconds;latest=None
    while time.monotonic()<deadline:
        client.heartbeat();status=client.poll()
        if status:latest=status
        time.sleep(.01)
    return latest


def test_native_parent_stall_does_not_stop_child_serial_and_latches_safe(rig):
    client,log=rig
    before=drive(client,.15);epoch=before['epoch']
    client.command('CMD',[.1,0.],epoch);client.command('PUMP',[1],epoch)
    drive(client,.1)
    baseline=[json.loads(line)['line'] for line in log.read_text().splitlines()]
    assert any(',0.100,0.000*' in x for x in baseline)
    start=time.monotonic()
    libc=ctypes.PyDLL(None);libc.usleep.argtypes=[ctypes.c_uint];libc.usleep(900000)
    after=drive(client,.15)
    assert after['epoch']==epoch
    assert after['fault']=='parent_lease_expired'
    assert after['healthy']
    assert after['read_max_ms']<300
    client.command('CMD',[.4,0.],epoch)
    drive(client,.1)
    rows=[json.loads(line) for line in log.read_text().splitlines()]
    late=[x['line'] for x in rows if x['mono']>start+.6]
    assert any(x.startswith('@CMD,2,') and ',0.000,0.000*' in x for x in late)
    assert any(x.startswith('@PUMP,2,') and x.split('*')[0].endswith(',0') for x in late)
    assert not any(',0.400,0.000*' in x for x in late)


def test_worker_crash_is_latched_without_restart(rig):
    client,_=rig;pid=client.process.pid
    client.process.kill();client.process.wait(timeout=2)
    assert client.poll() is None
    assert client.fault and client.process.pid==pid
    assert not client.command('CMD',[.3,0.],1)


def test_close_reaps_worker_and_keeps_unrelated_process(rig):
    client,log=rig
    unrelated=subprocess.Popen([sys.executable,'-c','import time;time.sleep(30)'])
    try:
        client.close();assert client.process.poll() is not None
        assert unrelated.poll() is None
        rows=[json.loads(x)['line'] for x in log.read_text().splitlines()]
        assert any(x.startswith('@PUMP,2,') and x.split('*')[0].endswith(',0') for x in rows)
    finally:unrelated.terminate();unrelated.wait(timeout=2)


def test_ipc_eof_shuts_down_worker(rig):
    client,_=rig
    client.endpoint.close()
    client.process.wait(timeout=5)
    assert client.process.returncode == 0


def test_parent_drops_stale_status_and_does_not_republish_same_frame(monkeypatch):
    from fire_robot_bringup import serial_bridge_node as bridge
    import threading
    now=100.
    monkeypatch.setattr(bridge, 'time', SimpleNamespace(monotonic=lambda: now))
    status=dict(mono=100.,ready=True,healthy=True,fault='',epoch=4,state='HEALTHY',
                slots={name:None for name in ipc.SLOTS})
    status['slots']['latest_state']=[dict(type='STATE'),4,99.9]
    process=SimpleNamespace(fault='',heartbeat=lambda:None,poll=lambda:status)
    node=SimpleNamespace(transport_process=process,state_lock=threading.Lock(),
                         _ipc_seen={},_ipc_fault_logged=False)
    for name in ipc.SLOTS:setattr(node,name,None)
    bridge.SerialBridgeNode._poll_transport(node)
    assert node.latest_state[2]==99.9
    node.latest_state=None
    bridge.SerialBridgeNode._poll_transport(node)
    assert node.latest_state is None
    status['mono']=99.5
    bridge.SerialBridgeNode._poll_transport(node)
    assert not node.session_ready and not node.telemetry_healthy
