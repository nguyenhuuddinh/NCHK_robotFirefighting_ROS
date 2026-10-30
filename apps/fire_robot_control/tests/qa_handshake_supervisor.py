"""Synthetic supervisor worker, real Qt client and private operator socket; no ROS."""
import os
from pathlib import Path
import threading
import time
import uuid
from services.frontend_lease import FrontendLease
from services.operator_channel import OperatorChannel
from services.operator_runner import OperatorBootstrap
from services.supervisor import main

class FakeSession:
    def __init__(self, **kw):
        self.session = str(uuid.uuid4())
        self.kind = kw['kind']
        assert self.kind == 'fire'
        self.emit = kw['emit']
        self.out = Path(kw['out'])
        self.binding = OperatorBootstrap(self.session, 'b'*32, 'c'*32, kw['packet'], Path(kw['runtime'])/'operator.sock')
        self.channel = None
    def heartbeat(self, now): pass
    def start(self):
        b = self.binding
        self.channel = OperatorChannel(b.socket_path, FrontendLease(b.session,b.generation,b.nonce,os.getuid(),time.monotonic()),clock=time.monotonic)
        self.emit(dict(event='SESSION', session=self.session, stage='CONSUMERS'))
        self.emit(dict(event='OPERATOR', session=self.session, generation=b.generation, nonce=b.nonce, socket_path=str(b.socket_path)))
        threading.Thread(target=self.loop, daemon=True).start()
    def loop(self):
        active = False
        try:
            end = time.monotonic()+8
            while time.monotonic() < end:
                key = self.channel.poll(time.monotonic())
                if self.channel.lease.active and not active:
                    active = True
                    self.emit(dict(event='SESSION', session=self.session, stage='READY_GATE2'))
                if key == 'Q':
                    (self.out/'handshake.json').write_text(__import__('json').dumps(dict(active=active, reason=self.channel.lease.reason)))
                    self.emit(dict(event='FINISHED', session=self.session, robot_stop_confirmed=active,
                        cleanup_clean=active, session_was_started=True, log_dir=str(self.out)))
                    return
                time.sleep(.01)
            self.emit(dict(event='FINISHED', session=self.session, robot_stop_confirmed=False,
                cleanup_clean=False, session_was_started=True, log_dir=str(self.out)))
        finally:
            self.channel.close()
    def send(self, event, argument=None): pass

if __name__ == '__main__':
    if os.environ.get('ROS_DOMAIN_ID') != '211' or os.environ.get('ROS_LOCALHOST_ONLY') != '1':
        raise SystemExit('QA requires localhost-only domain 211')
    raise SystemExit(main(session_factory=FakeSession))
