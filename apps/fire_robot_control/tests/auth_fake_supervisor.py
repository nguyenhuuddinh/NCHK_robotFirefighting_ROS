"""Real worker prompt/pump + supervisor with an inert PTY replacement."""
import os
from pathlib import Path
import sys
import threading
from services.live_session import LiveSession
from services.supervisor import main

class AuthSession(LiveSession):
    def __init__(self,**kw):
        super().__init__(**kw)
        class Child:
            code=None
            def read(obj):return b'[sudo] password for pi: '
            def write(obj,data):
                assert data==b'fixture-only\n'
                obj.code=0;return len(data)
        self.fixture_child=Child()
    def start(self):
        self._status('STARTING_PI')
        self._read_pty(self.fixture_child)
        if os.environ.get('FIRE_AUTH_QA_MODE')=='timeout':
            def expired():
                self.emit(dict(event='FINISHED',session=self.session,robot_stop_confirmed=None,
                    cleanup_clean=True,session_was_started=False,error='FIXTURE_AUTH_TIMEOUT'))
            threading.Timer(.5,expired).start()
    def send(self,event,argument=None):
        if event=='AUTH':
            super().send(event,argument);self._pump()
            self.emit(dict(event='FINISHED',session=self.session,robot_stop_confirmed=None,
                cleanup_clean=True,session_was_started=False))
        elif event=='STOP':
            assert self.fixture_child.code is None,'Cancel must not send credentials'
            self.emit(dict(event='FINISHED',session=self.session,robot_stop_confirmed=None,
                cleanup_clean=True,session_was_started=False))
if __name__=='__main__':
    assert os.environ.get('ROS_DOMAIN_ID')=='211' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    raise SystemExit(main(session_factory=AuthSession))
