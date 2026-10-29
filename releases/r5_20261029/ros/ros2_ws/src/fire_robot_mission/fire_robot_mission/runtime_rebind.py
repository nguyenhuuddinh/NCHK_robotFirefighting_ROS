"""Private runner/operator recovery handshake; no ROS or process effects.

An observed HOLD must precede RESUME of that exact generation/nonce. Files
cannot clear an unrelated fault, substitute a Pi, or authorize a pump replay.
"""
import json
import os
from pathlib import Path
import stat
import uuid


class RuntimeRebindGate:
    def __init__(self, start):
        self.path=Path(start.path).parent/'runtime_rebind.json'
        self.session=start.session;self.packet=start.packet
        self.generation=0;self.nonce=None;self.held=False;self.applied=0
        self.reason=None
        self.cancel_at=None;self.cancel_uuid=None;self.cancel_requested=False

    def read(self):
        try:
            fd=os.open(self.path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
            with os.fdopen(fd) as stream:
                meta=os.fstat(stream.fileno())
                if not stat.S_ISREG(meta.st_mode) or meta.st_uid!=os.getuid() or meta.st_mode&0o022:return None
                raw=stream.read(8193)
            d=json.loads(raw) if len(raw)<=8192 else None
            if (not isinstance(d,dict) or type(d.get('schema_version')) is not int or d['schema_version']!=1
                    or d.get('kind')!='R5_RUNTIME_REBIND' or d.get('session')!=self.session
                    or d.get('packet')!=self.packet or type(d.get('generation')) is not int
                    or d['generation']<1 or d.get('phase') not in ('HOLD','RESUME')
                    or type(d.get('after_pulse')) is not bool or not isinstance(d.get('reason'),str) or not d['reason']
                    or str(uuid.UUID(d['nonce']))!=d['nonce']):return None
            return d
        except (OSError,ValueError,TypeError,KeyError,AttributeError):return None

    def observe(self):
        d=self.read()
        if d is None:return None
        if d['phase']=='HOLD' and d['generation']>self.generation:
            self.generation=d['generation'];self.nonce=d['nonce'];self.held=True
            self.reason=str(d.get('reason','WAIT_CURRENT_IO'))
            return d
        if (d['phase']=='HOLD' and self.held and d['generation']==self.generation
                and d['nonce']==self.nonce):
            self.reason=d['reason']
            return None
        if (d['phase']=='RESUME'  and self.held and d['generation']==self.generation
                and d['nonce']==self.nonce and self.applied<self.generation):return d
        return None

    def complete(self):
        self.applied=self.generation;self.held=False;self.cancel_requested=False;self.reason='CURRENT_IO_RECOVERED_NAV_KEPT'

    def report(self):
        return dict(generation=self.generation,applied_generation=self.applied,held=self.held,
            reason=self.reason,cancel_pending=self.cancel_requested,cancel_uuid=self.cancel_uuid,nav_restarted=False,initial_pose_requested=False)
