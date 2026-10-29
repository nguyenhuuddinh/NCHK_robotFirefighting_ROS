"""Owned process cleanup ordering, awaiting the operator's actual STOP receipt.

No process is started here. The paired launcher must register only processes it
started with OwnedProcess, and supply a reader for that operator's fresh log.
Pi is stopped last; a missing/failed proof is recorded, never turned into PASS.
"""
import math
from operational_owned_process import OwnedProcess
from operational_logs import OwnedLogRegistry


class StackCleanup:
    GROUPS=(('operator',),('actor','alignment','ir_live','mission','guard','gateway','review'),
            ('ir_monitor','yolo','nav','rviz','startup_health'),('pi',))
    ROLES={role for group in GROUPS for role in group}
    def __init__(self,stop_receipt,*,child_logs=None):
        self.read_stop_receipt=stop_receipt;self.processes={};self.started=None;self.last=None
        self.stage=0;self.done=False;self.cause=None;self.stop_confirmed=False;self.errors=[]
        self.receipt_checked=False;self.child_logs=child_logs;self.child_audit=None
        if child_logs is not None and not isinstance(child_logs,OwnedLogRegistry):raise ValueError('OWNED_LOG_REGISTRY_REQUIRED')

    @classmethod
    def from_logs(cls,logs):
        if not isinstance(logs,OwnedLogRegistry) or 'operator' not in logs.logs:raise ValueError('OWNED_OPERATOR_LOG_REQUIRED')
        return cls(logs.logs['operator'].receipt,child_logs=logs)

    def register(self,role,process):
        if self.started is not None:raise ValueError('REGISTER_AFTER_STOP_FORBIDDEN')
        if role not in self.ROLES or role in self.processes:raise ValueError('UNKNOWN_OR_DUPLICATE_ROLE')
        if not isinstance(process,OwnedProcess) or process.process is None:raise ValueError('STARTED_OWNED_PROCESS_REQUIRED')
        if any(p.pid==process.pid for p in self.processes.values()):raise ValueError('PID_REGISTERED_TWICE')
        if self.child_logs is not None and not self.child_logs.matches(role,process):raise ValueError('OWNED_LOG_PROCESS_MISMATCH')
        self.processes[role]=process

    def stop(self,now,cause='Q'):
        if self.started is None:self.started=now;self.last=now;self.cause=cause
        return self.tick(now)

    def _operator_receipt(self):
        self.receipt_checked=True
        p=self.processes.get('operator')
        try:
            r=self.read_stop_receipt() if p is not None and p.done and p.rc==0 and not p.forced and p.error is None else None
            self.stop_confirmed=(isinstance(r,dict) and r.get('scope')=='R5_STOP_TELEMETRY_VERIFICATION'
                and r.get('requested') is True and r.get('done') is True and r.get('confirmed') is True
                and r.get('reason')=='FRESH_ZERO_OFF_HOME_RELEASE_CONFIRMED' and not r.get('blockers')
                and r.get('fault') is None and r.get('motion_authorized') is False and r.get('pulse_replayed') is False)
        except (OSError,ValueError,TypeError):self.stop_confirmed=False
        if not self.stop_confirmed:self.errors.append('OPERATOR_STOP_UNCONFIRMED')

    def tick(self,now):
        if self.done or self.started is None:return self.report()
        if type(now) not in (int,float) or not math.isfinite(now) or now<self.last:
            self.errors.append('STACK_CLOCK_INVALID');self.done=True;return self.report()
        self.last=now
        # Subsequent stop calls do not reset individual grace/deadline budgets.
        group=self.GROUPS[self.stage];active=[(r,self.processes[r]) for r in group if r in self.processes]
        for role,p in active:p.stop(now)
        if not all(p.done for _,p in active):return self.report()
        if self.stage==0:self._operator_receipt()
        for role,p in active:
            if p.error is not None or p.forced or p.rc!=0:self.errors.append('PROCESS_UNCLEAN:'+role)
        self.stage+=1
        if self.stage==len(self.GROUPS):
            self.done=True
            if self.child_logs is not None:
                try:self.child_audit=self.child_logs.audit(self.processes)
                except (OSError,ValueError,TypeError) as exc:
                    self.child_audit=dict(complete=False,clean=False,error=str(exc))
                if self.child_audit.get('clean') is not True:self.errors.append('CHILD_LOG_AUDIT_FAILED')
        return self.report()

    def report(self):
        return dict(requested=self.started is not None,done=self.done,cause=self.cause,
            stage='DONE' if self.done else '+'.join(self.GROUPS[self.stage]),
            stop_confirmed=self.stop_confirmed,errors=list(self.errors),
            group_cleanup_clean=self.done and self.stop_confirmed and not self.errors,
            clean=self.done and self.stop_confirmed and not self.errors and
                isinstance(self.child_audit,dict) and self.child_audit.get('complete') is True and self.child_audit.get('clean') is True,
            child_log_audit=self.child_audit,
            processes={role:p.report() for role,p in self.processes.items()},
            ROS_child_acceptance=('PENDING_CHILD_LOG_AUDIT' if self.child_audit is None else
                'PASS' if self.child_audit.get('clean') is True else 'FAIL'),hardware_scope='NO_ACTUATION_FROM_CLEANUP')
