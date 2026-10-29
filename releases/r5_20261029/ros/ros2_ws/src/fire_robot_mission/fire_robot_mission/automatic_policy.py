"""Automatic stage requests on the same guarded session, never motor outputs.

The user-run automatic profile supplies authority once. No synthetic GUI keys,
new controllers, renewed I budgets or implicit replay of an uncertain P.
"""
import json
import math
import os
from pathlib import Path
import re
import stat
import uuid


class AutomaticStartGate:
    def __init__(self, path, session, packet):
        if str(uuid.UUID(session)) != session or re.fullmatch('[0-9a-f]{64}',packet) is None:
            raise ValueError('AUTOMATIC_START_BINDING_INVALID')
        self.path=Path(path);self.session=session;self.packet=packet;self.seen=False
        if not self.path.is_absolute():raise ValueError('AUTOMATIC_START_PATH_NOT_ABSOLUTE')

    def ready(self):
        try:
            # This is a local runner receipt in its private per-run folder.
            # It cannot enable a different run/packet or start before READY.
            fd=os.open(self.path,os.O_RDONLY|os.O_NOFOLLOW|os.O_NONBLOCK)
            with os.fdopen(fd,'r') as stream:
                meta=os.fstat(stream.fileno())
                if not stat.S_ISREG(meta.st_mode) or meta.st_uid!=os.getuid() or meta.st_mode&0o022:
                    return False
                raw=stream.read(8193)
            if len(raw)>8192:return False
            d=json.loads(raw)
            valid=(isinstance(d,dict) and type(d.get('schema_version')) is int
                and type(d.get('ready')) is bool and d==dict(schema_version=1,kind='R5_AUTOMATIC_START',
                session=self.session,packet=self.packet,stage='READY_GATE2',ready=True))
            self.seen |= valid
            return valid
        except (OSError,ValueError,TypeError):return False


class AutomaticPolicy:
    def __init__(self, enabled=False, *, alignment_timeout_s=0., target_recovery_s=3., ir_final_confirm_s=5., no_goal_handoff_timeout_s=8.):
        self.enabled=enabled is True;self.cycle=None;self.p_attempts=0;self.last_p_at=None
        self.last_intent=None;self.blocker='AUTOMATIC_MODE_DISABLED'
        self.cycle_started=None;self.handoff_at=None;self.target_lost_since=None
        self.no_goal_wait_started=None
        values=(target_recovery_s,ir_final_confirm_s,no_goal_handoff_timeout_s)
        if type(alignment_timeout_s) not in (int,float) or not math.isfinite(alignment_timeout_s) or alignment_timeout_s<0:
            raise ValueError('AUTOMATIC_TASK_TIMEOUT_INVALID')
        if any(type(v) not in (int,float) or not math.isfinite(v) or v<=0 for v in values):
            raise ValueError('AUTOMATIC_TASK_TIMEOUT_INVALID')
        self.stage_timeout_s=float(alignment_timeout_s)
        self.target_wait_s=float(target_recovery_s);self.final_confirm_s=float(ir_final_confirm_s)
        self.no_goal_handoff_timeout_s=float(no_goal_handoff_timeout_s)

    def finish_incomplete_stage(self, core, now):
        cycle=core.cycles.current
        if cycle is None or core.cancel_requested or core.p_used:return
        # A fire cycle starts while Nav2 still owns a real goal. Its cancel,
        # zero-motion and exact handoff may take longer than target recovery.
        # Start both task clocks only after the first proven handoff; retain
        # that epoch when alignment/IR subsequently owns raw motion.
        if self.handoff_at is None and core._handoff(now):
            self.handoff_at=now;self.cycle_started=now
        # With no Nav2 goal there is no cancel to wait for. A lost target or
        # missing STOP witness must not strand this cycle before the normal
        # post-handoff task clocks can start.
        if cycle.nav_goal_uuid is None and self.handoff_at is None:
            if self.no_goal_wait_started is None:self.no_goal_wait_started=now
            if core._fire(now):self.target_lost_since=None
            elif self.target_lost_since is None:self.target_lost_since=now
        if self.handoff_at is not None:
            if core._fire(now):self.target_lost_since=None
            elif self.target_lost_since is None:self.target_lost_since=now
        reason=None
        budget=core.cycles.budget
        stage='IR' if budget.requested_at is not None else 'ALIGN'
        if self.handoff_at is not None and self.target_lost_since is not None and now-self.target_lost_since>=self.target_wait_s:
            reason=stage+'_TARGET_NOT_RECOVERED'
        elif (cycle.nav_goal_uuid is None and self.handoff_at is None
                and self.target_lost_since is not None
                and now-self.target_lost_since>=self.target_wait_s):
            reason=stage+'_TARGET_NOT_RECOVERED_BEFORE_HANDOFF'
        elif (cycle.nav_goal_uuid is None and self.handoff_at is None
                and self.no_goal_wait_started is not None
                and now-self.no_goal_wait_started>=self.no_goal_handoff_timeout_s):
            reason='NO_GOAL_HANDOFF_TIMEOUT'
        elif not budget.unbounded and budget.requested_at is not None and now-budget.requested_at>=budget.TIME_S+self.final_confirm_s:
            reason='IR_NOT_COMPLETED_WITHIN_BUDGET_AND_CONFIRM'
        elif self.stage_timeout_s>0 and budget.requested_at is None and self.cycle_started is not None and now-self.cycle_started>=self.stage_timeout_s:
            reason='ALIGN_TASK_TIMEOUT'
        report=core.get('ir_live' if stage=='IR' else 'alignment',now) or {}
        if report.get('state')=='ABORTED':reason=stage+'_INTERRUPTED:'+str(report.get('stop_reason'))
        status=core._status(now)
        if status and status.active_uuid not in (None,cycle.nav_goal_uuid):reason='NEW_GOAL_REQUESTED'
        if reason:
            core.interrupt_cycle(reason)
            if 'TARGET_NOT_RECOVERED' in reason:
                # The completed loss window itself is the observed loss. A
                # reappearance during release must not need a second clear.
                core.retry_fire_clear_required=False


    def next_intent(self, core, now, *, start_ready, blockers=()):
        self.blocker=None
        if not self.enabled:self.blocker='AUTOMATIC_MODE_DISABLED';return None
        if not start_ready:self.blocker='WAIT_RUNNER_AUTOMATIC_START';return None
        current=core.cycles.current
        cycle=current.cycle_id if current else None
        if cycle!=self.cycle:
            self.cycle=cycle;self.p_attempts=0;self.last_p_at=None
            self.cycle_started=None;self.handoff_at=None;self.target_lost_since=None
            self.no_goal_wait_started=None
        if not core.closed and not core.fault:self.finish_incomplete_stage(core,now)
        if core.closed or core.fault or core.identity_hold:
            self.blocker=core.fault or 'SESSION_STOP_OR_IDENTITY_HOLD';return None
        if blockers:self.blocker=str(blockers[0]);return None
        if not core.armed:
            proof=core._standby_proof(now)
            if (core.arm_requested_at is None and not proof['blockers']
                    and proof['ir'].get('grant_fresh') is True and proof['stable_s']>=3.):return 'ARM'
            self.blocker='WAIT_EXISTING_ARM_PROOFS';return None
        if current is None:
            return 'FIRE_WATCH' if not core.fire_permission else None
        if core.cancel_requested or core.p_used:return None
        if core.phase=='WAIT_T':return 'ALIGN'
        if core.phase=='WAIT_IR_I' and core.cycles.budget.requested_at is None:
            ir=core.get('ir',now) or {}
            bands=('APPROACH','TOO_FAR','SPRAY_RANGE','TOO_CLOSE') if core.allow_close else ('APPROACH','TOO_FAR','SPRAY_RANGE')
            if ir.get('ir_band') in bands and core._aligned(now):return 'APPROACH'
            self.blocker='WAIT_EXISTING_IR_ADMISSION';return None
        if core.phase=='WAIT_P':
            self.blocker=core._pulse_admission(now)
            if self.blocker or not core._aligned(now) or not core._off(now):return None
            # At most three admission attempts, and only after the actor
            # explicitly proved the prior edge unused. One accepted pulse.
            if self.p_attempts:
                f=core.feedback
                if (self.p_attempts>=3 or f.get('accepted') is not False
                        or not str(f.get('reason','')).startswith('P_REJECTED_BY_ACTOR:')):
                    self.blocker='AUTOMATIC_P_WAIT_RECEIPT_NO_REPLAY';return None
                if now-self.last_p_at<.25:return None
            return 'PULSE'
        return None

    def issued(self, intent, now, feedback):
        self.last_intent=intent
        if intent=='PULSE' and feedback.get('accepted') is True:
            self.p_attempts+=1;self.last_p_at=now

    def report(self, start_ready):
        return dict(enabled=self.enabled,start_ready=start_ready,last_intent=self.last_intent,
            blocker=self.blocker,p_admission_attempts=self.p_attempts,
            max_pulses_per_cycle=1,synthetic_keyboard=False,physical_commands_published=False)
