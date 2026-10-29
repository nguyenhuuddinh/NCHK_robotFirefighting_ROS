"""Connect real session reports to exact peer retirement, without any I/O."""
from .cycle_release import CycleReleaseCoordinator,ReleaseContext,ReleaseInputs,ROLES
from .operational_identity import identity_current,identity_key
from .operational_session_core import obj


class OperationalHandback:
    def __init__(self,id_factory=None):
        self.factory=id_factory;self.coordinator=None;self.active=None
        self.pulse_home=None;self.consumed=False;self.fault=None
        self.cancel_history=ReleaseInputs()

    def context(self,session,now):
        cycle=session.cycles.current
        value,at=session.inputs.get('identity',(None,None))
        if cycle is None or not identity_current(value,at,now):return None
        key=identity_key(value)
        return ReleaseContext(cycle.cycle_id,cycle.nav_goal_uuid,*key).as_dict()

    def step(self,session,now,*,graph_ok,raw_count,pump_count):
        if session.fault or session.closed or not session.armed:
            return dict(self.report(['SESSION_NOT_ARMED_OR_FAULT']),control=None)
        context=self.context(session,now)
        if session.identity_hold:return self.report(['WAIT_FRESH_SAME_BOOT_IDENTITY'])
        if session.cycles.current is not None:
            if self.active is None:
                if context is None:return self.report(['WAIT_CURRENT_BOOT_IDENTITY'])
                self.active=context;self.consumed=False;self.coordinator=None;self.pulse_home=None
            elif context is not None and context!=self.active:
                if self.consumed:
                    self.active=context;self.consumed=False;self.coordinator=None;self.pulse_home=None
                else:self.fault='CYCLE_OR_BOOT_CHANGED_BEFORE_RELEASE'
        if self.fault:
            session.stop_fault(self.fault);return self.report([self.fault])
        if not self.active or self.consumed and session.cycles.current is None:return self.report([])
        for key,(value,at) in session.inputs.items():self.cancel_history.update(key,value,at)
        self.cancel_history.stop_errors(ReleaseContext.parse(self.active),now,graph_ok,raw_count,pump_count)
        actor=obj(session.get('actor',now))
        # Capture the real pulse's starting home counter, never the later value.
        if session.p_used and self.pulse_home is None:
            n=actor.get('start_home_count')
            if actor.get('request_used') is True and type(n) is int and n>=0:self.pulse_home=n
        clear_probe=CycleReleaseCoordinator()
        for key,(value,at) in session.inputs.items():clear_probe.update(key,value,at)
        true_clear=not clear_probe._clear(now)
        if not session.p_used and true_clear:session.cancel_requested=True
        pulse_terminal=(session.p_used and session._actor_cycle_current(actor)
            and actor.get('request_used') is True and actor.get('state') in ('DONE','ABORTED'))
        operator_cancel=(session.p_used and session.cancel_requested
            and session.interruption_reason=='OPERATOR_CANCEL_AFTER_PULSE')
        sequence=obj(actor.get('spray_sequence'))
        interrupted_sequence=(pulse_terminal and actor.get('state')=='ABORTED'
            and sequence.get('release_without_clear') is True
            and sequence.get('actuation_started') is True)
        unused_pulse=(pulse_terminal and actor.get('state')=='ABORTED'
            and sequence.get('actuation_started') is False
            and actor.get('pump_on_feedback_seen') is False
            and actor.get('servo_sweep_feedback_seen') is False
            and actor.get('pulse_on_since') is None and actor.get('proposed_pump_on') is False)
        if pulse_terminal and actor.get('state')=='ABORTED':
            # An interrupted pulse is not a successful spray. Retirement
            # still requires the real starting home count, fresh OFF/home,
            # STOP, exact cancel history and clear sensors at every peer.
            session.interruption_reason=session.interruption_reason or (
                'PULSE_INTERRUPTED:'+str(actor.get('reason')))
        if (pulse_terminal and self.coordinator is None
                and (sequence.get('require_new_fire_event') is True or interrupted_sequence
                     and not (sequence.get('recoverable_permission_gap') is True
                              and sequence.get('reason')=='TARGET_NOT_RECOVERED_RETURN_NAV2'))):
            session.retry_fire_clear_required=True
            session.recovery_require_ir_clear=True
            session.retry_clear_since=None
        closing=session.cancel_requested or pulse_terminal
        if closing and self.coordinator is None:
            if operator_cancel and not pulse_terminal:
                # False arms/ACTIVE cancel tag drain the actual actor first.
                # Choose the immutable home policy AFTER it proves whether
                # this accepted edge ever entered ON, not from a P key alone.
                return self.report(['WAIT_ACTOR_TERMINAL_BEFORE_RELEASE_POLICY'])
            for role in ROLES:
                ack=obj(session.get(role,now)).get('cycle_release_ack')
                if (not isinstance(ack,dict) or ack.get('protocol')!='CYCLE_RELEASE_V1'
                        or ack.get('role')!=role or ack.get('phase')!='ACTIVE'
                        or ack.get('context')!=self.active or ack.get('blockers')!=[]):
                    return self.report(['WAIT_'+role.upper()+'_ACTIVE_REGISTRATION'])
            if session.p_used and self.pulse_home is None:return self.report(['WAIT_REAL_PULSE_HOME_BASELINE'])
            self.coordinator=CycleReleaseCoordinator(self.factory)
            self.coordinator.cancel_seen=set(self.cancel_history.cancel_seen)
            self.coordinator.terminal_seen=set(self.cancel_history.terminal_seen)
            actual_pulse=session.p_used and not unused_pulse
            completed_sequence=actual_pulse and actor.get('state')=='DONE' and sequence.get('release_without_clear') is True
            self.coordinator.begin(self.active,now,after_pulse=actual_pulse,
                completed_sequence_release=completed_sequence,
                home_before=self.pulse_home if actual_pulse else None,
                require_clear=actual_pulse and not operator_cancel and not completed_sequence and not interrupted_sequence,
                pulse_interrupted=actual_pulse and (operator_cancel or bool(pulse_terminal and actor.get('state')=='ABORTED')),
                operator_cancel=operator_cancel and actual_pulse,
                terminal_goal_release=not session.p_used and session.interruption_reason=='NAV_GOAL_TERMINAL_BEFORE_FIRE_HANDOFF')
        if self.coordinator is not None and not self.consumed:
            if operator_cancel:
                # X can arrive after an interrupted actor already started
                # PREPARE/COMMIT. Keep its context/token/STOP proofs and drop
                # only the clear wait, just as X before PREPARE already does.
                self.coordinator.cancel_interrupted_release()
            for key,(value,at) in session.inputs.items():
                self.coordinator.update(key,value,at)
            for role in ROLES:
                source='alignment' if role=='alignment' else role
                value,at=session.inputs.get(source,(None,None))
                if at is not None:self.coordinator.update(role,value,at)
            self.coordinator.step(now,graph_ok=graph_ok,raw_count=raw_count,pump_count=pump_count)
            if self.coordinator.receipt is not None:
                try:
                    session.accept_handback(self.coordinator.receipt,now,expected_context=self.active)
                    self.consumed=True
                except ValueError as exc:
                    self.fault=str(exc);session.stop_fault(self.fault)
        return self.report([])

    def report(self,blockers):
        control=self.coordinator.control() if self.coordinator else (
            dict(protocol='CYCLE_RELEASE_V1',phase='ACTIVE',context=self.active) if self.active else None)
        return dict(scope='R5_HANDBACK_CONTROL_PROPOSAL',control=control,
            release=self.coordinator.report() if self.coordinator else None,
            consumed=self.consumed,blockers=blockers,fault=self.fault,
            live_control_published=False)
