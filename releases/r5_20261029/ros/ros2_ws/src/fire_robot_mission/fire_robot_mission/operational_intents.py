"""R5 intent mapping only: no Twist, pump command or action client.

The existing live consumers retain their own sensor/ownership gates. A cycle
control proposal is broadcast before release and never replaced by a fake
CancelGoal response. No-goal stages require the independent STOP witness and
the same registered cycle context at all six consumers.
"""
import uuid
import copy
from .operational_cycle import valid_uuid
from .operational_identity import identity_current
from .operational_session_core import obj
from .cycle_release import ROLES

ARM_SUFFIXES=('live_nav_arm','nav_cancel_arm','alignment_arm_shadow',
              'live_alignment_arm','ir_approach_arm','pump_demo_arm')
ARM_TOPICS=tuple('/fire_mission/'+n for n in ARM_SUFFIXES)


class IntentMapper:
    def __init__(self,id_factory=None):
        self.factory=id_factory or (lambda:uuid.uuid4().hex)
        self.stationary_id=None;self.cycle=None

    @staticmethod
    def readiness(session,now,*,safety_ok):
        errors=[];graph=obj(session.get('graph',now))
        if graph.get('valid') is not True:errors.append('WAIT_EXACT_OPERATIONAL_GRAPH')
        if safety_ok is not True:errors.append('WAIT_SAFETY_PARAMETERS')
        identity,at=session.inputs.get('identity',(None,None))
        if not identity_current(identity,at,now) or session.identity_hold:errors.append('WAIT_CURRENT_BOOT_IDENTITY')
        if session.graph_hold:errors.append('WAIT_NAV_RAW_OWNER_NAME')
        for role in ROLES:
            value=obj(session.get(role,now))
            if value.get('cycle_protocol_enabled') is not True:errors.append('WAIT_'+role.upper()+'_CYCLE_PROTOCOL')
        return errors

    def step(self,session,report,handback,now,*,safety_ok):
        blockers=self.readiness(session,now,safety_ok=safety_ok)
        active=session.armed and not session.closed and session.fault is None and not blockers and not session.input_hold
        cycle=session.cycles.current
        cid=cycle.cycle_id if cycle else None
        if cid!=self.cycle:self.cycle=cid;self.stationary_id=None
        release=handback.get('release') or {}
        closing=bool(release and not handback.get('consumed')) or session.cancel_requested
        # No-goal uses a separate context contract; never send a fake UUID.
        real_goal=cycle is not None and valid_uuid(cycle.nav_goal_uuid)
        control=copy.deepcopy(handback.get('control')) if active else None
        if active and cycle is not None and control is None:
            blockers.append('WAIT_CYCLE_CONTROL_CONTEXT');active=False
        if (active and cycle is not None and session.cancel_requested and session.p_used
                and type(session.p_request_baseline) is int and control.get('phase')=='ACTIVE'):
            control['cancel_unused_pulse']=dict(baseline_index=session.p_request_baseline)
        if active and cycle is not None:
            for role in ROLES:
                ack=obj(session.get(role,now)).get('cycle_release_ack')
                if (not isinstance(ack,dict) or ack.get('context')!=control.get('context')
                        or ack.get('role')!=role or ack.get('protocol')!='CYCLE_RELEASE_V1'
                        or control.get('phase')=='ACTIVE' and (ack.get('phase')!='ACTIVE' or ack.get('blockers')!=[])):
                    # Registration is separate from intent arming. Continue to
                    # publish ACTIVE while every stage remains false.
                    blockers.append('WAIT_'+role.upper()+'_CYCLE_BINDING')
        bound=not any(b.endswith('_CYCLE_BINDING') for b in blockers)
        intents=report.get('intents') or {}
        no_goal=(cycle is not None and cycle.nav_goal_uuid is None
            and all(obj(session.get(role,now)).get('no_goal_supported') is True for role in ROLES))
        if cycle is not None and cycle.nav_goal_uuid is None and not no_goal:
            blockers.append('WAIT_NO_GOAL_CONSUMERS')
        stages=active and bound and (real_goal or no_goal) and not closing
        live=obj(session.get('ir_live',now))
        budget=report.get('budget') or {}
        soft_reasons=('', 'REALIGN_REQUIRED','OPERATOR_ARM_MISSING_OR_STALE',
            'EXACT_HANDOFF_PROOF_INVALID','IR_BAND_INVALID_OR_STALE','IR_BAND_HOLD','IR_DIRECTION_CHANGED')
        if (stages and control.get('phase')=='ACTIVE'
                and report.get('phase') in ('IR_RELEASE','IR_CAP_HOLD')
                and live.get('state') in ('ACQUIRING','MOVING','STOPPING')
                and live.get('stop_reason') in soft_reasons
                and type(live.get('step_count')) is int and live['step_count']>=0
                and budget.get('requested_at') is not None):
            # A false arm remains an immediate STOP. This bound request only
            # distinguishes a planned perception/budget hold from a lost arm.
            control['ir_stop_request']=dict(schema_version=1,step_count=live['step_count'],
                budget_started_at=budget['requested_at'],
                cause='I_BUDGET_EXHAUSTED' if budget.get('exhausted') else 'PERCEPTION_HOLD')
        turn=stages and intents.get('alignment') is True
        arms=dict(zip(ARM_TOPICS,(active and not closing and (cycle is None or real_goal) and session.navigation_cancel_uuid is None,
            active and (real_goal or no_goal),turn and real_goal,turn,
            stages and intents.get('ir') is True,
            stages and intents.get('pulse') is True)))
        stationary=stages and session.pending_stationary and (session.p_used or session.phase in ('IR_CONFIRM','WAIT_P'))
        if stationary and self.stationary_id is None:
            self.stationary_id=self.factory()
            if not valid_uuid(self.stationary_id):raise ValueError('STATIONARY_TOKEN_INVALID')
        request=dict(schema_version=1,active=bool(stationary),request_id=self.stationary_id,
                     goal_uuid=cycle.nav_goal_uuid if stationary else None)
        if stationary and no_goal:request['cycle_context']=control['context']
        if stationary and session.ir_recovering and session.last_ir_step>0:
            request['cycle_context']=control['context']
            request['ir_reentry']=dict(step_count=session.last_ir_step,
                total_progress_m=live.get('total_progress_m'),budget_started_at=budget.get('requested_at'))
        if stationary and session.allow_close:
            request['range_policy']='CLOSE_STATIONARY_TEST_V1'
        return dict(arms=arms,stationary_request=request,cycle_control=control,
            blockers=blockers,no_goal_motion_supported=no_goal,
            physical_commands_published=False)
