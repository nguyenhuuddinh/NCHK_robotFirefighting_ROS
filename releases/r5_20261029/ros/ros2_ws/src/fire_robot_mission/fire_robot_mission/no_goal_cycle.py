"""Independent no-active-goal witness. Never creates a goal UUID or cancel result.

The same R4 motion/scan/IR/actuator gates remain in the consumers. This module
only replaces the navigation prerequisite when a registered fire cycle has no
Nav2 goal. Equality of two missing goal UUIDs is never an identity check.
"""
import copy
from .cycle_release import ReleaseContext
from .operational_cycle import NavigationSnapshot,finite


def context_of(core):
    value=getattr(core,'r5_cycle_context',None)
    try:
        ctx=ReleaseContext.parse(value)
        return ctx.as_dict() if ctx.nav_goal_uuid is None else None
    except (ValueError,TypeError):return None


def same_cycle(payload,context):
    return (context is not None and isinstance(payload,dict)
            and payload.get('r5_cycle_context')==context
            and payload.get('goal_uuid') is None)


def cycle_bound(core):
    ctx=context_of(core)
    return bool(core.goal_uuid or ctx and getattr(core,'cycle_id',None)==ctx['cycle_id'])


def no_active(rows,received,now):
    try:
        snap=NavigationSnapshot.parse(rows,received)
        return received<=now and snap.active_uuid is None
    except (ValueError,TypeError):return False


def registered_navigation(core,rows,received,now,goal_uuid):
    """Current status for a registered cycle; never a cancel proof itself.

    Fresh exact guard/gateway proof remains a separate consumer requirement.
    A terminal UUID may be pruned; an active or contradictory row cannot
    replace it. Unregistered legacy consumers retain their original policy.
    """
    try:
        ctx=ReleaseContext.parse(getattr(core,'r5_cycle_context',None))
        status=NavigationSnapshot.parse(rows,received)
        return (ctx.nav_goal_uuid==goal_uuid and received<=now and status.active_uuid is None
            and (goal_uuid is None or not any(u==goal_uuid and s!=5 for u,s in status.rows)))
    except (ValueError,TypeError):return False


def stopped_pair(gateway,guard,context,*,require_stop=True):
    for role,value in [('gateway',gateway),('guard',guard)]:
        proof=value.get('no_goal_proof') if isinstance(value,dict) else None
        if (not isinstance(proof,dict) or proof.get('protocol')!='NO_ACTIVE_GOAL_V1'
                or proof.get('role')!=role or proof.get('context')!=context
                or proof.get('valid') is not True or proof.get('stop_witness_seen') is not True
                or proof.get('no_active_goal') is not True or proof.get('blockers')!=[]
                or require_stop and proof.get('stationary_released') is not True):return False
        if not same_cycle(value,context):return False
    return (guard.get('send_count')==0 and guard.get('cancel_service_accepted') is False
            and guard.get('cancel_status_seen') is False and guard.get('goal_status') is None)


class NoGoalWitness:
    def __init__(self,role):
        self.role=role;self.context=None;self.since=None;self.last_at=None
        self.seen=False;self.fault=None

    def step(self,context,now,*,rows,received,graph_ok,cmd_fresh,odom_fresh,rest,raw_count):
        if self.context is None:self.context=copy.deepcopy(context)
        elif context!=self.context:self.fault='NO_GOAL_CONTEXT_CHANGED_WITHOUT_RESET'
        if not finite(now) or self.last_at is not None and now<self.last_at:self.fault='NO_GOAL_CLOCK_DISCONTINUITY'
        if self.last_at is not None and finite(now) and now-self.last_at>.35:self.since=None
        self.last_at=now
        clear=no_active(rows,received,now)
        if rows is not None and not clear:self.fault=self.fault or 'NO_GOAL_STATUS_CHANGED_OR_INVALID'
        errors=[]
        if self.fault:errors.append(self.fault)
        if not graph_ok:errors.append('WAIT_NO_GOAL_EXACT_SOURCES')
        if not clear:errors.append('WAIT_NO_ACTIVE_GOAL')
        if not cmd_fresh or not odom_fresh:errors.append('WAIT_NO_GOAL_FRESH_MOTION_TELEMETRY')
        stationary=not errors and rest and raw_count==0
        if not stationary:self.since=None
        elif self.since is None:self.since=now
        if stationary and now-self.since>=.5:self.seen=True
        if not self.seen:errors.append('WAIT_NO_GOAL_STOP_HOLD')
        return dict(protocol='NO_ACTIVE_GOAL_V1',role=self.role,context=copy.deepcopy(context),
            valid=not errors,no_active_goal=clear,stop_witness_seen=self.seen,
            stationary_released=stationary and self.since is not None and now-self.since>=.5,blockers=errors)
