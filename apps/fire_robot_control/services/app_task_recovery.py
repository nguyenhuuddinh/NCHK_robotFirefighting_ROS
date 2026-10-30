"""App-only cancellation semantics; the sealed R5 files stay byte-identical.

No ROS context or actuator output is created here. The existing cancel guard
still sends exact UUID actions and every motion/STOP admission stays gated.
"""
import uuid
import time

from fire_robot_mission.operational_session_core import OperationalSessionCore, obj
from fire_robot_mission.operational_handback import OperationalHandback
from fire_robot_mission.operational_cycle import NavigationSnapshot, finite, valid_uuid, zero
from fire_robot_mission.operational_identity import identity_current, identity_key
from fire_robot_mission.cycle_endpoint import CycleEndpoint
from fire_robot_mission.live_nav_stop_gateway import LiveNavStopCore
from fire_robot_mission.cycle_blocked_goal import BlockedGoalCancel
from fire_robot_mission.nav2_cancel_guard import make_cancel_request
from fire_robot_mission.actuator_freshness import fresh_off, fresh_motion_off
from .app_nav_recovery import AppNavRecovery


class AppSessionCore(OperationalSessionCore):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.app_x_latched = False
        self.app_clear_since = None
        self.app_clear_seen = None
        self.app_cancel_id = None
        self.app_cancel_targets = set()
        self.app_navigation_session = uuid.uuid4().hex
        self.app_navigation_epoch = 0

    def _key(self, key, now):
        if key == 'X' and self.armed and not self.closed and self.fault is None:
            self.app_navigation_epoch += 1
            self.app_x_latched = True
            self.app_clear_since = None
            self.app_clear_seen = None
            self.fire_permission = False
            self.retry_fire_clear_required = True
            self.retry_clear_since = None
            if self.app_cancel_id is None:
                self.app_cancel_id = uuid.uuid4().hex
            # Retain only the UUID intent when its report is temporarily late.
            # Cached telemetry never grants STOP/rearm; the endpoint still
            # requires a CURRENT session, identity, topology and fresh STOP.
            gateway = obj(self.get('gateway', now)) or obj(self.inputs.get('gateway', ({}, None))[0])
            for goal in (gateway.get('cycle_goal_uuid'), gateway.get('goal_uuid')):
                if valid_uuid(goal):
                    self.app_cancel_targets.add(goal)
            status = self._status(now)
            if status and status.active_uuid:
                self.app_cancel_targets.add(status.active_uuid)
        super()._key(key, now)

    def apply_intent(self, intent, now, *, origin):
        if intent == 'FIRE_WATCH' and self.app_x_latched:
            # An automatic F cannot undo a user's X at the next tick.
            return self.report(now)
        return super().apply_intent(intent, now, origin=origin)

    def _advance(self, now):
        if self.app_x_latched:
            self.fire_permission = False
            fire = obj(obj(self.get('vision', now)).get('fire'))
            clear = (fire.get('detected_now') is False and fire.get('confirmed') is False
                     and not self.retry_fire_clear_required and self.cycles.current is None)
            if not clear:
                self.app_clear_since = self.app_clear_seen = None
            elif (self.app_clear_since is None or not finite(self.app_clear_seen)
                    or not 0 <= now-self.app_clear_seen <= .35):
                self.app_clear_since = now
            elif now - self.app_clear_since >= .5:
                self.app_x_latched = False
            if clear:
                self.app_clear_seen = now
        super()._advance(now)

    def report(self, now):
        report = super().report(now)
        report['app_navigation_lease'] = dict(session=self.app_navigation_session,
                                             cancel_epoch=self.app_navigation_epoch)
        cycle = self.cycles.current
        if self.cancel_requested and cycle is not None and not self.p_used:
            if self.app_cancel_id is None:
                self.app_cancel_id = uuid.uuid4().hex
            report['app_retirement_cancel'] = dict(schema_version=1,
                request_id=self.app_cancel_id, cycle_id=cycle.cycle_id,
                goal_uuid=cycle.nav_goal_uuid, before_pulse=True)
        else:
            report['app_retirement_cancel'] = None
        report['app_navigation_reset'] = (dict(schema_version=1,
            request_id=self.app_cancel_id, goal_uuids=sorted(self.app_cancel_targets))
            if self.app_cancel_id and cycle is None else None)
        gateway = obj(self.get('gateway', now))
        recovery = obj(gateway.get('app_navigation_recovery'))
        state = gateway.get('state')
        waiting = bool(self.navigation_cancel_uuid or cycle is not None)
        eligible = (not waiting and self.armed and not self.closed and not self.fault
                    and not (self.input_hold or self.identity_hold or self.graph_hold)
                    and recovery.get('state') not in ('WAIT_COMMAND', 'WAIT_STOP', 'VERIFYING', 'REACQUIRING', 'BLOCKED')
                    and (state in ('NAVIGATION', 'WAITING_FOR_INPUT', 'ARMING')
                         and not gateway.get('stop_reason')
                         or gateway.get('normal_rearm_ready') is True
                         or self.handback_completed and state == 'WAITING_FOR_INPUT'))
        report['app_goal_admission'] = dict(allowed=bool(eligible),
            reason='READY' if eligible else 'WAIT_TASK_STOP_OR_GATEWAY_READY',
            cancel_pending=bool(self.navigation_cancel_uuid or self.cancel_requested),
            fire_suppressed=self.app_x_latched)
        if report.get('phase') == 'NAVIGATION' and not waiting:
            if recovery.get('state') in ('WAIT_COMMAND', 'WAIT_STOP', 'VERIFYING', 'REACQUIRING', 'BLOCKED'):
                report['app_navigation_recovery'] = recovery
            elif gateway.get('released') is True or gateway.get('stop_reason'):
                report['app_navigation_recovery'] = dict(state='BLOCKED', reason=gateway.get('stop_reason'))
        return report


class AppHandback(OperationalHandback):
    def step(self, session, now, **kwargs):
        cycle = session.cycles.current
        if (self.coordinator is None and session.cancel_requested and not session.p_used
                and cycle is not None and cycle.nav_goal_uuid is not None):
            status = session._status(now)
            terminal = [] if status is None else [s for u, s in status.rows if u == cycle.nav_goal_uuid]
            if terminal in ([4], [5], [6]):
                # Existing typed release supports a REAL terminal before ON.
                # Never invent a CANCELED result for SUCCEEDED/ABORTED.
                session.interruption_reason = 'NAV_GOAL_TERMINAL_BEFORE_FIRE_HANDOFF'
            else:
                # Keep ACTIVE registration until exact cancel can finish.
                # PREPARE cannot demand a result whose request it prevents.
                pending = session.cancel_requested
                try:
                    session.cancel_requested = False
                    return super().step(session, now, **kwargs)
                finally:
                    session.cancel_requested = pending
        return super().step(session, now, **kwargs)


class AppNavCore(AppNavRecovery, LiveNavStopCore):
    def _normal_terminal_result(self, now):
        if self.status_at is None or now < self.status_at:
            return None
        result = super()._normal_terminal_result(now)
        if result is not None:
            return result
        # Only source/identity-validated exact terminal facts are retained by
        # this instance's endpoint. A contradictory current row never falls back.
        if any(uid == self.goal_uuid for uid, _ in self.statuses or ()):
            return None
        return getattr(self, 'app_terminal_history', {}).get(self.goal_uuid)

    def _normal_cycle_rearm(self, now, blockers, raw_ok, nav_ok, safety_ok,
                            stationary, cmd_zero, raw_count, status_count,
                            mission_count, odom_count, arm_count):
        args = (now, blockers, raw_ok, nav_ok, safety_ok, stationary, cmd_zero,
                raw_count, status_count, mission_count, odom_count, arm_count)
        if self._app_resume_same_goal(*args):
            return True
        session = getattr(self, 'app_operator_session', {})
        if (not getattr(self, 'app_session_current', False) or session.get('cycle') is not None
                or session.get('navigation_cancel_uuid') is not None):
            self.app_idle_since = self.app_idle_seen = None
            self.normal_ready_at = None
            return False
        if super()._normal_cycle_rearm(*args):
            self.app_idle_since = self.app_idle_seen = None
            return True
        new_goal = self._active_goal(now)
        perception = obj(obj(self.mission).get('perception'))
        no_fire = (perception.get('detected_now') is False and perception.get('confirmed') is False)
        suppressed = obj(session.get('app_goal_admission')).get('fire_suppressed') is True
        allowed_blockers = {'NAVIGATION_NOT_OWNED', 'NAV_GOAL_UUID_UNAVAILABLE', 'NAV_COMMAND_MISSING_OR_INVALID'}
        valid = (getattr(self, 'app_session_current', False) and session.get('cycle') is None
            and session.get('navigation_cancel_uuid') is None and not self.fire_latched
            and self.normal_terminal in (4, 5, 6)
            and self._normal_terminal_result(now) == self.normal_terminal
            and self.stop_reason in ('NAV_GOAL_SUCCEEDED', 'NAV_GOAL_CANCELED', 'NAV_GOAL_ABORTED')
            and raw_ok and nav_ok and safety_ok and raw_count == 0
            and status_count == mission_count == odom_count == arm_count == 1
            and set(blockers) <= allowed_blockers and stationary and cmd_zero
            and self._actuator_off(now) and self.arm and self.arm_at is not None
            and 0 <= now - self.arm_at <= self.arm_timeout
            and (no_fire or suppressed)
            and (new_goal is None or new_goal != self.goal_uuid and new_goal not in self.used_goal_uuids))
        if not valid:
            self.app_idle_since = self.app_idle_seen = None
            return False
        seen = getattr(self, 'app_idle_seen', None)
        if (getattr(self, 'app_idle_since', None) is None or not finite(seen)
                or not 0 <= now-seen <= self.mission_timeout):
            self.app_idle_since = now
        self.app_idle_seen = now
        if now - self.app_idle_since < 1.:
            return False
        self.normal_ready_at = now
        if new_goal is not None and not blockers:
            self.app_idle_since = self.app_idle_seen = None
            return self._start_next_goal(new_goal)
        return False


class AppBlockedGoalCancel(BlockedGoalCancel):
    """Drain a foreign Goal after a real pre-pulse terminal or exact cancel.

Only the app's existing typed terminal release lane differs from R5. It does
not turn SUCCEEDED/ABORTED into CANCELED or authorize motion/peer reset.
"""
    def step(self, endpoint, now):
        p, node, ctx = endpoint.participant, endpoint.node, endpoint.participant.context
        if ctx is None:
            return None
        if self.context != ctx:
            self.context, self.requests, self.last = ctx, {}, None
        try:
            rows, at = p.inputs.get('status', (None, None))
            status = NavigationSnapshot.parse(rows, at)
            if at > now:
                raise ValueError('FUTURE_STATUS')
        except (ValueError, TypeError):
            return dict(state='WAIT_VALID_NAV_STATUS')
        uid = status.active_uuid
        if uid == ctx.nav_goal_uuid:
            uid = None
        if uid is None and not any(not value.get('terminal') for value in self.requests.values()):
            return None
        original = [value for goal, value in status.rows if goal == ctx.nav_goal_uuid]
        request = p.request or {}
        typed_terminal = (endpoint._session(now) is not None and request.get('phase') in ('PREPARE', 'COMMIT')
            and request.get('terminal_goal_release') is True and request.get('after_pulse') is False
            and finite(p.request_at) and 0 <= now - p.request_at <= .35
            and original in ([4], [5], [6]))
        if ctx.nav_goal_uuid is not None and original not in ([], [5]) and not typed_terminal:
            return dict(state='WAIT_ORIGINAL_CANCEL_STATUS')
        errors = p.stop_errors(ctx, now, endpoint.graph_ok,
            len(node.get_publishers_info_by_topic('/cmd_vel_raw')),
            len(node.get_publishers_info_by_topic('/pump_cmd')))
        independent = set(errors) - {'WAIT_NO_ACTIVE_GOAL', 'WAIT_REAL_GOAL_CANCELED'}
        history = (ctx.nav_goal_uuid is None or typed_terminal
                   or (ctx.cycle_id, ctx.nav_goal_uuid, ctx.identity) in p.cancel_seen)
        if independent or not history:
            return dict(state='WAIT_BLOCKED_GOAL_STOP', blockers=sorted(independent), original_cancel_seen=history)
        if uid is not None and uid not in self.requests:
            if not node.cancel_client.service_is_ready():
                return dict(state='WAIT_CANCEL_SERVICE', goal_uuid=uid)
            record = dict(future=None, since=now, accepted=False, terminal=None)
            self.requests[uid] = record
            try:
                record['future'] = node.cancel_client.call_async(make_cancel_request(uid))
            except Exception as exc:
                node.get_logger().error('App retirement cancel send failed: '+type(exc).__name__)
                return dict(state='BLOCKED_GOAL_CANCEL_SEND_FAILED', goal_uuid=uid)
        for target, record in self.requests.items():
            if record.get('terminal'):
                continue
            future = record['future']
            if future is None:
                return dict(state='BLOCKED_GOAL_CANCEL_SEND_FAILED', goal_uuid=target)
            if future.done() and not record['accepted']:
                try:
                    response = future.result()
                    ids = [bytes(goal.goal_id.uuid).hex() for goal in response.goals_canceling]
                    record['accepted'] = response.return_code == 0 and ids == [target]
                except Exception as exc:
                    node.get_logger().error('App retirement cancel response failed: '+type(exc).__name__)
                    record['accepted'] = False
            actual = [value for goal, value in status.rows if goal == target]
            if actual in ([4], [5], [6]):
                record['terminal'] = actual[0]
                self.last = dict(state='BLOCKED_GOAL_TERMINAL', goal_uuid=target,
                    cancel_accepted=record['accepted'], actual_terminal=actual[0])
            elif now - record['since'] > 3.:
                return dict(state='BLOCKED_GOAL_CANCEL_UNCONFIRMED', goal_uuid=target)
            else:
                return dict(state='WAIT_BLOCKED_GOAL_TERMINAL', goal_uuid=target, cancel_accepted=record['accepted'])
        if status.active_uuid is not None:
            return dict(state='WAIT_NO_ACTIVE_GOAL')
        if getattr(node.core, 'failure', None) == 'POST_CANCEL_SAFETY_LOST':
            node.core.failure = None
        return None


class AppCycleEndpoint(CycleEndpoint):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.app_history = {}
        if self.role == 'guard':
            self.blocked_goals = AppBlockedGoalCancel()
        self._bind_guard_preflight()

    def _bind_guard_preflight(self):
        if self.role != 'guard':
            return
        core = getattr(self.node, self.core_attr)
        original = core._preflight
        def preflight(now):
            blockers, candidate = original(now)
            goal = self._retirement_goal(now)
            if goal is None:
                return blockers, candidate
            # Replace ONLY the fire-detection admission with an authenticated
            # retirement intent. Keep arm/topology/service/odom/cmd guards.
            semantic = {'CANCEL_INTENT_MISSING_OR_STALE', 'CANCEL_INTENT_MALFORMED',
                        'CANCEL_INTENT_CONTRACT_INVALID', 'CANCEL_INTENT_GOAL_NOT_UNIQUE',
                        'CANCEL_INTENT_GOAL_MISMATCH', 'NAV_GOAL_NOT_CANCELABLE'}
            return [b for b in blockers if b not in semantic], goal
        core._preflight = preflight

    def _session(self, now, *, allow_input_pause=False):
        p = self.participant
        session = p.get('operator_session', now) or {}
        identity, received = p.inputs.get('identity', (None, None))
        good = (self.graph_ok and p.error is None and self._source('/fire_mission/operator_state')
            and self._source('/navigate_to_pose/_action/status')
            and identity_current(identity, received, now) and not self.continuity.held
            and self.continuity.bound == identity_key(identity)
            and type(session.get('schema_version')) is int and session['schema_version'] == 1
            and session.get('scope') == 'R5_SESSION_INTENT_ONLY'
            and session.get('armed') is True and session.get('closed') is False
            and session.get('graph_hold') is not True
            and (allow_input_pause or not any(session.get(k) for k in ('input_hold', 'identity_hold')))
            and session.get('fault') is None and session.get('armed_identity') == list(self.continuity.bound or ()))
        return session if good else None

    def _retirement_goal(self, now):
        p = self.participant
        session = self._session(now)
        request = obj(obj(session).get('app_retirement_cancel'))
        if (p.context is None or p.error is not None or type(request.get('schema_version')) is not int
                or request['schema_version'] != 1
                or not valid_uuid(request.get('request_id')) or request.get('before_pulse') is not True
                or request.get('cycle_id') != p.context.cycle_id
                or request.get('goal_uuid') != p.context.nav_goal_uuid
                or not valid_uuid(request.get('goal_uuid'))
                or obj(obj(session).get('cycle')).get('cycle_id') != p.context.cycle_id
                or not finite(p.request_at) or not 0 <= now - p.request_at <= .35):
            return None
        try:
            rows, received = p.inputs.get('status', (None, None))
            status = NavigationSnapshot.parse(rows, received)
        except (ValueError, TypeError):
            return None
        if received > now or status.active_uuid != request['goal_uuid']:
            return None
        if [s for u, s in status.rows if u == request['goal_uuid']] not in ([1], [2]):
            return None
        errors = p.stop_errors(p.context, now, self.graph_ok,
            len(self.node.get_publishers_info_by_topic('/cmd_vel_raw')),
            len(self.node.get_publishers_info_by_topic('/pump_cmd')))
        if set(errors) - {'WAIT_NO_ACTIVE_GOAL', 'WAIT_REAL_GOAL_CANCELED'}:
            return None
        return request['goal_uuid']

    def before_step(self, now):
        previous = getattr(self.node, self.core_attr)
        gap_stop = (self.role == 'gateway' and getattr(previous, 'app_nav_ticket', None) is not None
                    and previous.stop_latched and previous.stop_reason == 'NAV_INPUT_OR_TOPOLOGY_LOST')
        saved_stop = (previous.stop_since, previous.released) if gap_stop else None
        super().before_step(now)
        if self.role != 'gateway':
            return
        now = time.monotonic()
        core = getattr(self.node, self.core_attr)
        if gap_stop and core is previous and not core.stop_latched:
            # R5's separate short identity/input-pause lane must not skip this
            # Nav2-gap ticket's new-command/continuous-proof requirement.
            core.stop_latched = True
            core.stop_reason = 'NAV_INPUT_OR_TOPOLOGY_LOST'
            core.stop_since, core.released = saved_stop
        core.app_recovery_clock = time.monotonic
        core.app_recovery_checked_at = now
        session = self._session(now)
        core.app_session_current = session is not None
        core.app_operator_session = session or {}
        # An authenticated operator may pause ARM while inputs are late. Keep
        # only its SAME intent token through that pause; _session() above and
        # the full R5 preflight must be current again before recovery can move.
        intent_session = self._session(now, allow_input_pause=True)
        lease = obj(obj(intent_session).get('app_navigation_lease'))
        runtime = obj(obj(intent_session).get('runtime_rebind'))
        # Pin this recovery to one operator instance, X epoch, device identity,
        # runtime generation and the endpoint GIDs already validated by R5.
        valid_lease = (intent_session is not None and valid_uuid(lease.get('session'))
            and type(lease.get('cancel_epoch')) is int and lease['cancel_epoch'] >= 0
            and intent_session.get('cycle') is None and intent_session.get('navigation_cancel_uuid') is None
            and not obj(intent_session.get('app_goal_admission')).get('cancel_pending')
            and runtime.get('held') is not True and not runtime.get('cancel_pending')
            and runtime.get('generation') == runtime.get('applied_generation')
            and not self.node.get_publishers_info_by_topic('/pump_cmd')
            and (self.participant.context is None or self.participant.phase == 'COMMITTED'))
        core.app_recovery_binding = ((lease['session'], lease['cancel_epoch'],
            tuple(self.continuity.bound), runtime.get('generation'),
            tuple(sorted(self.pins.items()))) if valid_lease else None)
        actuator, received = self.participant.inputs.get('actuator', (None, None))
        off_check = fresh_motion_off if obj(session).get('navigation_only') is True else fresh_off
        core.app_recovery_off = (session is not None and off_check(actuator, received, now))
        if session is None:
            core.app_idle_since = core.app_idle_seen = None
            core.app_terminal_history = {}
            return
        rows, received = self.participant.inputs.get('status', (None, None))
        if finite(received) and received <= now:
            for uid, status in rows or ():
                if uid == core.goal_uuid and valid_uuid(uid) and status in (4, 5, 6) and sum(u == uid for u, _ in rows) == 1:
                    self.app_history[uid] = status
            while len(self.app_history) > 128:
                self.app_history.pop(next(iter(self.app_history)))
        core.app_terminal_history = dict(self.app_history)
        reset = obj(session.get('app_navigation_reset'))
        targets = reset.get('goal_uuids')
        if (session.get('cycle') is not None or type(reset.get('schema_version')) is not int
                or reset['schema_version'] != 1
                or not valid_uuid(reset.get('request_id')) or not isinstance(targets, list)
                or any(not valid_uuid(u) for u in targets) or core.goal_uuid not in targets):
            return
        terminal = core._normal_terminal_result(now)
        _, stopped = core._stationary(now)
        if (terminal in (4, 5, 6) and not core.fire_latched and stopped
                and core._cmd_zero(now, True) and core._actuator_off(now)
                and not self.node.get_publishers_info_by_topic('/cmd_vel_raw')
                and not self.node.get_publishers_info_by_topic('/pump_cmd')
                and core.stop_reason in ('NAV_INPUT_OR_TOPOLOGY_LOST', 'ARMING_INPUT_LOST')):
            core.normal_terminal = terminal
            core.stop_reason = {4: 'NAV_GOAL_SUCCEEDED', 5: 'NAV_GOAL_CANCELED', 6: 'NAV_GOAL_ABORTED'}[terminal]

    def _reset(self, context):
        super()._reset(context)
        if self.role == 'guard':
            self.blocked_goals = AppBlockedGoalCancel()
        self._bind_guard_preflight()

    def decorate(self, payload, now, **kwargs):
        payload = super().decorate(payload, now, **kwargs)
        if self.role == 'gateway':
            import json
            stop = (payload.get('cycle_goal_uuid'), payload.get('stop_reason'))
            if stop != getattr(self, 'app_stop_logged', None):
                self.app_stop_logged = stop
                if stop[1]:
                    self.node.get_logger().info('APP_NAV_STOP '+json.dumps(dict(
                        goal_uuid=stop[0], reason=stop[1], blockers=payload.get('blockers'),
                        state=payload.get('state'), raw_publishers=payload.get('raw_publishers')),
                        allow_nan=False))
            recovery = obj(payload.get('app_navigation_recovery'))
            transition = (recovery.get('state'), recovery.get('goal_uuid'), recovery.get('reason'))
            if transition != getattr(self, 'app_recovery_logged', None):
                self.app_recovery_logged = transition
                if recovery:
                    self.node.get_logger().info('APP_NAV_RECOVERY '+json.dumps(recovery, allow_nan=False))
        return payload
