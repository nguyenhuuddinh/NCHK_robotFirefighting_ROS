"""App-only recovery of delayed navigation inputs for the SAME live Goal.

The R5 engine still stops, proves zero and relinquishes raw ownership. This
lane may reacquire it only after new input and a continuous stopped proof.
It never sends an action/Goal, changes a timeout, or forwards a cached command.
"""
from fire_robot_mission.operational_cycle import finite
from collections import Counter
from fire_robot_mission.live_nav_stop_gateway import RAW_ENDPOINT, EXPECTED_SHADOW_PUBLISHERS, SAFETY_ENDPOINT


class AppNavRecovery:
    RECOVERY_STABLE_S = 1.
    INPUT_GAPS = frozenset(('NAV_COMMAND_MISSING_OR_INVALID', 'ODOM_NOT_FRESH',
                           'MISSION_MISSING_OR_STALE', 'LIVE_ARM_MISSING_OR_STALE',
                           'MISSION_HEALTH_BLOCKED'))

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self.app_nav_ticket = None
        self.app_nav_recovery = None

    def _app_recovery_scope(self, now):
        # This is the retained intent identity, never freshness permission.
        # A late graph query invalidates the current proof interval, not the
        # same operator/Goal token. Changed sources/identity still revoke it.
        return getattr(self, 'app_recovery_binding', None)

    def _app_proof_current(self, now):
        checked = getattr(self, 'app_recovery_checked_at', None)
        return checked is None or finite(checked) and 0 <= now-checked <= self.mission_timeout

    def _start_next_goal(self, new_goal):
        self.app_nav_ticket = self.app_nav_recovery = None
        return super()._start_next_goal(new_goal)

    def _app_gap_only(self, blockers):
        # Keep the existing intent while data is late, never its permission to
        # move. A supervisor's odom-fresh flag may also fall during an odom gap;
        # any actual health/contract/ownership fault remains a hard boundary.
        mission = self.mission if isinstance(self.mission, dict) else {}
        health = mission.get('health') or {}
        odom_gap = ('MISSION_HEALTH_BLOCKED' not in blockers or
                    health.get('status') == 'OK' and not health.get('blockers')
                    and (mission.get('robot') or {}).get('odom_fresh') is False)
        return (bool(blockers) and set(blockers) <= self.INPUT_GAPS and odom_gap
                and isinstance(self.nav, (tuple, list)) and len(self.nav) == 6
                and all(finite(v) for v in self.nav)
                and all(abs(self.nav[i]) <= 1e-6 for i in (1, 2, 3, 4))
                and abs(self.nav[0]) <= self.max_nav_linear
                and abs(self.nav[5]) <= self.max_nav_angular)

    def _app_recovery_valid(self, now, blockers, raw_ok, nav_ok, safety_ok,
                            status_count, mission_count, odom_count,
                            arm_count, guard_count):
        ticket = self.app_nav_ticket
        if ticket is None:
            return False
        reason = None
        if self._app_recovery_scope(now) != ticket['binding']:
            reason = 'SESSION_OR_IDENTITY_CHANGED_OR_WITHDRAWN'
        elif self._active_goal(now) != ticket['goal_uuid'] or self.goal_uuid != ticket['goal_uuid']:
            reason = 'GOAL_CHANGED_OR_TERMINAL'
        elif self.fire_latched or self.normal_terminal is not None:
            reason = 'FIRE_OR_TERMINAL_HANDOFF'
        elif (not self.enabled or not raw_ok or not nav_ok or not safety_ok
                or not status_count == mission_count == odom_count == arm_count == guard_count == 1):
            reason = 'OWNERSHIP_OR_TOPOLOGY_CHANGED'
        elif blockers and not self._app_gap_only(blockers):
            reason = 'NAVIGATION_PREFLIGHT_CHANGED'
        elif self.stop_reason not in ('NAV_INPUT_OR_TOPOLOGY_LOST', 'ARMING_INPUT_LOST', ''):
            reason = 'STOP_CAUSE_CHANGED'
        if reason:
            self.app_nav_recovery = dict(state='BLOCKED', goal_uuid=ticket['goal_uuid'], reason=reason,
                                         blockers=list(blockers))
            self.app_nav_ticket = None
            return False
        return True

    def _app_resume_same_goal(self, now, blockers, raw_ok, nav_ok, safety_ok,
                              stationary, cmd_zero, raw_count, status_count,
                              mission_count, odom_count, arm_count):
        if not self._app_recovery_valid(now, blockers, raw_ok, nav_ok, safety_ok,
                status_count, mission_count, odom_count, arm_count,
                getattr(self, 'app_guard_count', 0)):
            return False
        ticket = self.app_nav_ticket
        new_command = (not blockers and self._app_proof_current(now)
                       and getattr(self, 'app_session_current', False)
                       and finite(self.nav_at) and ticket['since'] < self.nav_at <= now
                       and now - self.nav_at <= self.nav_timeout)
        stopped = (raw_count == 0 and stationary and cmd_zero and self._actuator_off(now)
                   and getattr(self, 'app_recovery_off', False) is True)
        ready = new_command and stopped
        seen = ticket.get('seen')
        if not ready:
            ticket['ready_since'] = ticket['seen'] = None
        else:
            if (ticket.get('ready_since') is None or not finite(seen)
                    or not 0 <= now - seen <= self.mission_timeout):
                ticket['ready_since'] = now
            ticket['seen'] = now
        stable = max(0., now - ticket['ready_since']) if ready else 0.
        self.app_nav_recovery = dict(state='VERIFYING' if ready else 'WAIT_STOP' if new_command else 'WAIT_COMMAND',
            reason='VERIFY_FRESH_STOP_AND_INPUTS' if ready else 'WAIT_ZERO_OFF_RELEASE' if new_command else 'NAV_COMMAND_GAP',
            goal_uuid=ticket['goal_uuid'], stable_s=round(stable, 3), blockers=list(blockers))
        if not new_command and set(blockers) - {'NAV_COMMAND_MISSING_OR_INVALID'}:
            self.app_nav_recovery['reason'] = 'WAIT_CURRENT_NAVIGATION_INPUTS'
        if stable < self.RECOVERY_STABLE_S:
            return False
        # Deliberately do not use _start_next_goal(): this is the SAME action,
        # not a new Goal, terminal result, consumed fire cycle or Nav2 restart.
        self.stop_latched = False
        self.stop_reason = ''
        self.stop_since = None
        self.released = False
        self.publisher_created_at = None
        # Reusing the action does not reuse the raw publisher's old GID proof.
        # Force R5's normal discovery/operator acknowledgement on reacquire.
        self.ever_forwarded = False
        self.app_nav_recovery = dict(state='REACQUIRING', reason='FRESH_SAME_GOAL_VERIFIED',
                                     goal_uuid=ticket['goal_uuid'], blockers=[])
        ticket['ready_since'] = ticket['seen'] = None
        return True

    def evaluate(self, now, publisher_owned, raw_sources, nav_sources,
                 cmd_sources, status_publishers, mission_publishers=0,
                 odom_publishers=0, arm_publishers=0, guard_publishers=0):
        clock = getattr(self, 'app_recovery_clock', None)
        if clock is not None:
            now = max(float(now), clock())
        was_stopped, forwarded = self.stop_latched, self.ever_forwarded
        self.app_guard_count = guard_publishers
        if self.app_nav_ticket is not None:
            # Invalidate BEFORE the R5 engine can forward its first command
            # after CREATE. A lease withdrawal in that window must win too.
            raw_ok = Counter(raw_sources) == (RAW_ENDPOINT if publisher_owned else Counter())
            nav_ok = Counter(nav_sources) == EXPECTED_SHADOW_PUBLISHERS
            safety_ok = Counter(cmd_sources) == SAFETY_ENDPOINT
            odom_fresh, _ = self._stationary(now)
            blockers = self._nav_eligible(now, nav_ok, safety_ok, status_publishers,
                mission_publishers, odom_publishers, arm_publishers, odom_fresh)
            if not self._app_recovery_valid(now, blockers, raw_ok, nav_ok, safety_ok,
                    status_publishers, mission_publishers, odom_publishers, arm_publishers, guard_publishers):
                was_stopped = True  # This tick cannot mint a replacement ticket.
                self.stop_latched = True
                if not self.stop_reason:
                    self.stop_reason = 'NAV_INPUT_OR_TOPOLOGY_LOST'
            elif (self.app_nav_recovery.get('state') == 'REACQUIRING'
                    and (blockers or not self._app_proof_current(now)
                         or not getattr(self, 'app_session_current', False)
                         or not (self._stationary(now)[1] and self._cmd_zero(now, safety_ok)
                             and self._actuator_off(now) and getattr(self, 'app_recovery_off', False) is True))):
                # Recheck the stopped/OFF proof through the CREATE -> first
                # forward window; the proof cannot be spent while it is stale.
                was_stopped = True
                self.stop_latched = True
                self.stop_reason = 'NAV_INPUT_OR_TOPOLOGY_LOST'
                self.app_nav_ticket['ready_since'] = self.app_nav_ticket['seen'] = None
                self.app_nav_recovery = dict(state='WAIT_STOP', reason='WAIT_ZERO_OFF_RELEASE',
                                             goal_uuid=self.goal_uuid, blockers=list(blockers))
        linear, angular, payload, action = super().evaluate(now, publisher_owned, raw_sources,
            nav_sources, cmd_sources, status_publishers, mission_publishers,
            odom_publishers, arm_publishers, guard_publishers)
        blockers = payload.get('blockers', [])
        topology = (payload.get('raw_owner_fingerprint_ok') is True
                    and payload.get('nav_shadow_fingerprint_ok') is True
                    and payload.get('safety_gate_fingerprint_ok') is True
                    and status_publishers == mission_publishers == odom_publishers == arm_publishers == guard_publishers == 1)
        if (not was_stopped and forwarded and self.stop_latched and not self.fire_latched
                and self.stop_reason == 'NAV_INPUT_OR_TOPOLOGY_LOST' and topology
                and self._app_recovery_scope(now) is not None and self._app_gap_only(blockers)
                and self._active_goal(now) == self.goal_uuid):
            self.app_nav_ticket = dict(binding=self._app_recovery_scope(now), goal_uuid=self.goal_uuid,
                                       since=now, ready_since=None, seen=None)
            self.app_nav_recovery = dict(state='WAIT_COMMAND', reason='NAV_COMMAND_GAP',
                                         goal_uuid=self.goal_uuid, blockers=list(blockers))
        if self.app_nav_ticket is not None:
            valid = self._app_recovery_valid(now, blockers,
                payload.get('raw_owner_fingerprint_ok') is True,
                payload.get('nav_shadow_fingerprint_ok') is True,
                payload.get('safety_gate_fingerprint_ok') is True,
                status_publishers, mission_publishers, odom_publishers, arm_publishers, guard_publishers)
            if valid and payload.get('state') == 'NAVIGATION':
                self.app_nav_recovery = dict(state='RESUMED', reason='FRESH_SAME_GOAL_COMMAND',
                                             goal_uuid=self.goal_uuid, blockers=[])
                self.app_nav_ticket = None
        if (self.app_nav_ticket is None and self.normal_terminal in (4, 5, 6)
                and payload.get('normal_rearm_ready') is True
                and self.stop_reason in ('NAV_GOAL_SUCCEEDED', 'NAV_GOAL_CANCELED', 'NAV_GOAL_ABORTED')):
            # X/real terminal retirement has its own verified new-Goal lane.
            # A historical recovery label must not keep that proven gate shut.
            self.app_nav_recovery = None
        payload['app_navigation_recovery'] = self.app_nav_recovery
        return linear, angular, payload, action
