"""Durable shadow session lifecycle. Never enables owners or publishes commands."""

import math

from fire_robot_mission.shadow_budget import LIMITS


class ShadowSessionRuntime:
    def __init__(self, store, manifest, *, timeout_s=0.35, absence_hold_s=1.5):
        if (type(timeout_s) not in (int, float) or not 0 < timeout_s <= .35 or
                type(absence_hold_s) not in (int, float) or not math.isfinite(absence_hold_s) or
                absence_hold_s < 1.5):
            raise ValueError('shadow runtime may not weaken time limits')
        self.store, self.manifest = store, manifest
        self.timeout_s, self.absence_hold_s = timeout_s, absence_hold_s
        self.owners = dict.fromkeys(manifest.required)
        self.owner_instances = {}
        self.recovery_latch = 'PERSISTED_SESSION_REQUIRES_RECOVERY' if store.state['pending'] else None
        self.blocker = self.recovery_latch
        self.navigation_blocker = None
        self.absence_since = self.absence_last = None
        self.absence_count = 0
        self.closed_count = 0

    def observe_owner(self, owner, report, effective_at):
        if owner not in self.owners:
            self.recovery_latch = 'OMITTED_OWNER_REPORTED:' + owner
            return
        self.owners[owner] = None
        if (type(report) is not dict or report.get('live_enabled') is not False or
                type(effective_at) not in (int, float) or not math.isfinite(effective_at) or
                type(report.get('source')) is not dict or
                type(report['source'].get('instance_id')) is not str):
            self.recovery_latch = 'SHADOW_OWNER_LIVE_OR_UNDECLARED:' + owner
            return
        source = report['source']
        previous = self.owner_instances.get(owner)
        if previous is not None and source['instance_id'] != previous:
            self.recovery_latch = 'SHADOW_OWNER_PROCESS_CHANGED:' + owner
            return
        self.owner_instances[owner] = source['instance_id']
        self.owners[owner] = effective_at

    def owner_blocker(self, now):
        if self.recovery_latch is not None:
            return self.recovery_latch
        if self.navigation_blocker is not None:
            return self.navigation_blocker
        if type(now) not in (int, float) or not math.isfinite(now) or now < 0:
            return 'SHADOW_CLOCK_INVALID'
        if any(at is None or not 0 <= now - at <= self.timeout_s for at in self.owners.values()):
            return 'SHADOW_OWNER_REPORT_NOT_CURRENT'
        return None

    def observe_absence(self, *, absent, event_at, now):
        if (type(now) not in (int, float) or not math.isfinite(now) or
                absent is not True or type(event_at) not in (int, float) or
                not math.isfinite(event_at) or not 0 <= now - event_at <= self.timeout_s):
            self.absence_since = self.absence_last = None
            self.absence_count = 0
            return False
        if self.absence_last is None or event_at != self.absence_last:
            if (self.absence_last is None or event_at < self.absence_last or
                    event_at - self.absence_last > self.timeout_s):
                self.absence_since, self.absence_count = now, 0
            self.absence_last = event_at
            self.absence_count += 1
        return self._absence_ready(now)

    def _absence_ready(self, now):
        return (self.absence_since is not None and self.absence_count >= 2 and
                now - self.absence_since >= self.absence_hold_s and
                self.absence_last is not None and 0 <= now - self.absence_last <= self.timeout_s)

    def try_begin(self, core, goal_uuid, now):
        self.blocker = self.owner_blocker(now)
        if self.blocker is None and self.store.state['pending'] is not None:
            self.blocker = 'PERSISTED_SESSION_REQUIRES_RECOVERY'
        if self.blocker is None and self.store.state['used']['sessions'] >= LIMITS['sessions']:
            self.blocker = 'SHADOW_ENCOUNTER_BUDGET_EXHAUSTED'
        if self.blocker is not None:
            return None
        identity = core.begin(goal_uuid)
        if identity is not None:
            # Must persist before this new identity can be broadcast.
            self.store.reserve_session(identity.session_id, goal_uuid)
        return identity

    def operator_request(self, core, now, *, reset_encounter=False, idle_goal=None):
        if type(reset_encounter) is not bool:
            raise ValueError('operator request must be explicit')
        pending = self.store.state['pending']
        if type(now) not in (int, float) or not math.isfinite(now) or now < 0:
            raise ValueError('invalid operator event time')
        if (core.phase == 'STOPPING' and core.close_request is not None and
                now < core.close_request.requested_at):
            raise ValueError('operator clock moved backward')
        if reset_encounter:
            if core.identity is not None or pending is not None:
                raise ValueError('encounter reset requires idle lifecycle')
            identity = core.begin(None)
            self.store.begin_reset_probe(identity.session_id)
            core.request_close(identity.session_id, 'OPERATOR_CANCEL', now)
        elif core.identity is None:
            goal = idle_goal if pending is None else pending['goal_uuid']
            identity = core.begin(goal)
            if identity is None:
                raise ValueError('invalid recovery goal identity')
            if pending is None:
                self.store.begin_probe(identity.session_id, goal, purpose='RECOVERY')
            else:
                self.store.recover(identity.session_id, goal)
            core.request_close(identity.session_id, 'SAFETY_ABORT', now)
        else:
            if core.phase == 'ACTIVE':
                core.request_close(core.identity.session_id, 'OPERATOR_CANCEL', now)
            elif not core.renew_close_for_recovery(now, operator_confirmed=True):
                raise ValueError('recovery close could not be renewed')
            self.store.recover(core.identity.session_id, core.identity.nav_goal_uuid)
        # Explicit operator action may establish NEW post-publish proof only.
        # No arm/watchdog/budget reset occurs here.
        self.recovery_latch = None
        self.blocker = 'NEW_POST_CLOSE_PROOF_REQUIRED'
        self.owners = dict.fromkeys(self.manifest.required)
        self.owner_instances.clear()
        self.absence_since = self.absence_last = None
        self.absence_count = 0
        return core.close_request

    def finish_after_publish(self, mission, result, owners, global_stop, *, now, clock, recheck=None):
        core = mission._fire_session
        self.blocker = self.owner_blocker(now)
        if self.blocker is not None or not result.ready:
            self.blocker = self.blocker or 'GLOBAL_STOP_PROOF_NOT_READY'
            return False
        evidence = result.evidence
        pending = self.store.state['pending']
        if pending is None or pending['session_id'] != evidence.session_id:
            self.blocker = 'SHADOW_LEDGER_SESSION_MISMATCH'
            return False
        resetting = pending['purpose'] == 'RESET'
        if resetting and (not self._absence_ready(now) or mission._detection_health(now)[0]):
            self.blocker = 'RESET_REQUIRES_FRESH_CONFIRMED_ABSENCE'
            return False
        blocker = core._cleanup_blocker(evidence, now)
        if blocker is not None:
            self.blocker = blocker
            return False
        self.store.mark_completion(evidence.session_id)
        if recheck is not None:
            # Actual ROS adapter rechecks clock correspondence and all cached
            # sources after durable I/O; no graph query or stamp renewal.
            result = recheck()
            if not result.ready:
                self.blocker = 'GLOBAL_PROOF_EXPIRED_DURING_COMMIT'
                return False
            evidence = result.evidence
        checked_at = clock()  # Include fsync/publication delay in evidence age.
        if resetting and (not self._absence_ready(checked_at) or mission._detection_health(checked_at)[0]):
            self.blocker = 'RESET_ABSENCE_EXPIRED_DURING_COMMIT'
            return False
        if self.owner_blocker(checked_at) is not None or not mission.finish_verified_session(evidence, checked_at):
            self.blocker = 'CLEANUP_PROOF_EXPIRED_DURING_COMMIT'
            return False
        # A failure here propagates before any IDLE heartbeat. The durable
        # pending record survives and requires explicit recovery after restart.
        self.store.finish_completion(evidence.session_id, reset_verified=resetting)
        owners.retire_after_close(core, evidence, checked_at)
        global_stop.retire_after_close(core, evidence, checked_at)
        self.closed_count += 1
        self.blocker = None
        self.absence_since = self.absence_last = None
        self.absence_count = 0
        return True

    def snapshot(self):
        return {'mode': 'SHADOW_SESSION_CLEANUP', 'actuation_authorized': False,
                'blocker': self.recovery_latch or self.navigation_blocker or self.blocker,
                'closed_count': self.closed_count,
                'budget': self.store.snapshot()}
