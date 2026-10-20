"""Durable pessimistic shadow planning reservations, never command authority.

One encounter record, one pending session, exclusive process lock. Writes
only on admission/completion/operator recovery, not on telemetry/timer ticks.
These reservations are NOT an accounting claim about physical motion/pumping.
"""

from copy import deepcopy
import fcntl
import json
import os
from pathlib import Path
import tempfile
from uuid import uuid4

from fire_robot_mission.session_context import _session_id_matches, _uuid


LIMITS = {'sessions': 3, 'pulse_count': 3, 'pump_ms': 15000,
          'rotation_urad': 6283185, 'approach_um': 2000000}


def valid_session(value):
    parts = value.split(':') if type(value) is str else []
    return len(parts) == 3 and _uuid(parts[1]) and _session_id_matches(value, parts[1])


class ShadowBudgetStore:
    def __init__(self, path, *, initialize=False):
        self._fd = None
        self._poisoned = False
        if type(initialize) is not bool or type(path) is not str or not path:
            raise ValueError('explicit absolute shadow budget path and boolean init required')
        self.path = Path(path)
        if not self.path.is_absolute() or not self.path.parent.is_dir():
            raise ValueError('budget parent must exist; path must be absolute')
        try:
            self._fd = os.open(str(self.path) + '.lock', os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
            fcntl.flock(self._fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
            if self.path.is_symlink():
                raise ValueError('budget symlink refused')
            if self.path.exists():
                if self.path.stat().st_size > 16384:
                    raise ValueError('budget record too large')
                self.state = json.loads(self.path.read_text())
                self._validate(self.state)
            elif initialize:
                self.state = {'schema_version': 1, 'encounter_id': uuid4().hex,
                              'limits': dict(LIMITS), 'used': dict.fromkeys(LIMITS, 0),
                              'pending': None, 'recoveries': 0, 'last_completed': None}
                self._write(self.state)
            else:
                raise ValueError('budget missing; initialization must be explicit')
        except Exception:
            self.close()
            raise

    @staticmethod
    def _validate(state):
        if (type(state) is not dict or set(state) != {
                'schema_version', 'encounter_id', 'limits', 'used', 'pending', 'recoveries', 'last_completed'} or
                type(state['schema_version']) is not int or state['schema_version'] != 1 or
                not _uuid(state['encounter_id']) or type(state['limits']) is not dict or
                state['limits'] != LIMITS or
                type(state['used']) is not dict or set(state['used']) != set(LIMITS) or
                any(type(value) is not int or not 0 <= value <= LIMITS[key]
                    for key, value in state['used'].items()) or
                type(state['recoveries']) is not int or not 0 <= state['recoveries'] <= 3 or
                (state['last_completed'] is not None and not valid_session(state['last_completed']))):
            raise ValueError('invalid/corrupt shadow budget record')
        pending = state['pending']
        sessions = state['used']['sessions']
        if (state['used']['pulse_count'] != sessions or state['used']['pump_ms'] != sessions * 5000 or
                any(state['used'][key] != LIMITS[key] * sessions // 3
                    for key in ('rotation_urad', 'approach_um'))):
            raise ValueError('inconsistent shadow reservations')
        if pending is not None:
            if (type(pending) is not dict or set(pending) != {
                    'session_id', 'goal_uuid', 'purpose', 'origin_session_id', 'completion_marked'} or
                    not valid_session(pending['session_id']) or not valid_session(pending['origin_session_id']) or
                    (pending['goal_uuid'] is not None and not _uuid(pending['goal_uuid'])) or
                    pending['purpose'] not in ('FIRE', 'RECOVERY', 'RESET') or
                    type(pending['completion_marked']) is not bool or
                    (pending['purpose'] == 'FIRE' and (sessions == 0 or
                     pending['origin_session_id'] != pending['session_id'])) or
                    (pending['purpose'] == 'RESET' and pending['goal_uuid'] is not None)):
                raise ValueError('invalid pending shadow session')

    def _write(self, state):
        self._validate(state)
        fd, temporary = tempfile.mkstemp(prefix=self.path.name + '.', dir=self.path.parent)
        try:
            with os.fdopen(fd, 'w') as stream:
                json.dump(state, stream, separators=(',', ':'), allow_nan=False)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, self.path)
            directory = os.open(self.path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(directory)
            finally:
                os.close(directory)
        finally:
            if os.path.exists(temporary):
                os.unlink(temporary)

    def _commit(self, state):
        # An uncertain write is fatal to the caller; do not keep acting on RAM.
        if self._fd is None or self._poisoned:
            raise RuntimeError('shadow budget closed or prior write uncertain; reopen required')
        try:
            self._write(state)
        except Exception:
            self._poisoned = True
            raise
        self.state = state

    def reserve_session(self, session_id, goal_uuid):
        if (not valid_session(session_id) or (goal_uuid is not None and not _uuid(goal_uuid)) or
                self.state['pending'] is not None or self.state['used']['sessions'] >= LIMITS['sessions']):
            raise ValueError('shadow encounter admission blocked')
        state = deepcopy(self.state)
        remaining = LIMITS['sessions'] - state['used']['sessions']
        for key in LIMITS:
            charge = 1 if key in ('sessions', 'pulse_count') else (
                5000 if key == 'pump_ms' else (LIMITS[key] - state['used'][key]) // remaining)
            state['used'][key] += charge
        state['pending'] = {'session_id': session_id, 'goal_uuid': goal_uuid, 'purpose': 'FIRE',
                            'origin_session_id': session_id, 'completion_marked': False}
        self._commit(state)

    def recover(self, session_id, goal_uuid):
        old = self.state['pending']
        if (old is None or not valid_session(session_id) or
                goal_uuid != old['goal_uuid']):
            raise ValueError('shadow recovery blocked')
        state = deepcopy(self.state)
        state['recoveries'] = min(3, state['recoveries'] + 1)
        state['pending'].update(session_id=session_id,
                                purpose='RESET' if old['purpose'] == 'RESET' else 'RECOVERY',
                                completion_marked=False)
        self._commit(state)

    def begin_reset_probe(self, session_id):
        self.begin_probe(session_id, None, purpose='RESET')

    def begin_probe(self, session_id, goal_uuid, *, purpose):
        if not valid_session(session_id) or self.state['pending'] is not None:
            raise ValueError('encounter reset requires idle ledger')
        if (purpose not in ('RESET', 'RECOVERY') or
                (goal_uuid is not None and not _uuid(goal_uuid)) or
                (purpose == 'RESET' and goal_uuid is not None)):
            raise ValueError('invalid maintenance probe')
        state = deepcopy(self.state)
        if purpose == 'RECOVERY':
            state['recoveries'] = min(3, state['recoveries'] + 1)
        state['pending'] = {'session_id': session_id, 'goal_uuid': goal_uuid, 'purpose': purpose,
                            'origin_session_id': session_id, 'completion_marked': False}
        self._commit(state)

    def mark_completion(self, session_id):
        pending = self.state['pending']
        if pending is None or pending['session_id'] != session_id:
            raise ValueError('completion session mismatch')
        if not pending['completion_marked']:
            state = deepcopy(self.state)
            state['pending']['completion_marked'] = True
            self._commit(state)

    def finish_completion(self, session_id, *, reset_verified=False):
        pending = self.state['pending']
        if (pending is None or pending['session_id'] != session_id or
                not pending['completion_marked'] or type(reset_verified) is not bool or
                (pending['purpose'] == 'RESET' and not reset_verified)):
            raise ValueError('completion not verified')
        state = deepcopy(self.state)
        if pending['purpose'] == 'RESET':
            state['encounter_id'] = uuid4().hex
            state['used'] = dict.fromkeys(LIMITS, 0)
            state['recoveries'] = 0
        state['pending'], state['last_completed'] = None, session_id
        self._commit(state)

    def snapshot(self):
        pending = self.state['pending']
        return {'planning_only': True, 'actuation_authorized': False,
                'encounter_id': self.state['encounter_id'], 'reserved': dict(self.state['used']),
                'limits': dict(LIMITS), 'recoveries': self.state['recoveries'],
                'recoveries_saturated': self.state['recoveries'] == 3,
                'pending_session_id': None if pending is None else pending['session_id'],
                'purpose': None if pending is None else pending['purpose']}

    def close(self):
        if self._fd is not None:
            os.close(self._fd)
            self._fd = None

    def __del__(self):
        self.close()
