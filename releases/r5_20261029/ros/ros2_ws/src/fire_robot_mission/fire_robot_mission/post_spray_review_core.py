"""Read-only post-spray decision preview; no robot commands."""

import math

from fire_robot_mission.pump_demo_core import POSSIBLE_EARLY_EXTINGUISH


def _fresh(received, now, limit):
    return received is not None and 0 <= now - received <= limit


def _obj(value):
    return value if isinstance(value, dict) else {}


class PostSprayReviewCore:
    """Review one pump pulse using fresh camera, IR and robot state."""

    def __init__(self, *, settle_s=1.0, confirm_s=1.0,
                 input_timeout_s=0.6, max_image_age_ms=300.0,
                 max_ir_age_ms=500.0, max_pulses=3,
                 minimum_off_s=0.5, ir_confirm_samples=5):
        values = (settle_s, confirm_s, input_timeout_s,
                  max_image_age_ms, max_ir_age_ms, minimum_off_s)
        if any(type(value) not in (int, float) or
               not math.isfinite(value) or value <= 0 for value in values):
            raise ValueError(
                'review timing limits must be positive and finite')
        if type(max_pulses) is not int or not 1 <= max_pulses <= 5:
            raise ValueError('max_pulses must be an integer from 1 to 5')
        if type(ir_confirm_samples) is not int or ir_confirm_samples not in (3,5):raise ValueError('IR_CONFIRM_SAMPLES_INVALID')
        self.ir_confirm_samples=ir_confirm_samples
        self.settle_s = float(settle_s)
        self.confirm_s = float(confirm_s)
        self.input_timeout_s = float(input_timeout_s)
        self.max_image_age_ms = float(max_image_age_ms)
        self.max_ir_age_ms = float(max_ir_age_ms)
        self.max_pulses = max_pulses
        self.minimum_off_s = float(minimum_off_s)
        self.inputs = {}
        self.phase = 'WAITING_FOR_PULSE'
        self.pulse_index = 0
        self.pulse_end_at = None
        self.candidate = None
        self.candidate_since = None
        self.pump_off_since = None
        self.goal_uuid = None

    def update(self, name, payload, now):
        if name not in ('pump', 'vision', 'ir', 'alignment', 'mission'):
            raise ValueError('unknown review input')
        self.inputs[name] = (payload, float(now))

    def _input(self, name, now):
        payload, received = self.inputs.get(name, (None, None))
        return payload if _fresh(received, now, self.input_timeout_s) and \
            isinstance(payload, dict) else None

    def _evidence(self, now):
        blockers = []
        vision = self._input('vision', now)
        ir = self._input('ir', now)
        mission = self._input('mission', now)
        alignment = self._input('alignment', now)
        if vision is None or vision.get('schema_version') != 1 or \
                vision.get('observe_only') is not True or \
                vision.get('state') != 'ok' or \
                not isinstance(vision.get('fire'), dict):
            blockers.append('VISION_MISSING_OR_INVALID')
        else:
            age = _obj(vision.get('source')).get('age_ms')
            if type(age) not in (int, float) or not math.isfinite(age) or \
                    not 0 <= age <= self.max_image_age_ms:
                blockers.append('VISION_SOURCE_STALE')
        if ir is None or ir.get('schema_version') != 1 or \
                ir.get('mode') != 'OBSERVE_ONLY' or \
                ir.get('observe_only') is not True or \
                ir.get('calibration') != 'PROVISIONAL_SINGLE_FLAME' or \
                ir.get('validated_for_actuation') is not False or \
                type(ir.get('stable_samples')) is not int or \
                ir['stable_samples'] < 3:
            blockers.append('IR_MISSING_OR_INVALID')
        else:
            age = ir.get('ir_receive_age_ms')
            if type(age) not in (int, float) or not math.isfinite(age) or \
                    not 0 <= age <= self.max_ir_age_ms:
                blockers.append('IR_SOURCE_STALE')
            if not isinstance(ir.get('blockers'), list):
                blockers.append('IR_BLOCKERS_INVALID')
            elif any(item in ir['blockers'] for item in (
                    'IR_TIMEOUT', 'IR_SAMPLE_INVALID',
                    'IR_SAMPLE_STALE_OR_INVALID_AGE', 'IR_JSON_INVALID')):
                blockers.append('IR_SOURCE_INVALID')
        if mission is None or mission.get('schema_version') != 1 or \
                mission.get('observe_only') is not True or \
                _obj(mission.get('robot')).get('odom_fresh') is not True or \
                _obj(mission.get('robot')).get('stationary') is not True or \
                _obj(mission.get('navigation')).get(
                    'goal_active') is not False or \
                _obj(mission.get('health')).get('status') != 'OK' or \
                _obj(mission.get('health')).get('blockers') != [] or \
                _obj(mission.get('safety')).get(
                    'motion_command_publisher') is not False or \
                _obj(mission.get('safety')).get(
                    'actuator_command_publisher') is not False:
            blockers.append('ROBOT_NOT_CONFIRMED_STOPPED')
        if blockers:
            return 'HOLD', blockers
        fire = vision['fire']
        detected = fire.get('detected_now')
        confirmed = fire.get('confirmed')
        band = ir.get('ir_band')
        if detected is False and confirmed is False:
            if band == 'NO_FLAME_SIGNATURE' and \
                    'NO_IR_FLAME_SIGNATURE' in ir.get('blockers', []):
                return 'EXTINGUISHED_CANDIDATE', []
            return 'HOLD', ['VISION_IR_DISAGREE']
        if detected is not True or confirmed is not True:
            return 'HOLD', ['FIRE_NOT_STABLY_CONFIRMED']
        if band == 'NO_FLAME_SIGNATURE':
            return 'HOLD', ['VISION_IR_DISAGREE']
        if band not in ('SPRAY_RANGE', 'APPROACH', 'TOO_FAR', 'TOO_CLOSE') or \
                ir.get('stable_samples', 0) < self.ir_confirm_samples:
            return 'HOLD', ['IR_BAND_NOT_STABLE']
        if alignment is None or alignment.get('schema_version') != 1 or \
                alignment.get('observe_only') is not True:
            return 'HOLD', ['ALIGNMENT_MISSING_OR_INVALID']
        if alignment.get('state') != 'CENTERED':
            return 'REALIGN_CANDIDATE', []
        if band == 'SPRAY_RANGE' and ir.get('zone') == 'SPRAY_RANGE' and \
                ir.get('blockers') == []:
            return 'RESPRAY_CANDIDATE', []
        if band in ('APPROACH', 'TOO_FAR') and \
                ir.get('zone') == band and ir.get('blockers') == []:
            return 'APPROACH_CANDIDATE', []
        return 'HOLD', ['RANGE_UNCERTAIN_OR_TOO_CLOSE']

    def step(self, now):
        now = float(now)
        pump = self._input('pump', now)
        if self.phase == 'NO_FIRE_EVIDENCE' and pump is not None and \
                pump.get('state') == 'WAITING_FOR_ADMISSION' and \
                pump.get('pump_on') is False and \
                pump.get('goal_uuid') is None:
            self.phase = 'WAITING_FOR_PULSE'
            self.pulse_index = 0
            self.pulse_end_at = None
            self.candidate = None
            self.candidate_since = None
            self.pump_off_since = None
            self.goal_uuid = None
        blockers = []
        if pump is None or pump.get('schema_version') != 1 or \
                pump.get('mode') != 'SUPERVISED_DEMO' or \
                pump.get('one_shot') is not True:
            blockers.append('PUMP_STATE_MISSING_OR_INVALID')
        elif pump.get('state') == 'FIRING' and \
                pump.get('pump_on') is True and \
                self.phase not in ('WAITING_FOR_PULSE', 'FIRING',
                                   'ABORTED'):
            candidate, evidence_blockers = self._evidence(now)
            if self.phase == 'RESPRAY_READY_SHADOW' and \
                    self.pulse_index < self.max_pulses and \
                    self.pump_off_since is not None and \
                    now - self.pump_off_since >= self.minimum_off_s and \
                    candidate == 'RESPRAY_CANDIDATE' and \
                    not evidence_blockers:
                self.pulse_index += 1
                self.phase = 'FIRING'
                self.pulse_end_at = None
                self.candidate = None
                self.candidate_since = None
            else:
                self.pulse_index += 1
                self.phase = 'ABORTED'
                blockers.append('UNEXPECTED_PUMP_RESTART')
                blockers.extend(evidence_blockers)
        elif pump.get('pump_on') is not False and self.phase not in (
                'WAITING_FOR_PULSE', 'FIRING'):
            blockers.append('PUMP_NOT_CONFIRMED_OFF')

        if not blockers and self.phase == 'WAITING_FOR_PULSE':
            if pump.get('state') == 'FIRING' and pump.get('pump_on') is True:
                self.pulse_index += 1
                self.goal_uuid = pump.get('goal_uuid')
                self.phase = 'FIRING'
        elif not blockers and self.phase == 'FIRING':
            if pump.get('state') == 'ABORTED':
                if pump.get('pump_on') is False and \
                        pump.get('stop_reason') in \
                        POSSIBLE_EARLY_EXTINGUISH:
                    self.phase = 'SETTLING'
                    self.pulse_end_at = now
                else:
                    self.phase = 'ABORTED'
            elif pump.get('state') == 'COMPLETE' and \
                    pump.get('pump_on') is False and \
                    pump.get('stop_reason') == 'PULSE_TIME_LIMIT':
                self.phase = 'SETTLING'
                self.pulse_end_at = now
            elif pump.get('state') != 'FIRING':
                blockers.append('PUMP_SEQUENCE_INVALID')

        decisions = {
            'EXTINGUISHED_CANDIDATE': 'NO_FIRE_EVIDENCE',
            'REALIGN_CANDIDATE': 'REALIGN_REQUIRED',
            'RESPRAY_CANDIDATE': 'RESPRAY_READY_SHADOW',
            'APPROACH_CANDIDATE': 'APPROACH_REQUIRED',
        }
        if self.phase in (*decisions.values(), 'RETRY_LIMIT_REACHED'):
            candidate, evidence_blockers = self._evidence(now)
            expected = decisions.get(candidate)
            if candidate == 'RESPRAY_CANDIDATE' and \
                    self.pulse_index >= self.max_pulses:
                expected = 'RETRY_LIMIT_REACHED'
            if blockers or evidence_blockers or \
                    expected != self.phase:
                self.phase = 'REVIEWING'
                self.candidate = None
                self.candidate_since = None
                blockers.extend(evidence_blockers)
        if self.phase == 'SETTLING' and not blockers and \
                now - self.pulse_end_at >= self.settle_s:
            self.phase = 'REVIEWING'
        if self.phase == 'REVIEWING' and not blockers:
            candidate, blockers = self._evidence(now)
            if candidate != self.candidate or blockers:
                self.candidate = candidate if not blockers else None
                self.candidate_since = now if not blockers else None
            elif self.candidate_since is not None and \
                    now - self.candidate_since >= self.confirm_s:
                if candidate == 'RESPRAY_CANDIDATE' and \
                        self.pulse_index >= self.max_pulses:
                    self.phase = 'RETRY_LIMIT_REACHED'
                else:
                    self.phase = decisions.get(candidate, 'REVIEWING')
        if self.phase in ('SETTLING', 'REVIEWING') and blockers:
            self.candidate = None
            self.candidate_since = None
        if not blockers and pump is not None and \
                pump.get('pump_on') is False:
            if self.pump_off_since is None:
                self.pump_off_since = now
        else:
            self.pump_off_since = None
        return {
            'schema_version': 1,
            'mode': 'SHADOW_ONLY',
            'state': self.phase,
            'pulse_index': self.pulse_index,
            'goal_uuid': self.goal_uuid,
            'max_pulses': self.max_pulses,
            'candidate': self.candidate,
            'blockers': sorted(set(blockers)),
            'safety': {'motion_command_publisher': False,
                       'pump_command_publisher': False,
                       'nav2_action_client': False,
                       'actuation_authorized': False},
        }
