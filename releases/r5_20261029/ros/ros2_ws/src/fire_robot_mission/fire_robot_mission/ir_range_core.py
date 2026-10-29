"""Pure, ROS-independent provisional IR band evaluation."""

from collections import deque
import math
from statistics import median


LEGACY_POLICY = 'LEGACY_BASELINE'
YOLO_RIGHT_ADC_POLICY = 'YOLO_RIGHT_ADC_20261025'


def _number(value):
    return (type(value) in (int, float) and math.isfinite(float(value)))


class IrRangePreviewCore:
    """
    Filter each eye separately and gate a provisional band on YOLO.

    Counts are *not* centimetres. The four bands below are tentative for the
    current candle and unchanged sensor potentiometers. This preview alone
    never authorises motion or pumping; the supervised demo requires separate
    explicit overrides and continuously checked safety gates.
    """

    def __init__(self, *, baselines, baseline_tolerance, min_active_drop,
                 right_thresholds, median_window, stable_samples,
                 ir_timeout_s, vision_timeout_s, max_ir_age_ms,
                 max_image_age_ms, spray_confirm_samples=5,
                 spray_reentry_margin_counts=50,
                 range_policy=LEGACY_POLICY):
        if range_policy not in (LEGACY_POLICY, YOLO_RIGHT_ADC_POLICY):
            raise ValueError('unknown IR range policy')
        if len(baselines) != 3 or any(
                type(value) is not int or not 0 <= value <= 32767
                for value in baselines):
            raise ValueError('baselines must be three ADC counts')
        if (type(baseline_tolerance) is not int
                or type(min_active_drop) is not int
                or not 0 <= baseline_tolerance < min_active_drop):
            raise ValueError('invalid IR baseline/active thresholds')
        if (len(right_thresholds) != 3 or any(
                type(value) is not int or not 0 <= value <= 32767
                for value in right_thresholds)
                or list(right_thresholds) != sorted(set(right_thresholds))):
            raise ValueError(
                'right thresholds must be three increasing counts')
        if (type(median_window) is not int or median_window < 1
                or median_window % 2 != 1
                or type(stable_samples) is not int or stable_samples < 2):
            raise ValueError('median_window must be odd; stability >= 2')
        if (type(spray_confirm_samples) is not int or
                spray_confirm_samples < stable_samples or
                type(spray_reentry_margin_counts) is not int or
                not 0 <= spray_reentry_margin_counts <
                right_thresholds[1] - right_thresholds[0]):
            raise ValueError('invalid spray confirmation or reentry margin')
        for value in (ir_timeout_s, vision_timeout_s, max_ir_age_ms,
                      max_image_age_ms):
            if not _number(value) or value <= 0.0:
                raise ValueError(
                    'freshness limits must be positive and finite')
        self.baselines = tuple(baselines)
        self.range_policy = range_policy
        self.baseline_tolerance = baseline_tolerance
        self.min_active_drop = min_active_drop
        self.right_thresholds = tuple(right_thresholds)
        self.stable_samples = stable_samples
        self.spray_confirm_samples = spray_confirm_samples
        self.spray_reentry_margin_counts = spray_reentry_margin_counts
        self.ir_timeout_s = float(ir_timeout_s)
        self.vision_timeout_s = float(vision_timeout_s)
        self.max_ir_age_ms = float(max_ir_age_ms)
        self.max_image_age_ms = float(max_image_age_ms)
        self._samples = deque(maxlen=median_window)
        self._ir_received = None
        self._ir_error = ''
        self._ir_sample_seq = 0
        self._candidate = None
        self._candidate_count = 0
        self._spray_reentry_locked = False
        self._spray_reentry_count = 0
        self._detection = None
        self._detection_received = None
        self._alignment = None
        self._alignment_received = None

    def _reset_ir(self):
        self._samples.clear()
        self._candidate = None
        self._candidate_count = 0
        self._spray_reentry_locked = False
        self._spray_reentry_count = 0

    def update_ir(self, payload, now):
        """Accept only a complete, fresh ADC snapshot from the Pi bridge."""
        self._ir_received = float(now)
        if (not isinstance(payload, dict)
                or type(payload.get('schema_version')) is not int
                or payload['schema_version'] != 1):
            self._ir_error = 'IR_PAYLOAD_INVALID'
        elif payload.get('valid') is not True:
            self._ir_error = 'IR_SAMPLE_INVALID'
        elif any(
                type(payload.get(key)) is not int
                or not 0 <= payload[key] <= 32767
                for key in ('left', 'center', 'right')):
            self._ir_error = 'IR_COUNTS_INVALID'
        elif any(not _number(payload.get(key))
                 or not 0 <= payload[key] <= self.max_ir_age_ms
                 for key in ('sample_age_ms', 'receive_age_ms')):
            self._ir_error = 'IR_SAMPLE_STALE_OR_INVALID_AGE'
        else:
            self._ir_error = ''
            self._ir_sample_seq += 1
            self._samples.append(tuple(payload[key] for key in
                                       ('left', 'center', 'right')))
            candidate, counts = self._classify(now)
            confirmed_spray = (
                self._candidate == 'SPRAY_RANGE' and
                self._candidate_count >= self.spray_confirm_samples and
                not self._spray_reentry_locked)
            if candidate == self._candidate:
                self._candidate_count += 1
            else:
                self._candidate = candidate
                self._candidate_count = 1
            if confirmed_spray and candidate in ('APPROACH', 'TOO_FAR'):
                self._spray_reentry_locked = True
                self._spray_reentry_count = 0
            elif candidate in ('UNKNOWN', 'NO_FLAME_SIGNATURE', 'TOO_CLOSE'):
                self._spray_reentry_locked = False
                self._spray_reentry_count = 0
            elif self._spray_reentry_locked:
                if (candidate == 'SPRAY_RANGE' and
                        counts[2] <= self.right_thresholds[1] -
                        self.spray_reentry_margin_counts):
                    self._spray_reentry_count += 1
                    if (self._spray_reentry_count >=
                            self.spray_confirm_samples):
                        self._spray_reentry_locked = False
                        self._spray_reentry_count = 0
                else:
                    self._spray_reentry_count = 0
            return
        self._reset_ir()

    def mark_ir_error(self, now):
        self._ir_received = float(now)
        self._ir_error = 'IR_JSON_INVALID'
        self._reset_ir()

    def update_detection(self, payload, now):
        self._detection = payload
        self._detection_received = float(now)

    def update_alignment(self, payload, now):
        self._alignment = payload
        self._alignment_received = float(now)

    def _vision_presence(self, now):
        """Fresh detection evidence only; alignment still gates movement."""
        payload = self._detection
        at = self._detection_received
        if (not _number(now) or at is None or
                not 0 <= now - at <= self.vision_timeout_s or
                not isinstance(payload, dict) or
                type(payload.get('schema_version')) is not int or
                payload.get('schema_version') != 1 or
                payload.get('observe_only') is not True or
                payload.get('state') != 'ok'):
            return 'UNKNOWN'
        source = payload.get('source')
        fire = payload.get('fire')
        age = source.get('age_ms') if isinstance(source, dict) else None
        if (not _number(age) or age < 0 or
                age + (now - at) * 1000 > self.max_image_age_ms or
                not isinstance(fire, dict) or
                type(fire.get('detected_now')) is not bool or
                type(fire.get('confirmed')) is not bool):
            return 'UNKNOWN'
        if fire['detected_now'] and fire['confirmed']:
            return 'CONFIRMED'
        if not fire['detected_now'] and not fire['confirmed']:
            return 'CLEAR'
        return 'UNCONFIRMED'

    def _classify(self, now=None):
        if not self._samples:
            return 'UNKNOWN', None
        counts = tuple(int(median(sample[index] for sample in self._samples))
                       for index in range(3))
        if self.range_policy == YOLO_RIGHT_ADC_POLICY:
            close_max, spray_max, approach_max = self.right_thresholds
            right = counts[2]
            if right <= close_max:
                return 'TOO_CLOSE', counts
            if right <= spray_max:
                return 'SPRAY_RANGE', counts
            if right <= approach_max:
                return 'APPROACH', counts
            vision = self._vision_presence(now)
            # Seeing a confirmed flame always prevents a no-flame claim.
            # A fresh centered preview and the existing owner/scan/arm gates
            # are still required by motion consumers before approaching.
            if vision == 'CONFIRMED':
                return 'APPROACH', counts
            if vision == 'CLEAR' and all(v > approach_max for v in counts):
                return 'NO_FLAME_SIGNATURE', counts
            return 'UNKNOWN', counts
        differences = tuple(self.baselines[index] - counts[index]
                            for index in range(3))
        if all(abs(value) <= self.baseline_tolerance for value in differences):
            return 'NO_FLAME_SIGNATURE', counts
        if not all(value >= self.min_active_drop for value in differences):
            return 'UNKNOWN', counts
        right = counts[2]
        close_max, spray_max, approach_max = self.right_thresholds
        if right <= close_max:
            return 'TOO_CLOSE', counts
        if right <= spray_max:
            return 'SPRAY_RANGE', counts
        if right <= approach_max:
            return 'APPROACH', counts
        return 'TOO_FAR', counts

    def _vision_blockers(self, now):
        blockers = []
        if self._detection_received is None or (
                now - self._detection_received > self.vision_timeout_s):
            blockers.append('YOLO_MISSING_OR_STALE')
        else:
            payload = self._detection
            if not isinstance(payload, dict) or (
                    payload.get('schema_version') != 1
                    or payload.get('observe_only') is not True
                    or payload.get('state') != 'ok'):
                blockers.append('YOLO_INVALID')
            else:
                fire = payload.get('fire')
                source = payload.get('source')
                if not isinstance(fire, dict):
                    fire = {}
                    blockers.append('YOLO_FIRE_INVALID')
                age = (source.get('age_ms')
                       if isinstance(source, dict) else None)
                if (not _number(age)
                        or abs(float(age)) > self.max_image_age_ms):
                    blockers.append('YOLO_SOURCE_STALE_OR_INVALID')
                if (fire.get('detected_now') is not True
                        or fire.get('confirmed') is not True):
                    blockers.append('FIRE_NOT_CONFIRMED')
        if self._alignment_received is None or (
                now - self._alignment_received > self.vision_timeout_s):
            blockers.append('ALIGNMENT_MISSING_OR_STALE')
        else:
            payload = self._alignment
            if not isinstance(payload, dict) or (
                    payload.get('schema_version') != 1
                    or payload.get('observe_only') is not True
                    or payload.get('state') != 'CENTERED'):
                blockers.append('FIRE_NOT_CENTERED')
            elif (not isinstance(payload.get('safety'), dict)
                    or payload['safety'].get('motion_command_publisher')
                    is not False
                    or payload['safety'].get('actuator_command_publisher')
                    is not False):
                blockers.append('ALIGNMENT_SAFETY_CONTRACT')
        return blockers

    def snapshot(self, now):
        """Return JSON-safe preview; never grant actuation authority."""
        now = float(now)
        candidate, counts = self._classify(now)
        blockers = []
        if self._ir_received is None:
            blockers.append('IR_NOT_RECEIVED')
        elif now - self._ir_received > self.ir_timeout_s:
            blockers.append('IR_TIMEOUT')
            self._reset_ir()
            candidate, counts = 'UNKNOWN', None
        elif self._ir_error:
            blockers.append(self._ir_error)
        if not blockers:
            required_samples = (self.spray_confirm_samples
                                if candidate == 'SPRAY_RANGE'
                                else self.stable_samples)
            if candidate == 'NO_FLAME_SIGNATURE':
                blockers.append('NO_IR_FLAME_SIGNATURE')
            elif candidate == 'UNKNOWN':
                blockers.append('IR_AMBIGUOUS')
            elif self._candidate_count < required_samples:
                blockers.append('IR_BAND_NOT_STABLE')
            elif candidate == 'SPRAY_RANGE' and self._spray_reentry_locked:
                blockers.append('IR_SPRAY_REENTRY_HYSTERESIS')
        blockers.extend(self._vision_blockers(now))
        if self.range_policy == YOLO_RIGHT_ADC_POLICY:
            # A new camera state cannot inherit stability from an old IR
            # classification before any fresh ADC samples in the new state.
            count = self._candidate_count if candidate == self._candidate else 0
            required = (self.spray_confirm_samples if candidate in
                        ('SPRAY_RANGE', 'NO_FLAME_SIGNATURE') else self.stable_samples)
            if count < required and 'IR_BAND_NOT_STABLE' not in blockers:
                blockers.append('IR_BAND_NOT_STABLE')
            if self._vision_presence(now) == 'UNKNOWN':
                if 'YOLO_SOURCE_STALE_OR_INVALID' not in blockers:
                    blockers.append('YOLO_SOURCE_STALE_OR_INVALID')
        else:
            count = self._candidate_count
        zone = candidate if not blockers else 'UNKNOWN'
        result = {
            'schema_version': 1,
            'mode': 'OBSERVE_ONLY',
            'observe_only': True,
            'zone': zone,
            'ir_band': candidate,
            'ir_receive_age_ms': (
                round(max(0.0, now - self._ir_received) * 1000.0, 3)
                if self._ir_received is not None else None),
            'ir_sample_seq': self._ir_sample_seq,
            'stable_samples': count,
            'filtered_counts': dict(zip(('left', 'center', 'right'), counts))
            if counts is not None else None,
            'blockers': blockers,
            'calibration': 'PROVISIONAL_SINGLE_FLAME',
            'validated_for_actuation': False,
            'safety': {
                'motion_command_publisher': False,
                'actuator_command_publisher': False,
            },
        }
        if self.range_policy == YOLO_RIGHT_ADC_POLICY:
            result.update(range_policy=self.range_policy,
                          range_channel='right',
                          right_thresholds=dict(zip(
                              ('too_close_max', 'spray_max', 'approach_max'),
                              self.right_thresholds)),
                          vision_presence=self._vision_presence(now),
                          no_flame_requires_all_eyes=True)
        return result
