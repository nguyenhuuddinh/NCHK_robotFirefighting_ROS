"""Installed pulse state policy inherited from R4d. Returns proposed commands; has no ROS/device I/O."""
from fire_robot_mission.ir_budget_stop import cap_report_valid

from .no_goal_cycle import context_of,same_cycle,stopped_pair,registered_navigation
import copy
import math
from fire_robot_mission.pump_demo_core import zero_twist
from fire_robot_mission.post_spray_review_core import PostSprayReviewCore
from fire_robot_mission.ir_approach_shadow_core import validated_ir_band
from .operational_contract import SERVO_MIN_DEG, SERVO_MAX_DEG
from .stationary_confirmation import StationaryConfirmation, common_errors
from .close_stationary_policy import StationaryTestPumpCore, allowed_bands, respray_context, close_ir_stop_context


def number(value):
    return type(value) in (int, float) and math.isfinite(value)


def uuid_ok(value):
    return isinstance(value, str) and len(value) == 32 and value != '0'*32 and all(c in '0123456789abcdef' for c in value)


class PulseSession:
    """One explicit P request, one pulse, fresh actuator OFF/home, shadow review.

    counts must come from an adapter checking node/name/namespace/type/QoS, never
    raw count_publishers alone. pulse_controller supplies the deployment graph.
    update timestamps are receipt times; source ages remain mandatory in payloads.
    """
    def __init__(self, *, enabled=False, allow_provisional_ir=False, allow_close_stationary=False, pulse_s=5., start_on_ack=False, ir_confirm_samples=5, unbounded_approach=False):
        self.r5_cycle_context=None;self.cycle_id=None
        self.r5_initialization = {k:v for k,v in locals().items() if k not in ('self','__class__')}
        if type(allow_close_stationary) is not bool:raise ValueError('close stationary opt-in must be bool')
        if type(ir_confirm_samples) is not int or ir_confirm_samples not in (3,5):raise ValueError('IR_CONFIRM_SAMPLES_INVALID')
        self.ir_confirm_samples=ir_confirm_samples
        self.unbounded_approach=unbounded_approach is True
        self.allow_close_stationary=allow_close_stationary
        self.pump = StationaryTestPumpCore(self,enabled=enabled, allow_provisional_ir=allow_provisional_ir,pulse_s=pulse_s,start_on_ack=start_on_ack)
        self.review = PostSprayReviewCore(max_pulses=1)
        self.inputs = {}
        self.phase = 'WAIT_P'
        self.used = False
        self.reason = ''
        self.goal = None
        self.last_now = None
        self.last_esp = None
        self.actuator_progress_at = None
        self.start_home = None
        self.on_command_esp_baseline = None
        self.on_ack = False
        self.sweep_ack = False
        self.off_at = self.off_since = None
        self.review_at = None
        self.post_review = None
        self.pulse_core = None
        self.normal_end = False
        self.close_review_since=None
        self.stationary=StationaryConfirmation(required_samples=ir_confirm_samples)
        self.prespray_cancel=None

    def cancel_before_spray(self, request, now):
        from fire_robot_mission.prespray_recovery import request_ok
        if not request_ok(request):return False
        if self.prespray_cancel is not None:
            return self.prespray_cancel == {k:request.get(k) for k in ('request_id','goal_uuid')}
        gw=self.get('gateway',now) or {}
        if not (request_ok(request) and not self.used and self.phase=='WAIT_P'
                and request['goal_uuid']==gw.get('cycle_goal_uuid')
                and gw.get('fire_latched') is True and gw.get('stop_reason')=='FIRE_ACQUIRED'):
            return False
        self.prespray_cancel={k:request[k] for k in ('request_id','goal_uuid')}
        self.phase='CANCELED_PRESPRAY';self.reason='EXPLICIT_CANCEL_BEFORE_P'
        self.pump.update_arm(False,now)
        return True

    def update(self, name, payload, received):
        if name not in {'ir','ir_live','mission','alignment','gateway','guard',
                        'actuator','odom','cmd','status','operator','vision','preview','stationary_request'}:
            raise ValueError('unknown R4d input')
        if not number(received):
            raise ValueError('invalid receipt time')
        self.inputs[name] = (payload, float(received))

    def get(self, name, now, limit=.35):
        payload, at = self.inputs.get(name, (None, None))
        if name == 'ir' and limit == .35:
            limit = .5
        return payload if at is not None and 0 <= now-at <= limit else None

    def actuator(self, now):
        a = self.get('actuator', now)
        if not isinstance(a,dict) or a.get('schema_version') != 1 or a.get('mode') != 'COMMAND_TELEMETRY_ONLY':
            return None
        age = a.get('receive_age_ms')
        if (not number(age) or age < 0 or age + (now-self.inputs['actuator'][1])*1000 > 350
                or type(a.get('esp_ms')) is not int or not 0 <= a['esp_ms'] < 2**32
                or type(a.get('home_command_completed_count')) is not int or a['home_command_completed_count'] < 0
                or type(a.get('pump_pwm_commanded_on')) is not bool
                or type(a.get('servo_pwm_commanded_on')) is not bool
                or a.get('pump_pwm_ready') is not True):
            return None
        return a

    @staticmethod
    def idle(a):
        return bool(a and a['pump_pwm_commanded_on'] is False
                    and a['servo_pwm_commanded_on'] is False and a.get('servo_phase') == 'IDLE')

    def bind_ok(self, now):
        a=self.get('alignment',now);g=self.get('guard',now);gw=self.get('gateway',now);ir=self.get('ir_live',now)
        context=context_of(self)
        if context is not None:
            parts=(a,g,gw) if self.stationary.state=='COMPLETE' else (a,g,gw,ir)
            return (all(same_cycle(v,context) for v in parts)
                and a.get('cycle_id')==context['cycle_id']
                and (self.stationary.state=='COMPLETE' or ir.get('cycle_id')==context['cycle_id'])
                and (self.cycle_id is None or self.cycle_id==context['cycle_id']))
        if self.stationary.state=='COMPLETE':
            return bool(all(isinstance(v,dict) for v in (a,g,gw))
                and a.get('goal_uuid')==g.get('goal_uuid')==gw.get('cycle_goal_uuid')==self.stationary.goal
                and (self.goal is None or self.goal==self.stationary.goal))
        return bool(all(isinstance(v,dict) for v in (a,g,gw,ir)) and uuid_ok(a.get('goal_uuid'))
                    and a['goal_uuid'] == g.get('goal_uuid') == gw.get('cycle_goal_uuid') == ir.get('goal_uuid')
                    and (self.goal is None or self.goal == a['goal_uuid']))

    def extra_blockers(self, now, counts, *, require_idle):
        errors=[];ir=self.get('ir',now);live=self.get('ir_live',now);a=self.actuator(now)
        if counts.get('actuator') != 1:errors.append('ACTUATOR_OWNER_INVALID')
        if counts.get('ir_live') != 1:errors.append('IR_LIVE_OWNER_INVALID')
        if self.committed():
            # The existing I/IR proof admitted ON; it is not a renewable
            # perception lease during water spray. Preserve real owner/STOP,
            # cycle, actuator and explicit-I heartbeat checks instead.
            if not (isinstance(live,dict) and live.get('schema_version')==1
                    and live.get('publisher_owned') is False
                    and live.get('command')=={'linear_x_m_s':0.,'angular_z_rad_s':0.}):
                errors.append('IR_HOLD_OWNER_NOT_RELEASED')
            if not self.bind_ok(now):errors.append('R4D_CROSS_PHASE_UUID_MISMATCH')
            if self.stationary.state=='COMPLETE':
                request=self.get('stationary_request',now) or {}
                if (counts.get('stationary_request')!=1 or request.get('active') is not True
                        or request.get('request_id')!=self.stationary.token):
                    errors.append('STATIONARY_REQUEST_CHANGED')
            if a is None:errors.append('ACTUATOR_NOT_FRESH_READY')
            elif a.get('servo_phase')=='LOCKED':errors.append('SERVO_FAULT')
            return errors
        bands=allowed_bands(self,now) if self.stationary.state=='COMPLETE' or respray_context(self,now) or close_ir_stop_context(self,now) else ('SPRAY_RANGE',)
        if (validated_ir_band(ir) not in bands or ir.get('zone') != ir.get('ir_band')
                or ir.get('blockers') != [] or ir.get('stable_samples',0) < self.ir_confirm_samples
                or type(ir.get('ir_sample_seq')) is not int or ir['ir_sample_seq'] <= 0
                or ir['ir_receive_age_ms'] + (now-self.inputs['ir'][1])*1000 > 500 + 1e-6):
            errors.append('R4D_SPRAY_RANGE_NOT_CONFIRMED')
        if self.stationary.state=='COMPLETE':
            errors+=common_errors(self,now,counts,require_idle=require_idle)
            request=self.get('stationary_request',now) or {}
            if request.get('request_id')!=self.stationary.token:errors.append('STATIONARY_REQUEST_CHANGED')
        elif not (isinstance(live,dict) and live.get('state') == 'COMPLETE'
                and (live.get('stop_reason') == 'SPRAY_RANGE_REACHED' or close_ir_stop_context(self,now))
                and live.get('enabled') is True and live.get('integrated_handoff') is True
                and live.get('continuous_approach') is True and live.get('publisher_owned') is False
                and live.get('blockers') == [] and live.get('recovery_required') is False
                and live.get('command') == {'linear_x_m_s':0.,'angular_z_rad_s':0.}):
            errors.append('R4C_SPRAY_STOP_PROOF_MISSING')
        if isinstance(live,dict) and (live.get('forward_motion_inhibited') is True or
                live.get('approach_budget_exhausted') is True) and not cap_report_valid(live):
            errors.append('R4D_CAP_STOP_CONTRACT_INVALID')
        if not self.bind_ok(now):errors.append('R4D_CROSS_PHASE_UUID_MISMATCH')
        if a is None:errors.append('ACTUATOR_NOT_FRESH_READY')
        elif a.get('servo_phase') == 'LOCKED':errors.append('SERVO_FAULT')
        elif require_idle and not self.idle(a):errors.append('ACTUATOR_NOT_FRESH_OFF')
        return errors

    def committed(self):
        return self.phase=='FIRING' and self.pump.committed()

    def feed_pump(self, now):
        for name in ('ir','mission','alignment','gateway','guard'):
            if name in self.inputs:self.pump.update(name,*self.inputs[name])
        for name,method in [('odom',self.pump.update_odom),('cmd',self.pump.update_cmd),('status',self.pump.update_statuses)]:
            if name in self.inputs:method(*self.inputs[name])
        live=self.get('operator',now,.4) is True
        self.pump.update_arm(live and self.phase in ('ARMING','FIRING'),now)
        return live

    def stop(self, now, reason, *, normal=False):
        self.reason=reason;self.normal_end=normal;self.off_at=now;self.off_since=None;self.phase='VERIFY_OFF'
        self.pump.update_arm(False,now)

    def step(self, now, counts, *, request=False, stop=False):
        if not number(now) or (self.last_now is not None and now < self.last_now):
            self.phase='ABORTED';self.reason='CLOCK_INVALID';return self.report(False,[])
        self.last_now=now
        if self.phase in ('DONE','ABORTED','CANCELED_PRESPRAY'):return self.report(False,[])
        operator_live=self.feed_pump(now)
        self.stationary.step(self,now,counts,stop=stop)
        a=self.actuator(now)
        if a is not None and self.last_esp is not None and ((a['esp_ms']-self.last_esp) % 2**32) >= 2**31:
            if self.used:self.stop(now,'ACTUATOR_CLOCK_REGRESSION')
        if a is not None:
            if self.last_esp is None or 0<(a['esp_ms']-self.last_esp)%2**32<2**31:
                self.actuator_progress_at=now
            self.last_esp=a['esp_ms']
        if (self.phase=='FIRING' and self.pump.start_on_ack and self.on_ack
                and self.pump.pulse_elapsed(now)<self.pump.pulse_s):
            # This is actuator telemetry, not flame tracking: a repeated MCU
            # sample must not be refreshed by local JSON receipt timestamps.
            # Normal 5 Hz reports may repeat between samples for <=350 ms.
            if (self.actuator_progress_at is None or now-self.actuator_progress_at>.35):
                self.stop(now,'ACTUATOR_PROGRESS_STALE')
            elif a is not None and a['pump_pwm_commanded_on'] is False:
                self.stop(now,'PUMP_PWM_OFF_DURING_PULSE')
        if stop and self.phase=='WAIT_P':
            self.phase='ABORTED';self.reason='OPERATOR_STOP';return self.report(False,[])
        if stop and self.phase in ('ARMING','FIRING'):self.stop(now,'OPERATOR_STOP')
        proposed=False;blockers=[]
        if self.phase=='WAIT_P':
            blockers=self.pump.blockers(now,counts)+self.extra_blockers(now,counts,require_idle=True)
            blockers=[s for s in blockers if s!='OPERATOR_ARM_MISSING_OR_STALE']
            if not operator_live:blockers.append('OPERATOR_HEARTBEAT_MISSING')
            # Early P is discarded, never saved for later readiness.
            if request and not blockers and not self.used:
                self.used=True;self.goal=self.get('alignment',now)['goal_uuid']
                if context_of(self) is not None:self.cycle_id=context_of(self)['cycle_id']
                self.start_home=a['home_command_completed_count'];self.phase='ARMING'
                self.pump.update_arm(True,now)
        if self.phase in ('ARMING','FIRING'):
            blockers=self.pump.blockers(now,counts)+self.extra_blockers(now,counts,require_idle=self.phase=='ARMING')
            if blockers:
                self.stop(now,blockers[0])
            else:
                # ACK must be a fresh report observed after this pulse's first
                # ON command. Repeated reports never restart the six seconds.
                if (self.pump.on_since is not None and a and a['pump_pwm_commanded_on']
                        and self.inputs['actuator'][1]>=self.pump.on_since
                        and self.on_command_esp_baseline is not None
                        and 0<(a['esp_ms']-self.on_command_esp_baseline)%2**32<2**31):
                    self.pump.acknowledge_on(self.inputs['actuator'][1],a['esp_ms'])
                proposed,pump_report=self.pump.step(now,counts)
                self.pulse_core=copy.deepcopy(pump_report)
                self.review.update('pump',pump_report,now)
                if proposed:
                    if self.on_command_esp_baseline is None:self.on_command_esp_baseline=a['esp_ms']
                    self.phase='FIRING'
                    if a['pump_pwm_commanded_on'] and self.pump.ack_on_since is not None:
                        self.on_ack=True
                        angle=a.get('servo_commanded_angle_deg')
                        if a.get('servo_phase')!='SWEEPING' or a['servo_pwm_commanded_on'] is not True or not number(angle) or not SERVO_MIN_DEG<=angle<=SERVO_MAX_DEG:
                            self.stop(now,'PUMP_SERVO_FEEDBACK_MISMATCH');proposed=False
                        else:self.sweep_ack=True
                    if not self.on_ack and now-self.pump.on_since > .5:
                        self.stop(now,'PUMP_ON_NOT_ACKNOWLEDGED');proposed=False
                    else:self.review.step(now)
                elif self.pump.state in ('COMPLETE','ABORTED'):
                    self.stop(now,self.pump.stop_reason,normal=self.pump.state=='COMPLETE')
        if self.phase=='VERIFY_OFF':
            proposed=False
            # Publish the real core after withdrawing its arm. Retaining the
            # earlier FIRING snapshot makes downstream review wait forever.
            _,pump_report=self.pump.step(now,counts)
            self.pulse_core=copy.deepcopy(pump_report)
            if a and self.idle(a):
                if self.off_since is None:self.off_since=now
                if now-self.off_since >= .4:
                    if not self.normal_end:self.phase='ABORTED'
                    elif not self.on_ack or not self.sweep_ack:self.phase='ABORTED';self.reason='ACTUATION_NOT_ACKNOWLEDGED'
                    elif a['home_command_completed_count'] > self.start_home:
                        self.phase='REVIEW';self.review_at=now
            else:self.off_since=None
            if a and a.get('servo_phase')=='LOCKED' and not a['pump_pwm_commanded_on']:
                self.phase='ABORTED';self.reason='SERVO_FAULT_OFF_OBSERVED'
            if self.phase=='VERIFY_OFF' and now-self.off_at > 3:
                self.phase='ABORTED';self.reason='OFF_OR_HOME_NOT_CONFIRMED'
        if self.phase=='REVIEW':
            # Post-review may classify fire loss, realign, approach or respray.
            # It can never restart this session's pulse.
            statuses=self.get('status',now,float('inf'))
            status_ok=isinstance(statuses,(list,tuple)) and bool(statuses) and all(
                isinstance(item,(list,tuple)) and len(item)==4 and item[0]==self.goal and item[1]==5
                for item in statuses)
            if self.r5_cycle_context is not None:
                rows=[(x[0],x[1]) for x in statuses or () if isinstance(x,(tuple,list)) and len(x)==4]
                status_ok=(statuses is not None and len(rows)==len(statuses)
                    and registered_navigation(self,rows,self.inputs['status'][1],now,self.goal))
            if (counts.get('raw') != 0 or any(counts.get(k) != 1 for k in
                    ('ir','ir_live','mission','alignment','gateway','guard','actuator','odom','cmd','status','arm','pump','pump_subscribers','vision','preview')) or not status_ok
                    or not self.idle(a) or not self.bind_ok(now)
                    or not zero_twist(self.get('cmd',now),.001)
                    or not zero_twist(self.get('odom',now),.02)):
                self.phase='ABORTED';self.reason='POST_REVIEW_STOP_PROOF_LOST'
            else:
                self.review.update('pump',{'schema_version':1,'mode':'SUPERVISED_DEMO','one_shot':True,
                    'state':'COMPLETE','goal_uuid':self.goal,'pump_on':False,'stop_reason':'PULSE_TIME_LIMIT'},now)
                for target,source in [('vision','vision'),('ir','ir'),('alignment','preview'),('mission','mission')]:
                    if source in self.inputs:self.review.update(target,*self.inputs[source])
                post=self.review.step(now);self.post_review=post
                if (self.allow_close_stationary and self.stationary.state=='COMPLETE' and self.stationary.band=='TOO_CLOSE'
                        and post['state']=='REVIEWING' and post['blockers']==['RANGE_UNCERTAIN_OR_TOO_CLOSE']
                        and not self.extra_blockers(now,counts,require_idle=True)):
                    # Preserve the actual REVIEWING state; a supervised near-range
                    # pump test is complete, not a claim of correct spray distance.
                    if self.close_review_since is None:self.close_review_since=now
                    if now-self.close_review_since>=1:
                        self.phase='DONE';self.reason='CLOSE_RANGE_TEST_OFF_HOME'
                elif post['state'] in ('NO_FIRE_EVIDENCE','RETRY_LIMIT_REACHED','REALIGN_REQUIRED','APPROACH_REQUIRED'):
                    self.phase='DONE';self.reason=post['state']
                else:
                    self.close_review_since=None
                if self.phase=='REVIEW' and now-self.review_at > 6:self.phase='ABORTED';self.reason='POST_REVIEW_INCOMPLETE'
        return self.report(proposed and self.phase=='FIRING',blockers)

    def report(self, proposed, blockers):
        return {'scope':'OFFLINE_CANDIDATE_NOT_ROBOT_COMMAND','state':self.phase,
                'proposed_pump_on':bool(proposed),'reason':self.reason,'goal_uuid':self.goal,
                'cycle_id':self.cycle_id,'r5_cycle_context':self.r5_cycle_context,
                'request_used':self.used,'blockers':sorted(set(blockers)),
                'prespray_cancel':self.prespray_cancel,
                'pump_on_feedback_seen':self.on_ack,'servo_sweep_feedback_seen':self.sweep_ack,
                'physical_pump_or_nozzle_verified':False,'automatic_respray':False,'post_review':self.post_review,
                'pulse_core':copy.deepcopy(self.pulse_core),
                'perception_policy':'REVIEW_AFTER_OFF_HOME' if self.committed() else 'REQUIRED_FOR_ADMISSION',
                'pulse_clock':'FIRST_FRESH_ESP_PWM_ON_ACK' if self.pump.start_on_ack else 'FIRST_HOST_ON_COMMAND',
                'pulse_on_ack_at':self.pump.ack_on_since,'pulse_on_ack_esp_ms':self.pump.ack_esp_ms,
                'pulse_remaining_s':(max(0.,self.pump.pulse_s-self.pump.pulse_elapsed(self.last_now))
                    if self.committed() and number(self.last_now) and self.pump.pulse_elapsed(self.last_now) is not None else None),
                'stationary_confirmation':self.stationary.report(),
                'close_stationary_test_enabled':self.allow_close_stationary}
