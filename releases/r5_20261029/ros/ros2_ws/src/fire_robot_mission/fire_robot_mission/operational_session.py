"""Persistent R5 operator observer and GUI with opt-in intent execution.

The executor can publish Bool arms and String control; it defaults OFF.
There are no Twist/pump command publishers, action clients, process launch or
SSH here. BOOT/current-connection identity and handback proposals are connected.
Live intent/control defaults remain OFF; no-goal proof and transient identity
holds are integrated offline. Full adapter/DDS/startup/teardown still block deployment.
"""
import json
import signal
import time

import rclpy
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import ReliabilityPolicy as RP
from action_msgs.msg import GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import CompressedImage
from std_msgs.msg import String, Bool

from .mission_supervisor import sensor_qos, action_status_qos
from .operational_contract import OPERATOR_NODE, PULSE_STATE_TOPIC, PULSE_NODE, exact_endpoint
from .operational_graph import SessionGraph
from .operational_session_core import OperationalSessionCore
from .pulse_transport import six
from .operational_identity import IDENTITY_TOPIC
from .operational_handback import OperationalHandback
from .operational_executor import OperationalExecutor
from .operational_intents import IntentMapper
from .operational_stop import StopVerification
from .operational_frame import FrameProof,IMAGE_TOPIC


JSON_INPUTS = {
    'vision': ('/yolo/detections', 'yolo_observer'),
    'preview': ('/yolo/alignment_preview', 'fire_alignment_observer'),
    'mission': ('/fire_mission/state', 'fire_mission_supervisor'),
    'gateway': ('/fire_mission/live_nav_stop_state', 'live_nav_stop_gateway'),
    'guard': ('/fire_mission/nav2_cancel_guard_state', 'nav2_cancel_guard'),
    'alignment': ('/fire_mission/live_alignment_state', 'live_alignment_takeover'),
    'ir_live': ('/fire_mission/ir_approach_live_state', 'ir_approach_live'),
    'ir': ('/fire_mission/ir_range_preview', 'ir_range_preview'),
    'ir_raw': ('/fire_sensors/ir_raw', 'serial_bridge_node'),
    'actuator': ('/fire_sensors/actuator_state', 'serial_bridge_node'),
    'identity': (IDENTITY_TOPIC, 'serial_bridge_node'),
    'review': ('/fire_mission/post_spray_review','post_spray_review'),
}
STATE_TOPIC = '/fire_mission/operator_state'
OPERATOR_TICK_PERIOD_S = .05
CALLBACK_MIN_WINDOW_S = .01
EXPECTED = [(topic, name, 'std_msgs/msg/String', RP.BEST_EFFORT)
            for topic, name in JSON_INPUTS.values()] + [
    (PULSE_STATE_TOPIC, PULSE_NODE, 'std_msgs/msg/String', RP.BEST_EFFORT),
    ('/cmd_vel', 'safety_watchdog', 'geometry_msgs/msg/Twist', RP.BEST_EFFORT),
    ('/odom', 'serial_bridge_node', 'nav_msgs/msg/Odometry', RP.BEST_EFFORT),
    ('/navigate_to_pose/_action/status', 'bt_navigator', 'action_msgs/msg/GoalStatusArray', RP.RELIABLE),
    (IMAGE_TOPIC,'yolo_observer','sensor_msgs/msg/CompressedImage',RP.BEST_EFFORT),
]


def stationary_proof_snapshot(core, report, now):
    """Small diagnostic only; never participates in actuator admission."""
    if report.get('phase') not in ('IR_CONFIRM', 'WAIT_P'):
        return None
    actor = core.get('actor', now)
    actor = actor if isinstance(actor, dict) else {}
    confirmation = actor.get('stationary_confirmation')
    confirmation = confirmation if isinstance(confirmation, dict) else {}
    ir = core.get('ir', now)
    ir = ir if isinstance(ir, dict) else {}
    actor_at = core.inputs.get('actor', (None, None))[1]
    blockers = actor.get('blockers')
    blockers = blockers if isinstance(blockers, list) else []
    ir_blockers = ir.get('blockers')
    ir_blockers = ir_blockers if isinstance(ir_blockers, list) else []
    return dict(phase=report['phase'], state=confirmation.get('state'),
        reason=confirmation.get('reason'), sample_count=confirmation.get('sample_count'),
        request_active=report.get('intents', {}).get('stationary_confirmation') is True,
        ir_band=ir.get('ir_band'), ir_sample_seq=ir.get('ir_sample_seq'),
        ir_source_age_ms=report.get('ir_source_age_ms'),
        actor_receive_age_ms=round((now-actor_at)*1000, 1) if actor_at is not None else None,
        first_ir_blocker=ir_blockers[0] if ir_blockers else None,
        first_actor_blocker=blockers[0] if blockers else None,
        p_admission_blocker=report.get('p_admission_blocker'))


class OperationalSessionNode(Node):
    def __init__(self):
        super().__init__(OPERATOR_NODE)
        self.declare_parameter('automatic_mode', False)
        self.declare_parameter('automatic_start_file', '')
        self.declare_parameter('automatic_runner_session', '')
        self.declare_parameter('automatic_packet', '')
        from .automatic_policy import AutomaticPolicy,AutomaticStartGate
        self.declare_parameter('alignment_task_timeout_s',0.)
        self.declare_parameter('target_recovery_timeout_s',3.)
        self.declare_parameter('no_goal_handoff_timeout_s',8.)
        self.declare_parameter('ir_final_confirm_timeout_s',5.)
        self.automatic=AutomaticPolicy(self.get_parameter('automatic_mode').value,
            alignment_timeout_s=self.get_parameter('alignment_task_timeout_s').value,
            target_recovery_s=self.get_parameter('target_recovery_timeout_s').value,
            no_goal_handoff_timeout_s=self.get_parameter('no_goal_handoff_timeout_s').value,
            ir_final_confirm_s=self.get_parameter('ir_final_confirm_timeout_s').value)
        self.automatic_start=None
        if self.automatic.enabled:
            self.automatic_start=AutomaticStartGate(self.get_parameter('automatic_start_file').value,
                self.get_parameter('automatic_runner_session').value,self.get_parameter('automatic_packet').value)
        self.declare_parameter('show_gui', False)
        self.declare_parameter('allow_close_stationary', False)
        self.declare_parameter('unbounded_approach', False)
        self.declare_parameter('ir_confirm_samples', 5)
        self.declare_parameter('require_fire_clear_on_start', False)
        self.declare_parameter('recovery_require_ir_clear', False)
        self.view = None
        self.core = OperationalSessionCore(allow_close_stationary=self.get_parameter('allow_close_stationary').value,require_boot_identity=True,
            unbounded_approach=self.get_parameter('unbounded_approach').value,ir_confirm_samples=self.get_parameter('ir_confirm_samples').value)
        if self.get_parameter('require_fire_clear_on_start').value is True:
            self.core.retry_fire_clear_required=True
            self.core.recovery_clear_policy=True
            self.core.recovery_require_ir_clear=self.get_parameter('recovery_require_ir_clear').value is True
            self.core.interruption_reason='RUNTIME_RECOVERY_ABORTED_OLD_ATTEMPT'
        self.handback=OperationalHandback()
        self.graph = SessionGraph()
        self.last_report = None
        self.last_phase = None
        self.last_graph_signature=None
        self.last_arm_log_at = None
        self.last_arm_ready = False
        self.last_stationary_log_at = None
        self.last_key_feedback = None
        self.intent_executor=OperationalExecutor(self)
        if self.automatic.enabled and not self.intent_executor.enabled:
            raise ValueError('AUTOMATIC_REQUIRES_LIVE_PROFILE')
        self.stop_verification=StopVerification();self.stop_pins={};self.stop_source_fault=None
        self.frame_proof=FrameProof();self.frame_warning=None
        from .runtime_rebind import RuntimeRebindGate
        self.runtime_rebind=RuntimeRebindGate(self.automatic_start) if self.automatic_start else None
        self.actor_report = None
        self.pump_request_observed = None
        for topic in [t[0] for t in EXPECTED]+[STATE_TOPIC, PULSE_STATE_TOPIC, '/pump_cmd']:
            if self.resolve_topic_name(topic) != topic:
                raise ValueError('OPERATOR_TOPIC_REMAP_FORBIDDEN')
        self.state_qos = sensor_qos()
        self.state_pub = self.create_publisher(String, STATE_TOPIC, self.state_qos)
        for name, (topic, _) in JSON_INPUTS.items():
            self.create_subscription(String, topic, lambda msg, key=name: self.on_json(key, msg), self.state_qos)
        self.create_subscription(Twist, '/cmd_vel', lambda msg: self.core.update('cmd', six(msg), time.monotonic()), self.state_qos)
        self.create_subscription(Odometry, '/odom', self.on_odom, self.state_qos)
        self.create_subscription(GoalStatusArray, '/navigate_to_pose/_action/status', self.on_status, action_status_qos())
        # Preserve the two explicit pump readers expected by the installed actor.
        self.create_subscription(Bool, '/pump_cmd', self.on_pump_request,
                                 rclpy.qos.QoSProfile(depth=10, reliability=RP.RELIABLE))
        self.create_subscription(String, PULSE_STATE_TOPIC, self.on_actor, self.state_qos)
        self.create_subscription(CompressedImage,IMAGE_TOPIC,self.on_image,self.state_qos)
        if self.get_parameter('show_gui').value is True:
            from .operational_view import SessionView
            self.view = SessionView(automatic=self.automatic.enabled)
        mode = 'ENABLED (explicit Gate2 opt-in)' if self.intent_executor.enabled else 'OFF'
        self.get_logger().warning('R5 candidate operator: live intents '+mode+'; integration acceptance pending')

    def on_json(self, key, msg):
        try:
            value = json.loads(msg.data)
        except (ValueError, TypeError):
            value = None
        self.core.update(key, value if isinstance(value, dict) else None, time.monotonic())

    def on_status(self, msg):
        try:
            rows = [(bytes(x.goal_info.goal_id.uuid).hex(), x.status) for x in msg.status_list]
        except (TypeError, ValueError, AttributeError):
            rows = None
        self.core.update('status', rows, time.monotonic())

    def on_odom(self, msg):
        from .operational_cycle import finite
        q = msg.pose.pose.orientation
        age = (self.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        valid = (msg.header.frame_id == 'odom' and msg.child_frame_id == 'base_link'
                 and all(finite(v) for v in (q.x,q.y,q.z,q.w))
                 and .9 <= q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w <= 1.1 and -.05 <= age <= .3)
        self.core.update('odom', six(msg.twist.twist) if valid else None, time.monotonic())

    def on_actor(self, msg):
        try:
            value = json.loads(msg.data)
        except (TypeError, ValueError):
            value = None
        self.actor_report = (value, time.monotonic())
        self.core.update('actor',value,time.monotonic())

    def on_pump_request(self, msg):
        self.pump_request_observed = (msg.data is True, time.monotonic())

    def on_image(self,msg):
        now=time.monotonic()
        frame=self.frame_proof.observe(msg,now,self.get_clock().now().nanoseconds)
        if frame is not None and self.view is not None:
            self.view.image,self.view.image_at=frame,now
        if frame is None and self.frame_warning!=self.frame_proof.reason:
            self.get_logger().warning('R5 camera prearm: '+self.frame_proof.reason)
        self.frame_warning=self.frame_proof.reason

    def recover_runtime(self, now, key):
        """Retire the interrupted attempt, preserving this Node, GUI and Nav2.

        Runner already drained the old mission; a new core cannot emit stages
        until ordinary three-second prearm and six-participant proofs return.
        Never carry a pending I/P, active fire cycle or old publisher lease.
        """
        gate=self.runtime_rebind
        if gate is None or key in ('Q','ESC') or self.core.closed:return
        d=gate.observe()
        if key=='X' and gate.held:
            status=self.core._status(now)
            if not gate.cancel_requested:
                gate.cancel_uuid=status.active_uuid if status is not None else None
            gate.cancel_requested=True;gate.cancel_at=now
        if d is None:return
        if d['phase']=='HOLD':
            self.core.stop_fault('RUNTIME_RECOVERY_HOLD:'+d['reason'])
            self.intent_executor.hold_stopped()
            return
        from .operational_identity import identity_current,identity_key
        boot,received=self.core.inputs.get('identity',(None,None))
        recoverable=('BOOT_OR_SERIAL_EPOCH_CHANGED',
            'INPUT_GID_CHANGED:/navigate_to_pose/_action/status',
            'LIVE_EXECUTOR_PROOF_LOST:INPUT_GID_CHANGED:/navigate_to_pose/_action/status',
            'LIVE_EXECUTOR_PROOF_LOST:WAIT_EXACT_OPERATIONAL_GRAPH','ACTUATOR_LOCKED')
        if (self.core.fault not in recoverable
                and not str(self.core.fault).startswith('RUNTIME_RECOVERY_HOLD:')):return
        from .actuator_freshness import fresh_motion_off
        from .operational_cycle import zero
        nav_only=d.get('navigation_only') is True
        a,aat=self.core.inputs.get('actuator',(None,None))
        nav_rest=(nav_only and fresh_motion_off(a,aat,now)
            and zero(self.core.get('cmd',now)) and zero(self.core.get('odom',now),.02,.05)
            and not self.get_publishers_info_by_topic('/cmd_vel_raw'))
        if (not identity_current(boot,received,now) or list(identity_key(boot) or ())!=d.get('identity_key')
                or not (nav_rest or self.core._rest(now)) or self.get_publishers_info_by_topic('/pump_cmd')):return
        old=self.core
        self.core=OperationalSessionCore(allow_close_stationary=old.allow_close,require_boot_identity=True,
            unbounded_approach=old.unbounded_approach,ir_confirm_samples=old.ir_confirm_samples)
        # External streams remain pinned by the persistent graph below. Old
        # peer reports/control receipts are not copied into the new attempt.
        self.core.inputs={k:v for k,v in old.inputs.items() if k in
            ('identity','actuator','ir_raw','vision','preview','cmd','odom','status')}
        # Before any pulse, current fire can start a fresh, fully validated
        # attempt immediately after rearm. An uncertain spray is never replayed.
        self.core.navigation_only=nav_only
        self.core.navigation_home_before=d.get('home_before')
        self.core.retry_fire_clear_required=d['after_pulse'] or gate.cancel_requested
        self.core.recovery_clear_policy=d['after_pulse']
        if gate.cancel_requested:
            current=self.core._status(now)
            # Cancel only the exact goal present when the user pressed X.
            # Never replace it with a different goal that arrived during HOLD.
            if current is not None and current.active_uuid==gate.cancel_uuid:
                self.core.navigation_cancel_uuid=gate.cancel_uuid
        self.core.recovery_require_ir_clear=d['after_pulse']
        self.core.interruption_reason='RUNTIME_RECOVERY_ABORTED_OLD_ATTEMPT'
        self.handback=OperationalHandback()
        # Drop only replaced mission endpoint pins. Keep external/Nav2 pins.
        peer_topics={topic for name,(topic,_) in JSON_INPUTS.items()
            if name in ('mission','gateway','guard','alignment','ir_live','review')}
        peer_topics.add(PULSE_STATE_TOPIC)
        for topic in peer_topics:self.graph.pins.pop(topic,None)
        self.graph.fault=None;self.graph.raw_pin=None
        from .operational_discovery import Discovery
        self.graph.discovery=Discovery();self.graph.discovery_cycle=None
        self.intent_executor.mapper=IntentMapper()
        from .automatic_policy import AutomaticPolicy
        self.automatic=AutomaticPolicy(True,alignment_timeout_s=self.automatic.stage_timeout_s,
            target_recovery_s=self.automatic.target_wait_s,ir_final_confirm_s=self.automatic.final_confirm_s,
            no_goal_handoff_timeout_s=self.automatic.no_goal_handoff_timeout_s)
        self.last_phase=self.last_graph_signature=None;self.actor_report=None
        gate.complete()
        self.get_logger().info('RUNTIME_REBIND '+json.dumps(gate.report(),allow_nan=False))

    def tick(self, key=None):
        now = time.monotonic()
        if self.stop_verification.start is not None:return self.stop_tick(now)
        self.recover_runtime(now,key)
        graph=self.graph.check(self,self.core,now,EXPECTED,RP.RELIABLE)
        self.core.update('graph',graph,now)
        self.core.graph_hold=graph.get('nav_owner_pending') is True
        signature=(graph['valid'],graph['fault'],tuple(graph['missing']),graph['discovery']['status'],
            graph['discovery']['reason'],tuple((x['node'],x['gid'])for x in graph['discovery']['endpoints']))
        if signature!=self.last_graph_signature:
            self.get_logger().info('OPERATOR_GRAPH '+json.dumps(graph,allow_nan=False))
            self.last_graph_signature=signature
        gateway=self.core.get('gateway',now) or {}
        if self.core.armed and gateway.get('stop_reason')=='NAV_OWNER_CONFIRMATION_TIMEOUT':
            self.core.stop_fault('NAV_OWNER_CONFIRMATION_TIMEOUT')
        self.intent_executor.poll(now)
        readiness=(IntentMapper.readiness(self.core,now,safety_ok=self.intent_executor.safety_ok(now))
                   if self.intent_executor.enabled else [])
        if not self.core.armed and not self.frame_proof.fresh(now):readiness.append('WAIT_FIRST_OR_FRESH_DECODED_CAMERA_FRAME')
        if readiness:
            if self.core.armed and graph.get('fault'):self.core.stop_fault('LIVE_EXECUTOR_PROOF_LOST:'+str(graph['fault']))
            else:self.core.clear_prearm()
        recoverable=[reason for reason in readiness if reason not in ('WAIT_CURRENT_BOOT_IDENTITY','WAIT_NAV_RAW_OWNER_NAME')]
        loop_gap=(self.core.last_now is not None and now-self.core.last_now>.35)
        if loop_gap and self.core.armed:
            recoverable.append('OPERATOR_CALLBACK_GAP')
            # Missed heartbeats may have already stopped a real consumer.
            # Retire that fire attempt; never replay its stage/P after a gap.
            self.core.interrupt_cycle('OPERATOR_CALLBACK_GAP')
        self.core.observe_input_hold(now,recoverable)
        report = self.core.step(now, [key] if key and not (key=='A' and readiness) else ())
        if key=='X' and self.runtime_rebind and self.runtime_rebind.cancel_at==now:
            self.core.event_index+=1
            self.core.feedback=dict(index=self.core.event_index,key='X',accepted=True,
                reason='CANCEL_QUEUED_WAIT_CURRENT_STOP_RECOVERY',at=now)
        if key=='A' and readiness:
            self.core.event_index+=1
            self.core.feedback=dict(index=self.core.event_index,key='A',accepted=False,reason=self.core.fault or readiness[0],at=now)
            if self.core.fault is None and not self.core.armed:self.core.reason=readiness[0]
        auto_ready=self.automatic_start.ready() if self.automatic_start else False
        if self.automatic_start and self.automatic_start.seen and not auto_ready:
            self.core.stop_fault('AUTOMATIC_START_PROOF_LOST')
        # Q/X and window close take precedence; automatic execution never
        # supplies stage requests while shutdown, cancel or hard fault holds.
        if key is None:
            intent=self.automatic.next_intent(self.core,now,start_ready=auto_ready,blockers=readiness)
            if intent:
                report=self.core.apply_intent(intent,now,origin='AUTOMATIC')
                self.automatic.issued(intent,now,report['operator_feedback'])
                self.get_logger().info('AUTOMATIC_INTENT '+json.dumps(dict(intent=intent,
                    feedback=report['operator_feedback'],cycle=report['cycle']),allow_nan=False))
        handback=self.handback.step(self.core,now,graph_ok=self.graph.fault is None
            and self.core.get('graph',now).get('valid') is True,
            raw_count=len(self.get_publishers_info_by_topic('/cmd_vel_raw')),
            pump_count=len(self.get_publishers_info_by_topic('/pump_cmd')))
        report=self.core.report(now)
        report['raw_owner_proof']=dict(valid=graph['valid'] and graph['discovery']['status']=='EXACT',
            count=graph['raw_count'],owner=graph['raw_owner'],
            gid=(graph['discovery']['endpoints'][0]['gid'] if graph['raw_count']==1 else None))
        report['automatic']=self.automatic.report(auto_ready)
        report['runtime_rebind']=self.runtime_rebind.report() if self.runtime_rebind else None
        report['automatic']['max_pulses_per_cycle']=(report.get('spray_sequence') or {}).get('max_pulses',1)
        report['handback']=handback
        execution=self.intent_executor.step(self.core,report,handback,now)
        report['execution']=execution
        report['live_intents_published']=execution.get('live_intents_published') is True
        report['live_intents_active']=execution.get('live_intents_active') is True
        report['integration_blockers'] = ['DDS_RUNTIME_ACCEPTANCE_PENDING','STARTUP_TEARDOWN_INTEGRATION_PENDING']
        report['camera_frame']=self.frame_proof.report(now)
        report['stationary_proof']=stationary_proof_snapshot(self.core,report,now)
        proof = report['startup_readiness']
        proof['adapter_blockers'] = list(readiness)
        proof['required_stable_s'] = 3.
        proof['ready'] = (not self.core.armed and self.core.fault is None
                          and not self.core.closed and not readiness
                          and not proof['blockers'] and proof['stable_s'] >= 3.)
        if not self.intent_executor.enabled:report['integration_blockers']+=['LIVE_INTENT_EXECUTOR_DISABLED','HANDSHAKE_CONTROL_PROPOSAL_ONLY']
        if report['closed']:
            self.request_shutdown(key or 'Q',now=now)
            return self.stop_tick(now)
        self.last_report = report
        self.state_pub.publish(String(data=json.dumps(report, allow_nan=False, separators=(',',':'))))
        self.log_arm_proof(report, now, key)
        if report['stationary_proof'] is None:
            self.last_stationary_log_at=None
        elif self.last_stationary_log_at is None or now-self.last_stationary_log_at>=1.:
            self.get_logger().info('OPERATOR_STATIONARY_PROOF '+json.dumps(
                report['stationary_proof'],allow_nan=False))
            self.last_stationary_log_at=now
        if self.last_phase != (report['phase'],report['reason']):
            self.get_logger().info(report['phase']+': '+report['reason'])
            boot, received = self.core.inputs.get('identity', ({}, now))
            boot = boot if isinstance(boot, dict) else {}
            actor = self.core.get('actor',now) or {}
            self.get_logger().info('OPERATOR_TRANSITION '+json.dumps(dict(
                phase=report['phase'], reason=report['reason'], fault=report['fault'],
                graph={k:graph.get(k) for k in ('valid','fault','errors','missing','discovery','raw_count','raw_owner')},
                identity={k:boot.get(k) for k in ('bridge_instance_id','connection_epoch',
                    'elf_sha256','valid','reason','state_age_ms','session_ready','telemetry_healthy')},
                identity_receive_age_ms=round((now-received)*1000,3),
                expected_identity=self.core.identity_lease.bound,
                receive_age_ms={n:round((now-at)*1000,3) for n,(_,at) in self.core.inputs.items()
                                if n in ('odom','actuator','ir_raw')},
                cycle=report['cycle'], budget=report['budget'],
                ir={k:report.get(k) for k in ('ir_band','ir_counts','ir_source_age_ms','range_confirm_samples','aligned_stop_proof')},
                recovery={n:{k:(self.core.get(n,now) or {}).get(k) for k in
                    ('state','stop_reason','publisher_owned','step_count','recovery_ready','blockers')}
                    for n in ('alignment','ir_live','guard','gateway')},
                actor={k:actor.get(k) for k in ('state','request_used','reason','blockers','pulse_request_feedback','pump_publisher_registered')},
                stationary_proof=report['stationary_proof'],
            ),allow_nan=False))
            self.last_phase = report['phase'],report['reason']
        return report

    def log_arm_proof(self, report, now, key):
        """Keep key receipts and bounded pre-arm evidence in the owned raw log."""
        proof = report['startup_readiness']
        ready = proof['ready']
        feedback = report['operator_feedback']
        if key or (feedback.get('key') is not None and feedback != self.last_key_feedback):
            self.get_logger().info(('AUTOMATIC_FEEDBACK ' if feedback.get('origin')=='AUTOMATIC' else 'OPERATOR_KEY ')+json.dumps(dict(
                received_key=key, feedback=report['operator_feedback'], armed=report['armed'],
                phase=report['phase'], startup_readiness=proof), allow_nan=False))
        self.last_key_feedback = dict(feedback)
        due = self.last_arm_log_at is None or now-self.last_arm_log_at >= 5.
        if not report['armed'] and (due or ready != self.last_arm_ready):
            ages = {name:round((now-at)*1000, 3)
                    for name, (_, at) in self.core.inputs.items()
                    if name in ('cmd','odom','actuator','vision','ir','ir_raw','identity')}
            self.get_logger().info('OPERATOR_ARM_PROOF '+json.dumps(dict(
                startup_readiness=proof, receive_age_ms=ages,
                camera_frame=report['camera_frame'],
                executor_blockers=(report['execution'].get('plan') or {}).get('blockers', [])),
                allow_nan=False))
            self.last_arm_log_at = now
        self.last_arm_ready = ready

    def request_shutdown(self,reason='Q',*,now=None):
        now=time.monotonic() if now is None else now
        self.stop_verification.begin(now,cause=reason,expected_identity=self.core.identity_lease.bound)
        if not self.core.closed:self.core.step(now,['Q'])
        self.intent_executor.close()

    def _stop_source(self,topic,name,typ,rel):
        infos=self.get_publishers_info_by_topic(topic)
        if not exact_endpoint(infos,name,typ,rel):return False
        gid=bytes(infos[0].endpoint_gid).hex()
        self.stop_pins.setdefault(topic,self.graph.pins.get(topic,gid))
        if self.stop_pins[topic]!=gid:
            self.stop_source_fault='STOP_SOURCE_GID_CHANGED:'+topic;return False
        return True

    def stop_tick(self,now=None):
        now=time.monotonic() if now is None else now
        self.intent_executor.hold_stopped()
        topics=('/cmd_vel','/odom',IDENTITY_TOPIC,'/fire_sensors/actuator_state')
        source_ok=all(self._stop_source(t,n,typ,q) for t,n,typ,q in EXPECTED if t in topics)
        inputs=dict(self.core.inputs)
        if not self._stop_source(PULSE_STATE_TOPIC,PULSE_NODE,'std_msgs/msg/String',RP.BEST_EFFORT):
            inputs.pop('actor',None)
        result=self.stop_verification.step(now,inputs,sources_ok=source_ok and self.stop_source_fault is None,
            raw_count=len(self.get_publishers_info_by_topic('/cmd_vel_raw')),
            pump_count=len(self.get_publishers_info_by_topic('/pump_cmd')))
        report=self.core.report(now)
        report.update(phase='STOP_CONFIRMED' if result['confirmed'] else 'STOP_UNCONFIRMED' if result['done'] else 'STOPPING_SESSION',
            reason=result['reason'],shutdown=result,execution=dict(enabled=self.intent_executor.enabled,
            live_intents_published=self.intent_executor.enabled,live_intents_active=False,stop_only=True),
            live_intents_published=self.intent_executor.enabled,live_intents_active=False)
        report['camera_frame']=self.frame_proof.report(now)
        self.last_report=report
        self.state_pub.publish(String(data=json.dumps(report,allow_nan=False,separators=(',',':'))))
        return report

    def close_session(self, reason='Q'):
        self.request_shutdown(reason)
        return self.stop_tick()


def main(args=None):
    previous = {s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    stopping = [False]; node = None; rc = 0
    def stop(*_): stopping[0] = True
    for sig in previous: signal.signal(sig,stop)
    try:
        rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
        node = OperationalSessionNode()
        next_tick = time.monotonic()
        while rclpy.ok():
            # spin_once dispatches only one ready callback. Drain callbacks
            # between paced UI/graph evaluations, rather than paying the full
            # GUI/graph cost once per individual telemetry message. Keep all
            # callbacks and state mutation on this same thread.
            remaining = max(0., next_tick-time.monotonic())
            rclpy.spin_once(node, timeout_sec=min(.01, remaining))
            if stopping[0]:node.request_shutdown('SIGNAL')
            tick_at = time.monotonic()
            if tick_at < next_tick and not stopping[0]:
                continue
            key = node.view.tick(node.last_report) if node.view is not None and not stopping[0] else None
            report=node.tick(key)
            if (report.get('shutdown') or {}).get('done'):break
            next_tick = max(tick_at+OPERATOR_TICK_PERIOD_S,
                            time.monotonic()+CALLBACK_MIN_WINDOW_S)
    except BaseException as exc:
        rc = 2
        if node is not None:
            node.get_logger().error('Operator exception: '+type(exc).__name__+': '+str(exc))
        else:
            import traceback
            traceback.print_exc()
    finally:
        try:
            if node is not None:
                try:
                    node.request_shutdown('FINALLY')
                    while rclpy.ok() and not node.stop_verification.done:
                        rclpy.spin_once(node,timeout_sec=.03)
                        node.stop_tick()
                    if not node.stop_verification.done:node.stop_verification.abort('ROS_CONTEXT_LOST')
                    if not node.stop_verification.confirmed or node.core.fault is not None:rc=2
                    node.get_logger().info('OPERATOR_STOP '+json.dumps(node.stop_verification.report(time.monotonic())))
                except BaseException as exc:
                    rc=2;node.get_logger().error('Operator stop verification failed: '+type(exc).__name__+': '+str(exc))
                finally:
                    try:
                        if node.view is not None:node.view.close()
                    except Exception as exc:
                        rc=2;node.get_logger().error('Operator view cleanup failed: '+str(exc))
                    finally:node.destroy_node()
        finally:
            try:
                if rclpy.ok(): rclpy.shutdown()
            finally:
                for sig, handler in previous.items(): signal.signal(sig,handler)
    return rc
