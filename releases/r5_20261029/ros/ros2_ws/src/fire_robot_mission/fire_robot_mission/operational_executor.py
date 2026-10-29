"""Opt-in ROS intent boundary. No Twist or /pump_cmd publisher is created."""
from rcl_interfaces.srv import GetParameters
from std_msgs.msg import Bool,String
import json
from .alignment_handoff_shadow import arm_qos
from .operational_intents import IntentMapper,ARM_TOPICS
from .stationary_confirmation import TOPIC as STATIONARY_TOPIC
from .cycle_release import CONTROL_TOPIC
import copy


class OperationalExecutor:
    def __init__(self,node):
        self.node=node;self.mapper=IntentMapper();self.pending=None
        self.query_at=0.;self.safety_ok_at=None;self.last=None;self.closed=False
        node.declare_parameter('enable_live_intents',False)
        self.enabled=node.get_parameter('enable_live_intents').value is True
        self.arms={};self.control=None;self.stationary=None;self.client=None
        if not self.enabled:return
        for topic in (*ARM_TOPICS,CONTROL_TOPIC,STATIONARY_TOPIC):
            if node.resolve_topic_name(topic)!=topic:raise ValueError('OPERATOR_INTENT_REMAP_FORBIDDEN')
        for topic in ARM_TOPICS:self.arms[topic]=node.create_publisher(Bool,topic,arm_qos())
        self.control=node.create_publisher(String,CONTROL_TOPIC,arm_qos())
        self.stationary=node.create_publisher(String,STATIONARY_TOPIC,arm_qos())
        self.client=node.create_client(GetParameters,'/safety_watchdog/get_parameters')

    def poll(self,now):
        if not self.enabled or self.closed:return
        if self.pending is not None:
            future,start=self.pending
            if future.done():
                self.pending=None
                try:
                    v=future.result().values
                    good=(len(v)==4 and [x.type for x in v]==[4,4,2,3]
                          and v[0].string_value=='/cmd_vel_raw' and v[1].string_value=='/cmd_vel'
                          and v[2].integer_value==1000 and v[3].double_value==10.)
                    self.safety_ok_at=now if good else None
                except Exception:self.safety_ok_at=None
            elif now-start>3.:
                self.client.remove_pending_request(future);future.cancel();self.pending=None;self.safety_ok_at=None
        elif now>=self.query_at:
            self.query_at=now+3.
            if self.client.service_is_ready():
                request=GetParameters.Request()
                request.names=['cmd_vel_input_topic','cmd_vel_output_topic','heartbeat_timeout_ms','cmd_vel_output_rate_hz']
                try:self.pending=(self.client.call_async(request),now)
                except Exception:self.safety_ok_at=None

    def safety_ok(self,now):return self.safety_ok_at is not None and 0<=now-self.safety_ok_at<=10.

    def step(self,session,report,handback,now):
        if not self.enabled or self.closed:
            return dict(enabled=False,live_intents_published=False,live_intents_active=False)
        plan=self.mapper.step(session,report,handback,now,safety_ok=self.safety_ok(now))
        if plan['cycle_control'] is None and (session.identity_hold or session.input_hold) and session.fault is None and not session.closed:
            plan['cycle_control']=copy.deepcopy(handback.get('control'))
            if plan['cycle_control'] is not None and plan['cycle_control'].get('phase')=='ACTIVE':
                # Peers may see different identity callback timing. This is
                # a STOP-only pause, never fresh identity or motion authority.
                plan['cycle_control']['session_hold']=dict(schema_version=1,
                    cause='RECOVERABLE_INPUT_GAP' if session.input_hold else 'SAME_BOOT_IDENTITY_RECOVERY')
        if plan['cycle_control'] is not None:
            self.control.publish(String(data=json.dumps(plan['cycle_control'],allow_nan=False)))
        for topic,pub in self.arms.items():pub.publish(Bool(data=plan['arms'][topic]))
        self.stationary.publish(String(data=json.dumps(plan['stationary_request'],allow_nan=False)))
        self.last=plan
        # An enabled Gate2 publisher emits safe false heartbeats before ARM.
        # Keep that transport activity separate from an intent that can grant
        # ownership to a live consumer. Startup may accept the former, never
        # the latter.
        active=(any(plan['arms'].values()) or plan['stationary_request']['active'] is True
                or plan['cycle_control'] is not None)
        return dict(enabled=True,live_intents_published=True,live_intents_active=bool(active),
                    plan=plan,safety_parameters_fresh=self.safety_ok(now))

    def close(self):
        if not self.enabled or self.closed:return
        self.closed=True
        self.hold_stopped()
        if self.pending is not None:
            future,_=self.pending
            self.client.remove_pending_request(future);future.cancel();self.pending=None

    def hold_stopped(self):
        """False heartbeat only; no ACTIVE control refresh during teardown."""
        if not self.enabled:return
        for pub in self.arms.values():pub.publish(Bool(data=False))
        self.stationary.publish(String(data='{"schema_version":1,"active":false}'))
