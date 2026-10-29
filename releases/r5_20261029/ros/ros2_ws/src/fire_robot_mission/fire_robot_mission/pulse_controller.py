#!/usr/bin/env python3
"""Installed pulse adapter, disabled by default; requires the R5 operational session."""
from .cycle_endpoint import CycleEndpoint

import json
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool,String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray
from rcl_interfaces.srv import GetParameters
from fire_robot_mission.mission_supervisor import action_status_qos,sensor_qos
from fire_robot_mission.alignment_handoff_shadow import arm_qos
from .operational_contract import OPERATOR_NODE, PULSE_NODE, PULSE_STATE_TOPIC
from .pulse_session import PulseSession
from .pulse_sequence import PulseSequenceSession
from .pulse_transport import Transport,exact,six
from .pulse_registration import registration_ready
from .stationary_confirmation import TOPIC
from fire_robot_mission.prespray_recovery import REQUEST_TOPIC, fresh
from fire_robot_mission.pump_demo_core import zero_twist

NAME=PULSE_NODE
INPUTS={
 'ir':('/fire_mission/ir_range_preview','ir_range_preview'),
 'ir_live':('/fire_mission/ir_approach_live_state','ir_approach_live'),
 'mission':('/fire_mission/state','fire_mission_supervisor'),
 'alignment':('/fire_mission/live_alignment_state','live_alignment_takeover'),
 'gateway':('/fire_mission/live_nav_stop_state','live_nav_stop_gateway'),
 'guard':('/fire_mission/nav2_cancel_guard_state','nav2_cancel_guard'),
 'actuator':('/fire_sensors/actuator_state','serial_bridge_node'),
 'vision':('/yolo/detections','yolo_observer'),
 'preview':('/yolo/alignment_preview','fire_alignment_observer')}


class PulseController(Node):
    def __init__(self):
        super().__init__(NAME)
        self.declare_parameter('enable_prespray_cancel',False)
        self.cancel_enabled=self.get_parameter('enable_prespray_cancel').value is True
        self.cancel_request=None;self.cancel_request_at=None
        self.declare_parameter('enable_pulse',False)
        self.declare_parameter('allow_provisional_ir',False)
        self.declare_parameter('allow_close_stationary',False)
        self.declare_parameter('max_pulses_per_cycle',1)
        self.declare_parameter('max_total_pump_on_s',15.)
        self.declare_parameter('respray_settle_s',2.)
        self.declare_parameter('respray_wait_s',30.)
        for key,value in dict(pulse_s=5.,start_on_ack=False,ir_confirm_samples=5,unbounded_approach=False,reposition_resets_quota=False).items():
            self.declare_parameter(key,value)
        rounds=self.get_parameter('max_pulses_per_cycle').value
        if type(rounds) is not int or not 1<=rounds<=3:
            raise ValueError('max_pulses_per_cycle must be an integer from 1 to 3')
        config=dict(enabled=self.get_parameter('enable_pulse').value,
            allow_provisional_ir=self.get_parameter('allow_provisional_ir').value,
            allow_close_stationary=self.get_parameter('allow_close_stationary').value,
            **{key:self.get_parameter(key).value for key in ('pulse_s','start_on_ack','ir_confirm_samples','unbounded_approach')})
        self.session=(PulseSession(**config) if rounds==1 else PulseSequenceSession(**config,
            max_pulses=rounds,max_total_on_s=self.get_parameter('max_total_pump_on_s').value,
            settle_s=self.get_parameter('respray_settle_s').value,
            retry_window_s=self.get_parameter('respray_wait_s').value,
            reposition_resets_quota=self.get_parameter('reposition_resets_quota').value))
        self.last_actor_trace=None
        self.transport=Transport(self.session);self.stop_requested=False
        self.safety_ok_at=None;self.pending=None;self.query_at=0
        topics=[v[0] for v in INPUTS.values()]+['/pump_cmd','/fire_mission/pump_demo_arm',
            '/cmd_vel_raw','/cmd_vel','/odom','/navigate_to_pose/_action/status',PULSE_STATE_TOPIC,TOPIC]
        if any(self.resolve_topic_name(t)!=t for t in topics):raise ValueError('R4d topic remapping forbidden')
        # No pump publisher exists during navigation/alignment/IR, even OFF.
        self.pub=None
        self.state_pub=self.create_publisher(String,PULSE_STATE_TOPIC,sensor_qos())
        for name,(topic,_) in INPUTS.items():
            self.create_subscription(String,topic,lambda m,key=name:self.transport.json_input(key,m.data,time.monotonic()),sensor_qos())
        self.create_subscription(Bool,'/fire_mission/pump_demo_arm',lambda m:self.transport.arm(m.data,time.monotonic()),arm_qos())
        self.create_subscription(String,TOPIC,lambda m:self.transport.json_input('stationary_request',m.data,time.monotonic()),arm_qos())
        self.create_subscription(Odometry,'/odom',lambda m:self.transport.odom(m,time.monotonic(),self.get_clock().now().nanoseconds),sensor_qos())
        self.create_subscription(Twist,'/cmd_vel',lambda m:self.session.update('cmd',six(m),time.monotonic()),sensor_qos())
        self.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',lambda m:self.transport.status(m,time.monotonic()),action_status_qos())
        self.client=self.create_client(GetParameters,'/safety_watchdog/get_parameters')
        if self.cancel_enabled:
            if self.resolve_topic_name(REQUEST_TOPIC)!=REQUEST_TOPIC:raise ValueError('pre-spray remap forbidden')
            self.create_subscription(String,REQUEST_TOPIC,self.on_prespray_cancel,arm_qos())
        self._cycle = CycleEndpoint(self, 'actor', 'session')
        if rounds>1 and not self._cycle.enabled:
            raise ValueError('multiple pulses require the R5 cycle protocol')
        self.create_timer(.1,self.tick)
        self.get_logger().warning(f'Pulse controller: up to {rounds} pulses, maximum {self.session.pump.pulse_s}s each; S3 servo 60-115 deg. Operational session and BOOT gate required.')

    def count(self,topic,name,typ,reliability,*,subscriber=False):
        fn=self.get_subscriptions_info_by_topic if subscriber else self.get_publishers_info_by_topic
        return exact(fn(topic),name,typ,reliability)

    def counts(self):
        be=ReliabilityPolicy.BEST_EFFORT;rel=ReliabilityPolicy.RELIABLE
        result={key:self.count(topic,name,'std_msgs/msg/String',be) for key,(topic,name) in INPUTS.items()}
        result.update(odom=self.count('/odom','serial_bridge_node','nav_msgs/msg/Odometry',be),
            cmd=self.count('/cmd_vel','safety_watchdog','geometry_msgs/msg/Twist',be),
            status=self.count('/navigate_to_pose/_action/status','bt_navigator','action_msgs/msg/GoalStatusArray',rel),
            arm=self.count('/fire_mission/pump_demo_arm',OPERATOR_NODE,'std_msgs/msg/Bool',rel),
            pump=self.count('/pump_cmd',NAME,'std_msgs/msg/Bool',rel),
            pump_subscribers=self.pump_subscribers(rel),
            stationary_request=self.count(TOPIC,OPERATOR_NODE,'std_msgs/msg/String',rel),
            pump_total=len(self.get_publishers_info_by_topic('/pump_cmd')),
            raw=len(self.get_publishers_info_by_topic('/cmd_vel_raw')))
        result['ir_endpoint_absent']=not self.get_publishers_info_by_topic(INPUTS['ir'][0])
        return result

    def pump_subscribers(self,rel):
        # The driver records Bool requests independently. Both readers are
        # required and every unexpected/duplicate subscriber blocks admission.
        infos=self.get_subscriptions_info_by_topic('/pump_cmd')
        return int(len(infos)==2 and all(exact([i for i in infos if i.node_name==name],
            name,'std_msgs/msg/Bool',rel) for name in ('serial_bridge_node',OPERATOR_NODE)))

    def poll_safety(self,now):
        if self.pending is not None:
            future,start=self.pending
            if future.done():
                self.pending=None
                try:
                    v=future.result().values
                    good=len(v)==4 and [x.type for x in v]==[4,4,2,3] and v[0].string_value=='/cmd_vel_raw' and v[1].string_value=='/cmd_vel' and v[2].integer_value==1000 and v[3].double_value==10.
                    self.safety_ok_at=now if good else None
                except Exception as exc:
                    self.safety_ok_at=None
                    self.get_logger().warning('R4d safety query failed: '+str(exc))
            elif now-start>3:
                self.client.remove_pending_request(future);future.cancel();self.pending=None;self.safety_ok_at=None
        elif now>=self.query_at:
            self.query_at=now+3
            if self.client.service_is_ready():
                req=GetParameters.Request();req.names=['cmd_vel_input_topic','cmd_vel_output_topic','heartbeat_timeout_ms','cmd_vel_output_rate_hz']
                self.pending=(self.client.call_async(req),now)

    def on_prespray_cancel(self,message):
        try:self.cancel_request=json.loads(message.data)
        except (ValueError,TypeError):self.cancel_request=None
        self.cancel_request_at=time.monotonic()

    def close_before_spray(self,now,counts,safe):
        if (self.cancel_enabled and fresh(self.cancel_request_at,now)
                and self.count(REQUEST_TOPIC,OPERATOR_NODE,'std_msgs/msg/String',ReliabilityPolicy.RELIABLE)==1):
            self.session.cancel_before_spray(self.cancel_request,now)
        if self.session.prespray_cancel is not None:
            self.transport.pending_p=None
            if self.pub is not None:
                self.pub.publish(Bool(data=False))
                if (safe and counts.get('actuator')==counts.get('cmd')==counts.get('odom')==1
                        and counts.get('raw')==0 and self.session.idle(self.session.actuator(now))
                        and zero_twist(self.session.get('cmd',now),.001)
                        and zero_twist(self.session.get('odom',now),.02)):
                    self.destroy_publisher(self.pub);self.pub=None

    def tick(self):
        if hasattr(self, '_cycle'):self._cycle.before_step(time.monotonic())
        now=time.monotonic();self.poll_safety(now)
        counts=self.transport.fresh_counts(now,self.counts())
        safe=self.safety_ok_at is not None and 0<=now-self.safety_ok_at<=10
        if not safe:counts['cmd']=0
        if not self.session.committed() and (counts['vision']!=1 or counts['preview']!=1):
            self.session.update('mission',None,now)
        self.close_before_spray(now,counts,safe)
        report=self.transport.step(now,counts,stop=self.stop_requested)
        if (hasattr(self.session,'stage') and self.session.stage in ('REVIEW','TERMINAL')
                and hasattr(self,'_cycle')):
            # A fresh OFF/home release permits the existing alignment node
            # to recenter. The sequence itself never owns a motor publisher.
            self._cycle._withdraw(now,force=True)
        if ((not hasattr(self, '_cycle') or self._cycle.stage_allowed(now))
                and self.pub is None and self.session.prespray_cancel is None and not self.stop_requested and not self.transport.arm_last
                and registration_ready(self.session,now,counts,len(self.get_publishers_info_by_topic('/pump_cmd')))):
            self.pub=self.create_publisher(Bool,'/pump_cmd',QoSProfile(depth=10,reliability=ReliabilityPolicy.RELIABLE))
            self.get_logger().info('R4D_PUMP_PUBLISHER_REGISTERED_AFTER_IR_SAFETY_PROOF; OFF only until explicit P')
        on=(report['proposed_pump_on'] and safe
            and (not hasattr(self, '_cycle') or self._cycle.stage_allowed(now)))
        if self.pub is not None:self.pub.publish(Bool(data=on))
        report.update(scope='R5_INSTALLED_PULSE_ADAPTER_CANDIDATE',published_pump_request=on if self.pub is not None else None,
                      safety_parameters_fresh=safe,counts=counts,pump_publisher_registered=self.pub is not None,
                      wall_ns=time.time_ns(),mono_ns=time.monotonic_ns(),
                      pulse_on_since=self.session.pump.on_since,off_at=self.session.off_at,
                      start_home_count=getattr(self.session,'start_home_count',self.session.start_home),
                      actuator=self.session.actuator(now))
        if self.session.prespray_cancel is not None:
            report['prespray_cancel']=dict(self.session.prespray_cancel,closed=self.pub is None)
        if hasattr(self, '_cycle'):report=self._cycle.decorate(report,time.monotonic())
        trace=(report.get('state'),(report.get('spray_sequence') or {}).get('pulse_index'),
            report.get('proposed_pump_on'),report.get('reason'),
            (report.get('actuator') or {}).get('pump_pwm_commanded_on'),
            (report.get('actuator') or {}).get('servo_phase'))
        if trace!=self.last_actor_trace:
            self.get_logger().info('ACTOR_TRANSITION '+json.dumps({k:report.get(k) for k in
                ('state','reason','wall_ns','mono_ns','r5_cycle_context','request_used','spray_sequence',
                 'proposed_pump_on','pulse_on_since','off_at','start_home_count','actuator')},allow_nan=False))
            self.last_actor_trace=trace
        self.state_pub.publish(String(data=json.dumps(report,allow_nan=False,separators=(',',':'))))

    def destroy_node(self):
        if self.pub is not None:
            for _ in range(3):self.pub.publish(Bool(data=False))
        return super().destroy_node()


def main(args=None):
    previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    stopping=[False]
    def stop(*_):stopping[0]=True
    signal.signal(signal.SIGINT,stop);signal.signal(signal.SIGTERM,stop)
    node=None;rc=0
    try:
        rclpy.init(args=args,signal_handler_options=SignalHandlerOptions.NO)
        node=PulseController()
        while rclpy.ok() and not stopping[0]:rclpy.spin_once(node,timeout_sec=.05)
    except BaseException as exc:
        rc=2
        if node is not None:node.get_logger().error('R4d controller stopping: '+type(exc).__name__+': '+str(exc))
        import traceback
        traceback.print_exc()
    finally:
        try:
            if node is not None:
                # Keep requesting OFF even when the main loop failed. Never let an
                # exception bypass the bounded independent stop confirmation.
                from .actuator_drain import Drain
                node.stop_requested=True
                guard=Drain(time.monotonic(),node.session.start_home if node.session.on_ack else None)
                record={'confirmed':False,'reason':'NO_STOP_TELEMETRY'}
                try:
                    while rclpy.ok() and not guard.done:
                        if node.pub is not None:node.pub.publish(Bool(data=False))
                        rclpy.spin_once(node,timeout_sec=.03);now=time.monotonic()
                        a,at=node.session.inputs.get('actuator',({},-1e100))
                        cmd,ct=node.session.inputs.get('cmd',(None,None))
                        odom,ot=node.session.inputs.get('odom',(None,None))
                        record=guard.step(now,a,at,cmd,ct,odom,ot,len(node.get_publishers_info_by_topic('/cmd_vel_raw')))
                except BaseException as exc:
                    rc=2;record={'confirmed':False,'reason':'ACTOR_DRAIN_EXCEPTION:'+type(exc).__name__}
                node.get_logger().info('ACTOR_DRAIN '+json.dumps(record))
                if not record.get('confirmed'):rc=2
                node.destroy_node()
        finally:
            try:
                if rclpy.ok():rclpy.shutdown()
            finally:
                for sig,handler in previous.items():signal.signal(sig,handler)

    return rc

if __name__=='__main__':raise SystemExit(main())
