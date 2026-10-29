"""Actual node constructors/callbacks/ticks on an in-memory transport.

Six participants are real; no participant core, ACK, reset, mission, gateway,
alignment or IR report is replaced. External sensors/Nav2/SafetyGate/actuator
are simulated. This is deliberately not a DDS or hardware acceptance test.
"""
import copy
import json
import math
import time
from types import SimpleNamespace as N
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy as RP
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from action_msgs.msg import GoalStatus,GoalStatusArray
from action_msgs.srv import CancelGoal
from std_msgs.msg import String,Bool
from fire_robot_bringup.serial_session_identity import SerialSessionIdentity
from fire_robot_mission.operational_contract import FIRMWARE_ELF_SHA256 as ELF
from fire_robot_mission.ir_range_core import IrRangePreviewCore
from fire_robot_mission.live_nav_stop_gateway import LiveNavStopGateway
from fire_robot_mission.nav2_cancel_guard import Nav2CancelGuard
from fire_robot_mission.live_alignment_takeover import LiveAlignmentTakeover
from fire_robot_mission.ir_approach_live import IrApproachLiveNode
from fire_robot_mission.mission_supervisor import FireMissionSupervisor
from fire_robot_mission.pulse_controller import PulseController
from fire_robot_mission.operational_session import OperationalSessionNode,STATE_TOPIC
from fire_robot_mission.post_spray_review import PostSprayReviewNode
from fire_robot_mission.nav2_cancel_intent_shadow import Nav2CancelIntentShadow
from fire_robot_mission.alignment_handoff_shadow import AlignmentHandoffShadow

BE=QoSProfile(depth=1,reliability=RP.BEST_EFFORT)
REL=QoSProfile(depth=1,reliability=RP.RELIABLE)


def typename(typ):return '/'.join(typ.__module__.split('.')[:2])+'/'+typ.__name__


class Future:
    def __init__(self):self.value=None;self.callbacks=[];self.ready=False
    def done(self):return self.ready
    def result(self):return self.value
    def cancel(self):self.ready=True
    def add_done_callback(self,cb):
        self.callbacks.append(cb)
        if self.ready:cb(self)
    def complete(self,value):
        self.value=value;self.ready=True
        for cb in self.callbacks:cb(self)


class Bus:
    def __init__(self,monkeypatch,overrides=None):
        self.now=1.;self.pubs=[];self.subs=[];self.queue=[];self.scheduled=[]
        self.sent=[];self.last={};self.logs=[];self.gid=0;self.cancelled=[];self.max_raw=0
        self.owner_events=[]
        self.drop=lambda publisher,subscriber,msg:False
        self.overrides=overrides or {}
        def init(node,name,*a,**kw):node.test_name=name;node.test_params={};node.test_timers=[]
        monkeypatch.setattr(Node,'__init__',init)
        monkeypatch.setattr(Node,'declare_parameter',lambda n,k,v:n.test_params.setdefault(k,v))
        monkeypatch.setattr(Node,'get_parameter',lambda n,k:N(value=self.overrides.get(n.test_name,{}).get(k,n.test_params[k])))
        monkeypatch.setattr(Node,'resolve_topic_name',lambda n,t:t)
        monkeypatch.setattr(Node,'get_name',lambda n:n.test_name)
        monkeypatch.setattr(Node,'create_publisher',lambda n,typ,t,q:self.publisher(n.test_name,typ,t,q))
        monkeypatch.setattr(Node,'create_subscription',lambda n,typ,t,cb,q:self.subscription(n.test_name,typ,t,cb,q))
        monkeypatch.setattr(Node,'destroy_publisher',lambda n,p:self.destroy(p))
        monkeypatch.setattr(Node,'create_timer',lambda n,period,cb:n.test_timers.append(cb))
        monkeypatch.setattr(Node,'get_publishers_info_by_topic',lambda n,t:[p.info for p in self.pubs if p.topic==t])
        monkeypatch.setattr(Node,'get_subscriptions_info_by_topic',lambda n,t:[s.info for s in self.subs if s.topic==t])
        monkeypatch.setattr(Node,'count_publishers',lambda n,t:sum(p.topic==t for p in self.pubs))
        monkeypatch.setattr(Node,'get_logger',lambda n:N(**{k:lambda *a,who=n.test_name:self.logs.append((self.now,who,a)) for k in ('info','warning','warn','error','debug')}))
        monkeypatch.setattr(Node,'get_clock',lambda n:N(now=lambda:N(nanoseconds=round(self.now*1e9))))
        monkeypatch.setattr(Node,'create_client',lambda n,typ,t:self.client(t))
        monkeypatch.setattr(time,'monotonic',lambda:self.now)
        monkeypatch.setattr(time,'monotonic_ns',lambda:round(self.now*1e9))

    def info(self,name,typ,q):
        self.gid+=1
        return N(node_name=name,node_namespace='/',topic_type=typename(typ),qos_profile=q,
                 endpoint_gid=list(self.gid.to_bytes(24,'big')))

    def publisher(self,name,typ,topic,q):
        p=N(info=self.info(name,typ,q),topic=topic,typ=typ)
        def publish(msg):
            assert p in self.pubs,'destroyed publisher used'
            self.sent.append((self.now,name,topic,copy.deepcopy(msg)))
            self.last[topic]=copy.deepcopy(msg)
            for s in list(self.subs):
                if s.topic==topic and not self.drop(p,s,msg):self.queue.append((s,copy.deepcopy(msg)))
        p.publish=publish;self.pubs.append(p)
        if topic in ('/cmd_vel_raw','/pump_cmd'):
            self.owner_events.append(dict(at=self.now,node=name,topic=topic,event='CREATE'))
        if topic=='/cmd_vel_raw':
            self.max_raw=max(self.max_raw,sum(x.topic==topic for x in self.pubs))
            assert self.max_raw<=1,'two simultaneous raw owners'
        return p

    def destroy(self,p):
        if p.topic in ('/cmd_vel_raw','/pump_cmd'):
            last=self.last.get(p.topic)
            zero=(last.data is False if p.topic=='/pump_cmd' else
                  last is not None and all(v==0. for v in (last.linear.x,last.linear.y,last.linear.z,last.angular.x,last.angular.y,last.angular.z)))
            assert zero,'publisher destroyed before zero/OFF'
            self.owner_events.append(dict(at=self.now,node=p.info.node_name,topic=p.topic,event='RELEASE',last_zero_or_OFF=True))
        self.pubs.remove(p)

    def subscription(self,name,typ,topic,cb,q):
        s=N(info=self.info(name,typ,q),topic=topic,callback=cb);self.subs.append(s);return s

    def flush(self,reverse=False):
        pending=self.queue;self.queue=[]
        if reverse:pending=list(reversed(pending))
        for s,m in pending:s.callback(m)

    def client(self,topic):
        def call(request):
            f=Future()
            if topic.endswith('get_parameters'):
                values=[N(type=4,string_value='/cmd_vel_raw'),N(type=4,string_value='/cmd_vel'),
                        N(type=2,integer_value=1000),N(type=3,double_value=10.)]
                self.scheduled.append((self.now+.01,lambda:f.complete(N(values=values))))
            else:
                assert topic=='/navigate_to_pose/_action/cancel_goal'
                uid=bytes(request.goal_info.goal_id.uuid).hex()
                assert uid!='0'*32 and request.goal_info.stamp.sec==request.goal_info.stamp.nanosec==0
                self.cancelled.append((self.now,uid))
                response=CancelGoal.Response(return_code=0,goals_canceling=[copy.deepcopy(request.goal_info)])
                self.scheduled.append((self.now+.05,lambda:f.complete(response)))
            return f
        return N(service_is_ready=lambda:True,call_async=call,remove_pending_request=lambda f:None)

    def advance(self):
        self.now=round(self.now+.05,8)
        for at,fn in list(self.scheduled):
            if self.now>=at:self.scheduled.remove((at,fn));fn()

    def report(self,topic):
        return json.loads(self.last[topic].data) if topic in self.last else {}


class Robot:
    instances=[]
    def __init__(self,monkeypatch,*,goal=False,reverse=False):
        overrides={name:{'enable_cycle_protocol':True} for name in
            ('live_nav_stop_gateway','nav2_cancel_guard','live_alignment_takeover','ir_approach_live','fire_mission_supervisor','fire_pulse_controller')}
        for n,k in [('live_nav_stop_gateway','enable_live_output'),('nav2_cancel_guard','enable_real_cancel'),
                    ('live_alignment_takeover','enable_live_alignment'),('ir_approach_live','enable_live_ir_jog'),
                    ('fire_pulse_controller','enable_pulse'),('fire_robot_operator','enable_live_intents'),
                    ('post_spray_review','use_operational_actor'),('alignment_handoff_shadow','require_live_cancel_proof')]:overrides.setdefault(n,{})[k]=True
        overrides['ir_approach_live'].update(integrated_handoff=True,continuous_approach=True,
            speed_m_s=.08,continuous_distance_m=.30,max_total_approach_m=.30,allow_provisional_ir=True)
        overrides['fire_pulse_controller']['allow_provisional_ir']=True
        self.bus=Bus(monkeypatch,overrides);self.reverse=reverse
        self.nodes=[FireMissionSupervisor(),LiveNavStopGateway(),Nav2CancelIntentShadow(),Nav2CancelGuard(),
            AlignmentHandoffShadow(),LiveAlignmentTakeover(),IrApproachLiveNode(),PulseController(),PostSprayReviewNode()]
        self.by_name={n.test_name:n for n in self.nodes};self.op=OperationalSessionNode()
        self.fire=False;self.ir_flame=None;self.identity_gap=False
        self.centered=False;self.right=5000;self.status=[('a'*32,2)] if goal else []
        self.x=self.yaw=0.;self.home=0;self.on_at=None;self.off_at=None;self.pump_requested=False
        self.device_on=False;self.home_done=True;self.pulses=[]
        self.identity=SerialSessionIdentity('ab'*16);self.seq=1
        self.identity.observe(dict(type='BOOT',seq=1,esp_ms=1,reset_reason=1,elf_sha256=ELF),1,self.bus.now)
        self.range=IrRangePreviewCore(baselines=(26060,26140,25825),baseline_tolerance=500,min_active_drop=1000,
            right_thresholds=(800,1000,20000),median_window=5,stable_samples=3,ir_timeout_s=.6,vision_timeout_s=.5,
            max_ir_age_ms=300.,max_image_age_ms=300.,spray_confirm_samples=5,spray_reentry_margin_counts=0,
            range_policy='YOLO_RIGHT_ADC_20261025')
        self.external={}
        for name,typ,t,q in [('serial_bridge_node',Odometry,'/odom',BE),('serial_bridge_node',String,'/fire_sensors/actuator_state',BE),
            ('serial_bridge_node',String,'/fire_sensors/ir_raw',BE),
            ('serial_bridge_node',String,'/fire_sensors/serial_session_identity',BE),('safety_watchdog',Twist,'/cmd_vel',BE),
            ('yolo_observer',String,'/yolo/detections',BE),('fire_alignment_observer',String,'/yolo/alignment_preview',BE),
            ('ir_range_preview',String,'/fire_mission/ir_range_preview',BE),('scan_qos_relay',LaserScan,'/scan',BE),
            ('bt_navigator',GoalStatusArray,'/navigate_to_pose/_action/status',REL)]:self.external[t]=self.bus.publisher(name,typ,t,q)
        self.nav=[self.bus.publisher(name,Twist,'/fire_mission/nav2_cmd_vel_shadow',REL) for name in ('velocity_smoother','behavior_server','behavior_server')]
        self.bus.subscription('serial_bridge_node',Bool,'/pump_cmd',lambda m:setattr(self,'pump_requested',m.data),REL)
        self.bus.subscription('safety_watchdog',Twist,'/cmd_vel_raw',lambda m:None,BE)
        self.reports=[];self.snapshots=[]
        self.instances.append(self)

    def emit(self,t,value):
        self.external[t].publish(String(data=json.dumps(value)) if isinstance(value,dict) else value)

    def inputs(self):
        b=self.bus;n=b.now
        raw=[p for p in b.pubs if p.topic=='/cmd_vel_raw']
        last=b.last.get('/cmd_vel_raw',Twist()) if raw else Twist()
        self.x+=last.linear.x*.05;self.yaw+=last.angular.z*.05
        odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link'
        odom.header.stamp.sec=int(n);odom.header.stamp.nanosec=round((n-int(n))*1e9)
        odom.pose.pose.orientation.z=math.sin(self.yaw/2);odom.pose.pose.orientation.w=math.cos(self.yaw/2)
        odom.pose.pose.position.x=self.x;odom.twist.twist=copy.deepcopy(last)
        self.emit('/odom',odom);self.emit('/cmd_vel',copy.deepcopy(last))
        for p in self.nav:p.publish(Twist())
        for at,uid in b.cancelled:
            self.status=[(u,5 if n>=at+.15 else 3) if u==uid and s in (1,2,3) else (u,s) for u,s in self.status]
        rows=[]
        for uid,s in self.status:
            row=GoalStatus(status=s);row.goal_info.goal_id.uuid=list(bytes.fromhex(uid));row.goal_info.stamp.sec=1;rows.append(row)
        self.emit('/navigate_to_pose/_action/status',GoalStatusArray(status_list=rows))
        detection=dict(schema_version=1,observe_only=True,state='ok',source={'age_ms':20},
            fire=dict(detected_now=self.fire,confirmed=self.fire))
        preview=dict(schema_version=1,observe_only=True,state=('CENTERED' if self.centered else 'ALIGNING_LEFT') if self.fire else 'NO_TARGET',
            source={'age_ms':20},preview=dict(target_angular_z_rad_s=0. if self.centered or not self.fire else .3,
            error_x_normalized=.01 if self.centered else .3,direction='STOP' if self.centered else 'LEFT'),
            safety=dict(motion_command_publisher=False,actuator_command_publisher=False))
        self.emit('/yolo/detections',detection);self.emit('/yolo/alignment_preview',preview)
        self.range.update_detection(detection,n);self.range.update_alignment(preview,n)
        adc=self.right if (self.fire if self.ir_flame is None else self.ir_flame) else 26000
        raw_ir=dict(schema_version=1,valid=True,left=adc,center=adc,right=adc,sample_age_ms=10.,receive_age_ms=10.)
        self.range.update_ir(raw_ir,n)
        self.emit('/fire_sensors/ir_raw',raw_ir)
        self.emit('/fire_mission/ir_range_preview',self.range.snapshot(n))
        scan=LaserScan(angle_min=-3.14,angle_max=3.14,angle_increment=6.28/359,range_min=.01,range_max=8.,ranges=[1.]*360)
        self.emit('/scan',scan)
        self.seq+=1;self.identity.observe(dict(type='STATE',seq=self.seq,esp_ms=round(n*1000)),1,n)
        if not self.identity_gap:self.emit('/fire_sensors/serial_session_identity',self.identity.snapshot(1,n,True,True))
        phase='IDLE';servo=False;angle=None
        if self.pump_requested:
            if not self.device_on:
                self.on_at=n;self.off_at=None;self.home_done=False
                self.pulses.append(dict(on_at=n,home_before=self.home))
            phase='SWEEPING';servo=True;angle=60. if n-self.on_at<2. else 115.
        elif self.on_at is not None:
            if self.off_at is None:
                self.off_at=n;self.pulses[-1]['off_at']=n
            if n-self.off_at<1.2:phase='RETURNING_HOME';servo=True;angle=90.
            elif not self.home_done:
                self.home+=1;self.home_done=True;self.pulses[-1]['home_after']=self.home
        self.device_on=self.pump_requested
        self.emit('/fire_sensors/actuator_state',dict(schema_version=1,mode='COMMAND_TELEMETRY_ONLY',
            esp_ms=round(n*1000),receive_age_ms=20,pump_pwm_ready=True,pump_pwm_commanded_on=self.pump_requested,
            servo_pwm_commanded_on=servo,servo_phase=phase,servo_commanded_angle_deg=angle,home_command_completed_count=self.home))

    def step(self,key=None,*,skip=()):
        b=self.bus;b.advance();self.inputs();b.flush(self.reverse)
        nodes=list(reversed(self.nodes)) if self.reverse else self.nodes
        for node in nodes:
            if node.test_name in skip:continue
            for callback in node.test_timers:callback()
            b.flush(self.reverse)
        result=self.op.tick(key);b.flush(self.reverse)
        self.reports.append(copy.deepcopy(result))
        self.snapshots.append({n.test_name:b.report(t) for n,t in [(self.by_name['live_nav_stop_gateway'],'/fire_mission/live_nav_stop_state'),
            (self.by_name['nav2_cancel_guard'],'/fire_mission/nav2_cancel_guard_state'),(self.by_name['live_alignment_takeover'],'/fire_mission/live_alignment_state'),
            (self.by_name['ir_approach_live'],'/fire_mission/ir_approach_live_state'),(self.by_name['fire_pulse_controller'],'/fire_mission/r4d_state')]})
        return result

    def until(self,predicate,limit=180):
        for _ in range(limit):
            r=self.step()
            if predicate(r):return r
            assert r['fault'] is None,self.diagnostic(r)
        raise AssertionError(self.diagnostic(r))

    def diagnostic(self,r):
        return dict(operator=r,six={n.test_name:dict(phase=n._cycle.participant.phase,error=n._cycle.participant.error,
            core_phase=getattr(getattr(n,n._cycle.core_attr),'phase',None),
            stop_reason=getattr(getattr(n,n._cycle.core_attr),'stop_reason',None)) for n in self.nodes if hasattr(n,'_cycle')},
            reports=self.snapshots[-1] if self.snapshots else {},logs=self.bus.logs[-12:])

    def arm(self):
        self.until(lambda r:r['reason']=='READY_PRESS_A')
        r=self.step('A');assert r['armed'],self.diagnostic(r);return r
