"""Read-only startup observations. Imports start no context or device."""
import argparse
from collections import deque
import json
import math
from pathlib import Path
import os
import signal
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.signals import SignalHandlerOptions
from rclpy.qos import ReliabilityPolicy as RP
from lifecycle_msgs.srv import GetState
from rcl_interfaces.srv import GetParameters
from geometry_msgs.msg import Twist,PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import NavigateToPose
from sensor_msgs.msg import LaserScan,CompressedImage
from std_msgs.msg import String
from tf2_ros import Buffer,TransformListener,TransformException
from .mission_supervisor import sensor_qos
from .operational_contract import exact_endpoint
from .operational_identity import IDENTITY_TOPIC,identity_current,identity_key
from .actuator_freshness import fresh_off, fresh_motion_off
from .pulse_transport import six
from .operational_cycle import zero
from .operational_frame import FrameProof,IMAGE_TOPIC


# Pinned pi_params.yaml profile. Validate the running broadcaster through its
# read-only parameter service before undoing its deliberate 100ms forward stamp.
# This is not permission to accept arbitrary future TF or change the baseline.
ODOM_TF_PROFILE = ('pi_receive_time', -100, 300)
OPERATOR_STARTUP_REASONS = frozenset((
    'WAIT_VALID_GRAPH_AND_DATA', 'WAIT_CURRENT_BOOT_IDENTITY',
    'WAIT_STABLE_STANDBY', 'WAIT_FIRST_OR_FRESH_DECODED_CAMERA_FRAME',
    'READY_PRESS_A'))


def diagnostic_json(value):
    """Invalid sensor numbers must remain blockers, never crash JSON recording."""
    if isinstance(value,float) and not math.isfinite(value):return None
    if isinstance(value,dict):return {k:diagnostic_json(v) for k,v in value.items()}
    if isinstance(value,(list,tuple)):return [diagnostic_json(v) for v in value]
    return value


class HealthCore:
    def __init__(self,session,role):
        self.session=session;self.role=role;self.inputs={};self.since={};self.last=None
        self.raw=FrameProof();self.annotated=FrameProof();self.initial_pose_at=None
        self.io_transitions=deque(maxlen=16);self.io_previous=None;self.io_transition_count=0
        self.io_false_counts={};self.io_first_failure=None;self.io_max_stable_s=0.
        self.localization_identity=None
        self.localization_initial_pose=None
    def update(self,key,value,now):self.inputs[key]=(value,now)
    def value(self,key,now,maxage=.35):
        v,at=self.inputs.get(key,(None,None))
        return v if at is not None and 0<=now-at<=maxage else None
    def report(self,now,*,sources,tf_ok=False,odom_tf_ok=False,action_ready=False,safety_ok=False,lifecycle_ok=False,io_detail=None):
        if self.last is not None and now<self.last:raise ValueError('HEALTH_MONOTONIC_REGRESSION')
        if self.last is not None and now-self.last>.75:self.since.clear()
        self.last=now;identity,at=self.inputs.get('identity',(None,None))
        act,act_at=self.inputs.get('actuator',(None,None))
        actuator=act if isinstance(act,dict) else {}
        io_checks=dict(graph=sources.get('io') is True, safety_route=safety_ok is True,
            odom_tf=odom_tf_ok is True, identity=identity_current(identity,at,now),
            cmd_zero=zero(self.value('cmd',now)), odom_stationary=zero(self.value('odom',now),.02,.05),
            actuator_off=fresh_off(act,act_at,now), actuator_schema=actuator.get('schema_version')==1,
            actuator_mode=actuator.get('mode')=='COMMAND_TELEMETRY_ONLY',
            actuator_pwm_ready=actuator.get('pump_pwm_ready') is True,
            actuator_home=type(actuator.get('home_command_completed_count')) is int and actuator['home_command_completed_count']>=0,
            scan=self.value('scan',now,.5) is True)
        io=all(io_checks.values())
        nav_checks=dict(io_checks, actuator_off=fresh_motion_off(act,act_at,now),
            actuator_pwm_ready=True)
        io_navigation=all(nav_checks.values())
        blockers=tuple(k for k,v in io_checks.items() if not v)
        for key in blockers:self.io_false_counts[key]=self.io_false_counts.get(key,0)+1
        ages={key:None if received is None else round((now-received)*1000,3)
              for key,(_,received) in self.inputs.items() if key in ('cmd','odom','scan','identity','actuator')}
        if blockers!=self.io_previous:
            self.io_transition_count+=1
            event=dict(sample_mono=now,blockers=list(blockers),receive_age_ms=ages)
            self.io_transitions.append(event)
            if blockers and self.io_first_failure is None:self.io_first_failure=event
            self.io_previous=blockers
        camera=sources.get('camera') is True and self.raw.fresh(now)
        pose_value,pose_at=self.inputs.get('pose',(False,None))
        # Localization evidence and current stationary I/O are independent
        # observations. A brief STATE/scan/ACT gap must still block admission,
        # without erasing five seconds of otherwise continuous localization.
        key=identity_key(identity)
        if key!=self.localization_identity or self.initial_pose_at!=self.localization_initial_pose:
            self.since.pop('localization',None)
            self.localization_identity=key
            self.localization_initial_pose=self.initial_pose_at
        localization=(key is not None and sources.get('pose') is True and pose_value is True and self.initial_pose_at is not None
              and pose_at is not None and pose_at>=self.initial_pose_at and tf_ok is True
              and action_ready is True and lifecycle_ok is True)
        pose=io and localization
        yolo=camera and sources.get('annotated') is True and self.annotated.fresh(now)
        op=self.value('operator',now);op=op if isinstance(op,dict) else {}
        live_published=op.get('live_intents_published')
        live_active=op.get('live_intents_active')
        safe_live_state=(live_published is False and live_active in (None,False)
                         or live_published is True and live_active is False)
        operator_checks=dict(source=sources.get('operator') is True,
            schema=op.get('schema_version')==1,scope=op.get('scope')=='R5_SESSION_INTENT_ONLY',
            wait_arm=op.get('phase')=='WAIT_ARM',reason=op.get('reason') in OPERATOR_STARTUP_REASONS,
            no_fault=op.get('fault') is None,open=op.get('closed') is False,unarmed=op.get('armed') is False,
            no_physical=op.get('physical_commands_published') is False,
            no_live_active=safe_live_state,no_motion=op.get('motion_authorized') is False)
        operator_present=all(operator_checks.values())
        operator_ready=operator_present and op.get('reason')=='READY_PRESS_A'
        # Keep the historical `operator` check as strict ARM readiness. Startup
        # consumes `operator_present`; the two meanings must never be conflated.
        checks=dict(io=bool(io),io_navigation=bool(io_navigation),pose_navigation=bool(io_navigation and localization),camera=bool(camera),pose=bool(pose),localization=bool(localization),yolo=bool(yolo),
                    operator=bool(operator_ready),operator_present=bool(operator_present),
                    operator_ready=bool(operator_ready))
        for k,v in checks.items():
            if not v:self.since.pop(k,None)
            else:self.since.setdefault(k,now)
        self.io_max_stable_s=max(self.io_max_stable_s,now-self.since.get('io',now))
        return dict(schema_version=1,session=self.session,role=self.role,sample_mono=now,
            checks=checks,stable_s={k:now-v for k,v in self.since.items()},
            source_faults=[k for k,v in sources.items() if not v],
            operator_diagnostics=diagnostic_json(dict(phase=op.get('phase'),reason=op.get('reason'),
                checks=operator_checks,blockers=[k for k,v in operator_checks.items() if not v],
                startup_readiness=op.get('startup_readiness'))),
            runtime_recovery=diagnostic_json(dict(
                operator_source_current=sources.get('operator') is True and op.get('schema_version')==1
                    and op.get('scope')=='R5_SESSION_INTENT_ONLY',
                operator={k:op.get(k) for k in ('phase','reason','fault','armed','closed','armed_identity','cycle','runtime_rebind')},
                actor=self.value('actor',now) if sources.get('actor') is True else None,
                actuator=(self.value('actuator',now) if sources.get('io') is True
                    or (io_detail or {}).get('graph',{}).get('actuator') is True else None),
                io_sources_current=sources.get('io') is True,
                raw_count=(io_detail or {}).get('raw_count'),pump_count=(io_detail or {}).get('pump_count'))),
            io_diagnostics=diagnostic_json(dict(checks=io_checks,blockers=list(blockers),receive_age_ms=ages,
                odom=self.value('odom',now),cmd=self.value('cmd',now),
                identity_source_age_ms=identity.get('state_age_ms') if isinstance(identity,dict) else None,
                actuator_source_age_ms=actuator.get('receive_age_ms'),
                transition_count=self.io_transition_count,false_sample_counts=dict(self.io_false_counts),
                first_failure=self.io_first_failure,recent_transitions=list(self.io_transitions),
                max_stable_s=self.io_max_stable_s,detail=io_detail)),
            raw_frame=self.raw.report(now),annotated_frame=self.annotated.report(now),
            localization_identity_key=None if self.localization_identity is None else list(self.localization_identity),
            identity_key=None if not isinstance(identity,dict) else [identity.get(k) for k in ('bridge_instance_id','connection_epoch','elf_sha256')],
            initial_pose_seen=self.initial_pose_at is not None,safety_verified=safety_ok,lifecycle_active=lifecycle_ok,commands_published=False)


class StartupHealthNode(Node):
    def __init__(self,session,role):
        super().__init__('r5_startup_health_'+role);self.core=HealthCore(session,role);self.pins={}
        q=sensor_qos();self.expected={}
        def sub(typ,topic,name,callback,key,rel=RP.BEST_EFFORT):
            self.expected[key]=(topic,name,typ.__module__.split('.')[0]+'/msg/'+typ.__name__,rel)
            self.create_subscription(typ,topic,callback,q if rel==RP.BEST_EFFORT else 10)
        sub(String,IDENTITY_TOPIC,'serial_bridge_node',lambda m:self.json('identity',m),'identity')
        sub(String,'/fire_sensors/actuator_state','serial_bridge_node',lambda m:self.json('actuator',m),'actuator')
        sub(Twist,'/cmd_vel','safety_watchdog',lambda m:self.core.update('cmd',six(m),time.monotonic()),'cmd')
        sub(Odometry,'/odom','serial_bridge_node',self.odom,'odom')
        sub(LaserScan,'/scan','scan_qos_relay',self.scan,'scan')
        self.tf=Buffer();self.tf_progress={};self.listener=TransformListener(self.tf,self);self.action=None
        self.control={};self.pending={};self.next_query={}
        self.control_clients={'safety':self.create_client(GetParameters,'/safety_watchdog/get_parameters'),
            'odom_tf_profile':self.create_client(GetParameters,'/odom_to_tf_broadcaster/get_parameters')}
        if role=='laptop':
            self.control_clients.update({name:self.create_client(GetState,'/'+name+'/get_state') for name in
                ('map_server','amcl','controller_server','smoother_server','planner_server','behavior_server','bt_navigator','velocity_smoother')})
        if role=='laptop':
            sub(CompressedImage,'/image_raw/compressed','native_mjpeg_camera',lambda m:self.frame(m,False),'camera')
            sub(CompressedImage,IMAGE_TOPIC,'yolo_observer',lambda m:self.frame(m,True),'annotated')
            sub(String,'/fire_mission/operator_state','fire_robot_operator',lambda m:self.json('operator',m),'operator')
            sub(String,'/fire_mission/r4d_state','fire_pulse_controller',lambda m:self.json('actor',m),'actor')
            sub(PoseWithCovarianceStamped,'/amcl_pose','amcl',self.pose,'pose',RP.RELIABLE)
            self.create_subscription(PoseWithCovarianceStamped,'/initialpose',self.initial_pose,10)
            self.action=ActionClient(self,NavigateToPose,'/navigate_to_pose')
    def json(self,key,msg):
        try:value=json.loads(msg.data)
        except (ValueError,TypeError):value=None
        self.core.update(key,value,time.monotonic())
    def age(self,msg):return (self.get_clock().now().nanoseconds-msg.header.stamp.sec*10**9-msg.header.stamp.nanosec)/1e9
    def odom(self,msg):
        valid=msg.header.frame_id=='odom' and msg.child_frame_id=='base_link' and -.02<=self.age(msg)<=.3
        self.core.update('odom',six(msg.twist.twist) if valid else None,time.monotonic())
    def scan(self,msg):
        self.core.update('scan',len(msg.ranges)>0 and -.02<=self.age(msg)<=.3,time.monotonic())
    def frame(self,msg,annotated):
        (self.core.annotated if annotated else self.core.raw).observe(msg,time.monotonic(),self.get_clock().now().nanoseconds)
    def pose(self,msg):
        q=msg.pose.pose.orientation;values=[q.x,q.y,q.z,q.w,*msg.pose.covariance,msg.pose.pose.position.x,msg.pose.pose.position.y]
        valid=(msg.header.frame_id=='map' and -.02<=self.age(msg)<=.5 and all(math.isfinite(x) for x in values)
               and .9<=sum(x*x for x in (q.x,q.y,q.z,q.w))<=1.1)
        self.core.update('pose',valid,time.monotonic())
    def initial_pose(self,msg):
        q=msg.pose.pose.orientation
        vals=[msg.pose.pose.position.x,msg.pose.pose.position.y,q.x,q.y,q.z,q.w,*msg.pose.covariance]
        if (msg.header.frame_id=='map' and -.02<=self.age(msg)<=1. and all(math.isfinite(x) for x in vals)
                and .9<=sum(x*x for x in (q.x,q.y,q.z,q.w))<=1.1):self.core.initial_pose_at=time.monotonic()
    def source(self,key):
        topic,name,typ,rel=self.expected[key];infos=self.get_publishers_info_by_topic(topic)
        # Mission/operator are owned recovery generations. Reacquire their
        # exact endpoint only after an observed absence; keep all I/O/Nav pins.
        if not infos and key in ('operator','actor'):
            self.pins.pop(key,None)
            self.core.inputs.pop(key,None)
        if not exact_endpoint(infos,name,typ,rel):return False
        gid=bytes(infos[0].endpoint_gid).hex();self.pins.setdefault(key,gid)
        return self.pins[key]==gid
    def poll_services(self):
        now=time.monotonic()
        for key,client in self.control_clients.items():
            pending=self.pending.get(key)
            if pending:
                future,at=pending
                if future.done():
                    self.pending.pop(key)
                    try:
                        result=future.result()
                        if key=='safety':
                            value=([p.type for p in result.values]==[4,4] and
                                [p.string_value for p in result.values]==['/cmd_vel_raw','/cmd_vel'])
                        elif key=='odom_tf_profile':
                            value=([p.type for p in result.values]==[4,2,2] and
                                (result.values[0].string_value,result.values[1].integer_value,
                                 result.values[2].integer_value)==ODOM_TF_PROFILE)
                        else:value=result.current_state.id==3
                        self.control[key]=(value,now)
                    except Exception as exc:
                        self.control[key]=(False,now);self.get_logger().warning('startup query '+key+': '+str(exc))
                elif now-at>=3.:
                    client.remove_pending_request(future);future.cancel();self.pending.pop(key)
                    self.control[key]=(False,now)
                continue
            if now<self.next_query.get(key,0):continue
            self.next_query[key]=now+3.
            try:
                if not client.service_is_ready():continue
                request=GetParameters.Request() if key in ('safety','odom_tf_profile') else GetState.Request()
                if key=='safety':request.names=['cmd_vel_input_topic','cmd_vel_output_topic']
                elif key=='odom_tf_profile':request.names=['tf_stamp_source','tf_stamp_offset_ms','odom_stale_timeout_ms']
                self.pending[key]=(client.call_async(request),now)
            except Exception as exc:
                self.control[key]=(False,now);self.get_logger().warning('startup query '+key+': '+str(exc))
    def sample(self):
        self.poll_services();now=time.monotonic()
        def current(key):
            valid,at=self.control.get(key,(False,None))
            return valid is True and at is not None and 0<=now-at<=15.
        graph={k:self.source(k) for k in ('identity','actuator','cmd','odom','scan')}
        graph.update(raw_owner_absent=not self.get_publishers_info_by_topic('/cmd_vel_raw'),
                     pump_owner_absent=not self.get_publishers_info_by_topic('/pump_cmd'))
        io=all(graph.values());tf_detail={}
        profile_ok=current('odom_tf_profile')
        def transform(parent,child):
            try:
                # Odom naturally arrives a few ms before this callback: asking
                # exactly now would extrapolate into the future indefinitely.
                value=self.tf.lookup_transform(parent,child,rclpy.time.Time())
                stamp=value.header.stamp.sec*10**9+value.header.stamp.nanosec
                if stamp<=0:return False
                key=(parent,child);old,at=self.tf_progress.get(key,(None,None))
                if old is not None and stamp<old:return False
                if stamp!=old:self.tf_progress[key]=(stamp,now);at=now
                if parent=='odom':
                    stamp_age=(self.get_clock().now().nanoseconds-stamp)/1e9
                    # Broadcaster: stamp = receive_time - offset. Thus the
                    # actual source age is stamp_age - offset, not stamp_age.
                    age=stamp_age-ODOM_TF_PROFILE[1]/1000.
                    tf_detail.update(stamp_age_ms=stamp_age*1000,source_age_ms=age*1000,
                        progress_age_ms=(now-at)*1000,profile_verified=profile_ok)
                    return profile_ok and -.02<=age<=.3 and 0<=now-at<=.3
                # Accepted R4 map->odom readiness uses newly received TF after
                # manual pose. AMCL can deliberately stamp ahead by its
                # transform tolerance, so do not reject that as future odom.
                return (self.core.initial_pose_at is not None and at>=self.core.initial_pose_at
                        and 0<=now-at<=1.)
            except TransformException:return False
        return self.core.report(now,sources=dict(io=io,**{k:self.source(k) for k in ('camera','annotated','pose','operator','actor') if k in self.expected}),
            tf_ok=transform('map','odom'),odom_tf_ok=transform('odom','base_link'),
            action_ready=self.action is not None and self.action.server_is_ready(),safety_ok=current('safety'),
            lifecycle_ok=all(current(k) for k in self.control_clients if k not in ('safety','odom_tf_profile')),
            io_detail=dict(graph=graph,odom_tf=tf_detail,odom_tf_profile_verified=profile_ok,
                raw_count=len(self.get_publishers_info_by_topic('/cmd_vel_raw')),
                pump_count=len(self.get_publishers_info_by_topic('/pump_cmd')),
                control={k:dict(valid=v,age_ms=(now-at)*1000) for k,(v,at) in self.control.items()}))


def main(args=None):
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('--out',type=Path,required=True)
    parser.add_argument('--session',required=True);parser.add_argument('--role',choices=['pi','laptop'],required=True)
    opts,ros_args=parser.parse_known_args(args);stop=[False];node=None
    previous={s:signal.getsignal(s) for s in (signal.SIGINT,signal.SIGTERM)}
    for s in previous:signal.signal(s,lambda *_:stop.__setitem__(0,True))
    try:
        rclpy.init(args=ros_args,signal_handler_options=SignalHandlerOptions.NO);node=StartupHealthNode(opts.session,opts.role)
        while rclpy.ok() and not stop[0]:
            rclpy.spin_once(node,timeout_sec=.05)
            data=node.sample();temp=opts.out.with_suffix('.tmp')
            temp.write_text(json.dumps(data,allow_nan=False)+'\n');os.replace(temp,opts.out)
    finally:
        if node is not None:node.destroy_node()
        if rclpy.ok():rclpy.shutdown()
        for s,h in previous.items():signal.signal(s,h)
