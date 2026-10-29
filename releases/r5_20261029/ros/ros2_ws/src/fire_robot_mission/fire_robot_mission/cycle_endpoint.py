"""Optional per-node release adapter; no new motion/pump publisher or ROS node.

Every ACK follows current telemetry and actual local publisher destruction.
COMMITTED follows a real idle-core replacement with configured parameters,
not an ACK synthesized by the operator or coordinator.
"""
import copy
import json
import time

from rclpy.qos import QoSProfile,ReliabilityPolicy as RP,DurabilityPolicy
from std_msgs.msg import String,Bool
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatusArray

from .cycle_release import CycleReleaseParticipant,CONTROL_TOPIC
from .operational_identity import IDENTITY_TOPIC,identity_current,identity_key,IdentityContinuity
from .operational_contract import OPERATOR_NODE,exact_endpoint,valid_gid
from .operational_cycle import finite,zero,NavigationSnapshot
from .actuator_freshness import fresh_off, fresh_motion_off


class CycleEndpoint:
    def __init__(self,node,role,core_attr='core'):
        self.node=node;self.role=role;self.core_attr=core_attr
        node.declare_parameter('enable_cycle_protocol',False)
        self.enabled=node.get_parameter('enable_cycle_protocol').value is True
        self.participant=CycleReleaseParticipant(role);self.pins={};self.graph_ok=False
        self.continuity=IdentityContinuity();self.identity_hold=False;self.resume_wait=False;self.last_payload=None;self.arm_wait_since=None
        self.session_hold=False;self.controlled_hold_seen=False;self.actor_off_wait=None;self.actor_off_esp=None
        self.controlled_hold_at=None
        self.nav_pause=False;self.nav_pause_at=None;self.nav_cancel_uuid=None
        self.nav_historical_goal=None
        self.nav_session_valid=False;self.nav_cancel_records={};self.nav_cancel_report=None;self.nav_cancel_targets=set()
        if role=='guard':
            from .cycle_blocked_goal import BlockedGoalCancel
            self.blocked_goals=BlockedGoalCancel()
        self.expected={
            CONTROL_TOPIC:(OPERATOR_NODE,'std_msgs/msg/String',RP.RELIABLE),
            IDENTITY_TOPIC:('serial_bridge_node','std_msgs/msg/String',RP.BEST_EFFORT),
            '/fire_sensors/actuator_state':('serial_bridge_node','std_msgs/msg/String',RP.BEST_EFFORT),
            '/cmd_vel':('safety_watchdog','geometry_msgs/msg/Twist',RP.BEST_EFFORT),
            '/odom':('serial_bridge_node','nav_msgs/msg/Odometry',RP.BEST_EFFORT),
            '/navigate_to_pose/_action/status':('bt_navigator','action_msgs/msg/GoalStatusArray',RP.RELIABLE),
            '/fire_mission/nav2_cancel_guard_state':('nav2_cancel_guard','std_msgs/msg/String',RP.BEST_EFFORT),
        }
        if not self.enabled:return
        if role in ('gateway','guard'):
            topic='/fire_mission/operator_state'
            self.expected[topic]=(OPERATOR_NODE,'std_msgs/msg/String',RP.BEST_EFFORT)
            node.create_subscription(String,topic,lambda msg:self.on_json('operator_session',msg),
                QoSProfile(depth=1,reliability=RP.BEST_EFFORT))
        for topic,name in (('/fire_mission/live_nav_stop_state','live_nav_stop_gateway'),
                ('/fire_mission/live_alignment_state','live_alignment_takeover'),
                ('/fire_mission/ir_approach_live_state','ir_approach_live'),
                ('/fire_mission/state','fire_mission_supervisor')):
            self.expected[topic]=(name,'std_msgs/msg/String',RP.BEST_EFFORT)
        for topic in self.expected:
            if node.resolve_topic_name(topic)!=topic:raise ValueError('CYCLE_PROTOCOL_REMAP_FORBIDDEN')
        qos=QoSProfile(depth=1,reliability=RP.BEST_EFFORT)
        rel=QoSProfile(depth=1,reliability=RP.RELIABLE)
        status_qos=QoSProfile(depth=1,reliability=RP.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        node.create_subscription(String,CONTROL_TOPIC,self.on_control,rel)
        for name,topic in [('identity',IDENTITY_TOPIC),('actuator','/fire_sensors/actuator_state'),
                           ('guard','/fire_mission/nav2_cancel_guard_state')]:
            node.create_subscription(String,topic,lambda msg,k=name:self.on_json(k,msg),qos)
        node.create_subscription(Twist,'/cmd_vel',lambda m:self.participant.update('cmd',self.six(m),time.monotonic()),qos)
        node.create_subscription(Odometry,'/odom',self.on_odom,qos)
        node.create_subscription(GoalStatusArray,'/navigate_to_pose/_action/status',self.on_status,status_qos)

    @staticmethod
    def six(t):return (t.linear.x,t.linear.y,t.linear.z,t.angular.x,t.angular.y,t.angular.z)

    def _source(self,topic):
        infos=self.node.get_publishers_info_by_topic(topic);name,typ,rel=self.expected[topic]
        if not exact_endpoint(infos,name,typ,rel):
            if len(infos)==1 and valid_gid(infos[0]):
                i=infos[0]
                if ((i.node_name,i.node_namespace)==('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_')
                        and i.topic_type==typ and i.qos_profile.reliability==rel
                        and self.pins.get(topic)==bytes(i.endpoint_gid).hex()):return True
                if topic in self.pins and self.pins[topic]!=bytes(i.endpoint_gid).hex():
                    self.participant.error='CYCLE_SOURCE_GID_CHANGED:'+topic
            return False
        gid=bytes(infos[0].endpoint_gid).hex()
        if topic in self.pins and self.pins[topic]!=gid:
            self.participant.error='CYCLE_SOURCE_GID_CHANGED:'+topic;return False
        self.pins.setdefault(topic,gid)
        return True

    def on_control(self,msg):
        if not self._source(CONTROL_TOPIC):return
        try:value=json.loads(msg.data)
        except (ValueError,TypeError):value=None
        self.participant.receive(value,time.monotonic())

    def on_json(self,name,msg):
        if name=='operator_session' and not self._source('/fire_mission/operator_state'):return
        try:value=json.loads(msg.data)
        except (ValueError,TypeError):value=None
        self.participant.update(name,value,time.monotonic())

    def on_status(self,msg):
        if self.role in ('gateway','guard') and not self._source('/navigate_to_pose/_action/status'):return
        try:rows=[(bytes(x.goal_info.goal_id.uuid).hex(),x.status) for x in msg.status_list]
        except (ValueError,TypeError,AttributeError):rows=None
        self.participant.update('status',rows,time.monotonic())

    def on_odom(self,msg):
        q=msg.pose.pose.orientation
        age=(self.node.get_clock().now().nanoseconds-(msg.header.stamp.sec*10**9+msg.header.stamp.nanosec))/1e9
        good=(msg.header.frame_id=='odom' and msg.child_frame_id=='base_link'
              and all(finite(v) for v in (q.x,q.y,q.z,q.w))
              and .9<=q.x*q.x+q.y*q.y+q.z*q.z+q.w*q.w<=1.1 and -.05<=age<=.3)
        self.participant.update('odom',self.six(msg.twist.twist) if good else None,time.monotonic())

    def stage_allowed(self,now):
        if not self.enabled:return True
        p=self.participant
        identity,at=p.inputs.get('identity',(None,None))
        return (not self.identity_hold and not self.continuity.held and self.graph_ok and identity_current(identity,at,now) and p.context is not None
                and identity_key(identity)==p.context.identity
                and p.error is None and p.phase=='ACTIVE'
                and p.request is not None and p.request.get('phase')=='ACTIVE'
                and p.request_at is not None and 0<=now-p.request_at<=.35
                and not (self.role=='actor' and isinstance(p.request.get('cancel_unused_pulse'),dict)))

    def _withdraw(self,now,*,force=False):
        core=getattr(self.node,self.core_attr)
        if self.role=='actor':
            core.update('operator',False,now)
            self.node.transport.pending_p=None
            if self.participant.closing:self.node.stop_requested=True
            pub=self.node.pub
            if pub is not None:
                pub.publish(Bool(data=False))
                if self.actor_off_wait is None:
                    self.actor_off_wait=now
                    a=self.participant.get('actuator',now)
                    self.actor_off_esp=a.get('esp_ms') if isinstance(a,dict) else None
                if self._actor_off_release_ready(now):
                    self.node.destroy_publisher(pub);self.node.pub=None;self.actor_off_wait=None
        elif self.role=='guard' and not force:
            # Exact CancelGoal may still have to settle; it cannot drive motion.
            pass
        elif hasattr(core,'update_arm'):
            core.update_arm(False,now)

    def _actor_off_release_ready(self,now):
        # Releasing an OFF publisher is not a cycle ACK. Do not make it wait
        # for Nav2 cancel/clear: those gates themselves wait for pump release.
        # Keep the endpoint through a NEW device OFF/home sample after STOP.
        p=self.participant;core=getattr(self.node,self.core_attr)
        a,at=p.inputs.get('actuator',(None,None));identity,it=p.inputs.get('identity',(None,None))
        if (not self.graph_ok or p.context is None or not identity_current(identity,it,now)
                or identity_key(identity)!=p.context.identity or not finite(self.actor_off_wait)
                or not finite(at) or at<=self.actor_off_wait or not fresh_off(a,at,now)
                or type(a.get('esp_ms')) is not int or not 0<=a['esp_ms']<2**32
                or a['esp_ms']==self.actor_off_esp
                or a.get('schema_version')!=1 or a.get('mode')!='COMMAND_TELEMETRY_ONLY'
                or a.get('pump_pwm_ready') is not True
                or len(self.node.get_publishers_info_by_topic('/cmd_vel_raw'))!=0
                or not zero(p.get('cmd',now)) or not zero(p.get('odom',now),.02,.05)):
            return False
        home=getattr(core,'start_home',None)
        if getattr(core,'used',False) and getattr(core,'actuation_started',True) and (type(home) is not int
                or type(a.get('home_command_completed_count')) is not int
                or a['home_command_completed_count']<=home):return False
        return True

    def _session_hold_current(self,now):
        p=self.participant;request=p.request or {};tag=request.get('session_hold')
        return (p.context is not None and p.error is None and self.graph_ok and p.phase=='ACTIVE'
            and request.get('phase')=='ACTIVE' and finite(p.request_at) and 0<=now-p.request_at<=.35
            and isinstance(tag,dict) and type(tag.get('schema_version')) is int and tag['schema_version']==1
            and tag.get('cause') in ('SAME_BOOT_IDENTITY_RECOVERY','RECOVERABLE_INPUT_GAP'))

    def before_step(self,now):
        if not self.enabled:return
        core=getattr(self.node,self.core_attr)
        if self.role=='guard':core.r5_retiring=self.participant.closing
        core.r5_cycle_context=(self.participant.context.as_dict()
            if self.participant.context is not None and self.participant.phase!='COMMITTED' else None)
        if hasattr(core,'pump'):core.pump.r5_cycle_context=core.r5_cycle_context
        self.graph_ok=all(self._source(t) for t in self.expected)
        p=self.participant
        expired=p.context is not None and (p.request_at is None or not 0<=now-p.request_at<=.35)
        bound=getattr(core,'goal_uuid',getattr(core,'goal',None))
        self.nav_historical_goal=None
        if (self.role=='gateway' and core.enabled and p.error is None and not expired
                and p.context is not None and p.phase=='ACTIVE'
                and p.request is not None and p.request.get('phase')=='ACTIVE'
                and p.context.nav_goal_uuid is None and bound is not None
                and core._normal_terminal_result(now) in (4,5,6) and self.graph_ok):
            try:
                rows,received=p.inputs.get('status',(None,None))
                status=NavigationSnapshot.parse(rows,received)
                terminal=[value for uid,value in status.rows if uid==bound]
                historical=(received<=now and status.active_uuid is None
                    and terminal==[core._normal_terminal_result(now)])
            except (ValueError,TypeError):historical=False
            if historical:
                # A terminal Nav2 UUID may still be bound while its publisher
                # drains. It is history, not a foreign active goal. The normal
                # no-goal engine must first zero/release; never reset its owner.
                self.nav_historical_goal=(p.context,bound)
                if (core.released and not p.stop_errors(p.context,now,self.graph_ok,
                        len(self.node.get_publishers_info_by_topic('/cmd_vel_raw')),
                        len(self.node.get_publishers_info_by_topic('/pump_cmd')))):
                    core._start_next_goal(None);bound=None;self.nav_historical_goal=None
        if p.context is not None and p.phase!='COMMITTED':
            if (bound is not None and bound!=p.context.nav_goal_uuid
                    and self.nav_historical_goal!=(p.context,bound)):p.error='LOCAL_CORE_GOAL_MISMATCH'
            cycle_id=getattr(core,'cycle_id',None)
            if p.context.nav_goal_uuid is None and cycle_id not in (None,p.context.cycle_id):
                p.error='LOCAL_CORE_CYCLE_MISMATCH'
        identity,at=p.inputs.get('identity',(None,None))
        identity_ok=identity_current(identity,at,now)
        if p.context is not None:
            if self.continuity.bound is None:self.continuity.bound=p.context.identity
            key=identity_key(identity)
            if key is not None and key!=p.context.identity:p.error='CYCLE_BOOT_EPOCH_CHANGED'
            identity_ok=identity_ok and key==p.context.identity
        actuator,act_at=p.inputs.get('actuator',(None,None))
        nav_session=p.get('operator_session',now) or {}
        navigation_only=(self.role in ('gateway','guard') and (p.context is None or p.phase=='COMMITTED')
            and nav_session.get('navigation_only') is True)
        off_check=fresh_motion_off if navigation_only else fresh_off
        if self.role=='gateway':core.r5_navigation_only=navigation_only
        stopped=(zero(p.get('cmd',now)) and zero(p.get('odom',now),.02,.05)
            and isinstance(actuator,dict) and finite(act_at) and off_check(actuator,act_at,now))
        authority=(p.context is not None or getattr(core,'arm',False)
            or getattr(self.node,'_raw_publisher',None) is not None)
        session=p.get('operator_session',now)
        # A navigation-only guard has no motion arm or fire-cycle context.
        # Bind its cancel-only lease to the authenticated operator's current
        # device identity; subsequent epoch changes remain permanent faults.
        cancel_authority=(self.role=='guard' and (p.context is None or p.phase=='COMMITTED') and self.graph_ok
            and p.error is None and identity_ok and isinstance(session,dict)
            and session.get('scope')=='R5_SESSION_INTENT_ONLY'
            and type(session.get('schema_version')) is int and session['schema_version']==1 and session.get('armed') is True
            and session.get('closed') is False and session.get('fault') is None
            and session.get('cycle') is None
            and session.get('armed_identity')==list(identity_key(identity) or ()))
        ready=self.continuity.observe(identity,at,now,armed=authority or cancel_authority,stopped=stopped)
        if self.continuity.fault:p.error=p.error or self.continuity.fault
        self.identity_hold=authority and not ready and p.error is None and not expired and self.graph_ok
        if self.identity_hold:self.resume_wait=True;self.arm_wait_since=None
        blocked=(p.error is not None or expired or not self.graph_ok or not identity_ok or not ready
                 or p.context is None and self.role in ('alignment','ir_live','actor'))
        session=p.get('operator_session',now)
        standalone=(self.role in ('gateway','guard') and (p.context is None or p.phase=='COMMITTED') and not p.error and self.graph_ok
            and isinstance(session,dict) and session.get('scope')=='R5_SESSION_INTENT_ONLY'
            and type(session.get('schema_version')) is int and session['schema_version']==1 and session.get('armed') is True
            and session.get('closed') is False and session.get('fault') is None
            and session.get('cycle') is None and self.continuity.bound is not None
            and session.get('armed_identity')==list(self.continuity.bound))
        self.nav_pause=bool(standalone and (session.get('identity_hold') is True or session.get('graph_hold') is True
            or session.get('input_hold') is True))
        self.nav_session_valid=bool(standalone)
        if self.role=='gateway':
            core.r5_interrupted_fire_bypass=bool(standalone and identity_ok and ready
                and (session.get('retry_fire_clear_required') is True or navigation_only))
        if (self.role=='gateway' and standalone and identity_ok and ready and stopped
                and session.get('recovery_clear_policy') is True
                and getattr(self.node,'_raw_publisher',None) is None
                and not self.node.get_publishers_info_by_topic('/pump_cmd')
                and core.fire_latched and not core.ever_forwarded
                and core.stop_reason=='FIRE_ACQUIRED' and core._active_goal(now)==core.goal_uuid):
            # A NEW Nav2 server can receive the user's new goal before the
            # recovered operator completes ARM. Old visual fire must not
            # permanently latch a goal that has never forwarded any motion.
            # This does not clear a used fire cycle or an actuator history.
            core.fire_latched=False;core.stop_latched=False;core.stop_reason=''
            core.stop_since=None;core.released=False
        if self.nav_pause or (self.role=='gateway' and (p.context is None or p.phase=='COMMITTED') and self.identity_hold):self.nav_pause_at=now
        if not standalone or session.get('navigation_cancel_uuid') is None:self.nav_cancel_report=None
        if standalone and session.get('navigation_cancel_uuid') is not None:
            self.nav_cancel_uuid=session['navigation_cancel_uuid']
            from .operational_cycle import valid_uuid
            if valid_uuid(self.nav_cancel_uuid):self.nav_cancel_targets.add(self.nav_cancel_uuid)
        if self.role=='gateway' and standalone and identity_ok and ready and stopped and not core.fire_latched:
            terminal=core._normal_terminal_result(now)
            if (core.goal_uuid in self.nav_cancel_targets and terminal in (4,5,6)
                    and core.stop_reason in ('NAV_INPUT_OR_TOPOLOGY_LOST','ARMING_INPUT_LOST')):
                core.normal_terminal=terminal
                core.stop_reason={4:'NAV_GOAL_SUCCEEDED',5:'NAV_GOAL_CANCELED',6:'NAV_GOAL_ABORTED'}[terminal]
            if (finite(self.nav_pause_at) and now-self.nav_pause_at<=1.
                    and core.stop_reason=='NAV_INPUT_OR_TOPOLOGY_LOST'
                    and core._active_goal(now)==core.goal_uuid and session.get('identity_hold') is False
                    and session.get('graph_hold') is not True
                    and session.get('input_hold') is not True
                    and session.get('navigation_cancel_uuid') is None and core.arm):
                # Only a same-epoch planned pause of the SAME active goal.
                # Normal navigation preflight is still evaluated before output.
                core.stop_latched=False;core.stop_reason='';core.stop_since=None;core.released=False
                self.nav_pause_at=None
        if self.role=='ir_live':
            request=p.request or {};stop=request.get('ir_stop_request')
            core.r5_stop_request=(stop if not blocked and not p.closing and p.phase=='ACTIVE'
                and request.get('phase')=='ACTIVE' and not expired
                and isinstance(stop,dict) and type(stop.get('schema_version')) is int and stop['schema_version']==1
                and stop.get('cause') in ('PERCEPTION_HOLD','I_BUDGET_EXHAUSTED')
                and type(stop.get('step_count')) is int and stop['step_count']==core.step_count
                and finite(stop.get('budget_started_at')) and stop['budget_started_at']>0
                and p.context is not None and bound==p.context.nav_goal_uuid else None)
        if self.role=='actor':
            cancel=(p.request or {}).get('cancel_unused_pulse')
            if (not blocked and p.phase=='ACTIVE' and isinstance(cancel,dict)
                    and set(cancel)=={'baseline_index'}):
                if getattr(core,'used',False):self.node.stop_requested=True
                else:self.node.transport.cancel_unused(cancel['baseline_index'],now)
                self._withdraw(now,force=True)
        if self.role=='actor' and not getattr(core,'used',False):
            request=core.get('stationary_request',now)
            if (isinstance(request,dict) and request.get('active') is False
                    and (getattr(core.stationary,'token',None) is not None or self.actor_off_wait is not None)):
                self._withdraw(now,force=True)
        self.session_hold=self._session_hold_current(now)
        if self.session_hold:
            self.controlled_hold_seen=True
            self.controlled_hold_at=now
            if self.role=='actor':self._withdraw(now,force=True)
        if (self.role=='guard' and p.context is not None and not p.closing
                and not blocked and stopped and getattr(core,'failure',None) is not None
                and getattr(core,'complete',False) and getattr(core,'goal_uuid',None)==p.context.nav_goal_uuid
                and (p.context.cycle_id,p.context.nav_goal_uuid,p.context.identity) in p.cancel_seen
                and not set(p.stop_errors(p.context,now,self.graph_ok,
                    len(self.node.get_publishers_info_by_topic('/cmd_vel_raw')),
                    len(self.node.get_publishers_info_by_topic('/pump_cmd')))) - {'WAIT_REAL_GOAL_CANCELED'}):
            # Preserve a PROVEN exact cancellation across a downstream task
            # interruption. Fresh same-epoch STOP/graph/identity and zero
            # owners are required; never resend or fabricate a cancel.
            core.failure=None
        if blocked or p.closing or p.phase=='COMMITTED' and self.role in ('actor','alignment','ir_live','guard'):
            if not (self.identity_hold and self.role in ('gateway','guard','alignment','ir_live') and not p.closing):
                self._withdraw(now,force=blocked or p.phase=='COMMITTED')

    def pause_tick(self,now):
        """Retain one motion owner at zero while same-epoch proof recovers.

        Only these existing nodes can publish zero through their own endpoint.
        Their actual R4 engines are suspended, not reset or fed fake status.
        X/closing and hard faults always fall through to normal STOP/release.
        """
        if not self.enabled or self.role not in ('gateway','guard','alignment','ir_live'):return False
        p=self.participant;core=getattr(self.node,self.core_attr)
        if (p.context is None or p.phase=='COMMITTED') and self.role in ('gateway','guard'):
            if p.error or not self.graph_ok or not self.nav_session_valid:return False
            if self.role=='guard':
                self._navigation_cancel(now)
                return False
            # X withdraws navigation even during a planned identity hold.
            # The normal gateway engine must drain zero and release its owner
            # before the guard can issue the exact, cancel-only action.
            if (p.get('operator_session',now) or {}).get('navigation_cancel_uuid') is not None:return False
            arm_at=getattr(core,'arm_at',None)
            arm_fresh=core.arm and finite(arm_at) and 0<=now-arm_at<=.4
            waiting=(self.identity_hold or self.nav_pause or finite(self.nav_pause_at) and not arm_fresh)
            if not waiting:return False
            if getattr(self.node,'_raw_publisher',None) is not None:self.node._raw_publisher.publish(Twist())
            payload=copy.deepcopy(self.last_payload) if self.last_payload else dict(state='WAIT_IDENTITY')
            payload.update(r5_identity_hold=True,blockers=['WAIT_FRESH_SAME_BOOT_AND_OPERATOR_HOLD'],
                command=dict(linear_x_m_s=0.,angular_z_rad_s=0.))
            payload=self.decorate(payload,now,remember=False)
            self.node._state_publisher.publish(String(data=json.dumps(payload,allow_nan=False)))
            return True
        if (self.role=='guard' and (p.closing or p.phase=='ACTIVE') and p.error is None and self.graph_ok
                and finite(p.request_at) and 0<=now-p.request_at<=.35):
            waiting=self.blocked_goals.step(self,now)
            if waiting is not None:
                payload=copy.deepcopy(self.last_payload) if self.last_payload else {}
                payload.update(blockers=[waiting['state']],blocked_goal_cancel=waiting)
                payload=self.decorate(payload,now,remember=False)
                self.node.publisher.publish(String(data=json.dumps(payload,allow_nan=False)))
                return True
        if p.error or p.closing or p.phase=='COMMITTED':self.resume_wait=False;return False
        if (not self.graph_ok or p.context is None or not finite(p.request_at)
                or not 0<=now-p.request_at<=.35):
            # A transient identity pause cannot outlive control authority or
            # hide graph loss. Let the real engine STOP/release normally.
            self.resume_wait=False;return False
        if self.session_hold:self.resume_wait=True
        arm_at=getattr(core,'arm_at',getattr(core,'arm_received',None))
        arm_fresh=(getattr(core,'arm',False) and finite(arm_at) and 0<=now-arm_at<=.4)
        owned=getattr(self.node,'_raw_publisher',None) is not None
        need_arm=self.resume_wait and (owned or self.role=='guard') and not arm_fresh
        if need_arm and not self.identity_hold and not self.session_hold and self.role=='ir_live':
            if self.arm_wait_since is None:self.arm_wait_since=now
            if now-self.arm_wait_since>=.4:
                # Original I may have expired during the hold. Do not keep
                # the raw owner forever waiting for an arm that cannot return.
                core.r5_identity_release=True;self.resume_wait=False
                self.arm_wait_since=None;return False
        if not self.identity_hold and not self.session_hold and not need_arm:
            if self.resume_wait:
                # Paused time remains in the session's original I budget;
                # avoid integrating it as a turn angle or progress stall.
                if hasattr(core,'last_step_at'):core.last_step_at=now
                if hasattr(core,'last_progress_at') and core.last_progress_at is not None:core.last_progress_at=now
            self.resume_wait=False;return False
        if owned:self.node._raw_publisher.publish(Twist())
        payload=copy.deepcopy(self.last_payload) if self.last_payload else dict(schema_version=1,
            enabled=getattr(core,'enabled',False),state='WAIT_IDENTITY',goal_uuid=getattr(core,'goal_uuid',None))
        payload.update(r5_identity_hold=True,command=dict(linear_x_m_s=0.,angular_z_rad_s=0.),
            publisher_owned=owned,blockers=['WAIT_FRESH_SAME_BOOT_IDENTITY'])
        if isinstance(payload.get('no_goal_proof'),dict):
            payload['no_goal_proof'].update(valid=False,stationary_released=False,blockers=['WAIT_FRESH_SAME_BOOT_IDENTITY'])
        # decorate checks publishers and identity; it cannot ACK a reset here.
        payload=self.decorate(payload,now,remember=False)
        publisher=(self.node.publisher if self.role=='guard' else
                   self.node.state_publisher if self.role=='ir_live' else self.node._state_publisher)
        publisher.publish(String(data=json.dumps(payload,allow_nan=False,separators=(',',':'))))
        return True

    def _navigation_cancel(self,now):
        """Cancel a standalone Nav2 goal once; never manufacture terminal proof.

        Nav2 status is an event stream, not a heartbeat. A retained snapshot
        is usable only with its still-pinned source, current operator lease,
        current device identity and fresh independent STOP/OFF evidence.
        """
        from .operational_cycle import NavigationSnapshot,valid_uuid
        from .nav2_cancel_guard import make_cancel_request
        if getattr(getattr(self.node,self.core_attr),'enabled',False) is not True:return
        p=self.participant;session=p.get('operator_session',now) or {}
        request=session.get('navigation_cancel_uuid')
        if not valid_uuid(request) or request!=self.nav_cancel_uuid:return
        report=dict(goal_uuid=request,state='WAIT_VALID_NAV_STATUS',status_policy='PINNED_EVENT_STREAM')
        self.nav_cancel_report=report
        try:
            rows,at=p.inputs.get('status',(None,None));status=NavigationSnapshot.parse(rows,at)
            if at>now:return
        except (ValueError,TypeError):return
        report.update(status_receive_age_ms=(now-at)*1000)
        identity,it=p.inputs.get('identity',(None,None))
        if (self.continuity.held or not identity_current(identity,it,now)
                or identity_key(identity)!=self.continuity.bound):
            report['state']='WAIT_CURRENT_IDENTITY';return
        record=self.nav_cancel_records.get(request)
        if record is not None:
            report.update(send_count=1,cancel_accepted=record.get('accepted'))
            future=record.get('future')
            if future is None:
                report['state']='CANCEL_SEND_FAILED';return
            if future.done() and record.get('accepted') is None:
                try:
                    response=future.result()
                    ids=[bytes(g.goal_id.uuid).hex() for g in response.goals_canceling]
                    record['accepted']=response.return_code==0 and ids==[request]
                except Exception as exc:
                    record['accepted']=False
                    self.node.get_logger().error(f'Navigation-only CancelGoal response failed for {request}: {exc}')
                report['cancel_accepted']=record['accepted']
            actual=[v for u,v in status.rows if u==request]
            if actual in ([4],[5],[6]):
                report.update(state='NAV_GOAL_TERMINAL',actual_terminal=actual[0]);return
            report['state']=('CANCEL_REJECTED_WAIT_TERMINAL' if record.get('accepted') is False else
                'CANCEL_UNCONFIRMED' if now-record['since']>3. else 'WAIT_REAL_NAV_TERMINAL')
            return
        if status.active_uuid!=request:
            report['state']='WAIT_EXACT_ACTIVE_UUID';return
        a,aat=p.inputs.get('actuator',(None,None))
        if (not zero(p.get('cmd',now)) or not zero(p.get('odom',now),.02,.05)
                or len(self.node.get_publishers_info_by_topic('/cmd_vel_raw'))!=0
                or len(self.node.get_publishers_info_by_topic('/pump_cmd'))!=0
                or not finite(aat) or not (fresh_motion_off(a,aat,now) if session.get('navigation_only') is True else fresh_off(a,aat,now))
                or a.get('schema_version')!=1 or a.get('mode')!='COMMAND_TELEMETRY_ONLY'
                or a.get('pump_pwm_ready') is not True and session.get('navigation_only') is not True):
            report['state']='WAIT_STOP_OFF_OWNER_RELEASE';return
        if not self.node.cancel_client.service_is_ready():
            report['state']='WAIT_CANCEL_SERVICE';return
        # Record before sending. A response failure or replay of this UUID
        # cannot cause another action call, including across a fire cycle.
        record=dict(future=None,since=now,accepted=None)
        self.nav_cancel_records[request]=record
        try:
            record['future']=self.node.cancel_client.call_async(make_cancel_request(request))
            report.update(state='CANCEL_SENT',send_count=1)
            self.node.get_logger().info(f'Navigation-only CancelGoal sent for UUID {request}')
        except Exception as exc:
            report.update(state='CANCEL_SEND_FAILED',send_count=1)
            self.node.get_logger().error(f'Navigation-only CancelGoal send failed for {request}: {exc}')

    def _reset(self,context):
        node=self.node;core=getattr(node,self.core_attr)
        if getattr(node,'_raw_publisher',None) is not None or getattr(node,'pub',None) is not None:
            raise RuntimeError('LOCAL_PUBLISHER_STILL_OWNED')
        configuration=getattr(core,'r5_initialization',None)
        if configuration is None:raise RuntimeError('CORE_CONFIGURATION_NOT_CAPTURED')
        new=type(core)(**configuration)
        old_used=set(getattr(core,'used_goal_uuids',()))
        if context.nav_goal_uuid:old_used.add(context.nav_goal_uuid)
        if hasattr(new,'used_goal_uuids'):new.used_goal_uuids.update(old_used)
        if hasattr(new,'pump') and context.nav_goal_uuid:new.pump.used_goal_uuids.add(context.nav_goal_uuid)
        if hasattr(core,'prespray_enabled'):new.prespray_enabled=core.prespray_enabled
        if hasattr(core,'prespray'):new.prespray.enabled=core.prespray.enabled
        # Nav2 status is an event stream. Every consumer must retain the
        # source-pinned last event across a cycle-local core replacement;
        # waiting for Nav2 to republish strands a subsequent no-goal fire.
        retained_status=None
        if self._source('/navigate_to_pose/_action/status'):
            if self.role in ('gateway','guard'):
                rows,received=self.participant.inputs.get('status',(None,None))
            elif self.role=='mission':
                rows,received=core._goal_status_records,core._goal_status_received
            elif self.role=='alignment':
                rows,received=core.statuses,time.monotonic()
            elif self.role=='ir_live':
                rows,received=getattr(node,'_statuses',None),time.monotonic()
            elif self.role=='actor':
                rows,received=core.inputs.get('status',(None,None))
            else:rows,received=None,None
            try:
                pairs=[(item[0],item[1]) for item in rows]
                snapshot=NavigationSnapshot.parse(pairs,received)
            except (ValueError,TypeError,IndexError):pass
            else:
                retained_status=(snapshot.rows,rows,received)
                if self.role in ('gateway','guard'):new.update_statuses(snapshot.rows,received)
                elif self.role=='mission':new.update_goal_statuses(snapshot.rows,received)
                elif self.role=='alignment':new.update_statuses(snapshot.rows)
                elif self.role=='actor':new.update('status',copy.deepcopy(rows),received)
        setattr(node,self.core_attr,new)
        self.session_hold=self.controlled_hold_seen=False
        self.controlled_hold_at=None
        self.actor_off_wait=self.actor_off_esp=None
        if self.role=='guard':
            from .cycle_blocked_goal import BlockedGoalCancel
            self.blocked_goals=BlockedGoalCancel()
        if hasattr(node,'_proofs'):node._proofs={}
        if hasattr(node,'_statuses'):
            node._statuses=tuple(retained_status[0]) if retained_status is not None else None
        if self.role=='actor':
            from .pulse_transport import Transport
            node.transport=Transport(new);node.stop_requested=False
            node.cancel_request=node.cancel_request_at=None

    def decorate(self,payload,now,*,remember=True):
        if not self.enabled:return payload
        if self.role=='guard' and self.nav_cancel_report is not None:
            payload=dict(payload,navigation_cancel=copy.deepcopy(self.nav_cancel_report))
        if self.role=='guard' and self.blocked_goals.last is not None:
            payload=dict(payload,blocked_goal_cancel=self.blocked_goals.last)
        if remember:self.last_payload=copy.deepcopy(payload)
        core=getattr(self.node,self.core_attr);p=self.participant
        if p.context is not None and p.phase!='COMMITTED':
            bound=getattr(core,'goal_uuid',getattr(core,'goal',None))
            if (bound is not None and bound!=p.context.nav_goal_uuid
                    and self.nav_historical_goal!=(p.context,bound)):
                p.error='LOCAL_CORE_GOAL_MISMATCH'
        if self.role=='guard':p.update('guard',payload,now)
        ack=p.step(now,graph_ok=self.graph_ok,
            raw_count=len(self.node.get_publishers_info_by_topic('/cmd_vel_raw')),
            pump_count=len(self.node.get_publishers_info_by_topic('/pump_cmd')),
            owns_raw=getattr(self.node,'_raw_publisher',None) is not None,
            owns_pump=getattr(self.node,'pub',None) is not None,
            pending_cancel=getattr(self.node,'cycle_pending_cancel',False),reset=self._reset)
        payload=dict(payload,cycle_protocol_enabled=True,no_goal_supported=True,cycle_release_ack=ack,
            r5_cycle_context=p.context.as_dict() if p.context else None)
        return payload
