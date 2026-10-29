"""Observer graph checks with endpoint pins and the accepted initial-IR grace."""
from .operational_contract import exact_endpoint, valid_gid, OPERATOR_NODE, PULSE_NODE
from .operational_discovery import Discovery
from .operational_cycle import NavigationSnapshot
from .operational_identity import identity_current
from .operational_session_core import obj, zero


class SessionGraph:
    def __init__(self):
        self.pins = {}
        self.raw_pin = None
        self.fault = None
        self.discovery = Discovery()
        self.discovery_cycle=None
        self.nav_discovery=Discovery()
        self.raw_wait_gid=self.raw_wait_since=None

    def check(self, node, core, now, expected, reliability):
        errors = []; missing = []; deferred = []
        perception={'/yolo/detections','/yolo/alignment_preview','/fire_sensors/ir_raw','/fire_mission/ir_range_preview'}
        committed=core.pulse_committed(now)
        with_self = list(expected)+[('/fire_mission/operator_state',OPERATOR_NODE,
                                    'std_msgs/msg/String',node.state_qos.reliability)]
        for topic, name, typ, rel in with_self:
            infos = node.get_publishers_info_by_topic(topic)
            if not infos:
                # Missing perception may obscure a committed finite pulse. It
                # cannot admit ON or authenticate a new/changed endpoint.
                (deferred if committed and topic in perception and topic in self.pins else missing).append(topic)
                continue
            # Initial UNKNOWN metadata is not an authenticated source. Wait
            # for its exact name; after pinning, the same GID may temporarily
            # lose its name as already supported by the live IR endpoint pins.
            if len(infos)==1 and valid_gid(infos[0]):
                i=infos[0];gid=bytes(i.endpoint_gid).hex()
                unknown=(i.node_name,i.node_namespace)==('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_')
                if unknown and i.topic_type==typ and i.qos_profile.reliability==rel:
                    if topic not in self.pins:
                        missing.append(topic)
                        continue
                    if self.pins[topic]==gid:
                        continue
                    errors.append('INPUT_GID_CHANGED:'+topic)
                    continue
            if not exact_endpoint(infos, name, typ, rel):
                errors.append('INPUT_ENDPOINT_INVALID:'+topic)
                continue
            gid = bytes(infos[0].endpoint_gid).hex()
            if topic in self.pins and self.pins[topic] != gid:
                errors.append('INPUT_GID_CHANGED:'+topic)
            self.pins.setdefault(topic, gid)
        infos = node.get_publishers_info_by_topic('/cmd_vel_raw')
        endpoints = []
        for i in infos:
            endpoints.append(dict(node=i.node_name, namespace=i.node_namespace,
                type=i.topic_type, reliability=str(i.qos_profile.reliability),
                gid=bytes(i.endpoint_gid).hex() if valid_gid(i) else ''))
        # DDS may withdraw the name before withdrawing an already authenticated
        # endpoint. Keep only that exact live GID/type/QoS pin, never a new GID.
        pinned_name_used = False
        if len(endpoints) == 1 and self.raw_pin is not None:
            endpoint = endpoints[0]
            if ((endpoint['node'], endpoint['namespace']) ==
                    ('_NODE_NAME_UNKNOWN_', '_NODE_NAMESPACE_UNKNOWN_')
                    and endpoint['gid'] == self.raw_pin[1]
                    and endpoint['type'] == 'geometry_msgs/msg/Twist'
                    and endpoint['reliability'] == 'ReliabilityPolicy.BEST_EFFORT'):
                endpoint['node'], endpoint['namespace'] = self.raw_pin[0], '/'
                pinned_name_used = True
        budget = core.cycles.budget
        live = obj(core.get('ir_live', now))
        initial = (budget is not None and budget.requested_at is not None
                   and core.phase in ('IR_RUNNING', 'IR_RESUME') and not core.ir_has_run
                   and not core.ir_recovering and 0 <= now-budget.requested_at < 3.
                   and core._aligned(now,initial_zero_only=True) and core._off(now)
                   and zero(core.get('cmd', now)) and zero(core.get('odom', now), .02, .05)
                   and live.get('state') in ('WAITING_FOR_ADMISSION', 'ACQUIRING')
                   and live.get('command') == {'linear_x_m_s':0., 'angular_z_rad_s':0.})
        current_cycle=core.cycles.current.cycle_id if core.cycles.current else None
        if (self.discovery_cycle is not None and self.discovery_cycle!=current_cycle
                and self.discovery_cycle in core.cycles.retired and not endpoints and core._rest(now)):
            self.discovery=Discovery();self.discovery_cycle=None
        if current_cycle is not None and self.discovery_cycle is None:self.discovery_cycle=current_cycle
        # Discovery's opaque key scopes a zero-only owner wait. It is never
        # sent to Nav2 or inserted into any status/cancel/goal_uuid report.
        gateway=obj(core.get('gateway',now));status=None
        try:
            rows,received=core.inputs.get('status',(None,None))
            status=NavigationSnapshot.parse(rows,received)
            if received>now:status=None
        except (ValueError,TypeError):pass
        identity,at=core.inputs.get('identity',(None,None))
        nav_initial=(core.cycles.current is None and core.armed and not core.fault
            and status is not None and status.active_uuid is not None
            and core.navigation_cancel_uuid in (None,status.active_uuid)
            and identity_current(identity,at,now) and core._off(now)
            and zero(core.get('cmd',now)) and zero(core.get('odom',now),.02,.05)
            and gateway.get('schema_version')==1 and gateway.get('enabled') is True
            and (gateway.get('state') in ('ARMING','WAITING_FOR_INPUT','RELEASED_STOPPED')
                 or core.navigation_cancel_uuid is not None and gateway.get('state') in ('STOPPING','RELEASING'))
            and gateway.get('command')=={'linear_x_m_s':0.,'angular_z_rad_s':0.}
            and gateway.get('nav_shadow_fingerprint_ok') is True
            and gateway.get('safety_gate_fingerprint_ok') is True
            and core.get('vision',now) is not None and (not core._fire(now) or core.retry_fire_clear_required))
        if not endpoints and core._rest(now):self.nav_discovery=Discovery()
        unknown=(len(endpoints)==1 and (endpoints[0]['node'],endpoints[0]['namespace'])==
                 ('_NODE_NAME_UNKNOWN_','_NODE_NAMESPACE_UNKNOWN_'))
        nav_pending=self.nav_discovery.since is not None and not self.nav_discovery.resolved
        # A new R5 raw endpoint may be discovered before its node name.
        # This is a STOP-only input hold, never motion admission. Keep the
        # unknown GID stable until exact metadata arrives; do not poison the
        # lifetime of Discovery with a temporary name lookup failure.
        r5_unknown=(unknown and core.armed and core.cycles.current is not None
            and not core.p_used and identity_current(identity,at,now)
            and len(infos)==1 and valid_gid(infos[0])
            and endpoints[0]['type']=='geometry_msgs/msg/Twist'
            and endpoints[0]['reliability']=='ReliabilityPolicy.BEST_EFFORT'
            and self.raw_pin is None)
        if r5_unknown:
            gid=endpoints[0]['gid']
            if self.raw_wait_gid is None:self.raw_wait_gid=gid;self.raw_wait_since=now
            changed=gid!=self.raw_wait_gid
            if now-self.raw_wait_since>=.75:core.interrupt_cycle('RAW_NAME_DISCOVERY_TIMEOUT')
            missing.append('/cmd_vel_raw:waiting_for_name')
            decision=dict(status='INVALID' if changed else 'PENDING',
                reason='RAW_DISCOVERY_IDENTITY_CHANGED' if changed else 'R5_RAW_NAME_DISCOVERY_ZERO_ONLY',
                observed_at=now,since=self.raw_wait_since,limit_s=.75,
                goal_uuid=core.cycles.current.nav_goal_uuid,cycle_id=current_cycle,
                endpoints=endpoints,context=dict(valid=False,zero_only=True))
        elif nav_pending or nav_initial and unknown:
            decision=self.nav_discovery.observe(now,endpoints,dict(valid=nav_initial,
                goal_uuid=status.active_uuid if status else None,cycle_id=None,
                expected_owner='live_nav_stop_gateway'))
        else:
            decision = self.discovery.observe(now, endpoints, dict(valid=initial,
                goal_uuid=core.cycles.current.nav_goal_uuid if core.cycles.current else None,
                cycle_id=core.cycles.current.cycle_id if core.cycles.current else None))
        if not unknown:self.raw_wait_gid=self.raw_wait_since=None
        if decision['status'] == 'INVALID':
            errors.append(decision['reason'])
        if decision['status'] == 'EXACT' and endpoints:
            pin = (endpoints[0]['node'], endpoints[0]['gid'])
            if self.raw_pin is not None and pin != self.raw_pin:
                errors.append('RAW_OWNER_OR_GID_CHANGED_WITHOUT_STOP_RELEASE')
            self.raw_pin = pin
        elif not endpoints and core._rest(now):
            self.raw_pin = None
        # A single optional pump owner is visible but never created by this node.
        pumps = node.get_publishers_info_by_topic('/pump_cmd')
        if pumps and not exact_endpoint(pumps, PULSE_NODE, 'std_msgs/msg/Bool', reliability):
            errors.append('PUMP_ENDPOINT_INVALID')
        for suffix in ('live_nav_arm','nav_cancel_arm','alignment_arm_shadow',
                       'live_alignment_arm','ir_approach_arm','pump_demo_arm'):
            topic='/fire_mission/'+suffix
            arms=node.get_publishers_info_by_topic(topic)
            if arms and not exact_endpoint(arms,OPERATOR_NODE,'std_msgs/msg/Bool',reliability):
                errors.append('ARM_ENDPOINT_INVALID:'+topic)
        if errors and self.fault is None:
            self.fault = errors[0]
        return dict(valid=not missing and self.fault is None, fault=self.fault,
                    missing=missing, deferred_perception=deferred, errors=errors, raw_count=len(infos),
                    raw_owner=endpoints[0]['node'] if len(endpoints)==1 else None,
                    discovery=decision, gid_pins=dict(self.pins), raw_pinned_name_used=pinned_name_used,
                    nav_owner_pending=decision['status']=='PENDING' and decision.get('reason')=='INITIAL_NAV_NAME_DISCOVERY_ZERO_ONLY',
                    nav_status_event_stream=(not missing and self.fault is None
                        and '/navigate_to_pose/_action/status' in self.pins),
                    callback_provenance_verified=False)
