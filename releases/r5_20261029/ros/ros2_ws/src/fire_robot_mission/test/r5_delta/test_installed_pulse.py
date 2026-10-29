"""Changed package/identity/profile boundary; actual adapter + cores, fake I/O."""
import copy
import json
from types import SimpleNamespace as N

import pytest
from action_msgs.msg import GoalStatusArray, GoalStatus
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String, Bool
from rclpy.qos import ReliabilityPolicy as RP

from fixture_base import ready_core
from fire_robot_mission import pulse_controller as pc
from fire_robot_mission.operational_contract import OPERATOR_NODE, PULSE_NODE


def ready():
    data = {k: copy.deepcopy(v[0]) for k, v in ready_core().inputs.items()}
    uid = 'a' * 32
    data['gateway']['cycle_goal_uuid'] = uid
    data['mission']['perception']['source_age_ms'] = 20
    data['ir'].update(ir_band='SPRAY_RANGE', stable_samples=5, ir_sample_seq=20,
                      ir_receive_age_ms=20,
                      filtered_counts={'left': 1400, 'center': 5500, 'right': 950})
    data['ir_live'] = dict(schema_version=1, enabled=True, integrated_handoff=True,
        continuous_approach=True, state='COMPLETE', stop_reason='SPRAY_RANGE_REACHED',
        goal_uuid=uid, publisher_owned=False, blockers=[], recovery_required=False,
        command=dict(linear_x_m_s=0., angular_z_rad_s=0.))
    data['actuator'] = dict(schema_version=1, mode='COMMAND_TELEMETRY_ONLY',
        esp_ms=1000, receive_age_ms=20, pump_pwm_ready=True,
        pump_pwm_commanded_on=False, servo_pwm_commanded_on=False,
        servo_phase='IDLE', servo_commanded_angle_deg=None, home_command_completed_count=0)
    data['vision'] = dict(schema_version=1, observe_only=True, state='ok',
        source={'age_ms': 20}, fire=dict(detected_now=True, confirmed=True))
    data['preview'] = dict(schema_version=1, observe_only=True, state='CENTERED', source={'age_ms': 20})
    return data


def endpoint(name, typ='std_msgs/msg/String', rel=RP.BEST_EFFORT, seed=1):
    return N(node_name=name, node_namespace='/', topic_type=typ,
             qos_profile=N(reliability=rel), endpoint_gid=[seed] + [0] * 23)


def adapter(monkeypatch, enabled=True, cycle_protocol=False):
    """Run the real constructor/tick/counts, including both pump subscribers."""
    params = {}; callbacks = {}; sent = {}; pubs = {}; created = []
    clock = N(now=1.)
    cls = pc.PulseController
    monkeypatch.setattr(pc.Node, '__init__', lambda self, name: None)
    monkeypatch.setattr(cls, 'declare_parameter', lambda self, n, v: params.setdefault(n, v))
    overrides = {'enable_pulse': enabled, 'allow_provisional_ir': enabled, 'enable_cycle_protocol':cycle_protocol}
    monkeypatch.setattr(cls, 'get_parameter', lambda self, n: N(value=overrides.get(n, params[n])))
    monkeypatch.setattr(cls, 'resolve_topic_name', lambda self, topic: topic)
    def publisher(self, typ, topic, qos):
        created.append(topic); sent[topic] = []
        pubs[topic] = [endpoint(PULSE_NODE, 'std_msgs/msg/' + typ.__name__, qos.reliability, 21)]
        return N(publish=lambda msg: sent[topic].append(msg.data))
    monkeypatch.setattr(cls, 'create_publisher', publisher)
    monkeypatch.setattr(cls, 'create_subscription', lambda self, typ, topic, fn, qos: callbacks.setdefault(topic, fn))
    monkeypatch.setattr(cls, 'create_client', lambda *a: N())
    monkeypatch.setattr(cls, 'create_timer', lambda *a: None)
    monkeypatch.setattr(cls, 'get_logger', lambda self: N(info=lambda *a: None, warning=lambda *a: None, error=lambda *a: None))
    monkeypatch.setattr(cls, 'get_clock', lambda self: N(now=lambda: N(nanoseconds=int(clock.now*1e9))))
    node = cls()
    for topic, name in pc.INPUTS.values(): pubs[topic] = [endpoint(name)]
    for topic, name, typ, rel in (
        ('/odom', 'serial_bridge_node', 'nav_msgs/msg/Odometry', RP.BEST_EFFORT),
        ('/cmd_vel', 'safety_watchdog', 'geometry_msgs/msg/Twist', RP.BEST_EFFORT),
        ('/navigate_to_pose/_action/status', 'bt_navigator', 'action_msgs/msg/GoalStatusArray', RP.RELIABLE),
        ('/fire_mission/pump_demo_arm', OPERATOR_NODE, 'std_msgs/msg/Bool', RP.RELIABLE),
        (pc.TOPIC, OPERATOR_NODE, 'std_msgs/msg/String', RP.RELIABLE)):
        pubs[topic] = [endpoint(name, typ, rel)]
    subs = {'/pump_cmd': [endpoint(name, 'std_msgs/msg/Bool', RP.RELIABLE, i+30)
                          for i, name in enumerate(('serial_bridge_node', OPERATOR_NODE))]}
    node.get_publishers_info_by_topic = lambda topic: pubs.get(topic, [])
    node.get_subscriptions_info_by_topic = lambda topic: subs.get(topic, [])
    node.poll_safety = lambda now: None
    monkeypatch.setattr(pc.time, 'monotonic', lambda: clock.now)
    def feed(data, at, arm=False):
        clock.now = at; node.safety_ok_at = at
        for key, (topic, _) in pc.INPUTS.items():
            value = copy.deepcopy(data[key])
            if key == 'actuator': value['esp_ms'] = int(at*1000)
            callbacks[topic](String(data=json.dumps(value)))
        callbacks[pc.TOPIC](String(data='{"schema_version":1,"active":false}'))
        callbacks['/cmd_vel'](Twist())
        odom = Odometry(); odom.header.frame_id = 'odom'; odom.child_frame_id = 'base_link'
        odom.header.stamp.sec = int(at); odom.header.stamp.nanosec = int((at-int(at))*1e9)
        odom.pose.pose.orientation.w = 1.
        callbacks['/odom'](odom)
        status = GoalStatus(); status.goal_info.goal_id.uuid = list(bytes.fromhex('a'*32))
        status.status = 5; status.goal_info.stamp.sec = 100
        callbacks['/navigate_to_pose/_action/status'](GoalStatusArray(status_list=[status]))
        callbacks['/fire_mission/pump_demo_arm'](Bool(data=arm))
    return N(node=node, feed=feed, pubs=pubs, subs=subs, sent=sent, created=created, clock=clock)


def test_installed_default_disabled_despite_ready_IR(monkeypatch):
    a = adapter(monkeypatch, enabled=False)
    a.feed(ready(), 1.); a.node.tick()
    assert '/pump_cmd' not in a.created
    assert not a.node.session.used


def test_real_graph_lazy_registration_then_explicit_P_and_OFF_home(monkeypatch):
    a = adapter(monkeypatch); data = ready()
    data['ir_live']['state'] = 'STOPPING'
    a.feed(data, 1.); a.node.tick()
    assert '/pump_cmd' not in a.created
    data['ir_live']['state'] = 'COMPLETE'
    a.feed(data, 1.1); a.node.tick()
    assert a.created.count('/pump_cmd') == 1
    assert a.node.counts()['pump_subscribers'] == 1
    assert a.sent['/pump_cmd'] == [False]
    a.feed(data, 1.2); a.node.tick()
    assert not any(a.sent['/pump_cmd'])
    for i in range(100):
        at = 1.3 + i*.1
        if at >= 1.9:
            data['actuator'].update(pump_pwm_commanded_on=True, servo_pwm_commanded_on=True,
                servo_phase='SWEEPING', servo_commanded_angle_deg=60.)
        if at >= 6.9:
            data['actuator'].update(pump_pwm_commanded_on=False, servo_phase='RETURNING_HOME')
        if at >= 8.1:
            data['actuator'].update(servo_phase='IDLE', servo_pwm_commanded_on=False,
                servo_commanded_angle_deg=None, home_command_completed_count=1)
        a.feed(data, at, arm=True); a.node.tick()
        if a.node.session.phase in ('DONE', 'ABORTED'): break
    report = json.loads(a.sent[pc.PULSE_STATE_TOPIC][-1])
    assert report['state'] == 'DONE', report
    assert report['pump_on_feedback_seen'] and report['servo_sweep_feedback_seen']
    assert any(a.sent['/pump_cmd']) and a.sent['/pump_cmd'][-1] is False
    assert not report['physical_pump_or_nozzle_verified']
    a.feed(data, at+.1, arm=False); a.node.tick()
    a.feed(data, at+.2, arm=True); a.node.tick()
    assert a.sent['/pump_cmd'][-1] is False  # One P never rearms implicitly.


@pytest.mark.parametrize('fault', ['legacy_operator', 'unknown', 'namespace', 'type',
    'qos', 'zero_gid', 'duplicate_pub', 'missing_reader', 'duplicate_reader', 'foreign_reader'])
def test_installed_identity_contract_blocks_registration(monkeypatch, fault):
    a = adapter(monkeypatch); info = a.pubs['/fire_mission/pump_demo_arm'][0]
    if fault == 'legacy_operator': info.node_name = 'qa_baseline_combined_laptop'
    if fault == 'unknown': info.node_name = '_NODE_NAME_UNKNOWN_'
    if fault == 'namespace': info.node_namespace = '/other'
    if fault == 'type': info.topic_type = 'std_msgs/msg/String'
    if fault == 'qos': info.qos_profile.reliability = RP.BEST_EFFORT
    if fault == 'zero_gid': info.endpoint_gid = [0]*24
    if fault == 'duplicate_pub': a.pubs['/fire_mission/pump_demo_arm'].append(copy.deepcopy(info))
    if fault == 'missing_reader': a.subs['/pump_cmd'].pop()
    if fault == 'duplicate_reader': a.subs['/pump_cmd'].append(copy.deepcopy(a.subs['/pump_cmd'][1]))
    if fault == 'foreign_reader': a.subs['/pump_cmd'][1].node_name = 'other'
    a.feed(ready(), 1.); a.node.tick()
    assert '/pump_cmd' not in a.created, fault


@pytest.mark.parametrize('angle,accepted', [(45., False), (59.9, False), (60., True),
                                         (115., True), (115.1, False)])
def test_installed_servo60_feedback_controls_real_pulse(monkeypatch, angle, accepted):
    a = adapter(monkeypatch); data = ready()
    a.feed(data, 1.); a.node.tick()
    for at in (1.1, 1.3, 1.6):
        a.feed(data, at, arm=True); a.node.tick()
    assert a.sent['/pump_cmd'][-1] is True
    data['actuator'].update(pump_pwm_commanded_on=True, servo_pwm_commanded_on=True,
                           servo_phase='SWEEPING', servo_commanded_angle_deg=angle)
    a.feed(data, 1.7, arm=True); a.node.tick()
    assert a.sent['/pump_cmd'][-1] is accepted
    if not accepted: assert a.node.session.reason == 'PUMP_SERVO_FEEDBACK_MISMATCH'


def test_early_P_not_retained_across_missing_IR(monkeypatch):
    a = adapter(monkeypatch); data = ready(); data['ir_live']['state'] = 'STOPPING'
    a.feed(data, 1., arm=False); a.node.tick()
    a.feed(data, 1.1, arm=True); a.node.tick()
    data['ir_live']['state'] = 'COMPLETE'
    a.feed(data, 1.2, arm=True); a.node.tick()
    assert '/pump_cmd' not in a.created and not a.node.session.used
    a.feed(data, 1.3, arm=False); a.node.tick()
    assert '/pump_cmd' in a.created and not any(a.sent['/pump_cmd'])


@pytest.mark.parametrize('source', ['vision', 'preview'])
def test_source_plus_receive_age_blocks_installed_transport(monkeypatch, source):
    a = adapter(monkeypatch); data = ready(); data[source]['source']['age_ms'] = 280
    a.feed(data, 1.); a.clock.now = 1.03; a.node.tick()
    assert '/pump_cmd' not in a.created
    assert a.node.session.get(source, 1.03) is None


@pytest.mark.parametrize('source', ['vision', 'preview'])
def test_lost_image_proof_during_pulse_immediately_requests_OFF(monkeypatch, source):
    a = adapter(monkeypatch); data = ready()
    a.feed(data, 1.); a.node.tick()
    for at in (1.1, 1.3, 1.6):
        a.feed(data, at, arm=True); a.node.tick()
    assert a.sent['/pump_cmd'][-1] is True
    data[source]['source']['age_ms'] = 301
    a.feed(data, 1.7, arm=True); a.node.tick()
    assert a.sent['/pump_cmd'][-1] is False
    assert a.node.session.phase == 'VERIFY_OFF'
