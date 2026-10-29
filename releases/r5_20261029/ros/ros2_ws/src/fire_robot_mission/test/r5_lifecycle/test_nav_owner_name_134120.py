"""134120/134848: Nav2 publisher name discovery at zero, no hardware/DDS."""
import copy
import pytest
from rclpy.node import Node
from test_ir_freshness_hold_115816 import robot


def acquiring(monkeypatch,tmp_path,delay=.25,*,operator_only=False):
    h=robot(monkeypatch,tmp_path,goal=False)
    original=Node.get_publishers_info_by_topic;created=[None];publisher=[None]
    def infos(node,topic):
        rows=original(node,topic)
        if topic!='/cmd_vel_raw' or not rows:return rows
        if rows[0].node_name!='live_nav_stop_gateway':return rows
        if publisher[0] is not rows[0]:publisher[0]=rows[0];created[0]=h.bus.now
        if h.bus.now-created[0]>=delay:return rows
        if operator_only and node is not h.op:return rows
        rows=[copy.copy(row)for row in rows]
        for row in rows:row.node_name='_NODE_NAME_UNKNOWN_';row.node_namespace='_NODE_NAMESPACE_UNKNOWN_'
        return rows
    monkeypatch.setattr(Node,'get_publishers_info_by_topic',infos)
    for p in h.nav:
        send=p.publish
        def publish(msg,send=send):
            msg=copy.deepcopy(msg)
            if any(s in (1,2,3)for u,s in h.status):msg.linear.x=.06
            send(msg)
        monkeypatch.setattr(p,'publish',publish)
    h.status=[('a'*32,2)]
    return h,created


@pytest.mark.parametrize('operator_only',[False,True])
def test_initial_nav_unknown_name_stays_zero_then_resumes_and_X_newgoal(monkeypatch,tmp_path,operator_only):
    h,created=acquiring(monkeypatch,tmp_path,operator_only=operator_only)
    h.until(lambda r:created[0] is not None)
    h.until(lambda r:h.x>.02)
    assert h.op.core.fault is None and h.bus.max_raw==1
    h.step('X');h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    x=h.x;h.status.append(('56'*16,2));h.until(lambda r:h.x>x+.02)
    assert h.op.core.fault is None


@pytest.mark.parametrize('operator_only',[False,True])
def test_unknown_period_never_forwards_nonzero_or_creates_second_owner(monkeypatch,tmp_path,operator_only):
    h,created=acquiring(monkeypatch,tmp_path,.35,operator_only=operator_only)
    h.until(lambda r:created[0] is not None)
    while h.bus.now-created[0]<.35:
        assert h.x==0.
        assert not any(m.linear.x or m.angular.z for at,who,t,m in h.bus.sent if t=='/cmd_vel_raw')
        h.step()
    h.until(lambda r:h.x>.01)
    assert h.bus.max_raw==1


@pytest.mark.parametrize('key',['X','Q'])
def test_X_or_Q_during_pending_acquisition_drains_without_graph_fault(monkeypatch,tmp_path,key):
    h,created=acquiring(monkeypatch,tmp_path,.4)
    h.until(lambda r:r.get('graph_hold') is True)
    h.step(key)
    if key=='X':
        h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
        assert len(h.bus.cancelled)==1
    else:
        assert h.until(lambda r:r.get('shutdown',{}).get('done'))['shutdown']['confirmed']
    assert h.x==0. and h.op.core.fault is None


@pytest.mark.parametrize('bad',['deadline','gid_switch','wrong_name','wrong_type','wrong_qos','nonzero','epoch'])
def test_unsafe_acquisition_never_clears_fault_with_X(monkeypatch,tmp_path,bad):
    from std_msgs.msg import String
    from rclpy.qos import ReliabilityPolicy as RP
    import json
    h,created=acquiring(monkeypatch,tmp_path,2.)
    h.until(lambda r:r.get('graph_hold') is True)
    if bad=='epoch':
        h.silence_serial=True;value=copy.deepcopy(h.op.core.inputs['identity'][0]);value['connection_epoch']+=1
        h.external['/fire_sensors/serial_session_identity'].publish(String(data=json.dumps(value)));h.bus.flush()
    elif bad!='deadline':
        old=Node.get_publishers_info_by_topic
        def infos(node,topic):
            rows=old(node,topic)
            if topic=='/cmd_vel_raw' and rows:
                rows=[copy.copy(x)for x in rows]
                if bad=='gid_switch':rows[0].endpoint_gid=list((999).to_bytes(24,'big'))
                if bad=='wrong_name':rows[0].node_name='rogue';rows[0].node_namespace='/'
                if bad=='wrong_type':rows[0].topic_type='std_msgs/msg/Bool'
                if bad=='wrong_qos':
                    from rclpy.qos import QoSProfile
                    rows[0].qos_profile=QoSProfile(depth=1,reliability=RP.RELIABLE)
            return rows
        monkeypatch.setattr(Node,'get_publishers_info_by_topic',infos)
        if bad=='nonzero':
            # A claimed zero-only wait with actual motion is never admitted.
            h.op.core.inputs['odom']=((.1,0,0,0,0,0),h.bus.now)
            h.op.core.inputs['cmd']=((.1,0,0,0,0,0),h.bus.now)
            h.op.tick()
    for _ in range(30):h.step()
    assert h.op.core.fault is not None
    before=h.op.core.fault;r=h.step('X')
    assert r['fault']==before and not any(r['intents'].values()) and not r['operator_feedback']['accepted']
    assert h.x==0. and not h.pulses
    assert any('OPERATOR_GRAPH 'in str(a)for at,who,a in h.bus.logs if who=='fire_robot_operator')


def test_missing_required_source_is_logged_and_does_not_authorize_motion(monkeypatch,tmp_path):
    h,created=acquiring(monkeypatch,tmp_path,.4)
    h.until(lambda r:r.get('graph_hold') is True)
    p=h.external['/odom'];h.bus.pubs.remove(p)
    # Call node directly so the fixture does not try publishing a removed input.
    r=h.op.tick('X')
    assert r['fault'] is not None and not any(r['intents'].values())
    graphs=[str(a)for at,who,a in h.bus.logs if who=='fire_robot_operator' and 'OPERATOR_GRAPH 'in str(a)]
    assert '/odom'in graphs[-1]


@pytest.mark.parametrize('target',['ir','epoch'])
def test_name_handshake_does_not_break_automatic_fire_or_epoch_stop(monkeypatch,tmp_path,target):
    h,created=acquiring(monkeypatch,tmp_path)
    h.until(lambda r:h.x>.02)
    if target=='epoch':
        from std_msgs.msg import String
        import json
        h.silence_serial=True;identity=copy.deepcopy(h.op.core.inputs['identity'][0]);identity['connection_epoch']+=1
        h.external['/fire_sensors/serial_session_identity'].publish(String(data=json.dumps(identity)));h.bus.flush()
        r=h.step();assert r['fault']=='BOOT_OR_SERIAL_EPOCH_CHANGED' and not any(r['intents'].values())
    else:
        h.fire=True;h.until(lambda r:abs(h.yaw)>.025);h.centered=True
        x=h.x;h.until(lambda r:h.x>x+.02)
        h.right=950;h.until(lambda r:h.home==1,limit=350)
        assert len(h.pulses)==1 and h.off_at-h.on_at<=5.1+1e-6


@pytest.mark.parametrize('bad',['multiple','zero_gid'])
def test_unknown_owner_still_requires_one_valid_gid(monkeypatch,tmp_path,bad):
    h,created=acquiring(monkeypatch,tmp_path,2.)
    h.until(lambda r:r.get('graph_hold') is True)
    original=Node.get_publishers_info_by_topic
    def infos(node,topic):
        rows=original(node,topic)
        if topic=='/cmd_vel_raw' and rows:
            if bad=='multiple':return rows+rows
            rows=[copy.copy(x)for x in rows];rows[0].endpoint_gid=[0]*24
        return rows
    monkeypatch.setattr(Node,'get_publishers_info_by_topic',infos)
    r=h.step()
    assert r['fault'] is not None and h.x==0.


def test_wrong_operator_gid_confirmation_never_authorizes_motion_or_silent_wait(monkeypatch,tmp_path):
    import json
    from std_msgs.msg import String
    h,created=acquiring(monkeypatch,tmp_path,.1)
    send=h.op.state_pub.publish
    def publish(msg):
        value=json.loads(msg.data)
        if 'raw_owner_proof'in value:value['raw_owner_proof']['gid']='ff'*24
        send(String(data=json.dumps(value)))
    monkeypatch.setattr(h.op.state_pub,'publish',publish)
    h.until(lambda r:created[0] is not None)
    for _ in range(35):r=h.step()
    assert h.x==0. and r['fault']=='NAV_OWNER_CONFIRMATION_TIMEOUT'
    h.step('Q')
    for _ in range(30):
        r=h.step()
        if r.get('shutdown',{}).get('done'):break
    assert r['shutdown']['confirmed']


def test_manual_push_after_goal_is_a_standby_wait_not_a_persistent_fault(monkeypatch,tmp_path):
    h,created=acquiring(monkeypatch,tmp_path)
    h.until(lambda r:h.x>.02);h.status=[('a'*32,4)]
    h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    emit=h.emit;push=[True]
    def pushing(topic,msg):
        if topic=='/odom' and push[0]:
            msg=copy.deepcopy(msg);msg.twist.twist.linear.x=-.06
        emit(topic,msg)
    h.emit=pushing
    for _ in range(20):h.step()
    assert h.op.core.fault is None
    push[0]=False;h.until(lambda r:r['reason']=='NAV2_READY_NEW_GOAL')
    x=h.x;h.status.append(('56'*16,2));h.until(lambda r:h.x>x+.02)
    assert h.op.core.fault is None
