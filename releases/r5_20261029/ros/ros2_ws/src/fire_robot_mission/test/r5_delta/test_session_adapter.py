"""Actual operator constructor/callback/graph/tick with no ROS or GUI runtime."""
import copy
import json
import signal
from types import SimpleNamespace as N
import pytest
from std_msgs.msg import String
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.qos import ReliabilityPolicy as RP

from fire_robot_mission import operational_session as adapter
from fire_robot_mission.operational_contract import OPERATOR_NODE
from fire_robot_mission.operational_view import status_lines, key_name
from test_installed_pulse import endpoint
from test_session_core import data


def node(monkeypatch):
    cls=adapter.OperationalSessionNode;params={};created=[];subscriptions={};sent=[];clock=N(now=1.)
    monkeypatch.setattr(adapter.Node,'__init__',lambda self,name:None)
    monkeypatch.setattr(cls,'declare_parameter',lambda self,k,v:params.setdefault(k,v))
    monkeypatch.setattr(cls,'get_parameter',lambda self,k:N(value=params[k]))
    monkeypatch.setattr(cls,'resolve_topic_name',lambda self,t:t)
    def pub(self,typ,topic,qos):
        created.append((typ,topic))
        return N(publish=lambda msg:sent.append(json.loads(msg.data)))
    monkeypatch.setattr(cls,'create_publisher',pub)
    monkeypatch.setattr(cls,'create_subscription',lambda self,typ,topic,fn,qos:subscriptions.setdefault(topic,fn))
    monkeypatch.setattr(cls,'get_logger',lambda self:N(info=lambda *a:None,warning=lambda *a:None,error=lambda *a:None))
    monkeypatch.setattr(cls,'get_clock',lambda self:N(now=lambda:N(nanoseconds=int(clock.now*1e9))))
    n=cls()
    pubs={topic:[endpoint(name,typ,rel,seed=i+1)] for i,(topic,name,typ,rel) in enumerate(adapter.EXPECTED)}
    pubs[adapter.STATE_TOPIC]=[endpoint(OPERATOR_NODE,seed=50)]
    n.get_publishers_info_by_topic=lambda topic:pubs.get(topic,[])
    monkeypatch.setattr(adapter.time,'monotonic',lambda:clock.now)
    def feed(d,at,*,send_status=True):
        clock.now=at
        for key,(topic,_) in adapter.JSON_INPUTS.items():subscriptions[topic](String(data=json.dumps(d[key])))
        subscriptions['/cmd_vel'](Twist())
        odom=Odometry();odom.header.frame_id='odom';odom.child_frame_id='base_link'
        odom.header.stamp.sec=int(at);odom.header.stamp.nanosec=round((at-int(at))*1e9)
        odom.pose.pose.orientation.w=1.;subscriptions['/odom'](odom)
        status=[]
        for uid,value in d['status']:
            item=GoalStatus();item.status=value;item.goal_info.goal_id.uuid=list(bytes.fromhex(uid));status.append(item)
        if send_status:
            subscriptions['/navigate_to_pose/_action/status'](GoalStatusArray(status_list=status))
    return N(node=n,pubs=pubs,sent=sent,created=created,feed=feed,clock=clock,subscriptions=subscriptions)


def test_actual_operator_only_state_publication_and_real_source_age(monkeypatch):
    a=node(monkeypatch);d=data()
    for at in (1.+n*.1 for n in range(31)):a.feed(d,at);a.node.tick()
    r=a.node.tick('A')
    assert r['armed'] and a.created==[(String,adapter.STATE_TOPIC)]
    assert '/pump_cmd' in a.subscriptions  # required independent reader only
    assert not r['physical_commands_published'] and not r['live_intents_published']
    assert r['integration_blockers']
    d['vision']['source']['age_ms']=301
    a.feed(d,4.1);r=a.node.tick('F')
    assert not r['fire_confirmed'] and r['cycle'] is None


@pytest.mark.parametrize('fault',['duplicate','foreign','gid','namespace','qos','missing'])
def test_real_graph_does_not_declare_ready_from_count_only(monkeypatch,fault):
    a=node(monkeypatch);d=data();topic=adapter.JSON_INPUTS['vision'][0]
    if fault=='duplicate':a.pubs[topic].append(copy.deepcopy(a.pubs[topic][0]))
    if fault=='foreign':a.pubs[topic][0].node_name='other'
    if fault=='namespace':a.pubs[topic][0].node_namespace='/other'
    if fault=='qos':a.pubs[topic][0].qos_profile.reliability=RP.RELIABLE
    if fault=='missing':a.pubs[topic]=[]
    a.feed(d,1.);a.node.tick()
    if fault=='gid':a.pubs[topic][0].endpoint_gid=[100]+[0]*23
    a.feed(d,4.);r=a.node.tick('A')
    assert not r['armed']
    assert r['fault'] or r['reason']=='WAIT_VALID_GRAPH_AND_DATA'
    assert a.created==[(String,adapter.STATE_TOPIC)]


def test_unknown_raw_outside_initial_I_is_visible_fault_not_motion(monkeypatch):
    a=node(monkeypatch);d=data()
    a.pubs['/cmd_vel_raw']=[endpoint('_NODE_NAME_UNKNOWN_','geometry_msgs/msg/Twist',seed=90)]
    a.pubs['/cmd_vel_raw'][0].node_namespace='_NODE_NAMESPACE_UNKNOWN_'
    a.feed(d,1.);r=a.node.tick()
    assert r['fault']=='RAW_UNKNOWN_OUTSIDE_ZERO_ADMISSION'
    assert not any(r['intents'].values())


def test_core_WAIT_does_not_end_main_loop_but_Q_does(monkeypatch):
    events=[];context={'ok':False};handlers={signal.SIGINT:None,signal.SIGTERM:None};previous=dict(handlers)
    monkeypatch.setattr(adapter.signal,'getsignal',lambda sig:handlers[sig])
    monkeypatch.setattr(adapter.signal,'signal',lambda sig,fn:handlers.__setitem__(sig,fn))
    monkeypatch.setattr(adapter.rclpy,'init',lambda **kw:context.update(ok=True))
    monkeypatch.setattr(adapter.rclpy,'ok',lambda:context['ok'])
    monkeypatch.setattr(adapter.rclpy,'spin_once',lambda *a,**k:events.append('spin'))
    monkeypatch.setattr(adapter.rclpy,'shutdown',lambda: (events.append('shutdown'),context.update(ok=False)))
    keys=iter([None,None,'Q'])
    n=N(view=N(tick=lambda report:next(keys)),last_report={},
        tick=lambda key:dict(phase='WAIT_TARGET',closed=key=='Q'),
        close_session=lambda:events.append('close'),destroy_node=lambda:events.append('destroy'))
    monkeypatch.setattr(adapter,'OperationalSessionNode',lambda:n)
    assert adapter.main()==0 and events.count('spin')==3
    assert events[-3:]==['close','destroy','shutdown'] and handlers==previous


def test_UI_exception_cleans_node_before_context_shutdown(monkeypatch):
    events=[];context={'ok':False};handlers={signal.SIGINT:None,signal.SIGTERM:None}
    monkeypatch.setattr(adapter.signal,'getsignal',lambda sig:handlers[sig])
    monkeypatch.setattr(adapter.signal,'signal',lambda sig,fn:handlers.__setitem__(sig,fn))
    monkeypatch.setattr(adapter.rclpy,'init',lambda **kw:context.update(ok=True))
    monkeypatch.setattr(adapter.rclpy,'ok',lambda:context['ok'])
    monkeypatch.setattr(adapter.rclpy,'spin_once',lambda *a,**kw:None)
    monkeypatch.setattr(adapter.rclpy,'shutdown',lambda:(events.append('shutdown'),context.update(ok=False)))
    def fail(*a):raise RuntimeError('view fault')
    n=N(view=N(tick=fail),last_report={},close_session=lambda:events.append('close'),
        destroy_node=lambda:events.append('destroy'),get_logger=lambda:N(error=lambda *a:events.append('error')))
    monkeypatch.setattr(adapter,'OperationalSessionNode',lambda:n)
    assert adapter.main()==2
    assert events==['error','close','destroy','shutdown']


@pytest.mark.parametrize('code,key',[(ord('i'),'I'),(ord('p'),'P'),(ord('x'),'X'),(27,'ESC'),(255,None)])
def test_camera_keys_have_no_side_effect_on_permissions(code,key):
    assert key_name(code)==key


def test_UI_describes_IR_ADC_and_key_rejection_with_wait_reason():
    r=dict(phase='WAIT_TARGET',reason='IR_RETAINED_WAIT_FRESH_YOLO',ir_band='APPROACH',
        ir_counts={'left':25000,'center':24000,'right':900},
        operator_feedback=dict(key='I',accepted=False,reason='KEY_NOT_ALLOWED'))
    text='\n'.join(status_lines(r))
    assert '900' in text and 'WAIT_TARGET' in text and 'accepted=False' in text
    assert 'LIVE OUTPUT DISCONNECTED' in text


@pytest.mark.parametrize('source',['arm','pump','operator'])
def test_foreign_arm_pump_or_second_operator_is_visible_fault(monkeypatch,source):
    a=node(monkeypatch);d=data()
    topic={'arm':'/fire_mission/ir_approach_arm','pump':'/pump_cmd','operator':adapter.STATE_TOPIC}[source]
    a.pubs[topic]=[endpoint('foreign','std_msgs/msg/String' if source=='operator' else 'std_msgs/msg/Bool',RP.RELIABLE)]
    a.feed(d,1.);r=a.node.tick()
    assert r['fault'] and not any(r['intents'].values())


def initial_IR(a):
    d=data()
    for at in (1.+n*.1 for n in range(31)):a.feed(d,at);a.node.tick()
    a.node.tick('A');a.clock.now=4.1;a.feed(d,4.1);a.node.tick('F')
    d['status']=[('a'*32,5)]
    d['ir']['zone']=d['ir']['ir_band']='APPROACH';d['ir']['filtered_counts']['right']=5000
    d['ir_live'].update(state='WAITING_FOR_ADMISSION',stop_reason='',publisher_owned=False)
    a.feed(d,4.2);a.node.tick('I')
    return d


def unknown(a):
    i=endpoint('_NODE_NAME_UNKNOWN_','geometry_msgs/msg/Twist',seed=90)
    i.node_namespace='_NODE_NAMESPACE_UNKNOWN_';a.pubs['/cmd_vel_raw']=[i]
    return i


def test_initial_unknown_name_resolves_same_GID_only_while_zero(monkeypatch):
    a=node(monkeypatch);d=initial_IR(a);i=unknown(a)
    a.feed(d,4.3);r=a.node.tick()
    assert not r['fault'] and a.node.graph.discovery.since==4.3
    i.node_name='ir_approach_live';i.node_namespace='/'
    a.feed(d,4.4);r=a.node.tick()
    assert not r['fault'] and a.node.graph.discovery.resolved
    assert not r['motion_authorized']


@pytest.mark.parametrize('fault',['gid','timeout','moving'])
def test_unknown_grace_cannot_hide_changed_gid_deadline_or_motion(monkeypatch,fault):
    a=node(monkeypatch);d=initial_IR(a);i=unknown(a)
    a.feed(d,4.3);a.node.tick()
    at=4.4
    if fault=='gid':
        i.node_name='ir_approach_live';i.node_namespace='/';i.endpoint_gid=[91]+[0]*23
    if fault=='timeout':at=5.1
    if fault=='moving':d['ir_live']['state']='MOVING'
    a.feed(d,at);r=a.node.tick()
    assert r['fault'] and not any(r['intents'].values())


def test_event_driven_status_is_retained_with_actual_receipt_time_and_pinned_source(monkeypatch):
    a=node(monkeypatch);d=data()
    for at in (1.+n*.1 for n in range(31)):a.feed(d,at);a.node.tick()
    a.node.tick('A')
    a.feed(d,200.,send_status=False)
    r=a.node.tick('F')
    assert r['cycle']['nav_goal_uuid']=='a'*32 and not r['fault']
    assert r['nav_status_receive_age_s']==196.
    assert r['phase']=='WAIT_STOP'  # still no cancel or no-active proof invented
    topic='/navigate_to_pose/_action/status'
    a.pubs[topic][0].endpoint_gid=[199]+[0]*23
    a.feed(d,200.1,send_status=False);r=a.node.tick()
    assert r['fault']=='INPUT_GID_CHANGED:'+topic and not any(r['intents'].values())


def test_retained_empty_status_starts_context_but_does_not_make_no_goal_motion_proof(monkeypatch):
    a=node(monkeypatch);d=data();d['status']=[]
    for at in (1.+n*.1 for n in range(31)):a.feed(d,at);a.node.tick()
    a.node.tick('A');a.feed(d,200.,send_status=False);r=a.node.tick('F')
    assert r['phase']=='WAIT_NO_GOAL_HANDSHAKE' and r['cycle']['nav_goal_uuid'] is None
    assert not r['intents']['alignment'] and not r['motion_authorized']


def test_source_files_have_no_live_command_publisher_or_process_launcher():
    import ast
    from pathlib import Path
    source=Path(adapter.__file__).read_text();tree=ast.parse(source)
    pubs=[n for n in ast.walk(tree) if isinstance(n,ast.Call) and isinstance(n.func,ast.Attribute)
          and n.func.attr=='create_publisher']
    assert len(pubs)==1 and isinstance(pubs[0].args[0],ast.Name) and pubs[0].args[0].id=='String'
    assert pubs[0].args[1].id=='STATE_TOPIC'
    assert not any(isinstance(n,ast.Name) and n.id in ('ActionClient','ExecuteProcess','subprocess') for n in ast.walk(tree))


def test_view_keeps_polling_Q_on_a_stale_image_without_opening_real_GUI(monkeypatch):
    from fire_robot_mission.operational_view import SessionView
    import numpy as np
    view=object.__new__(SessionView);view.np=np;view.name='fake';view.visible=False
    view.image=None;view.image_at=None;events=[]
    view.cv2=N(WND_PROP_AUTOSIZE=1,FONT_HERSHEY_SIMPLEX=0,
        getWindowProperty=lambda *a:1.,putText=lambda panel,text,*args:events.append(text),
        imshow=lambda *a:events.append('imshow'),waitKey=lambda *a:ord('q'))
    assert view.tick(dict(phase='WAIT_TARGET',reason='WAIT_FRESH_YOLO'))=='Q'
    assert any('STALE / NO IMAGE' in x for x in events)


def test_initial_unknown_input_waits_for_exact_name_then_same_GID_remains_pinned(monkeypatch):
    a=node(monkeypatch);d=data();topic=adapter.JSON_INPUTS['vision'][0];i=a.pubs[topic][0]
    i.node_name='_NODE_NAME_UNKNOWN_';i.node_namespace='_NODE_NAMESPACE_UNKNOWN_'
    a.feed(d,1.);r=a.node.tick()
    assert not r['fault'] and r['phase']=='WAIT_ARM' and not r['armed']
    i.node_name='yolo_observer';i.node_namespace='/'
    for at in (2.+n*.1 for n in range(31)):a.feed(d,at);a.node.tick()
    r=a.node.tick('A');assert r['armed']
    i.node_name='_NODE_NAME_UNKNOWN_';i.node_namespace='_NODE_NAMESPACE_UNKNOWN_'
    a.feed(d,5.1);r=a.node.tick()
    assert not r['fault'] and r['armed']
    i.endpoint_gid=[198]+[0]*23
    a.feed(d,5.2);r=a.node.tick()
    assert r['fault']=='INPUT_GID_CHANGED:'+topic and not r['armed']


def test_close_publishes_final_STOP_intents_while_context_still_valid(monkeypatch):
    a=node(monkeypatch);d=data()
    for at in (1.+n*.1 for n in range(31)):a.feed(d,at);a.node.tick()
    a.node.tick('A');assert a.node.core.armed
    a.node.close_session()
    assert a.sent[-1]['closed'] and a.sent[-1]['phase']=='STOPPED'
    assert not any(a.sent[-1]['intents'].values())


@pytest.mark.parametrize('band,label',[('TOO_CLOSE','QUA GAN'),('SPRAY_RANGE','TRONG VUNG PHUN'),('APPROACH','CAN TIEN')])
def test_IR_range_is_readable_in_camera_status(band,label):
    assert label in '\n'.join(status_lines(dict(ir_band=band)))
