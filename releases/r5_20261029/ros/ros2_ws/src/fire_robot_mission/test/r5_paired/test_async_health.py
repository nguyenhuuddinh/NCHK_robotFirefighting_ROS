from types import SimpleNamespace as N
import pytest
from fire_robot_mission.startup_health import StartupHealthNode

class Future:
    def __init__(self,result=None,done=False):self.value=result;self.ready=done;self.cancelled=False
    def done(self):return self.ready
    def result(self):
        if isinstance(self.value,Exception):raise self.value
        return self.value
    def cancel(self):self.cancelled=True
class Client:
    def __init__(self,future):self.future=future;self.sent=[];self.removed=[]
    def service_is_ready(self):return True
    def call_async(self,request):self.sent.append(request);return self.future
    def remove_pending_request(self,f):self.removed.append(f)


def node(monkeypatch,future):
    now=[10.];monkeypatch.setattr('fire_robot_mission.startup_health.time.monotonic',lambda:now[0])
    n=object.__new__(StartupHealthNode);n.control={};n.pending={};n.next_query={};n.control_clients={'safety':Client(future)}
    monkeypatch.setattr(n,'get_logger',lambda:N(warning=lambda s:None));return n,now


def test_async_health_uses_one_pending_request_not_blocking_sleep_or_ros_cli(monkeypatch):
    f=Future();n,now=node(monkeypatch,f);n.poll_services();n.poll_services()
    assert len(n.control_clients['safety'].sent)==1 and 'safety' in n.pending
    now[0]+=3.;n.poll_services();assert f.cancelled and n.control_clients['safety'].removed==[f] and n.control['safety'][0] is False
    n.poll_services();assert len(n.control_clients['safety'].sent)==2


@pytest.mark.parametrize('result',[None,ValueError('bad'),N(values=[N(type=4,string_value='/wrong'),N(type=4,string_value='/cmd_vel')]),N(values=[N(type=4,string_value='/cmd_vel_raw'),N(type=4,string_value='/cmd_vel')])])
def test_async_safety_response_must_keep_exact_raw_to_cmd_route(monkeypatch,result):
    f=Future(result,True);n,now=node(monkeypatch,f);n.poll_services();n.poll_services()
    assert n.control['safety'][0] is (hasattr(result,'values') and result.values[0].string_value=='/cmd_vel_raw')


def test_service_query_exception_records_false_and_next_retry_still_bounded(monkeypatch):
    n,now=node(monkeypatch,Future());n.control_clients['safety'].call_async=lambda r:(_ for _ in ()).throw(RuntimeError('fake DDS discovery error'))
    n.poll_services();assert not n.pending and n.control['safety'][0] is False and n.next_query['safety']==13.


@pytest.mark.parametrize('role',['pi','laptop'])
def test_actual_health_node_constructor_and_sample_have_no_publishers_or_Node_property_collision(monkeypatch,role):
    import sys
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).parents[1]/'r5_full_adapter'))
    from harness import Bus
    from fire_robot_mission import startup_health as module
    bus=Bus(monkeypatch)
    monkeypatch.setattr(module,'Buffer',lambda:N(lookup_transform=lambda *a:(_ for _ in ()).throw(module.TransformException('no fake TF yet'))))
    monkeypatch.setattr(module,'TransformListener',lambda *a:None)
    monkeypatch.setattr(module,'ActionClient',lambda *a:N(server_is_ready=lambda:False))
    node=module.StartupHealthNode('session',role);report=node.sample()
    assert report['commands_published'] is False and not bus.pubs
    assert len(node.control_clients)==(2 if role=='pi' else 10)
    assert report['checks']['io'] is False


def test_startup_health_does_not_admit_existing_raw_or_pump_owner(monkeypatch):
    import sys
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).parents[1]/'r5_full_adapter'))
    from harness import Bus,BE
    from fire_robot_mission import startup_health as module
    from geometry_msgs.msg import Twist
    bus=Bus(monkeypatch)
    monkeypatch.setattr(module,'Buffer',lambda:N(lookup_transform=lambda *a:(_ for _ in ()).throw(module.TransformException('no fake TF yet'))))
    monkeypatch.setattr(module,'TransformListener',lambda *a:None)
    monkeypatch.setattr(module,'ActionClient',lambda *a:N(server_is_ready=lambda:False))
    node=module.StartupHealthNode('session','laptop')
    monkeypatch.setattr(node,'source',lambda k:True)
    monkeypatch.setattr(node.core,'report',lambda now,**kw:kw)
    assert node.sample()['sources']['io'] is True
    bus.publisher('foreign',Twist,'/cmd_vel_raw',BE)
    assert node.sample()['sources']['io'] is False


@pytest.mark.parametrize('mode',['fresh_odom_lag_and_AMCL_future','stale_odom','cached_map_stops_progressing'])
def test_latest_tf_readiness_accepts_normal_odom_lag_but_keeps_age_and_progress_gates(monkeypatch,mode):
    import sys
    from pathlib import Path
    sys.path.insert(0,str(Path(__file__).parents[1]/'r5_full_adapter'))
    from harness import Bus
    from fire_robot_mission import startup_health as module
    bus=Bus(monkeypatch);bus.now=10.;stamps=dict(odom=9.95,map=11.)
    if mode=='stale_odom':stamps['odom']=9.6
    def lookup(parent,child,at):
        assert at.nanoseconds==0,'Latest TF required; exact now would extrapolate'
        stamp=round(stamps[parent]*1e9)
        return N(header=N(stamp=N(sec=stamp//10**9,nanosec=stamp%10**9)))
    monkeypatch.setattr(module,'Buffer',lambda:N(lookup_transform=lookup))
    monkeypatch.setattr(module,'TransformListener',lambda *a:None)
    monkeypatch.setattr(module,'ActionClient',lambda *a:N(server_is_ready=lambda:False))
    node=module.StartupHealthNode('session','laptop');node.core.initial_pose_at=9.9
    node.control['odom_tf_profile']=(True,10.)
    monkeypatch.setattr(node.core,'report',lambda now,**kw:kw)
    first=node.sample()
    assert first['odom_tf_ok'] is (mode!='stale_odom') and first['tf_ok'] is True
    if mode=='cached_map_stops_progressing':
        bus.now=11.1;r=node.sample();assert not r['tf_ok'] and not r['odom_tf_ok']
