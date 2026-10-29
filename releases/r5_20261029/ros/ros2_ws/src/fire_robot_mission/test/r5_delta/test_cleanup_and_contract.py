"""New installed entry point/config boundaries; never starts ROS or hardware."""
import ast
import importlib
import signal
from pathlib import Path
from types import SimpleNamespace as N

import pytest
import yaml

from fire_robot_mission import ir_approach_live as ir
from fire_robot_mission import pulse_controller as pc
from fire_robot_mission.operational_contract import valid_gid

PACKAGE = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize('fault', ['none', 'spin', 'destroy', 'init'])
def test_installed_IR_entry_cleanup_order_and_signal_restore(monkeypatch, fault):
    events=[]; handlers={signal.SIGINT: 'old-int', signal.SIGTERM: 'old-term'}
    previous=dict(handlers); context={'ok': False}
    monkeypatch.setattr(ir.signal, 'getsignal', lambda sig: handlers[sig])
    monkeypatch.setattr(ir.signal, 'signal', lambda sig, fn: handlers.__setitem__(sig, fn))
    def init(**kwargs):
        events.append('init')
        assert kwargs['signal_handler_options'] == ir.SignalHandlerOptions.NO
        if fault=='init': raise RuntimeError('init fault')
        context['ok']=True
    def destroy():
        assert context['ok']; events.append('destroy/zero')
        if fault=='destroy': raise RuntimeError('destroy fault')
    def spin(node, **kwargs):
        events.append('spin')
        if fault=='spin': raise RuntimeError('spin fault')
        handlers[signal.SIGINT](signal.SIGINT,None)
    def shutdown():
        events.append('shutdown'); context['ok']=False
    monkeypatch.setattr(ir.rclpy,'init',init); monkeypatch.setattr(ir.rclpy,'ok',lambda: context['ok'])
    monkeypatch.setattr(ir.rclpy,'shutdown',shutdown); monkeypatch.setattr(ir.rclpy,'spin_once',spin)
    monkeypatch.setattr(ir,'IrApproachLiveNode',lambda: N(destroy_node=destroy))
    if fault=='none': ir.main()
    else:
        with pytest.raises(RuntimeError): ir.main()
    assert handlers==previous
    assert not context['ok']
    if fault!='init': assert events.index('destroy/zero') < events.index('shutdown')


@pytest.mark.parametrize('fault', ['spin', 'destroy'])
def test_pulse_exception_still_drains_OFF_before_shutdown(monkeypatch, fault):
    events=[]; context={'ok':False}; handlers={signal.SIGINT:None,signal.SIGTERM:None}
    previous=dict(handlers); now=[0.]
    def tick(): now[0]+=.1; return now[0]
    monkeypatch.setattr(pc.time,'monotonic',tick)
    monkeypatch.setattr(pc.signal,'getsignal',lambda sig: handlers[sig])
    monkeypatch.setattr(pc.signal,'signal',lambda sig,fn: handlers.__setitem__(sig,fn))
    monkeypatch.setattr(pc.rclpy,'init',lambda **k: context.update(ok=True))
    monkeypatch.setattr(pc.rclpy,'ok',lambda: context['ok'])
    def shutdown(): events.append('shutdown');context['ok']=False
    monkeypatch.setattr(pc.rclpy,'shutdown',shutdown)
    calls=[0]
    def spin(node,**kwargs):
        calls[0]+=1
        if calls[0]==1:
            if fault=='spin': raise RuntimeError('spin fault')
            handlers[signal.SIGINT](signal.SIGINT,None)
    monkeypatch.setattr(pc.rclpy,'spin_once',spin)
    def destroy():
        assert context['ok'];events.append('destroy')
        if fault=='destroy': raise RuntimeError('destroy fault')
    node=N(pub=N(publish=lambda msg: events.append('ON' if msg.data else 'OFF')),
           session=N(start_home=None,on_ack=False,inputs={}),
           get_publishers_info_by_topic=lambda t: [], destroy_node=destroy,
           get_logger=lambda: N(error=lambda *a: None,info=lambda *a: None))
    monkeypatch.setattr(pc,'PulseController',lambda:node)
    if fault=='destroy':
        with pytest.raises(RuntimeError): pc.main()
    else: assert pc.main()==2  # Missing stop evidence cannot become success.
    assert 'ON' not in events and 'OFF' in events
    assert events.index('OFF') < events.index('destroy') < events.index('shutdown')
    assert handlers==previous and not context['ok']


@pytest.mark.parametrize('value', [None, [], [0]*24, [1]*23, [256]+[0]*23, [True]+[0]*23])
def test_invalid_graph_gid_is_not_validated_as_owner(value):
    assert not valid_gid(N(endpoint_gid=value))


def test_configuration_and_entry_are_installed_package_relative():
    params=yaml.safe_load((PACKAGE/'config/operational_params.yaml').read_text())
    p=params['fire_pulse_controller']['ros__parameters']
    assert p['enable_pulse'] is p['allow_provisional_ir'] is p['allow_close_stationary'] is False
    i=params['ir_approach_live']['ros__parameters']
    assert i['enable_live_ir_jog'] is False and i['enable_reverse_jog'] is False
    assert i['speed_m_s']==.08 and i['max_total_approach_m']==i['continuous_distance_m']==.30
    bands=yaml.safe_load((PACKAGE/'config/ir_range_preview.yaml').read_text())['ir_range_preview']['ros__parameters']
    assert (bands['right_too_close_max'],bands['right_spray_max'],bands['right_approach_max'])==(800,1000,20000)
    assert bands['range_policy']=='YOLO_RIGHT_ADC_20261025'
    assert 'fire_pulse_controller = fire_robot_mission.pulse_controller:main' in (PACKAGE/'setup.py').read_text()
    tree=ast.parse((PACKAGE/'launch/operational_components.launch.py').read_text())
    assert not any(isinstance(n,ast.Name) and n.id in {'ExecuteProcess','SetEnvironmentVariable'} for n in ast.walk(tree))


def test_runtime_modules_do_not_import_QA_or_modify_sys_path():
    names=['pulse_controller','pulse_session','pulse_transport','pulse_registration',
           'actuator_drain','actuator_freshness','stationary_confirmation',
           'close_stationary_policy','ir_budget_stop','prespray_recovery']
    for name in names:
        mod=importlib.import_module('fire_robot_mission.'+name)
        path=Path(mod.__file__).resolve()
        assert 'fire_robot_mission' in path.parts
        source=path.read_text(); tree=ast.parse(source)
        for n in ast.walk(tree):
            if isinstance(n,ast.ImportFrom):
                assert not (n.module or '').startswith(('r3_','r4','test_'))
        assert 'sys.path' not in source and 'mission_overlay' not in source
