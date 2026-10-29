import importlib.util
from pathlib import Path
import yaml
import pytest
from operational_entry import packet_digest,read_health
from operational_preflight import validate_flags,validate_gate2_flags,GATE2_PARAMETERS
from r5_laptop_session import commands

ROOT=Path(__file__).parents[5]


def test_full_consumer_launch_is_all_six_plus_review_shadow_preview_and_false_flags(monkeypatch,tmp_path):
    import launch.logging
    monkeypatch.setattr(launch.logging.launch_config,'_log_dir',str(tmp_path))
    p=ROOT/'ros2_ws/src/fire_robot_mission/launch/operational_stack_observe.launch.py'
    spec=importlib.util.spec_from_file_location('observe_launch',p);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'get_package_share_directory',lambda name:str(ROOT/'ros2_ws/src'/name))
    ld=module.generate_launch_description();names=[n.node_executable for n in ld.entities]
    assert len(names)==10 and len(set(names))==10
    assert set(('fire_mission_supervisor','live_nav_stop_gateway','nav2_cancel_guard','live_alignment_takeover','ir_approach_live','fire_pulse_controller','post_spray_review'))<=set(names)
    cfg=yaml.safe_load((ROOT/'ros2_ws/src/fire_robot_mission/config/operational_params.yaml').read_text());validate_flags(cfg)
    assert 'enable_live_intents:=false' in commands(Path('map.yaml'),Path('view.rviz'))['operator']


@pytest.mark.parametrize('flag',['enable_pulse','enable_cycle_protocol','enable_live_intents','allow_provisional_ir','allow_close_stationary','use_operational_actor'])
def test_no_live_or_provisional_override_may_sneak_into_observe_entrypoint(flag):
    with pytest.raises(ValueError):validate_flags({'node':{'ros__parameters':{flag:True}}})


def test_packet_pin_rejects_changed_source_and_path_escape(tmp_path):
    import hashlib
    f=tmp_path/'safe.py';f.write_text('safe')
    (tmp_path/'operational_packet.sha256').write_text(hashlib.sha256(f.read_bytes()).hexdigest()+'  safe.py\n')
    assert len(packet_digest(tmp_path))==64
    f.write_text('changed')
    with pytest.raises(ValueError):packet_digest(tmp_path)
    (tmp_path/'operational_packet.sha256').write_text('0'*64+'  ../outside.py\n')
    with pytest.raises(ValueError):packet_digest(tmp_path)


def test_health_file_needs_fresh_this_session_and_no_command_writer(tmp_path):
    import json
    p=tmp_path/'health.json';value=dict(schema_version=1,session='this',sample_mono=10.,commands_published=False)
    p.write_text(json.dumps(value));assert read_health(p,'this',10.)==value
    for session,now in [('other',10.),('this',11.),('this',9.)]:
        with pytest.raises(ValueError):read_health(p,session,now)
    value['commands_published']=True;p.write_text(json.dumps(value))
    with pytest.raises(ValueError):read_health(p,'this',10.)


def test_gate2_profile_is_separate_exact_and_keeps_reverse_prespray_off(monkeypatch,tmp_path):
    import launch.logging
    monkeypatch.setattr(launch.logging.launch_config,'_log_dir',str(tmp_path))
    p=ROOT/'ros2_ws/src/fire_robot_mission/launch/operational_stack_gate2.launch.py'
    spec=importlib.util.spec_from_file_location('gate2_launch',p);module=importlib.util.module_from_spec(spec);spec.loader.exec_module(module)
    monkeypatch.setattr(module,'get_package_share_directory',lambda name:str(ROOT/'ros2_ws/src'/name))
    ld=module.generate_launch_description();names=[n.node_executable for n in ld.entities]
    assert len(names)==10 and len(set(names))==10
    cfg=yaml.safe_load((ROOT/'ros2_ws/src/fire_robot_mission/config/operational_gate2.yaml').read_text())
    assert validate_gate2_flags(cfg) is True
    assert cfg['ir_approach_live']['ros__parameters']['enable_reverse_jog'] is False
    assert cfg['ir_approach_live']['ros__parameters']['max_total_approach_m']==.30
    assert cfg['fire_pulse_controller']['ros__parameters']['enable_prespray_cancel'] is False
    assert cfg['fire_pulse_controller']['ros__parameters']['allow_close_stationary'] is True
    assert cfg['post_spray_review']['ros__parameters']['max_pulses']==1
    cmd=commands(Path('map.yaml'),Path('view.rviz'),gate2=True)
    assert 'operational_stack_gate2.launch.py' in cmd['mission']
    assert 'enable_live_intents:=true' in cmd['operator']
    assert 'allow_close_stationary:=true' in cmd['operator']


@pytest.mark.parametrize('mutation',[('drop',None),('reverse',True),('cap',.31),('pulse',2)])
def test_gate2_profile_rejects_missing_or_weakened_exact_parameter(mutation):
    import copy
    cfg={node:{'ros__parameters':copy.deepcopy(params)} for node,params in GATE2_PARAMETERS.items()}
    kind,value=mutation
    if kind=='drop':cfg.pop('nav2_cancel_guard')
    elif kind=='reverse':cfg['ir_approach_live']['ros__parameters']['enable_reverse_jog']=value
    elif kind=='cap':cfg['ir_approach_live']['ros__parameters']['max_total_approach_m']=value
    else:cfg['post_spray_review']['ros__parameters']['max_pulses']=value
    with pytest.raises(ValueError):validate_gate2_flags(cfg)
