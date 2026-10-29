"""Read-only entrypoint checks for observe and explicit Gate2 profiles."""
import importlib.util
import json
import os
from pathlib import Path
import subprocess
import yaml
from ament_index_python.packages import get_package_share_directory

PACKAGES=dict(pi=('fire_robot_bringup','fire_robot_description','fire_robot_safety','fire_robot_mission'),
              laptop=('fire_robot_mission','fire_robot_navigation','fire_robot_perception'))

GATE2_PARAMETERS={
    'fire_pulse_controller':dict(enable_cycle_protocol=True,enable_pulse=True,
        enable_prespray_cancel=False,allow_provisional_ir=True,allow_close_stationary=True,
        max_pulses_per_cycle=3,max_total_pump_on_s=18.,respray_settle_s=0.,respray_wait_s=0.,
        pulse_s=6.,start_on_ack=True,ir_confirm_samples=3,unbounded_approach=True,reposition_resets_quota=True),
    'ir_approach_live':dict(enable_cycle_protocol=True,enable_live_ir_jog=True,
        allow_provisional_ir=True,enable_reverse_jog=False,integrated_handoff=True,
        continuous_approach=True,unbounded_approach=True,speed_m_s=.08,continuous_distance_m=.30,max_total_approach_m=.30),
    'live_nav_stop_gateway':dict(enable_cycle_protocol=True,enable_live_output=True,
        enable_prespray_cancel=False),
    'nav2_cancel_guard':dict(enable_cycle_protocol=True,enable_real_cancel=True),
    'live_alignment_takeover':dict(enable_cycle_protocol=True,enable_live_alignment=True),
    'fire_mission_supervisor':dict(enable_cycle_protocol=True,enable_prespray_cancel=False),
    'post_spray_review':dict(use_operational_actor=True,max_pulses=1,ir_confirm_samples=3),
    'fire_robot_operator':dict(unbounded_approach=True,ir_confirm_samples=3),
    'ir_range_preview':dict(spray_confirm_samples=3),
    'alignment_handoff_shadow':dict(require_live_cancel_proof=True),
}

def validate_flags(config):
    for node,row in config.items():
        params=row.get('ros__parameters',{})
        for key,value in params.items():
            if (key.startswith('enable_') or key in ('use_operational_actor','allow_provisional_ir','allow_close_stationary','automatic_mode')) and value is not False:
                raise ValueError('OBSERVE_PROFILE_REJECTS_LIVE_OVERRIDE:'+node+':'+key)

def validate_gate2_flags(config):
    if not isinstance(config,dict) or set(config)!=set(GATE2_PARAMETERS):
        raise ValueError('GATE2_NODE_SET_NOT_EXACT')
    for node,expected in GATE2_PARAMETERS.items():
        row=config.get(node)
        if not isinstance(row,dict) or row.get('ros__parameters')!=expected:
            raise ValueError('GATE2_PARAMETERS_NOT_EXACT:'+node)
    return True

def validate_gate2_profile(root):
    root=Path(root)
    config=root/'ros2_ws/src/fire_robot_mission/config/operational_gate2.yaml'
    launch_file=root/'ros2_ws/src/fire_robot_mission/launch/operational_stack_gate2.launch.py'
    if not config.is_file() or not launch_file.is_file():raise ValueError('GATE2_PROFILE_SOURCE_MISSING')
    validate_gate2_flags(yaml.safe_load(config.read_text()))
    share=Path(get_package_share_directory('fire_robot_mission'))
    for source,installed in ((config,share/'config'/config.name),(launch_file,share/'launch'/launch_file.name)):
        if not installed.is_file() or installed.read_bytes()!=source.read_bytes():
            raise ValueError('GATE2_PROFILE_NOT_INSTALLED:'+source.name)
    return dict(profile='R5_GATE2_SUPERVISED',live_capable=True,reverse_jog=False,
                max_total_approach_m=None,unbounded_approach=True,max_pulses=3,max_total_pump_on_s=18.,
                quota_policy='CONSECUTIVE_AFTER_REPOSITION',pulse_clock='FIRST_FRESH_ESP_PWM_ON_ACK')

def installed_sources(root,role):
    root=Path(root);src=root/'ros2_ws/src';checked=[]
    for package in PACKAGES[role]:
        share=Path(get_package_share_directory(package))
        for folder,pattern in (('launch','*launch.py'),('config','*')):
            for p in (src/package/folder).glob(pattern):
                if p.is_file():
                    actual=share/folder/p.name
                    if not actual.exists() or p.read_bytes()!=actual.read_bytes():raise ValueError('INSTALLED_CONFIG_NOT_CANDIDATE:'+str(p))
                    checked.append(str(actual))
        module=src/package/package
        for p in module.glob('*.py'):
            name=package if p.name=='__init__.py' else package+'.'+p.stem
            spec=importlib.util.find_spec(name)
            if spec is None or spec.origin is None or Path(spec.origin).read_bytes()!=p.read_bytes():raise ValueError('INSTALLED_PYTHON_NOT_CANDIDATE:'+name)
            checked.append(spec.origin)
    validate_flags(yaml.safe_load((src/'fire_robot_mission/config/operational_params.yaml').read_text()))
    return checked

def preflight(root,role,*,run=subprocess.run):
    if os.environ.get('ROS_DOMAIN_ID','0')!='0' or os.environ.get('ROS_LOCALHOST_ONLY','0')!='0':raise ValueError('UNREVIEWED_ROS_DOMAIN_OR_LOCALHOST')
    if os.environ.get('RMW_FASTRTPS_USE_QOS_FROM_XML','0')=='1':raise ValueError('UNREVIEWED_XML_QOS_OVERRIDE')
    checked=installed_sources(root,role)
    pattern='[s]erial_bridge_node|[n]ative_mjpeg_camera|[b]t_navigator|[f]ire_mission_supervisor|[y]olo_observer|[c]ontroller_server|[s]afety_watchdog|[r]5_startup_health|[f]ire_robot_operator'
    existing=run(['pgrep','-af',pattern],capture_output=True,text=True,timeout=3,check=False)
    if existing.returncode!=1:raise ValueError('EXISTING_ROBOT_OR_UNKNOWN_PROCESS_SNAPSHOT')
    interface,wired,address,peer=('wlan0','eth0','10.0.0.1','10.0.0.253') if role=='pi' else ('wlp8s0','enp7s0','10.0.0.253','10.0.0.1')
    # Do not alter network settings. Use the reviewed WiFi pair only.
    if Path('/sys/class/net/'+wired+'/operstate').read_text().strip()=='up':raise ValueError('LAN_MUST_BE_DISCONNECTED_FOR_WIFI_PROFILE')
    info=run(['ip','-j','-4','address','show','dev',interface],capture_output=True,text=True,timeout=3,check=False)
    route=run(['ip','-j','route','get',peer],capture_output=True,text=True,timeout=3,check=False)
    if info.returncode or route.returncode:raise ValueError('WIFI_ROUTE_NOT_AVAILABLE')
    rows=json.loads(info.stdout);routes=json.loads(route.stdout)
    if not any(a.get('local')==address for row in rows for a in row.get('addr_info',[])) or not routes or routes[0].get('dev')!=interface:
        raise ValueError('WIFI_ADDRESS_OR_ROUTE_MISMATCH')
    return dict(installed_files_verified=len(checked),role=role,interface=interface,address=address,live=False)
