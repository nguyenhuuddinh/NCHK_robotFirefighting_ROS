"""Execute mapping/STOP-only main with fake ROS/processes; local files/socket only."""
import json
from pathlib import Path
import os
import signal
import tempfile
import time
from types import SimpleNamespace as Obj
from unittest.mock import patch
import uuid
import pytest
import rclpy
import operational_entry as original
from services import mapping_runner as mapping
import services.app_peer
from services.operator_runner import OperatorBootstrap
from services.operator_bootstrap import create_bootstrap_file
from proof_fixture import clean_local,clean_remote

@pytest.mark.parametrize('verify',[False,True])
def test_actual_mapping_main_seal_before_slam_or_stop_only_no_consumers(verify):
    with tempfile.TemporaryDirectory() as tmp:
        base=Path(tmp);os.chmod(base,0o700);maps=base/'maps';maps.mkdir()
        session=str(uuid.uuid4());packet='a'*64;calls=[];inits=[]
        boot=OperatorBootstrap(session,'b'*32,'c'*32,packet,base/'keyboard.sock')
        pairing=base/'pairing.json';pairing.write_text(json.dumps({'packet':packet}))
        (base/'display.json').write_text(json.dumps(dict(session=session,sample_mono=time.monotonic(),view_ready=True)))
        class Peer:
            def __init__(self,*_):self.bind={'packet':packet}
            def startup(self,cmd):
                calls.append(('peer',cmd))
                return dict(phase='WAIT_LAPTOP') if cmd=='STATE' else dict(phase='RUNNING_OBSERVE',sealed=True,health={'io':True})
            def stop(self):calls.append(('pi_stop',));return clean_remote({'packet':packet})
        class Backend:
            def __init__(self):self.child=Obj(write=lambda _:None,close_after_exit=lambda:None)
            def pump(self):pass
        class Process:
            done=False
            def __init__(self,role):self.role=role
            def tick(self,_):
                if self.role=='rviz':self.done=True
            def stop(self,_):self.done=True
            def report(self):return dict(group_exit_confirmed=True)
        class Local:
            def __init__(self,out,sid,**_):
                self.processes={};self.stream=tempfile.TemporaryFile();self.stream.write(b'MAPPING_MONITOR_READY\n');self.stream.flush()
                self.registry=Obj(logs={'operator':Obj(file=self.stream)})
            def start(self,role,argv,**_):
                calls.append(('start',role,list(argv)));self.processes[role]=Process(role)
            def check(self,_):
                if 'rviz' in self.processes:signal.getsignal(signal.SIGINT)(None,None)
            def drain(self,**_):calls.append(('local_drain',));return clean_local()
            def close_logs(self):self.stream.close()
        endpoint=Obj(node_name='slam_toolbox',node_namespace='/',topic_type='nav_msgs/msg/OccupancyGrid',endpoint_gid=bytes([1]*16))
        class Probe:
            def get_publishers_info_by_topic(self,topic):return [endpoint] if topic=='/map' else []
            def get_node_names_and_namespaces(self):return [('slam_toolbox','/')]
            def create_subscription(self,typ,topic,callback,qos):
                callback(Obj(header=Obj(frame_id='map'),info=Obj(width=1,height=1),data=[0]))
            def destroy_node(self):pass
        handlers={sig:signal.getsignal(sig) for sig in (signal.SIGINT,signal.SIGTERM)}
        try:
            with create_bootstrap_file(boot,base) as file:
                fd=os.dup(file.fileno())
                args=['--bootstrap-fd',str(fd),'--pairing',str(pairing),'--remote-script','fake-peer',
                      '--out',str(base/'laptop'),'--map-root',str(maps)]
                if verify:args.append('--verify-stop')
                with patch.object(original,'packet_digest',return_value=packet),patch.object(original,'LocalProcesses',Local), \
                     patch.object(original,'run_clock_gate'),patch.object(original,'read_health',return_value={'checks':{'io':True},'stable_s':{'io':1.}}), \
                     patch('operational_preflight.preflight'),patch.object(services.app_peer,'AppPeer',Peer), \
                     patch.object(mapping,'PtyBackend',Backend),patch('services.monitor_log.start_monitor',side_effect=lambda local, argv:local.start('operator',argv)),patch.object(Path,'home',return_value=base), \
                     patch.object(rclpy,'init',side_effect=lambda **kw:inits.append(kw)), \
                     patch.object(rclpy,'create_node',return_value=Probe()),patch.object(rclpy,'spin_once'), \
                     patch.object(rclpy,'ok',return_value=True),patch.object(rclpy,'shutdown'):
                    rc=mapping.run(args)
        finally:
            for sig,handler in handlers.items():signal.signal(sig,handler)
        failure=base/"laptop/failure.json"
        assert rc==0,(failure.read_text() if failure.exists() else "no failure receipt",calls)
        receipt=json.loads((base/'laptop/receipt.json').read_text())
        assert receipt['profile']==('DESKTOP_STOP_RECOVERY' if verify else 'DESKTOP_MAPPING')
        assert calls.index(('local_drain',)) < calls.index(('pi_stop',))
        starts=[row[1] for row in calls if row[0]=='start']
        if verify:
            assert starts==['operator','startup_health']
            assert ('peer','SEAL') not in calls and not inits
        else:
            assert starts==['operator','startup_health','nav','rviz']
            assert calls.index(('peer','SEAL')) < next(i for i,row in enumerate(calls) if row[:2]==('start','nav'))
            assert 'slam.launch.py' in next(row for row in calls if row[:2]==('start','nav'))[2][4]
            teleop = next(row[2] for row in calls if row[:2] == ('start','rviz'))
            assert teleop == ['ros2','run','teleop_twist_keyboard','teleop_twist_keyboard','--ros-args',
                              '-r','cmd_vel:=/cmd_vel_raw','-p','speed:=0.1','-p','turn:=0.8']
        assert not any('nav2' in str(row) or 'perception.launch' in str(row) for row in calls)
