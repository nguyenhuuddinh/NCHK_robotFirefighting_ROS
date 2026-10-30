"""Scheduling adapter preserves the checker and schema; no actual ROS init."""
import json
from pathlib import Path
import signal
import tempfile
import types
import unittest
from unittest.mock import patch
from services.app_health_entry import main,health_command

class HealthEntryTests(unittest.TestCase):
    def fake(self,original):
        package=types.ModuleType('fire_robot_mission');package.startup_health=original
        return {'fire_robot_mission':package}
    def test_default_mode_dispatches_original_r5_main(self):
        calls=[];original=types.SimpleNamespace(main=lambda args:calls.append(args))
        with patch.dict('sys.modules',self.fake(original)):
            main(['--out','/tmp/not-written','--session','s','--role','pi'])
        self.assertEqual(calls,[['--out','/tmp/not-written','--session','s','--role','pi']])
    def test_sampled_mode_keeps_checker_data_and_timer_outside_spin_loop(self):
        with tempfile.TemporaryDirectory() as tmp:
            out=Path(tmp)/'health.json';events=[]
            class Node:
                def __init__(self,session,role):events.append(('node',session,role));self.cb=None
                def create_timer(self,seconds,cb):events.append(('timer',seconds));self.cb=cb
                def sample(self):events.append(('sample',));return dict(schema_version=1,commands_published=False,session='s',checks={'io':False})
                def destroy_node(self):events.append(('destroy',))
            def spin(node,**_):
                node.cb();signal.getsignal(signal.SIGTERM)(None,None)
            rclpy=types.ModuleType('rclpy');rclpy.init=lambda **_:events.append(('init',));rclpy.ok=lambda:True
            rclpy.spin_once=spin;rclpy.shutdown=lambda:events.append(('shutdown',))
            signals=types.ModuleType('rclpy.signals');signals.SignalHandlerOptions=types.SimpleNamespace(NO=0)
            modules=self.fake(types.SimpleNamespace(StartupHealthNode=Node));modules.update({'rclpy':rclpy,'rclpy.signals':signals})
            previous=signal.getsignal(signal.SIGTERM)
            with patch.dict('sys.modules',modules):main(['--out',str(out),'--session','s','--role','pi','--mode','sampled'])
            self.assertIn(('timer',.2),events);self.assertEqual(events.count(('sample',)),1)
            self.assertIn(('destroy',),events);self.assertIn(('shutdown',),events)
            self.assertIs(signal.getsignal(signal.SIGTERM),previous)
            self.assertFalse(json.loads(out.read_text())['checks']['io'])
            self.assertFalse(out.with_suffix('.tmp').exists())
    def test_command_has_no_ros2_run_installed_node_override(self):
        cmd=health_command('--out','x','--session','s','--role','pi')
        self.assertIn('services.app_health_entry',cmd);self.assertNotIn('ros2',cmd)
