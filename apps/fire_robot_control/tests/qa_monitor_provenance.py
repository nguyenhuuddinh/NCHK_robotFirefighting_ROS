"""App STOP monitor must never become the pinned R5 operator source."""
import os
import time
import rclpy
from fire_robot_mission.operational_session import OperationalSessionNode
from fire_robot_mission.startup_health import StartupHealthNode

def test_real_ros_monitor_has_separate_name_no_control_publishers_or_operator_pin():
    assert os.environ.get('ROS_DOMAIN_ID')=='211' and os.environ.get('ROS_LOCALHOST_ONLY')=='1'
    rclpy.init(args=['monitor_provenance','--ros-args','-r','__node:=fire_robot_app_stop_monitor',
        '-p','show_gui:=false','-p','enable_live_intents:=false','-p','automatic_mode:=false'])
    monitor=health=None
    try:
        monitor=OperationalSessionNode()
        # Avoid applying the monitor's global name remap to the health node.
        from unittest.mock import patch
        import fire_robot_mission.startup_health as module
        real_init=module.Node.__init__
        def isolated_init(self,name,**kw):return real_init(self,name,use_global_arguments=False,**kw)
        with patch.object(module.Node,'__init__',isolated_init):
            health=StartupHealthNode('fixture','laptop')
        assert monitor.get_name()=='fire_robot_app_stop_monitor'
        assert monitor.intent_executor.enabled is False
        deadline=time.monotonic()+1.3
        while time.monotonic()<deadline:
            rclpy.spin_once(health,timeout_sec=.01)
        infos=health.get_publishers_info_by_topic('/fire_mission/operator_state')
        assert any(x.node_name=='fire_robot_app_stop_monitor' for x in infos)
        assert health.source('operator') is False
        assert 'operator' not in health.pins
        assert not health.get_publishers_info_by_topic('/cmd_vel_raw')
        assert not health.get_publishers_info_by_topic('/pump_cmd')
    finally:
        if health is not None:health.destroy_node()
        if monitor is not None:
            monitor.intent_executor.close();monitor.destroy_node()
        rclpy.shutdown()
