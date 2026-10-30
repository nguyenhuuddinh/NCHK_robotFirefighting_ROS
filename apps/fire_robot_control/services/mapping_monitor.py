"""STOP-only use of the existing R5 operator. No tick/arm/fire policy runs."""
import json
import signal
import time


def main():
    import rclpy
    from rclpy.signals import SignalHandlerOptions
    from fire_robot_mission.operational_session import OperationalSessionNode
    stopping = [False]
    for sig in (signal.SIGINT, signal.SIGTERM):
        signal.signal(sig, lambda *_: stopping.__setitem__(0, True))
    rclpy.init(args=["mapping_stop_monitor", "--ros-args", "-r", "__node:=fire_robot_app_stop_monitor",
                     "-p", "show_gui:=false",
                     "-p", "enable_live_intents:=false", "-p", "automatic_mode:=false"],
                signal_handler_options=SignalHandlerOptions.NO)
    node = None
    rc = 2
    try:
        node = OperationalSessionNode()
        print("MAPPING_MONITOR_READY", flush=True)
        # Callbacks only. core.step/tick/AutomaticPolicy are deliberately not
        # executed while mapping. Existing watchdog handles teleop timeouts.
        while rclpy.ok() and not stopping[0]:
            rclpy.spin_once(node, timeout_sec=.03)
    finally:
        if node is not None:
            node.request_shutdown("MAPPING_STOP")
            while rclpy.ok() and not node.stop_verification.done:
                rclpy.spin_once(node, timeout_sec=.03)
                node.stop_tick()
            if not node.stop_verification.done:
                node.stop_verification.abort("ROS_CONTEXT_LOST")
            result = node.stop_verification.report(time.monotonic())
            node.get_logger().info("OPERATOR_STOP " + json.dumps(result))
            rc = 0 if result["confirmed"] and node.core.fault is None else 2
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return rc


if __name__ == "__main__":
    raise SystemExit(main())
