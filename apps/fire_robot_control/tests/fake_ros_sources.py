#!/usr/bin/env python3
"""Publish fake display-only telemetry for an explicitly isolated GUI test."""
import argparse
import os
import time


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--seconds", type=float, default=8.0)
    parser.add_argument("--duplicate-operator", action="store_true")
    args = parser.parse_args()
    if (os.environ.get("FIRE_ROBOT_ALLOW_FAKE_ROS") != "1"
            or os.environ.get("ROS_DOMAIN_ID") != "212"
            or os.environ.get("ROS_LOCALHOST_ONLY") != "1"
            or not 0 < args.seconds <= 15):
        parser.error("Fake publishers require explicit opt-in, domain 212 and localhost only")

    import cv2
    import numpy as np
    import rclpy
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import CompressedImage
    from std_msgs.msg import String

    rclpy.init()
    nodes = [rclpy.create_node(name) for name in
             ("yolo_observer", "fire_robot_operator", "serial_bridge_node")]
    if args.duplicate_operator:
        nodes.append(rclpy.create_node("foreign_operator"))
    camera = nodes[0].create_publisher(CompressedImage, "/yolo/image_annotated/compressed", qos_profile_sensor_data)
    operator = nodes[1].create_publisher(String, "/fire_mission/operator_state", qos_profile_sensor_data)
    actuator = nodes[2].create_publisher(String, "/fire_sensors/actuator_state", qos_profile_sensor_data)
    foreign = nodes[3].create_publisher(String, "/fire_mission/operator_state", qos_profile_sensor_data) if args.duplicate_operator else None
    image = np.zeros((180, 320, 3), dtype=np.uint8)
    cv2.putText(image, "FAKE TEST", (25, 100), cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 2)
    ok, jpg = cv2.imencode(".jpg", image)
    if not ok:
        raise RuntimeError("Cannot create test JPEG")
    end = time.monotonic() + args.seconds
    try:
        while time.monotonic() < end:
            frame = CompressedImage()
            frame.header.stamp = nodes[0].get_clock().now().to_msg()
            frame.header.frame_id = "camera_frame"
            frame.format = "jpeg"
            frame.data = jpg.tobytes()
            camera.publish(frame)
            message = String(data='{"schema_version":1,"scope":"R5_SESSION_INTENT_ONLY","task":{"stage":"NAVIGATION","reason":"FIXTURE"},"yolo_fresh":true,"fire_confirmed":false,"ir_counts":{"left":23000,"center":24000,"right":25000},"ir_source_age_ms":10,"range_confirm_samples":0,"ir_confirm_required":3}')
            operator.publish(message)
            if foreign:
                foreign.publish(message)
            actuator.publish(String(data='{"schema_version":1,"mode":"COMMAND_TELEMETRY_ONLY","pump_pwm_commanded_on":false,"servo_phase":"IDLE","receive_age_ms":2}'))
            for node in nodes:
                rclpy.spin_once(node, timeout_sec=0)
            time.sleep(0.1)
    finally:
        for node in nodes:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
