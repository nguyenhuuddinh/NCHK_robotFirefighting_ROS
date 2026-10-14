#!/usr/bin/env python3
"""Display the existing compressed YOLO overlay without changing ROS state."""

import cv2
import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage


def main():
    """Display YOLO's compressed annotated image until Q or shutdown."""
    rclpy.init()
    node = rclpy.create_node('fire_annotated_viewer')
    current = [None]

    def on_image(message):
        current[0] = cv2.imdecode(
            np.frombuffer(message.data, dtype=np.uint8), cv2.IMREAD_COLOR)

    node.create_subscription(
        CompressedImage, '/yolo/image_annotated/compressed',
        on_image, qos_profile_sensor_data)
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.03)
            if current[0] is not None:
                cv2.imshow('Fire robot - YOLO camera', current[0])
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
