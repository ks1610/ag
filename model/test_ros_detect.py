#!/usr/bin/env python3
"""Run YOLO box detection on the ROS topic /camera/image_raw."""
import os

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from ultralytics import YOLO

MODEL_PATH = os.path.expanduser('~/ag/model/best.pt')


class DetectTest(Node):
    def __init__(self):
        super().__init__('detect_test')
        self.bridge = CvBridge()
        self.model = YOLO(MODEL_PATH)
        self.last_log_ns = 0
        self.create_subscription(Image, '/camera/image_raw', self.cb, qos_profile_sensor_data)
        self.get_logger().info(f'Model classes: {self.model.names}')
        self.get_logger().info('Listening on /camera/image_raw')

    def cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        result = self.model.predict(frame, conf=0.4, imgsz=320, verbose=False)[0]

        # Log at most once per second so the console stays readable.
        now = self.get_clock().now().nanoseconds
        if now - self.last_log_ns < 1_000_000_000:
            return
        self.last_log_ns = now

        if result.boxes is None or len(result.boxes) == 0:
            self.get_logger().info('no box')
            return
        for box in result.boxes:
            name = self.model.names[int(box.cls)]
            x1, y1, x2, y2 = box.xyxy[0].tolist()
            self.get_logger().info(
                f'{name} conf={float(box.conf):.2f} cx={(x1+x2)/2:.0f} cy={(y1+y2)/2:.0f}')


def main():
    rclpy.init()
    node = DetectTest()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
