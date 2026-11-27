#!/usr/bin/env python3
"""
mission_node.py (real robot)

Flow:
  1. Nav2 drives to the place-1 pose (map frame).
  2. Robot drives straight forward for N wheel revolutions, measured from
     encoder-based /odom.
  3. Publishes the pick request to /pick_box_request. box_picker_node then
     detects the box with the webcam, centers on it and picks it up.
"""
import json
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import String
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult


class MissionNode(Node):

    def __init__(self):
        super().__init__('mission_node')

        defaults = {
            'place_x': 2.875,
            'place_y': 0.530,
            'place_yaw': 2.795,
            'forward_revs': 5.0,
            'wheel_radius': 0.03,
            'drive_speed': 0.08,
            'drive_timeout': 30.0,
            'pick_x': 2.875,
            'pick_y': 0.530,
            'pick_yaw': 2.795,
            'qr_code': 'mmb-6769',
        }
        self.p = {}
        for name, value in defaults.items():
            self.declare_parameter(name, value)
            self.p[name] = self.get_parameter(name).value

        self.odom_xy = None
        self.create_subscription(Odometry, '/odom', self._odom_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.pick_pub = self.create_publisher(String, '/pick_box_request', 10)

    def _odom_cb(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)

    def stop(self):
        for _ in range(5):
            self.cmd_pub.publish(Twist())
            time.sleep(0.05)

    def drive_forward(self, revs):
        distance = revs * 2.0 * math.pi * self.p['wheel_radius']
        self.get_logger().info(f'Driving forward {revs} rev = {distance:.3f} m')

        # Wait for the first odometry message.
        t_wait = time.monotonic()
        while self.odom_xy is None:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t_wait > 5.0:
                self.get_logger().error('No /odom received, aborting drive.')
                return False

        sx, sy = self.odom_xy
        twist = Twist()
        twist.linear.x = float(self.p['drive_speed'])
        traveled = 0.0
        t0 = time.monotonic()
        ok = False
        try:
            while True:
                rclpy.spin_once(self, timeout_sec=0.05)
                if self.odom_xy is not None:
                    x, y = self.odom_xy
                    traveled = math.hypot(x - sx, y - sy)
                if traveled >= distance:
                    ok = True
                    break
                if time.monotonic() - t0 > self.p['drive_timeout']:
                    self.get_logger().error(
                        f'Drive timeout: traveled {traveled:.3f} of {distance:.3f} m')
                    break
                self.cmd_pub.publish(twist)   # bridge needs a steady stream
                time.sleep(0.1)
        finally:
            self.stop()

        self.get_logger().info(f'Drive done, traveled {traveled:.3f} m')
        return ok

    def send_pick_request(self):
        t0 = time.monotonic()
        while self.pick_pub.get_subscription_count() == 0:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.monotonic() - t0 > 15.0:
                self.get_logger().error('box_picker_node is not subscribed.')
                return False

        payload = {
            'x': float(self.p['pick_x']),
            'y': float(self.p['pick_y']),
            'yaw': float(self.p['pick_yaw']),
            'qr_code': str(self.p['qr_code']),
        }
        msg = String()
        msg.data = json.dumps(payload)
        self.pick_pub.publish(msg)
        self.get_logger().info(f'Pick request sent: {msg.data}')
        return True


def make_pose(navigator, x, y, yaw):
    pose = PoseStamped()
    pose.header.frame_id = 'map'
    pose.header.stamp = navigator.get_clock().now().to_msg()
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.orientation.z = math.sin(yaw / 2.0)
    pose.pose.orientation.w = math.cos(yaw / 2.0)
    return pose


def main():
    rclpy.init()
    node = MissionNode()
    navigator = BasicNavigator()
    p = node.p

    try:
        node.get_logger().info('Waiting for Nav2 to become active...')
        navigator.waitUntilNav2Active()

        # 1. Drive to place 1 and pose.
        node.get_logger().info(
            f"Navigating to place 1 ({p['place_x']}, {p['place_y']}, {p['place_yaw']})")
        navigator.goToPose(make_pose(navigator, p['place_x'], p['place_y'], p['place_yaw']))
        while not navigator.isTaskComplete():
            rclpy.spin_once(node, timeout_sec=0.1)

        if navigator.getResult() != TaskResult.SUCCEEDED:
            node.get_logger().error('Nav2 did not reach place 1. Mission stopped.')
            return

        node.get_logger().info('Reached place 1. Settling...')
        time.sleep(1.0)

        # 2. Drive forward N wheel revolutions.
        if not node.drive_forward(p['forward_revs']):
            node.get_logger().error('Forward drive failed. Pick request NOT sent.')
            return

        # 3. Send pick request.
        time.sleep(0.5)
        node.send_pick_request()

    except KeyboardInterrupt:
        pass
    finally:
        node.stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()