#!/usr/bin/env python3
"""
request_pick.py
----------------
Convenience CLI to trigger box_picker_node.

Usage:
    python3 request_pick.py --qr mmb-6769
    python3 request_pick.py --x 1.8 --y 0.6 --qr mmb-6769
    python3 request_pick.py --x 1.8 --y 0.6 --yaw 1.57 --qr mmb-6769
"""
import argparse
import json
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def main():
    parser = argparse.ArgumentParser(description='Send a pick-box request to box_picker_node.')
    parser.add_argument('--x', type=float, default=3.228, help='Target X position (map frame, meters)')
    parser.add_argument('--y', type=float, default=0.620, help='Target Y position (map frame, meters)')
    parser.add_argument('--yaw', type=float, default=3.131, help='Target yaw in radians')
    parser.add_argument('--qr', type=str, required=True,
                         help='Expected product code encoded in the box QR label, e.g. mmb-6769')
    args = parser.parse_args()

    rclpy.init()
    node = Node('pick_box_request_sender')
    pub = node.create_publisher(String, '/pick_box_request', 10)
    time.sleep(1.0)  # give discovery a moment so the message isn't dropped

    payload = {'x': args.x, 'y': args.y, 'yaw': args.yaw, 'qr_code': args.qr}
    msg = String(data=json.dumps(payload))
    pub.publish(msg)
    node.get_logger().info(f'Sent pick request: {msg.data}')

    time.sleep(0.5)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()