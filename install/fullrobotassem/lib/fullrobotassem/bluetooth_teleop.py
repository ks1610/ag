#!/usr/bin/env python3
"""Bluetooth serial teleop: app letters -> /cmd_vel.

App commands:  F forward, B backward, R spin right, L spin left,
               FR forward+right, FL forward+left.
'S' is ignored (the app sends it as a separator); the robot stops when
no command arrives for `cmd_timeout` seconds.
"""
import os
import re
import select
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

# Longest match first so "FR" is not read as "F" then "R"
TOKEN = re.compile(r'FR|FL|F|B|R|L|S')


class BluetoothTeleop(Node):
    def __init__(self):
        super().__init__('bluetooth_teleop')
        self.declare_parameter('port', '/dev/rfcomm0')
        self.declare_parameter('baud', 9600)
        self.declare_parameter('linear_speed', 0.20)    # m/s (plugin max 0.27)
        self.declare_parameter('spin_speed', 0.8)       # rad/s (plugin max 1.5)
        self.declare_parameter('curve_angular', 0.5)    # rad/s for FR / FL
        self.declare_parameter('cmd_timeout', 0.0)      # s without data -> stop
        self.declare_parameter('debug', False)          # log every received packet

        self.port = self.get_parameter('port').value
        self.baud = self.get_parameter('baud').value
        v = self.get_parameter('linear_speed').value
        w = self.get_parameter('spin_speed').value
        c = self.get_parameter('curve_angular').value
        self.timeout = self.get_parameter('cmd_timeout').value
        self.debug = self.get_parameter('debug').value

        self.commands = {
            'F':  (v,    0.0),
            'B':  (-v,   0.0),
            'R':  (0.0, -w),
            'L':  (0.0,  w),
            'FR': (v,   -c),
            'FL': (v,    c),
            'S':  (0.0,  0.0),
        }

        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.ser = None
        self.port_ino = None
        self.last_error = ''
        self.current = (0.0, 0.0)
        self.last_rx = self.get_clock().now()
        self.create_timer(0.05, self.loop)  # 20 Hz
        self.get_logger().info(f'Waiting for Bluetooth serial on {self.port}')

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud, timeout=0)
            self.port_ino = os.stat(self.port).st_ino
            self.get_logger().info('Bluetooth connected')
            self.last_error = ''
        except (serial.SerialException, OSError) as e:
            self.ser = None
            if str(e) != self.last_error:   # log each new error once
                self.last_error = str(e)
                self.get_logger().warn(f'Cannot open {self.port}: {e}')

    def close(self):
        # Always release the port, otherwise rfcomm cannot free the device
        try:
            if self.ser is not None:
                self.ser.close()
        except Exception:
            pass
        self.ser = None
        self.current = (0.0, 0.0)

    def port_alive(self):
        """False if the phone hung up or /dev/rfcomm0 was recreated."""
        try:
            if os.stat(self.port).st_ino != self.port_ino:
                return False
            p = select.poll()
            p.register(self.ser.fileno(), select.POLLIN)
            for _, ev in p.poll(0):
                if ev & (select.POLLHUP | select.POLLERR | select.POLLNVAL):
                    return False
            return True
        except (OSError, ValueError):
            return False

    def loop(self):
        if self.ser is not None and not self.port_alive():
            self.get_logger().warn('Bluetooth disconnected, waiting for reconnect')
            self.close()
        if self.ser is None:
            self.connect()
            self.current = (0.0, 0.0)
        else:
            try:
                n = self.ser.in_waiting
                if n:
                    raw = self.ser.read(n)
                    tokens = TOKEN.findall(raw.decode('ascii', errors='ignore').upper())
                    if self.debug:
                        self.get_logger().info(f'rx={raw!r} tokens={tokens}')
                    if tokens:
                        self.current = self.commands[tokens[-1]]
                        self.last_rx = self.get_clock().now()
            except (serial.SerialException, OSError):
                self.get_logger().warn('Bluetooth lost, stopping robot')
                self.close()

        if self.timeout > 0:
            age = (self.get_clock().now() - self.last_rx).nanoseconds / 1e9
            if age > self.timeout:
                self.current = (0.0, 0.0)

        msg = Twist()
        msg.linear.x, msg.angular.z = self.current
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = BluetoothTeleop()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.pub.publish(Twist())
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
