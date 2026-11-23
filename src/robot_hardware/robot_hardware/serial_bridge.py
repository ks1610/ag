
import time
import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        self.declare_parameter(
            'serial_port',
            '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
        )
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('command_timeout', 0.3)
        self.declare_parameter('max_linear', 0.2)
        self.declare_parameter('max_angular', 1.0)

        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_command_time = None
        self.last_tx_log_time = 0.0

        try:
            self.serial = serial.Serial(
                port=self.serial_port,
                baudrate=self.baud_rate,
                timeout=0.01,
                write_timeout=0.1
            )

            # Arduino 1 may reset when the serial port opens.
            time.sleep(2.0)
            self.serial.reset_input_buffer()
            self.serial.reset_output_buffer()

            self.get_logger().info(
                f'Connected to Arduino 1: {self.serial_port}'
            )

        except serial.SerialException as error:
            raise RuntimeError(
                f'Cannot open serial port: {error}'
            )

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        # Send commands at 20 Hz.
        self.timer = self.create_timer(
            0.05,
            self.send_serial_command
        )

        self.get_logger().info(
            f'Subscribed to {self.cmd_vel_topic}'
        )

    def cmd_vel_callback(self, msg):
        self.linear_x = max(
            -self.max_linear,
            min(self.max_linear, msg.linear.x)
        )

        self.angular_z = max(
            -self.max_angular,
            min(self.max_angular, msg.angular.z)
        )

        self.last_command_time = time.monotonic()

    def send_serial_command(self):
        now = time.monotonic()

        # Stop if no fresh ROS command has arrived.
        if (
            self.last_command_time is None
            or now - self.last_command_time > self.command_timeout
        ):
            linear = 0.0
            angular = 0.0
        else:
            linear = self.linear_x
            angular = self.angular_z

        # ROS units -> Arduino units.
        linear_mm_s = round(linear * 1000.0)
        angular_mrad_s = round(angular * 1000.0)

        # Match the existing Arduino 1 UART protocol.
        message = (
            f'V {linear_mm_s} {angular_mrad_s}\n'
        )

        try:
            self.serial.write(message.encode('ascii'))

            # Log transmitted command once per second.
            if now - self.last_tx_log_time >= 1.0:
                self.get_logger().info(
                    f'TX Arduino 1: {message.strip()}'
                )
                self.last_tx_log_time = now

            # Read all currently available telemetry lines.
            while self.serial.in_waiting:
                response = self.serial.readline().decode(
                    'ascii',
                    errors='replace'
                ).strip()

                if response:
                    self.get_logger().debug(
                        f'RX Arduino 1: {response}'
                    )

        except serial.SerialException as error:
            self.get_logger().error(
                f'Serial communication error: {error}'
            )

    def destroy_node(self):
        try:
            if hasattr(self, 'serial') and self.serial.is_open:
                self.serial.write(b'V 0 0\n')
                self.serial.flush()
                self.serial.close()

        except serial.SerialException:
            pass

        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)

    node = None

    try:
        node = SerialBridge()
        rclpy.spin(node)

    except (RuntimeError, KeyboardInterrupt) as error:
        if not isinstance(error, KeyboardInterrupt):
            print(error)

    finally:
        if node is not None:
            node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
