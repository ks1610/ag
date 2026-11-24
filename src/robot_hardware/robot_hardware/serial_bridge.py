import math
import time

import serial

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


class SerialBridge(Node):
    def __init__(self):
        super().__init__('serial_bridge')

        # Command and serial parameters
        self.declare_parameter(
            'serial_port',
            '/dev/serial/by-id/usb-1a86_USB_Serial-if00-port0'
        )
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('cmd_vel_topic', '/cmd_vel')
        self.declare_parameter('command_timeout', 0.3)
        self.declare_parameter('max_linear', 0.2)
        self.declare_parameter('max_angular', 1.0)

        # Deadband compensation (values are AFTER speed_scale, in m/s, rad/s).
        # 0.0 disables the feature.
        self.declare_parameter('min_linear', 0.0)
        self.declare_parameter('min_angular', 0.0)

        # Speed and wheel balance
        self.declare_parameter('speed_scale', 0.6)
        self.declare_parameter('left_slower_percent', 8.0)

        # Turn direction: set True if the robot turns opposite to the command
        self.declare_parameter('invert_turn', False)

        # Odometry parameters: calibrate on the real robot.
        self.declare_parameter('ticks_per_rev', 0.0)
        self.declare_parameter('wheel_radius', 0.0)
        self.declare_parameter('wheel_separation', 0.30)
        self.declare_parameter('invert_left', False)
        self.declare_parameter('invert_right', False)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        self.declare_parameter('odom_topic', '/odom')

        # Read parameters (all declared above)
        self.serial_port = self.get_parameter('serial_port').value
        self.baud_rate = self.get_parameter('baud_rate').value
        self.cmd_vel_topic = self.get_parameter('cmd_vel_topic').value
        self.command_timeout = self.get_parameter('command_timeout').value
        self.max_linear = self.get_parameter('max_linear').value
        self.max_angular = self.get_parameter('max_angular').value

        self.min_linear = float(self.get_parameter('min_linear').value)
        self.min_angular = float(self.get_parameter('min_angular').value)

        self.speed_scale = float(self.get_parameter('speed_scale').value)
        self.left_slower_percent = float(
            self.get_parameter('left_slower_percent').value)
        self.invert_turn = bool(self.get_parameter('invert_turn').value)

        self.ticks_per_rev = float(self.get_parameter('ticks_per_rev').value)
        self.wheel_radius = float(self.get_parameter('wheel_radius').value)
        self.wheel_separation = float(
            self.get_parameter('wheel_separation').value)
        self.invert_left = bool(self.get_parameter('invert_left').value)
        self.invert_right = bool(self.get_parameter('invert_right').value)
        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.odom_topic = self.get_parameter('odom_topic').value

        self.odom_enabled = (
            self.ticks_per_rev > 0.0
            and self.wheel_radius > 0.0
            and self.wheel_separation > 0.0
        )
        if self.odom_enabled:
            self.meters_per_tick = (
                2.0 * math.pi * self.wheel_radius / self.ticks_per_rev
            )
        else:
            self.meters_per_tick = 0.0
            self.get_logger().warn(
                'Odometry disabled. Set ticks_per_rev, wheel_radius and '
                'wheel_separation to calibrated values.'
            )

        self.get_logger().info(
            f'Speed scale={self.speed_scale:.2f}, left wheel '
            f'{self.left_slower_percent:.1f}% slower than right, '
            f'track={self.wheel_separation:.3f} m, '
            f'invert_turn={self.invert_turn}, '
            f'min_linear={self.min_linear:.3f}, '
            f'min_angular={self.min_angular:.3f}'
        )

        self.linear_x = 0.0
        self.angular_z = 0.0
        self.last_command_time = None
        self.last_tx_log_time = 0.0

        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0
        self.total_left = 0
        self.total_right = 0

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
            raise RuntimeError(f'Cannot open serial port: {error}')

        self.subscription = self.create_subscription(
            Twist,
            self.cmd_vel_topic,
            self.cmd_vel_callback,
            10
        )

        self.odom_pub = self.create_publisher(Odometry, self.odom_topic, 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        # Send commands at 20 Hz.
        self.timer = self.create_timer(0.05, self.send_serial_command)

        self.get_logger().info(f'Subscribed to {self.cmd_vel_topic}')

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

    def shape_command(self, linear, angular):
        """Apply speed reduction, left/right balance and deadband lift."""
        # Decide from the ORIGINAL command, before balancing adds a
        # small linear component to pure rotations.
        in_place = abs(linear) < 1e-3

        track = self.wheel_separation
        if track <= 0.0:
            linear *= self.speed_scale
            angular *= self.speed_scale
        else:
            half = track / 2.0
            v_left = linear - angular * half
            v_right = linear + angular * half

            gain_right = self.speed_scale
            gain_left = self.speed_scale * (
                1.0 - self.left_slower_percent / 100.0
            )
            v_left *= gain_left
            v_right *= gain_right

            linear = (v_left + v_right) / 2.0
            angular = (v_right - v_left) / track

        linear = max(-self.max_linear, min(self.max_linear, linear))
        angular = max(-self.max_angular, min(self.max_angular, angular))

        if in_place:
            # Rotating in place: lift only the angular part.
            if 0.0 < abs(angular) < self.min_angular:
                angular = math.copysign(self.min_angular, angular)
        else:
            # Driving: lift only the linear part.
            if 0.0 < abs(linear) < self.min_linear:
                linear = math.copysign(self.min_linear, linear)

        return linear, angular

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

        # Correct the turn direction if the wheels turn opposite to the command.
        if self.invert_turn:
            angular = -angular

        linear, angular = self.shape_command(linear, angular)

        # ROS units -> Arduino units (mm/s and mrad/s).
        linear_mm_s = round(linear * 1000.0)
        angular_mrad_s = round(angular * 1000.0)
        message = f'V {linear_mm_s} {angular_mrad_s}\n'

        try:
            self.serial.write(message.encode('ascii'))

            if now - self.last_tx_log_time >= 1.0:
                self.get_logger().info(
                    f'TX Arduino 1: {message.strip()} | '
                    f'encoder totals L={self.total_left} R={self.total_right}'
                )
                self.last_tx_log_time = now

            while self.serial.in_waiting:
                line = self.serial.readline().decode(
                    'ascii', errors='replace'
                ).strip()
                if not line:
                    continue

                self.get_logger().debug(f'RX Arduino 1: {line}')

                telemetry = self.parse_telemetry(line)
                if telemetry is not None:
                    self.handle_telemetry(telemetry)

        except serial.SerialException as error:
            self.get_logger().error(f'Serial communication error: {error}')

    def parse_telemetry(self, line):
        # Expected: O <dt_ms> <left_ticks> <right_ticks> <btn_upper> <btn_lower> <lift_state>
        parts = line.split()
        if len(parts) != 7 or parts[0] != 'O':
            return None
        try:
            return {
                'dt_ms': int(parts[1]),
                'left': int(parts[2]),
                'right': int(parts[3]),
                'btn_upper': int(parts[4]),
                'btn_lower': int(parts[5]),
                'lift_state': int(parts[6]),
            }
        except ValueError:
            return None

    def handle_telemetry(self, t):
        self.total_left += t['left']
        self.total_right += t['right']

        if not self.odom_enabled:
            return

        dt = t['dt_ms'] / 1000.0
        if dt <= 0.0:
            return

        sign_l = -1.0 if self.invert_left else 1.0
        sign_r = -1.0 if self.invert_right else 1.0
        dl = sign_l * t['left'] * self.meters_per_tick
        dr = sign_r * t['right'] * self.meters_per_tick

        ds = (dl + dr) / 2.0
        dtheta = (dl - dr) / self.wheel_separation

        mid = self.theta + dtheta / 2.0
        self.x += ds * math.cos(mid)
        self.y += ds * math.sin(mid)
        self.theta = normalize_angle(self.theta + dtheta)

        self.publish_odometry(ds / dt, dtheta / dt)

    def publish_odometry(self, v, w):
        stamp = self.get_clock().now().to_msg()
        qz = math.sin(self.theta / 2.0)
        qw = math.cos(self.theta / 2.0)

        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame
        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.orientation.z = qz
        odom.pose.pose.orientation.w = qw
        odom.twist.twist.linear.x = v
        odom.twist.twist.angular.z = w
        self.odom_pub.publish(odom)

        tf = TransformStamped()
        tf.header.stamp = stamp
        tf.header.frame_id = self.odom_frame
        tf.child_frame_id = self.base_frame
        tf.transform.translation.x = self.x
        tf.transform.translation.y = self.y
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self.tf_broadcaster.sendTransform(tf)

    def stop_motors(self):
        for _ in range(3):
            try:
                self.serial.write(b'V 0 0\n')
                self.serial.flush()
            except serial.SerialException:
                break
            time.sleep(0.02)

    def destroy_node(self):
        try:
            if hasattr(self, 'serial') and self.serial.is_open:
                self.stop_motors()
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