#!/usr/bin/env python3
"""
box_picker_node.py

Quy trình:
1. Nhận yêu cầu /pick_box_request (JSON: x, y, yaw, qr_code).
2. Dùng Nav2 đi đến vị trí mục tiêu.
3. Dùng YOLO phát hiện thùng.
4. Xoay tại chỗ để đưa tâm thùng vào giữa ảnh RGB.
5. Tiến đến thùng bằng LiDAR, dừng tại lidar_approach_stop.
6. (Tùy chọn) Tiến đoạn cuối bằng odometry.
7. Thực hiện chuỗi thao tác gắp.

LƯU Ý:
- Camera RGB chỉ dùng phát hiện/căn giữa, không dùng để đo chiều sâu.
- LiDAR có range_min=0.30 m trong cấu hình hiện tại.
- Đoạn tiến cuối bằng odometry là đoạn không có đo khoảng cách LiDAR tin cậy.
  Chỉ bật enable_final_odom_approach sau khi xác minh hình học và an toàn robot.
"""

import json
import math
import time
import threading
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from std_msgs.msg import String, Float64
from sensor_msgs.msg import Image, LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseStamped, Twist
from cv_bridge import CvBridge

from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult

try:
    from pyzbar.pyzbar import decode as qr_decode
    HAVE_PYZBAR = True
except ImportError:
    HAVE_PYZBAR = False

try:
    from ultralytics import YOLO
    HAVE_YOLO = True
except ImportError:
    HAVE_YOLO = False

try:
    import torch
    import open_clip
    from PIL import Image as PILImage
    HAVE_CLIP = True
except ImportError:
    HAVE_CLIP = False


CONFIRM_FRAMES = 3
HISTORY_LEN = 5
CLIP_THRESH = 0.60

POSITIVE_PROMPTS = [
    "a photo of a cardboard box",
    "a photo of a brown cardboard package",
]
NEGATIVE_PROMPTS = [
    "a photo of a wooden door",
    "a photo of a picture frame",
    "a photo of a wall",
    "a photo of wooden furniture",
    "a photo of a table or floor",
    "a photo of a window",
    "a photo of a cabinet",
]


def deg2rad(degrees):
    return degrees * math.pi / 180.0


def clamp(value, low, high):
    return max(low, min(high, value))


class BoxPickerNode(Node):

    def __init__(self):
        super().__init__('box_picker_node')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------
        defaults = {
            'model_path': '/home/khanhtrinh/trinh/ag/model/cardbox_train/weights/best.pt',
            'conf_thresh': 0.4,
            'imgsz': 320,
            'use_clip': True,
            'clip_thresh': CLIP_THRESH,
            'detect_timeout': 15.0,
            'qr_timeout': 5.0,

            'nav_timeout': 120.0,
            'nav_retries': 2,
            'nav_stall_timeout': 25.0,
            'nav_stall_timeout_wall': 40.0,
            'nav_retry_backup_distance': 0.3,
            'nav_retry_backup_speed': 0.08,
            'nav_retry_spin_rad': 2.6,
            'refine_approach': True,
            'refine_timeout': 20.0,

            # Visual centering
            'center_timeout_wall': 30.0,
            'center_tolerance': 0.06,
            'servo_angular_gain': 0.6,
            'servo_max_angular': 0.35,
            'servo_direction_sign': 1.0,

            # LiDAR approach
            'approach_speed': 0.08,
            'lidar_approach_speed_limit': 0.04,
            'lidar_approach_stop': 0.35,
            'lidar_sector_deg': 10.0,
            'lidar_min_points': 5,
            'lidar_data_timeout': 0.5,
            'approach_timeout_wall': 120.0,
            'approach_log_interval': 1.0,

            # Geometry from the URDF supplied earlier
            'camera_x': 0.303411,
            'lidar_x': 0.143411,
            'camera_target_distance': 0.05,

            # Final odometry segment:
            # 0.35 - (0.303411 - 0.143411) - 0.05 = 0.14 m
            'enable_final_odom_approach': True,
            'final_odom_speed': 0.03,
            'final_odom_timeout_wall': 12.0,
            'final_odom_distance_limit': 0.14,

            # Manipulator
            'upper_body_down': -0.30,
            'upper_body_up': 0.065,
            'left_arm_open_deg': 60.0,     
            'right_arm_open_deg': -60.0,   
            'open_duration': 2.0,
            'left_arm_pick_deg': -40.0,
            'right_arm_pick_deg': 40.0,
            'lower_duration': 4.0,
            'arm_duration': 2.0,

            'require_qr': False,
        }

        for name, value in defaults.items():
            self.declare_parameter(name, value)

        def param(name):
            return self.get_parameter(name).value

        self.model_path = str(param('model_path'))
        self.conf_thresh = float(param('conf_thresh'))
        self.imgsz = int(param('imgsz'))
        self.use_clip = bool(param('use_clip')) and HAVE_CLIP
        self.clip_thresh = float(param('clip_thresh'))
        self.detect_timeout = float(param('detect_timeout'))
        self.qr_timeout = float(param('qr_timeout'))

        self.nav_timeout = float(param('nav_timeout'))
        self.nav_retries = int(param('nav_retries'))
        self.nav_stall_timeout = float(param('nav_stall_timeout'))
        self.nav_stall_timeout_wall = float(param('nav_stall_timeout_wall'))
        self.nav_retry_backup_distance = float(param('nav_retry_backup_distance'))
        self.nav_retry_backup_speed = float(param('nav_retry_backup_speed'))
        self.nav_retry_spin_rad = float(param('nav_retry_spin_rad'))
        self.refine_approach = bool(param('refine_approach'))
        self.refine_timeout = float(param('refine_timeout'))

        self.center_timeout_wall = float(param('center_timeout_wall'))
        self.center_tolerance = float(param('center_tolerance'))
        self.servo_angular_gain = float(param('servo_angular_gain'))
        self.servo_max_angular = float(param('servo_max_angular'))
        self.servo_direction_sign = float(param('servo_direction_sign'))

        self.approach_speed = float(param('approach_speed'))
        self.lidar_approach_speed_limit = float(param('lidar_approach_speed_limit'))
        self.lidar_approach_stop = float(param('lidar_approach_stop'))
        self.lidar_sector_deg = float(param('lidar_sector_deg'))
        self.lidar_min_points = int(param('lidar_min_points'))
        self.lidar_data_timeout = float(param('lidar_data_timeout'))
        self.approach_timeout_wall = float(param('approach_timeout_wall'))
        self.approach_log_interval = float(param('approach_log_interval'))

        self.camera_x = float(param('camera_x'))
        self.lidar_x = float(param('lidar_x'))
        self.camera_target_distance = float(param('camera_target_distance'))

        self.enable_final_odom_approach = bool(param('enable_final_odom_approach'))
        self.final_odom_speed = float(param('final_odom_speed'))
        self.final_odom_timeout_wall = float(param('final_odom_timeout_wall'))
        self.final_odom_distance_limit = float(param('final_odom_distance_limit'))
        
        self.left_arm_open = deg2rad(float(param('left_arm_open_deg')))
        self.right_arm_open = deg2rad(float(param('right_arm_open_deg')))
        self.open_duration = float(param('open_duration'))
        self.upper_body_down = float(param('upper_body_down'))
        self.upper_body_up = float(param('upper_body_up'))
        self.left_arm_pick = deg2rad(float(param('left_arm_pick_deg')))
        self.right_arm_pick = deg2rad(float(param('right_arm_pick_deg')))
        self.lower_duration = float(param('lower_duration'))
        self.arm_duration = float(param('arm_duration'))
        self.require_qr = bool(param('require_qr'))

        # Calculate final distance from the geometry.
        self.final_odom_distance = (
            self.lidar_approach_stop
            - (self.camera_x - self.lidar_x)
            - self.camera_target_distance
        )
        self.final_odom_distance = min(
            self.final_odom_distance, self.final_odom_distance_limit
        )

        # ---------------------------------------------------------
        # Model initialization
        # ---------------------------------------------------------
        if not HAVE_YOLO:
            raise RuntimeError(
                'ultralytics is not installed. Install it before starting this node.'
            )

        self.bridge = CvBridge()
        self.latest_frame = None
        self.latest_frame_time = 0.0
        self.frame_lock = threading.Lock()

        self.det_model = YOLO(self.model_path)

        self.device = (
            'cuda' if HAVE_CLIP and torch.cuda.is_available() else 'cpu'
        )
        self.clip_model = None
        self.clip_preprocess = None
        self.clip_tokenizer = None
        self.clip_text_feat = None
        self.n_positive = len(POSITIVE_PROMPTS)

        if self.use_clip:
            self.get_logger().info('Loading CLIP verifier...')
            self.clip_model, _, self.clip_preprocess = (
                open_clip.create_model_and_transforms(
                    'ViT-B-32',
                    pretrained='laion2b_s34b_b79k'
                )
            )
            self.clip_model = self.clip_model.to(self.device).eval()
            self.clip_tokenizer = open_clip.get_tokenizer('ViT-B-32')

            labels = POSITIVE_PROMPTS + NEGATIVE_PROMPTS
            with torch.no_grad():
                features = self.clip_model.encode_text(
                    self.clip_tokenizer(labels).to(self.device)
                )
                features /= features.norm(dim=-1, keepdim=True)
            self.clip_text_feat = features

        # ---------------------------------------------------------
        # ROS subscriptions and publishers
        # ---------------------------------------------------------
        self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_cb,
            qos_profile_sensor_data
        )
        self.create_subscription(
            String,
            '/pick_box_request',
            self.request_cb,
            10
        )

        self.result_pub = self.create_publisher(
            String, '/pick_box_result', 10
        )
        self.upper_pub = self.create_publisher(
            Float64, '/upper_body/cmd_pos', 10
        )
        self.left_arm_pub = self.create_publisher(
            Float64, '/left_arm/cmd_pos', 10
        )
        self.right_arm_pub = self.create_publisher(
            Float64, '/right_arm/cmd_pos', 10
        )
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/cmd_vel', 10
        )

        # ---------------------------------------------------------
        # Sensor state
        # ---------------------------------------------------------
        self.scan_msg = None
        self.scan_received_time = 0.0
        self.odom_msg = None
        self.odom_received_time = 0.0
        self.sensor_lock = threading.Lock()

        # ---------------------------------------------------------
        # Nav2 and worker state
        # ---------------------------------------------------------
        self.navigator = BasicNavigator()
        self._nav_ready = False
        self.busy = False
        self._busy_lock = threading.Lock()

        if self.require_qr and not HAVE_PYZBAR:
            self.get_logger().error(
                'require_qr=True but pyzbar is not installed.'
            )
        if not self.require_qr:
            self.get_logger().warn('QR verification is disabled.')

        self.get_logger().info(
            'box_picker_node ready. Publish JSON to /pick_box_request.'
        )
        self.get_logger().info(
            f'LiDAR stop={self.lidar_approach_stop:.2f} m; '
            f'final odom segment={self.final_odom_distance:.3f} m; '
            f'final odom enabled={self.enable_final_odom_approach}'
        )

    # =============================================================
    # Sensor callbacks and safe snapshots
    # =============================================================
    def image_cb(self, msg):
        try:
            frame = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8'
            )
            with self.frame_lock:
                self.latest_frame = frame
                self.latest_frame_time = time.monotonic()
        except Exception as exc:
            self.get_logger().warning(
                f'image conversion failed: {exc}'
            )

    def scan_cb(self, msg):
        with self.sensor_lock:
            self.scan_msg = msg
            self.scan_received_time = time.monotonic()

    def odom_cb(self, msg):
        with self.sensor_lock:
            self.odom_msg = msg
            self.odom_received_time = time.monotonic()

    def get_latest_frame(self):
        with self.frame_lock:
            if self.latest_frame is None:
                return None
            return self.latest_frame.copy(), self.latest_frame_time

    def get_scan_snapshot(self):
        with self.sensor_lock:
            if self.scan_msg is None:
                return None, 0.0
            return self.scan_msg, self.scan_received_time

    def get_front_lidar_distance(self):
        """Median range in the front sector. Returns None for stale/invalid data."""
        scan, received = self.get_scan_snapshot()

        if scan is None:
            return None
        if time.monotonic() - received > self.lidar_data_timeout:
            return None

        sector = math.radians(self.lidar_sector_deg)
        values = []

        for index, distance in enumerate(scan.ranges):
            angle = scan.angle_min + index * scan.angle_increment

            if abs(angle) > sector:
                continue
            if not math.isfinite(distance):
                continue
            if distance < scan.range_min or distance > scan.range_max:
                continue

            values.append(float(distance))

        if len(values) < self.lidar_min_points:
            return None

        return float(np.median(values))

    def get_odom_pose(self):
        with self.sensor_lock:
            msg = self.odom_msg
            received = self.odom_received_time

        if msg is None:
            return None
        if time.monotonic() - received > self.lidar_data_timeout:
            return None

        p = msg.pose.pose.position
        q = msg.pose.pose.orientation

        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        return p.x, p.y, yaw

    @staticmethod
    def forward_displacement(initial_pose, current_pose):
        x0, y0, yaw0 = initial_pose
        x1, y1, _ = current_pose
        dx = x1 - x0
        dy = y1 - y0
        return dx * math.cos(yaw0) + dy * math.sin(yaw0)

    def stop_robot(self):
        self.cmd_vel_pub.publish(Twist())

    # =============================================================
    # Request handling
    # =============================================================
    def request_cb(self, msg):
        self.get_logger().info(f'Received request: {msg.data}')

        with self._busy_lock:
            if self.busy:
                self.get_logger().warning(
                    'A pick request is already running; ignoring this request.'
                )
                return
            self.busy = True

        try:
            request = json.loads(msg.data)
            x = float(request['x'])
            y = float(request['y'])
            yaw = float(request.get('yaw', 0.0))
            qr_code = str(request.get('qr_code', ''))
        except Exception as exc:
            self.get_logger().error(f'Invalid request payload: {exc}')
            self.publish_result(False, f'bad request payload: {exc}')
            with self._busy_lock:
                self.busy = False
            return

        worker = threading.Thread(
            target=self.execute_pick,
            args=(x, y, yaw, qr_code),
            daemon=True
        )
        worker.start()

    def publish_result(self, success, reason):
        message = String()
        message.data = json.dumps({
            'success': bool(success),
            'reason': str(reason)
        })
        self.result_pub.publish(message)

    # =============================================================
    # Main pick flow
    # =============================================================
    def execute_pick(self, x, y, yaw, qr_code):
        success = False
        reason = 'unknown error'

        try:
            self.get_logger().info(
                f'Pick request: x={x:.2f}, y={y:.2f}, yaw={yaw:.2f}, '
                f'QR="{qr_code}"'
            )

            if not self._nav_ready:
                self.navigator.waitUntilNav2Active()
                self._nav_ready = True

            if not self.navigate_to(x, y, yaw):
                reason = 'navigation failed or timed out'
                return

            self.get_logger().info(
                'Arrived at target pose. Searching for the box...'
            )
            bbox = self.detect_box(timeout=self.detect_timeout)
            if bbox is None:
                reason = 'no cardboard box detected'
                return

            if self.require_qr:
                if not HAVE_PYZBAR:
                    reason = 'QR requested but pyzbar is unavailable'
                    return

                qr_ok, matched = self.verify_qr(
                    bbox, qr_code, timeout=self.qr_timeout
                )
                if not qr_ok:
                    reason = f'QR mismatch or unreadable; expected "{qr_code}"'
                    return
                self.get_logger().info(f'QR confirmed: {matched}')

            if not self.center_and_approach():
                reason = 'failed to center on or approach the box'
                return

            self.get_logger().info(
                'Approach completed. Starting manipulator pick sequence.'
            )
            self.pick_sequence()

            success = True
            reason = (
                f'box picked; QR={qr_code}'
                if self.require_qr else
                'box pick sequence completed; QR check skipped'
            )

        except Exception as exc:
            self.get_logger().error(f'Pick process failed: {exc}')
            reason = f'exception: {exc}'

        finally:
            self.stop_robot()
            self.publish_result(success, reason)
            with self._busy_lock:
                self.busy = False
            self.get_logger().info(
                f'Pick request finished: success={success}, reason={reason}'
            )

    # =============================================================
    # Nav2
    # =============================================================
    def navigate_to(self, x, y, yaw):
        for attempt in range(1, self.nav_retries + 2):
            outcome = self.navigate_once(
                x, y, yaw, attempt, timeout=self.nav_timeout
            )

            if outcome == 'ok':
                if self.refine_approach:
                    self.refine_navigation(x, y, yaw)
                return True

            if outcome == 'timeout':
                return False

            if attempt <= self.nav_retries:
                self.get_logger().warning(
                    f'Navigation attempt {attempt} ended with {outcome}; '
                    'running recovery.'
                )
                self.recovery_maneuver()

        return False

    def navigate_once(self, x, y, yaw, attempt, timeout):
        goal = PoseStamped()
        goal.header.frame_id = 'map'
        goal.header.stamp = self.navigator.get_clock().now().to_msg()
        goal.pose.position.x = x
        goal.pose.position.y = y
        goal.pose.orientation.z = math.sin(yaw / 2.0)
        goal.pose.orientation.w = math.cos(yaw / 2.0)

        self.get_logger().info(
            f'[nav attempt {attempt}] goal=({x:.2f}, {y:.2f}, {yaw:.2f})'
        )
        self.navigator.goToPose(goal)

        wall_start = time.monotonic()
        sim_start = self.navigator.get_clock().now().nanoseconds * 1e-9
        best_distance = None
        best_progress_wall = wall_start
        best_progress_sim = sim_start
        last_log_wall = wall_start

        while rclpy.ok() and not self.navigator.isTaskComplete():
            now_wall = time.monotonic()
            now_sim = self.navigator.get_clock().now().nanoseconds * 1e-9
            feedback = self.navigator.getFeedback()

            if feedback is not None:
                distance = float(feedback.distance_remaining)

                if best_distance is None or distance < best_distance - 0.05:
                    best_distance = distance
                    best_progress_wall = now_wall
                    best_progress_sim = now_sim

                if now_wall - last_log_wall >= 5.0:
                    elapsed_sim = now_sim - sim_start
                    elapsed_wall = now_wall - wall_start
                    rtf = elapsed_sim / max(elapsed_wall, 1e-6)
                    self.get_logger().info(
                        f'[nav] sim={elapsed_sim:.1f}s / '
                        f'wall={elapsed_wall:.1f}s (RTF={rtf:.2f}), '
                        f'distance_remaining={distance:.2f}m'
                    )
                    last_log_wall = now_wall

                stalled_wall = (
                    now_wall - best_progress_wall > self.nav_stall_timeout_wall
                )
                stalled_sim = (
                    now_sim - best_progress_sim > self.nav_stall_timeout
                )

                if stalled_wall or stalled_sim:
                    self.get_logger().warning(
                        'Nav2 made no sufficient progress; cancelling goal.'
                    )
                    self.navigator.cancelTask()
                    return 'stalled'

            if now_sim - sim_start > timeout:
                self.get_logger().error('Nav2 timeout; cancelling goal.')
                self.navigator.cancelTask()
                return 'timeout'

            time.sleep(0.1)

        result = self.navigator.getResult()
        self.get_logger().info(f'Nav2 result: {result}')
        return 'ok' if result == TaskResult.SUCCEEDED else str(result)

    def refine_navigation(self, x, y, yaw):
        self.get_logger().info('Running short Nav2 refinement pass...')
        outcome = self.navigate_once(
            x, y, yaw, 'refine', timeout=self.refine_timeout
        )
        if outcome != 'ok':
            self.get_logger().warning(
                f'Refinement did not succeed ({outcome}); continuing.'
            )

    def recovery_maneuver(self):
        try:
            self.get_logger().info('Nav2 recovery: backing up...')
            self.navigator.backup(
                backup_dist=self.nav_retry_backup_distance,
                backup_speed=self.nav_retry_backup_speed,
                time_allowance=15
            )
            while rclpy.ok() and not self.navigator.isTaskComplete():
                time.sleep(0.1)
            self.get_logger().info(
                f'Backup result: {self.navigator.getResult()}'
            )
        except Exception as exc:
            self.get_logger().warning(f'Backup recovery failed: {exc}')

        try:
            self.get_logger().info('Nav2 recovery: spinning...')
            self.navigator.spin(
                spin_dist=self.nav_retry_spin_rad,
                time_allowance=20
            )
            while rclpy.ok() and not self.navigator.isTaskComplete():
                time.sleep(0.1)
            self.get_logger().info(
                f'Spin result: {self.navigator.getResult()}'
            )
        except Exception as exc:
            self.get_logger().warning(f'Spin recovery failed: {exc}')

    # =============================================================
    # YOLO detection and CLIP
    # =============================================================
    def detect_box(self, timeout=15.0):
        history = deque(maxlen=HISTORY_LEN)
        deadline = time.monotonic() + timeout
        best_box = None
        last_frame_time = 0.0
        frames_seen = 0
        last_log = time.monotonic()

        while time.monotonic() < deadline and rclpy.ok():
            snapshot = self.get_latest_frame()

            if snapshot is None:
                time.sleep(0.05)
                continue

            frame, frame_time = snapshot

            # Do not count the same camera frame multiple times.
            if frame_time <= last_frame_time:
                time.sleep(0.02)
                continue
            last_frame_time = frame_time
            frames_seen += 1

            result = self.det_model(
                frame,
                conf=self.conf_thresh,
                imgsz=self.imgsz,
                verbose=False
            )[0]

            found = False
            top_conf = 0.0

            if result.boxes is not None and len(result.boxes) > 0:
                boxes = result.boxes.xyxy.cpu().numpy()
                confs = result.boxes.conf.cpu().numpy()
                idx = int(np.argmax(confs))
                top_conf = float(confs[idx])

                candidate = boxes[idx]
                if top_conf >= self.conf_thresh:
                    if not self.use_clip or self.clip_accepts(frame, candidate):
                        best_box = candidate
                        found = True

            history.append(found)

            if time.monotonic() - last_log >= 2.0:
                self.get_logger().info(
                    f'detect_box: frames={frames_seen}, '
                    f'top_conf={top_conf:.2f}, '
                    f'confirmed={sum(history)}/{len(history)}'
                )
                last_log = time.monotonic()

            if sum(history) >= CONFIRM_FRAMES and best_box is not None:
                return best_box

        self.get_logger().error(
            f'YOLO detection timeout after {frames_seen} new frames.'
        )
        return None

    def clip_accepts(self, frame, box):
        crop = self.crop_with_pad(frame, box)
        if crop is None:
            return False

        try:
            image = self.clip_preprocess(
                PILImage.fromarray(
                    cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
                )
            ).unsqueeze(0).to(self.device)

            with torch.no_grad():
                features = self.clip_model.encode_image(image)
                features /= features.norm(dim=-1, keepdim=True)
                probabilities = (
                    100.0 * features @ self.clip_text_feat.T
                ).softmax(dim=-1)[0]

            positive_score = probabilities[:self.n_positive].sum().item()
            return positive_score >= self.clip_thresh

        except Exception as exc:
            self.get_logger().warning(
                f'CLIP verification failed; accepting YOLO result: {exc}'
            )
            return True

    @staticmethod
    def crop_with_pad(frame, box, pad=0.1):
        height, width = frame.shape[:2]
        x1, y1, x2, y2 = [float(v) for v in box]

        pad_x = int((x2 - x1) * pad)
        pad_y = int((y2 - y1) * pad)

        x1 = max(int(x1) - pad_x, 0)
        y1 = max(int(y1) - pad_y, 0)
        x2 = min(int(x2) + pad_x, width)
        y2 = min(int(y2) + pad_y, height)

        crop = frame[y1:y2, x1:x2]
        if crop.shape[0] < 24 or crop.shape[1] < 24:
            return None
        return crop

    # =============================================================
    # QR verification
    # =============================================================
    @staticmethod
    def read_qr_texts(frame):
        if frame is None or not HAVE_PYZBAR:
            return []
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        decoded = qr_decode(gray)
        return [
            item.data.decode('utf-8', errors='ignore')
            for item in decoded
        ]

    @staticmethod
    def qr_matches(texts, expected_code):
        expected = expected_code.strip().lower()
        for text in texts:
            if expected and expected in text.lower():
                return True, text
        return False, None

    def verify_qr(self, bbox, expected_code, timeout=5.0):
        deadline = time.monotonic() + timeout
        last_frame_time = 0.0

        while time.monotonic() < deadline and rclpy.ok():
            snapshot = self.get_latest_frame()
            if snapshot is None:
                time.sleep(0.05)
                continue

            frame, frame_time = snapshot
            if frame_time <= last_frame_time:
                time.sleep(0.02)
                continue
            last_frame_time = frame_time

            crop = self.crop_with_pad(frame, bbox, pad=0.3)
            texts = self.read_qr_texts(crop)
            if not texts:
                texts = self.read_qr_texts(frame)

            matched, value = self.qr_matches(texts, expected_code)
            if matched:
                return True, value

            if texts:
                self.get_logger().warning(
                    f'QR detected but does not match: {texts}'
                )
            time.sleep(0.05)

        return False, None

    # =============================================================
    # Visual centering + LiDAR + odometry approach
    # =============================================================
    def center_box(self):
        """Rotate in place until the YOLO box center is within tolerance."""
        self.get_logger().info('Phase 1: centering box in RGB image...')

        deadline = time.monotonic() + self.center_timeout_wall
        centered_frames = 0
        last_frame_time = 0.0

        while rclpy.ok() and time.monotonic() < deadline:
            snapshot = self.get_latest_frame()

            if snapshot is None:
                self.stop_robot()
                time.sleep(0.05)
                continue

            frame, frame_time = snapshot
            if frame_time <= last_frame_time:
                time.sleep(0.02)
                continue
            last_frame_time = frame_time

            result = self.det_model(
                frame,
                conf=self.conf_thresh,
                imgsz=self.imgsz,
                verbose=False
            )[0]

            if result.boxes is None or len(result.boxes) == 0:
                self.stop_robot()
                self.get_logger().error(
                    'YOLO lost the box during centering.'
                )
                return False

            boxes = result.boxes.xyxy.cpu().numpy()
            confs = result.boxes.conf.cpu().numpy()
            idx = int(np.argmax(confs))

            x1, _, x2, _ = boxes[idx]
            box_center_x = (x1 + x2) / 2.0
            frame_center_x = frame.shape[1] / 2.0
            offset = (box_center_x - frame_center_x) / frame_center_x

            if abs(offset) <= self.center_tolerance:
                self.stop_robot()
                centered_frames += 1
                if centered_frames >= CONFIRM_FRAMES:
                    self.get_logger().info('Box centered successfully.')
                    return True
            else:
                centered_frames = 0
                angular = (
                    -self.servo_direction_sign
                    * self.servo_angular_gain
                    * offset
                )
                angular = clamp(
                    angular,
                    -self.servo_max_angular,
                    self.servo_max_angular
                )

                twist = Twist()
                twist.angular.z = angular
                self.cmd_vel_pub.publish(twist)

            time.sleep(0.05)

        self.stop_robot()
        self.get_logger().error('Centering timeout.')
        return False

    def lidar_approach(self):
        """Move forward until the front-sector LiDAR median reaches threshold."""
        self.get_logger().info(
            f'Phase 2: LiDAR approach; stop at '
            f'{self.lidar_approach_stop:.2f} m.'
        )

        deadline = time.monotonic() + self.approach_timeout_wall
        last_log = 0.0
        last_distance = None

        while rclpy.ok() and time.monotonic() < deadline:
            distance = self.get_front_lidar_distance()

            if distance is None:
                self.stop_robot()
                self.get_logger().error(
                    'No valid/stale front LiDAR data. Robot stopped.'
                )
                return False

            # Reject sudden jumps that may indicate the scan is looking at
            # a different object. This is a basic sanity check, not tracking.
            if last_distance is not None and abs(distance - last_distance) > 0.35:
                self.stop_robot()
                self.get_logger().error(
                    f'LiDAR range jumped unexpectedly '
                    f'({last_distance:.2f} -> {distance:.2f} m).'
                )
                return False

            last_distance = distance

            if distance <= self.lidar_approach_stop:
                self.stop_robot()
                self.get_logger().info(
                    f'LiDAR approach reached threshold: {distance:.3f} m.'
                )
                return True

            twist = Twist()
            twist.linear.x = min(
                self.approach_speed,
                self.lidar_approach_speed_limit
            )
            self.cmd_vel_pub.publish(twist)

            if time.monotonic() - last_log >= self.approach_log_interval:
                self.get_logger().info(
                    f'LiDAR distance: {distance:.3f} m'
                )
                last_log = time.monotonic()

            time.sleep(0.05)

        self.stop_robot()
        self.get_logger().error(
            f'LiDAR approach timeout after '
            f'{self.approach_timeout_wall:.1f} wall seconds.'
        )
        return False

    def final_odom_approach(self):
        """
        Final short movement based on odometry.

        IMPORTANT: LiDAR range_min is 0.30 m. This segment may enter the
        LiDAR blind zone, so odometry is not an obstacle sensor.
        """
        if not self.enable_final_odom_approach:
            self.get_logger().warning(
                'Final odometry approach disabled. Stopping at LiDAR threshold.'
            )
            return False

        distance_goal = self.final_odom_distance

        if distance_goal <= 0.0:
            self.get_logger().error(
                f'Invalid final odometry distance: {distance_goal:.3f} m.'
            )
            return False

        if distance_goal > self.final_odom_distance_limit + 1e-6:
            self.get_logger().error(
                'Final odometry distance exceeds configured safety limit.'
            )
            return False

        initial_pose = self.get_odom_pose()
        if initial_pose is None:
            self.get_logger().error(
                'No fresh /odom message before final approach.'
            )
            return False

        self.get_logger().warning(
            f'Phase 3: odometry-only final movement '
            f'{distance_goal:.3f} m at {self.final_odom_speed:.3f} m/s. '
            'LiDAR may be below its 0.30 m minimum range.'
        )

        deadline = time.monotonic() + self.final_odom_timeout_wall

        while rclpy.ok() and time.monotonic() < deadline:
            current_pose = self.get_odom_pose()

            if current_pose is None:
                self.stop_robot()
                self.get_logger().error(
                    'Odometry became stale during final movement.'
                )
                return False

            travelled = self.forward_displacement(
                initial_pose, current_pose
            )

            if travelled < -0.02:
                self.stop_robot()
                self.get_logger().error(
                    'Odometry indicates unexpected backward movement.'
                )
                return False

            if travelled >= distance_goal:
                self.stop_robot()
                self.get_logger().info(
                    f'Final odometry movement completed: {travelled:.3f} m.'
                )
                return True

            twist = Twist()
            twist.linear.x = min(self.final_odom_speed, 0.03)
            self.cmd_vel_pub.publish(twist)
            time.sleep(0.05)

        self.stop_robot()
        self.get_logger().error(
            'Final odometry movement timed out; robot stopped.'
        )
        return False

    def center_and_approach(self):
        self.stop_robot()

        if not self.center_box():
            self.stop_robot()
            return False

        if not self.lidar_approach():
            self.stop_robot()
            return False

        if not self.final_odom_approach():
            self.stop_robot()
            return False

        self.stop_robot()
        self.get_logger().info(
            'Centering and approach phases completed.'
        )
        return True

    # =============================================================
    # Manipulator sequence
    # =============================================================
    def pick_sequence(self):
        self.get_logger().info('Opening arms before lowering...')
        self.left_arm_pub.publish(Float64(data=self.left_arm_open))
        self.right_arm_pub.publish(Float64(data=self.right_arm_open))
        time.sleep(self.open_duration)

        self.get_logger().info('Lowering upper body...')
        self.upper_pub.publish(
            Float64(data=self.upper_body_down)
        )
        time.sleep(self.lower_duration)

        self.get_logger().info('Moving arms to grip...')
        self.left_arm_pub.publish(
            Float64(data=self.left_arm_pick)
        )
        self.right_arm_pub.publish(
            Float64(data=self.right_arm_pick)
        )
        time.sleep(self.arm_duration)

        self.get_logger().info('Lifting upper body...')
        self.upper_pub.publish(
            Float64(data=self.upper_body_up)
        )
        time.sleep(self.lower_duration)

        self.get_logger().info('Manipulator sequence completed.')


def main(args=None):
    rclpy.init(args=args)
    node = BoxPickerNode()

    executor = rclpy.executors.SingleThreadedExecutor()
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Keyboard interrupt; stopping robot.')
    finally:
        node.stop_robot()
        executor.remove_node(node)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
