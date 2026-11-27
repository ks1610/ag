import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg = 'fullrobotassem'
    pkg_share = get_package_share_directory(pkg)

    args = [
        DeclareLaunchArgument('place_x', default_value='2.875'),
        DeclareLaunchArgument('place_y', default_value='0.530'),
        DeclareLaunchArgument('place_yaw', default_value='2.795'),
        DeclareLaunchArgument('forward_revs', default_value='5.0'),
        DeclareLaunchArgument('drive_speed', default_value='0.08'),
        DeclareLaunchArgument('pick_x', default_value='2.875'),
        DeclareLaunchArgument('pick_y', default_value='0.530'),
        DeclareLaunchArgument('pick_yaw', default_value='2.795'),
        DeclareLaunchArgument('qr_code', default_value='mmb-6769'),
        DeclareLaunchArgument('camera_device', default_value='/dev/video0'),
        DeclareLaunchArgument('model_path',
                              default_value=os.path.expanduser('~/ag/model/best.pt')),
    ]

    # Nav2 + LiDAR + bridge + URDF (your existing working launch)
    navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'navigation_real.launch.py')
        )
    )

    # Webcam -> /camera/image_raw
    camera = Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name='webcam',
        output='screen',
        parameters=[{
            'video_device': LaunchConfiguration('camera_device'),
            'image_size': [640, 480],
        }],
        remappings=[('image_raw', '/camera/image_raw')],
    )

    # YOLO box detection + centering + pick (real-robot settings)
    box_picker = Node(
        package=pkg,
        executable='box_picker_node.py',
        name='box_picker_node',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'model_path': LaunchConfiguration('model_path'),
            'conf_thresh': 0.4,
            'imgsz': 320,
            'use_clip': False,
            'detect_timeout': 15.0,
            'qr_timeout': 5.0,
            'nav_timeout': 120.0,
            'upper_body_down': -0.30,
            'upper_body_up': 0.065,
            'left_arm_pick_deg': -40.0,
            'right_arm_pick_deg': 40.0,
            'lower_duration': 4.0,
            'arm_duration': 2.0,
        }],
    )

    # Mission: go to place 1 -> drive forward N revs -> send pick request
    mission = Node(
        package=pkg,
        executable='mission_node.py',
        name='mission_node',
        output='screen',
        parameters=[{
            'place_x': ParameterValue(LaunchConfiguration('place_x'), value_type=float),
            'place_y': ParameterValue(LaunchConfiguration('place_y'), value_type=float),
            'place_yaw': ParameterValue(LaunchConfiguration('place_yaw'), value_type=float),
            'forward_revs': ParameterValue(LaunchConfiguration('forward_revs'), value_type=float),
            'drive_speed': ParameterValue(LaunchConfiguration('drive_speed'), value_type=float),
            'pick_x': ParameterValue(LaunchConfiguration('pick_x'), value_type=float),
            'pick_y': ParameterValue(LaunchConfiguration('pick_y'), value_type=float),
            'pick_yaw': ParameterValue(LaunchConfiguration('pick_yaw'), value_type=float),
            'qr_code': LaunchConfiguration('qr_code'),
            'wheel_radius': 0.03,
        }],
    )

    ld = LaunchDescription()
    for a in args:
        ld.add_action(a)
    ld.add_action(navigation)
    ld.add_action(camera)
    ld.add_action(box_picker)
    ld.add_action(mission)
    return ld