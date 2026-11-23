import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

# ==================================================================
# Odometry calibration
#   TICKS_PER_REV    : encoder counts per wheel revolution
#   WHEEL_RADIUS     : wheel radius in meters
#   WHEEL_SEPARATION : track width in meters (matches URDF)
# ==================================================================
TICKS_PER_REV = 1342.5
WHEEL_RADIUS = 0.03
WHEEL_SEPARATION = 0.243

# Drive settings that gave a straight forward run
SPEED_SCALE = 0.3
LEFT_SLOWER_PERCENT = 28.0


def generate_launch_description():
    if TICKS_PER_REV <= 0.0 or WHEEL_RADIUS <= 0.0 or WHEEL_SEPARATION <= 0.0:
        raise RuntimeError('Calibration values must be greater than zero.')

    pkg = get_package_share_directory('fullrobotassem')

    urdf_file = os.path.join(pkg, 'urdf', 'fullrobotassem.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # 1. Lidar driver (publishes /scan in frame lidar_link)
    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ldlidar_sl_ros2'),
                         'launch', 'ld14.launch.py')
        ),
    )

    # 2. Hardware bridge: /cmd_vel -> Arduino 1, publishes /odom and TF
    bridge = Node(
        package='robot_hardware',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'invert_right': True,
            'invert_turn': True,
            'swap_sides': True,
            'speed_scale': SPEED_SCALE,
            'left_slower_percent': LEFT_SLOWER_PERCENT,
            'ticks_per_rev': TICKS_PER_REV,
            'wheel_radius': WHEEL_RADIUS,
            'wheel_separation': WHEEL_SEPARATION,
        }],
    )

    # 3. Joint states at zero, so wheels, arms and camera get transforms
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc}],
    )

    # 4. Robot model and TF from the URDF
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
    )

    # 5. SLAM: builds map -> odom from /scan and /odom
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(
                pkg, 'config', 'mapper_params_online_async.yaml'),
        }.items(),
    )

    # 6. RViz, using the saved config if it exists
    rviz_cfg = os.path.expanduser('~/robot_model.rviz')
    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen',
    )

    return LaunchDescription([
        lidar,
        bridge,
        joint_state_publisher,
        robot_state_publisher,
        slam_toolbox,
        rviz,
    ])
