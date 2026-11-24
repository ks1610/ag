import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


# ============================================================
# Hardware calibration - keep values from mapping_all.launch.py
# ============================================================

TICKS_PER_REV = 1342.5
WHEEL_RADIUS = 0.03
WHEEL_SEPARATION = 0.243

# 1.0 so that Nav2's velocity limits are the real limits.
SPEED_SCALE = 1.0
LEFT_SLOWER_PERCENT = 28.0

# Deadband compensation (m/s and rad/s, after scaling).
# PLACEHOLDERS: measure the lowest speeds that reliably move the robot.
MIN_LINEAR = 0.05
MIN_ANGULAR = 0.25


def generate_launch_description():

    robot_pkg = get_package_share_directory('fullrobotassem')
    lidar_pkg = get_package_share_directory('ldlidar_sl_ros2')
    nav2_pkg = get_package_share_directory('nav2_bringup')

    urdf_file = os.path.join(
        robot_pkg, 'urdf', 'fullrobotassem.urdf'
    )

    nav2_params_file = os.path.join(
        robot_pkg, 'config', 'nav2_real_params.yaml'
    )

    default_map_file = os.path.expanduser(
        '~/ag/maps/my_map.yaml'
    )

    default_rviz_file = os.path.join(
        nav2_pkg, 'rviz', 'nav2_default_view.rviz'
    )

    # --------------------------------------------------------
    # Launch arguments
    # --------------------------------------------------------

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')
    auto_global_localize = LaunchConfiguration('auto_global_localize')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map_file,
        description='Saved map YAML file'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=nav2_params_file,
        description='Nav2 parameters YAML file'
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate Nav2 lifecycle nodes'
    )

    declare_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )

    declare_global_localize = DeclareLaunchArgument(
        'auto_global_localize',
        default_value='true',
        description='Auto global localization and spin-to-converge at startup'
    )

    # --------------------------------------------------------
    # Load robot description
    # --------------------------------------------------------

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    # --------------------------------------------------------
    # 1. LiDAR driver
    # --------------------------------------------------------

    lidar = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                lidar_pkg,
                'launch',
                'ld14.launch.py'
            )
        )
    )

    # --------------------------------------------------------
    # 2. Hardware bridge
    # --------------------------------------------------------

    bridge = Node(
        package='robot_hardware',
        executable='serial_bridge',
        name='serial_bridge',
        output='screen',
        parameters=[{
            'invert_right': True,
            'invert_turn': True,

            'speed_scale': SPEED_SCALE,
            'left_slower_percent': LEFT_SLOWER_PERCENT,

            'max_linear': 0.2,
            'max_angular': 1.0,
            'min_linear': MIN_LINEAR,
            'min_angular': MIN_ANGULAR,

            'ticks_per_rev': TICKS_PER_REV,
            'wheel_radius': WHEEL_RADIUS,
            'wheel_separation': WHEEL_SEPARATION,
        }]
    )

    # --------------------------------------------------------
    # 3. Joint states
    # --------------------------------------------------------

    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description
        }]
    )

    # --------------------------------------------------------
    # 4. Robot description and URDF transforms
    # --------------------------------------------------------

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': use_sim_time,
        }]
    )

    # --------------------------------------------------------
    # 5. Nav2 bringup
    # --------------------------------------------------------

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_pkg,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,

            'slam': 'False',
            'use_localization': 'True',

            'use_composition': 'False',
            'use_respawn': 'False',
        }.items()
    )

    delayed_nav2 = TimerAction(
        period=3.0,
        actions=[nav2_bringup]
    )

    # --------------------------------------------------------
    # 6. Auto global localization + spin to converge
    # --------------------------------------------------------

    global_localize_cmd = TimerAction(
        period=8.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'service', 'call',
                    '/reinitialize_global_localization',
                    'std_srvs/srv/Empty', '{}'
                ],
                output='screen',
                condition=IfCondition(auto_global_localize)
            )
        ]
    )

    # Spin 0.6 rad/s for 6 s. With SPEED_SCALE = 1.0 this is now a REAL
    # 0.6 rad/s (before it was about 0.18), so keep clear space around the robot.
    auto_converge_spin = TimerAction(
        period=9.5,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'topic', 'pub', '--times', '30', '-r', '5',
                    '/cmd_vel_smoothed', 'geometry_msgs/msg/Twist',
                    '{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.6}}'
                ],
                output='screen',
                condition=IfCondition(auto_global_localize)
            )
        ]
    )

    # --------------------------------------------------------
    # 7. RViz
    # --------------------------------------------------------

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_file],
        parameters=[{
            'use_sim_time': use_sim_time
        }],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    return LaunchDescription([
        declare_map,
        declare_params,
        declare_sim_time,
        declare_autostart,
        declare_rviz,
        declare_global_localize,

        lidar,
        bridge,
        joint_state_publisher,
        robot_state_publisher,

        delayed_nav2,
        global_localize_cmd,
        auto_converge_spin,
        rviz,
    ])