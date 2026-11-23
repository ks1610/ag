import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    # --------------------------------------------------
    # Package paths
    # --------------------------------------------------

    pkg_share = get_package_share_directory('fullrobotassem')
    nav2_share = get_package_share_directory('nav2_bringup')

    urdf_file = os.path.join(
        pkg_share,
        'urdf',
        'fullrobotassem.urdf'
    )

    default_map = os.path.expanduser(
        '~/ag/maps/my_map.yaml'
    )

    default_params = os.path.join(
        pkg_share,
        'config',
        'nav2_real_params.yaml'
    )

    default_rviz_config = os.path.join(
        nav2_share,
        'rviz',
        'nav2_default_view.rviz'
    )

    # --------------------------------------------------
    # Launch arguments
    # --------------------------------------------------

    map_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('use_rviz')

    declare_map = DeclareLaunchArgument(
        'map',
        default_value=default_map,
        description='Path to the saved map YAML file'
    )

    declare_params = DeclareLaunchArgument(
        'params_file',
        default_value=default_params,
        description='Path to Nav2 parameters'
    )

    declare_sim_time = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    declare_autostart = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Automatically activate Nav2 nodes'
    )

    declare_rviz = DeclareLaunchArgument(
        'use_rviz',
        default_value='true',
        description='Start RViz'
    )

    # --------------------------------------------------
    # Robot description and TF
    # --------------------------------------------------

    with open(urdf_file, 'r') as file:
        robot_description = file.read()

    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {
                'robot_description': robot_description,
                'use_sim_time': use_sim_time
            }
        ]
    )

    # --------------------------------------------------
    # Nav2 bringup
    # --------------------------------------------------

    nav2_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                nav2_share,
                'launch',
                'bringup_launch.py'
            )
        ),
        launch_arguments={
            'map': map_file,
            'params_file': params_file,
            'use_sim_time': use_sim_time,
            'autostart': autostart,
        }.items()
    )

    # --------------------------------------------------
    # RViz
    # --------------------------------------------------

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=[
            '-d',
            default_rviz_config
        ],
        parameters=[
            {
                'use_sim_time': use_sim_time
            }
        ],
        condition=IfCondition(use_rviz),
        output='screen'
    )

    # --------------------------------------------------
    # Launch description
    # --------------------------------------------------

    return LaunchDescription([

        declare_map,
        declare_params,
        declare_sim_time,
        declare_autostart,
        declare_rviz,

        robot_state_publisher,
        nav2_bringup,
        rviz,

    ])
