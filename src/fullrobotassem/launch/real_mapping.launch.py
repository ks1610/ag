import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    pkg = get_package_share_directory('fullrobotassem')

    urdf_file = os.path.join(pkg, 'urdf', 'fullrobotassem.urdf')
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

    # Robot model and TF from the URDF (base_footprint -> base_link -> lidar_link)
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_desc, 'use_sim_time': False}],
    )

    # SLAM: builds map -> odom using /scan and /odom
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'),
                         'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'false',
            'slam_params_file': os.path.join(
                pkg, 'config', 'mapper_params_online_async.yaml'),
        }.items(),
    )

    # RViz with the saved config if it exists, otherwise an empty window
    rviz_cfg = os.path.expanduser('~/robot_model.rviz')
    rviz_args = ['-d', rviz_cfg] if os.path.exists(rviz_cfg) else []
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=rviz_args,
        output='screen',
    )

    return LaunchDescription([robot_state_publisher, slam_toolbox, rviz])
