import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    package_name = 'fullrobotassem'
    pkg_share = get_package_share_directory(package_name)
    nav2_bringup_share = get_package_share_directory('nav2_bringup')

    # Thư mục cha chứa package (install/fullrobotassem/share) để Gazebo load mesh
    pkg_parent_share = os.path.dirname(pkg_share)

    # Thêm biến môi trường cho Gazebo Sim
    set_gz_resource_path = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=pkg_parent_share
    )

    default_urdf_path = os.path.join(pkg_share, 'urdf', 'fullrobotassem.urdf')
    default_world_path = os.path.join(pkg_share, 'worlds', 'maze2.sdf')
    default_map_path = os.path.join(pkg_share, 'maps', 'maze2_map.yaml')
    default_params_path = os.path.join(pkg_share, 'config', 'nav2_params.yaml')
    default_bridge_config = os.path.join(pkg_share, 'config', 'bridge.yaml')
    default_rviz_config_path = os.path.join(nav2_bringup_share, 'rviz', 'nav2_default_view.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')
    map_yaml_file = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')

    declare_use_sim_time_cmd = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_map_yaml_cmd = DeclareLaunchArgument('map', default_value=default_map_path)
    declare_params_file_cmd = DeclareLaunchArgument('params_file', default_value=default_params_path)

    robot_state_publisher_cmd = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': open(default_urdf_path).read()
        }]
    )

    gazebo_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {default_world_path}'}.items()
    )

    spawn_robot_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-name', 'fullrobotassem', '-file', default_urdf_path, '-z', '0.1'],
        output='screen'
    )

    bridge_cmd = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': default_bridge_config,
            'use_sim_time': use_sim_time
        }],
        output='screen'
    )

    nav2_bringup_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_share, 'launch', 'bringup_launch.py')
        ),
        launch_arguments={
            'map': map_yaml_file,
            'use_sim_time': use_sim_time,
            'params_file': params_file
        }.items()
    )

    rviz_cmd = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz_config_path],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    ld = LaunchDescription()
    ld.add_action(set_gz_resource_path)
    ld.add_action(declare_use_sim_time_cmd)
    ld.add_action(declare_map_yaml_cmd)
    ld.add_action(declare_params_file_cmd)

    ld.add_action(robot_state_publisher_cmd)
    ld.add_action(gazebo_cmd)
    ld.add_action(spawn_robot_cmd)
    ld.add_action(bridge_cmd)
    ld.add_action(nav2_bringup_cmd)
    ld.add_action(rviz_cmd)

    return ld