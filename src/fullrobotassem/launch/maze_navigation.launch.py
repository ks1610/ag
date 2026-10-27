# ros2 run teleop_twist_keyboard teleop_twist_keyboard
# ros2 launch fullrobotassem maze_navigation.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    pkg_share = get_package_share_directory('fullrobotassem')
    
    urdf_file = os.path.join(pkg_share, 'urdf', 'fullrobotassem.urdf')
    world_file = os.path.join(pkg_share, 'worlds', 'maze2.sdf')

    # Xử lý nội dung URDF bằng xacro/command
    robot_description = Command(['xacro ', urdf_file])

    install_share_path = os.path.dirname(pkg_share)
    os.environ['GZ_SIM_RESOURCE_PATH'] = f"{install_share_path}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}"

    # 1. Robot State Publisher
    robot_state_publisher_node = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{
            'robot_description': ParameterValue(robot_description, value_type=str),
            'use_sim_time': True
        }]
    )

    # THÊM MỚI: Joint State Publisher để xuất trạng thái các khớp bánh xe
    joint_state_publisher_node = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': True}]
    )

    # 2. Khởi động Gazebo Sim với world maze2
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('ros_gz_sim'), 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items()
    )

    # 3. Spawn (đưa) robot từ ROS 2 vào Gazebo Sim
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        output='screen',
        arguments=['-topic', 'robot_description',
                   '-name', 'fullrobotassem',
                   '-z', '0.5']
    )

    # 4. Bridge kết nối các topic giữa ROS 2 và Gazebo
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        parameters=[{
            'config_file': os.path.join(pkg_share, 'config', 'bridge.yaml'),
            'use_sim_time': True
        }],
        output='screen'
    )

    return LaunchDescription([
        robot_state_publisher_node,
        joint_state_publisher_node, # Đưa joint_state_publisher vào đây
        gazebo_launch,
        spawn_robot,
        bridge
    ])