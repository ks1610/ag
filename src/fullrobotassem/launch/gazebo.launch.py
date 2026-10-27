# killall -9 gz server gz client ruby python3 ros2 ros2_run robot_state_publisher parameter_bridge rviz2 2>/dev/null
# cd ~/trinh/ag
# rm -rf build/ install/ log/
# colcon build --symlink-install
# source install/setup.bash
# ros2 launch fullrobotassem gazebo.launch.py

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node

def generate_launch_description():
    pkg_path = get_package_share_directory('fullrobotassem')
    
    # Trỏ đường dẫn để Gazebo Sim đọc được file mesh STL trong package ROS
    set_gz_env = SetEnvironmentVariable(
        name='GZ_SIM_RESOURCE_PATH',
        value=os.path.join(get_package_share_directory('fullrobotassem'), '..')
    )

    urdf_file_path = os.path.join(pkg_path, 'urdf', 'fullrobotassem.urdf')

    with open(urdf_file_path, 'r') as infp:
        robot_description_content = infp.read()

    robot_description = {'robot_description': robot_description_content}

    # 1. Khởi động robot_state_publisher
    node_robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[robot_description]
    )

    # Lấy đường dẫn tới file World mới tạo
    world_file_path = os.path.join(pkg_path, 'worlds', 'maze2.sdf')

    # 2. Khởi động Gazebo Sim với World tùy chỉnh
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file_path}'}.items(), # Đã truyền file SDF và -r để tự động Play
    )

    # 3. Spawn robot vào Gazebo Sim
    spawn_entity = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=['-topic', 'robot_description', '-entity', 'fullrobotassem', '-z', '0.2'],
        output='screen'
    )
    
    # 4. Bridge chuyển đổi dữ liệu /scan từ Gazebo (gz) sang ROS 2
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan'
        ],
        output='screen'
    )

    # 5. Khởi động RViz2 tự động
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen'
    )

    return LaunchDescription([
        set_gz_env,
        gazebo,
        node_robot_state_publisher,
        spawn_entity,
        bridge,
        rviz_node,
    ])