# ros2 launch fullrobotassem mapping.launch.py

# Bluetooth app:

# cd ~/bluetooth_robot_kit
# ./2_start_bluetooth.sh

# Default controler:
# ros2 run teleop_twist_keyboard teleop_twist_keyboard

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    pkg_fullrobotassem = get_package_share_directory('fullrobotassem')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    resource_path = os.path.join(get_package_share_directory('fullrobotassem'), '..')
    set_env = SetEnvironmentVariable(name='GZ_SIM_RESOURCE_PATH', value=resource_path)

    use_sim_time = LaunchConfiguration('use_sim_time', default='true')

    world_file = os.path.join(pkg_fullrobotassem, 'worlds', 'maze2.sdf')
    urdf_file = os.path.join(pkg_fullrobotassem, 'urdf', 'fullrobotassem.urdf')
    box_urdf_file = os.path.join(pkg_fullrobotassem, 'urdf', 'cardboard_box.urdf')

    # Đọc file URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()
    
    # Đọc file URDF thùng carton
    with open(box_urdf_file, 'r') as f:
        box_desc = f.read()
    
    # 1. Mở Gazebo Sim với maze2.sdf
    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')
        ),
        launch_arguments={'gz_args': f'-r {world_file}'}.items(),
    )

    # 2. Publish TF & Robot Description
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'robot_description': robot_desc
        }]
    )

    # 3. Spawn Robot, Cardboard Box vào môi trường Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-string', robot_desc,
            '-name', 'fullrobotassem',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.2'
        ],
        output='screen'
    )

    box_sdf_file = os.path.join(pkg_fullrobotassem, 'urdf', 'cardboard_box.sdf')

    spawn_box = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', box_sdf_file,
            '-name', 'cardboard_box',
            '-x', '0.0',
            '-y', '0.5',
            '-z', '0.15'
        ],
        output='screen'
    )

    # 4. Cầu nối chủ đề (ROS <-> Gazebo Bridge)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
            '/camera/image_raw@sensor_msgs/msg/Image[gz.msgs.Image',
            '/camera/camera_info@sensor_msgs/msg/CameraInfo[gz.msgs.CameraInfo',
            '/joint_states@sensor_msgs/msg/JointState[gz.msgs.Model',
            '/upper_body/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/left_shoulder/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/right_shoulder/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/left_arm/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double',
            '/right_arm/cmd_pos@std_msgs/msg/Float64]gz.msgs.Double'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 5. Thuật toán quét lập bản đồ SLAM Toolbox (Sử dụng cấu hình chuẩn của ROS 2)
    slam_toolbox = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(get_package_share_directory('slam_toolbox'), 'launch', 'online_async_launch.py')),
        launch_arguments={
            'use_sim_time': 'true',
            'slam_params_file': os.path.join(pkg_fullrobotassem, 'config', 'mapper_params_online_async.yaml')
        }.items()
    )

    # 6. Mở RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    #7 . Bluetooth Teleop Node
    bluetooth_teleop = Node(
    package='fullrobotassem',
    executable='bluetooth_teleop.py',
    name='bluetooth_teleop',
    output='screen',
    parameters=[{
        'port': '/dev/rfcomm0',
        'baud': 9600,
        'linear_speed': 0.20,
        'spin_speed': 0.8,
        'curve_angular': 0.5,
        'cmd_timeout': 0.0,
    }]
    )

    # 8. Upper body + arms control panel
    upper_body_panel = Node(
        package='fullrobotassem',
        executable='upper_body_panel.py',
        name='upper_body_panel',
        output='screen',
        parameters=[{
            'min_pos': -0.30,
            'max_pos': 0.065,
            'step': 0.01,
            'speed': 0.05,
            'shoulder_min': -3.14,
            'shoulder_max': 1.57,
            'shoulder_step': 0.1,
            'shoulder_speed': 0.8,
            'arm_min': -1.57,
            'arm_max': 1.57,
            'arm_step': 0.1,
            'arm_speed': 0.8,
        }]
    )

    return LaunchDescription([
        set_env,
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Dùng thời gian mô phỏng'),
        gazebo,
        robot_state_publisher,
        spawn_robot,
        spawn_box,
        bridge,
        slam_toolbox,
        rviz,
        bluetooth_teleop,
        upper_body_panel
    ])
    