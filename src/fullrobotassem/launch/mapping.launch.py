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
    
    # Đọc file URDF
    with open(urdf_file, 'r') as f:
        robot_desc = f.read()

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

    # BỔ SUNG: Joint State Publisher để xuất trạng thái các khớp bánh xe (Khắc phục lỗi No transform)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{'use_sim_time': use_sim_time}]
    )

    # 3. Spawn Robot vào môi trường Gazebo
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

    # 4. Cầu nối chủ đề (ROS <-> Gazebo Bridge)
    bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=[
            '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
            '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
            '/odom@nav_msgs/msg/Odometry[gz.msgs.Odometry',
            '/tf@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
            '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'
        ],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    # 5. Thuật toán quét lập bản đồ SLAM Toolbox
    slam_toolbox = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'solver_plugin': 'solver_plugins::CeresSolver',
            'ceres_linear_solver': 'SPARSE_NORMAL_CHOLESKY',
            'ceres_preconditioner': 'SCHUR_JACOBI',
            'ceres_trust_strategy': 'LEVENBERG_MARQUARDT',
            'base_frame': 'base_footprint',
            'odom_frame': 'odom',
            'map_frame': 'map',
            'scan_topic': '/scan',
            'mode': 'mapping',
            'transform_timeout': 0.2,
            'tf_buffer_duration': 30.0,
            'max_laser_range': 30.0,
            'minimum_time_interval': 0.1,
            'resolution': 0.05
        }]
    )

    # 6. Mở RViz2
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen'
    )

    return LaunchDescription([
        set_env,
        DeclareLaunchArgument('use_sim_time', default_value='true', description='Dùng thời gian mô phỏng'),
        gazebo,
        robot_state_publisher,
        joint_state_publisher,  # Đưa node này vào danh sách chạy
        spawn_robot,
        bridge,
        slam_toolbox,
        rviz
    ])