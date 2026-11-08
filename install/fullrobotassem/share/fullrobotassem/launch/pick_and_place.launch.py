import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    package_name = 'fullrobotassem'
    pkg_share = get_package_share_directory(package_name)

    declare_box_x_cmd = DeclareLaunchArgument('box_x', default_value='1.8')
    declare_box_y_cmd = DeclareLaunchArgument('box_y', default_value='0.6')
    declare_box_z_cmd = DeclareLaunchArgument('box_z', default_value='0.2')

    declare_pick_x_cmd = DeclareLaunchArgument('pick_x', default_value='3.228')
    declare_pick_y_cmd = DeclareLaunchArgument('pick_y', default_value='0.620')
    declare_pick_yaw_cmd = DeclareLaunchArgument('pick_yaw', default_value='3.131')

    declare_model_path_cmd = DeclareLaunchArgument(
        'model_path',
        default_value='/home/khanhtrinh/trinh/ag/model/cardbox_train/weights/best.pt'
    )

    nav2_auto_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_share, 'launch', 'nav2_auto.launch.py')
        )
    )

    spawn_box_cmd = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-file', os.path.join(pkg_share, 'urdf', 'cardboard_box.sdf'),
            '-name', 'cardboard_box',
            '-x', LaunchConfiguration('box_x'),
            '-y', LaunchConfiguration('box_y'),
            '-z', LaunchConfiguration('box_z'),
        ],
        output='screen'
    )

    box_picker_cmd = Node(
        package=package_name,
        executable='box_picker_node.py',
        name='box_picker_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'model_path': LaunchConfiguration('model_path'),
            'conf_thresh': 0.4,
            'imgsz': 320,
            'use_clip': True,
            'clip_thresh': 0.60,
            'detect_timeout': 15.0,
            'qr_timeout': 5.0,
            'nav_timeout': 120.0,
            'upper_body_down': -0.30,
            'upper_body_up': 0.065,
            'left_arm_pick_deg': -40.0,
            'right_arm_pick_deg': 40.0,
            'lower_duration': 4.0,
            'arm_duration': 2.0,
        }]
    )

    ld = LaunchDescription()
    ld.add_action(declare_box_x_cmd)
    ld.add_action(declare_box_y_cmd)
    ld.add_action(declare_box_z_cmd)
    ld.add_action(declare_pick_x_cmd)
    ld.add_action(declare_pick_y_cmd)
    ld.add_action(declare_pick_yaw_cmd)
    ld.add_action(declare_model_path_cmd)
    ld.add_action(nav2_auto_cmd)
    ld.add_action(spawn_box_cmd)
    ld.add_action(box_picker_cmd)
    return ld