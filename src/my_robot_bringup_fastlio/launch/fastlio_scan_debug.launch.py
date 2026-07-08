from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('my_robot_bringup_fastlio')

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py',
            ])
        )
    )

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py',
            ])
        ),
        launch_arguments={
            'config_path': PathJoinSubstitution([bringup_share, 'config']),
            'config_file': 'mid360_fastlio.yaml',
            'rviz': 'false',
        }.items(),
    )

    body_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_footprint_tf',
        output='screen',
        arguments=[
            '--x', '0.0',
            '--y', '0.0',
            '--z', '0.0',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'body',
            '--child-frame-id', 'base_footprint',
        ],
    )

    scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='fastlio_scan_converter',
        output='screen',
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_footprint',
            'transform_tolerance': 0.2,
            'min_height': -0.5,
            'max_height': 0.8,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.0087,
            'scan_time': 0.1,
            'range_min': 1.0,
            'range_max': 20.0,
            'use_inf': True,
            'lazy': False,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_scan_debug',
        arguments=[
            '-d',
            PathJoinSubstitution([bringup_share, 'rviz', 'fastlio_scan_debug.rviz']),
        ],
        output='screen',
    )

    return LaunchDescription([
        livox_launch,
        body_to_base_tf,
        TimerAction(
            period=2.0,
            actions=[fastlio_launch],
        ),
        TimerAction(
            period=4.0,
            actions=[scan_node, rviz_node],
        ),
    ])
