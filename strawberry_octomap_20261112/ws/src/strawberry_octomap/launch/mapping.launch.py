"""Exclusive candidate mapping; no base bringup, navigation, or motion commands."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def setup(context):
    share = Path(get_package_share_directory('strawberry_octomap'))
    driver_share = Path(get_package_share_directory('livox_ros_driver2'))
    output = Path(LaunchConfiguration('output_dir').perform(context))
    if not output.is_absolute() or not output.is_dir():
        raise RuntimeError('output_dir must be an existing absolute session directory')
    lidar = Node(
        package='livox_ros_driver2', executable='livox_ros_driver2_node',
        name='livox_lidar_publisher', namespace='strawberry_octomap',
        output='screen', cwd=str(output),
        parameters=[{
            'xfer_format': 1, 'multi_topic': 0, 'data_src': 0,
            'publish_freq': 10.0, 'output_data_type': 0,
            'frame_id': 'livox_frame',
            'lvx_file_path': str(output / 'unused.lvx'),
            'user_config_path': str(driver_share / 'config/MID360_config.json'),
            'cmdline_input_bd_code': 'livox0000000001',
        }],
        remappings=[('/livox/lidar', '/strawberry_octomap/livox/lidar'),
                    ('/livox/imu', '/strawberry_octomap/livox/imu'),
                    ('livox/lidar', '/strawberry_octomap/livox/lidar'),
                    ('livox/imu', '/strawberry_octomap/livox/imu')],
    )
    fastlio = Node(
        package='fast_lio', executable='fastlio_mapping',
        name='fastlio_mapping', namespace='strawberry_octomap',
        output='screen', cwd=str(output),
        parameters=[str(share / 'config/fastlio.yaml')],
        remappings=[
            ('/livox/lidar', '/strawberry_octomap/livox/lidar'),
            ('/livox/imu', '/strawberry_octomap/livox/imu'),
            ('/fastlio/free_scan', '/strawberry_octomap/fastlio/free_scan'),
            ('/fastlio/free_scan_pose', '/strawberry_octomap/fastlio/free_scan_pose'),
            ('/fastlio/native_height_reference', '/strawberry_octomap/fastlio/native_height_reference'),
            ('/Odometry', '/strawberry_octomap/odometry'),
            ('/path', '/strawberry_octomap/path'),
            ('/cloud_registered', '/strawberry_octomap/cloud_registered'),
            ('/cloud_registered_body', '/strawberry_octomap/cloud_registered_body'),
            ('/cloud_effected', '/strawberry_octomap/cloud_effected'),
            ('/Laser_map', '/strawberry_octomap/unused_native_map'),
        ],
    )
    bridge = Node(
        package='strawberry_octomap', executable='observation_bridge.py',
        name='observation_bridge', namespace='strawberry_octomap',
        output='screen', cwd=str(output),
    )
    server = Node(
        package='strawberry_octomap', executable='occupancy_server',
        name='octomap_server', namespace='manual_octomap',
        parameters=[str(share / 'config/octomap.yaml'), {
            'output_dir': str(output),
            'publish_web_map': LaunchConfiguration('publish_web_map').perform(context).lower() == 'true',
        }],
        remappings=[('cloud_in', '/strawberry_octomap/cloud_in'),
                    ('projected_map', '/strawberry_octomap/map')],
        output='screen', cwd=str(output),
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz2', namespace='strawberry_octomap',
        arguments=['-d', str(share / 'rviz/mapping.rviz')],
        condition=IfCondition(LaunchConfiguration('start_rviz')),
        output='screen', cwd=str(output),
    )
    guards = [RegisterEventHandler(OnProcessExit(
        target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Critical candidate mapping node exited'))],
    )) for node in (lidar, fastlio, bridge, server)]
    return guards + [bridge, server, lidar, TimerAction(period=2.0, actions=[fastlio]), rviz]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('output_dir'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('publish_web_map', default_value='false'),
        OpaqueFunction(function=setup),
    ])
