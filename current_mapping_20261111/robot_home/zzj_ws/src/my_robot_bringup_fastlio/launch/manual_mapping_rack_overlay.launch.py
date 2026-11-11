"""Old slam_toolbox base map plus independently confirmed elevated obstacles."""
from datetime import datetime
from zoneinfo import ZoneInfo

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('my_robot_bringup_fastlio')
    session = 'rack_overlay_' + datetime.now(ZoneInfo('Asia/Shanghai')).strftime('%Y%m%d_%H%M%S')
    use_sim_time = LaunchConfiguration('use_sim_time')
    common = {'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_overlay_rviz', default_value='true'),
        DeclareLaunchArgument('record_bag', default_value='true'),
        DeclareLaunchArgument('enable_overlay_map_autosave', default_value='true'),
        DeclareLaunchArgument('min_height', default_value='0.3'),
        DeclareLaunchArgument('max_height', default_value='1.0'),
        DeclareLaunchArgument('map_output_dir', default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'), 'maps', 'manual_mapping_rack_overlay', session])),
        DeclareLaunchArgument('bag_output_dir', default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'), 'rosbags', session])),
    ]
    baseline = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            share, 'launch', 'manual_mapping.launch.py'])),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'start_rviz': 'false', 'start_base': 'false',
            'use_octomap': 'false', 'enable_map_autosave': 'false',
        }.items(),
    )
    overlay = Node(
        package='my_robot_bringup_fastlio', executable='rack_obstacle_layer',
        name='rack_obstacle_layer', output='screen', parameters=[
            PathJoinSubstitution([share, 'config', 'manual_rack_overlay.yaml']),
            common, {
                'min_height': ParameterValue(LaunchConfiguration('min_height'), value_type=float),
                'max_height': ParameterValue(LaunchConfiguration('max_height'), value_type=float),
            },
        ],
    )
    savers = []
    for name, topic in [('baseline', '/map'), ('fused', '/map_rack_overlay')]:
        savers.append(Node(
            package='my_robot_bringup_fastlio', executable='map_autosaver',
            name='rack_' + name + '_autosaver', output='screen',
            condition=IfCondition(LaunchConfiguration('enable_overlay_map_autosave')),
            parameters=[common, {
                'map_topic': topic,
                'output_directory': ParameterValue(LaunchConfiguration('map_output_dir'), value_type=str),
                'map_name': name, 'map_name_prefix': name,
                'periodic_save_interval': 60.0,
                'free_threshold': 0.19, 'occupied_threshold': 0.65,
            }],
        ))
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz_rack_overlay', output='screen',
        condition=IfCondition(LaunchConfiguration('start_overlay_rviz')),
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'manual_mapping_rack_overlay.rviz'])],
        parameters=[common],
    )
    recorder = ExecuteProcess(
        cmd=['ros2', 'bag', 'record', '--output', LaunchConfiguration('bag_output_dir'),
             '--qos-profile-overrides-path', PathJoinSubstitution([
                 share, 'config', 'rack_overlay_record_qos.yaml']),
             '/cloud_registered_body', '/Odometry', '/livox/imu', '/tf', '/tf_static',
             '/map', '/map_rack_overlay', '/rack_layer/confirmed_map', '/rosout'],
        condition=IfCondition(LaunchConfiguration('record_bag')),
        output='screen', sigterm_timeout='15.0', sigkill_timeout='5.0',
    )
    # The baseline's timers resolve launch configurations after this include returns.
    # Keep those configurations available; independent overlay switches prevent the
    # baseline's start_rviz=false / enable_map_autosave=false from disabling ours.
    return LaunchDescription(arguments + [baseline, overlay, *savers, rviz, recorder])
