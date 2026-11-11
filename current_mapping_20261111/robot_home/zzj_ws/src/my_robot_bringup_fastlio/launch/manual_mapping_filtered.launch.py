"""Versioned manual mapping output pipeline, using the existing FAST-LIO pose."""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('my_robot_bringup_fastlio')
    sim = LaunchConfiguration('use_sim_time')
    sim_param = {'use_sim_time': ParameterValue(sim, value_type=bool)}
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_sensors', default_value='true'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('enable_map_autosave', default_value='true'),
        DeclareLaunchArgument('filter_config', default_value='manual_filter_v1.yaml'),
        DeclareLaunchArgument('min_z', default_value='-0.2'),
        DeclareLaunchArgument('max_z', default_value='0.8'),
        DeclareLaunchArgument('map_name', default_value=''),
        DeclareLaunchArgument('map_output_dir', default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'), 'maps', 'manual_mapping_v1'])),
    ]
    sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            share, 'launch', 'fastlio_mid360.launch.py'])),
        condition=IfCondition(LaunchConfiguration('start_sensors')),
        launch_arguments={
            'use_sim_time': sim,
            'config_file': 'mid360_manual_filtered.yaml',
        }.items(),
    )
    filtering = Node(
        package='my_robot_bringup_fastlio', executable='manual_cloud_filter.py',
        name='manual_cloud_filter', output='screen', parameters=[
            PathJoinSubstitution([share, 'config', LaunchConfiguration('filter_config')]),
            sim_param,
        ],
    )
    projection = Node(
        package='my_robot_bringup_fastlio', executable='height_projection_map',
        name='height_projection_map', output='screen', parameters=[
            PathJoinSubstitution([share, 'config', 'height_projection.yaml']), sim_param,
            {'cloud_topic': '/manual_mapping/filtered_map',
             'map_topic': '/manual_mapping/map',
             'min_z': ParameterValue(LaunchConfiguration('min_z'), value_type=float),
             'max_z': ParameterValue(LaunchConfiguration('max_z'), value_type=float)},
        ],
    )
    saver = Node(
        package='my_robot_bringup_fastlio', executable='map_autosaver',
        name='manual_filtered_autosaver', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_map_autosave')),
        parameters=[sim_param, {
            'map_topic': '/manual_mapping/map',
            'output_directory': ParameterValue(LaunchConfiguration('map_output_dir'), value_type=str),
            'map_name': ParameterValue(LaunchConfiguration('map_name'), value_type=str),
            'map_name_prefix': 'manual_filtered',
            'periodic_save_interval': 60.0,
            'free_threshold': 0.19,
            'occupied_threshold': 0.65,
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='manual_filtered_rviz',
        condition=IfCondition(LaunchConfiguration('start_rviz')), output='screen',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'manual_mapping_filtered.rviz'])],
        parameters=[sim_param],
    )
    return LaunchDescription(arguments + [sensors, filtering, projection, saver, rviz])
