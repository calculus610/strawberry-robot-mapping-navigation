"""Manual test: cumulative FAST-LIO cloud -> height-only binary projection."""
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
    use_sim_time = LaunchConfiguration('use_sim_time')
    arguments = [
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('enable_map_autosave', default_value='true'),
        DeclareLaunchArgument('min_z', default_value='-0.2'),
        DeclareLaunchArgument('max_z', default_value='0.8'),
        DeclareLaunchArgument('map_output_dir', default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'), 'maps', 'manual_mapping_height_projection'])),
        DeclareLaunchArgument('map_name', default_value=''),
    ]
    fastlio = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            share, 'launch', 'fastlio_mid360.launch.py'])),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
    )
    projection = Node(
        package='my_robot_bringup_fastlio', executable='height_projection_map',
        name='height_projection_map', output='screen', parameters=[
            PathJoinSubstitution([share, 'config', 'height_projection.yaml']),
            {'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
             'min_z': ParameterValue(LaunchConfiguration('min_z'), value_type=float),
             'max_z': ParameterValue(LaunchConfiguration('max_z'), value_type=float)},
        ],
    )
    saver = Node(
        package='my_robot_bringup_fastlio', executable='map_autosaver',
        name='height_projection_autosaver', output='screen',
        condition=IfCondition(LaunchConfiguration('enable_map_autosave')),
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'map_topic': '/map_height_projection',
            'output_directory': ParameterValue(LaunchConfiguration('map_output_dir'), value_type=str),
            'map_name': ParameterValue(LaunchConfiguration('map_name'), value_type=str),
            'map_name_prefix': 'height_projection',
            'periodic_save_interval': 60.0,
            # Preserve gray 205 as unknown when reloaded in trinary mode:
            # 1 - 205/255 = 0.196... must remain ABOVE free_threshold.
            'free_threshold': 0.19,
            'occupied_threshold': 0.65,
        }],
    )
    rviz = Node(
        package='rviz2', executable='rviz2', name='rviz_height_projection',
        output='screen', condition=IfCondition(LaunchConfiguration('start_rviz')),
        arguments=['-d', PathJoinSubstitution([
            share, 'rviz', 'manual_mapping_height_projection.rviz'])],
        parameters=[{'use_sim_time': ParameterValue(use_sim_time, value_type=bool)}],
    )
    # No base driver, LaserScan conversion, slam_toolbox, OctoMap, or new TF.
    # Both the projection and RViz use FAST-LIO's existing camera_init frame.
    return LaunchDescription(arguments + [fastlio, projection, saver, rviz])
