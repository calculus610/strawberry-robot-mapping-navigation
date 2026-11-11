from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    bringup_share = FindPackageShare('my_robot_bringup_fastlio')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )
    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Start RViz for manual mapping',
    )
    use_octomap_arg = DeclareLaunchArgument(
        'use_octomap',
        default_value='true',
        choices=['true', 'false'],
        description='Use height-aware OctoMap projection for RViz and manual map saving',
    )
    selected_map_topic = PythonExpression([
        "'/map_octomap' if '", LaunchConfiguration('use_octomap'),
        "' == 'true' else '/map'",
    ])
    selected_rviz_file = PythonExpression([
        "'manual_mapping_octomap.rviz' if '", LaunchConfiguration('use_octomap'),
        "' == 'true' else 'fastlio_2d_slam.rviz'",
    ])

    start_base_arg = DeclareLaunchArgument(
        'start_base',
        default_value='false',
        description=(
            'Launch a local Wheeltec chassis driver. Keep false when the shared '
            'robot-base.service is active.'
        ),
    )
    enable_map_autosave_arg = DeclareLaunchArgument(
        'enable_map_autosave',
        default_value='true',
        description='Periodically save the selected 2D map and perform a final save on Ctrl-C',
    )
    map_output_dir_arg = DeclareLaunchArgument(
        'map_output_dir',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            'maps',
            'manual_mapping',
        ]),
        description='Directory for timestamped Nav2 .pgm and .yaml map files',
    )
    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description=(
            'Map file prefix. Empty creates manual_YYYYMMDD_HHMMSS so older '
            'mapping sessions are not overwritten.'
        ),
    )
    map_autosave_interval_arg = DeclareLaunchArgument(
        'map_autosave_interval',
        default_value='60.0',
        description='Periodic backup interval in seconds; zero disables periodic backup',
    )
    car_mode_arg = DeclareLaunchArgument(
        'car_mode',
        default_value='',
        description='Override Wheeltec car mode; empty uses wheeltec_param.yaml',
    )
    imu_mode_arg = DeclareLaunchArgument(
        'imu_mode',
        default_value='',
        description='Override Wheeltec IMU mode; empty uses wheeltec_param.yaml',
    )
    body_to_base_x_arg = DeclareLaunchArgument(
        'body_to_base_x',
        default_value='-0.188',
        description='X coordinate of fastlio_base_footprint in body frame; MID360 is 0.188 m in front of the base origin',
    )
    body_to_base_y_arg = DeclareLaunchArgument(
        'body_to_base_y',
        default_value='0.0',
        description='Y coordinate of fastlio_base_footprint in body frame',
    )
    body_to_base_z_arg = DeclareLaunchArgument(
        'body_to_base_z',
        default_value='-0.25',
        description='Z coordinate of fastlio_base_footprint in body frame; MID360 is 0.25 m above the base origin',
    )
    body_to_base_roll_arg = DeclareLaunchArgument(
        'body_to_base_roll',
        default_value='0.0',
        description='Roll from body to fastlio_base_footprint; zero assumes aligned axes',
    )
    body_to_base_pitch_arg = DeclareLaunchArgument(
        'body_to_base_pitch',
        default_value='0.0',
        description='Pitch from body to fastlio_base_footprint; zero assumes aligned axes',
    )
    body_to_base_yaw_arg = DeclareLaunchArgument(
        'body_to_base_yaw',
        default_value='0.0',
        description='Yaw from body to fastlio_base_footprint; zero assumes aligned axes',
    )

    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('turn_on_wheeltec_robot'),
                'launch',
                'base_serial.launch.py',
            ])
        ),
        condition=IfCondition(LaunchConfiguration('start_base')),
        launch_arguments={
            'car_mode': LaunchConfiguration('car_mode'),
            'imu_mode': LaunchConfiguration('imu_mode'),
        }.items(),
    )

    mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_share,
                'launch',
                'fastlio_2d_slam.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'rviz_config': PathJoinSubstitution([
                bringup_share, 'rviz', selected_rviz_file,
            ]),
            'body_to_base_x': LaunchConfiguration('body_to_base_x'),
            'body_to_base_y': LaunchConfiguration('body_to_base_y'),
            'body_to_base_z': LaunchConfiguration('body_to_base_z'),
            'body_to_base_roll': LaunchConfiguration('body_to_base_roll'),
            'body_to_base_pitch': LaunchConfiguration('body_to_base_pitch'),
            'body_to_base_yaw': LaunchConfiguration('body_to_base_yaw'),
        }.items(),
    )

    # Keep the existing scan/SLAM map for comparison and its map TF.
    # The namespaced server consumes current frames, not the growing /Laser_map.
    octomap_node = Node(
        package='octomap_server',
        executable='octomap_server_node',
        name='octomap_server',
        namespace='manual_octomap',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_octomap')),
        parameters=[
            PathJoinSubstitution([bringup_share, 'config', 'octomap_manual.yaml']),
            {'use_sim_time': ParameterValue(
                LaunchConfiguration('use_sim_time'), value_type=bool)},
        ],
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('projected_map', '/map_octomap'),
        ],
    )

    map_autosaver = Node(
        package='my_robot_bringup_fastlio',
        executable='map_autosaver',
        name='map_autosaver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_map_autosave')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_topic': ParameterValue(selected_map_topic, value_type=str),
            'output_directory': ParameterValue(
                LaunchConfiguration('map_output_dir'),
                value_type=str,
            ),
            'map_name': ParameterValue(
                LaunchConfiguration('map_name'),
                value_type=str,
            ),
            'map_name_prefix': ParameterValue(PythonExpression([
                "'manual_octomap' if '", LaunchConfiguration('use_octomap'),
                "' == 'true' else 'manual'",
            ]), value_type=str),
            'periodic_save_interval': ParameterValue(
                LaunchConfiguration('map_autosave_interval'),
                value_type=float,
            ),
            'free_threshold': 0.25,
            'occupied_threshold': 0.65,
        }],
    )

    return LaunchDescription([
        use_sim_time_arg,
        start_rviz_arg,
        start_base_arg,
        use_octomap_arg,
        enable_map_autosave_arg,
        map_output_dir_arg,
        map_name_arg,
        map_autosave_interval_arg,
        car_mode_arg,
        imu_mode_arg,
        body_to_base_x_arg,
        body_to_base_y_arg,
        body_to_base_z_arg,
        body_to_base_roll_arg,
        body_to_base_pitch_arg,
        body_to_base_yaw_arg,
        base_launch,
        mapping_launch,
        TimerAction(period=9.0, actions=[octomap_node]),
        map_autosaver,
    ])
