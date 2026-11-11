from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import EnvironmentVariable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    bringup_share = FindPackageShare('my_robot_bringup_fastlio')

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true',
    )

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([bringup_share, 'config']),
        description='Directory of FAST-LIO config file',
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='mid360_fastlio_autonomous.yaml',
        description='Autonomous-mapping FAST-LIO config file name',
    )

    scan_params_file_arg = DeclareLaunchArgument(
        'scan_params_file',
        default_value=PathJoinSubstitution([
            bringup_share,
            'config',
            'pointcloud_to_laserscan_fastlio_autonomous.yaml',
        ]),
        description='Autonomous pointcloud_to_laserscan parameter file',
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Launch RViz together with mapping',
    )

    rviz_config_arg = DeclareLaunchArgument(
        'rviz_config',
        default_value=PathJoinSubstitution([
            bringup_share,
            'rviz',
            'fastlio_2d_slam.rviz',
        ]),
        description='RViz config file path',
    )

    nav2_params_file_arg = DeclareLaunchArgument(
        'nav2_params_file',
        default_value=PathJoinSubstitution([
            bringup_share,
            'config',
            'nav2_wheeltec_mid360.yaml',
        ]),
        description='Nav2 parameter file',
    )

    autostart_arg = DeclareLaunchArgument(
        'autostart',
        default_value='true',
        description='Autostart the Nav2 lifecycle manager',
    )

    use_respawn_arg = DeclareLaunchArgument(
        'use_respawn',
        default_value='false',
        description='Respawn Nav2 nodes if they crash',
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Nav2 log level',
    )

    nav2_start_delay_arg = DeclareLaunchArgument(
        'nav2_start_delay',
        default_value='10.0',
        description='Delay before Nav2 starts, in seconds',
    )

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
        description='Periodically save /map and save once more on Ctrl-C',
    )

    map_output_dir_arg = DeclareLaunchArgument(
        'map_output_dir',
        default_value=PathJoinSubstitution([
            EnvironmentVariable('HOME'),
            'maps',
            'autonomous_mapping',
        ]),
        description='Directory for autonomous Nav2 .pgm and .yaml map files',
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description=(
            'Map file prefix. Empty creates autonomous_YYYYMMDD_HHMMSS so '
            'older sessions are not overwritten.'
        ),
    )

    map_autosave_interval_arg = DeclareLaunchArgument(
        'map_autosave_interval',
        default_value='60.0',
        description='Periodic map backup interval in seconds; zero disables it',
    )

    car_mode_arg = DeclareLaunchArgument(
        'car_mode',
        default_value='',
        description='Override Wheeltec car_mode; empty uses wheeltec_param.yaml',
    )

    imu_mode_arg = DeclareLaunchArgument(
        'imu_mode',
        default_value='',
        description='Override Wheeltec imu_mode; empty uses wheeltec_param.yaml',
    )

    ranger_avoid_flag_arg = DeclareLaunchArgument(
        'ranger_avoid_flag',
        default_value='',
        description='Override Wheeltec ranger avoidance flag',
    )

    body_to_base_x_arg = DeclareLaunchArgument(
        'body_to_base_x',
        default_value='-0.188',
        description='X coordinate of fastlio_base_footprint in body frame (inverse of the MID360 forward mounting offset)',
    )

    body_to_base_y_arg = DeclareLaunchArgument(
        'body_to_base_y',
        default_value='0.0',
        description='Static TF Y offset from FAST-LIO body to fastlio_base_footprint',
    )

    body_to_base_z_arg = DeclareLaunchArgument(
        'body_to_base_z',
        default_value='-0.25',
        description='Z coordinate of fastlio_base_footprint in body frame (inverse of the MID360 upward mounting offset)',
    )

    body_to_base_roll_arg = DeclareLaunchArgument(
        'body_to_base_roll',
        default_value='0.0',
        description='Static TF roll offset from FAST-LIO body to fastlio_base_footprint',
    )

    body_to_base_pitch_arg = DeclareLaunchArgument(
        'body_to_base_pitch',
        default_value='0.0',
        description='Static TF pitch offset from FAST-LIO body to fastlio_base_footprint',
    )

    body_to_base_yaw_arg = DeclareLaunchArgument(
        'body_to_base_yaw',
        default_value='0.0',
        description='Static TF yaw offset from FAST-LIO body to fastlio_base_footprint',
    )

    wheeltec_base_launch = IncludeLaunchDescription(
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
            'ranger_avoid_flag': LaunchConfiguration('ranger_avoid_flag'),
        }.items(),
    )

    fastlio_slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_share,
                'launch',
                'fastlio_2d_slam.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_path': LaunchConfiguration('config_path'),
            'config_file': LaunchConfiguration('config_file'),
            'scan_params_file': LaunchConfiguration('scan_params_file'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'body_to_base_x': LaunchConfiguration('body_to_base_x'),
            'body_to_base_y': LaunchConfiguration('body_to_base_y'),
            'body_to_base_z': LaunchConfiguration('body_to_base_z'),
            'body_to_base_roll': LaunchConfiguration('body_to_base_roll'),
            'body_to_base_pitch': LaunchConfiguration('body_to_base_pitch'),
            'body_to_base_yaw': LaunchConfiguration('body_to_base_yaw'),
        }.items(),
    )

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_share,
                'launch',
                'nav2_fastlio.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_respawn': LaunchConfiguration('use_respawn'),
            'log_level': LaunchConfiguration('log_level'),
        }.items(),
    )

    cmd_vel_shutdown_guard = Node(
        package='my_robot_bringup_fastlio',
        executable='cmd_vel_shutdown_guard',
        name='cmd_vel_shutdown_guard',
        output='screen',
        parameters=[{
            'cmd_vel_topic': '/cmd_vel',
            'repeat_count': 10,
            'repeat_interval_ms': 20,
        }],
    )

    map_autosaver = Node(
        package='my_robot_bringup_fastlio',
        executable='map_autosaver',
        name='map_autosaver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_map_autosave')),
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'map_topic': '/map',
            'output_directory': ParameterValue(
                LaunchConfiguration('map_output_dir'),
                value_type=str,
            ),
            'map_name': ParameterValue(
                LaunchConfiguration('map_name'),
                value_type=str,
            ),
            'map_name_prefix': 'autonomous',
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
        config_path_arg,
        config_file_arg,
        scan_params_file_arg,
        start_rviz_arg,
        rviz_config_arg,
        nav2_params_file_arg,
        autostart_arg,
        use_respawn_arg,
        log_level_arg,
        nav2_start_delay_arg,
        start_base_arg,
        enable_map_autosave_arg,
        map_output_dir_arg,
        map_name_arg,
        map_autosave_interval_arg,
        car_mode_arg,
        imu_mode_arg,
        ranger_avoid_flag_arg,
        body_to_base_x_arg,
        body_to_base_y_arg,
        body_to_base_z_arg,
        body_to_base_roll_arg,
        body_to_base_pitch_arg,
        body_to_base_yaw_arg,
        wheeltec_base_launch,
        cmd_vel_shutdown_guard,
        map_autosaver,
        fastlio_slam_launch,
        TimerAction(
            period=LaunchConfiguration('nav2_start_delay'),
            actions=[nav2_launch],
        ),
    ])
