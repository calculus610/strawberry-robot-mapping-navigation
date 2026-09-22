from ament_index_python.packages import get_package_prefix
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def assert_official_nav2_underlay():
    expected_prefix = '/opt/ros/humble'
    packages = ('nav2_bringup', 'nav2_mppi_controller')
    unexpected = {
        package: get_package_prefix(package)
        for package in packages
        if get_package_prefix(package) != expected_prefix
    }
    if unexpected:
        details = ', '.join(
            f'{package}={prefix}'
            for package, prefix in unexpected.items()
        )
        raise RuntimeError(
            'Autonomous mapping requires the official /opt/ros/humble Nav2, '
            f'but the current shell resolved {details}. Start it through the '
            'isolated Web autonomous-mapping entry or ./scripts/robot.sh shell.'
        )


def generate_launch_description():
    assert_official_nav2_underlay()
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
        default_value='false',
        description='Launch RViz together with mapping (disabled by default for stability)',
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

    explorer_params_file_arg = DeclareLaunchArgument(
        'explorer_params_file',
        default_value=PathJoinSubstitution([
            bringup_share,
            'config',
            'ackermann_explorer_fastlio.yaml',
        ]),
        description='Ackermann-aware frontier explorer parameter file',
    )

    explorer_executable_arg = DeclareLaunchArgument(
        'explorer_executable',
        default_value='ackermann_explore',
        description=(
            'Frontier explorer executable. Use explore only for the legacy '
            'Euclidean-distance baseline.'
        ),
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

    start_explorer_arg = DeclareLaunchArgument(
        'start_explorer',
        default_value='true',
        description='Launch the configured frontier exploration node',
    )

    explorer_dry_run_arg = DeclareLaunchArgument(
        'explorer_dry_run',
        default_value='false',
        description=(
            'Evaluate and visualize one frontier selection without sending '
            'NavigateToPose'
        ),
    )

    explorer_start_delay_arg = DeclareLaunchArgument(
        'explorer_start_delay',
        default_value='30.0',
        description='Delay before frontier exploration starts, in seconds',
    )

    return_to_start_arg = DeclareLaunchArgument(
        'return_to_start',
        default_value='true',
        description='Return to the first recorded exploration pose when complete',
    )

    return_timeout_arg = DeclareLaunchArgument(
        'return_timeout',
        default_value='120.0',
        description='Maximum return-to-start navigation time in seconds',
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
        description='Enable periodic, completion-triggered, and Ctrl-C map saves',
    )

    map_output_dir_arg = DeclareLaunchArgument(
        'map_output_dir',
        default_value='~/maps/autonomous_mapping',
        description='Directory for autonomous Nav2 .pgm and .yaml map files',
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='',
        description='Map prefix; empty creates autonomous_YYYYMMDD_HHMMSS',
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

    autonomous_mapping_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_share,
                'launch',
                'autonomous_mapping.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_path': LaunchConfiguration('config_path'),
            'config_file': LaunchConfiguration('config_file'),
            'scan_params_file': LaunchConfiguration('scan_params_file'),
            'start_rviz': LaunchConfiguration('start_rviz'),
            'rviz_config': LaunchConfiguration('rviz_config'),
            'nav2_params_file': LaunchConfiguration('nav2_params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_respawn': LaunchConfiguration('use_respawn'),
            'log_level': LaunchConfiguration('log_level'),
            'nav2_start_delay': LaunchConfiguration('nav2_start_delay'),
            'start_base': LaunchConfiguration('start_base'),
            'enable_map_autosave': LaunchConfiguration('enable_map_autosave'),
            'map_output_dir': LaunchConfiguration('map_output_dir'),
            'map_name': LaunchConfiguration('map_name'),
            'map_autosave_interval': LaunchConfiguration('map_autosave_interval'),
            'car_mode': LaunchConfiguration('car_mode'),
            'imu_mode': LaunchConfiguration('imu_mode'),
            'ranger_avoid_flag': LaunchConfiguration('ranger_avoid_flag'),
            'body_to_base_x': LaunchConfiguration('body_to_base_x'),
            'body_to_base_y': LaunchConfiguration('body_to_base_y'),
            'body_to_base_z': LaunchConfiguration('body_to_base_z'),
            'body_to_base_roll': LaunchConfiguration('body_to_base_roll'),
            'body_to_base_pitch': LaunchConfiguration('body_to_base_pitch'),
            'body_to_base_yaw': LaunchConfiguration('body_to_base_yaw'),
        }.items(),
    )

    explore_lite_node = Node(
        package='explore_lite',
        executable=LaunchConfiguration('explorer_executable'),
        name='ackermann_frontier_explorer',
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_explorer')),
        parameters=[
            LaunchConfiguration('explorer_params_file'),
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'dry_run': LaunchConfiguration('explorer_dry_run'),
                'return_to_start': ParameterValue(
                    LaunchConfiguration('return_to_start'),
                    value_type=bool,
                ),
                'return_timeout': ParameterValue(
                    LaunchConfiguration('return_timeout'),
                    value_type=float,
                ),
                'save_on_completion': ParameterValue(
                    LaunchConfiguration('enable_map_autosave'),
                    value_type=bool,
                ),
            },
        ],
    )

    explorer_exit_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=explore_lite_node,
            on_exit=[
                Shutdown(
                    reason=(
                        'Ackermann explorer exited after completion or a '
                        'fatal error'
                    )
                ),
            ],
        )
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_path_arg,
        config_file_arg,
        scan_params_file_arg,
        start_rviz_arg,
        rviz_config_arg,
        nav2_params_file_arg,
        explorer_params_file_arg,
        explorer_executable_arg,
        autostart_arg,
        use_respawn_arg,
        log_level_arg,
        nav2_start_delay_arg,
        start_explorer_arg,
        explorer_dry_run_arg,
        explorer_start_delay_arg,
        return_to_start_arg,
        return_timeout_arg,
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
        autonomous_mapping_launch,
        explorer_exit_handler,
        TimerAction(
            period=LaunchConfiguration('explorer_start_delay'),
            actions=[explore_lite_node],
        ),
    ])
