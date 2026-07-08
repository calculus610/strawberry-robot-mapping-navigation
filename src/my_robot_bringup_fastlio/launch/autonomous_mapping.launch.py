from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


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
        default_value='mid360_fastlio.yaml',
        description='FAST-LIO config file name',
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
        default_value='true',
        description='Launch Wheeltec chassis driver',
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
        default_value='0.0',
        description='Static TF X offset from FAST-LIO body to base_footprint',
    )

    body_to_base_y_arg = DeclareLaunchArgument(
        'body_to_base_y',
        default_value='0.0',
        description='Static TF Y offset from FAST-LIO body to base_footprint',
    )

    body_to_base_z_arg = DeclareLaunchArgument(
        'body_to_base_z',
        default_value='0.0',
        description='Static TF Z offset from FAST-LIO body to base_footprint',
    )

    body_to_base_roll_arg = DeclareLaunchArgument(
        'body_to_base_roll',
        default_value='0.0',
        description='Static TF roll offset from FAST-LIO body to base_footprint',
    )

    body_to_base_pitch_arg = DeclareLaunchArgument(
        'body_to_base_pitch',
        default_value='0.0',
        description='Static TF pitch offset from FAST-LIO body to base_footprint',
    )

    body_to_base_yaw_arg = DeclareLaunchArgument(
        'body_to_base_yaw',
        default_value='0.0',
        description='Static TF yaw offset from FAST-LIO body to base_footprint',
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

    return LaunchDescription([
        use_sim_time_arg,
        config_path_arg,
        config_file_arg,
        start_rviz_arg,
        rviz_config_arg,
        nav2_params_file_arg,
        autostart_arg,
        use_respawn_arg,
        log_level_arg,
        nav2_start_delay_arg,
        start_base_arg,
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
        fastlio_slam_launch,
        TimerAction(
            period=LaunchConfiguration('nav2_start_delay'),
            actions=[nav2_launch],
        ),
    ])
