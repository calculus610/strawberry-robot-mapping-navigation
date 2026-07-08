from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
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

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=PathJoinSubstitution([
            bringup_share,
            'config',
            'nav2_wheeltec_mid360.yaml',
        ]),
        description='Nav2 parameter file for Wheeltec + MID360 exploration',
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

    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('nav2_bringup'),
                'launch',
                'navigation_launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'params_file': LaunchConfiguration('params_file'),
            'autostart': LaunchConfiguration('autostart'),
            'use_respawn': LaunchConfiguration('use_respawn'),
            'log_level': LaunchConfiguration('log_level'),
            'use_composition': 'False',
        }.items(),
    )

    return LaunchDescription([
        use_sim_time_arg,
        params_file_arg,
        autostart_arg,
        use_respawn_arg,
        log_level_arg,
        nav2_launch,
    ])
