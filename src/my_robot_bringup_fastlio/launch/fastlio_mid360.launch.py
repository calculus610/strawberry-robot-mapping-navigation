from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    config_path_arg = DeclareLaunchArgument(
        'config_path',
        default_value=PathJoinSubstitution([
            FindPackageShare('my_robot_bringup_fastlio'),
            'config'
        ]),
        description='Directory of FAST-LIO config file'
    )

    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value='mid360_fastlio.yaml',
        description='FAST-LIO config file name'
    )

    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('livox_ros_driver2'),
                'launch_ROS2',
                'msg_MID360_launch.py'
            ])
        )
    )

    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('fast_lio'),
                'launch',
                'mapping.launch.py'
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_path': LaunchConfiguration('config_path'),
            'config_file': LaunchConfiguration('config_file'),
            'rviz': 'false',
        }.items()
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_path_arg,
        config_file_arg,
        livox_launch,
        TimerAction(
            period=2.0,
            actions=[fastlio_launch]
        )
    ])
