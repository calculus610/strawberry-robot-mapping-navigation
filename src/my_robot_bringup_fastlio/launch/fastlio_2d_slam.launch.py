from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
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

    scan_params_file_arg = DeclareLaunchArgument(
        'scan_params_file',
        default_value=PathJoinSubstitution([
            bringup_share,
            'config',
            'pointcloud_to_laserscan_fastlio.yaml',
        ]),
        description='pointcloud_to_laserscan parameter file',
    )

    start_rviz_arg = DeclareLaunchArgument(
        'start_rviz',
        default_value='true',
        description='Launch RViz together with FAST-LIO 2D SLAM',
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

    body_to_base_x_arg = DeclareLaunchArgument(
        'body_to_base_x',
        default_value='-0.188',
        description=(
            'X coordinate of fastlio_base_footprint in FAST-LIO body frame. '
            'The MID360 body is mounted 0.188 m in front of the base origin, '
            'so the inverse body->base transform is -0.188 m.'
        ),
    )

    body_to_base_y_arg = DeclareLaunchArgument(
        'body_to_base_y',
        default_value='0.0',
        description='Static TF Y offset from FAST-LIO body to fastlio_base_footprint',
    )

    body_to_base_z_arg = DeclareLaunchArgument(
        'body_to_base_z',
        default_value='-0.25',
        description=(
            'Z coordinate of fastlio_base_footprint in FAST-LIO body frame. '
            'The MID360 body is mounted 0.25 m above the base origin, '
            'so the inverse body->base transform is -0.25 m.'
        ),
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

    fastlio_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                bringup_share,
                'launch',
                'fastlio_mid360.launch.py',
            ])
        ),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'config_path': LaunchConfiguration('config_path'),
            'config_file': LaunchConfiguration('config_file'),
        }.items(),
    )

    body_to_base_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_fastlio_base_footprint_tf',
        output='screen',
        arguments=[
            '--x', LaunchConfiguration('body_to_base_x'),
            '--y', LaunchConfiguration('body_to_base_y'),
            '--z', LaunchConfiguration('body_to_base_z'),
            '--roll', LaunchConfiguration('body_to_base_roll'),
            '--pitch', LaunchConfiguration('body_to_base_pitch'),
            '--yaw', LaunchConfiguration('body_to_base_yaw'),
            '--frame-id', 'body',
            '--child-frame-id', 'fastlio_base_footprint',
        ],
    )

    scan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='fastlio_scan_converter',
        output='screen',
        remappings=[
            ('cloud_in', '/cloud_registered_body'),
            ('scan', '/scan_raw'),
        ],
        parameters=[LaunchConfiguration('scan_params_file')],
    )

    scan_relay_node = Node(
        package='my_robot_bringup_fastlio',
        executable='scan_qos_relay',
        name='scan_qos_relay',
        output='screen',
        parameters=[
            PathJoinSubstitution([
                bringup_share,
                'config',
                'scan_qos_relay.yaml',
            ])
        ],
    )

    slam_toolbox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare('slam_toolbox'),
                'launch',
                'online_async_launch.py',
            ])
        ),
        launch_arguments={
            'slam_params_file': PathJoinSubstitution([
                bringup_share,
                'config',
                'slam_toolbox_fastlio.yaml',
            ]),
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }.items(),
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_fastlio_2d_slam',
        arguments=[
            '-d',
            LaunchConfiguration('rviz_config'),
        ],
        output='screen',
        condition=IfCondition(LaunchConfiguration('start_rviz')),
    )

    return LaunchDescription([
        use_sim_time_arg,
        config_path_arg,
        config_file_arg,
        scan_params_file_arg,
        start_rviz_arg,
        rviz_config_arg,
        body_to_base_x_arg,
        body_to_base_y_arg,
        body_to_base_z_arg,
        body_to_base_roll_arg,
        body_to_base_pitch_arg,
        body_to_base_yaw_arg,
        fastlio_bringup,
        body_to_base_tf,
        TimerAction(
            period=5.0,
            actions=[scan_node, scan_relay_node],
        ),
        TimerAction(
            period=8.0,
            actions=[slam_toolbox_launch, rviz_node],
        ),
    ])
