from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, RegisterEventHandler, EmitEvent, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = Path(get_package_share_directory('native_height_projection'))
    session = LaunchConfiguration('session_dir')
    fastlio = Node(
        package='fast_lio', executable='fastlio_mapping', output='screen',
        parameters=[str(share / 'config/native180_fastlio.yaml'),
                    {'map_file_path': [session, '/native_map_final_camera_init.pcd']}],
        remappings=[('/Laser_map', '/fastlio/native_map'), ('/map_save', '/native_map_save')])
    projector = Node(
        package='native_height_projection', executable='native_height_projector.py', output='screen',
        parameters=[str(share / 'config/projection.yaml'), {'output_dir': session}])
    rviz = Node(
        package='rviz2', executable='rviz2', name='native180_rviz',
        arguments=['-d', str(share / 'rviz/manual_native180.rviz')],
        output='log', condition=IfCondition(LaunchConfiguration('start_rviz')))
    driver = IncludeLaunchDescription(PythonLaunchDescriptionSource(str(
        Path(get_package_share_directory('livox_ros_driver2')) / 'launch_ROS2/msg_MID360_launch.py')),
        condition=IfCondition(LaunchConfiguration('start_lidar')))
    # Shut down the session if a required mapper exits. ROS launch owns its
    # own children; no global pkill, no velocity publisher and no base startup.
    handlers = [RegisterEventHandler(OnProcessExit(target_action=node,
        on_exit=[EmitEvent(event=Shutdown(reason='Required mapping process exited'))]))
        for node in (fastlio, projector)]
    return LaunchDescription([
        DeclareLaunchArgument('session_dir', description='Unique output directory'),
        DeclareLaunchArgument('start_rviz', default_value='true'),
        DeclareLaunchArgument('start_lidar', default_value='true'),
        *handlers, projector, driver, TimerAction(period=2.0, actions=[fastlio]), rviz])
