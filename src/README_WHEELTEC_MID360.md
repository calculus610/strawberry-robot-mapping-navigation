# Wheeltec MID360 autonomous mapping overlay

This directory is a clean source set for the Wheeltec MID360 autonomous mapping
workspace. It keeps the previous autonomous mapping logic, includes the Wheeltec
chassis driver, and removes the old Scout chassis source packages.

## What is included

- `FAST_LIO_ROS2`: FAST-LIO mapping from MID360 point cloud and IMU.
- `livox_ros_driver2`: Livox MID360 ROS2 driver.
- `m-explore-ros2/explore`: explore_lite frontier exploration node.
- `m-explore-ros2/explore_lite_msgs`: messages used by explore_lite.
- `my_robot_bringup_fastlio`: integration launch/config package.
- `turn_on_wheeltec_robot`: Wheeltec chassis driver used instead of Scout.
- `wheeltec_robot_msg`: message package required by the Wheeltec chassis driver.
- `depend/ackermann_msgs-ros2`: local Ackermann message dependency used by the
  Wheeltec chassis driver.

## What is not included

- `scout_ros2` and `ugv_sdk`: removed from the clean delivery because the robot
  now uses the Wheeltec chassis driver.
- `slam_toolbox`: this is expected to come from the ROS2 system installation.
  The launch file calls it through `FindPackageShare('slam_toolbox')`.
- `nav2_bringup`, `pointcloud_to_laserscan`, `tf2_ros`, `rviz2`: these are also
  expected to come from the ROS2 system installation.
The original Wheeltec chassis driver is included in this clean source set.

## Expected target workspace layout

This directory can be copied as the working `src` for the autonomous mapping
workspace. It intentionally does not include unrelated Wheeltec demos, camera
packages, voice packages, or the original 2D lidar stack.

The target workspace should contain at least:

```text
wheeltec_ros2/
  src/
    turn_on_wheeltec_robot/
    wheeltec_robot_msg/
    depend/
      ackermann_msgs-ros2/
    FAST_LIO_ROS2/
    livox_ros_driver2/
    m-explore-ros2/
      explore/
      explore_lite_msgs/
    my_robot_bringup_fastlio/
  low_battery_manager/
```

`low_battery_manager` is intentionally placed next to `src`, not inside it. It
is built with `colcon build --base-paths low_battery_manager` and deployed as a
boot-time systemd service.

## Main launch

```bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py
```

This starts:

```text
turn_on_wheeltec_robot/base_serial.launch.py
livox_ros_driver2 MID360 driver
FAST-LIO
pointcloud_to_laserscan
scan_qos_relay
slam_toolbox
Nav2
explore_lite
```

## Low-battery autostart

`low_battery_manager` is an independent safety package outside the mapping
source set. It should be deployed as a systemd service so the voltage monitor
starts when the robot boots, instead of depending on the autonomous exploration
launch file.

## Main TF chain

```text
map -> camera_init -> body -> base_footprint
```

`body -> base_footprint` is launched as a static transform. Its default offset is
zero and should be adjusted to match the actual MID360 mounting position.
