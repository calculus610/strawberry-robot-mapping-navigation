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
- `nav2_bringup`, `nav2_mppi_controller`, `pointcloud_to_laserscan`, `tf2_ros`,
  `rviz2`: these are also expected to come from the ROS2 system installation.
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
      serial_ros2/
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

`serial_ros2` is the explicit source dependency of the local Wheeltec chassis
driver. Keeping it here avoids loading the unrelated shared `wheeltec_ros2`
workspace merely to obtain `serialConfig.cmake`.

## 开机后启动

每次开机只选择下面一种模式启动，不要再单独启动
`fastlio_2d_slam.launch.py`、`base_serial.launch.py` 或旧的
`wheeltec_lidar.launch.py`。

### 自主探索建图

让 Nav2 和 explore_lite 自动控制小车探索并建图：

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py
```

该命令复用系统中已运行的公共底盘服务，并启动 MID360、FAST-LIO、二维栅格
建图、Nav2、explore_lite。不要同时启动第二套底盘节点；只有在公共底盘服务已被
明确停止时才可增加 `start_base:=true`。为避免嵌入式主机上的 RViz 段错误和
资源抢占，RViz 默认不启动；
需要可视化时可在命令末尾增加 `start_rviz:=true`。启动后不要再运行下面的手动建图命令。

`nav2_bringup` 和 `nav2_mppi_controller` 使用 `/opt/ros/humble` 中通过
APT 安装的官方 Nav2 1.1.20。该控制器使用 Ackermann 运动模型，匹配
`senior_akm` 底盘；不要改回不约束最小转弯半径的 DWB。

底盘使用稳定设备名 `/dev/wheeltec_controller`（当前实际指向
`/dev/ttyCH343USB1`），不要改成 Jetson 板载串口 `/dev/ttyTHS1`。
串口读取超时已缩短到 20 ms，并且无数据超时时不会把无效字节送进
帧解析器，因此 `/cmd_vel` 回调不会被阻塞、`/odom` 也能维持约 20 Hz。
底盘节点每 2 秒打印一次实际下发的 `linear.x` 和 `angular.z`，可用于
区分导航配置问题与下位机协议问题。

### 键盘遥控建图

只启动建图链路，不启动 Nav2 和 explore_lite：

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup_fastlio manual_mapping.launch.py
```

启动后会为本次任务生成一个带时间戳的地图名，并每 60 秒把最新 `/map`
原子更新到 `~/maps/manual_mapping/`。正常按一次 `Ctrl-C` 结束时会再保存最终版本：

```text
~/maps/manual_mapping/manual_YYYYMMDD_HHMMSS.pgm
~/maps/manual_mapping/manual_YYYYMMDD_HHMMSS.yaml
```

这两个文件可直接交给 Nav2 `map_server`。终端出现 `Saved ... (shutdown)` 后再关闭
SSH 窗口。需要固定名称或其他目录时可以这样启动：

```bash
ros2 launch my_robot_bringup_fastlio manual_mapping.launch.py \
  map_name:=floor1 \
  map_output_dir:=$HOME/maps/nav2
```

建图过程中也可以立即保存；该调用仍会使用本次启动时确定的名称：

```bash
ros2 service call /map_autosaver/save_now std_srvs/srv/Trigger '{}'
```

`map_autosave_interval:=0` 可关闭周期备份，`enable_map_autosave:=false` 可完全关闭
自动保存。周期备份与退出保存写入同一组临时文件后再替换正式文件，避免读取到
只写了一半的地图。`kill -9` 或突然断电无法执行最终保存，但通常仍会保留最近一次
周期备份。

该命令默认复用公共底盘服务，不会再次占用 `/dev/wheeltec_controller`。只有在公共
底盘服务已被明确停止时才可使用 `start_base:=true`。

若 `systemctl` 显示底盘服务为 `active`，但实际不存在 `/wheeltec_robot` 或
`/cmd_vel`，说明外层 launch 进程仍在而串口驱动子进程已经退出。在 Mac 工作区执行：

```bash
./scripts/robot.sh restart-base
```

该命令会通过 `systemctl restart` 干净停止底盘服务的完整进程组，重新启动后再检查
`wheeltec_robot_node`，不会按模糊进程名误杀导航组或 Web 服务组的节点。

该启动文件默认打开同一个 RViz 窗口并同时显示：

- `FAST-LIO 3D Map`：`/Laser_map` 累计三维点云，用于判断 FAST-LIO 是否先发生漂移；
- `FAST-LIO Current Scan`：`/cloud_registered` 当前配准点云；
- `FAST-LIO Path` 和 `FAST-LIO Odometry`：三维里程计轨迹与位姿；
- `Slam Toolbox 2D Map`：`/map` 二维闭环建图结果；
- `Projected 2D Scan`：送入 Slam Toolbox 的 `/scan`。

当前二维栅格图直接采用 FAST-LIO 的 `camera_init` 里程计位姿：
`use_scan_matching` 和 `do_loop_closing` 均保持关闭。原因是长直走廊中的二维
扫描沿前进方向高度重复，重复进行二维 scan matching 曾把地图长度压缩到三维
地图的约一半；相似房间的二维回环也曾产生错误吸附。重新开启前应先录制数据包
并分别验证相邻扫描约束和回环约束。

RViz 使用 `Orbit` 视角，不再固定为俯视，可用鼠标拖动进行三维环绕观察。

另开终端进行键盘遥控：

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

### 启动后检查

```bash
ros2 topic info /scan
ros2 topic info /map
ros2 run tf2_ros tf2_echo camera_init fastlio_base_footprint
```

正常情况下 `/scan` 和 `/map` 的 `Publisher count` 都应为 `1`，TF 应连续输出。
刚启动时短暂出现一两条 `Invalid frame ID` 是节点初始化造成的。

自主探索模式启动：

```text
shared robot-base.service (not launched again)
livox_ros_driver2 MID360 driver
FAST-LIO
pointcloud_to_laserscan
scan_qos_relay
slam_toolbox
Nav2
explore_lite
```

手动建图模式复用公共底盘服务，只启动 MID360、FAST-LIO、点云转激光、
QoS 中继、slam_toolbox 和 RViz。

## Low-battery autostart

`low_battery_manager` is an independent safety package outside the mapping
source set. It should be deployed as a systemd service so the voltage monitor
starts when the robot boots, instead of depending on the autonomous exploration
launch file.

## Main TF chain

```text
map -> camera_init -> body -> fastlio_base_footprint
```

`body -> fastlio_base_footprint` is launched as a static transform because
FAST-LIO already publishes `camera_init -> body`. The separate frame prevents
the mapping tree from competing with the shared EKF transform
`odom_combined -> base_footprint`. On this robot the MID360 body/IMU is mounted
0.188 m in front of and 0.25 m above the chassis origin. With aligned axes, the
inverse transform published by the launch files is therefore
`body -> fastlio_base_footprint = [-0.188, 0.0, -0.25, 0.0, 0.0, 0.0]`.
