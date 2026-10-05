# Wheeltec MID360 autonomous mapping overlay

This directory is a clean source set for the Wheeltec MID360 autonomous mapping
workspace. It keeps the previous autonomous mapping logic, includes the Wheeltec
chassis driver, and removes the old Scout chassis source packages.

> Historical note: the "clean source set" framing and the expected target
> layout below come from the 2026-07 delivery era. The current canonical
> workspace is `zzj_ws` (see `docs/WORKSPACE_LAYOUT.md`); `low_battery_manager`
> is NOT part of this repository (it is deployed on the robot side and has not
> been recovered into this repo).

## What is included

- `FAST_LIO_ROS2`: FAST-LIO mapping from MID360 point cloud and IMU.
- `livox_ros_driver2`: Livox MID360 ROS2 driver.
- `m-explore-ros2/explore`: legacy explore_lite plus the
  `ackermann_explore` decision node used by autonomous mapping.
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

## Historical delivery context: expected target layout (2026-07)

The tree below is the delivery-era target layout kept for historical record;
the current canonical workspace root is `zzj_ws/`, not `wheeltec_ros2/`.

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

让 Nav2 和阿克曼前沿探索器自动控制小车探索并建图：

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py
```

该命令复用系统中已运行的公共底盘服务，并启动 MID360、FAST-LIO、二维栅格
建图、Nav2、`ackermann_explore`。不要同时启动第二套底盘节点；只有在公共底盘服务已被
明确停止时才可增加 `start_base:=true`。自主建图当前默认启动 RViz；如果嵌入式
主机出现资源不足或 RViz 不稳定，可在命令末尾增加 `start_rviz:=false`。
启动后不要再运行下面的手动建图命令。

`nav2_bringup` 和 `nav2_mppi_controller` 使用 `/opt/ros/humble` 中通过
APT 安装的官方 Nav2 1.1.20。该控制器使用 Ackermann 运动模型，匹配
`senior_akm` 底盘；不要改回不约束最小转弯半径的 DWB。自主建图 launch
会在启动任何机器人进程前校验这两个包的来源；如果当前终端继承了
`wheeltec_ros2` 的 Nav2 overlay，将直接报错而不是带着错误环境运行。

`ackermann_explore` 不会把未知栅格的质心直接交给 Nav2。它先在已知自由区
生成面向 frontier 的观察位姿，再调用 Smac Hybrid-A* 验证候选路径，并按
路径长度、倒车距离、换向次数、初始转向、净空和信息增益评分。导航期间保持
当前目标，只有成功、失败或超时后才重新选择，避免旧 `explore_lite` 的周期性
目标抢占。节点会等待 `controller_server`、`planner_server` 和
`bt_navigator` 都进入 `active` 才开始选择；启动竞态造成的临时目标拒绝不会
拉黑 frontier，也不会进入探索完成计数。

稳定地图上连续 3 轮确实没有检测到 frontier 时，探索器才会发布完成
状态并返回本次启动后首次记录的机器人位姿。已有 frontier 但暂时没有有效
观察位或 Smac 可达路径属于受阻，不计入完成证据。首次记录起点后的 60 秒为启动
保护期，这期间不会自动完成。返航成功、失败或超过 120 秒后，
都会停止继续派发导航目标并请求保存最终二维地图。自主建图同时每 60 秒通过
临时文件 + 同目录 rename 的方式逐文件更新一次地图备份（PGM/YAML 两个文件
之间没有事务性原子保证）：

```text
~/maps/autonomous_mapping/autonomous_YYYYMMDD_HHMMSS.pgm
~/maps/autonomous_mapping/autonomous_YYYYMMDD_HHMMSS.yaml
```

周期保存按 60 秒间隔尝试，但不能保证每个间隔都有完整且成功的 PGM/YAML
文件对；使用前应检查两个文件。正常 `Ctrl-C` 的退出保存是额外的尽力尝试。
可以通过 `return_to_start:=false` 禁用返航，通过 `map_autosave_interval:=0`
只保留完成和退出保存尝试，通过 `enable_map_autosave:=false` 完全关闭保存。

第一次部署后可先执行不移动的选择测试：

```bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py \
  explorer_dry_run:=true
```

该模式会完成一次 frontier 检测、Hybrid-A* 路径评估和 RViz 标记，但不会把
目标发送给 `NavigateToPose`。

底盘使用稳定设备名 `/dev/wheeltec_controller`（当前实际指向
`/dev/ttyCH343USB1`），不要改成 Jetson 板载串口 `/dev/ttyTHS1`。
串口读取超时已缩短到 20 ms，并且无数据超时时不会把无效字节送进
帧解析器，因此 `/cmd_vel` 回调不会被阻塞、`/odom` 也能维持约 20 Hz。
底盘节点每 2 秒打印一次实际下发的 `linear.x` 和 `angular.z`，可用于
区分导航配置问题与下位机协议问题。

公共底盘驱动带有独立的 `/cmd_vel` 看门狗：收到过速度命令后，如果连续
0.5 秒没有新命令，会尝试向下位机发送一次零速度。自主建图 launch 还会启动
`cmd_vel_shutdown_guard`，在收到 `Ctrl-C`（SIGINT）或 SIGTERM 时尝试连续发布
10 次零速度。前者用于应对指令中断，后者用于正常退出时尝试停车；两者都不能
保证物理停车或替代硬件急停，也不要求自主建图进程拥有或关闭公共底盘服务。

### 键盘遥控建图

只启动建图链路，不启动 Nav2 和自主探索器：

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup_fastlio manual_mapping.launch.py
```

启动后会为本次任务生成一个带时间戳的地图名，并每 60 秒尝试把最新 `/map`
逐文件（临时文件 + rename）更新到 `~/maps/manual_mapping/`。PGM/YAML
文件对不具备事务性原子保证；正常按一次 `Ctrl-C` 结束时会再尝试保存：

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
自动保存。周期备份与退出保存尝试逐文件写入临时文件后再替换正式文件，降低读取到
半写入文件的风险；PGM/YAML 文件对不具备事务性原子保证。`kill -9` 或突然断电无法执行最终保存，但通常仍会保留最近一次
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

另开终端进行键盘遥控（若未安装，先
`sudo apt install ros-humble-teleop-twist-keyboard`）：

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
ackermann_explore
```

手动建图模式复用公共底盘服务，只启动 MID360、FAST-LIO、点云转激光、
QoS 中继、slam_toolbox 和 RViz。

## Low-battery autostart

`low_battery_manager` is an independent safety package outside the mapping
source set. It should be deployed as a systemd service so the voltage monitor
starts when the robot boots, instead of depending on the autonomous exploration
launch file. (Historical delivery note: this package lives on the robot side and
has not been recovered into this repository; the description is retained as
deployment context, not as a claim about the current repo contents.)

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

## 已知限制与恢复状态（2026-09）

- 本仓库是项目的 **canonicalized recovered Mac development state**；与机器人端
  正式运行时未完成对账。机器人端 2026-08-24 的正式 nav2 yaml
  （SHA-256 前缀 `15431e20...`）与 `/home/wheeltec/run_auto_mapping.sh`
  均未回收。**恢复完成对账前，禁止从本仓库向机器人执行任何同步。**
- 导航行为树使用 `navigate_{to_pose,through_poses}_no_spin.xml`，behavior_server
  仅启用 Wait 插件：恢复逻辑为清代价地图 + 等待 + 重试 + 失败上抛给探索器。
  **移除 Nav2 BackUp 恢复行为不等于禁用倒车**——车辆仍可倒车，Smac 仍可规划
  含倒车的路径（`reverse_penalty` 提高 其代价），探索器仍会评估含有限倒车的
  planner 路径。
- 0.80 m 最小转弯半径是 Nav2 配置层的工程假设；本仓库没有实测标定依据。
- 恢复的 MPPI 时序（controller_frequency 15 Hz、`model_dt` 0.07）为非相等
  配置：按 Humble MPPI 行为会触发上游警告且 control-sequence shifting 关闭。
  详情与历史对照见《手动建图与自主建图技术栈及实现原理》。
- slam_toolbox 的 `scan_queue_size: 5` 是针对实测 queue-full 行为的项目本地
  调优，不是上游通用推荐值。
- 恢复的阿克曼探索版本是 **current baseline under active experimental
  evaluation**；最终历史 revision 缺少完整的端到端实机验证证据。
- 停车保护（底盘看门狗 / shutdown guard / 探索器零速）是 best-effort 软件
  缓解，不能替代硬件急停。
