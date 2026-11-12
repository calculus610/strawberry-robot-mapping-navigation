# zzj_ws

## 当前手动建图：FAST-LIO + OctoMap（2026-11-12）

当前独立工程位于 **[strawberry_octomap_20261112](strawberry_octomap_20261112/README.md)**。
Web 对接同事请直接阅读 **[手动建图系统原理与 Web 接口说明](strawberry_octomap_20261112/docs/手动建图系统原理与Web接口说明.md)**，其中包含启动指令、地图保存位置、ROS topic/service/TF、QoS、会话状态、录包与内存记录。

这份源码快照对应机器人当前 5 cm / hit 0.75 / miss 0.45 / 0.30 m 盲区配置。保留 FAST-LIO 定位，使用 OctoMap 在线更新并投影二维图；停止只冻结保存，不运行旧版离线优化。新增提交只包含源码、配置、文档及来源哈希，不上传建图数据。

## 历史实车算法快照（2026-11-11）

此前手动建图全链位于 **[current_mapping_20261111](current_mapping_20261111/README.md)**：
原版 FAST-LIO 与高度投影、A005/C/D 后处理、旧占据撤销候选、新的一键实车入口，以及 Livox 驱动和 SDK2 源码。
快照来自机器人实际部署，逐文件哈希及依赖版本随附；本次不上传地图、点云、观测记录或现场数据。
离机分析从 [全流程算法说明](current_mapping_20261111/docs/ALGORITHM_CHAIN.md) 开始。
根部 `native180/` 是历史 V1/3 m 实验，不能替代当前快照中的 `manual_native180_20261022`。

以下保留原工作空间的通用说明。

ROS 2 Humble workspace for the Wheeltec MID360 robot driver and bringup source.

Version-controlled content is kept in `src/` plus workspace-level documentation and helper files.

Generated build outputs are intentionally ignored:

- `build/`
- `install/`
- `log/`

The robot must provide the official Nav2 1.1.20 runtime:

```bash
sudo apt update
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-pointcloud-to-laserscan
```

The Wheeltec chassis driver also requires the `serial` source package kept at
`src/depend/serial_ros2`; it is pinned to upstream commit
`def526d378501e0d19d0a8724971688a3b174080` because ROS Humble does not provide
the compatible legacy `serial` API as an APT package.

Do not add this workspace to the shared robot account's `.bashrc`. The helper
always builds in a clean shell containing only `/opt/ros/humble` as the
underlay:

```bash
./scripts/robot.sh sync-dry
./scripts/robot.sh sync
./scripts/robot.sh build
./scripts/robot.sh shell
```

`shell` opens the robot workspace with `/opt/ros/humble/setup.bash` and
`~/zzj_ws/install/setup.bash` already sourced, without reading `.bashrc`.
Start autonomous exploration from that shell with:

```bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py
```

Sourcing `~/zzj_ws/install/setup.bash` activates a package environment hook
which removes only duplicate Nav2/DWB paths inherited from other workspaces.
It keeps the navigation, Web, and Autoware teams' non-Nav2 packages available.
The autonomous launch then validates that `nav2_bringup` and
`nav2_mppi_controller` resolve to `/opt/ros/humble` before starting any robot
process.

Autonomous exploration uses the `ackermann_explore` executable by default.
It waits until the Nav2 controller, planner, and BT navigator lifecycle nodes
are all active before evaluating or sending a frontier goal. A temporarily
rejected goal is retried without blacklisting the frontier, and automatic
completion is disabled for the first 60 seconds after the start pose is
recorded.
Before the first motion test, run one selection cycle without sending a
navigation goal:

```bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py \
  explorer_dry_run:=true
```

The mapping launches reuse the shared `robot-base.service` by default and do
not start a second chassis driver. The FAST-LIO mapping and Nav2 chain uses the
private `fastlio_base_footprint` frame so the shared EKF can continue owning
`odom_combined -> base_footprint` without creating a multiple-parent TF tree.
Only use `start_base:=true` after the shared chassis service has been stopped
intentionally.

If the service reports `active` but `/wheeltec_robot` or `/cmd_vel` is missing,
restart and verify the complete chassis service from the Mac with:

```bash
./scripts/robot.sh restart-base
```

This may ask for the robot account's sudo password. It uses `systemctl restart`
so any stale chassis process is stopped as part of the service cgroup before a
new driver is started; it does not kill unrelated ROS processes by name.

For a manual robot-side terminal, source this workspace and launch normally:

```bash
cd ~/zzj_ws
source install/setup.bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py
```

For the Mac-edit/robot-build SSH workflow, see
[`docs/REMOTE_DEVELOPMENT.md`](docs/REMOTE_DEVELOPMENT.md). The helper script is
`scripts/robot.sh`; it keeps generated ROS build directories on the Ubuntu robot
and syncs source changes incrementally from the Mac.
