# Wheeltec MID360 小车自动建图问题排查与解决报告

日期：2026年7月15日  
项目目录：`~/zzj_ws`  
目标：将旧车 FAST-LIO、slam_toolbox、Nav2 和 explore_lite 自动建图功能迁移到新底盘。

> 2026年7月22日更新：本文保留问题发生时的历史背景，但临时的
> `setup_zzj.bash` 和工作区 `vendor` 已移除。当前环境使用官方 Nav2 1.1.20，
> 启动前依次执行 `source /opt/ros/humble/setup.bash` 和
> `source ~/zzj_ws/install/setup.bash`。

## 一、最终结论

本次无法自动建图并不是单一故障，而是 TF、Nav2版本、车辆运动模型、代价地图、重复节点以及底盘串口等多个问题叠加造成。

目前已确认：

- MID360 点云数据正常
- `/scan` 已恢复为单一发布者
- FAST-LIO 位姿链路能够工作
- slam_toolbox 可以发布 `/map`
- Nav2 各生命周期节点能够进入 `active`
- MPPI 使用 `Ackermann` 运动模型
- explore_lite 能生成探索边界
- 底盘串口上行反馈恢复
- `/odom` 稳定约为 `20 Hz`
- 底盘能够接收 `/cmd_vel`
- RViz 已显示二维地图、激光数据和探索边界点

当前系统已经具备自动探索建图的完整数据链路，后续主要工作是现场运动参数调优和长时间稳定性验证。

---

## 二、问题与解决方法汇总

| 序号 | 遇到的问题 | 根本原因 | 解决方法 | 验证结果 |
|---|---|---|---|---|
| 1 | 临时环境脚本命令曾因中文弯引号失败 | Bash 将弯引号当作文件名的一部分 | 临时脚本现已移除，改为加载 `/opt/ros/humble/setup.bash` 和 `install/setup.bash` | 标准 ROS 环境脚本可以正常加载 |
| 2 | ROS 2 包和配置可能加载到旧工作区版本 | `wheeltec_ros2` 与 `zzj_ws` 中存在重复的 Nav2、底盘驱动等包，环境覆盖顺序错误 | 在不读取 `.bashrc` 的隔离 shell 中从 `/opt/ros/humble` 单一 underlay 重建 | 当前配置和可执行文件能从 `zzj_ws` 加载，Nav2 来自 `/opt/ros/humble` |
| 3 | `body` 与 `base_footprint` 不在同一 TF 树 | FAST-LIO 发布 `camera_init → body`，底盘和 Nav2 使用 `base_footprint`，二者缺少连接 | 增加 `body → base_footprint` 静态 TF，并将偏移量设置为可配置启动参数 | TF 可以形成 `map → camera_init → body → base_footprint` 完整链路 |
| 4 | 对 `camera_init`、`body` 坐标系含义不清楚 | 迁移时混用了 FAST-LIO 坐标系和底盘坐标系 | 明确：`camera_init` 是 FAST-LIO 局部连续里程计坐标系；`body` 是 FAST-LIO IMU/雷达机体坐标系；`base_footprint` 是底盘在地面的投影坐标系 | Nav2、slam_toolbox 和点云转换统一使用正确坐标系 |
| 5 | RViz 初始只有网格或所有内容挤在中心 | `map`、TF 尚未发布，Fixed Frame 设置为 `map`；同时显示比例过小 | 修复 TF 和地图链路；RViz 使用 `map` 作为 Fixed Frame，通过 `F` 聚焦或滚轮调整比例 | 当前 RViz 已能显示地图、激光和机器人坐标轴 |
| 6 | `/scan` 出现两个发布者 | 遗留的 `scan_qos_relay` 进程脱离原启动终端后仍在后台运行，新启动又创建了一个 | 定位并终止遗留 PID，只保留当前启动生成的 `scan_qos_relay` | `/scan Publisher count: 1` |
| 7 | Nav2 节点 active，但小车不能自动移动 | 节点 active 只代表服务器运行，不代表控制器正在获得目标或底盘真正执行速度 | 分别检查生命周期、`/cmd_vel` 发布者/订阅者、底盘实际发送日志和 `/odom` | 成功区分导航问题和底盘问题 |
| 8 | 原 DWB 控制器不适合新车 | 新车为阿克曼转向底盘，不能像差速车一样原地旋转，DWB轨迹和恢复动作不匹配 | 将局部控制器改为 Nav2 MPPI，设置 `motion_model: Ackermann` 和 `min_turning_r: 0.80` | `FollowPath.motion_model` 返回 `Ackermann` |
| 9 | MPPI 加载存在版本/ABI冲突 | `wheeltec_ros2` 中的旧版 Nav2 库与 `/opt/ros/humble` 当前 Nav2 版本不兼容 | 通过 APT 安装官方 Nav2 MPPI 1.1.20，并从干净 `/opt/ros/humble` underlay 重建 | MPPI及各 Critic能够正常创建和激活 |
| 10 | 阿克曼底盘恢复行为可能要求原地旋转 | Nav2默认行为树包含 `Spin`，阿克曼底盘无法完成原地旋转 | 新增无 `Spin` 的 NavigateToPose 和 NavigateThroughPoses 行为树，保留清图、等待和后退恢复 | 避免因原地旋转能力不足导致恢复失败 |
| 11 | 多次出现 TF 延迟、外推错误 | FAST-LIO、slam_toolbox、Nav2 和 explore_lite 的时间处理存在约1秒延迟，而原容差仅0.1～0.5秒 | 将相关 `transform_tolerance` 和 `transform_timeout` 调整为2秒 | TF查询稳定性提高 |
| 12 | 局部代价地图提示雷达原点超出高度范围 | VoxelLayer原始范围无法覆盖姿态变化后的雷达原点 | 设置 `origin_z: -0.5`、`z_resolution: 0.10`、`z_voxels: 16`，覆盖约 `-0.5～1.1 m` | 雷达原点落入有效体素范围 |
| 13 | 中间设置 `z_voxels: 32` 后出现错误 | Nav2 VoxelLayer最多支持16个Z层 | 保持16层，通过增大 `z_resolution` 扩大高度范围 | 消除 `only support up to 16 z values` 错误 |
| 14 | explore_lite启动后很快停止 | 系统刚启动时地图和TF尚未稳定，explore_lite过早判断边界不可达 | 启动延迟由18秒增加到30秒；移动进度允许时间调整为20秒 | explore_lite能够在地图稳定后再生成目标 |
| 15 | `/cmd_vel` 曾经有数据，但小车完全不动 | 工作区把底盘串口错误地改成了 Jetson 板载串口 `/dev/ttyTHS1` | 对比原厂启动文件和实际设备，将端口恢复为 `/dev/wheeltec_controller` | 正确设备实际指向 `/dev/ttyCH343USB1` |
| 16 | 底盘驱动打开串口，但 `/odom` 完全没有数据 | 驱动连接错端口，因此收不到底盘24字节反馈帧 | 从两个串口分别采样原始数据，确认 `/dev/ttyCH343USB1` 持续返回 `7B ... 7D` 帧 | 找到正确底盘串口 |
| 17 | Nav2以10～20 Hz发布速度，但底盘回调处理不及时 | 串口读取超时原为2000 ms；没有数据时会阻塞ROS回调，导致下位机看门狗停止车辆 | 串口读取超时改为20 ms | `/cmd_vel` 可以及时进入底盘回调 |
| 18 | 超时改为20 ms后仍无 `/odom` | 无数据超时时，旧代码仍将无效字节写入帧解析器，破坏 `7B ... 7D` 帧同步 | 检查 `Stm32_Serial.read()` 返回长度，未读到字节时立即返回，不更新解析状态 | `/odom` 恢复为约20 Hz |
| 19 | 难以判断 Nav2是否真正向下位机发送速度 | 只能看到 `/cmd_vel` 话题，无法确认底盘回调和串口发送函数是否执行 | 在底盘回调中增加每2秒一次的限频日志，输出实际发送的线速度和角速度 | 日志曾连续显示 `linear.x=0.250`、`angular.z≈0.275` |
| 20 | `/cmd_vel` 后来显示低频或停止发布 | explore_lite多次尝试后，小车没有产生位移，Nav2报 `Failed to make progress`，最终取消目标 | 修复底盘端口和里程计后重新启动整套系统 | 证明低频是探索停止后的结果，不是最初原因 |
| 21 | 嵌入式主机同时运行 RViz 时资源占用较高 | FAST-LIO、点云转换、slam_toolbox、Nav2和RViz同时运行，CPU负载较高 | 自主启动时默认关闭RViz，需要时用 `start_rviz:=true` 或单独启动 | 降低启动阶段资源竞争 |
| 22 | RViz地图边缘出现绿色球 | explore_lite发布的探索边界候选点 `/explore/frontiers` | 明确其为正常的 Frontier Marker，不是障碍物；不需要时可关闭 `Frontier Markers` | 当前显示符合自动探索逻辑 |
| 23 | 不清楚手动建图、自动建图和保存地图方式 | 原启动流程分散，容易重复启动底盘、雷达或转换节点 | 增加统一自动探索、手动建图启动入口，并在README中整理操作步骤 | 后续开机可按固定流程执行 |

---

## 三、关键配置调整

### 1. Nav2控制器

当前采用：

```yaml
plugin: nav2_mppi_controller::MPPIController
motion_model: Ackermann

AckermannConstraints:
  min_turning_r: 0.80
```

同时设置：

```yaml
vx_max: 0.25
vx_min: -0.15
wz_max: 0.30
movement_time_allowance: 20.0
required_movement_radius: 0.15
```

主要配置文件：[nav2_wheeltec_mid360.yaml](/home/wheeltec/zzj_ws/src/my_robot_bringup_fastlio/config/nav2_wheeltec_mid360.yaml)

### 2. TF结构

最终目标结构为：

```text
map
└── camera_init
    └── body
        └── base_footprint
```

- `map → camera_init`：slam_toolbox
- `camera_init → body`：FAST-LIO
- `body → base_footprint`：静态TF

### 3. 底盘串口

错误配置：

```text
/dev/ttyTHS1
```

正确配置：

```text
/dev/wheeltec_controller
└── /dev/ttyCH343USB1
```

对应文件：[base_serial.launch.py](/home/wheeltec/zzj_ws/src/turn_on_wheeltec_robot/launch/base_serial.launch.py:46)

### 4. 串口解析器

主要修改：

- 超时从2000 ms改为20 ms
- 未读取到字节时不更新帧状态
- 增加实际下发速度日志

对应文件：[wheeltec_robot.cpp](/home/wheeltec/zzj_ws/src/turn_on_wheeltec_robot/src/wheeltec_robot.cpp:236)

---

## 四、最终验证结果

已完成的实测验证：

```text
/controller_server: active
/planner_server: active
/bt_navigator: active
```

```text
FollowPath.motion_model = Ackermann
```

```text
/scan Publisher count = 1
/map Publisher count = 1
```

底盘反馈：

```text
/odom average rate ≈ 20 Hz
```

底盘速度回调日志：

```text
send cmd_vel: linear.x=0.250 linear.y=0.000 angular.z=0.275
```

RViz验证：

- 二维占据栅格地图正常显示
- 激光点正常显示
- Navigation状态为 `active`
- explore_lite绿色边界标记正常显示

因此可以确认，传感器、定位、建图、导航、探索目标、速度指令和底盘反馈已经形成完整链路。

---

## 五、后续标准操作流程

### 自动探索建图

确保周围安全后执行：

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py
```

需要同时启动RViz：

```bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py \
  start_rviz:=true
```

### 单独启动RViz

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
rviz2 -d ~/zzj_ws/install/my_robot_bringup_fastlio/share/my_robot_bringup_fastlio/rviz/fastlio_2d_slam.rviz
```

### 保存地图

先在RViz点击 `Pause`，确认小车停止，然后执行：

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
mkdir -p ~/zzj_ws/maps

ros2 run nav2_map_server map_saver_cli \
  -f ~/zzj_ws/maps/mid360_map \
  --ros-args -p save_map_timeout:=10.0
```

生成：

```text
~/zzj_ws/maps/mid360_map.yaml
~/zzj_ws/maps/mid360_map.pgm
```

### 健康检查

```bash
ros2 topic info /scan
ros2 topic info /map
ros2 topic hz /odom
ros2 topic hz /cmd_vel
ros2 run tf2_ros tf2_echo camera_init base_footprint
```

---

## 六、后续注意事项

- `/dev/wheeltec_controller` 和 `/dev/wheeltec_mic` 当前都可能指向同一个USB串口，后续应检查udev规则，避免麦克风节点与底盘节点抢占设备。
- 自动探索停止后 `/cmd_vel` 不再发布属于正常现象。
- 不要重复启动 `base_serial.launch.py`、点云转激光或 `scan_qos_relay`，否则会出现重复话题和串口占用。
- 不要将 `z_voxels` 设置为大于16。
- 阿克曼底盘不能原地旋转，不应恢复默认包含 `Spin` 的行为树。
- `min_turning_r: 0.80`、最大速度和障碍膨胀半径仍需根据实际场地进行调优。
- 当前工作区存在尚未提交的修改，建议完成现场测试后建立Git提交，保留本次可运行版本。

核心说明和开机操作已整理在 [README_WHEELTEC_MID360.md](/home/wheeltec/zzj_ws/src/README_WHEELTEC_MID360.md:57)。
