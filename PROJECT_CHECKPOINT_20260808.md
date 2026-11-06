# Ackermann 自主建图项目交接 Checkpoint
生成时间：2026-08-08
生成模型：GLM-5.2 (Claude Code)

## 1. 当前系统架构

MID360 雷达
-> FAST-LIO (/laser_mapping)
-> /cloud_registered_body
-> pointcloud_to_laserscan (/fastlio_scan_converter)
-> /scan_raw
-> scan_qos_relay
-> /scan
-> slam_toolbox (online_async)
-> /map
-> Nav2 (navigation_launch.py)
-> Smac Hybrid-A* (planner_server, GridBased)
-> MPPI Ackermann (controller_server, FollowPath)
-> /cmd_vel_nav -> velocity_smoother -> /cmd_vel
-> wheeltec_robot_node (robot-base.service)
-> STM32 底盘

Explorer (ackermann_explore, explore_lite 包) 位于决策层，发送 NavigateToPose 目标。

## 2. 当前运行身份

- ROS 2 Humble
- workspace: ~/zzj_ws (自主系统) + ~/wheeltec_ros2 (底盘/WHEELTEC 官方)
- robot-base.service: systemd system service, active, 运行 turn_on_wheeltec_robot.launch.py
- wheeltec_robot_node: 来自 wheeltec_ros2/src/turn_on_wheeltec_robot
- Nav2: 实际使用 /opt/ros/humble 官方二进制（env-hooks/zzj_nav2_isolation.sh 剥离 wheeltec_ros2 重复 nav2 路径）
- 主要 launch 入口: ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py start_explorer:=false start_base:=false start_rviz:=false
- start_base=false: robot-base.service 已运行底盘驱动，避免重复启动
- 自主系统包: ~/zzj_ws/src/my_robot_bringup_fastlio (launch+config+节点)
- Explorer: ~/zzj_ws/src/m-explore-ros2/explore/src/ackermann_explore.cpp (2528行)

## 3. 已正式关闭的问题

以下问题已通过实验/代码证据关闭，不应重新调查，除非出现新反证。

### manual 链
已通过。MID360 -> FAST-LIO -> /scan -> slam_toolbox -> /map 运动中稳定。

### Scan/TF 时间戳
已确认 /cloud_registered_body、/Odometry、camera_init->body TF、/scan_raw、/scan 的 header.stamp 全链一致（delta=0）。pointcloud_to_laserscan 和 scan_qos_relay 原样保留 stamp。persistent TF probe 中 scan 时刻 can_transform 0ms 100% 成功。TF 树未断开。不要重新提出"TF 树断开"或"stamp 不一致"旧假设。

### local costmap marking
已确认两个串联阻断条件并正式修复（不得自行回滚）：
1. obstacle_layer.scan.observation_persistence: 0.5 -> 0.0（scan stamp 老 1.3-2.0s > 0.5s，purgeStaleObservations 清空 buffer，marking 不执行）
2. obstacle_layer.min_obstacle_height: 默认 0.0 -> -1.0（layer 级高度过滤，纸箱点 z=-0.286 < 0.0 被 continue）

修复文件: ~/zzj_ws/src/my_robot_bringup_fastlio/config/nav2_wheeltec_mid360.yaml (行 253 + 行 259)
已 build (colcon build --packages-select my_robot_bringup_fastlio --symlink-install)
备份: ~/nav2_wheeltec_mid360.yaml.bak_expB_persist

实物纸箱验证: /scan -> LETHAL(100) + INSCRIBED(99) + inflation 正常；移除后约 0.1s 清除，无 ghost obstacle。

## 4. Nav2 实车验证结果

### 阶段 4A: 简单单目标
- NavigateToPose SUCCEEDED, 6.4s
- 实际轨迹 0.976m, 终点误差 0.030m, yaw 误差 1.2°
- linear.x max=0.220, angular.z max_abs=0.169, sign_switches=1
- 控制自然，无摆动、无撞墙、停车正常
- rosbag: ~/bags/nav2_single_goal_20260808_140952/

### 阶段 4B: 实体纸箱避障
- 机器人主动从右侧绕障
- 用户现场最近距离约 5-10cm，未碰撞
- local costmap 障碍全程存在（marking 修复有效）
- 控制自然，转向自然，停车正常
- 最终 goal 未到达（pos_error 0.795m），但不是 controller 撞墙问题
- rosbag: ~/bags/nav2_obstacle_test_20260808_142447/

## 5. 4B 未到达目标的已确认根因

初始 (t=1173s): goal cell cost=85 (inflation, 可通行), goal 到最近 /map 障碍 0.309m, start->goal CONNECTED。

约 4.9s 后 (t=1177.8s): slam_toolbox /map 在 goal 0.5m 范围内新增 10 个 occupied cells（机器人绕行时新观测写入）。goal 到最近 /map 障碍距离: 0.309m -> 0.165m。

inscribed_radius 约 0.175m。0.165m < 0.175m -> goal 进入 INSCRIBED zone。

global costmap static_layer 读取更新后的 /map -> inflation_layer 重新计算 -> goal cell cost: 85 -> 99 (INSCRIBED)。

Smac Hybrid-A* 正确拒绝 INSCRIBED goal（"no valid path found"）。BT recovery(wait) 无法恢复，连续 13 次规划失败后 bt_navigator abort "Goal failed"。NavigateToPose 最终 ABORTED。

这不是 planner bug，也不是 controller bug。是自主建图 /map 动态更新 + inflation 几何导致目标可达性动态变化的固有特性。

## 6. FAST-LIO yaw 问题

撤回此前"-0.4 度 -> 15.3 度 静止漂移 16 度"结论。

两个数据来自不可比时段（-0.4 度来自 A+B 刚启动后，15.3 度来自运行一段时间后，非同一静止时段）。

同一 bag 静止段: PRE-MOTION yaw=15.10 度 +/-0.31 度, POST-MOTION yaw=12.35 度 +/-0.16 度。仅小幅波动（range 0.7-2.3 度）+ xy 漂移 7-10cm/10s。

不得重新引用"16 度静止漂移"。

## 7. 当前 ackermann_explore 状态

基于 m-explore-ros2 发展出的本地自定义 Ackermann Explorer（非纯原版开源）。代码: ~/zzj_ws/src/m-explore-ros2/explore/src/ackermann_explore.cpp (2528 行)。

已有:
- frontier 生成 (FrontierSearch)
- observation pose candidate (standoff + inward angle sampling)
- FREE_SPACE-only candidate (knownFreeClearance, cost=0 only)
- clearance >= footprint_clearance_ (0.50m)
- ComputePathToPose 预检
- 发送前 live global costmap clearance 检查 (globalCostmapHasClearance)
- NavigateToPose 发送 + result callback
- failure handling (blacklist + retry + cooldown)

已确认缺陷:
- active NavigateToPose 发送后，没有动态 goal revalidation
- 如果 /map 更新令 goal 失效（clearance 降至 < 0.50m 或 cell 变 INSCRIBED），Explorer 只有等 Nav2 最终 ABORTED 后才知道
- 真实 4B 案例中这类等待约 82 秒（BT recovery 循环）

代码审查结论: MODIFY（非 REWRITE）。目标选择 + 失败恢复主体可保留，仅缺少 active-goal revalidation。

暂时不要实施 revalidation 修改。在修改前需先评估开源 Explorer 替代方案（见第 8 节）。

## 8. 新的 Explorer 选型方向

在继续修改 ackermann_explore 之前，需优先评估成熟开源 ROS2 Frontier Explorer 方案。

重点候选: mertgulerx/frontier_exploration_ros2

需比较它和当前 ackermann_explore:
- ROS2 Humble 兼容性
- Nav2 接口
- 是否直接控制底盘
- 是否只发送 NavigateToPose
- global/local costmap 使用
- blocked active goal 处理
- goal preemption
- clearance
- frontier 生成
- suppression/cooldown
- 与 Ackermann Smac+MPPI 的兼容性

不要因为它不是"Ackermann 专用"就直接淘汰。Explorer 负责去哪，Smac+MPPI 负责 Ackermann 约束如何去。

## 9. 当前禁止事项

下一模型未经用户明确批准不得:
- 重新调查 manual 链
- 回滚两个 costmap 修复 (min_obstacle_height=-1.0, observation_persistence=0.0)
- 调 TF
- 调 footprint
- 调 MPPI
- 调 turning radius
- 改 planner
- 直接重写 ackermann_explore
- 启动连续自主探索
- 直接用新 Explorer 控制实车

## 10. AI 协作协议

- 不要根据单条日志宣布唯一根因
- 每个工程问题遵循: 事实 -> 候选假设 -> 最小单变量验证 -> 结果 -> 用户批准 -> 修改
- 默认只读
- 修改必须明确: 文件、diff、风险等级、回滚方式
- 实车实验必须用户明确绿灯
- 如果遇到与 Checkpoint 矛盾的新证据，允许挑战 Checkpoint，但必须展示直接证据，不能凭推测推翻已关闭结论

## 附: SSH 连接信息

- IP: 192.168.8.208
- 用户名: wheeltec
- 密码: dongguan
- 平台: NVIDIA Jetson (Tegra, aarch64), Ubuntu 22.04, ROS2 Humble

## 附: rosbag 索引

- ~/bags/nav2_single_goal_20260808_140952/ (阶段4A 单目标, 40MiB)
- ~/bags/nav2_obstacle_test_20260808_142447/ (阶段4B 障碍避障, 199MiB)
- ~/ab_launch.log (A+B launch 日志)
## 11. frontier_exploration_ros2 PILOT-0

**日期**：2026-08-08  
**结论**：PILOT-NEEDS-ADAPTATION

### 1. 上游元数据
- commit `ec530d2` (2026-07-10)，branch `main`，license Apache-2.0
- 在 `~/frontier_pilot_ws/` 隔离 build 成功（仅 source `/opt/ros/humble`，不依赖 zzj_ws）
- executable：`frontier_explorer`、`frontier_exploration_ctl`

### 2. 控制接口确认
- 0 处 `/cmd_vel` / `geometry_msgs::msg::Twist`
- 纯 Nav2 NavigateToPose action client（参数 `navigate_to_pose_action_name`）
- 无差速/原地旋转/运动学模型代码

### 3. 上游相对当前 ackermann_explore 的优势
- **active-goal costmap revalidation**（`costmapCallback` + `localCostmapCallback` 每帧 `frontier_cost_status` → 实时 cancel）——这是我们当前缺失且最需要的
- **suppression**（地理区域 blacklist + attempt threshold + TTL + progress timeout）
- **preemption 防抖**（2s 最小间隔 + stable replacement candidate）
- **completion lifecycle**（默认 `return_to_start_on_complete: false`，不关闭 launch）

### 4. ⚠️ 重大安全事件
PILOT-0 中执行 `frontier_exploration_ctl start` 导致**真实 NavigateToPose 被 dispatch 并被 Nav2 accept**，小车发生了实际移动。

**上游没有原生 dry-run 机制**。`autostart=false` 保持 cold idle，但 `control_service start` 即发送真实目标。

**强制规则**：以后任何 Explorer dry-run 必须使用技术硬隔离（隔离 ROS_DOMAIN_ID、dummy action name、或不启动 Nav2），不得仅依赖文字约束。**禁止在真实 `navigate_to_pose` action 可达时执行 START，除非用户明确批准实车导航。**

### 5. 默认 goal selection 不满足当前安全基线
PILOT 采到 **4 个唯一候选**（共 6 次 dispatch，因紧急停止未完成预定采样）：

| 候选 | global cost | LETHAL 距离 | CURRENT 标准 | ComputePath |
|---|---|---|---|---|
| (0.49,-0.64) | 0 (FREE) | 0.457m | ✗ (<0.50m) | SUCCESS |
| (1.67,0.10) | 55 (INFLATION) | 0.265m | ✗ (≠FREE) | SUCCESS |
| (1.57,0.30) | 62 (INFLATION) | 0.242m | ✗ (≠FREE) | SUCCESS |
| (2.91,-3.67) | -1 (UNKNOWN) | 0.175m | ✗ (≠FREE) | ABORTED |

- **0/4 clearance ≥ 0.50m**（CURRENT 基线）
- **2/4 INFLATION 目标**
- **1/4 UNKNOWN 目标**
- **ComputePath 3/4 SUCCESS**（75%）

> 注意：样本仅 4 个且测试中机器人发生位移，不能作为总体统计，但已足以证明默认配置下存在不安全候选。

### 6. `frontier_selection_min_distance=0.8` 语义
**含义 A**：`get_frontier_snapshot` 中过滤距离机器人 < 0.8m 的 frontier（`try_send_next_goal:75`）。不是从 frontier 边界退后形成观察位。

### 7. Goal 位置来源
`frontier_policy.cpp:26-29`：`frontier.goal_point.value_or(frontier.center_point)`。`goal_point` 由 `choose_accessible_frontier_goal` 在 frontier centroid 附近的 free-space cell 中选取。**无几何 standoff**（没有 CURRENT 的 `try_candidate` standoff 0.80-1.20m）。

### 8. 当前决策：PILOT-NEEDS-ADAPTATION
- **不**原封不动替换当前 Explorer
- **暂停**继续扩展旧 `ackermann_explore`
- 目标架构：`frontier_exploration_ros2` 负责 frontier/lifecycle/preemption/suppression/revalidation；在此基础上增加薄的 **Ackermann goal safety validator**

### 9. 最小安全层设计（优先保留）
1. **UNKNOWN reject**：显式处理 cost=-1（≠ `occ_threshold` 语义）
2. **safe costmap rule**：不接受 INFLATION cell（cost>0 或 cost>极低阈值）
3. **geometry clearance**：基于真实 LETHAL/INSCRIBED 距离，≥ footprint_clearance
4. **ComputePathToPose 预检**：作为第二阶段（不立即引入异步复杂度）

> 不采用“`occ_threshold` 65→5 即可解决”的简化结论——UNKNOWN=-1 可能不会被简单阈值排除，必须确认并显式处理 unknown。

### 10. 下一步
1. 只读设计 goal validator 在 `send_frontier_goal`（`frontier_explorer_core_dispatch.cpp:874`）中的最小插入点
2. 确认 UNKNOWN/cost/threshold 语义边界
3. 暂不修改源码、不启动机器人
4. 后续 PILOT-1 必须使用隔离 ROS_DOMAIN_ID 或 dummy action name 的 dry-run
