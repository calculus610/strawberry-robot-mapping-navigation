[English](README.md) | [简体中文](README.zh-CN.md)

# 草莓采摘机器人建图与导航系统

本仓库保存了草莓采摘机器人项目中 Wheeltec R680 阿克曼底盘的 ROS 2 Humble 建图与导航开发栈，属于**规范化整理后的 Mac 端恢复状态**。它整合了 Livox MID-360 传感器链路、建图、Nav2 导航和项目自有的自主探索逻辑；**不能将其视为已经与机器人正式运行时完成对账的副本**。

## 项目概览

系统以 FAST-LIO 提供激光惯性里程计，将 MID-360 数据投影为二维激光扫描供 slam_toolbox 建图，并通过 Nav2 规划和控制阿克曼底盘。项目还整合了启动与退出生命周期、地图自动保存和软件层停车保护。仓库包含上游软件与项目集成成果，并不表示所有组件都由本项目从零编写。

## 系统架构

```text
Livox MID-360 → Livox 驱动 → FAST-LIO → 二维扫描投影/转发 → slam_toolbox 地图
                                    ↘ 里程计 + 地图 → Nav2 → Wheeltec 阿克曼底盘
前沿 → ackermann_explore 候选观察位 → Nav2 全局规划器路径
     → 按阿克曼约束评估路径 → 导航目标
```

Nav2 的 Smac Hybrid-A* 全局规划器使用 Reeds-Shepp 运动模型生成路径，MPPI 控制器采用 Ackermann 模型。`ackermann_explore` **不实现** `Hybrid-A*`；它生成候选观察位，并依据行驶距离、倒车和转向等指标评估 Nav2 规划器返回的路径。

## 核心能力

- Wheeltec R680 阿克曼底盘与 Livox MID-360 集成。
- FAST-LIO 里程计、二维扫描投影和 slam_toolbox 建图。
- Nav2 Smac Hybrid-A* / Reeds-Shepp 全局规划与 MPPI Ackermann 控制。
- 面向阿克曼约束的前沿探索、候选路径检查和导航目标选择。
- 周期性及退出时的地图保存尝试，以及启动与运行时生命周期协调。
- 底盘速度指令看门狗、退出时零速度保护和探索结束停车尝试。这些都是**尽力而为的软件缓解措施**，不是经过认证的紧急停车保证。

## 仓库结构

- `src/`：ROS 2 软件包，包括上游/引入的源码、本地补丁、系统集成和项目自有实现。
- `docs/`：启动、架构、工作空间与系统交接文档。
- `scripts/robot.sh`：历史 Mac/机器人辅助脚本；其中 Mac → 机器人的同步命令**已暂停使用**。
- `README.md`：对应的英文仓库主页。

生成的 `build/`、`install/`、`log/` 目录不纳入源码版本管理。

## 运行入口——仅供对照

以下命令描述恢复后的开发启动入口，前提是在**已完成机器人端对账**的 ROS 2 Humble 工作空间中使用。它们不构成将当前 Mac 状态部署到正式机器人运行时的许可。

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py explorer_dry_run:=true start_rviz:=false
```

`explorer_dry_run:=true` 只运行一轮前沿检测、候选观察位与规划路径评估，不向 `NavigateToPose` 发送目标；它不能验证 MPPI 控制、底盘行为或现场安全。文档中的正常自主探索启动命令不带此参数，但应先完成机器人端对账和受控验证。

RViz 是可选组件。恢复后的开发 launch 默认 `start_rviz=true`；无图形界面或资源受限时可用 `start_rviz:=false`。工作空间的环境 hook 会过滤部分冲突的 Nav2 overlay 路径，自主启动入口也会检查关键 Nav2 软件包是否解析到 `/opt/ros/humble`；这些检查不代表机器人正式运行时已经完成对账。

**同步暂停：**不要将 `./scripts/robot.sh sync-dry`、`sync` 或 `sync-build` 当作当前部署步骤。必须先取回并比对机器人端独有状态。历史流程见 [REMOTE_DEVELOPMENT.md](docs/REMOTE_DEVELOPMENT.md)，**目前不获准用于部署**。

## 安全与恢复状态

- 本仓库是**规范化整理后的 Mac 端恢复状态**，尚未与机器人正式运行时全面对账。
- 机器人端 2026-08-24 的 `nav2_wheeltec_mid360.yaml`（SHA-256 前缀 `15431e20...`）和 `/home/wheeltec/run_auto_mapping.sh` 尚未取回。因此，完成机器人端对账前，Mac → 机器人同步保持暂停。
- **0.80 m** 最小转弯半径是配置层的工程假设，并非实测标定值。
- 恢复的 MPPI 时序为 **15 Hz**、`model_dt=0.07`；在 ROS 2 Humble 中会触发已知警告，control-sequence shifting 保持关闭。
- 当前阿克曼探索版本仍是实验性基线，缺少最终版本完整的端到端实机验证证据。
- 地图自动保存只是尽力尝试；PGM/YAML 文件对不是事务性原子写入，使用前应检查两份文件。
- 软件看门狗与停车逻辑不能保证物理停车，也不能替代硬件急停。

## 文档索引

- [Wheeltec MID-360 平台与运行说明](src/README_WHEELTEC_MID360.md)
- [自主建图启动说明](docs/自主建图启动说明.md)
- [手动建图与自主建图技术栈及实现原理](docs/手动建图与自主建图技术栈及实现原理.md)
- [手动建图与自主建图系统交接手册](docs/手动建图与自主建图系统交接手册.md)
- [工作空间结构与仓库规则](docs/WORKSPACE_LAYOUT.md)
- [Mac/机器人远程开发历史流程——同步暂停](docs/REMOTE_DEVELOPMENT.md)

## 第三方软件与成果归属

本仓库同时包含**引入的上游软件**（如 FAST-LIO、Livox 驱动、m-explore、Wheeltec 底盘驱动及相关依赖）、**带有本地项目补丁的上游软件**、**项目配置与系统集成**，以及 `ackermann_explore` 等**项目自有实现**。Nav2 和 slam_toolbox 是在本项目中集成、配置的上游 ROS 组件，不属于项目从零实现的软件。具体组件背景可参阅各软件包历史及技术文档；本 README 不宣称已完成逐项来源清单。
