# 原前180手动建图继承与优化

本目录以机器人实际部署的 `/home/wheeltec/manual_native180_20261022` 为基线。
基线源码与现场逐文件一致，备份标签 `native180-original-20261106`，完整备份信息见 `baseline_verified_manifest.json`。
原版前端约定保存在 `ORIGINAL_FRONTEND_CONTRACT.md`。

**默认Stage0保留原版行为。V1～V3为独立优化入口，精度尚不能认定与原版等同**：同一实际静止录包、原生字节约10Hz对照，原版最大估计位移约1.7cm，V1约25.6cm、V2约9.6cm、V3约10.9cm；同长包慢速回放V3曾达1.8m。接口与保存通过不代表定位精度通过；完整记录见 `verification/VALIDATION.md`。

## 保持的主链与接口

完整帧同步/IMU去畸变后按原安装对应的机体+X裁剪前180°。匹配体素0.20m、原生树体素0.10m。
最终障碍仅来源于原生ikd-tree当前快照；仍为固定地面参考下0.10～1.00m、5cm二维格、九层射线证据、黑格优先。
`/map` 的坐标系为 `native_projection`，位姿来自 `/Odometry`。原有三维服务 `/native_map_save`、会话监督进程与二维/三维/空闲证据保存均保留。

## 2026-11-09 V1 单变量基线

新部署目录 `/home/wheeltec/manual_native180_v1_3m_20261109` 仅取消额外的70%抽样：共享 `optimization.retention_percent=100`，保留3m、前180°、每3点预处理、0.20m匹配体素、0.10m原生树和0.05m投影格。
新V1入口为 `start_native_v1_3m.sh`。此目录内V2/V3也继承100%，但本轮仅验收V1，不能据此宣布V2/V3稳定。
机器人原目录 `/home/wheeltec/manual_native180_restored_20261106` 及其三条旧启动指令保留，仍为70%。旧V1脚本名称不用于新发布。
新启动器保存安装配置和有效阶段、地图输出路径、保留比例等会话信息。现场启动器已有 `ROS_LOCALHOST_ONLY=1`，新部署继承此值；本轮未调整网络或Web控制逻辑。
本轮验证和回滚记录见 `verification/V1_3M_20261109.md`。 100%版本未在三轮中全部达到5cm目标，因此本轮只建立可复现基线，不声明稳定版，也不继续修改其它参数。下文的70%数值和既有误差属于11月6日历史版本，不是新V1实测。

## 历史版本入口（2026-11-06）

在机器人新隔离目录 `/home/wheeltec/manual_native180_restored_20261106` 下：

- `start_native_v0_original.sh`：关闭所有新增过滤，作为原版行为对照。
- `start_native_v1_70pct_3m.sh`：前向裁剪后、0.20m匹配体素之前，选取光心3m内的有限点并按输入顺序均匀保留floor(N*70/100)。

- `start_native_v2_smooth.sh`：在V1基础上增加局部加权机体竖直平面拟合与保守极端孤点过滤，仍在匹配前处理。
- `start_native_v3_temporal.sh`：完整版本；在V2基础上，用连续真实背景射线证据清理同一原生树中的旧点。首次建树和逐帧入树均保持原版节奏，不延迟定位所需的几何支持。

平滑沿机体XY方向修正，每点最多15mm，保留原点其余字段和机体高度；低支持度、墙角、细杆等条件不足时保留原位。隔离判定使用抽样前的原始有限前向邻点，含3m外近邻，但这些外部点不进入匹配。
合成墙RMS 5.44mm降为2.04mm；墙角、96mm缝、16mm细杆测试通过。此为合成几何证据，不能替代倾斜车体和实际场景验收。

70%的分母是原有雷达预处理和全帧去畸变之后、前180°且3m内的有效点；不等于MID360原始全部回波的70%。
3m约束本帧参与匹配/入树的候选，不裁掉历史地图。九层free仍使用未抽样、未拟合的真实前向观测，允许3m外背景回波用于旧版空闲证据计算。

所有入口接受 `start_rviz:=false`。与原版一致，包装脚本仍检查机器人图形会话、加载隔离环境并拒绝重复传感器/建图进程。
地图仍保存到 `/home/wheeltec/maps/manual_native180/` 下每轮独立的 `native180_vN_*` 目录；后端必须使用输出的 `SESSION_DIR`。
`optimization_profile.json` 记录本轮版本。正常结束仍向本轮监督进程发送SIGINT，并等保存日志和完整结果。

最终标签中的四个入口共享同一份已验证实现；早期V1/V2标签保留作为阶段检查点。新增版本位于本目录，历史 `zzj_ws` 版本说明不作为本次前端接入依据。

需要在相同隔离目录重新编译时，加载 `/opt/ros/humble/setup.bash`、`/home/wheeltec/zzj_ws/install/local_setup.bash`，进入本目录的 `ws`，执行：

```bash
colcon build --packages-select fast_lio native_height_projection --executor sequential --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
```

使用新的会话入口前，先正常结束正在运行的建图会话；不要并行启动两个雷达/建图实例。原 `prepare_adapter.py` 是历史基线生成工具，不是新版本重建步骤。

## 时序清理与适用边界

时序模块只记录观测证据；最终三维和二维数据源仍是同一原生树，没有第二张累积地图。
真实前向原始回波保护当前命中和遮挡；对3m内旧点，至少4帧且跨越0.3秒的真实射线必须经过旧点2cm以内，且距离当前回波端点留出15cm，才允许精确删除。
单帧射线预算2048，旋转抽取；匹配约束不足、IMU缺失、时间回退、过长间隔或运动超限时中断清理证据。
清理只在可靠匹配、估计平移速度不超过0.05m/s、角速度不超过0.10rad/s时进行。初始建树和每帧新增点遵循原逻辑，因此行人可能暂时入图，离开后也必须获得真实背景覆盖才会清除。持续站立、被遮挡或无射线穿越的点会保留；没有按时间强制擦除。
这是建图残影处理，不是语义行人检测或底盘避障控制。5cm栅格仍会产生离散台阶，局部平滑无法保证消除全部锯齿。原生空树边界沿用原版发布/投影处理，尚无端到端全树清空场景验收。

## 算法依据

平滑采用局部加权一阶拟合，结合邻域统计的极端孤点筛选；是参考 [PCL MLS](https://pointclouds.org/documentation/tutorials/resampling.html) 与 [PCL SOR](https://pointclouds.org/documentation/tutorials/statistical_outlier.html) 思路的独立C++实现，未直接调用完整MLS高阶重采样。结合室内墙面选择限制XY修正的竖直局部平面，不跨门洞拼接全局直线，不补造遮挡后的点。
时序清理参考 [OctoMap](https://github.com/OctoMap/octomap) 的命中/空闲/未知语义及射线遍历；[DUFOMap](https://github.com/KTH-RPL/dufomap) 是动态地图对照候选。当前没有引入新的SLAM、OctoMap树或DUFOMap地图，不改变原生地图归属。

## 验证与回滚

验证详情和实际限制见 `verification/VALIDATION.md`，机器可读结果见 `verification/results/`。所有离线ROS验证使用独立DDS域、关闭硬件雷达启动、没有发送运动指令。
回滚：正常结束新版本会话后，执行原入口 `/home/wheeltec/manual_native180_20261022/start_manual_native180.sh`。不删除地图、不修改原版源码或底盘/导航服务。

## 补充现场静态录包

三轮新V1均达到本静止录包5cm首轮目标。 历史录包仍存在米级失稳，尚未通过行驶及动态场景验收。详见 `verification/FRESH_STATIC_SCENE_20261109.md`。
