# 手动建图分阶段版本（2026-11-06）

## 基线与范围

机器人工作区为 `/home/wheeltec/zzj_ws`。基线源码（含原有未提交修改）提交为 `a697ac2`，GitHub 分支 `backup/manual-baseline-20261106`、标签 `manual-baseline-20261106`。机器人完整文件备份为 `/home/wheeltec/zzj_ws_backups/20261106_manual_versions/baseline_full_ws.tar.gz`（不含运行日志 `log/`，包含 `.git`、源码、旧 build/install、工作区地图及旧恢复副本），SHA-256：`55ed2d03bdeafcdea7cab898cf9abdb4fa6409971d3017fc88e09875607b8482`。源码 Git 不提交运行时数据、构建产物、凭据或恢复目录。原 GitHub `main` 和机器人原分支未被重置。

本次用户明确授权备份、三阶段实现、构建、验证、独立脚本及 GitHub 推送，不需要逐步确认。输出链路调整属于 Level 2；FAST-LIO 的额外观测接口与跨模块接入按 Level 3 审视，限于本任务明确要求的点云处理。原有手动入口保留。

## 事实、解释与验证边界

- 已确认：原手动入口有 OctoMap 和较新的 `manual_mapping_height_projection.launch.py`。后者从累计 `/Laser_map` 投影；高度为 camera_init 原始 Z 的 [-0.2, 0.8] m，分辨率 0.05 m，未占据格保持未知。
- 已确认：原 `dense_publish_en=false` 使用 0.2 m VoxelGrid 质心；预处理 `point_filter_num=3`。`body` 是 IMU 原点，雷达外参在线估计。质心不应当作真实回波做自由空间清理。
- 支持的推断：独立侧路从稠密去畸变观测采样，能让比例和范围可核查；小邻域限幅拟合可减小直墙噪声；后续自由空间证据可以清除曾出现但已离开的障碍残影。
- 尚未验证的假设：在本机的所有货架/人员移动速度下，默认平滑和时序阈值均能满足目标。5 cm 格子上的斜墙本身仍会阶梯状；滤波无法补偿位姿误差。
- 缺少证据：带人员和细杆真值标注、完整路线实地验收。真实录包可以证明可运行和性能，不等于动态人员识别准确率。
- 反证条件：采样数不等于 floor(0.7N)、采样点距光心超过3m、墙角/细杆被抹去、遮挡或无回波导致清理、持续过载或时序错配，均否决相应配置。

## V1：70% 与 3 米

新配置 `mid360_manual_filtered.yaml` 保持估计器/标定设置，开启稠密去畸变观测和额外 `PoseStamped /manual_mapping/lidar_origin_body`；后者使用当前在线外参和点云相同时间戳。额外观测默认关闭，不增加 TF 边。新配置关闭旧累计 `/Laser_map` 缓存/PCD保存以避免重复积累。

`manual_cloud_filter` 先移除非有限 XYZ，再按实际雷达光心计算三维欧氏距离，保留 **<=3 m**，然后按等间隔索引取 **floor(0.70 × N)**。N 是该帧3m内的有效去畸变点数，**不是雷达所有原始回波数**；FAST-LIO 原有约1/3的预处理仍保留。新侧路使用稠密观测，因此不能把70%误解为原20cm体素输出的70%。体素累计去重、后续离群滤波和时序准入还会降低最终地图点数。

每帧使用对应时刻 `camera_init <- body` 变换，不使用 latest 替代；缺光心、TF错配/超时就丢帧并记录指标。新累计云 `/manual_mapping/filtered_map` 经原高度投影节点生成 `/manual_mapping/map`。每个5cm三维体素保留观测点，不构造新表面。容量上限500000个体素，满时拒绝新增并报告，不静默遗忘旧地图。

## 启动与保存

```bash
cd /home/wheeltec/zzj_ws
./scripts/start_manual_baseline.sh
./scripts/start_manual_v1_70pct_3m.sh
# 无图形界面：加 start_rviz:=false
```

一次只启动一个手动版本。脚本检测重复传感器/建图进程并拒绝启动，不主动结束已有会话。地图默认自动存入 `~/maps/manual_mapping_v1`，文件名带时间。旧入口原有保存路径保留。原底盘服务由现有启动器负责，本次脚本不发送速度或启动导航。

清理当前shell的其他工作区污染后构建：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/zzj_ws/install/livox_ros_driver2/local_setup.bash
cd /home/wheeltec/zzj_ws
MAKEFLAGS=-j1 colcon build --packages-select fast_lio my_robot_bringup_fastlio --executor sequential --parallel-workers 1
```

测试：`PYTHONPATH=src/my_robot_bringup_fastlio python3 -m unittest discover -s tests/manual_mapping -p 'test_*.py' -v`。ROS集成测试使用专用域和回放，不接底盘控制。检查 `/manual_mapping/filter_metrics` 的 sampled/in_range、范围、丢帧、耗时和容量。

## 回滚

运行行为回滚：结束本次新版本的启动终端，再用 `start_manual_baseline.sh` 或原有 `start_manual_height_projection.sh`。新源码中的光心输出默认关闭；基线启动配置不会启用。

源码/二进制精确回滚：先停止本次进程，逐文件比较本次版本和当前文件，确认没有用户后续更改，再从基线标签恢复本次涉及的文件；新文件仅在未被后续修改时移除。重新构建所涉及两个包。也可把完整 tar 解到新的独立目录核对，禁止直接覆盖整个工作区或使用大范围 `git reset --hard`。原备份含 build/install 中的绝对路径符号链接；在新目录运行必须重新构建，不能直接依赖旧链接。

## V1 验证结果

机器人原生构建 fast_lio 与 my_robot_bringup_fastlio 成功；原有 PCL CMake/Boost 编译提示保留。8项采样/解码/容量测试通过。隔离 ROS 域实际运行两轮合成输入，每轮25帧，1000个范围内有效点每帧恰好保留700点；收到真实PointCloud2和OccupancyGrid输出。缺少光心被拒绝；时间回退、发布者端点更换均锁定新输入，原图保留。端点检测依赖DDS发现，不是逐消息来源认证。耗时P95约8.9ms（合成1000有效点，不是整机完整SLAM负载）。结果见 docs/validation/v1_*_ros_contract.json。

## V2：保守拟合后移除极端离群点

启动 `./scripts/start_manual_v2_smooth.sh`，地图自动存入 `~/maps/manual_mapping_v2`。先执行V1，再做一阶局部加权最小二乘平面拟合，保持Z，仅对近垂直且二维邻域充分的平面限幅修正XY。12cm邻域、至少10点、两项特征值门槛和切向双侧支撑保护转角、立柱和门洞；最大位移1.5cm。投影若越过3m则保留原坐标。之后仅删除kNN距离超过均值3σ、最近邻距离>=15cm、且抽样前完整有限点云10cm内没有支撑的极端孤点。完整支撑包含3m边界外回波，但输出不超3m。世界坐标复检允许1nm浮点误差。

这是基于成熟MLS/SOR原理的独立NumPy/SciPy实现，未把PCL直接链接到新节点。PCL采用BSD-3-Clause；SciPy/NumPy由机器人现有软件包提供。参考 [PCL MLS源码](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.12.1/surface/include/pcl/surface/impl/mls.hpp)、[SOR源码](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.12.1/filters/include/pcl/filters/impl/statistical_outlier_removal.hpp)、[PCL许可证](https://github.com/PointCloudLibrary/pcl/blob/pcl-1.12.1/LICENSE.txt)。全局RANSAC直线可能跨门洞拼接，不用于这片3D墙面；高阶MLS可能圆化转角，当前选择局部一阶近似曲面。

支持证据：合成墙面的P90法向噪声7.09mm降至1.34mm，转角/细杆/门边保护和3m边界反例通过。最小实验依次为相同输入的原始、仅拟合、拟合后滤波，每轮只改变一个开关。任何真实细杆被删、门洞被补、墙面整体偏移都构成反证。真实场景无几何真值时不以视觉光滑宣称精度提升。

V2实机验证：20/20单元测试通过；隔离ROS集成25帧通过，点云和二维地图可用，合成1000有效点的处理P95为36.4ms。详见 `docs/validation/v2_stamp_ros_contract.json`。该结果不等于完整路线的几何精度或实时性能验收。
