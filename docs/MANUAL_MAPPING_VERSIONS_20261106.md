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

## V3：时序准入与可见空闲清理

启动 `./scripts/start_manual_v3_temporal.sh`，地图自动存入 `~/maps/manual_mapping_v3`。V1/V2脚本仍可运行。每个5cm三维体素至少获得5帧、跨1秒的命中才准入；短暂停留可能被延迟准入。已准入点只有在后续至少4帧、跨0.3秒的明确空闲穿越后才移除。没有射线/被遮挡/不在范围内不计空闲，未知不会被当作free；不通过TTL删除静态地图。原始未平滑、未70%抽样回波用于3D射线与HIT保护，拟合后的点仅用于地图写入。

射线基于当前估计光心。3m外真实回波可证明3m内的空闲，但不会在3m外或3m截断处创建障碍。最近角度回波、实际命中体素的停止格、端点15cm缓冲和相邻体素保护减少掠射误删；同帧HIT优先、同一体素每帧只计一次。新版本默认仅在光心速度<=0.05m/s、角速度<=0.10rad/s、帧间隔<=0.5s时清理，因为去畸变终点仍用扫描末光心近似逐束起点。运动时仍记录静态观测。停止后有有效背景观测才清理残影。

开源选型：采用 [OctoMap](https://github.com/OctoMap/octomap) 的hit/free/unknown和命中优先语义，配合 [Amanatides–Woo 3D遍历](https://www.eecs.yorku.ca/~amana/research/grid.pdf)，以NumPy独立实现稀疏地图与批量射线；不是直接安装新的SLAM或动态感知框架。[DUFOMap](https://github.com/KTH-RPL/dufomap)（BSD-3-Clause）提供保守自由空间思路，作为后续离线基准候选。[FreeDOM](https://github.com/LC-Robotics/FreeDOM)（MIT）官方ROS1实现需要移植，当前不引入。[Removert](https://github.com/gisbi-kim/removert)上游未明确许可证，[ERASOR](https://github.com/LimHyungTae/ERASOR)（GPLv3）依赖地面高度先验，未直接引入当前Humble室内近距离链路。

边界：全程静止的人员、停留超过准入门槛后没有背景穿越观测的人员，无法仅靠此方法保证排除。此模块用于建图残影清理，不做人类语义识别或底盘避障控制；实时安全障碍源由原导航系统负责，不能改接经过人员删除的地图替代实时障碍检测。

最小对照实验：同一合成墙/人员轨迹只切动态开关，分别覆盖移动人、停留后离开、遮挡、永久静止、不同高度、3m外背景、同帧命中顺序、时间倒退及容量。支持证据须同时满足残影清除和静态墙/薄杆保留；只清掉更多点不是成功。真实录包若无配准/真值，仅用于输入分布和耗时测量，不能据此宣称人物准确率。

## 实机验证与运行限制

- ROS2 Humble / Jetson原生构建成功。37项算法测试通过（后续射线预算回归结果另记）。三个阶段分别通过实际ROS点云→过滤→高度投影测试；25帧合成输入均逐帧验证70%与3m，V3合成1000个有效点的处理P95约47.2ms。
- 122帧真实雷达录包和2433条IMU经保留点时间的CustomMsg适配后驱动本次C++ FAST-LIO，输出119组点云、光心和TF精确时间戳交集。光心有119个不同估计值。Python消息构造使回放累计延迟约30.5s，因此这项仅验证实际接口与数据配对，不是1×实时性能验收。
- 对60帧真实原始输入做离线分布/耗时测试：采样计数和范围错误均为0；V2拟合12212点、移除819个极端孤点。未经FAST-LIO质量和盲区筛选的数据不能作为真实建图真值；该测试使用身份位姿，动态清理明确关闭。
- 同样的约6656回波合成静态房间，未限制射线的清理P95约195ms，超过10Hz单帧预算。这一观察触发单变量性能实验：只限制每帧用于清理的射线数量，原始命中/遮挡保护保留全量。

上述验证证明软件合同和所列场景通过，尚未完成带行人/货架真值的完整路线精度与召回验收。5cm输出栅格仍可能显示斜墙阶梯；静止行人和被遮挡残影可能保留。

最终射线预算A/B：同6656回波、同种子、24帧，`max_clear_rays=0` 与 `2048` 的实机清理P95分别为172.8ms、92.1ms；两组地图8932体素、静态移除0。默认2048只作用于空闲遍历，完整raw命中保护不抽样，跨帧轮换射线覆盖，代价是残影清除可能延后。单条支持射线的合成反例从0.6s清理延后到1.3s仍能完成。最终40/40算法测试通过，V3 ROS合成集成处理P95约44.9ms。完整SLAM/画图/大地图的10Hz实时预算仍取决于场景，不据此保证。

## 版本与备份清单

| 版本 | Git标签 | Git分支 | 工作区启动脚本 |
|---|---|---|---|
| 原始现场基线 | `manual-baseline-20261106` | `backup/manual-baseline-20261106` | 原 `src/my_robot_bringup_fastlio/scripts/start_manual_height_projection.sh`；新工作区也有 `scripts/start_manual_baseline.sh` |
| V1 | `manual-v1-70pct-3m-20261106` | `manual/20261106-s1-70pct-3m` | `scripts/start_manual_v1_70pct_3m.sh` |
| V2 | `manual-v2-smooth-20261106` | `manual/20261106-s2-smooth` | `scripts/start_manual_v2_smooth.sh` |
| V3 | `manual-v3-temporal-20261106` | `manual/20261106-s3-temporal` | `scripts/start_manual_v3_temporal.sh` |

远端为 `https://github.com/calculus610/strawberry-robot-mapping-navigation.git`。各版本使用独立分支和标签，没有强推main。机器人 `/home/wheeltec/zzj_ws_backups/20261106_manual_versions/` 保存完整基线、baseline/v1/v2/v3增量Git bundle及测试输出；bundle按基线→V1→V2→V3依赖顺序恢复，Git仓库中也已导入对应引用。`SHA256SUMS` 用于本地备份校验。工作区部署为最新源码，原Git当前分支与用户未提交工作保留。

基线源码提交 `a697ac2`，V1源码标签提交 `3ad5849`（分支后续提交 `6ddd566`仅增加实机结果），V2提交 `1e9bd27`；V3以同名标签解析。最终源码改动限于FAST-LIO可选光心输出、my_robot_bringup_fastlio新增输出处理模块/配置/启动入口、版本脚本、测试和本说明。

现场既有导航/雷达会话未被本次测试停止。新版本已部署但未代替现有硬件会话自动启动。切换时先通过原启动器结束其会话，再执行对应脚本。回放测试产生的FAST-LIO文本日志单独归档，原文本日志已恢复。
