# 草莓架 OctoMap 观测合同

## 输入和坐标

[观测桥](../ws/src/strawberry_octomap/scripts/observation_bridge.py)订阅以下实验主题：

| 主题 | 消息 | 原始 `frame_id` | 解释 |
| --- | --- | --- | --- |
| `/strawberry_octomap/fastlio/free_scan` | `PointCloud2` | `native_scan_lidar` | 主线预处理、去畸变与前向裁剪后的真实端点，坐标在扫描末雷达系 |
| `/strawberry_octomap/fastlio/free_scan_pose` | `PoseStamped` | `camera_init` | 同戳扫描末雷达光心的位置与旋转 |
| `/strawberry_octomap/fastlio/native_height_reference` | `PoseStamped` | `camera_init` | 主线静止初始化确定的地面原点和地面坐标系朝向 |

观测桥收到首个有效地面参考后，发布静态 `camera_init → strawberry_ground`。每组端点与位姿按完整纳秒戳匹配。对已配对的观测，先发布同戳动态 `camera_init → strawberry_lidar`，再将点云的 `frame_id` 改为 `strawberry_lidar`，原时间戳和端点坐标原样保留。后端从 TF 获取 `strawberry_ground ← strawberry_lidar`，因此点和射线光心使用同一个变换。两个实验子坐标均以 `camera_init` 为父坐标。TF 和点云分别传输，先调用 TF 发布并不保证订阅端先收到 TF，后端的消息过滤器须按点云时间戳等待变换。

输出 `/strawberry_octomap/cloud_in` 使用可靠传输、深度 2。点云和光心输入使用 Best Effort、深度 2，固定参考使用可靠、Transient Local、深度 1。扫描末光心是整帧射线的代理，尤其在移动时需要区分它和逐点采集光心。桥接层保留主线已有过滤结果，不再抽点、改高度、变造端点或填补未返回方向。

## 配对和诊断

- 桥接缓存合计最多 8 个未配对时间戳和 8 MiB 点云载荷。单帧最多 20 万点，等待配对超过 1 秒、缓存超限或已转发更晚配对时丢弃旧条目。该上限覆盖桥接缓存，DDS 队列和正在处理的消息另计。
- 无参考时丢弃观测。拒绝错误坐标系、非法时间戳、各输入流非单调时间戳、点云布局错误及非有限或无效的位姿。桥检查 `float32 XYZ` 字段和载荷布局，非有限点由 OctoMap 的 PCL 输入过滤处理。固定参考的重复发布可以使用不同时间戳。如果后续有效参考的平移或旋转发生变化，桥记录 `OBSERVATION_REFERENCE_CHANGED`，清空待配对缓存并以非零状态退出。启动器据此结束本次建图，需重新开始独立会话。
- `OBSERVATION_READY` 表示首个有效配对已转发，后端可独立报告是否已经完成建图。`OBSERVATION_STATUS` 约每 10 秒报告状态、最近转发时间戳、缓存和分类丢弃计数。两条 Best Effort 输入若同时漏掉同一帧，桥接层无法仅凭配对计数发现。

## 首版占据与投影

使用标准 `octomap_server`。`filter_ground_plane=false`，点云在地面坐标系中融合，应用投影带之前保留有效 Z 范围内的三维端点与射线。已分类的占据体素只有与地面相对高度 `[0.10, 1.25) m` 相交才参与二维占据投影。同一 XY 柱有带内占据即黑，带内相关体素均已知自由才白，其余灰。架下射线可更新其经过的三维体素。

这个版本使用标准 OctoMap 从光心到真实回波的射线语义。它沿光心至回波前更新自由，按配置量程处理超距真实回波。现有 FAST-LIO 预处理近距阈值仍用于过滤端点。旧投影器另外使用的 0.50 m 自由段起点和 0.10 m 回波前保护不进入本版 OctoMap。由此需要验证近距体素标白，以及擦边射线是否会削弱 4 cm 杆或桌板占据。未返回方向、遮挡和车体驶过均不生成额外自由射线。桥接与投影也不自动拟合地面、填满架体外接矩形或保护历史命中时间戳。 [OctoMap 2.3.1 输入和更新](https://github.com/OctoMap/octomap_mapping/blob/2.3.1/octomap_server/src/octomap_server.cpp#L374)

## 现场判定

由用户检查 55 cm 台面和 4 cm 支撑在架下背景穿越时的保留、65–110 cm 架体的二维覆盖、已知空通道的黑点和灰格、人离开且重新获得有效背景后的更新。实际地面进入 0.10 m 高度带、近距未知被当作自由、真实细杆或台面因穿越丢失，都应作为反例保留。架体没有回波支持的区域可能保持灰色或形成缺口，整片禁行覆盖须与实测边界对照确认。50–100 m² 完整会话还需实测预览时效和包括 FAST-LIO、OctoMap、投影、预览及保存的新增内存峰值。当前说明和代码未经过现场运行验收。
