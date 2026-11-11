# 11 月 11 日机器人建图算法阅读索引

`robot_home/` 按机器人目录组织源码、参数和入口，便于离机阅读。下面的 A 到 E 是同一次建图的二维地图阶段。现场验收需要另行保存和授权提供运行数据，仓库只归档算法及其依赖源码和配置。

## 从启动到候选地图

```text
manual_map_lab_cleanup_20261111/start_test.sh
  └─ mapping_session.py
     ├─ manual_map_postprocess_test/start_test.sh --no-preview
     │  └─ session.py
     │     ├─ recorder.py：同戳记录前向端点、光心位姿和固定地面参考
     │     └─ manual_native180_20261022/run_session.py：启动 MID-360、FAST-LIO、投影和 RViz
     │        按 Ctrl+C 后保存 A，再离线处理 A → B → C → D
     └─ run_cleanup.sh --source-session <本次 SESSION_DIR>：D 或 C → E
```

1. 最外层包装器核对原入口脚本的 SHA-256，接收子协调器返回的 `SESSION_DIR`，确认它是本次新建目录。等终端显示 `PROJECTION_READY` 后手动行驶。第一次 Ctrl+C 只转发给本次建图协调器。保存过程中额外的 Ctrl+C 不会重复转发。协调器等待投影与原生树最终保存，随后排空并封口记录器，校验保存标记、时间戳、文件刷新、哈希和地图转换，再推进 A005、冲突和散点阶段。 [新入口](../robot_home/manual_map_lab_cleanup_20261111/start_test.sh) · [包装器 `main`、`run_child`](../robot_home/manual_map_lab_cleanup_20261111/mapping_session.py) · [原协调器 `main`](../robot_home/manual_map_postprocess_test/session.py)
2. 原协调器以 `session.json.final_map` 标明最终二维结果。C 失败时可保留 B 并跳过 D，D 失败时可保留 C。外层仅在本次原协调器成功退出、`status` 和 `phase` 都是 `complete`、最终图指向本次完成的 C 或 D 时启动 E。单独的 `run_cleanup.sh` 用于重处理既有会话，可显式传入源目录。 [C/D 阶段与回退](../robot_home/manual_map_postprocess_test/session.py) · [离线重处理入口](../robot_home/manual_map_lab_cleanup_20261111/run_cleanup.sh)
3. E 清理阶段再按 Ctrl+C 可取消，已保存的源会话仍可查看。E 写入独立候选目录，默认请求在机器人桌面打开本次 `comparison.png`。`--no-preview` 保留 PNG 并跳过桌面打开请求。`--max-seconds` 控制 E 的超时，默认 600 秒。`--protect-roi` 接受本次 PGM 的零基半开行列区间 `r0,r1,c0,c1`，可重复传入。`--review-roi` 只用于统计。导航继续由使用者选择地图。 [包装器和参数](../robot_home/manual_map_lab_cleanup_20261111/mapping_session.py) · [E 预览](../robot_home/manual_map_lab_cleanup_20261111/preview_cleanup.py)

## 在线原版和 A

1. MID-360 驱动发布 `/livox/lidar` 和 `/livox/imu`。从原版 launch 查驱动启动项、FAST-LIO 与投影器配置，再从驱动 `launch_ROS2/`、`msg/` 和 `src/` 追踪消息。Livox SDK2 提供雷达通信接口。 [原版监督器](../robot_home/manual_native180_20261022/run_session.py) · [原版 launch](../robot_home/manual_native180_20261022/ws/src/native_height_projection/launch/manual_native180.launch.py) · [驱动源码](../robot_home/zzj_ws/src/livox_ros_driver2/) · [SDK2 源码](../robot_home/livox-SDK2/)
2. FAST-LIO 预处理按原配置检查标签、线号及 `0.50 m` 近距，`point_filter_num=3`。使用完整扫描时间窗口同步 IMU、去畸变，之后按名义安装旋转保留机体前向 `X≥0` 的点。车辆转向后可逐步纳入原来身后的方向。点到面匹配使用 `0.20 m` 表面体素和原生 ikd-tree，树的插入降采样参数是 `0.10 m`。匹配前的去畸变前向端点和扫描末光心位姿分别由 `/fastlio/free_scan`、`/fastlio/free_scan_pose` 同戳发布。原生树当前有效点在 `camera_init` 下发布至 `/fastlio/native_map`，里程计仍由 `/Odometry` 提供。本入口使用 10 月 22 日原版，其他在线过滤档位属于独立入口。 [FAST-LIO 主循环](../robot_home/manual_native180_20261022/ws/src/fast_lio/src/laserMapping.cpp) · [前向裁剪与参考](../robot_home/manual_native180_20261022/ws/src/fast_lio/src/native_height_export.hpp) · [参数](../robot_home/manual_native180_20261022/ws/src/native_height_projection/config/native180_fastlio.yaml)
3. 启动静止窗口根据重力、位姿、外参与 `0.20 m` 雷达底面高度、`0.047 m` 底面至光心偏移建立一次固定地面参考。设地面原点为 `g`、地面基矩阵为 `B`，行向量 `p` 变换为 `(p - g) @ B`。投影器按固定地面高度 `0.10–1.00 m`、XY `0.05 m` 将原生树快照投为黑色占据。同格一条合格回波即可标黑。画布来自有效原生点 XY 包络，外扩 `0.50 m`，最多一千万格。每次从当前快照重新构造黑格。 [参考计算](../robot_home/manual_native180_20261022/ws/src/fast_lio/src/native_height_export.hpp) · [`Projector`](../robot_home/manual_native180_20261022/ws/src/native_height_projection/scripts/native_height_projector.py) · [`project_snapshot`](../robot_home/manual_native180_20261022/ws/src/native_height_projection/scripts/projection_core.py) · [投影参数](../robot_home/manual_native180_20261022/ws/src/native_height_projection/config/projection.yaml)
4. 在线空闲计算沿有真实终点的射线做三维格遍历，仅累积离光心 `0.50 m` 之外且距终点至少 `0.10 m` 的穿越。高度步长 `0.10 m`，共九层，允许跨帧累积。带内端点和当前黑格可清除该列的空闲证据。黑格优先，其余格九层都有穿越才白，否则灰。各层各有射线穿过是栅格判定规则，单条射线只覆盖其几何路径。实时 RViz 显示这张原版 `/map`。 [自由证据 `Evidence::ray`、`fe_integrate`、`fe_apply`](../robot_home/manual_native180_20261022/ws/src/native_height_projection/src/free_evidence.cpp) · [投影节点](../robot_home/manual_native180_20261022/ws/src/native_height_projection/scripts/native_height_projector.py)

## B、C、D 如何改变二维地图

| 阶段 | 证据和判定 | PGM 转换及输出 |
| --- | --- | --- |
| A → B（A005） | 顺序读取已封口的全部帧。每帧用雷达坐标的 5 cm 体素保留输入顺序的首个真实点，随后用记录位姿和固定地面参考转换。带内回波按 64×64 世界格块累积命中，再映射至原图 PGM。一个有效命中即可补原灰格。 | 灰 `205` → 黑 `0`，输出 `enhanced/`。 |
| B → C（Free 冲突） | 共同窗口自录制首帧至原图、原自由证据及录制末戳三者的最早值。每帧同一格仅计一次，选择在 A 为白且被至少三帧命中的格。高度簇、站位和射线代理另做证据诊断，当前策略按未校准冲突输出。 | 白 `254` → 灰 `205`，输出 `enhanced_conflict/`。 |
| C → D（孤立散点） | 对 C 的黑格一次做八连通标记。整块须有 1–3 格、距边界留足 4 格、与其他黑块格心最短距离至少 `0.20 m`。整块恰有一帧命中且其一格八邻域环无其他帧命中时才撤回。零命中或多帧命中的块保留。 | 黑 `0` → 灰 `205`，输出 `enhanced_scatter/`。 |

从 A005 的 `selected_band` 和 `HitTiles` 查点选择与映射，再读 `stream_hits`、`enhance` 和协调器写入的 `a005_checkpoint.json`。C 从 `algorithm_run` 和 `classify_cell` 追到射线代理及 `conflict_verify.py`。D 从 `find_geometry`、`read_support` 追到 `compose` 和 `verify_result`。这些阶段沿用原图几何、YAML 和历史位姿，在 Ctrl+C 保存之后处理二维副本。 [A005 计算](../robot_home/manual_map_postprocess_test/stream_kernel.py) · [A005 提交](../robot_home/manual_map_postprocess_test/offline_enhance.py) · [C 工作器](../robot_home/manual_map_postprocess_test/offline_conflict.py) · [C 判定](../robot_home/manual_map_postprocess_test/conflict_core.py) · [C 射线](../robot_home/manual_map_postprocess_test/ray_geometry.py) · [C 核验](../robot_home/manual_map_postprocess_test/conflict_verify.py) · [D 工作器](../robot_home/manual_map_postprocess_test/offline_scatter.py)

## E 旧占据撤销候选

1. `Source` 先核对 A、B、C 和所选最终 C/D 的会话身份、封口、产物哈希、PGM 转换和 YAML 字节。`projection_source_camera_init.pcd` 须能复现 A 的投影边界及黑格，用它作为原生支持来源。D 已撤回的格从 E 的源占据中排除。 [E `Source`、`select_final_map`](../robot_home/manual_map_lab_cleanup_20261111/offline_cleanup.py)
2. 第一遍 `capture_supports` 扫描所有封口帧，包括晚于投影快照的帧。对最终仍为黑的格同时保留原生点与落点的三维子区支持。子区由 5 cm 列内的 2.5 cm XY 子格和 2.5 cm 高度槽划分，以样本包围盒中心和半对角线记位置及半径。记录支持的首末命中时间和独立命中帧数。无可信时间关联的原生点继续保护占据。至少五帧且首末跨度至少两秒的持续支持也受保护。 [E `capture_supports`](../robot_home/manual_map_lab_cleanup_20261111/offline_cleanup.py)
3. 第二遍 `Source.frames`、`MissReplay.step` 重放整段记录。更晚的疑似重合端点，包括邻格、近高及移动帧，可推进最后命中并清掉旧票。投票要求相邻位姿连续低速至少 `0.5 s`，平移不超过 `0.02 m/s`，角速度不超过 `0.01 rad/s`，相邻帧间隔不超过 `0.25 s`。录制存在大于 250 ms 的已观测间隔时，E 拒绝该输入。 [E 时序和运动门槛](../robot_home/manual_map_lab_cleanup_20261111/offline_cleanup.py)
4. 从扫描末光心到更远实际回波构造三维射线。目标支持须落在同位置、同高度容差内的受保护线段。默认近端 `0.50 m`、远端 `5.0 m`、终点前保留 `0.15 m`，空间和高度容差各 `15 mm`。支持半径与假设的 `7 mm` 射线余量、`3 mm` 命中余量共同占用容差。近共线前景回波否决该帧的模糊穿越，低于桌板的射线只能支持其实际经过的高度。每个支持至少需要三站，逐站两两检查名义 `0.30 m` 光心位移、`20°` 水平方位差和 `1 s` 间隔，并在距离和时间门槛上加代码规定的误差余量。缺少回波的方向不计票。 [E `Policy`、`measure_ray`、`independent`](../robot_home/manual_map_lab_cleanup_20261111/cleanup_core.py) · [E `MissReplay.step`](../robot_home/manual_map_lab_cleanup_20261111/offline_cleanup.py)
5. 只有源图黑格的**全部**三维支持都达到票数，且格子在显式保护区之外，`classify` 才允许 `compose_map` 做 `0→205`。持续支撑是启发式，长期停留的人或重复噪点也可能被保留。候选包保存源图副本 `before/`、`map.pgm` 与 `map.yaml`、撤回和保护掩码、`cells.csv`、`changes.csv`、`supports.csv`、`report.json` 及前后对比图。每个见证可追到帧戳、帧号、文件偏移、端点索引和坐标。输出校验通过后以新目录提交。 [E `classify`、`write_evidence`、`execute`](../robot_home/manual_map_lab_cleanup_20261111/offline_cleanup.py) · [E `compose_map`](../robot_home/manual_map_lab_cleanup_20261111/cleanup_core.py)

## 数据格式和核查边界

- `observations.bin` 文件头是 8 字节 `A005OBS1`。每帧先读小端 `<qI7d`，依次是纳秒戳、点数、扫描末光心 XYZ 和 XYZW 四元数，随后是点数乘 12 字节的小端 `float32` XYZ。位姿在 `camera_init`，端点在 `native_scan_lidar`。设旋转为 `R`、光心为 `t`，转换为 `(p_lidar @ R.T + t - g) @ B`。固定参考、帧数、点数、范围、文件长度及 SHA 可从录制报告和检查点复核。端点和位姿用纳秒戳精确配对，保存的是去畸变后的前向端点。记录缺少 Livox 原始包、反射率、逐点时间和逐点光心。两个 BEST_EFFORT 话题同时漏掉同一帧时，精确配对也可能完整。 [录制器 `Recorder`、`xyz_bytes`](../robot_home/manual_map_postprocess_test/recorder.py) · [E `Source.frames`](../robot_home/manual_map_lab_cleanup_20261111/offline_cleanup.py)
- ROS 占据值为黑 `100`、白 `0`、灰 `-1`。PGM 字节为黑 `0`、白 `254`、灰 `205`，图像行自上而下，世界格 Y 向上。`original/map.yaml` 提供原点与分辨率。`projection_source_camera_init.pcd` 对应投影快照。`native_map_final_camera_init.pcd` 对应 FAST-LIO 退出时的原生树，两者可有不同采样时刻和点数。灰格表示未知，黑转灰后的格仍须按未知处理。 [投影保存](../robot_home/manual_native180_20261022/ws/src/native_height_projection/scripts/native_height_projector.py) · [原生保存](../robot_home/manual_native180_20261022/ws/src/fast_lio/src/laserMapping.cpp)
- `0.10–1.00 m` 高度带使用启动时固定地面。相对于该参考高度超过 1.00 m 的回波不进入此占据带。草莓棚架体最高约 1.10 m，且地面有小幅起伏，整架覆盖需现场测量。5 cm 是网格间距。细杆反射、遮挡、抽点、扫描内运动、外参和姿态误差仍会影响证据。C 和 E 的判定未校准，D 依赖小连通块和时间启发式，真实细支撑可能被误撤回。应先核查 C/D 输入图是否保留桌板和桌腿，再检验 E 的撤回位置和残影。源图已经缺失的桌腿不能靠 E 恢复。效果由使用者现场验收。 [投影参数](../robot_home/manual_native180_20261022/ws/src/native_height_projection/config/projection.yaml) · [E 实验规则](../robot_home/manual_map_lab_cleanup_20261111/README.md)
- 2 GB 是整套建图程序的验收目标。E 在 Linux 对自身进程应用 `1536 MiB` 地址空间上限，并在提交前检查自身峰值 RSS。该限制仅适用于 E。整链峰值需要覆盖 Livox、FAST-LIO、投影、DDS、记录、预览和保存的实际运行。 [E 资源限制与记录](../robot_home/manual_map_lab_cleanup_20261111/offline_cleanup.py)
- 录包、`observations.bin`、PCD、生成地图、掩码、对照图、日志、逐格证据和会话报告留在运行环境。仓库内的链接指向源码或配置。可从这些文件审查条件和数据契约，复现成图及验证内存需使用者单独提供现场输入并执行。
