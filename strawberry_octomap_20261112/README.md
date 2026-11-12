# Strawberry OctoMap 独立工程

使用现有 FAST-LIO 定位，官方 OctoMap 在线更新三维占据，再生成保守二维候选图。停止时保存当前图，不运行旧 A–E 后处理。

**此快照对应 2026-11-12 已部署、正在现场迭代的版本，尚不能据此宣称已完成最终验收。** 当前参数为 5 cm、hit 0.75、miss 0.45、盲区 0.30 m；详细接口与已知限制见交接文档。

- [架构、取舍与验收依据](docs/ARCHITECTURE.md)
- [观测与光心坐标契约](docs/observation_contract.md)
- [后端与二维投影](docs/backend.md)
- [启动、保存、内存与 Web 集成边界](docs/runtime.md)
- [当前系统原理与 Web 接口交接说明](docs/手动建图系统原理与Web接口说明.md)
- [旧源码来源及哈希](provenance/vendor_sources.json)

## 目录

```text
strawberry_octomap_20261112/
  docs/                     架构和运行说明
  provenance/               源码来源与本次发布哈希清单
  ws/src/fast_lio/           主线源码副本，不引入无降采样版本
  ws/src/livox_ros_driver2/  独立驱动源码副本
  ws/src/strawberry_octomap/ 观测桥、官方服务器派生类、配置与显示
  ws/build,install,log/     仅机器人生成的独立构建产物
  scripts/                  预编译及冻结保存入口
  integration/web/          已批准的 Web 兼容补丁与部署记录
  runtime/                  会话状态和本工程锁
  outputs/<会话>/            地图、内存、日志；不进入 GitHub
```

机器人路径为 `/home/wheeltec/strawberry_octomap_20261112`。不 source 旧工作空间；系统 Humble、OctoMap、Nav2 map_io 和 Livox SDK2 作为只读依赖。

## 机器人运行入口

独立构建成功后，在 SSH 中执行：

```bash
bash /home/wheeltec/strawberry_octomap_20261112/start_mapping.sh --web-map
```

此命令只启动已编译版本，不在启动时编译。机器人桌面打开本工程 RViz，显示 `/strawberry_octomap/map`。机器人启动时保持静止，等待 `OBSERVATION_READY` 与 `OCTOMAP_MAP_READY`。无桌面时追加 `--no-rviz`。

**Web 会话识别补丁已单独批准并部署。** 上述命令以 `--web-map` 同时提供网页的 `/map` 显示。等待两个就绪标记后，在 Web 申请控制权，再手动控制移动。控制权和速度检查沿用原管理器；未执行实际控制测试。网页的“开始建图/保存”按钮仍属于原流程，本工程由此 SSH 入口启动、Ctrl+C 保存。省略 `--web-map` 时只发布独立候选话题。

结束按一次 **Ctrl+C**。入口先请求后端停止接收扫描，保存同一版 `map.pgm`、`map.yaml`、`octree.ot`、`metadata.json`，随后关闭本会话节点。看到 `SESSION_SAVED=...` 才表示流程完整；失败会保留日志并明确报错。

内存从建图启动起每 10 秒记录 `memory.csv`，包含启动、建图与保存阶段，结束生成 `memory_summary.json`。编译期间不记录。没有 2 GB 自动停止；10 秒采样可能漏掉更短的瞬时峰值。

## 构建与恢复

由助手准备构建；需要手工重建时执行：

```bash
bash /home/wheeltec/strawberry_octomap_20261112/scripts/build_robot.sh
```

构建顺序为 Livox → FAST-LIO → 新后端，关闭测试、单任务编译，写入源码/安装产物哈希。直接运行启动脚本会检查哈希，拒绝混用旧产物。

回到原版本只需正常退出本候选会话，再使用原启动命令。旧源码、参数和地图没有被覆盖。不需要删除这个新工程，也不要执行大范围 reset。

## GitHub 源码快照说明

本目录包含算法源码、配置、启动/保存脚本和 Web 对接说明，不包含地图、点云、rosbag、运行日志、现场会话状态、构建产物或原始部署记录。文档中列出的 `outputs/`、`runtime/` 与部署记录路径供机器人现场排查使用，不随本仓库上传。当前发布依据见 [published_snapshot.json](provenance/published_snapshot.json)，上游来源见 [vendor_sources.json](provenance/vendor_sources.json)。

克隆后在已具备 ROS 2 Humble、OctoMap、Nav2 MapIO 和 Livox SDK2 的机器人上，按本文维护入口重新构建；仓库不携带本机 `build_stamp.json`。旧 `docs/ARCHITECTURE.md` 中的首版参数仅供历史设计参考，当前值以主配置及 [Web 交接文档](docs/手动建图系统原理与Web接口说明.md) 为准。

`integration/web/deploy_approved.py` 是当时批准部署的维护脚本，依赖本仓库外的本机 Tailscale SSH 配置，不是通用安装器；克隆不等于自动获得机器人访问权或部署授权。已有机器人不要重复部署 Web 补丁。
