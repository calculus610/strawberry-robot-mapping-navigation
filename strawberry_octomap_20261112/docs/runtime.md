# 独立建图的运行与保存

## 机器人入口

首次由维护者在机器人上准备独立构建。构建从系统 ROS 2 Humble 和系统 Livox SDK2 起步，Livox 驱动、FAST-LIO 与 OctoMap 接入代码编译至本工程的 `ws/install`。

```bash
bash /home/wheeltec/strawberry_octomap_20261112/scripts/build_robot.sh
```

建图入口核对源码和安装产物的构建记录，校验实际 ROS 包前缀后启动。

```bash
bash /home/wheeltec/strawberry_octomap_20261112/start_mapping.sh --web-map
```

机器人桌面不可用时追加 `--no-rviz`。等待终端出现 `OBSERVATION_READY` 和 `OCTOMAP_MAP_READY`，再手动控制移动。结束时按一次 Ctrl+C，等终端出现 `SESSION_SAVED`。若显示 `SESSION_FAILED`，查看本次目录内的 `mapping.log` 与 `finalize.log`。

终端显示就绪等待原因、观测状态与成图进度。若日志出现 `OBSERVATION_REFERENCE_CHANGED`，本次会话标记为失败，已生成的文件可用于诊断。

## 会话与输出

`SESSION_DIR` 指向本次 `outputs/<会话 ID>`。建图使用 ROS 域 0 和本机发现。会话协调器持有本工程锁与原建图的共享锁，直到本次子进程退出。旧共享锁必须已存在。启动前还会检查占用雷达、建图或导航的进程。

第一次停止请求让后端封口并保存，随后只向本次 ROS launch 发送 SIGINT。一次完整保存包括：

- `map.pgm` 和 `map.yaml`，供二维地图读取。
- `octree.ot`，保存 OctoMap 体素状态。
- `metadata.json` 和 `session.json`，记录本次保存和会话状态。
- `memory.csv` 和 `memory_summary.json`，记录本会话进程的 RSS、PSS 和系统内存。采样从建图启动起每 10 秒进行，覆盖保存阶段，短于采样间隔的峰值可能漏记。内存记录只用于观察。

`runtime/session.json` 记录最近一次会话状态。FAST-LIO 子进程的工作目录是本次输出目录。FAST-LIO 编译时指定的诊断日志目录位于本工程的 `ws/src/fast_lio/Log`。

默认地图话题为 `/strawberry_octomap/map`。显式追加 `--web-map` 时，入口在持有互斥锁的条件下检查是否已有 `/map` 发布者，检查通过才允许本次后端额外发布 `/map`。该选项用于网页直接订阅实时地图。本工程会话识别已按单独批准部署到管理器，网页实际控制仍由用户验收；具体边界见 [Web 接入说明](../integration/web/README.md)。
