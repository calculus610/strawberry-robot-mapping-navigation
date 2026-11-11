# 2026-11-11 当前实车手动建图全链源码

这是从机器人只读收集的**当前部署源码快照**，用于离开机器人后阅读、分析和继续讨论修改。
每个机器人来源文件的路径、SHA-256、大小和执行权限见 [SOURCE_MANIFEST.json](SOURCE_MANIFEST.json)。
收集时间和实际系统依赖见 [DEPENDENCIES.json](DEPENDENCIES.json)。归档过程没有启动建图、重放、编译或功能测试。

本次只上传算法、配置、依赖源码、许可证和说明；**不上传地图、点云、观测记录、录包或现场测试数据**。
源码归档完整不代表算法已经通过验收，当前清理参数仍未校准。

## 当前入口与历史版本

机器人上当前一键入口是：

```bash
bash /home/wheeltec/manual_map_lab_cleanup_20261111/start_test.sh
```

实际调用链：

```text
manual_map_lab_cleanup_20261111/start_test.sh
  └─ mapping_session.py：只绑定本次新建图的 SESSION_DIR
      └─ manual_map_postprocess_test/start_test.sh → session.py
          ├─ recorder.py：真实回波与位姿的旁路录制
          └─ manual_native180_20261022/run_session.py
              └─ native_height_projection/manual_native180.launch.py
                  ├─ zzj_ws/livox_ros_driver2 → Livox SDK2 → MID360
                  ├─ fast_lio：定位、去畸变、原生树与真实观测
                  └─ native_height_projection：二维投影与分层自由证据
      按一次 Ctrl+C → 等待原图、观测保存 → A005 → C → D
      └─ run_cleanup.sh → offline_cleanup.py：本次 C/D 图的独立候选 E
```

仓库根部原有的 `native180/` 保留 11 月 9 日 V1/3 m 等历史实验。
**当前入口调用的是本目录归档的 `manual_native180_20261022`，不能用历史 `native180/` 代替它分析。**
`robot_home/zzj_ws/src/FAST_LIO_ROS2` 也属于底层工作空间中的另一份源码；当前在线 FAST-LIO 来自独立的原生建图 overlay。

## 文件在哪里

| 路径 | 用途 |
| --- | --- |
| [robot_home/manual_map_lab_cleanup_20261111](robot_home/manual_map_lab_cleanup_20261111) | 最新一键入口、离线旧占据撤销、桌面/细杆保护条件、候选图及对比输出 |
| [robot_home/manual_map_postprocess_test](robot_home/manual_map_postprocess_test) | 原实车采集协调、观测格式、A005、自由冲突 C、孤立散点 D、保存与回退 |
| [robot_home/manual_native180_20261022](robot_home/manual_native180_20261022) | 当前使用的原生 FAST-LIO、投影、九层自由证据、配置、launch、RViz |
| [robot_home/zzj_ws/src](robot_home/zzj_ws/src) | 机器人实际 underlay 源码，含 Livox 驱动、消息、底盘及已有导航/建图工具；不是每个包都由当前入口启动 |
| [robot_home/livox-SDK2](robot_home/livox-SDK2) | 机器人构建所用 SDK 源码与第三方依赖；没有可确认的上游 Git 提交，使用快照哈希标识 |
| [runtime_reference/home/wheeltec](runtime_reference/home/wheeltec) | 从安装目录额外读取的实际运行配置、launch 和投影 Python 文件，便于比对源目录与安装目录 |
| [docs/ALGORITHM_CHAIN.md](docs/ALGORITHM_CHAIN.md) | 全流程算法、关键文件阅读顺序、证据规则和当前边界 |

源码内的 CMake、package.xml、头文件、消息定义和随附许可证一并保留。
`build/`、通用 `install/`、日志、缓存、历史备份、演示媒体和运行数据没有归档。
`runtime_reference/` 是明确列出的少量运行文本副本，不是完整安装目录。

## 回家后如何分析

打开本分支 `manual/20261111-current-live-stack`，先读 [算法说明](docs/ALGORITHM_CHAIN.md)，再沿上述目录查看实现。
本机同一 checkout 也包含完整快照，不需要 SSH。只阅读和分析源码不需要 ROS 或雷达。

以下文件最适合开始阅读：

1. [mapping_session.py](robot_home/manual_map_lab_cleanup_20261111/mapping_session.py)：新会话身份、Ctrl+C、保存与清理衔接。
2. [postprocess/session.py](robot_home/manual_map_postprocess_test/session.py)：原采集、停止、A005/C/D、失败回退。
3. [laserMapping.cpp](robot_home/manual_native180_20261022/ws/src/fast_lio/src/laserMapping.cpp) 和 [native_height_export.hpp](robot_home/manual_native180_20261022/ws/src/fast_lio/src/native_height_export.hpp)：原生点云、真实观测与位姿来源。
4. [projection_core.py](robot_home/manual_native180_20261022/ws/src/native_height_projection/scripts/projection_core.py) 和 [free_evidence.cpp](robot_home/manual_native180_20261022/ws/src/native_height_projection/src/free_evidence.cpp)：高度投影、九层自由证据及黑格优先。
5. [offline_cleanup.py](robot_home/manual_map_lab_cleanup_20261111/offline_cleanup.py) 和 [cleanup_core.py](robot_home/manual_map_lab_cleanup_20261111/cleanup_core.py)：旧占据撤回条件、几何判据、保护和资源上限。

仓库不含现场输入数据，因此仅克隆源码不会生成新的实测地图。

## 运行环境和复现边界

机器人使用 Jetson Orin Nano 8 GB、MID360、Ubuntu 22.04 arm64 和 ROS 2 Humble。
实际依赖包版本、Python 版本及已安装二进制的指纹在 `DEPENDENCIES.json`，构建依赖声明在各包 `CMakeLists.txt` / `package.xml`。
离线清理使用 Python 3.10+、NumPy、PyYAML、Pillow；原 A005/C/D 流程还使用 SciPy，在线采集使用 ROS 2 和 Livox 消息。

归档保留部署原字节：脚本中的 `/home/wheeltec/...` 绝对路径、版本哈希保护、桌面 Xauthority 及 ROS 环境没有为 Mac 改写。
因此这是一份可离机分析的完整源码快照，**不是解压后即可在任意机器启动实车的安装包**。
原 `protected_before.json` 包含安装文件/二进制指纹，重新构建产生新指纹时须重新核对部署，不能直接删掉保护检查。
SDK 二进制没有上传；SDK 源目录构建产物与已安装运行库的指纹记录可用于识别机器人使用的版本，不声称知道其上游提交。

新候选只在原地图完整保存后生成，不修改 FAST-LIO 定位或原图，也不自动切换导航地图。
1536 MiB 限制针对离线清理子进程；不能把它当作整套建图已实测低于 2 GB 的证明。
启动、建图质量、薄杆/桌面保护及整机峰值内存仍由用户执行验证。

## 许可证与归档方式

随附第三方许可证原样保留。原生 FAST-LIO 的 `LICENSE` 是 GPLv2 文本，而其 `package.xml` 标为 BSD；本归档记录该上游材料不一致，不擅自改写许可证或把整套代码重新授权。
驱动、SDK 及其第三方代码请分别查看自身的 LICENSE / NOTICE 文件。

本分支在既有提交 `28ae42e3bd171ff03956b438d75daee14f677fca` 上新增此快照和根 README 索引，历史版本保留。
此次归档不更新机器人部署。要回到原有仓库视图可切回 `manual/20261109-v1-3m-baseline`；无需删除快照或改动机器人。
