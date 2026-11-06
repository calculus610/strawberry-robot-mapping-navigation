# Wheeltec 工作空间结构

## 目录职责

Mac 是源码与 Git 的主工作区，建议只保留：

```text
zzj_ws_wheeltec/
  src/                 ROS 2 源码
  scripts/             SSH、同步、审计工具
  docs/                项目和排障文档
  .codex/              项目硬件/工具链上下文
  README.md
```

机器人 `/home/wheeltec/zzj_ws` 是构建与运行工作区，正常包含：

```text
src/                   从 Mac 同步的源码
build/                 colcon 生成，不进 Git、不回传 Mac
install/               colcon 生成，不直接编辑
log/                   colcon 日志，不进 Git、不回传 Mac
maps/                  机器人运行数据，不由 Mac 覆盖
.robot-sync-backups/   同步前自动保存的远端旧文件
```

## 工作规则

1. 所有配置、launch、RViz 和行为树修改都落在 `src/`。
2. 使用 `colcon build --symlink-install`，不要直接编辑 `install/`。
3. 怀疑安装树漂移时运行 `./scripts/robot.sh audit-install`。
4. 同步前运行 `./scripts/robot.sh sync-dry`，确认后才运行 `sync`。
5. `build/install/log` 只存在于机器人；Mac 上即使存在，也是不可移植的 Linux 生成物。
6. FAST-LIO 与 Livox Driver 是 Git 子模块；其中的项目定制不能通过删除或重置来“清理”。
7. Nav2 运行时由 `/opt/ros/humble` 中的官方 APT 包提供；不要把系统包复制到工作区。
8. 不修改共享账号的 `.bashrc`；构建与运行使用隔离 shell，再依次加载
   `/opt/ros/humble/setup.bash` 和 `install/setup.bash`。
9. 底盘需要的旧 `serial` API 没有 Humble APT 包，使用
   `src/depend/serial_ros2` 中锁定的独立源码，不再借用 `wheeltec_ros2/install`。

## 2026-07-22 审计结论

- 初次审计时机器人端共发现 8 个源码包和安装包；清理隐式依赖后增加了明确的
  `serial` 源码包，当前为 9 个。
- 首次审计的 68 个可编辑资源一致，只有
  `my_robot_bringup_fastlio/config/slam_toolbox_fastlio.yaml` 在 `install` 中较新。
- 该文件已先备份旧 `src`，再将新回环参数写回 `src`；Mac 源文件与新版本哈希一致。
- `my_robot_bringup_fastlio` 已用 `--symlink-install` 单包重建成功。
- 2026-07-22 使用 `/opt/ros/humble` 单一 underlay 全量重建成功，9 个包全部完成；
  审计结果为 `matched=70`、`different=0`、`install_only=0`、`broken=0`。
- Nav2 1.1.20 和 Python 模块均来自 `/opt/ros/humble`，实际隔离启动中 MPPI
  FollowPath、8 个 Critic 和所有 Nav2 lifecycle 节点成功激活。
- Mac 中复制来的 `build/install/log` 是机器人生成物，其中安装树包含大量指向
  `/home/wheeltec/zzj_ws/src` 的绝对符号链接，不适合在 Mac 保存。
- 机器人 `src` 中存在大量 macOS AppleDouble `._*` 文件；这些是元数据垃圾，不是源码。
- 根目录中的 apt 包名空文件是命令残留，不是依赖声明。系统依赖应记录在 README、
  `package.xml` 或安装脚本中。
- `src/FAST_LIO_ROS2` 和 `src/livox_ros_driver2` 包含真实项目修改，必须保留并在后续
  形成明确的子模块提交或补丁，不能直接恢复到上游版本。

## 尚需完成的 Git 整理

当前主仓库以及两个子模块都有尚未提交的项目修改。合理顺序是：

1. 清除 AppleDouble、Python 缓存和生成目录。
2. 在机器人上完成全工作空间构建验证。
3. 分别提交 FAST-LIO 和 Livox Driver 的项目定制。
4. 更新主仓库的子模块指针，并提交其余 Wheeltec 集成修改。
5. 根据需要再添加 GitHub、Gitee 或私有 Git 远程仓库。

在构建验证之前，不为追求 `git status` 空白而丢弃现有修改。
