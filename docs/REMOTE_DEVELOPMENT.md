# 在 Mac 上开发、在 Wheeltec 上构建

这套工作流把 Mac 作为源码和 Git 的主工作区，把 Wheeltec Ubuntu 主机作为
真实的 ROS 2 构建、运行和硬件调试环境。不要再来回复制整个工作空间。

## 1. 填写机器人地址

```bash
cd /Users/calculus_mac/Documents/zzj_ws_wheeltec
cp .robot.env.example .robot.env
```

用文本编辑器打开 `.robot.env`，填写机器人 IP：

```bash
ROBOT_HOST="机器人IP"
```

仓库当前配置表明机器人用户名和工作空间通常是：

```text
wheeltec
/home/wheeltec/zzj_ws
```

如果实际不同，再修改对应项。`.robot.env` 不会被 Git 提交。

## 2. 建立免密码 SSH

Mac 和机器人需要在同一局域网，或者通过 VPN/路由能够互相访问。

```bash
./scripts/robot.sh doctor
./scripts/robot.sh key
./scripts/robot.sh check
```

执行 `key` 时通常需要最后输入一次机器人登录密码。之后会使用专用密钥
`~/.ssh/wheeltec_robot_ed25519`。

## 3. 先只读检查机器人

```bash
./scripts/robot.sh context
./scripts/robot.sh audit-install
./scripts/robot.sh exec pwd
./scripts/robot.sh exec git status --short
./scripts/robot.sh shell
```

`shell` 会打开机器人终端，中文输入仍然来自 Mac。Codex 也可以通过这些命令读取
机器人文件、编译错误、ROS 话题和日志。

下载机器人工作空间内的单个文件或目录：

```bash
./scripts/robot.sh get log/latest_build
./scripts/robot.sh get src/some_package/config/example.yaml ./robot-downloads/
```

## 4. 安全地同步源码

第一次必须先预览：

```bash
./scripts/robot.sh sync-dry
```

预览会比较文件内容，而不只看时间戳。确认输出中的目标路径和改动正确后，再执行：

```bash
./scripts/robot.sh sync
```

同步方向只有 Mac 到机器人。默认不会删除机器人上的文件，并且排除了：

- `.git/`
- `.robot.env`
- `build/`、`install/`、`devel/`、`log/`
- FAST-LIO 的 `Log/`、`PCD/` 和 Python `__pycache__/`
- 机器人生成的 `maps/` 和 `.robot-sync-backups/`
- 常见缓存和 macOS 元数据

正式同步在覆盖远端旧文件前，会将旧版本保存到机器人工作空间中的：

```text
.robot-sync-backups/日期-时间/
```

因此普通覆盖可以恢复；`.robot-sync-backups/` 本身不会从 Mac 同步或被删除。

如果怀疑有人直接修改过 `install/`，先运行：

```bash
./scripts/robot.sh audit-install
```

只有 `DIFF` 和 `INSTALL_ONLY` 需要调查；`SUMMARY` 中 `different=0` 表示已安装的
可编辑资源与 `src` 一致。不要直接把整个 `install/` 复制回 `src`。

不要在还没确认两端目录职责前启用 `ROBOT_SYNC_DELETE=1`。

## 5. 在机器人上构建

```bash
./scripts/robot.sh build
```

或者一次完成同步和构建：

```bash
./scripts/robot.sh sync-build
```

默认远端命令等效于下面的流程，但会先使用 `env -i` 清空当前终端从
`.bashrc` 继承的其他 ROS 工作空间：

```bash
source /opt/ros/humble/setup.bash
cd /home/wheeltec/zzj_ws
colcon build --symlink-install
```

它可以在 `.robot.env` 中调整。脚本不会自动启动机器人、写串口、刷固件或改系统服务。
它也不会读取或修改机器人用户的 `.bashrc`。

`./scripts/robot.sh shell` 会用相同方式进入隔离环境，并自动加载：

```bash
source /opt/ros/humble/setup.bash
source /home/wheeltec/zzj_ws/install/setup.bash
```

## 6. Git 的职责

Git 主要运行在 Mac。机器人只接收可构建文件，不同步 Mac 的 `.git` 目录。

每次修改后的建议流程：

```bash
git status --short
git diff
./scripts/robot.sh sync-build
git add 具体文件名
git commit -m "说明这次修改"
```

只添加本次确认过的文件，不建议初学阶段使用 `git add .`。当前仓库没有 GitHub/GitLab
远程地址，这不影响本地版本管理；以后有远程仓库 URL 时再添加 `origin`。

## 常见问题

### SSH 超时

检查机器人 IP、两端是否在同一网络，以及机器人是否启用了 SSH：

```bash
sudo systemctl status ssh
```

如果机器人没有 SSH 服务，可在机器人端安装并启用：

```bash
sudo apt install openssh-server
sudo systemctl enable --now ssh
```

### Host key changed

不要直接忽略。先确认机器人确实重装过系统或该 IP 被重新分配，再删除旧的主机密钥记录。

### 可以直接访问机器人本地文件吗

可以。`shell`、`exec` 和 `get` 都直接访问机器人文件系统。SSH 本身不会让目录自动显示在
Finder 中；日常开发更推荐本地源码加增量同步，而不是 SSHFS 挂载整个 ROS 工作空间。
