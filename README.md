# zzj_ws

ROS 2 Humble workspace for the Wheeltec MID360 robot driver and bringup source.

Version-controlled content is kept in `src/` plus workspace-level documentation and helper files.

`src/FAST_LIO_ROS2` and `src/livox_ros_driver2` are vendored in this repository
instead of being Git submodules. This keeps the robot-specific MID360
integration changes available from a single clone.

Generated build outputs are intentionally ignored:

- `build/`
- `install/`
- `log/`

The robot must provide the official Nav2 1.1.20 runtime:

```bash
sudo apt update
sudo apt install \
  ros-humble-nav2-bringup \
  ros-humble-nav2-mppi-controller \
  ros-humble-pointcloud-to-laserscan
```

The Wheeltec chassis driver also requires the `serial` source package kept at
`src/depend/serial_ros2`; it is pinned to upstream commit
`def526d378501e0d19d0a8724971688a3b174080` because ROS Humble does not provide
the compatible legacy `serial` API as an APT package.

Do not add this workspace to the shared robot account's `.bashrc`. The helper
always builds in a clean shell containing only `/opt/ros/humble` as the
underlay:

```bash
./scripts/robot.sh sync-dry
./scripts/robot.sh sync
./scripts/robot.sh build
./scripts/robot.sh shell
```

`shell` opens the robot workspace with `/opt/ros/humble/setup.bash` and
`~/zzj_ws/install/setup.bash` already sourced, without reading `.bashrc`.
Start autonomous exploration from that shell with:

```bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py
```

The mapping launches reuse the shared `robot-base.service` by default and do
not start a second chassis driver. The FAST-LIO mapping and Nav2 chain uses the
private `fastlio_base_footprint` frame so the shared EKF can continue owning
`odom_combined -> base_footprint` without creating a multiple-parent TF tree.
Only use `start_base:=true` after the shared chassis service has been stopped
intentionally.

If the service reports `active` but `/wheeltec_robot` or `/cmd_vel` is missing,
restart and verify the complete chassis service from the Mac with:

```bash
./scripts/robot.sh restart-base
```

This may ask for the robot account's sudo password. It uses `systemctl restart`
so any stale chassis process is stopped as part of the service cgroup before a
new driver is started; it does not kill unrelated ROS processes by name.

For a manual robot-side terminal, first enter an isolated shell and then use
the normal ROS setup files:

```bash
cd ~/zzj_ws
env -i HOME="$HOME" USER="$USER" LOGNAME="$USER" SHELL=/bin/bash \
  TERM="${TERM:-xterm-256color}" DISPLAY="${DISPLAY:-}" \
  XAUTHORITY="${XAUTHORITY:-}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 LC_ALL=C.UTF-8 bash --noprofile --norc
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py
```

For the Mac-edit/robot-build SSH workflow, see
[`docs/REMOTE_DEVELOPMENT.md`](docs/REMOTE_DEVELOPMENT.md). The helper script is
`scripts/robot.sh`; it keeps generated ROS build directories on the Ubuntu robot
and syncs source changes incrementally from the Mac.
