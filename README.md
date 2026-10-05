# zzj_ws

ROS 2 Humble workspace for the Wheeltec R680 Ackermann (`senior_akm`) robot:
chassis driver, FAST-LIO / slam_toolbox mapping and Nav2 autonomous
exploration source for the Livox MID-360 based stack.

Version-controlled content is kept in `src/` plus workspace-level documentation and helper files.

Third-party robotics components (`FAST_LIO_ROS2`, `livox_ros_driver2`,
`m-explore-ros2`, `turn_on_wheeltec_robot`, `wheeltec_robot_msg`, `depend/*`)
are currently vendored in this repository at known upstream revisions, with
project-specific patches applied locally. This keeps the robot-specific MID360
integration changes available from a single clone. Repository-level third-party
attribution will be finalized in a later provenance / repository-hygiene step
during this recovery.

Conceptual data chain (the explorer does not implement Hybrid-A* itself;
Nav2/Smac generates the paths, while `ackermann_explore` generates candidate
observation poses and evaluates the planner-generated paths using
Ackermann-relevant criteria):

```text
Livox MID-360 -> FAST-LIO -> 2D scan projection / relay -> slam_toolbox
  -> Nav2 (Smac Hybrid-A* + MPPI Ackermann) -> Ackermann-aware frontier explorer
```

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
underlay.

**HISTORICAL / SUSPENDED:** The sync workflow below is retained for reference.
Do not run `sync-dry` or `sync` until robot-only state has been recovered and
reconciled.

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

Sourcing `~/zzj_ws/install/setup.bash` activates a package environment hook
which filters specified `*/install/nav2_*`, `dwb_*`, and `nav_2d_*`-style
path entries in its handled environment variables. It retains `/opt/ros/humble`
and nonmatching paths from this and other workspaces; it does not isolate every
possible overlay. The autonomous launch checks that `nav2_bringup` and
`nav2_mppi_controller` resolve to `/opt/ros/humble` before starting any robot
process.

Autonomous exploration uses the `ackermann_explore` executable by default.
It waits until the Nav2 controller, planner, and BT navigator lifecycle nodes
are all active before evaluating or sending a frontier goal. A temporarily
rejected goal is retried without blacklisting the frontier, and automatic
completion is disabled for the first 60 seconds after the start pose is
recorded.
Before the first motion test, run one selection cycle without sending a
navigation goal:

```bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py \
  explorer_dry_run:=true
```

RViz is optional. The recovered development launch starts it by default
(`start_rviz=true`); for headless SSH sessions, systems without a display
environment, or resource-constrained runs, pass `start_rviz:=false`.

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

For a manual robot-side terminal, source this workspace and launch normally:

```bash
cd ~/zzj_ws
source install/setup.bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py
```

A plain robot terminal also reads the shared `.bashrc`; the Nav2 isolation is
enforced by the environment hook installed with `install/setup.bash` together
with the launch-time underlay assertion above.

For the historical Mac-edit/robot-build SSH workflow, see
[`docs/REMOTE_DEVELOPMENT.md`](docs/REMOTE_DEVELOPMENT.md). That guide has not
received its targeted recovery update and is **not approved for deployment**;
its Mac-to-robot sync steps remain suspended. The helper script is
`scripts/robot.sh`; it kept generated ROS build directories on the Ubuntu robot
and synced source changes incrementally from the Mac.

## Caveats and open items

- This repository is the **canonicalized recovered Mac development state** of
  the project. The formal robot runtime is NOT fully reconciled with it.
- The robot's formal `nav2_wheeltec_mid360.yaml` recorded on 2026-08-24
  (SHA-256 prefix `15431e20...`) has not been recovered; the version here is the
  latest Mac-preserved configuration, not a proven robot copy.
- `/home/wheeltec/run_auto_mapping.sh` (robot-side startup entry) has not been
  recovered.
- Therefore **no Mac -> robot synchronization** may be performed from this
  repository until the robot-only artifacts above are retrieved and diffed.
- The 0.80 m minimum turning radius is a configured engineering assumption; the
  repository holds no physical calibration evidence.
- The recovered MPPI timing (15 Hz controller, `model_dt` 0.07) is a non-equal
  configuration that triggers an upstream Humble warning (control-sequence
  shifting off); see the technical stack document for details.
- The recovered Ackermann exploration revision is a **current baseline under
  active experimental evaluation**; the final historical revision lacks complete
  end-to-end field-validation evidence.

## Documentation index

- Platform / runtime guide: [`src/README_WHEELTEC_MID360.md`](src/README_WHEELTEC_MID360.md)
- Quick autonomous mapping startup: [`docs/自主建图启动说明.md`](docs/自主建图启动说明.md)
- Technical stack and implementation: [`docs/手动建图与自主建图技术栈及实现原理.md`](docs/手动建图与自主建图技术栈及实现原理.md)
- System handover manual: [`docs/手动建图与自主建图系统交接手册.md`](docs/手动建图与自主建图系统交接手册.md)
- Workspace layout and repository rules: [`docs/WORKSPACE_LAYOUT.md`](docs/WORKSPACE_LAYOUT.md)
- Historical, suspended Mac/robot remote workflow: [`docs/REMOTE_DEVELOPMENT.md`](docs/REMOTE_DEVELOPMENT.md)
