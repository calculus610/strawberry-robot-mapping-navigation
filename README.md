[English](README.md) | [简体中文](README.zh-CN.md)

# Strawberry Robot Mapping & Navigation

This repository contains the **canonicalized recovered Mac development state** of the ROS 2 Humble mapping and navigation stack for the Wheeltec R680 Ackermann robot in a strawberry-harvesting robot project. It brings together chassis integration, a Livox MID-360 sensor pipeline, mapping, Nav2 navigation, and project-specific autonomous exploration. It is a recovered development workspace, **not a verified copy of the formal robot runtime**.

## Project overview

The stack uses FAST-LIO for lidar-inertial odometry, projects MID-360 data into a 2D scan for slam_toolbox mapping, and uses Nav2 for planned Ackermann motion. The project integrates these components with the Wheeltec chassis, launch lifecycle, map autosaving, and software stop mitigations. Upstream components remain upstream work; their presence here does not imply they were written from scratch for this project.

## System architecture

```text
Livox MID-360 → Livox driver → FAST-LIO → 2D scan projection/relay → slam_toolbox map
                                   ↘ odometry + map → Nav2 → Wheeltec Ackermann chassis
Frontiers → ackermann_explore candidate observation poses → Nav2 planner paths
          → Ackermann-aware path evaluation → navigation goals
```

Nav2's Smac Hybrid-A* planner generates paths using a Reeds-Shepp motion model; its MPPI controller uses an Ackermann model. `ackermann_explore` **does not implement** `Hybrid-A*`: it proposes observation poses and evaluates planner-generated paths using criteria such as travel, reversing, and steering.

## Core capabilities

- Wheeltec R680 Ackermann chassis and Livox MID-360 integration.
- FAST-LIO odometry, 2D scan projection, and slam_toolbox mapping.
- Nav2 global planning with Smac Hybrid-A* / Reeds-Shepp and MPPI Ackermann control.
- Ackermann-aware frontier exploration, candidate path checks, and navigation goal selection.
- Periodic and exit-time map-save attempts, plus launch and runtime lifecycle coordination.
- Chassis command watchdog, shutdown zero-velocity guard, and exploration completion stop attempts. These are **best-effort software mitigations**, not certified emergency-stop guarantees.

## Repository structure

- `src/` — ROS 2 packages, including upstream/vendored sources, local patches, integration, and custom project code.
- `docs/` — startup, architecture, workspace, and handover documentation.
- `scripts/robot.sh` — historical Mac/robot helper; its Mac → robot synchronization commands are **SUSPENDED**.
- `README.zh-CN.md` — full Simplified Chinese counterpart to this landing page.

Generated `build/`, `install/`, and `log/` directories are not source-controlled.

## Runtime entry — reference only

The commands below describe the recovered development launch on a prepared, **robot-side-reconciled** ROS 2 Humble workspace. They are not approval to deploy this Mac state to the formal robot runtime.

```bash
cd ~/zzj_ws
source /opt/ros/humble/setup.bash
source install/setup.bash
ros2 launch my_robot_bringup_fastlio autonomous_exploration.launch.py explorer_dry_run:=true start_rviz:=false
```

`explorer_dry_run:=true` runs one frontier/candidate/planner selection cycle without sending a `NavigateToPose` goal. It does not validate MPPI control, chassis behavior, or field safety. The documented normal launch omits that argument, but requires robot-side reconciliation and controlled validation first.

RViz is optional. The recovered development launch defaults to `start_rviz=true`; use `start_rviz:=false` for headless or resource-constrained sessions. The package environment hook filters selected conflicting Nav2 overlay paths, and the autonomous launch checks that key Nav2 packages resolve to `/opt/ros/humble`. These checks do not establish that the full robot runtime has been reconciled.

**SUSPENDED:** Do not run `./scripts/robot.sh sync-dry`, `sync`, or `sync-build` as deployment steps. Mac → robot synchronization remains suspended until robot-only state is recovered and compared. The historical workflow is documented in [REMOTE_DEVELOPMENT.md](docs/REMOTE_DEVELOPMENT.md), which is **not currently approved for deployment**.

## Safety and recovery status

- The formal robot runtime has **not** been fully reconciled with this canonicalized recovered Mac development state.
- The robot-side 2026-08-24 `nav2_wheeltec_mid360.yaml` (SHA-256 prefix `15431e20...`) and `/home/wheeltec/run_auto_mapping.sh` have not been recovered. Mac → robot synchronization therefore remains suspended pending robot-side reconciliation.
- The configured minimum turning radius of **0.80 m** is an engineering assumption, not a physically calibrated value.
- The recovered MPPI timing of **15 Hz** with `model_dt=0.07` triggers the known ROS 2 Humble warning and leaves control-sequence shifting off.
- The current Ackermann exploration revision is an experimental baseline; complete final end-to-end field-validation evidence is unavailable.
- Map autosaving is a best-effort attempt. Its PGM/YAML pair is not transactionally atomic; verify both files before use.
- Software watchdog and stop behavior cannot guarantee physical stopping or replace a hardware emergency stop.

## Documentation

- [Wheeltec MID-360 platform and runtime guide](src/README_WHEELTEC_MID360.md)
- [Autonomous mapping startup guide](docs/自主建图启动说明.md)
- [Manual and autonomous mapping technical stack](docs/手动建图与自主建图技术栈及实现原理.md)
- [System handover manual](docs/手动建图与自主建图系统交接手册.md)
- [Workspace layout and repository rules](docs/WORKSPACE_LAYOUT.md)
- [Historical Mac/robot remote-development workflow — SUSPENDED](docs/REMOTE_DEVELOPMENT.md)

## Third-party software and attribution

This repository combines **upstream/vendored software** (including FAST-LIO, the Livox driver, m-explore, Wheeltec drivers, and supporting dependencies), **upstream software with local project patches**, **project configuration and integration**, and **project-specific implementations** such as `ackermann_explore`. Nav2 and slam_toolbox are upstream ROS components integrated and configured here, not original project implementations. Refer to package histories and the technical documentation for component-level context; this README does not claim a complete provenance inventory.
