#!/usr/bin/env bash
set -euo pipefail
version="${1:?Expected baseline or version number}"
shift
workspace="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/.." && pwd)"

# Never stop or reuse an unrelated sensor/mapping session implicitly.
if pgrep -f '[/](fastlio_mapping|octomap_server_node|async_slam_toolbox_node|height_projection_map|livox_ros_driver2_node|livox_lidar_publisher2)( |$)' >/dev/null; then
  echo 'A sensor/mapping session is already running. Stop it from its original launcher first.' >&2
  exit 1
fi
if [[ "$version" == baseline ]]; then
  launch_file=manual_mapping_height_projection.launch.py
  defaults=()
else
  [[ "$version" =~ ^[123]$ ]] || { echo 'Version must be baseline, 1, 2, or 3.' >&2; exit 2; }
  launch_file=manual_mapping_filtered.launch.py
  defaults=("filter_config:=manual_filter_v${version}.yaml"
            "map_output_dir:=${HOME}/maps/manual_mapping_v${version}")
fi
test -r "$workspace/install/local_setup.bash" || {
  echo 'Build fast_lio and my_robot_bringup_fastlio first; see docs/MANUAL_MAPPING_VERSIONS_20261106.md.' >&2
  exit 1
}
exec env -i HOME="$HOME" USER="${USER:-wheeltec}" LOGNAME="${USER:-wheeltec}" \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 LC_ALL=C.UTF-8 DISPLAY="${DISPLAY:-:0}" \
  XAUTHORITY="${XAUTHORITY:-/run/user/$(id -u)/gdm/Xauthority}" \
  XDG_RUNTIME_DIR="/run/user/$(id -u)" \
  DBUS_SESSION_BUS_ADDRESS="unix:path=/run/user/$(id -u)/bus" \
  ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 \
  bash --noprofile --norc -c '
    set -e
    workspace="$1"; launch_file="$2"; shift 2
    source /opt/ros/humble/setup.bash
    source "$workspace/install/local_setup.bash"
    exec ros2 launch my_robot_bringup_fastlio "$launch_file" "$@"
  ' manual-version "$workspace" "$launch_file" "${defaults[@]}" "$@"
