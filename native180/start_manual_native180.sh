#!/usr/bin/env bash
set -euo pipefail
experiment_dir=$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)
stage="${NATIVE_OPTIMIZATION_STAGE:-0}"
[[ "$stage" =~ ^[012]$ ]] || { echo 'Unsupported optimization stage.' >&2; exit 2; }
for argument in "$@"; do
  [[ "$argument" != optimization_stage:=* ]] || { echo 'Use the versioned startup script to select a stage.' >&2; exit 2; }
done
if pgrep -f '[/](fastlio_mapping|octomap_server_node|async_slam_toolbox_node|height_projection_map|rack_obstacle_layer|livox_ros_driver2_node|native_height_projector.py)( |$)' >/dev/null; then
  printf '%s\n' 'A sensor/mapping process is already running. Stop its original session first.' >&2
  exit 1
fi
test -S /tmp/.X11-unix/X0 || { printf '%s\n' 'Robot desktop DISPLAY=:0 unavailable.' >&2; exit 1; }
test -r /run/user/1000/gdm/Xauthority || { printf '%s\n' 'Robot desktop authorization unavailable.' >&2; exit 1; }
test -r "$experiment_dir/ws/install/local_setup.bash"
exec 9>"$experiment_dir/session.lock"
flock -n 9 || { printf '%s\n' 'This experiment is already starting/running.' >&2; exit 1; }
mkdir -p /home/wheeltec/maps/manual_native180
session_dir=$(mktemp -d "/home/wheeltec/maps/manual_native180/native180_v${stage}_$(date +%Y%m%d_%H%M%S)_XXXXXX")
cp "$experiment_dir/ws/src/native_height_projection/config/"*.yaml "$session_dir/"
cp "$experiment_dir/deployment_manifest.json" "$session_dir/"
printf '{"optimization_stage":%s}\n' "$stage" > "$session_dir/optimization_profile.json"
mkdir -p "$experiment_dir/ws/src/fast_lio/Log"
printf '%s\n' "SESSION_DIR=$session_dir" \
  'Keep stationary on flat ground until NATIVE_HEIGHT_REFERENCE_READY and PROJECTION_READY, then drive manually.' \
  'Input: forward180 after full-frame deskew. 3D: native ikd-tree only. 2D: ground-relative 0.10--1.00 m.' \
  'Native map voxel=0.10 m. XY grid=0.05 m. All 9 height layers need ray-free evidence for white.' \
  'Robot desktop RViz; /map occupied=black, confirmed free=white, unknown=gray. No rosbag. No motion commands.' \
  'Ctrl-C once and WAIT for MAP_SAVED and NATIVE_MAP_SAVED before powering off.'
exec env -i \
  HOME=/home/wheeltec USER=wheeltec LOGNAME=wheeltec \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus \
  QT_QPA_PLATFORM=xcb ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 \
  FAST_LIO_TIMECHAIN_DIAG=1 ROS_LOG_DIR="$session_dir/logs" \
  EXPERIMENT_DIR="$experiment_dir" SESSION_DIR="$session_dir" NATIVE_OPTIMIZATION_STAGE="$stage" \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    source /home/wheeltec/zzj_ws/install/local_setup.bash
    source "$EXPERIMENT_DIR/ws/install/local_setup.bash"
    exec python3 "$EXPERIMENT_DIR/run_session.py" "$SESSION_DIR" "optimization_stage:=$NATIVE_OPTIMIZATION_STAGE" "$@"
  ' native180 "$@"
