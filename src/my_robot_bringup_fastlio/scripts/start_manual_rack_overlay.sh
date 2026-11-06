#!/usr/bin/env bash
set -euo pipefail
if pgrep -f '[/](fastlio_mapping|octomap_server_node|async_slam_toolbox_node|height_projection_map|rack_obstacle_layer|livox_ros_driver2_node)( |$)' >/dev/null; then
  printf '%s\n' 'A sensor/mapping process is running. Stop its original session before starting this experiment.' >&2
  exit 1
fi
test -S /tmp/.X11-unix/X0 || { printf '%s\n' 'Robot desktop DISPLAY=:0 unavailable.' >&2; exit 1; }
test -r /run/user/1000/gdm/Xauthority || { printf '%s\n' 'Robot desktop authorization unavailable.' >&2; exit 1; }
printf '%s\n' 'Rack overlay test: keep the robot stationary until HEIGHT_REFERENCE_READY, then wait for the old 2D map.'
printf '%s\n' 'The supplementary layer needs 3 observation positions separated by at least 10 cm. No automatic motion is commanded.'
exec env -i \
  HOME=/home/wheeltec USER=wheeltec LOGNAME=wheeltec \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 LC_ALL=C.UTF-8 \
  DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus \
  QT_QPA_PLATFORM=xcb ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    source /home/wheeltec/zzj_ws/install/local_setup.bash
    exec ros2 launch my_robot_bringup_fastlio manual_mapping_rack_overlay.launch.py "$@"
  ' rack-overlay "$@"
