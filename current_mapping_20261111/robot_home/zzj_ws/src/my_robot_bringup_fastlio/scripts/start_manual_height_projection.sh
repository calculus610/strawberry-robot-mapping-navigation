#!/usr/bin/env bash
set -euo pipefail

# Refuse duplicate sensor/mapping sessions; never stop another session here.
if pgrep -f '[/](fastlio_mapping|octomap_server_node|async_slam_toolbox_node|height_projection_map|livox_ros_driver2_node)( |$)' >/dev/null; then
  printf '%s\n' 'A sensor/mapping process is already running. Stop its original session before starting this test.' >&2
  exit 1
fi

# Use the robot's existing graphical login. This does not enable X forwarding
# or change access control, and never prints Xauthority contents.
test -S /tmp/.X11-unix/X0 || { printf '%s\n' 'Robot desktop DISPLAY=:0 is unavailable.' >&2; exit 1; }
test -r /run/user/1000/gdm/Xauthority || { printf '%s\n' 'Robot desktop authorization file is unavailable.' >&2; exit 1; }

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
    exec ros2 launch my_robot_bringup_fastlio manual_mapping_height_projection.launch.py "$@"
  ' height-projection "$@"
