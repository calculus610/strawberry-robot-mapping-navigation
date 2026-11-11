#!/usr/bin/env bash
set -euo pipefail
tool_dir=$(cd -- "$(dirname -- "$0")" && pwd)
exec env -i \
  HOME=/home/wheeltec USER=wheeltec LOGNAME=wheeltec \
  PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin \
  LANG=C.UTF-8 LC_ALL=C.UTF-8 PYTHONDONTWRITEBYTECODE=1 \
  DISPLAY=:0 XAUTHORITY=/run/user/1000/gdm/Xauthority \
  XDG_RUNTIME_DIR=/run/user/1000 DBUS_SESSION_BUS_ADDRESS=unix:path=/run/user/1000/bus \
  QT_QPA_PLATFORM=xcb ROS_DOMAIN_ID=0 ROS_LOCALHOST_ONLY=0 FAST_LIO_TIMECHAIN_DIAG=1 \
  POSTPROCESS_TOOL_DIR="$tool_dir" \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    source /home/wheeltec/zzj_ws/install/local_setup.bash
    source /home/wheeltec/manual_native180_20261022/ws/install/local_setup.bash
    exec python3 -B "$POSTPROCESS_TOOL_DIR/session.py" "$@"
  ' manual_postprocess "$@"
