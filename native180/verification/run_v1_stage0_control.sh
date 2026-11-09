#!/usr/bin/env bash
set -eo pipefail
root=/home/wheeltec/manual_native180_v1_3m_20261109
backup=/home/wheeltec/manual_native180_backups/20261109_v1_3m
test "$(cat "$backup/comparison_run2/exit_code")" = 0
test ! -e "$backup/stage0_control"
exec 9>"$root/session.lock"
flock -n 9
if pgrep -f '[/](fastlio_mapping|livox_ros_driver2_node|native_height_projector.py)( |$)' >/dev/null; then
  echo 'Another mapping/sensor process is running.' >&2
  exit 1
fi
trap 'printf "%s\n" "$?" > "$backup/stage0_control_exit_code"' EXIT
env -i HOME=/home/wheeltec USER=wheeltec PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 \
  ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 FAST_LIO_TIMECHAIN_DIAG=1 \
  bash --noprofile --norc -c '
    set -e
    source /opt/ros/humble/setup.bash
    source /home/wheeltec/zzj_ws/install/local_setup.bash
    source "$1/ws/install/local_setup.bash"
    test "$(ros2 pkg prefix fast_lio)" = "$1/ws/install/fast_lio"
    python3 "$1/verification/check_recorded_native180_fast.py" --runner "$1/run_session.py" --bag /home/wheeltec/rosbags/stationary_ground_20261021_131314/bag/bag_0.db3 --stage 0 --retention-percent 100 --rate 1.0 --output "$2/stage0_control"
  ' native-stage0-test "$root" "$backup" > "$backup/stage0_control.log" 2>&1
