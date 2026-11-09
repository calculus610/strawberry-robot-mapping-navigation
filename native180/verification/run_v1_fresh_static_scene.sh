#!/usr/bin/env bash
set -eo pipefail
root=/home/wheeltec/manual_native180_v1_3m_20261109
backup=/home/wheeltec/manual_native180_backups/20261109_v1_3m
until test -f "$backup/fresh_capture_exit_code"; do sleep 2; done
test "$(cat "$backup/fresh_capture_exit_code")" = 0
out="$backup/fresh_static_comparison"
bag="$backup/fresh_static_scene/bag/bag_0.db3"
test -f "$bag"
test ! -e "$out"
mkdir "$out"
exec 9>"$root/session.lock"
flock -n 9
trap 'printf "%s\n" "$?" > "$out/exit_code"' EXIT
cp "$backup/fresh_static_scene/capture_report.json" "$out/"
cp "$root/ws/install/native_height_projection/share/native_height_projection/config/native180_fastlio.yaml" "$out/"
run_case() {
  local stage="$1" label="$2"
  if pgrep -f '[/](fastlio_mapping|livox_ros_driver2_node|native_height_projector.py)( |$)' >/dev/null; then
    echo 'Another sensor/mapping process is running.' >&2
    return 1
  fi
  printf 'START %s %s\n' "$label" "$(date -Is)"
  python3 -c '
import pathlib,resource,subprocess,sys,time
started=time.monotonic()
result=subprocess.run(sys.argv[2:])
usage=resource.getrusage(resource.RUSAGE_CHILDREN)
pathlib.Path(sys.argv[1]).write_text(f"wall_s={time.monotonic()-started:.6f} user_cpu_s={usage.ru_utime:.6f} sys_cpu_s={usage.ru_stime:.6f} max_rss_kb={usage.ru_maxrss}\n")
raise SystemExit(result.returncode)
' "$out/${label}_resources.txt" \
    env -i HOME=/home/wheeltec USER=wheeltec PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/bin LANG=C.UTF-8 ROS_DOMAIN_ID=83 ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 FAST_LIO_TIMECHAIN_DIAG=1 \
    bash --noprofile --norc -c '
      set -e
      source /opt/ros/humble/setup.bash
      source /home/wheeltec/zzj_ws/install/local_setup.bash
      source "$1/ws/install/local_setup.bash"
      test "$(ros2 pkg prefix fast_lio)" = "$1/ws/install/fast_lio"
      test "$(ros2 pkg prefix native_height_projection)" = "$1/ws/install/native_height_projection"
      python3 "$1/verification/check_recorded_native180_fast.py" --runner "$1/run_session.py" --bag "$5" --stage "$3" --retention-percent 100 --rate 1.0 --output "$2/$4"
    ' native-fresh-test "$root" "$out" "$stage" "$label" "$bag" > "$out/${label}.log" 2>&1
  python3 - "$out/$label/recorded_native180_result.json" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
assert 'error' not in r and r['map_saved_and_reconstructed'],r
if r['stage']==1: assert r['input_contract_verified'],r
assert r['odometry']['frames_received']>700,r
print(json.dumps({'case':sys.argv[1],'odometry':r['odometry']}),flush=True)
PY
  printf 'FINISH %s %s\n' "$label" "$(date -Is)"
}
run_case 0 original
run_case 1 v1_run1
run_case 1 v1_run2
run_case 1 v1_run3
