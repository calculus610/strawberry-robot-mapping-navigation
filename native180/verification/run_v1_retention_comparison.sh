#!/usr/bin/env bash
set -eo pipefail
root=/home/wheeltec/manual_native180_v1_3m_20261109
out=/home/wheeltec/manual_native180_backups/20261109_v1_3m/comparison_run2
bag=/home/wheeltec/rosbags/stationary_ground_20261021_131314/bag/bag_0.db3
test "$(cat "$root/build_exit_code")" = 0
exec 9>"$root/session.lock"
flock -n 9 || { echo 'New V1 session already running.' >&2; exit 1; }
test ! -e "$out"
mkdir "$out"
trap 'printf "%s\n" "$?" > "$out/exit_code"' EXIT
source /opt/ros/humble/setup.bash
source /home/wheeltec/zzj_ws/install/local_setup.bash
source "$root/ws/install/local_setup.bash"
export PYTHONDONTWRITEBYTECODE=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1
c++ -std=c++14 -O2 -Wall -Wextra -pedantic -I "$root/ws/src/fast_lio/src" "$root/verification/test_native_cloud_filters.cpp" -o "$out/test_geometry"
"$out/test_geometry" > "$out/geometry.log" 2>&1
python3 "$root/verification/check_projection_units.py" --runtime "$root" --projection-fixture "$root/verification/test_projection.py" --free-fixture "$root/verification/test_free_evidence.py" --library "$root/ws/install/native_height_projection/lib/native_height_projection/libfree_evidence.so" > "$out/projection_units.log" 2>&1

# Both arms use exactly the same rebuilt mapper, projector, observer and pacing.
# Only the explicit retention parameter differs. No old binary/log is touched.
python3 - "$root" "$out" <<'PY'
from pathlib import Path
import hashlib,json,shutil,sys,yaml
root,out=map(Path,sys.argv[1:])
source=root/'ws/install/native_height_projection'
relative=Path('share/native_height_projection/config/native180_fastlio.yaml')
original=(source/relative).read_text()
assert 'retention_percent: 100' in original
for percent in (70,100):
    prefix=out/'profiles'/f'p{percent}'
    shutil.copytree(source,prefix)
    config=original.replace('retention_percent: 100',f'retention_percent: {percent}')
    (prefix/relative).write_text(config)
    # Strictly prove all other installed config values remain identical.
    before=yaml.safe_load(original); after=yaml.safe_load(config)
    after['/**']['ros__parameters']['optimization']['retention_percent']=100
    assert before==after
(out/'comparison_design.json').write_text(json.dumps({
  'root':str(root),'mapper_sha256':hashlib.sha256((root/'ws/install/fast_lio/lib/fast_lio/fastlio_mapping').read_bytes()).hexdigest(),
  'single_variable':'optimization.retention_percent','arms':[70,100],
  'rate':1.0,'stage':1,'repetitions':3,'runtime_logging':False,
  'stationary_max_displacement_target_m':0.05,
  'timing_scope':'Python resource.getrusage(RUSAGE_CHILDREN) measures the whole replay job CPU, not mapper per-frame latency',
  'test_order':['p70_r1','p100_r1','p100_r2','p70_r2','p70_r3','p100_r3'],
},indent=2)+'\n')
PY

run_case() {
  local percent="$1" repeat="$2" domain="$3"
  local label="p${percent}_r${repeat}"
  if pgrep -f '[/](fastlio_mapping|livox_ros_driver2_node|native_height_projector.py)( |$)' >/dev/null; then
    echo 'Another sensor/mapping process is running; stop the comparison.' >&2
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
    env -i HOME=/home/wheeltec USER=wheeltec PATH=/usr/local/sbin:/usr/local/bin:/usr/sbin:/usr/bin:/sbin:/bin LANG=C.UTF-8 \
    ROS_DOMAIN_ID="$domain" ROS_LOCALHOST_ONLY=1 OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 PYTHONDONTWRITEBYTECODE=1 FAST_LIO_TIMECHAIN_DIAG=1 \
    bash --noprofile --norc -c '
      set -e
      source /opt/ros/humble/setup.bash
      source /home/wheeltec/zzj_ws/install/local_setup.bash
      source "$1/ws/install/local_setup.bash"
      export AMENT_PREFIX_PATH="$2/profiles/p$3:$AMENT_PREFIX_PATH"
      prefix=$(ros2 pkg prefix native_height_projection)
      test "$prefix" = "$2/profiles/p$3"
      test "$(ros2 pkg prefix fast_lio)" = "$1/ws/install/fast_lio"
      python3 "$1/verification/check_recorded_native180_fast.py" --runner "$1/run_session.py" --bag "$5" --stage 1 --retention-percent "$3" --rate 1.0 --output "$2/$4"
    ' native-v1-test "$root" "$out" "$percent" "$label" "$bag" > "$out/${label}.log" 2>&1
  python3 - "$out/$label/recorded_native180_result.json" <<'PY'
import json,sys
r=json.load(open(sys.argv[1]))
assert 'error' not in r,r
assert r['input_contract_verified'] and r['map_saved_and_reconstructed'],r
assert r['odometry']['frames_received']>600,r['odometry']
print(json.dumps({'result':sys.argv[1],'odometry':r['odometry'],'matching':r['matching']['verified_matching_frames']}),flush=True)
PY
  printf 'FINISH %s %s\n' "$label" "$(date -Is)"
}

run_case 70 1 83
run_case 100 1 83
run_case 100 2 83
run_case 70 2 83
run_case 70 3 83
run_case 100 3 83
printf 'ALL_SIX_CONTRACT_AND_SAVE_CHECKS_PASSED\n'
