#!/usr/bin/env python3
"""Build or supervise one isolated strawberry OctoMap session on the robot."""
import argparse
from contextlib import ExitStack, contextmanager
import fcntl
import hashlib
import json
import os
from pathlib import Path
import signal
import stat
import subprocess
import time
import uuid

from memory_monitor import MemoryMonitor, OWNER_ENV, process_identity

ROOT = Path(__file__).resolve().parent
WORKSPACE = ROOT / 'ws'
RUNTIME = ROOT / 'runtime'
OUTPUTS = ROOT / 'outputs'
BUILD_STAMP = ROOT / 'build_stamp.json'
SHARED_LOCK = Path('/home/wheeltec/manual_native180_20261022/session.lock')
PACKAGES = ('livox_ros_driver2', 'fast_lio', 'strawberry_octomap')
EXECUTABLES = (
    'livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node',
    'fast_lio/lib/fast_lio/fastlio_mapping',
    'strawberry_octomap/lib/strawberry_octomap/occupancy_server',
    'strawberry_octomap/lib/strawberry_octomap/observation_bridge.py',
)
BUILD_OUTPUTS = (*EXECUTABLES, 'strawberry_octomap/share/strawberry_octomap/launch/mapping.launch.py',
                 'local_setup.bash')
BUSY_EXECUTABLES = frozenset((
    'fastlio_mapping', 'livox_ros_driver2_node', 'occupancy_server',
    'observation_bridge.py', 'native_height_projector.py', 'octomap_server_node',
    'async_slam_toolbox_node', 'height_projection_map', 'rack_obstacle_layer',
    'controller_server', 'planner_server', 'amcl',
))
FINALIZE_SECONDS = 120
READY_MARKERS = ('OBSERVATION_READY', 'OCTOMAP_MAP_READY')
REFERENCE_CHANGED = 'OBSERVATION_REFERENCE_CHANGED'
VISIBLE_LOG_MARKERS = ('WARN', 'ERROR', 'FAILED', 'WRONG_', 'WAIT', 'OBSERVATION_STATUS',
                       'OCTOMAP_PROGRESS', REFERENCE_CHANGED)
STOP = None

MAPPING_COMMAND = r'''
set -e
source /opt/ros/humble/setup.bash
source "$1/ws/install/local_setup.bash"
for package in livox_ros_driver2 fast_lio strawberry_octomap; do
    test "$(/opt/ros/humble/bin/ros2 pkg prefix "$package")" = "$1/ws/install/$package" || {
        echo "WRONG_PACKAGE_PREFIX=$package" >&2
        exit 1
    }
done
exec /opt/ros/humble/bin/ros2 launch --noninteractive strawberry_octomap mapping.launch.py \
    "output_dir:=$2" "start_rviz:=$3" "publish_web_map:=$4"
'''

CHECK_MAP_COMMAND = r'''
set -e
source /opt/ros/humble/setup.bash
exec /usr/bin/python3 -B "$1/session.py" --check-map-topic
'''

FINALIZE_COMMAND = r'''
set -e
source /opt/ros/humble/setup.bash
source "$1/ws/install/local_setup.bash"
exec /usr/bin/python3 -B "$1/scripts/finalize_map.py" --timeout "$2"
'''

BUILD_COMMAND = r'''
set -e
source /opt/ros/humble/setup.bash
export CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1
exec colcon build --packages-select livox_ros_driver2 fast_lio strawberry_octomap \
    --executor sequential --parallel-workers 1 --event-handlers console_direct+ \
    --cmake-clean-first --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
    -DROS_EDITION=ROS2 -DDISTRO_ROS=humble
'''


def emit(message):
    try:
        print(message, flush=True)
    except (BrokenPipeError, OSError):
        pass


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def request_stop(signum, _frame):
    global STOP
    if STOP is None:
        STOP = signum


@contextmanager
def exclusive_lock(path, create=False):
    flags = os.O_NOFOLLOW | os.O_CLOEXEC | (os.O_WRONLY | os.O_CREAT if create else os.O_RDONLY)
    try:
        fd = os.open(path, flags, 0o600)
    except FileNotFoundError as error:
        raise RuntimeError(f'找不到共享互斥锁 {path}，请先核对原建图安装') from error
    try:
        require(stat.S_ISREG(os.fstat(fd).st_mode), f'互斥锁不是普通文件 {path}')
        try:
            fcntl.flock(fd, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            raise RuntimeError(f'已有会话持有互斥锁 {path}') from error
        yield
    finally:
        os.close(fd)


def sha(path):
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def atomic_json(path, data):
    temp = path.with_suffix(path.suffix + '.tmp')
    with temp.open('w', encoding='utf-8') as stream:
        json.dump(data, stream, ensure_ascii=False, indent=2)
        stream.write('\n')
        stream.flush()
        os.fsync(stream.fileno())
    temp.replace(path)


def source_fingerprint():
    source = WORKSPACE / 'src'
    files = {}
    for directory, subdirs, names in os.walk(source):
        subdirs[:] = [name for name in subdirs if name not in ('Log', '__pycache__', '.git')]
        for name in names:
            path = Path(directory) / name
            if not path.is_file() or name.endswith(('.pyc', '.pyo')) or name == '.DS_Store':
                continue
            require(path.resolve().is_relative_to(source), f'源码指向独立工作空间之外 {path}')
            files[str(path.relative_to(source))] = sha(path)
    require(bool(files), '独立工作空间源码为空')
    return hashlib.sha256(json.dumps(files, sort_keys=True).encode()).hexdigest()


def installed_hashes():
    install = WORKSPACE / 'install'
    for name in BUILD_OUTPUTS:
        path = install / name
        require(path.is_file() and path.resolve().is_relative_to(install), f'缺少独立安装产物 {path}')
        if name in EXECUTABLES:
            require(os.access(path, os.X_OK), f'安装产物不可执行 {path}')
    result = {}
    for root in (install / name for name in PACKAGES):
        for path in root.rglob('*'):
            if path.is_file() and '__pycache__' not in path.parts and path.suffix not in ('.pyc', '.pyo'):
                require(path.resolve().is_relative_to(install), f'安装产物指向独立目录之外 {path}')
                result[str(path.relative_to(install))] = sha(path)
    for path in install.iterdir():
        if path.is_file():
            require(path.resolve().is_relative_to(install), f'安装产物指向独立目录之外 {path}')
            result[path.name] = sha(path)
    return result


def active_mapping():
    found = []
    for item in Path('/proc').iterdir():
        if not item.name.isdecimal():
            continue
        try:
            arguments = (item / 'cmdline').read_bytes().split(b'\0')
        except OSError:
            continue
        if any(Path(os.fsdecode(arg)).name in BUSY_EXECUTABLES for arg in arguments[:3] if arg):
            found.append(int(item.name))
    return sorted(found)


def build_only():
    with exclusive_lock(RUNTIME / 'session.lock', create=True):
        require(not active_mapping(), '当前存在建图或导航进程，请先正常结束')
        fingerprint = source_fingerprint()
        BUILD_STAMP.unlink(missing_ok=True)
        (WORKSPACE / 'src/fast_lio/Log').mkdir(exist_ok=True)
        result = subprocess.run(['/bin/bash', '--noprofile', '--norc', '-c', BUILD_COMMAND],
                                cwd=WORKSPACE, check=False)
        require(result.returncode == 0, '独立构建失败，请检查 colcon 日志')
        require(source_fingerprint() == fingerprint, '构建期间源码发生变化')
        atomic_json(BUILD_STAMP, {'source_fingerprint': fingerprint,
                                 'installed_sha256': installed_hashes(), 'built_epoch': time.time()})
        emit(f'BUILD_COMPLETE={BUILD_STAMP}')
        return 0


def verify_build():
    require(BUILD_STAMP.is_file(), '缺少已验证构建，请先执行 scripts/build_robot.sh')
    stamp = json.loads(BUILD_STAMP.read_text(encoding='utf-8'))
    fingerprint = source_fingerprint()
    require(stamp.get('source_fingerprint') == fingerprint
            and stamp.get('installed_sha256') == installed_hashes(),
            '源码或安装产物与构建记录不一致，请重新准备独立构建')
    return fingerprint


def check_map_topic():
    import rclpy

    rclpy.init(args=[])
    node = None
    try:
        node = rclpy.create_node('strawberry_octomap_map_guard')
        deadline = time.monotonic() + 2.0
        while time.monotonic() < deadline:
            require(node.count_publishers('/map') == 0, '检测到现有 /map 发布者，不能启用 Web 地图')
            rclpy.spin_once(node, timeout_sec=0.1)
        require(node.count_publishers('/map') == 0, '检测到现有 /map 发布者，不能启用 Web 地图')
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def child_command(script, output, *args):
    return ['/bin/bash', '--noprofile', '--norc', '-c', script, 'strawberry-octomap',
            str(ROOT), str(output), *args]


def report_status(output, status):
    atomic_json(output / 'session.json', status)
    atomic_json(RUNTIME / 'session.json', status)


def send_owned_interrupt(pid, starttime):
    try:
        fd = os.pidfd_open(pid)
    except ProcessLookupError:
        return
    try:
        identity = process_identity(Path('/proc') / str(pid))
        if identity is not None and identity[1] == starttime:
            signal.pidfd_send_signal(fd, signal.SIGINT)
    except ProcessLookupError:
        pass
    finally:
        os.close(fd)


def sample(monitor, status):
    if not status.get('memory_recording_error'):
        try:
            monitor.sample(status['phase'])
        except (OSError, ValueError, RuntimeError) as error:
            status['memory_recording_error'] = str(error)
            emit(f'MEMORY_RECORDING_FAILED={error}')


def watch(output, status, environment, monitor):
    mapping_log = output / 'mapping.log'
    markers = set()
    finalizer = None
    save_started = None
    shutdown_sent = False
    orphan_signals = set()
    next_orphan_check = 0.0
    with mapping_log.open('x', encoding='utf-8') as writer, mapping_log.open(encoding='utf-8', errors='replace') as reader, (
            output / 'finalize.log').open('x', encoding='utf-8') as finalize_log:
        launch = subprocess.Popen(child_command(MAPPING_COMMAND, output, str(status['rviz']).lower(),
                                                 str(status['web_map']).lower()),
                                  cwd=output, env=environment, stdin=subprocess.DEVNULL,
                                  stdout=writer, stderr=subprocess.STDOUT, start_new_session=True)
        try:
            identity = process_identity(Path('/proc') / str(launch.pid))
            status.update(child_pid=launch.pid, child_starttime=identity[1] if identity else None)
            report_status(output, status)
            while True:
                for _ in range(150):
                    line = reader.readline()
                    if not line:
                        break
                    detected = [key for key in READY_MARKERS if key in line]
                    markers.update(detected)
                    if REFERENCE_CHANGED in line:
                        status['mapping_fatal_error'] = REFERENCE_CHANGED
                    if detected or any(marker in line for marker in VISIBLE_LOG_MARKERS):
                        emit(line.rstrip())
                now = time.monotonic()
                code = launch.poll()
                if code is None and status['phase'] == 'starting' and len(markers) == len(READY_MARKERS):
                    status['phase'] = 'mapping'
                    report_status(output, status)
                if code is None and STOP is not None and save_started is None:
                    status.update(phase='saving', stop_signal=STOP)
                    report_status(output, status)
                    emit('STOP_REQUESTED：正在封口并保存地图，请等待完成。')
                    save_started = now
                    # The finalizer does not inherit the foreground terminal's signals.
                    finalizer = subprocess.Popen(
                        ['/bin/bash', '--noprofile', '--norc', '-c', FINALIZE_COMMAND, 'strawberry-finalize',
                         str(ROOT), str(FINALIZE_SECONDS)],
                        cwd=output, env=environment, stdin=subprocess.DEVNULL, stdout=finalize_log,
                        stderr=subprocess.STDOUT, start_new_session=True)
                sample(monitor, status)
                if finalizer is not None and 'finalize_exit_code' not in status:
                    final_code = finalizer.poll()
                    if final_code is None and now - save_started > FINALIZE_SECONDS + 10 and not status.get('finalize_timed_out'):
                        # Bound only the request helper. The launch stops via SIGINT.
                        finalizer.kill()
                        status['finalize_timed_out'] = True
                    if final_code is not None:
                        status['finalize_exit_code'] = final_code
                        if code is None:
                            launch.send_signal(signal.SIGINT)
                        shutdown_sent = True
                finalizer_done = finalizer is None or finalizer.poll() is not None
                if code is not None and finalizer_done and now >= next_orphan_check:
                    next_orphan_check = now + 1.0
                    remaining = monitor.owned_identities()
                    remaining.pop(os.getpid(), None)
                    if not remaining:
                        return code
                    for pid, started in remaining.items():
                        if (pid, started) not in orphan_signals:
                            send_owned_interrupt(pid, started)
                            orphan_signals.add((pid, started))
                time.sleep(0.1)
        finally:
            try:
                if finalizer is not None and finalizer.poll() is None:
                    finalizer.kill()
            finally:
                if launch.poll() is None and not shutdown_sent:
                    launch.send_signal(signal.SIGINT)
            while launch.poll() is None:
                sample(monitor, status)
                time.sleep(0.2)
            while True:
                remaining = monitor.owned_identities()
                remaining.pop(os.getpid(), None)
                if not remaining:
                    break
                for pid, started in remaining.items():
                    if (pid, started) not in orphan_signals:
                        send_owned_interrupt(pid, started)
                        orphan_signals.add((pid, started))
                sample(monitor, status)
                time.sleep(0.5)


def validate_outputs(output):
    for name in ('map.pgm', 'map.yaml', 'octree.ot', 'metadata.json'):
        path = output / name
        require(not path.is_symlink() and path.is_file() and path.stat().st_size > 0,
                f'缺少本次保存产物 {path}')
    metadata = json.loads((output / 'metadata.json').read_text(encoding='utf-8'))
    require(isinstance(metadata, dict), '地图元数据格式无效')


def run_mapping(no_rviz, web_map):
    require(os.environ.get('ROS_DOMAIN_ID') == '0' and os.environ.get('ROS_LOCALHOST_ONLY') == '1',
            '请通过 start_mapping.sh 使用本机 ROS 域')
    require(hasattr(os, 'pidfd_open') and hasattr(signal, 'pidfd_send_signal'),
            '机器人 Python/Linux 缺少 PID 安全信号接口')
    with ExitStack() as locks:
        locks.enter_context(exclusive_lock(RUNTIME / 'session.lock', create=True))
        locks.enter_context(exclusive_lock(SHARED_LOCK))
        busy = active_mapping()
        require(not busy, f'已有建图、传感器或导航进程，请先正常结束 {busy}')
        fingerprint = verify_build()
        if not no_rviz:
            require(Path('/tmp/.X11-unix/X0').is_socket() and os.access(os.environ['XAUTHORITY'], os.R_OK),
                    '机器人桌面不可用，可通过 --no-rviz 启动')
        if web_map:
            guard = subprocess.run(['/bin/bash', '--noprofile', '--norc', '-c', CHECK_MAP_COMMAND,
                                    'strawberry-map-guard', str(ROOT)], timeout=10, check=False)
            require(guard.returncode == 0, '未能确认 /map 无现存发布者，Web 地图未启用')
        (WORKSPACE / 'src/fast_lio/Log').mkdir(exist_ok=True)
        token = uuid.uuid4().hex
        output = OUTPUTS / token
        output.mkdir(parents=True)
        (output / 'logs').mkdir()
        environment = dict(os.environ, **{OWNER_ENV: token}, ROS_LOG_DIR=str(output / 'logs'))
        identity = process_identity(Path('/proc/self'))
        require(identity is not None, '无法读取协调器进程身份')
        status = {'status': 'running', 'phase': 'starting', 'session': str(output),
                  'session_id': token, 'supervisor_pid': os.getpid(), 'supervisor_starttime': identity[1],
                  'rviz': not no_rviz, 'web_map': web_map, 'source_fingerprint': fingerprint,
                  'started_epoch': time.time()}
        report_status(output, status)
        emit(f'SESSION_DIR={output}')
        emit('等 OBSERVATION_READY 和 OCTOMAP_MAP_READY 后开始手动建图。结束按一次 Ctrl+C。')
        monitor = None
        try:
            require(STOP is None, '建图启动已取消')
            busy = active_mapping()
            require(not busy, f'检测到并行任务，停止启动 {busy}')
            monitor = MemoryMonitor(output, token, os.getpid(), sample_interval_seconds=10.0)
            code = watch(output, status, environment, monitor)
            status['launch_exit_code'] = code
            # The launch may exit with unread lines still in the log.
            with (output / 'mapping.log').open(encoding='utf-8', errors='replace') as stream:
                if any(REFERENCE_CHANGED in line for line in stream):
                    status['mapping_fatal_error'] = REFERENCE_CHANGED
            require('mapping_fatal_error' not in status,
                    '固定地面参考在建图期间改变，保存文件不能标记为完整地图')
            require(STOP is not None, '建图进程提前退出，查看 mapping.log')
            require(status.get('finalize_exit_code') == 0 and not status.get('finalize_timed_out'),
                    '地图保存失败，查看 finalize.log')
            validate_outputs(output)
            require(code == 0, '地图已写出，ROS launch 异常退出，请检查 mapping.log')
            status.update(status='complete', phase='complete')
        except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
            status.update(status='failed', phase='failed', error=str(error))
        finally:
            status['finished_epoch'] = time.time()
            if monitor is not None:
                try:
                    monitor.finish(status)
                except (OSError, ValueError, RuntimeError) as error:
                    status['memory_summary_error'] = str(error)
                    emit(f'MEMORY_SUMMARY_FAILED={error}')
            report_status(output, status)
        if status['status'] == 'complete':
            emit(f'MAP={output / "map.yaml"}')
            emit(f'SESSION_SAVED={output}')
            return 0
        emit(f'SESSION_FAILED={status["error"]}')
        return 1


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--build-only', action='store_true')
    parser.add_argument('--check-map-topic', action='store_true', help=argparse.SUPPRESS)
    parser.add_argument('--no-rviz', action='store_true')
    parser.add_argument('--web-map', action='store_true', help='显式发布额外的 /map 供网页显示')
    args = parser.parse_args()
    require(Path('/proc/self/stat').is_file(), '此入口需要机器人 Linux procfs')
    if args.check_map_topic:
        return check_map_topic()
    RUNTIME.mkdir(exist_ok=True)
    if args.build_only:
        return build_only()
    for signum in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(signum, request_stop)
    return run_mapping(args.no_rviz, args.web_map)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, ValueError, RuntimeError, subprocess.SubprocessError) as error:
        emit(f'STRAWBERRY_OCTOMAP_FAILED={error}')
        raise SystemExit(1)
