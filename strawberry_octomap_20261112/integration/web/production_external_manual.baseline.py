"""Recognize the existing SSH mapper without owning, starting or stopping it.

This is only a cleanup/rearm exception for a known, non-driving launch tree.
Task-start conflicts remain unchanged. Unknown or ambiguous processes fail closed.
"""
from collections import Counter, defaultdict
import json
import os
from pathlib import Path
import re
from stat import S_ISREG
from typing import NamedTuple


ROOTS = (
    Path('/home/wheeltec/manual_native180_restored_20261106'),
    Path('/home/wheeltec/manual_native180_v1_3m_20261109'),
)
SESSION_ROOT = Path('/home/wheeltec/maps/manual_native180')
NO_DOWNSAMPLE_ROOT = Path('/home/wheeltec/manual_no_downsample_20261112')
NO_DOWNSAMPLE_SESSION_ROOT = Path('/home/wheeltec/maps/manual_no_downsample')
NO_DOWNSAMPLE_LOCK = Path('/home/wheeltec/manual_native180_20261022/session.lock')
NO_DOWNSAMPLE_TOKEN = b'NO_DOWNSAMPLE_OWNER_TOKEN='
MANUAL_NODES = frozenset(('/laser_mapping', '/native_height_projector', '/livox_lidar_publisher'))
LIVOX_EXECUTABLE = Path('/home/wheeltec/zzj_ws/install/livox_ros_driver2/lib/livox_ros_driver2/livox_ros_driver2_node')
PYTHON_EXECUTABLE = Path('/usr/bin/python3')
ROS2_ENTRY = Path('/opt/ros/humble/bin/ros2')
PROCESS_ERRORS = (OSError, ValueError, IndexError, RuntimeError)
FLOCK = re.compile(r'lock:\s+\d+:\s+FLOCK\s+ADVISORY\s+WRITE\s+\d+\s+([\da-f]+):([\da-f]+):(\d+)\s+0\s+EOF')


class _Process(NamedTuple):
    path: Path
    parent: str
    started: int
    argv: list[str]


class ExternalManualSession:
    def __init__(self, roots=ROOTS, session_root=SESSION_ROOT, proc_root=Path('/proc')):
        self.roots = tuple(Path(root).resolve() for root in roots)
        self.session_root = Path(session_root).resolve()
        self.proc_root = Path(proc_root)

    @staticmethod
    def _read(path, limit=16384):
        with path.open('rb') as stream:
            data = stream.read(limit + 1)
        if len(data) > limit:
            raise ValueError('Oversize process metadata')
        return data

    def _process(self, pid):
        path = self.proc_root / str(pid)
        if path.stat().st_uid != os.getuid():
            raise ValueError('Different process owner')
        fields = self._read(path / 'stat').decode().rsplit(') ', 1)[1].split()
        if fields[0] in ('Z', 'X', 'x'):
            raise ValueError('Exited process')
        argv = self._read(path / 'cmdline').decode().rstrip('\0').split('\0')
        return _Process(path, fields[1], int(fields[19]), argv)

    def _holds_session_lock(self, path, lock_path):
        lock_file = lock_path.stat(follow_symlinks=False)
        descriptor = (path / 'fd/9').stat()
        identity = (lock_file.st_dev, lock_file.st_ino)
        if not S_ISREG(lock_file.st_mode) or (descriptor.st_dev, descriptor.st_ino) != identity:
            return False
        # flock(1) locks the inherited open file description. Its recorded PID
        # may be the exited helper, so bind the record to the device and inode.
        for line in self._read(path / 'fdinfo/9').decode().splitlines():
            match = FLOCK.fullmatch(line)
            if match and (os.makedev(int(match[1], 16), int(match[2], 16)), int(match[3])) == identity:
                return True
        return False

    def _session_launch(self, process, children):
        path, argv = process.path, process.argv
        if len(argv) < 4 or not re.fullmatch(r'python3(?:\.\d+)?', Path(argv[0]).name):
            return None
        runner = Path(argv[1])
        root = runner.parent
        if root not in self.roots or runner != root / 'run_session.py' or runner.resolve() != runner:
            return None
        session = Path(argv[2])
        if (session.parent != self.session_root or not re.fullmatch(r'native180_[A-Za-z0-9_]+', session.name)
                or not session.is_dir() or session.resolve() != session):
            return None
        env = dict(entry.split('=', 1) for entry in self._read(path / 'environ', 131072).decode().split('\0') if '=' in entry)
        stage = env.get('NATIVE_OPTIMIZATION_STAGE')
        if (stage not in ('0', '1', '2', '3') or env.get('EXPERIMENT_DIR') != str(root)
                or env.get('SESSION_DIR') != str(session)
                or env.get('ROS_DOMAIN_ID', '0') != os.environ.get('ROS_DOMAIN_ID', '0')):
            return None
        expected = f'optimization_stage:={stage}'
        extra = argv[3:]
        if (not extra or extra[0] != expected
                or any(arg not in ('start_rviz:=true', 'start_rviz:=false', 'start_lidar:=true', 'start_lidar:=false') for arg in extra[1:])):
            return None
        # The original launcher keeps the locked descriptor across exec.
        if not self._holds_session_lock(path, root / 'session.lock'):
            return None
        launches = []
        for child in children.get(path.name, ()):
            command = child.argv
            if (len(command) >= 7
                    and child.started >= process.started
                    and re.fullmatch(r'python3(?:\.\d+)?', Path(command[0]).name)
                    and Path(command[1]) == ROS2_ENTRY
                    and command[2:] == ['launch', 'native_height_projection', 'manual_native180.launch.py',
                                        f'session_dir:={session}', *extra]):
                launches.append(child)
        if len(launches) != 1:
            return None
        return root, launches[0]

    def _holds_no_downsample_lock(self, path):
        lock_file = NO_DOWNSAMPLE_LOCK.stat(follow_symlinks=False)
        if not S_ISREG(lock_file.st_mode):
            return False
        identity = lock_file.st_dev, lock_file.st_ino
        # This supervisor uses Python flock on a descriptor chosen by open().
        for fd in (path / 'fd').iterdir():
            try:
                descriptor = fd.stat()
                if (descriptor.st_dev, descriptor.st_ino) != identity:
                    continue
                fdinfo = self._read(path / 'fdinfo' / fd.name).decode().splitlines()
            except PROCESS_ERRORS:
                continue
            for line in fdinfo:
                match = FLOCK.fullmatch(line)
                if match and (os.makedev(int(match[1], 16), int(match[2], 16)), int(match[3])) == identity:
                    return True
        return False

    def _no_downsample_owner_token(self, process):
        entries = self._read(process.path / 'environ', 131072).split(b'\0')
        domain = os.environ.get('ROS_DOMAIN_ID', '0')
        domains = [value for value in entries if value.startswith(b'ROS_DOMAIN_ID=')]
        if domains != [f'ROS_DOMAIN_ID={domain}'.encode()]:
            return None
        tokens = [value[len(NO_DOWNSAMPLE_TOKEN):] for value in entries if value.startswith(NO_DOWNSAMPLE_TOKEN)]
        if len(tokens) != 1 or not re.fullmatch(rb'[0-9a-f]{32}', tokens[0]):
            return None
        return tokens[0]

    def _no_downsample_session_launch(self, process, children):
        runner = NO_DOWNSAMPLE_ROOT / 'session.py'
        supervisor_argv = [str(PYTHON_EXECUTABLE), '-B', str(runner)]
        if process.argv not in (supervisor_argv, [*supervisor_argv, '--no-rviz']):
            return None
        python = PYTHON_EXECUTABLE.resolve(strict=True)
        if runner.resolve(strict=True) != runner or (process.path / 'exe').resolve(strict=True) != python:
            return None

        start_rviz = 'start_rviz:=false' if process.argv[-1] == '--no-rviz' else 'start_rviz:=true'
        launches = []
        for child in children.get(process.path.name, ()):
            argv = child.argv
            if (len(argv) != 9 or child.started < process.started
                    or not re.fullmatch(r'python3(?:\.\d+)?', Path(argv[0]).name)
                    or argv[1:6] != [str(ROS2_ENTRY), 'launch', '--noninteractive',
                                     'native_height_projection', 'manual_native180.launch.py']
                    or not argv[6].startswith('session_dir:=')
                    or argv[7:] != [start_rviz, 'start_lidar:=true']
                    or (child.path / 'exe').resolve(strict=True) != python):
                continue
            launches.append(child)
        if len(launches) != 1:
            return None

        launch = launches[0]
        session = Path(launch.argv[6].removeprefix('session_dir:='))
        if (session.parent != NO_DOWNSAMPLE_SESSION_ROOT
                or not re.fullmatch(r'[0-9]{8}_[0-9]{6}_[a-z0-9_]{8}', session.name)
                or launch.argv[6] != f'session_dir:={session}'
                or not session.is_dir() or session.resolve(strict=True) != session):
            return None
        state_path = session / 'session.json'
        if not S_ISREG(state_path.stat(follow_symlinks=False).st_mode):
            return None
        state = json.loads(self._read(state_path))
        if (not isinstance(state, dict) or state.get('session') != str(session)
                or state.get('status') != 'running' or state.get('phase') not in ('starting', 'mapping')
                or type(state.get('child_pid')) is not int or state['child_pid'] != int(launch.path.name)
                or not self._holds_no_downsample_lock(process.path)):
            return None
        return NO_DOWNSAMPLE_ROOT, launch

    def _node_processes(self, launch, children, root, conflicts):
        install = root / 'ws/install'
        executables = {
            '/laser_mapping': install / 'fast_lio/lib/fast_lio/fastlio_mapping',
            '/native_height_projector': install / 'native_height_projection/lib/native_height_projection/native_height_projector.py',
            '/livox_lidar_publisher': LIVOX_EXECUTABLE,
        }
        expected = {name: executables[name].resolve(strict=True) for name in conflicts}
        python = PYTHON_EXECUTABLE.resolve(strict=True)
        pending = list(children.get(launch.path.name, ()))
        seen = {launch.path.name}
        matches = defaultdict(list)
        while pending:
            child = pending.pop()
            path, argv = child.path, child.argv
            if path.name in seen or child.started < launch.started:
                continue
            seen.add(path.name)
            pending.extend(children.get(path.name, ()))
            if not argv[0]:
                continue
            try:
                executable = (path / 'exe').resolve(strict=True)
            except OSError:
                continue
            for name, target in expected.items():
                if name == '/native_height_projector':
                    matched = executable == python and len(argv) > 1 and Path(argv[1]).resolve() == target
                else:
                    matched = executable == target
                if matched:
                    matches[name].append(child)
        if all(len(matches[name]) == 1 for name in conflicts):
            return [matches[name][0] for name in conflicts]
        return []

    def permits_rearm(self, conflicts, node_names):
        conflicts = set(conflicts)
        if not conflicts or not conflicts.issubset(MANUAL_NODES):
            return False
        names = Counter('/' + (namespace.strip('/') + '/' + name).strip('/') for name, namespace in node_names)
        if any(names[name] != 1 for name in conflicts):
            return False
        try:
            process_paths = list(self.proc_root.iterdir())
        except OSError:
            return False
        processes = []
        children = defaultdict(list)
        for path in process_paths:
            if not path.name.isdigit():
                continue
            try:
                process = self._process(path.name)
            except PROCESS_ERRORS:
                continue
            processes.append(process)
            children[process.parent].append(process)

        sessions = []
        for process in processes:
            try:
                launch = self._session_launch(process, children)
                if launch is None:
                    launch = self._no_downsample_session_launch(process, children)
                if launch is not None:
                    sessions.append((process, *launch))
            except PROCESS_ERRORS:
                continue
        if len(sessions) != 1:
            return False
        supervisor, root, launch = sessions[0]
        try:
            nodes = self._node_processes(launch, children, root, conflicts)
            if root == NO_DOWNSAMPLE_ROOT and nodes:
                token = self._no_downsample_owner_token(launch)
                if token is None or any(self._no_downsample_owner_token(node) != token for node in nodes):
                    return False
                if self._no_downsample_session_launch(supervisor, children) != (root, launch):
                    return False
            # Check for exits, reparenting or PID reuse during the scan.
            return bool(nodes and all(self._process(p.path.name) == p for p in (supervisor, launch, *nodes)))
        except PROCESS_ERRORS:
            return False
