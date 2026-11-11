#!/usr/bin/env python3
"""Start a NEW original mapping session, then clean only that completed session.

This wrapper owns only its direct children. The original coordinator owns and
stops the ROS processes. Never infer an input by sorting historical sessions.
"""
import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import subprocess
import tempfile
import time
import uuid


ROOT = Path(__file__).resolve().parent
ORIGINAL = Path('/home/wheeltec/manual_map_postprocess_test')
SESSIONS = Path('/home/wheeltec/maps/manual_postprocess_test')
RUNS = Path('/home/wheeltec/maps/manual_lab_cleanup_runs')
OUTPUT = Path('/home/wheeltec/maps/manual_lab_cleanup')
# The reviewed exec chain, signal protocol and SESSION_DIR marker are required.
LAUNCH_SHA = {
    'start_test.sh': 'e66fd13eb0f1b35434f7b623317d34367e68c70a524d9fb69e16814c87c926a4',
    'session.py': '23e1732a39629fbf55810f487aa698762b320ef48caa9efb59ff7556859e4523',
}
STOP = {'count': 0, 'disconnect': False}


def signal_request(signum, _frame):
    STOP['count'] += 1
    if signum != signal.SIGINT:
        STOP['disconnect'] = True


def emit(message):
    # Losing the SSH output must not prevent the original graceful save.
    try:
        print(message, flush=True)
    except (BrokenPipeError, OSError):
        pass


def require(condition, message):
    if not condition:
        raise RuntimeError(message)


def save_state(run, state):
    temporary = run / 'entry.json.tmp'
    with temporary.open('w') as stream:
        json.dump(state, stream, indent=2, ensure_ascii=False)
        stream.write('\n')
    temporary.replace(run / 'entry.json')


def run_child(command, log, stage, on_line, initial_signals=None):
    """Tail a file so terminal loss cannot block a child on a full stdout pipe."""
    if initial_signals is None:
        initial_signals = STOP['count']
    sent = False
    ready = stage != 'mapping'
    failure = None
    with log.open('x') as writer, log.open() as reader:
        child = subprocess.Popen(command, stdin=subprocess.DEVNULL,
                                 stdout=writer, stderr=subprocess.STDOUT,
                                 start_new_session=True)
        try:
            while True:
                # poll first, drain second: an exiting child may have final lines.
                exit_code = child.poll()
                for line in reader:
                    text = line.rstrip('\r\n')
                    emit(text)
                    if stage == 'mapping' and text.startswith('SESSION_DIR='):
                        # This marker is emitted only AFTER signal handlers exist.
                        ready = True
                    if failure is None:
                        try:
                            on_line(text)
                        except Exception as error:
                            failure = error
                if exit_code is not None:
                    if failure is not None:
                        raise failure
                    return exit_code
                requested = (STOP['count'] > 0 if stage == 'mapping'
                             else STOP['count'] > initial_signals)
                if ready and not sent and (requested or failure or STOP['disconnect']):
                    # Send only to the direct coordinator, never the ROS group.
                    child.send_signal(signal.SIGINT)
                    sent = True
                    if stage == 'mapping':
                        emit('正在结束本次建图并保存。请等待；保存期间再次 Ctrl+C 不会重复转发。')
                    else:
                        emit('已请求取消本次离线清理；原始地图保留。')
                time.sleep(0.1)
        finally:
            if child.poll() is None:
                # Unexpected wrapper failure still requests graceful coordinator
                # shutdown. Normal operation sends exactly one signal above.
                if not sent:
                    child.send_signal(signal.SIGINT)
                child.wait()


def preview(path):
    env = dict(os.environ)
    env.update(DISPLAY=':0', XAUTHORITY='/run/user/1000/gdm/Xauthority',
               XDG_RUNTIME_DIR='/run/user/1000',
               DBUS_SESSION_BUS_ADDRESS='unix:path=/run/user/1000/bus')
    env.pop('WAYLAND_DISPLAY', None)
    env.pop('POSTPROCESS_OWNER_TOKEN', None)
    try:
        viewer = subprocess.Popen(['/usr/bin/xdg-open', str(path)], env=env,
                                  stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL,
                                  stderr=subprocess.DEVNULL, start_new_session=True)
        emit('已请求在机器人桌面打开本次对比图。')
        return {'status': 'requested', 'pid': viewer.pid, 'path': str(path)}
    except OSError as error:
        emit('桌面预览未启动，请手动打开：' + str(path))
        return {'status': 'failed', 'error': str(error), 'path': str(path)}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--no-preview', action='store_true')
    parser.add_argument('--max-seconds', type=float, default=600)
    parser.add_argument('--protect-roi', action='append', default=[])
    parser.add_argument('--review-roi')
    args = parser.parse_args()
    if not 0 < args.max_seconds <= 3600:
        parser.error('--max-seconds must be in (0, 3600]')
    for value in args.protect_roi + ([args.review_roi] if args.review_roi else []):
        try:
            r0, r1, c0, c1 = map(int, value.split(','))
            if not 0 <= r0 < r1 or not 0 <= c0 < c1:
                raise ValueError
        except ValueError:
            parser.error('ROI must be nonnegative r0,r1,c0,c1 with increasing bounds')
    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, signal_request)
    for name, expected in LAUNCH_SHA.items():
        require(hashlib.sha256((ORIGINAL / name).read_bytes()).hexdigest() == expected,
                '原建图入口版本已改变，停止启动以免误用会话协议：' + name)
    for name in ('run_cleanup.sh', 'offline_cleanup.py', 'cleanup_core.py', 'preview_cleanup.py'):
        require((ROOT / name).is_file(), 'Missing cleanup file: ' + name)
    existing = {p.name for p in SESSIONS.iterdir()} if SESSIONS.exists() else set()
    RUNS.mkdir(parents=True, exist_ok=True)
    run = Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d_%H%M%S_'), dir=RUNS))
    state = {'status': 'running', 'phase': 'mapping', 'source_session': None,
             'start_epoch': time.time(), 'original_entry_sha256': LAUNCH_SHA,
             'automatic_navigation_selection': False}
    save_state(run, state)
    session = None

    def mapping_line(line):
        nonlocal session
        if not line.startswith('SESSION_DIR='):
            return
        require(session is None, 'Original coordinator emitted duplicate SESSION_DIR')
        path = Path(line.split('=', 1)[1])
        require(path.is_absolute() and path == path.resolve() and path.parent == SESSIONS
                and path.name not in existing and path.is_dir(),
                'Refusing unexpected or historical SESSION_DIR: ' + str(path))
        session = path
        state['source_session'] = str(session)
        save_state(run, state)
        emit('本次新建图目录：' + str(session))

    try:
        emit('开始全新建图。等 PROJECTION_READY 后手动行驶；结束时按一次 Ctrl+C，随后自动保存和清理。')
        emit('本次入口日志：' + str(run))
        code = run_child(['bash', str(ORIGINAL / 'start_test.sh'), '--no-preview'],
                         run / 'mapping.log', 'mapping', mapping_line)
        require(code == 0 and session is not None,
                '本次建图未成功完成；不会读取旧会话。查看 ' + str(run / 'mapping.log'))
        checkpoint = json.loads((session / 'session.json').read_text())
        require(checkpoint.get('session') == str(session)
                and checkpoint.get('status') == 'complete'
                and checkpoint.get('phase') == 'complete',
                '本次会话未完成保存，停止后续清理')
        require(checkpoint.get('final_map') in {
            str(session / 'enhanced_scatter/map.yaml'),
            str(session / 'enhanced_conflict/map.yaml')},
            '本次未生成 C/D 最终图；原流程结果保留，不改用历史地图')
        if STOP['disconnect']:
            state.update(status='stopped', phase='saved_without_cleanup')
            emit('终端断开或收到终止请求；本次原图已保存，未启动离线清理。')
            return 130
        # Capture BEFORE announcing this phase; a Ctrl+C after that announcement
        # must remain a cancellation even if it precedes the Popen call.
        cleanup_signals = STOP['count']
        destination = OUTPUT / (session.name + '_' + time.strftime('%Y%m%d_%H%M%S_') + uuid.uuid4().hex[:8])
        state.update(phase='cleanup', candidate_dir=str(destination))
        save_state(run, state)
        emit('本次新地图保存完成，开始离线旧占据撤销；此阶段再次 Ctrl+C 可取消清理。')
        command = ['bash', str(ROOT / 'run_cleanup.sh'), '--source-session', str(session),
                   '--output-dir', str(destination), '--max-seconds', str(args.max_seconds)]
        for value in args.protect_roi:
            command.extend(['--protect-roi', value])
        if args.review_roi:
            command.extend(['--review-roi', args.review_roi])
        code = run_child(command, run / 'cleanup.log', 'cleanup', lambda _line: None,
                         initial_signals=cleanup_signals)
        if code in (130, -signal.SIGINT):
            state.update(status='cancelled', phase='cleanup_cancelled')
            emit('本次离线清理已取消；本次原图保留：' + checkpoint['final_map'])
            return 130
        require(code == 0, '本次清理中止或失败；本次原图保留：' + str(session))
        report = json.loads((destination / 'report.json').read_text())
        require(report.get('status') == 'complete' and report.get('source_session') == str(session)
                and report.get('output_dir') == str(destination), 'Candidate source identity mismatch')
        comparison = destination / 'comparison.png'
        require(comparison.is_file() and (destination / 'map.yaml').is_file(), 'Candidate outputs missing')
        state.update(status='complete', phase='complete', comparison=str(comparison),
                     changed_cells=report['invariants']['changed_cells'])
        if not args.no_preview and not STOP['disconnect']:
            state['desktop_preview'] = preview(comparison)
        emit('本次新建图及候选输出完成，效果待你验收。')
        emit('本次原图：' + checkpoint['final_map'])
        emit('本次候选：' + str(destination / 'map.yaml'))
        emit('本次前后对比：' + str(comparison))
        return 0
    except Exception as error:
        state.update(status='failed', error=str(error))
        emit('本次流程未完成：' + str(error))
        emit('入口日志：' + str(run))
        return 1
    finally:
        state['end_epoch'] = time.time()
        save_state(run, state)


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except Exception as error:
        emit('启动失败：' + str(error))
        raise SystemExit(1)
