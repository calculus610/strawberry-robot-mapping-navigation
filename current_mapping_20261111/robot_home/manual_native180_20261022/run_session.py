#!/usr/bin/env python3
"""Own the ROS launch session and forward terminal interruption exactly once."""
from pathlib import Path
import os
import shutil
import signal
import subprocess
import sys


def main():
    if len(sys.argv) < 2:
        raise SystemExit('Usage: run_session.py SESSION_DIR [launch arguments]')
    root = Path(__file__).resolve().parent
    session = Path(sys.argv[1]).resolve()
    # Separate the child process group from the terminal's foreground group.
    # Ctrl-C reaches this supervisor; ROS launch then signals its children once.
    child = subprocess.Popen(['ros2', 'launch', 'native_height_projection',
        'manual_native180.launch.py', f'session_dir:={session}', *sys.argv[2:]],
        start_new_session=True)
    stopping = False

    def stop(signum, frame):
        nonlocal stopping
        if stopping:
            return
        stopping = True
        print('STOP_REQUESTED: waiting for final 2D/3D saves; please keep power on.', flush=True)
        if child.poll() is None:
            child.send_signal(signal.SIGINT)

    for sig in (signal.SIGINT, signal.SIGTERM, signal.SIGHUP):
        signal.signal(sig, stop)
    result = child.wait()
    # The standalone experiment's original FastLIO debug streams are now closed.
    # Preserve them per session without changing FastLIO's file handling.
    debug = root/'ws/src/fast_lio/Log'
    if debug.is_dir():
        destination = session/'fastlio_logs'
        destination.mkdir(parents=True, exist_ok=True)
        for path in debug.iterdir():
            if path.is_file():
                shutil.copy2(path, destination/path.name)
    required = ['map.yaml', 'map.pgm', 'projection_source_camera_init.pcd',
                'native_map_final_camera_init.pcd', 'free_evidence.npz']
    missing = [name for name in required if not (session/name).is_file()]
    if missing:
        print(f'SESSION_SAVE_INCOMPLETE: {missing}; session={session}', flush=True)
    else:
        print(f'SESSION_SAVED: {session}', flush=True)
    return result if result != 0 else (1 if missing else 0)


if __name__ == '__main__':
    raise SystemExit(main())
