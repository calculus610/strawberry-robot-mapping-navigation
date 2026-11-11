#!/usr/bin/env python3
"""Own one original mapping session, then run offline enhancement."""
import argparse
import fcntl
import hashlib
import json
import os
import re
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import tempfile
import time
import uuid

ROOT = Path(__file__).resolve().parent
NATIVE = Path('/home/wheeltec/manual_native180_20261022')
LIVE_OUTPUT = Path('/home/wheeltec/maps/manual_postprocess_test')
STOP = {'signal':None, 'count':0}
LOG = None

def emit(message):
    text = str(message)
    if LOG:
        LOG.write(text+'\n'); LOG.flush()
    try: print(text,flush=True)
    except (BrokenPipeError,OSError): pass

def atomic_json(path, data):
    temporary = path.with_suffix(path.suffix+'.tmp')
    with temporary.open('w') as file:
        json.dump(data,file,indent=2,allow_nan=False); file.write('\n')
        file.flush(); os.fsync(file.fileno())
    temporary.replace(path)

def signal_stop(sig,_):
    STOP['count'] += 1
    if STOP['signal'] is None: STOP['signal'] = sig

def proc_table():
    rows = {}
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit(): continue
        try:
            raw = (entry/'stat').read_text()
            fields = raw[raw.rfind(')')+2:].split()
            if fields[0] in ('Z','X'): continue
            rows[int(entry.name)] = (int(fields[1]),int(fields[19]))
        except (OSError,ValueError,IndexError): pass
    return rows

def rss_kib(pid):
    try:
        for line in Path('/proc',str(pid),'status').read_text().splitlines():
            if line.startswith('VmRSS:'): return int(line.split()[1])
    except (OSError,ValueError): pass
    return 0

class Owned:
    def __init__(self):
        self.identities={}; self.peak_rss={}; self.tokens={}
    def token(self,role):
        return self.tokens.setdefault(role,uuid.uuid4().hex)
    def capture(self, role, process=None):
        table = proc_table()
        known = self.identities.setdefault(role,{})
        selected = {pid for pid,start in known.items() if pid in table and table[pid][1]==start}
        marker=('POSTPROCESS_OWNER_TOKEN='+self.token(role)).encode()
        # A per-role inherited marker also finds a launch child reparented during startup.
        for pid in table:
            try:
                p=Path('/proc',str(pid))
                if p.stat().st_uid==os.getuid() and marker in (p/'environ').read_bytes().split(b'\0'):
                    selected.add(pid)
            except OSError: pass
        while True:
            extra = {pid for pid,(ppid,start) in table.items() if ppid in selected}
            if extra.issubset(selected): break
            selected |= extra
        for pid in selected: known[pid] = table[pid][1]
        self.peak_rss[role] = max(self.peak_rss.get(role,0),sum(rss_kib(pid) for pid in selected))
    def survivors(self, role):
        table = proc_table()
        return [pid for pid,start in self.identities.get(role,{}).items()
                if pid in table and table[pid][1]==start]
    def send(self,role,pid,sig):
        expected=self.identities.get(role,{}).get(pid)
        if expected is None: return
        fd=None
        try:
            fd=os.pidfd_open(pid,0)
            row=proc_table().get(pid)
            if row is not None and row[1]==expected:
                signal.pidfd_send_signal(fd,sig)
        except ProcessLookupError: pass
        finally:
            if fd is not None: os.close(fd)
    def cleanup(self,role):
        # Refresh ownership before each pass; never signal global process-name matches.
        for sig,wait in ((signal.SIGTERM,3),(signal.SIGKILL,1)):
            self.capture(role)
            for pid in self.survivors(role): self.send(role,pid,sig)
            until=time.monotonic()+wait
            while self.survivors(role) and time.monotonic()<until: time.sleep(.1)

def active_mapping():
    names={'fastlio_mapping','octomap_server_node','async_slam_toolbox_node',
           'height_projection_map','rack_obstacle_layer','livox_ros_driver2_node','native_height_projector.py'}
    found=[]
    for entry in Path('/proc').iterdir():
        if not entry.name.isdigit(): continue
        try:
            argv=(entry/'cmdline').read_bytes().split(b'\0')
            if any(Path(a.decode(errors='replace')).name in names for a in argv if a):
                found.append(int(entry.name))
        except (OSError,ValueError): pass
    return found

def protected_check():
    manifest=json.loads((ROOT/'protected_before.json').read_text())
    changed=[]
    for filename, expected in manifest['protected_files'].items():
        p=Path(filename)
        if not p.is_file() or hashlib.sha256(p.read_bytes()).hexdigest()!=expected: changed.append(filename)
    if changed: raise RuntimeError('Original files differ from verified baseline: '+repr(changed))
    if hashlib.sha256((ROOT/'stream_kernel.py').read_bytes()).hexdigest()!=manifest['kernel_sha256']:
        raise RuntimeError('Verified A005 kernel hash changed')
    return len(manifest['protected_files'])

def file_sha(path):
    digest=hashlib.sha256()
    with Path(path).open('rb') as stream:
        for block in iter(lambda:stream.read(1<<20),b''): digest.update(block)
    return digest.hexdigest()

def save_a005_checkpoint(session,status,source_hashes):
    """Freeze the verified A005 boundary independently of mutable session.json."""
    from conflict_verify import verify_a005
    report=verify_a005(session)
    evidence=status.get('final_save_evidence',{})
    if not evidence.get('native_save_after_stop') or not evidence.get('map_stamps_after_stop'):
        raise RuntimeError('A005 checkpoint needs confirmed final original saves')
    stats=json.loads((session/'original/projection_stats.json').read_text())
    if int(evidence['map_stamps_after_stop'][-1])!=stats['stamp_ns']:
        raise RuntimeError('A005 checkpoint final stamp mismatch')
    data={'status':'complete','phase':'a005_complete','session':str(session.resolve()),
          'report_sha256':file_sha(session/'report.json'),
          'original_map_sha256':file_sha(session/'original/map.pgm'),
          'a005_map_sha256':file_sha(session/'enhanced/map.pgm'),
          'a005_yaml_sha256':file_sha(session/'enhanced/map.yaml'),
          'recording_sha256':file_sha(session/'recording.json'),
          'observations_sha256':report['observations']['sha256'],
          'fixed_original_save_verified':True,'final_save_evidence':evidence,
          'source_sha256':source_hashes}
    # Link is atomic and non-overwriting, including a competing empty destination.
    fd,temporary=tempfile.mkstemp(prefix='.a005-checkpoint-',dir=session)
    try:
        with os.fdopen(fd,'w') as file:
            json.dump(data,file,indent=2,allow_nan=False);file.write('\n')
            file.flush();os.fsync(file.fileno())
        os.link(temporary,session/'a005_checkpoint.json')
    finally:
        Path(temporary).unlink(missing_ok=True)
    return data

def conflict_stage(session,status,source_hashes,recording,start,tick,poll_logs,owned,side_env,signal_count):
    """A new-stage failure keeps the already verified A005 selected."""
    from conflict_verify import verify_result, verify_a005
    status.update(a005_status='complete',enhanced_map=str(session/'enhanced/map.yaml'),
                  final_map=str(session/'enhanced/map.yaml'),conflict_status='pending')
    status['final_preview']=str(session/'comparison.png')
    status['preview_kind']='a005'
    status['phase']='enhancing_conflict'
    atomic_json(session/'session.json',status)
    child=None; interrupted=None; timed_out=False
    try:
        save_a005_checkpoint(session,status,source_hashes)
        if STOP['count']>signal_count:
            raise InterruptedError('Additional terminal signal before conflict enhancement')
        budget=max(120,min(3600,recording['frames']*.1))
        emit('CONFLICT_ENHANCEMENT_STARTED: checking recorded hits in original Free cells.')
        child=start('conflict',[sys.executable,'-B',str(ROOT/'offline_conflict.py'),
                               '--session',str(session),'--max-seconds',str(budget)],side_env)
        until=time.monotonic()+budget+5
        while child.poll() is None:
            if STOP['count']>signal_count:
                interrupted='Additional terminal signal during conflict enhancement'
                break
            if time.monotonic()>until:
                interrupted='Conflict enhancement exceeded its time budget'
                timed_out=True
                break
            tick()
        if interrupted:
            owned.cleanup('conflict')
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        poll_logs()
        # Commit may have completed immediately before a signal. Only full verification
        # can accept that result; exit code alone is not an artifact-integrity check.
        if (session/'enhanced_conflict').exists():
            result=verify_result(session)
            status.update(conflict_status='complete',final_map=str(session/'enhanced_conflict/map.yaml'),
                          conflict_report=str(session/'enhanced_conflict/report.json'),
                          conflict_changed_cells=result['invariants']['changed_cells'],
                          final_preview=None,preview_kind=None)
            if interrupted or child.returncode!=0:
                status['conflict_completion_note']='Verified complete commit survived worker interruption'
            emit('CONFLICT_ENHANCEMENT_COMPLETE: '+status['final_map'])
        else:
            if timed_out:raise TimeoutError(interrupted)
            if interrupted or STOP['count']>signal_count:
                raise InterruptedError(interrupted or 'Additional terminal signal at conflict boundary')
            raise RuntimeError('Conflict worker returned without a verified result (exit '+str(child.returncode)+')')
    except BaseException as error:
        if child is not None and child.poll() is None:
            owned.cleanup('conflict')
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        # A fallback is usable only while its own immutable inputs still verify.
        verify_a005(session)
        status.update(conflict_status=('cancelled' if isinstance(error,InterruptedError) else
                                      'timed_out' if isinstance(error,TimeoutError) else 'failed'),
                      conflict_error=repr(error),final_map=str(session/'enhanced/map.yaml'))
        emit('CONFLICT_FALLBACK_A005: '+repr(error))
    atomic_json(session/'session.json',status)
    return status['final_map']

def scatter_stage(session,status,recording,start,tick,poll_logs,owned,side_env,signal_count):
    """Filter isolated black components from either source; retain the conflict map on failure."""
    from conflict_verify import verify_result as verify_conflict
    from offline_scatter import verify_result as verify_scatter
    if status.get('conflict_status')!='complete':
        status['scatter_status']='skipped_upstream_fallback'
        atomic_json(session/'session.json',status)
        return status['final_map']
    fallback=str(session/'enhanced_conflict/map.yaml')
    verify_conflict(session)
    status.update(final_map=fallback,scatter_status='running',phase='enhancing_scatter',
                  final_preview=None,preview_kind=None)
    atomic_json(session/'session.json',status)
    child=None; interrupted=None; timed_out=False
    try:
        if STOP['count']>signal_count:
            raise InterruptedError('Additional terminal signal before scatter filtering')
        budget=max(120,min(3600,recording['frames']*.1))
        emit('SCATTER_FILTER_STARTED: checking isolated original and A005 obstacles; no new Free cells.')
        status['scatter_time_budget_seconds']=budget
        child=start('scatter',[sys.executable,'-B',str(ROOT/'offline_scatter.py'),
                              '--session',str(session),'--max-seconds',str(budget)],side_env)
        until=time.monotonic()+budget+5
        while child.poll() is None:
            if STOP['count']>signal_count:
                interrupted='Additional terminal signal during scatter filtering'; break
            if time.monotonic()>until:
                interrupted='Scatter filtering exceeded its time budget'; timed_out=True; break
            tick()
        if interrupted:
            owned.cleanup('scatter')
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        poll_logs()
        if (session/'enhanced_scatter').exists():
            result=verify_scatter(session)
            changed=result['invariants']['changed_cells']
            status.update(scatter_status='complete',final_map=str(session/'enhanced_scatter/map.yaml'),
                          scatter_report=str(session/'enhanced_scatter/report.json'),
                          scatter_changed_cells=changed)
            if interrupted or child.returncode!=0:
                status['scatter_completion_note']='Verified complete commit survived worker interruption'
            emit('SCATTER_FILTER_COMPLETE: '+str(changed)+' Occupied -> Unknown; '+status['final_map'])
        else:
            if timed_out:raise TimeoutError(interrupted)
            if interrupted or STOP['count']>signal_count:
                raise InterruptedError(interrupted or 'Additional terminal signal at scatter boundary')
            raise RuntimeError('Scatter worker returned without a verified result (exit '+str(child.returncode)+')')
    except BaseException as error:
        if child is not None and child.poll() is None:
            owned.cleanup('scatter')
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        verify_conflict(session)
        status.update(scatter_status=('cancelled' if isinstance(error,InterruptedError) else
                                     'timed_out' if isinstance(error,TimeoutError) else 'failed'),
                      scatter_error=repr(error),final_map=fallback)
        emit('SCATTER_FALLBACK_CONFLICT: '+repr(error))
    atomic_json(session/'session.json',status)
    return status['final_map']

def preview_stage(session,status,start,tick,owned,side_env,signal_count):
    """Preview failure cannot invalidate a successfully committed map."""
    if STOP['count']>signal_count:
        status['preview_status']='skipped_after_additional_signal'
        if status['final_map']!=str(session/'enhanced/map.yaml'):
            status['final_preview']=None
        return
    status.update(phase='rendering_final_preview',preview_status='running')
    # Never present the A005 image as a preview of the selected conflict map.
    status['final_preview']=None
    atomic_json(session/'session.json',status)
    child=None
    try:
        child=start('preview',[sys.executable,'-B',str(ROOT/'final_preview.py'),
                              '--session',str(session),'--map',status['final_map']],side_env)
        preview_budget=max(60,status.get('scatter_time_budget_seconds',0)+10)
        until=time.monotonic()+preview_budget
        while child.poll() is None:
            if STOP['count']>signal_count:raise InterruptedError('Final preview cancelled')
            if time.monotonic()>until:raise TimeoutError('Final preview exceeded its time budget')
            tick()
        if child.returncode!=0 or not (session/'final_comparison.png').is_file():
            raise RuntimeError('Final preview not saved')
        status.update(preview_status='complete',final_preview=str(session/'final_comparison.png'),
                      preview_kind='saved_stages_comparison')
    except BaseException as error:
        if child is not None and child.poll() is None:
            owned.cleanup('preview')
            try:child.wait(timeout=3)
            except subprocess.TimeoutExpired:pass
        status.update(preview_status='failed',preview_error=repr(error))
        if status['final_map']==str(session/'enhanced/map.yaml') and (session/'comparison.png').is_file():
            status.update(final_preview=str(session/'comparison.png'),preview_kind='a005')
        emit('FINAL_PREVIEW_UNAVAILABLE: '+repr(error))

def main():
    global LOG
    parser=argparse.ArgumentParser(description='Original mapping, one Ctrl+C, then A005 and Free-conflict refinement. No motion control.')
    parser.add_argument('--check',action='store_true',help='Read-only dependency and baseline check; start no ROS nodes')
    parser.add_argument('--no-preview',action='store_true',help='Save the final comparison without requesting a desktop image viewer')
    parser.add_argument('--isolated-test',action='store_true',help=argparse.SUPPRESS)
    parser.add_argument('--test-parent',type=Path,help=argparse.SUPPRESS)
    args=parser.parse_args()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP): signal.signal(sig,signal_stop)
    if args.isolated_test:
        host=os.environ.get('POSTPROCESS_HOST_NET_NS')
        if not host or os.readlink('/proc/self/ns/net')==host or os.environ.get('ROS_DOMAIN_ID')!='188' or os.environ.get('ROS_LOCALHOST_ONLY')!='1':
            raise RuntimeError('Replay mode requires isolated network namespace and ROS domain 188')
        parent=(args.test_parent or ROOT/'verification/replay_sessions').resolve()
        if not parent.is_relative_to(ROOT/'verification'): raise RuntimeError('Test output outside verification')
    else:
        if args.test_parent is not None: raise RuntimeError('--test-parent requires isolated replay')
        parent=LIVE_OUTPUT
        if os.environ.get('ROS_DOMAIN_ID')!='0': raise RuntimeError('Manual mode expects original ROS domain 0')
    if not hasattr(os,'pidfd_open') or not hasattr(signal,'pidfd_send_signal'):
        raise RuntimeError('PID-safe cleanup needs Linux pidfd support')
    count=protected_check()
    for filename in ('recorder.py','offline_enhance.py','stream_kernel.py','offline_conflict.py',
                     'conflict_core.py','ray_geometry.py','conflict_verify.py','offline_scatter.py','final_preview.py'):
        if not (ROOT/filename).is_file(): raise RuntimeError('Missing '+filename)
    import numpy, yaml, scipy, PIL, rclpy
    from livox_ros_driver2.msg import CustomMsg
    import offline_conflict, conflict_verify, offline_scatter, final_preview
    if args.check:
        print(json.dumps({'status':'ready','protected_files':count,'output_root':str(parent),
                          'disk_free_GiB':shutil.disk_usage(ROOT).free/1024**3,
                          'active_mapping_pids':active_mapping(),
                          'postprocess_stages':['A005','Free_conflict_v1','Isolated_scatter_v1'],
                          'final_map_relative':'enhanced_scatter/map.yaml',
                          'fallback_map_relative':'enhanced_conflict/map.yaml',
                          'upstream_fallback_map_relative':'enhanced/map.yaml',
                          'desktop_preview_launcher':shutil.which('xdg-open')},indent=2))
        return 0
    if active_mapping(): raise RuntimeError('A sensor/mapping process already runs; stop its original session first')
    if not args.isolated_test:
        if not Path('/tmp/.X11-unix/X0').is_socket() or not os.access('/run/user/1000/gdm/Xauthority',os.R_OK):
            raise RuntimeError('Robot local desktop DISPLAY=:0 or Xauthority unavailable')
    # Same runtime lock as the original launcher; original code/config remains untouched.
    lock=(NATIVE/'session.lock').open('a')
    fcntl.flock(lock,fcntl.LOCK_EX|fcntl.LOCK_NB)
    parent.mkdir(parents=True,exist_ok=True)
    if shutil.disk_usage(parent).free < 2*1024**3: raise RuntimeError('Less than 2 GiB free on output filesystem')
    session=Path(tempfile.mkdtemp(prefix=time.strftime('%Y%m%d_%H%M%S_'),dir=parent))
    original=session/'original'; original.mkdir()
    (session/'logs').mkdir()
    LOG=(session/'session.log').open('x')
    emit('SESSION_DIR='+str(session))
    emit('Original map: '+str(original/'map.yaml'))
    emit('Enhanced map after shutdown: '+str(session/'enhanced/map.yaml'))
    emit('Final map after conflict refinement: '+str(session/'enhanced_conflict/map.yaml'))
    emit('Final test map after scatter filtering: '+str(session/'enhanced_scatter/map.yaml'))
    emit('Keep stationary on flat ground until PROJECTION_READY. Drive manually. Ctrl+C ONCE, then WAIT.')
    for p in (NATIVE/'ws/src/native_height_projection/config').glob('*.yaml'):
        if not p.name.startswith('._'): shutil.copy2(p,original/p.name)
    shutil.copy2(NATIVE/'deployment_manifest.json',original/'deployment_manifest.json')
    source_hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in ROOT.iterdir()
                   if p.is_file() and p.suffix in ('.py','.sh')}
    status={'status':'running','phase':'starting','session':str(session),'start_epoch':time.time(),
            'isolated_test':args.isolated_test,'source_sha256':source_hashes,'protected_files':count}
    atomic_json(session/'session.json',status)
    env=os.environ.copy()
    env.update(ROS_LOG_DIR=str(session/'logs'),PYTHONDONTWRITEBYTECODE='1')
    side_env=dict(env,OPENBLAS_NUM_THREADS='1',OMP_NUM_THREADS='1')
    owned=Owned(); children={}; handles={}; tails={}; failure=None; success=False

    def start(role, command, environment):
        logpath=session/(role+'.log')
        handle=logpath.open('x'); handles[role]=handle
        process=subprocess.Popen(command,stdout=handle,stderr=subprocess.STDOUT,env=dict(environment,POSTPROCESS_OWNER_TOKEN=owned.token(role)),start_new_session=True)
        children[role]=process; owned.capture(role,process)
        tails[role]=logpath.open()
        return process

    def poll_logs():
        for role,reader in tails.items():
            for line in reader:
                if any(t in line for t in ('PROJECTION_READY','NATIVE_HEIGHT_REFERENCE_READY','MAP_SAVED',
                                           'SESSION_SAVED','FAILED','ERROR','ENHANCED_MAP_SAVED','RECORDING_CLOSED',
                                           'CONFLICT_MAP_SAVED','SCATTER_MAP_SAVED','FINAL_PREVIEW_SAVED')):
                    emit('['+role+'] '+line.rstrip())

    last_sample=0
    def tick():
        nonlocal last_sample
        poll_logs()
        if time.monotonic()-last_sample>=1:
            for role,child in children.items(): owned.capture(role,child)
            last_sample=time.monotonic()
        time.sleep(.1)

    def phase(value):
        status['phase']=value; atomic_json(session/'session.json',status)

    try:
        rec=start('recorder',[sys.executable,'-B',str(ROOT/'recorder.py'),'--session',str(session)],side_env)
        until=time.monotonic()+25
        while not (session/'recorder.ready.json').is_file():
            if rec.poll() is not None: raise RuntimeError('Recorder failed before ready')
            if STOP['signal'] is not None: raise RuntimeError('Interrupted before mapping started')
            if time.monotonic()>until: raise RuntimeError('Recorder readiness timeout')
            tick()
        phase('mapping')
        mapper=start('mapping',[sys.executable,'-B',str(NATIVE/'run_session.py'),str(original),
                                'start_lidar:='+('false' if args.isolated_test else 'true'),
                                'start_rviz:='+('false' if args.isolated_test else 'true')],env)
        stop_at=None
        stop_log_offset=None
        stop_files={}
        while mapper.poll() is None:
            if rec.poll() is not None and failure is None:
                failure='Recorder exited during mapping; keeping original output, refusing enhancement'
            if (STOP['signal'] is not None or failure) and stop_at is None:
                phase('saving_original'); emit('STOP_REQUESTED: waiting for original saves. Do not power off.')
                stop_log_offset=(session/'mapping.log').stat().st_size
                for name in ('map.pgm','map.yaml','projection_stats.json','native_map_final_camera_init.pcd'):
                    p=original/name
                    stop_files[name]=(p.stat().st_ino,p.stat().st_mtime_ns) if p.exists() else None
                mapper.send_signal(signal.SIGINT); stop_at=time.monotonic()
            if stop_at is not None and time.monotonic()-stop_at>180:
                raise RuntimeError('Original shutdown exceeded 180 seconds; enhancement refused')
            tick()
        owned.capture('mapping',mapper); poll_logs()
        if stop_at is None: failure=failure or 'Original mapping exited without requested stop'
        if mapper.returncode!=0: failure=failure or 'Original session exit code '+str(mapper.returncode)
        if owned.survivors('mapping'):
            until=time.monotonic()+3
            while owned.survivors('mapping') and time.monotonic()<until: tick()
            if owned.survivors('mapping'): raise RuntimeError('Original mapping descendants still running')
        phase('closing_recording')
        if rec.poll() is None: rec.send_signal(signal.SIGINT)
        until=time.monotonic()+20
        while rec.poll() is None:
            if time.monotonic()>until: raise RuntimeError('Recorder close timeout')
            tick()
        poll_logs()
        if rec.returncode!=0: failure=failure or 'Recorder did not close cleanly'
        required=['map.yaml','map.pgm','projection_stats.json','projection_source_camera_init.pcd',
                  'native_map_final_camera_init.pcd','free_evidence.npz']
        missing=[n for n in required if not (original/n).is_file()]
        if missing: failure=failure or 'Original save missing '+repr(missing)
        full_log=(session/'mapping.log').read_bytes()
        tail=full_log[stop_log_offset or 0:].decode(errors='replace')
        text=full_log.decode(errors='replace')
        if any(word in text for word in ('MAP_SAVE_FAILED','NATIVE_MAP_SAVE_FAILED','SESSION_SAVE_INCOMPLETE')):
            failure=failure or 'Original mapper reported a save failure'
        if re.search(r'\[(?:fastlio_mapping|native_height_projector.py)-\d+\]: process has died',text):
            failure=failure or 'A required original node crashed'
        marker='MAP_SAVED '+str(original/'map.yaml')+' stamp_ns='
        final_stamps=re.findall(re.escape(marker)+r'(\d+)',tail)
        if not final_stamps or 'NATIVE_MAP_SAVED' not in tail or 'SESSION_SAVED:' not in tail:
            failure=failure or 'No confirmed final 2D/3D saves after stop request'
        for name,before in stop_files.items():
            p=original/name
            if p.exists() and before is not None and (p.stat().st_ino,p.stat().st_mtime_ns)==before:
                failure=failure or 'Final output was not refreshed after stop: '+name
        if final_stamps and (original/'projection_stats.json').is_file():
            stats=json.loads((original/'projection_stats.json').read_text())
            if int(final_stamps[-1])!=stats.get('stamp_ns'):
                failure=failure or 'Final save log stamp differs from saved map metadata'
        status['final_save_evidence']={'stop_log_byte_offset':stop_log_offset,
                                      'map_stamps_after_stop':final_stamps,
                                      'native_save_after_stop':'NATIVE_MAP_SAVED' in tail}
        if failure: raise RuntimeError(failure)
        recording=json.loads((session/'recording.json').read_text())
        if recording.get('status')!='complete': raise RuntimeError('Recording integrity failed')
        status['recorded_frames']=recording['frames']; status['recorded_bytes']=recording['bytes']
        offline_signal_count=STOP['count']
        phase('enhancing')
        emit('OFFLINE_ENHANCEMENT_STARTED: mapping stopped; processing recorded observations.')
        offline=start('offline',[sys.executable,'-B',str(ROOT/'offline_enhance.py'),'--session',str(session)],side_env)
        offline_deadline=time.monotonic()+max(120,min(3600,recording['frames']*.1))
        while offline.poll() is None:
            if STOP['count']>offline_signal_count:
                raise RuntimeError('Offline enhancement cancelled by another terminal signal')
            if time.monotonic()>offline_deadline:
                raise RuntimeError('Offline enhancement exceeded its time budget')
            tick()
        poll_logs()
        report=json.loads((session/'report.json').read_text()) if (session/'report.json').is_file() else {}
        if offline.returncode!=0 or report.get('status')!='complete' or not (session/'enhanced/map.yaml').is_file():
            raise RuntimeError('Offline enhancement failed; original and observations retained')
        conflict_verify.verify_a005(session)
        protected_check()
        success=True
        conflict_stage(session,status,source_hashes,recording,start,tick,poll_logs,owned,side_env,offline_signal_count)
        scatter_stage(session,status,recording,start,tick,poll_logs,owned,side_env,offline_signal_count)
        preview_stage(session,status,start,tick,owned,side_env,offline_signal_count)
        protected_check()
        status.update(status='complete',phase='complete')
        atomic_json(session/'session.json',status)
        finish=('FINISHED: ' if status.get('scatter_status')=='complete' else
                'FINISHED_WITH_SCATTER_FALLBACK: ' if status.get('conflict_status')=='complete' else
                'FINISHED_WITH_A005_FALLBACK: ')
        emit(finish+status['final_map'])
        if status.get('final_preview'):emit('COMPARISON: '+status['final_preview'])
    except BaseException as error:
        failure=repr(error)
        success=False
        status.update(status='failed',error=failure)
        emit('SESSION_FAILED: '+failure)
        emit('Original files, verified A005 checkpoint if present, and recording retained in '+str(session))
    finally:
        for role,child in children.items():
            owned.capture(role,child)
            if child.poll() is None or owned.survivors(role):
                owned.cleanup(role)
                try: child.wait(timeout=3)
                except subprocess.TimeoutExpired: pass
        poll_logs()
        status.update(end_epoch=time.time(),stop_signal=STOP['signal'],stop_signal_count=STOP['count'],
                      sampled_peak_rss_kib_by_process_tree=owned.peak_rss,
                      resource_note='1 Hz sampled process-tree RSS; not system memory or exact peak PSS')
        atomic_json(session/'session.json',status)
        for reader in tails.values(): reader.close()
        for handle in handles.values(): handle.close()
        lock.close(); LOG.close(); LOG=None
    if success:
        if args.isolated_test or args.no_preview:
            status['desktop_preview']={'status':'disabled','reason':'isolated replay or --no-preview'}
        elif status.get('final_preview'):
            try:
                status['desktop_preview']=final_preview.request_local_preview(Path(status['final_preview']))
            except Exception as error:
                status['desktop_preview']={'status':'failed','reason':repr(error)}
        else:
            status['desktop_preview']={'status':'disabled','reason':'no verified preview image'}
        atomic_json(session/'session.json',status)
        emit('DESKTOP_PREVIEW: '+json.dumps(status['desktop_preview'],ensure_ascii=False))
    return 0 if success else 1

if __name__=='__main__':
    try: raise SystemExit(main())
    except Exception as e:
        emit('START_FAILED: '+repr(e)); raise SystemExit(1)
