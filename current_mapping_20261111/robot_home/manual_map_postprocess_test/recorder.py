#!/usr/bin/env python3
"""Record exact-stamp observations without performing map enhancement."""
import argparse
import hashlib
import json
import os
from pathlib import Path
import resource
import shutil
import signal
import struct
import sys
import time
import traceback

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.signals import SignalHandlerOptions
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped

NATIVE_LIB = Path('/home/wheeltec/manual_native180_20261022/ws/install/native_height_projection/lib/native_height_projection')
sys.path.insert(0, str(NATIVE_LIB))
from projection_core import rotation_matrix

HEADER = struct.Struct('<qI7d')
MAX_FRAME_POINTS = 200_000
MAX_PENDING = 40
MAX_PENDING_BYTES = 16 * 1024 * 1024
DISK_RESERVE = 1024**3

def atomic_json(path, value):
    temp = path.with_suffix(path.suffix + '.tmp')
    with temp.open('w') as file:
        json.dump(value, file, indent=2, allow_nan=False)
        file.write('\n'); file.flush(); os.fsync(file.fileno())
    temp.replace(path)

def stamp(msg):
    return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

def xyz_bytes(msg):
    fields = {f.name: f for f in msg.fields}
    if any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1 for n in ('x', 'y', 'z')):
        raise ValueError('Expected float32 XYZ')
    n = int(msg.width) * int(msg.height)
    if n <= 0 or n > MAX_FRAME_POINTS:
        raise ValueError('Invalid or oversized point frame')
    if msg.point_step < 12 or msg.row_step < msg.width * msg.point_step or len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Truncated or invalid PointCloud2')
    if any(fields[n].offset + 4 > msg.point_step for n in ('x', 'y', 'z')):
        raise ValueError('Point field outside point_step')
    dtype = np.dtype(dict(names=['x','y','z'], formats=[('>' if msg.is_bigendian else '<')+'f4']*3,
                          offsets=[fields[n].offset for n in ('x','y','z')], itemsize=msg.point_step))
    src = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                     strides=(msg.row_step, msg.point_step))
    xyz = np.stack([src[k].reshape(-1) for k in ('x','y','z')], axis=1).astype('<f4', copy=False)
    if not np.isfinite(xyz).all():
        raise ValueError('Nonfinite observation point; refusing silent filtering')
    return n, xyz.tobytes()

class Recorder(Node):
    def __init__(self, out):
        super().__init__('manual_postprocess_recorder')
        self.out = out
        self.file = (out/'observations.bin').open('xb')
        self.digest = hashlib.sha256()
        self.bytes = 0
        self.put(b'A005OBS1')
        self.scans = {}; self.poses = {}; self.pending_bytes = 0
        self.frames = self.points = 0
        self.received = {'scan':0, 'pose':0}
        self.last_seen = {'scan':-1, 'pose':-1}
        self.first = self.last = None
        self.reference = None
        self.errors = []
        self.max_pending = {'scan':0, 'pose':0, 'bytes':0}
        self.max_gap_ns = 0; self.gaps_over_250ms = 0
        self.last_received_wall = time.monotonic()
        self.last_flush = time.monotonic()
        self.stop_requested = None
        self.abnormal_signal = None
        q = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                            durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(PointCloud2, '/fastlio/free_scan', lambda m:self.receive('scan',m), q)
        self.create_subscription(PoseStamped, '/fastlio/free_scan_pose', lambda m:self.receive('pose',m), q)
        self.create_subscription(PoseStamped, '/fastlio/native_height_reference', self.on_reference, latched)
        atomic_json(out/'recorder.ready.json', {'pid':os.getpid(), 'format':'A005OBS1', 'ready_epoch':time.time()})

    def put(self, data):
        self.file.write(data); self.digest.update(data); self.bytes += len(data)

    def fail(self, message):
        if not self.errors:
            self.errors.append(str(message))
            print('RECORDING_ERROR '+str(message), flush=True)

    def on_reference(self, msg):
        try:
            if msg.header.frame_id != 'camera_init':
                raise ValueError('Unexpected reference frame')
            p,q = msg.pose.position,msg.pose.orientation
            ground = [p.x,p.y,p.z]
            rot = rotation_matrix([q.x,q.y,q.z,q.w])
            if not np.isfinite(ground).all():
                raise ValueError('Nonfinite ground')
            ref = dict(frame_id=msg.header.frame_id, ground=ground, rotation=rot.tolist(), stamp_ns=stamp(msg))
            if self.reference is not None:
                if not all(np.allclose(ref[k], self.reference[k], atol=1e-9, rtol=0) for k in ('ground','rotation')):
                    raise ValueError('Fixed ground reference changed')
            else:
                self.reference = ref
        except Exception as e:
            self.fail(e)

    def receive(self, kind, msg):
        if self.errors: return
        try:
            expected = 'native_scan_lidar' if kind == 'scan' else 'camera_init'
            if msg.header.frame_id != expected:
                raise ValueError(kind+' frame mismatch: '+msg.header.frame_id)
            t = stamp(msg)
            if t <= self.last_seen[kind]:
                raise ValueError(kind+' duplicate or nonmonotonic timestamp')
            self.last_seen[kind] = t
            self.received[kind] += 1
            self.last_received_wall = time.monotonic()
            if kind == 'scan':
                n,data = xyz_bytes(msg)
                self.scans[t] = (n,data); self.pending_bytes += len(data)
            else:
                p,q = msg.pose.position,msg.pose.orientation
                pose = (p.x,p.y,p.z,q.x,q.y,q.z,q.w)
                if not np.isfinite(pose).all() or np.linalg.norm(pose[3:]) < 1e-9:
                    raise ValueError('Invalid paired pose')
                self.poses[t] = pose
            self.flush_pairs()
            for key, queue in [('scan',self.scans),('pose',self.poses)]:
                self.max_pending[key] = max(self.max_pending[key],len(queue))
            self.max_pending['bytes'] = max(self.max_pending['bytes'],self.pending_bytes)
            if max(len(self.scans),len(self.poses)) > MAX_PENDING or self.pending_bytes > MAX_PENDING_BYTES:
                raise ValueError('Pairing queue limit exceeded; no frames silently discarded')
        except Exception as e:
            self.fail(e)

    def flush_pairs(self):
        while self.scans or self.poses:
            t = min(min(self.scans, default=2**63), min(self.poses, default=2**63))
            if t not in self.scans or t not in self.poses: break
            n,data = self.scans.pop(t); pose = self.poses.pop(t)
            self.pending_bytes -= len(data)
            if self.last is not None and t <= self.last:
                raise ValueError('Nonmonotonic paired stream')
            if self.last is not None:
                gap = t-self.last
                self.max_gap_ns = max(self.max_gap_ns,gap)
                self.gaps_over_250ms += gap > 250_000_000
            self.put(HEADER.pack(t,n,*pose)); self.put(data)
            self.frames += 1; self.points += n
            if self.first is None: self.first = t
            self.last = t

    def maintenance(self):
        now = time.monotonic()
        if now-self.last_flush >= 1:
            self.file.flush(); os.fsync(self.file.fileno())
            self.last_flush = now
            if shutil.disk_usage(self.out).free < DISK_RESERVE:
                self.fail('Less than 1 GiB free disk; stopping recording and requesting mapper shutdown')

    def finish(self):
        try:
            self.flush_pairs()
            if self.scans or self.poses: self.fail('Unpaired observations remain at shutdown')
            if not self.frames: self.fail('No observations were recorded')
            if self.reference is None: self.fail('Missing fixed ground reference')
            self.file.flush(); os.fsync(self.file.fileno())
        except Exception as e:
            self.fail(e)
        finally:
            self.file.close()
        warnings = ['BEST_EFFORT DDS: matching scan/pose streams cannot alone prove zero simultaneous losses.']
        if self.gaps_over_250ms:
            warnings.append('Paired timestamp gaps over 250 ms: source pause or paired delivery loss is not distinguished.')
        result = dict(status='failed' if self.errors else 'complete', format='A005OBS1',
                      frames=self.frames,points=self.points,bytes=self.bytes,sha256=self.digest.hexdigest(),
                      first_stamp_ns=self.first,last_stamp_ns=self.last,reference=self.reference,
                      errors=self.errors,pending_scan_stamps=sorted(self.scans),pending_pose_stamps=sorted(self.poses),
                      stats={'received_scan':self.received['scan'],'received_pose':self.received['pose']},
                      max_pending=self.max_pending, max_gap_ns=self.max_gap_ns,
                      gaps_over_250ms=self.gaps_over_250ms,warnings=warnings,
                      peak_rss_kib=resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                      selection='all exact-stamp pairs observed by sidecar; not projector accepted-frame hooks')
        atomic_json(self.out/'recording.json', result)
        return not self.errors

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--session',required=True)
    args = parser.parse_args()
    out = Path(args.session).resolve(strict=True)
    allowed = [Path('/home/wheeltec/maps/manual_postprocess_test'),
               Path(__file__).resolve().parent/'verification']
    if not any(out.is_relative_to(p) and out != p for p in allowed):
        raise SystemExit('Session outside permitted output roots')
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    node = Recorder(out)
    def stop(sig, _):
        if sig != signal.SIGINT: node.abnormal_signal = sig
        if node.stop_requested is None: node.stop_requested = time.monotonic()
    for sig in (signal.SIGINT,signal.SIGTERM,signal.SIGHUP): signal.signal(sig,stop)
    okay = False
    try:
        while not node.errors:
            rclpy.spin_once(node,timeout_sec=0.05)
            if node.abnormal_signal is not None:
                node.fail('Recorder received abnormal termination signal '+str(node.abnormal_signal))
                break
            node.maintenance()
            if node.stop_requested is not None:
                elapsed = time.monotonic()-node.stop_requested
                quiet = time.monotonic()-node.last_received_wall
                publishers = sum(node.count_publishers(t) for t in ('/fastlio/free_scan','/fastlio/free_scan_pose'))
                if elapsed >= 1 and quiet >= 1 and publishers == 0: break
                if elapsed >= 12:
                    node.fail('Observation publishers still present or shutdown drain did not settle')
        okay = node.finish()
    except BaseException as e:
        node.fail(repr(e))
        try: node.finish()
        except Exception: traceback.print_exc()
    finally:
        node.destroy_node()
        if rclpy.ok(): rclpy.shutdown()
    print('RECORDING_CLOSED '+str(out/'recording.json'),flush=True)
    return 0 if okay else 1

if __name__ == '__main__':
    raise SystemExit(main())
