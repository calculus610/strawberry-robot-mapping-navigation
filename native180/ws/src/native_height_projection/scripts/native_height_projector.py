#!/usr/bin/env python3
"""Native obstacle projection plus conservative observed-free height columns."""
from array import array
import io
import json
import os
from pathlib import Path
import time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from rclpy.executors import ExternalShutdownException
from sensor_msgs.msg import PointCloud2
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import OccupancyGrid
from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
from projection_core import project_snapshot, rotation_matrix
from free_evidence import FreeEvidence


def cloud_xyz(msg):
    fields = {f.name: f for f in msg.fields}
    if any(n not in fields or fields[n].datatype != 7 or fields[n].count != 1 for n in ('x', 'y', 'z')):
        raise ValueError('Expected float32 x/y/z fields')
    if msg.width == 0 or msg.height == 0:
        return np.empty((0, 3), dtype=np.float32)
    if len(msg.data) < msg.row_step * msg.height:
        raise ValueError('Truncated PointCloud2')
    endian = '>' if msg.is_bigendian else '<'
    dtype = np.dtype(dict(names=['x', 'y', 'z'], formats=[endian+'f4']*3,
                         offsets=[fields[n].offset for n in ('x', 'y', 'z')], itemsize=msg.point_step))
    points = np.ndarray((msg.height, msg.width), dtype=dtype, buffer=msg.data,
                        strides=(msg.row_step, msg.point_step))
    return np.stack([points[n].reshape(-1) for n in ('x', 'y', 'z')], axis=1)


class Projector(Node):
    def __init__(self):
        super().__init__('native_height_projector')
        self.low = float(self.declare_parameter('min_height', 0.10).value)
        self.high = float(self.declare_parameter('max_height', 1.00).value)
        self.resolution = float(self.declare_parameter('resolution', 0.05).value)
        self.max_cells = int(self.declare_parameter('max_grid_cells', 10_000_000).value)
        self.height_step = float(self.declare_parameter('free_height_step', 0.10).value)
        self.free = FreeEvidence(self.resolution, self.low, self.high, self.height_step)
        self.pending_scans, self.pending_poses = {}, {}
        self.last_observation_stamp = -1
        self.observation_frames = self.observation_points = 0
        self.observation_ms = self.observation_max_ms = 0.0
        self.last_free = (np.empty((0,2), np.int32), np.empty(0, np.uint32))
        self.frame = 'native_projection'
        output = str(self.declare_parameter('output_dir', '').value)
        if not output:
            raise ValueError('output_dir is required')
        self.output = Path(output)
        self.output.mkdir(parents=True, exist_ok=True)
        self.reference = None
        self.reference_error = False
        self.last_map = self.last_source = self.last_stats = None
        self.last_stamp = -1
        self.last_log = 0.0
        latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                             durability=DurabilityPolicy.TRANSIENT_LOCAL)
        live = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        scan_qos = QoSProfile(depth=5, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.pub = self.create_publisher(OccupancyGrid, '/map', latched)
        self.tf = StaticTransformBroadcaster(self)
        self.ref_sub = self.create_subscription(PoseStamped, '/fastlio/native_height_reference', self.on_reference, latched)
        self.cloud_sub = self.create_subscription(PointCloud2, '/fastlio/native_map', self.on_cloud, live)
        self.scan_sub = self.create_subscription(PointCloud2, '/fastlio/free_scan', self.on_scan, scan_qos)
        self.pose_sub = self.create_subscription(PoseStamped, '/fastlio/free_scan_pose', self.on_pose, scan_qos)
        self.timer = self.create_timer(60.0, self.save)
        self.get_logger().info(f'WAITING_FOR_FIXED_REFERENCE: height=[{self.low:.2f}, {self.high:.2f}] m above ground; free requires every {self.height_step:.2f}m layer; native obstacles win; output={output}')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec

    def on_scan(self, msg):
        if msg.header.frame_id == 'native_scan_lidar':
            self.match_observation(msg, self.pending_scans)

    def on_pose(self, msg):
        if msg.header.frame_id == 'camera_init':
            self.match_observation(msg, self.pending_poses)

    def match_observation(self, msg, pending):
        stamp = self.stamp(msg)
        if self.reference is None or self.reference_error or stamp <= self.last_observation_stamp:
            return
        pending[stamp] = msg
        # Exact timestamp pairing. Lost messages reduce evidence; never reuse a pose.
        for cache in (self.pending_scans, self.pending_poses):
            for old in sorted(cache)[:-12]:
                del cache[old]
        if stamp not in self.pending_scans or stamp not in self.pending_poses:
            return
        scan = self.pending_scans.pop(stamp)
        pose = self.pending_poses.pop(stamp).pose
        if self.free.bounds is None:
            return
        begin = time.monotonic()
        try:
            q, p = pose.orientation, pose.position
            rotation = rotation_matrix([q.x,q.y,q.z,q.w])
            optical_origin = np.array([p.x,p.y,p.z], dtype=np.float64)
            if not np.isfinite(optical_origin).all():
                raise ValueError('Non-finite optical pose')
            points = cloud_xyz(scan)
            ground, basis = self.reference
            local = (points @ rotation.T + optical_origin - ground) @ basis
            origin = (optical_origin-ground) @ basis
            self.free.integrate(local, origin)
            self.observation_frames += 1
            self.observation_points += len(points)
            self.last_observation_stamp = stamp
            self.observation_ms = (time.monotonic()-begin)*1000
            self.observation_max_ms = max(self.observation_max_ms, self.observation_ms)
        except (ValueError, OverflowError, MemoryError) as e:
            self.get_logger().error(f'FREE_OBSERVATION_REJECTED: {e}')

    def on_reference(self, msg):
        if msg.header.frame_id != 'camera_init':
            self.get_logger().error('Reference rejected: expected camera_init')
            return
        p, q = msg.pose.position, msg.pose.orientation
        ground = np.array([p.x, p.y, p.z])
        try:
            rotation = rotation_matrix([q.x, q.y, q.z, q.w])
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        if not np.isfinite(ground).all():
            self.get_logger().error('Non-finite reference rejected')
            return
        if self.reference is not None:
            if not (np.allclose(ground, self.reference[0], atol=1e-9, rtol=0) and
                    np.allclose(rotation, self.reference[1], atol=1e-9, rtol=0)):
                self.get_logger().error('REFERENCE_CHANGED: restart this experiment; refusing to mix coordinate references')
                self.reference_error = True
            return
        self.reference = ground, rotation
        transform = TransformStamped()
        transform.header.stamp = self.get_clock().now().to_msg()
        transform.header.frame_id = 'camera_init'
        transform.child_frame_id = self.frame
        transform.transform.translation.x, transform.transform.translation.y, transform.transform.translation.z = (float(v) for v in ground)
        transform.transform.rotation = q
        # A new experimental child frame only; native camera_init/body TF stays intact.
        self.tf.sendTransform(transform)
        self.get_logger().info(f'PROJECTION_READY fixed ground={ground.tolist()} height=[{self.low}, {self.high}]')

    def on_cloud(self, msg):
        if self.reference is None or self.reference_error or msg.header.frame_id != 'camera_init':
            return
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        if stamp <= self.last_stamp:
            return
        begin = time.monotonic()
        try:
            xyz = cloud_xyz(msg)
            if not len(xyz):
                # A completely empty native snapshot clears the previous map too.
                if self.last_map is None:
                    return
                result = OccupancyGrid()
                result.header = msg.header
                result.header.frame_id = self.frame
                result.info = self.last_map.info
                result.data = array('b', [-1]) * (result.info.width * result.info.height)
                self.free.clear()
                stats = dict(native_points=0, selected_points=0, occupied_cells=0,
                             free_cells=0, unknown_cells=len(result.data),
                             width=result.info.width, height=result.info.height)
            else:
                grid, origin, stats = project_snapshot(xyz, *self.reference, self.low,
                    self.high, self.resolution, max_cells=self.max_cells)
                # Bounds remain exclusively those computed from the native snapshot.
                # Free rays cannot expand, shrink or otherwise define the canvas.
                self.free.set_bounds(origin, grid.shape)
                stats['free_cells'] = self.free.apply(grid)
                stats['unknown_cells'] = int(np.count_nonzero(grid == -1))
                result = OccupancyGrid()
                result.header.stamp = msg.header.stamp
                result.header.frame_id = self.frame
                result.info.map_load_time = msg.header.stamp
                result.info.resolution = self.resolution
                result.info.height, result.info.width = grid.shape
                result.info.origin.position.x, result.info.origin.position.y = (float(v) for v in origin)
                result.info.origin.orientation.w = 1.0
                result.data = array('b', grid.tobytes())
            self.pub.publish(result)
            stats.update(stamp_ns=stamp, frame=self.frame, source_frame='camera_init',
                         min_height=self.low, max_height=self.high, resolution=self.resolution,
                         ground=self.reference[0].tolist(), rotation=self.reference[1].tolist(),
                         free_height_step=self.height_step,
                         free_layers=int(np.ceil((self.high-self.low)/self.height_step-1e-9)),
                         free_cache_columns=self.free.columns, free_rays=self.free.rays,
                         free_observation_frames=self.observation_frames,
                         free_observation_points=self.observation_points,
                         free_observation_stamp_ns=self.last_observation_stamp,
                         free_observation_ms=round(self.observation_ms,3),
                         free_observation_max_ms=round(self.observation_max_ms,3),
                         projection_ms=round((time.monotonic()-begin)*1000, 3))
            self.last_free = self.free.snapshot()
            self.last_map, self.last_source, self.last_stats = result, xyz, stats
            self.last_stamp = stamp
            if time.monotonic() - self.last_log >= 10:
                self.get_logger().info('PROJECT ' + json.dumps(stats, separators=(',', ':')))
                self.last_log = time.monotonic()
        except (ValueError, OverflowError, MemoryError) as e:
            self.get_logger().error(f'PROJECTION_REJECTED: {e}; previous output retained')

    def save(self):
        if self.last_map is None:
            return
        msg = self.last_map
        try:
            grid = np.frombuffer(msg.data, dtype=np.int8).reshape(msg.info.height, msg.info.width)
            pixels = np.where(grid == 100, 0, np.where(grid == 0, 254, 205)).astype(np.uint8)[::-1]
            self.atomic('map.pgm', f'P5\n{msg.info.width} {msg.info.height}\n255\n'.encode() + pixels.tobytes())
            self.atomic('map.yaml', (
                'image: map.pgm\nmode: trinary\n'
                f'resolution: {self.resolution:.8f}\n'
                f'origin: [{msg.info.origin.position.x:.8f}, {msg.info.origin.position.y:.8f}, 0.0]\n'
                'negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.19\n').encode())
            self.atomic('projection_stats.json', (json.dumps(self.last_stats, indent=2)+'\n').encode())
            cache = io.BytesIO()
            np.savez_compressed(cache, xy=self.last_free[0], masks=self.last_free[1],
                                resolution=self.resolution, low=self.low, high=self.high,
                                height_step=self.height_step, map_stamp_ns=self.last_stamp,
                                observation_stamp_ns=self.last_stats['free_observation_stamp_ns'])
            self.atomic('free_evidence.npz', cache.getvalue())
            # Exact last source of the saved 2D map, not an extra accumulated map.
            xyz = np.asarray(self.last_source, dtype='<f4')
            header = ('# .PCD v0.7\nVERSION 0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\n'
                      f'COUNT 1 1 1\nWIDTH {len(xyz)}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {len(xyz)}\nDATA binary\n')
            self.atomic('projection_source_camera_init.pcd', header.encode()+xyz.tobytes())
            self.get_logger().info(f'MAP_SAVED {self.output}/map.yaml stamp_ns={self.last_stamp}')
        except (OSError, ValueError, MemoryError) as e:
            # rclpy's context may already be closed during the final save.
            print(f'MAP_SAVE_FAILED {e}', flush=True)

    def atomic(self, filename, data):
        target = self.output / filename
        temp = self.output / (filename + '.tmp')
        with temp.open('wb') as f:
            f.write(data)
            f.flush()
            os.fsync(f.fileno())
        temp.replace(target)


def main():
    rclpy.init()
    node = None
    try:
        node = Projector()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.save()
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
