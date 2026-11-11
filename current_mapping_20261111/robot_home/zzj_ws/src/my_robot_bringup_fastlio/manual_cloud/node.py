"""Standalone ROS 2 Humble node for the manual mapping cloud pipeline."""

from array import array
from collections import Counter, OrderedDict, deque
from dataclasses import dataclass
import json
from math import acos, isfinite
from time import monotonic, perf_counter

from geometry_msgs.msg import PoseStamped
import numpy as np
import rclpy
from rclpy.clock import Clock, ClockType
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header, String
from tf2_ros import Buffer, TransformException, TransformListener

from .sampling import StaticVoxelMap, distances_from, read_xyz, sample_frame


WORLD_FRAME = "camera_init"
BODY_FRAME = "body"
CLOUD_TOPIC = "/cloud_registered_body"
MAX_PENDING = 12
MAX_ORIGINS = 32
WAIT_SECONDS = 1.0
POLL_SECONDS = 0.02
XYZ_FIELDS = [
    PointField(name=name, offset=offset, datatype=PointField.FLOAT32, count=1)
    for name, offset in (("x", 0), ("y", 4), ("z", 8))
]


def _stamp_ns(stamp):
    # tf2 treats zero as a request for the latest transform.
    if stamp.sec < 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        return 0
    return stamp.sec * 1_000_000_000 + stamp.nanosec


def _xyz(message):
    return np.array([message.x, message.y, message.z], dtype=np.float64)


def _rigid_transform(message):
    translation = _xyz(message.translation)
    q = message.rotation
    quaternion = np.array([q.x, q.y, q.z, q.w], dtype=np.float64)
    length = np.linalg.norm(quaternion)
    if not np.isfinite(translation).all() or not isfinite(length) or length < 1e-12:
        raise ValueError("invalid body-to-world transform")
    quaternion /= length
    x, y, z, w = quaternion
    rotation = np.array([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])
    return translation, rotation, quaternion


def _apply_transform(points, rotation, translation):
    with np.errstate(over="ignore", invalid="ignore"):
        return np.einsum("ni,ji->nj", points, rotation) + translation


def _xyz_cloud(points, stamp):
    points = np.ascontiguousarray(points, dtype="<f4")
    return PointCloud2(
        header=Header(stamp=stamp, frame_id=WORLD_FRAME),
        height=1,
        width=len(points),
        fields=XYZ_FIELDS,
        is_bigendian=False,
        point_step=12,
        row_step=12 * len(points),
        data=array("B", points.tobytes()),
        is_dense=True,
    )


@dataclass
class PendingCloud:
    cloud: PointCloud2
    stamp_ns: int
    arrived: float
    last_tf_error: str | None = None


class ManualCloudFilter(Node):
    def __init__(self):
        super().__init__("manual_cloud_filter")
        self.stage = self.declare_parameter("stage", 1).value
        self.radius = self.declare_parameter("max_range_m", 3.0).value
        self.retention = self.declare_parameter("retention_percent", 70).value
        voxel_size = self.declare_parameter("voxel_size_m", 0.05).value
        max_voxels = self.declare_parameter("max_voxels", 500_000).value
        if (
            self.stage not in (1, 2, 3)
            or not isfinite(self.radius) or self.radius <= 0
            or not 0 <= self.retention <= 100
            or not isfinite(voxel_size) or voxel_size <= 0
            or not 1 <= max_voxels <= 500_000
        ):
            raise ValueError("invalid manual mapping filter parameters")

        # Earlier variants are deployable without the later Python modules.
        if self.stage >= 2:
            from .smoothing import SmoothingOptions, smooth_and_filter

            self._smoothing_options = SmoothingOptions()
            self._smooth = smooth_and_filter
        if self.stage == 3:
            from .temporal import TemporalMap

            self._clear_max_translation = self.declare_parameter(
                "clear_max_translation_m_s", 0.05
            ).value
            self._clear_max_rotation = self.declare_parameter(
                "clear_max_rotation_rad_s", 0.10
            ).value
            if not all(
                isfinite(limit) and limit >= 0
                for limit in (self._clear_max_translation, self._clear_max_rotation)
            ):
                raise ValueError("clear motion limits must be finite and nonnegative")

            self._map = TemporalMap({
                "dynamic": True,
                "voxel_size": voxel_size,
                "max_range": self.radius,
                "max_voxels": max_voxels,
                "hit_frames": self.declare_parameter("hit_frames", 5).value,
                "hit_seconds": self.declare_parameter("hit_seconds", 1.0).value,
                "free_frames": self.declare_parameter("free_frames", 4).value,
                "free_seconds": self.declare_parameter("free_seconds", 0.3).value,
                "endpoint_margin": self.declare_parameter("endpoint_margin_m", 0.15).value,
                "max_clear_rays": self.declare_parameter("max_clear_rays", 2048).value,
            })
        else:
            self._map = StaticVoxelMap(voxel_size, max_voxels)

        self._pending = deque()
        self._origins = OrderedDict()
        self._totals = Counter()
        self._last_cloud_ns = 0
        self._cloud_publisher_gid = None
        self._requires_restart = False
        self._map_stamp = None
        self._previous_pose = None
        self._map_publish_ms = None
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)
        self._wall_clock = Clock(clock_type=ClockType.STEADY_TIME)

        self._frame_pub = self.create_publisher(
            PointCloud2, "/manual_mapping/filtered_frame", qos_profile_sensor_data
        )
        self._map_pub = self.create_publisher(
            PointCloud2, "/manual_mapping/filtered_map",
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._metrics_pub = self.create_publisher(String, "/manual_mapping/filter_metrics", 10)
        self.create_subscription(
            PointCloud2, CLOUD_TOPIC, self._on_cloud, qos_profile_sensor_data
        )
        self.create_subscription(
            PoseStamped, "/manual_mapping/lidar_origin_body", self._on_origin, qos_profile_sensor_data
        )
        self.create_timer(POLL_SECONDS, self._poll, clock=self._wall_clock)
        self.create_timer(1.0, self._publish_map, clock=self._wall_clock)
        self.get_logger().info(
            f"Manual cloud filter stage {self.stage}, radius {self.radius:g} m, "
            f"retention {self.retention}%, voxels {voxel_size:g} m, capacity {max_voxels}"
        )

    def _emit_metrics(self, stamp_ns, **metrics):
        self._metrics_pub.publish(String(data=json.dumps({
            "stage": self.stage,
            "stamp_ns": stamp_ns,
            "radius_m": self.radius,
            "retention_percent": self.retention,
            "pending_clouds": len(self._pending),
            "map_voxels": len(self._map),
            "last_map_publish_ms": self._map_publish_ms,
            "requires_restart": self._requires_restart,
            "totals": dict(self._totals),
            **metrics,
        }, allow_nan=False)))

    def _drop(self, reason, stamp_ns, **metrics):
        self._totals["frames_dropped"] += 1
        self._totals[reason] += 1
        self._emit_metrics(stamp_ns, status="dropped", reason=reason, **metrics)

    def _require_restart(self, reason, stamp_ns, detail, **metrics):
        self._requires_restart = True
        abandoned = len(self._pending)
        self._pending.clear()
        self._origins.clear()
        self._tf_buffer.clear()
        self._previous_pose = None
        if self.stage == 3:
            self._map.reset_evidence()
        self._totals["frames_dropped"] += abandoned
        self._totals["frames_abandoned_on_restart"] += abandoned
        self.get_logger().warning(
            f"{detail} Discarded {abandoned} waiting clouds and retained {len(self._map)} map voxels. "
            "Restart the filter node before accepting clouds again."
        )
        self._drop(reason, stamp_ns, **metrics)

    def _on_cloud(self, cloud):
        self._totals["clouds_received"] += 1
        stamp_ns = _stamp_ns(cloud.header.stamp)
        if self._requires_restart:
            self._drop("restart_required", stamp_ns)
            return
        if not stamp_ns:
            self._drop("invalid_cloud_stamp", stamp_ns)
            return
        if cloud.header.frame_id != BODY_FRAME:
            self._drop("unexpected_cloud_frame", stamp_ns, frame_id=cloud.header.frame_id)
            return

        # Humble rclpy exposes endpoint GIDs through the ROS graph. Check the
        # discovered source before queuing any cloud for the current world map.
        publishers = self.get_publishers_info_by_topic(CLOUD_TOPIC)
        if len(publishers) > 1:
            self._require_restart(
                "multiple_cloud_publishers", stamp_ns,
                f"Discovered {len(publishers)} publishers on the cloud input.",
                publisher_count=len(publishers),
            )
            return
        gid = bytes(publishers[0].endpoint_gid) if publishers else b""
        if not any(gid):
            self._drop("discovery_pending", stamp_ns)
            return
        if self._cloud_publisher_gid is None:
            self._cloud_publisher_gid = gid
        if gid != self._cloud_publisher_gid:
            self._require_restart(
                "cloud_publisher_changed", stamp_ns, "The discovered cloud publisher changed.",
                previous_gid=self._cloud_publisher_gid.hex(), observed_gid=gid.hex(),
            )
            return

        if stamp_ns == self._last_cloud_ns:
            self._drop("duplicate_cloud_stamp", stamp_ns)
            return
        if stamp_ns < self._last_cloud_ns:
            # A restarted producer may have initialized a different world frame.
            self._totals["frames_abandoned_on_stamp_reset"] += len(self._pending)
            self._totals["stamp_resets"] += 1
            self._require_restart(
                "stamp_regression", stamp_ns,
                f"Cloud timestamp moved backward from {self._last_cloud_ns} to {stamp_ns}.",
                previous_stamp_ns=self._last_cloud_ns,
            )
            return

        self._last_cloud_ns = stamp_ns
        if len(self._pending) == MAX_PENDING:
            oldest = self._pending.popleft()
            self._origins.pop(oldest.stamp_ns, None)
            self._drop("queue_overflow", oldest.stamp_ns, tf_error=oldest.last_tf_error)
        self._pending.append(PendingCloud(cloud, stamp_ns, monotonic()))

    def _on_origin(self, origin):
        if self._requires_restart:
            return
        stamp_ns = _stamp_ns(origin.header.stamp)
        position = _xyz(origin.pose.position)
        if not stamp_ns or origin.header.frame_id != BODY_FRAME or not np.isfinite(position).all():
            self._totals["invalid_origins"] += 1
            return
        self._origins[stamp_ns] = position
        self._origins.move_to_end(stamp_ns)
        if len(self._origins) > MAX_ORIGINS:
            self._origins.popitem(last=False)
            self._totals["origin_cache_evictions"] += 1

    def _poll(self):
        # Process at most one ready frame per tick. No waiting inside a callback.
        now = monotonic()
        while self._pending:
            pending = self._pending[0]
            age = now - pending.arrived
            origin = self._origins.get(pending.stamp_ns)
            if age >= WAIT_SECONDS:
                self._pending.popleft()
                self._origins.pop(pending.stamp_ns, None)
                if origin is None:
                    reason = "origin_timeout"
                else:
                    reason = "tf_timeout" if pending.last_tf_error else "queue_timeout"
                self._drop(reason, pending.stamp_ns, wait_ms=age * 1000,
                           tf_error=pending.last_tf_error)
                continue
            if origin is None:
                return

            stamp = Time.from_msg(pending.cloud.header.stamp)
            try:
                transform = self._tf_buffer.lookup_transform(WORLD_FRAME, BODY_FRAME, stamp)
            except TransformException as error:
                pending.last_tf_error = str(error)
                return

            self._pending.popleft()
            self._origins.pop(pending.stamp_ns, None)
            try:
                self._process_frame(pending, origin, transform.transform, age)
            except ValueError as error:
                self._drop("invalid_frame_data", pending.stamp_ns, detail=str(error))
                self.get_logger().warning(str(error), throttle_duration_sec=5.0)
            return

    def _motion(self, stamp_ns, origin, quaternion):
        previous = self._previous_pose
        self._previous_pose = (stamp_ns, origin, quaternion)
        if previous is None:
            return {"allow_clear": False, "reason": "first_pose"}

        previous_stamp, previous_origin, previous_quaternion = previous
        dt = (stamp_ns - previous_stamp) / 1e9
        if not 0 < dt <= 0.5:
            return {"allow_clear": False, "reason": "pose_gap", "dt_s": dt}

        translation_speed = float(np.linalg.norm(origin - previous_origin)) / dt
        cosine = min(1.0, abs(float(np.dot(quaternion, previous_quaternion))))
        rotation_speed = 2.0 * acos(cosine) / dt
        if not isfinite(translation_speed):
            return {"allow_clear": False, "reason": "invalid_pose_delta", "dt_s": dt}
        allow_clear = (
            translation_speed <= self._clear_max_translation
            and rotation_speed <= self._clear_max_rotation
        )
        return {
            "allow_clear": allow_clear,
            "reason": "low_motion" if allow_clear else "high_motion",
            "dt_s": dt,
            "translation_m_s": translation_speed,
            "rotation_rad_s": rotation_speed,
        }

    def _process_frame(self, pending, origin_body, transform, wait):
        start = perf_counter()
        cloud = pending.cloud
        sample = sample_frame(read_xyz(cloud), origin_body, self.radius, self.retention)
        sampled_at = perf_counter()
        translation, rotation, quaternion = _rigid_transform(transform)
        origin_world = rotation @ origin_body + translation
        selected = _apply_transform(sample.selected_points, rotation, translation)
        if not np.isfinite(origin_world).all() or not np.isfinite(selected).all():
            raise ValueError("nonfinite coordinates after body-to-world transform")

        geometry = None
        if self.stage >= 2:
            raw = _apply_transform(sample.finite_points, rotation, translation)
            raw = raw[np.isfinite(raw).all(axis=1)]
            selected, geometry = self._smooth(
                selected, raw, self._smoothing_options,
                crop_origin=origin_world, crop_radius=self.radius,
            )
        minimum = sample.selected_range_min_m
        maximum = sample.selected_range_max_m
        # Validate the output independently of the smoothing boundary check.
        if self.stage >= 2:
            output_ranges = distances_from(selected, origin_world)
            within_radius = output_ranges <= self.radius + 1e-9
            geometry["outside_range_after_smoothing"] = int(np.count_nonzero(~within_radius))
            selected = selected[within_radius]
            output_ranges = output_ranges[within_radius]
            minimum = float(output_ranges.min()) if len(output_ranges) else None
            maximum = float(output_ranges.max()) if len(output_ranges) else None
        if np.any(np.abs(selected) > np.finfo(np.float32).max):
            raise ValueError("filtered coordinates exceed the PointCloud2 float32 range")
        geometric_at = perf_counter()

        motion = None
        if self.stage == 3:
            motion = self._motion(pending.stamp_ns, origin_world, quaternion)
            accumulation = self._map.update(
                selected, raw, origin_world, pending.stamp_ns / 1e9,
                allow_clear=motion["allow_clear"],
            )
            if accumulation["status"] != "ok":
                self._drop("temporal_" + accumulation["status"], pending.stamp_ns,
                           counts=sample.counts, accumulation=accumulation)
                return
        else:
            accumulation = self._map.update(selected)
        accumulated_at = perf_counter()
        self._map_stamp = cloud.header.stamp
        self._frame_pub.publish(_xyz_cloud(selected, cloud.header.stamp))
        published_at = perf_counter()

        rejected = accumulation["dropped_at_capacity"]
        if rejected:
            self._totals["capacity_rejected_voxels"] += rejected
            self.get_logger().warning(
                f"Map capacity reached. Rejected {rejected} new voxels in the current frame.",
                throttle_duration_sec=5.0,
            )
        self._totals["frames_processed"] += 1
        self._emit_metrics(
            pending.stamp_ns,
            status="processed",
            counts={**sample.counts, "filtered_frame": len(selected)},
            sampled_range_min_m=sample.selected_range_min_m,
            sampled_range_max_m=sample.selected_range_max_m,
            filtered_range_min_m=minimum,
            filtered_range_max_m=maximum,
            origin_body=origin_body.tolist(),
            origin_world=origin_world.tolist(),
            tf="cloud_stamp",
            geometry=geometry,
            motion=motion,
            accumulation=accumulation,
            timing_ms={
                "pair_and_tf_wait": wait * 1000,
                "decode_and_sample": (sampled_at - start) * 1000,
                "transform_and_geometry": (geometric_at - sampled_at) * 1000,
                "accumulation": (accumulated_at - geometric_at) * 1000,
                "frame_publish": (published_at - accumulated_at) * 1000,
                "total_processing": (published_at - start) * 1000,
            },
        )

    def _publish_map(self):
        if self._map_stamp is None:
            return
        start = perf_counter()
        self._map_pub.publish(_xyz_cloud(self._map.points(), self._map_stamp))
        self._map_publish_ms = (perf_counter() - start) * 1000


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = ManualCloudFilter()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
