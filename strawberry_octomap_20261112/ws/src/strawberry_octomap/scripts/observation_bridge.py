#!/usr/bin/env python3
"""Pair FAST-LIO observations and expose the scan-end optical frame to OctoMap."""

from collections import Counter, OrderedDict
from dataclasses import dataclass
import math
import time

import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


PREFIX = "/strawberry_octomap"
WORLD_FRAME = "camera_init"
INPUT_SCAN_FRAME = "native_scan_lidar"
GROUND_FRAME = "strawberry_ground"
OPTICAL_FRAME = "strawberry_lidar"
MAX_PENDING = 8
MAX_POINTS = 200_000
MAX_PAYLOAD_BYTES = 8 * 1024 * 1024
PAIR_TIMEOUT_SECONDS = 1.0
DIAGNOSTIC_SECONDS = 10.0
REFERENCE_TOLERANCE = 1e-9
QUATERNION_NORM_TOLERANCE = 1e-3


@dataclass(frozen=True)
class RigidPose:
    translation: tuple[float, float, float]
    rotation: tuple[float, float, float, float]


@dataclass(frozen=True)
class PendingObservation:
    kind: str
    observation: PointCloud2 | RigidPose
    payload_bytes: int
    arrived: float


def stamp_ns(message):
    stamp = message.header.stamp
    value = stamp.sec * 1_000_000_000 + stamp.nanosec
    if value <= 0 or not 0 <= stamp.nanosec < 1_000_000_000:
        raise ValueError("invalid observation timestamp")
    return value


def read_pose(message):
    if message.header.frame_id != WORLD_FRAME:
        raise ValueError(f"expected pose frame {WORLD_FRAME}")
    position = message.pose.position
    orientation = message.pose.orientation
    translation = (position.x, position.y, position.z)
    rotation = (orientation.x, orientation.y, orientation.z, orientation.w)
    if not all(math.isfinite(value) for value in translation + rotation):
        raise ValueError("nonfinite pose")
    norm = math.hypot(*rotation)
    if abs(norm - 1.0) > QUATERNION_NORM_TOLERANCE:
        raise ValueError("invalid quaternion norm")
    return RigidPose(translation, tuple(value / norm for value in rotation))


def same_reference(previous, current):
    if any(abs(a - b) > REFERENCE_TOLERANCE for a, b in zip(previous.translation, current.translation)):
        return False
    # q and -q encode the same rotation.
    sign = 1 if sum(a * b for a, b in zip(previous.rotation, current.rotation)) >= 0 else -1
    return all(abs(a - sign * b) <= REFERENCE_TOLERANCE
               for a, b in zip(previous.rotation, current.rotation))


def make_transform(stamp, child, pose):
    transform = TransformStamped()
    transform.header.stamp = stamp
    transform.header.frame_id = WORLD_FRAME
    transform.child_frame_id = child
    translation = transform.transform.translation
    translation.x, translation.y, translation.z = pose.translation
    rotation = transform.transform.rotation
    rotation.x, rotation.y, rotation.z, rotation.w = pose.rotation
    return transform


def validate_scan(message):
    if message.header.frame_id != INPUT_SCAN_FRAME:
        raise ValueError(f"expected cloud frame {INPUT_SCAN_FRAME}")
    if not 0 < message.width * message.height <= MAX_POINTS:
        raise ValueError("invalid or oversized point count")
    payload_bytes = len(message.data)
    if (message.point_step < 12 or message.row_step < message.width * message.point_step
            or payload_bytes != message.row_step * message.height or payload_bytes > MAX_PAYLOAD_BYTES):
        raise ValueError("invalid or oversized PointCloud2 payload")
    xyz = [field for field in message.fields if field.name in ("x", "y", "z")]
    if len(xyz) != 3 or {field.name for field in xyz} != {"x", "y", "z"}:
        raise ValueError("expected exactly one x, y and z field")
    if any(field.datatype != PointField.FLOAT32 or field.count != 1
           or field.offset + 4 > message.point_step for field in xyz):
        raise ValueError("expected in-bounds float32 XYZ fields")
    offsets = sorted(field.offset for field in xyz)
    if offsets[1] < offsets[0] + 4 or offsets[2] < offsets[1] + 4:
        raise ValueError("overlapping XYZ fields")
    return payload_bytes


class ObservationBridge(Node):
    def __init__(self):
        super().__init__("observation_bridge")
        self.reference = None
        self.pending = OrderedDict()
        self.pending_bytes = 0
        self.last_input_stamp = {"scan": 0, "pose": 0}
        self.last_published_stamp = 0
        self.counts = Counter()
        self.last_rejection = "none"
        self.last_diagnostic = time.monotonic()

        scan_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.BEST_EFFORT)
        reference_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                                   durability=DurabilityPolicy.TRANSIENT_LOCAL)
        cloud_qos = QoSProfile(depth=2, reliability=ReliabilityPolicy.RELIABLE)
        self.static_tf = StaticTransformBroadcaster(self)
        self.dynamic_tf = TransformBroadcaster(self, qos=QoSProfile(depth=MAX_PENDING))
        self.cloud_pub = self.create_publisher(PointCloud2, f"{PREFIX}/cloud_in", cloud_qos)
        self.create_subscription(PointCloud2, f"{PREFIX}/fastlio/free_scan", self.on_scan, scan_qos)
        self.create_subscription(PoseStamped, f"{PREFIX}/fastlio/free_scan_pose", self.on_pose, scan_qos)
        self.create_subscription(PoseStamped, f"{PREFIX}/fastlio/native_height_reference",
                                 self.on_reference, reference_qos)
        self.create_timer(1.0, self.housekeeping)
        self.get_logger().info("OBSERVATION_WAIT_REFERENCE waiting for FAST-LIO fixed ground reference")

    def on_reference(self, message):
        self.counts["reference_received"] += 1
        try:
            stamp_ns(message)
            reference = read_pose(message)
        except ValueError as error:
            self.counts["reference_invalid"] += 1
            self.last_rejection = str(error)
            return
        if self.reference is not None:
            if not same_reference(self.reference, reference):
                discarded = len(self.pending)
                self.pending.clear()
                self.pending_bytes = 0
                self.get_logger().error(
                    "OBSERVATION_REFERENCE_CHANGED fixed ground changed; "
                    f"discarded_pending={discarded}; exiting"
                )
                raise RuntimeError("fixed ground reference changed during mapping")
            return

        self.static_tf.sendTransform(make_transform(message.header.stamp, GROUND_FRAME, reference))
        self.reference = reference
        self.get_logger().info("OBSERVATION_REFERENCE_SET waiting for a valid same-stamp scan and pose")

    def accept_input(self, kind):
        self.counts[f"{kind}_received"] += 1
        if self.reference is None:
            self.counts["dropped_before_reference"] += 1
            return False
        return True

    def on_scan(self, message):
        if not self.accept_input("scan"):
            return
        try:
            stamp = stamp_ns(message)
            payload_bytes = validate_scan(message)
        except ValueError as error:
            self.counts["scan_invalid"] += 1
            self.last_rejection = str(error)
            return
        self.receive("scan", stamp, message, payload_bytes)

    def on_pose(self, message):
        if not self.accept_input("pose"):
            return
        try:
            stamp = stamp_ns(message)
            optical_pose = read_pose(message)
        except ValueError as error:
            self.counts["pose_invalid"] += 1
            self.last_rejection = str(error)
            return
        self.receive("pose", stamp, optical_pose, 0)

    def evict_oldest(self, reason):
        _, item = self.pending.popitem(last=False)
        self.pending_bytes -= item.payload_bytes
        self.counts[f"unpaired_{reason}_{item.kind}"] += 1

    def expire_pending(self, now):
        while self.pending and now - next(iter(self.pending.values())).arrived >= PAIR_TIMEOUT_SECONDS:
            self.evict_oldest("timeout")

    def receive(self, kind, stamp, observation, payload_bytes):
        if stamp <= self.last_input_stamp[kind]:
            self.counts[f"{kind}_nonmonotonic"] += 1
            return
        self.last_input_stamp[kind] = stamp
        if stamp <= self.last_published_stamp:
            self.counts[f"{kind}_late"] += 1
            return

        now = time.monotonic()
        self.expire_pending(now)
        paired = self.pending.pop(stamp, None)
        if paired is None:
            while len(self.pending) >= MAX_PENDING or self.pending_bytes + payload_bytes > MAX_PAYLOAD_BYTES:
                self.evict_oldest("capacity")
            self.pending[stamp] = PendingObservation(kind, observation, payload_bytes, now)
            self.pending_bytes += payload_bytes
            return

        self.pending_bytes -= paired.payload_bytes
        if kind == "scan":
            scan, optical_pose = observation, paired.observation
        else:
            scan, optical_pose = paired.observation, observation
        self.dynamic_tf.sendTransform(make_transform(scan.header.stamp, OPTICAL_FRAME, optical_pose))
        scan.header.frame_id = OPTICAL_FRAME
        self.cloud_pub.publish(scan)
        self.last_published_stamp = stamp
        self.counts["pairs_published"] += 1
        for older_stamp in tuple(self.pending):
            if older_stamp < stamp:
                older = self.pending.pop(older_stamp)
                self.pending_bytes -= older.payload_bytes
                self.counts[f"unpaired_superseded_{older.kind}"] += 1
        if self.counts["pairs_published"] == 1:
            self.get_logger().info(f"OBSERVATION_READY first valid pair forwarded stamp_ns={stamp}")

    def housekeeping(self):
        now = time.monotonic()
        self.expire_pending(now)
        if now - self.last_diagnostic < DIAGNOSTIC_SECONDS:
            return
        self.last_diagnostic = now
        if self.reference is None:
            state = "WAIT_REFERENCE"
        elif not self.last_published_stamp:
            state = "WAIT_PAIR"
        else:
            state = "FORWARDING"
        counts = " ".join(f"{name}={count}" for name, count in sorted(self.counts.items()))
        self.get_logger().info(
            f"OBSERVATION_STATUS state={state} pending={len(self.pending)} "
            f"pending_bytes={self.pending_bytes} last_published_stamp_ns={self.last_published_stamp} "
            f"{counts} last_rejection={self.last_rejection}"
        )


def main():
    rclpy.init()
    node = None
    try:
        node = ObservationBridge()
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
