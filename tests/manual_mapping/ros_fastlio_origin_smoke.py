#!/usr/bin/env python3
"""Replay a completed MID360 bag into FAST-LIO alone on a local ROS domain."""

import argparse
from contextlib import closing
import json
import os
from pathlib import Path
import signal
import sqlite3
import subprocess
import time

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from livox_ros_driver2.msg import CustomMsg, CustomPoint
from rclpy.qos import qos_profile_sensor_data
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import Imu, PointCloud2, PointField
from tf2_msgs.msg import TFMessage


NANOSECONDS = 1_000_000_000
DEFAULT_BAG = Path(
    "/home/wheeltec/zzj_ws_backups/20261106_manual_versions/"
    "live_sensor_validation/live_sensor_validation_0.db3"
)
POINT_TYPE = "sensor_msgs/msg/PointCloud2"
IMU_TYPE = "sensor_msgs/msg/Imu"
CLOUD_TOPIC = "/cloud_registered_body"
ORIGIN_TOPIC = "/manual_mapping/lidar_origin_body"
LIVOX_LAYOUT = {
    "x": (0, PointField.FLOAT32),
    "y": (4, PointField.FLOAT32),
    "z": (8, PointField.FLOAT32),
    "intensity": (12, PointField.FLOAT32),
    "tag": (16, PointField.UINT8),
    "line": (17, PointField.UINT8),
    "timestamp": (18, PointField.FLOAT64),
}
LIVOX_DTYPE = np.dtype(list(zip(LIVOX_LAYOUT, ("<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"))))


def stamp_ns(stamp):
    return stamp.sec * NANOSECONDS + stamp.nanosec


def xyz(vector):
    return [vector.x, vector.y, vector.z]


def livox_message(cloud):
    fields = {field.name: (field.offset, field.datatype) for field in cloud.fields}
    if (
        fields != LIVOX_LAYOUT
        or any(field.count != 1 for field in cloud.fields)
        or cloud.is_bigendian
        or cloud.height != 1
        or cloud.width == 0
        or cloud.point_step != LIVOX_DTYPE.itemsize
        or cloud.row_step != cloud.width * cloud.point_step
        or len(cloud.data) != cloud.row_step
    ):
        raise ValueError("recorded PointCloud2 does not match the packed MID360 layout")

    base = stamp_ns(cloud.header.stamp)
    if base <= 0:
        raise ValueError("recorded LiDAR stamp must be positive")
    points = np.frombuffer(cloud.data, dtype=LIVOX_DTYPE)
    # Both operands use the recorded Float64 absolute-ns precision so the
    # header's sub-ULP remainder cannot make a genuine zero offset negative.
    offsets = np.rint(points["timestamp"] - np.float64(base))
    if not np.all(np.isfinite(offsets) & (offsets >= 0) & (offsets < 200_000_000)):
        raise ValueError(f"recorded LiDAR point offsets outside [0, 0.2 s) at {base}")

    reflectivity = points["intensity"]
    if not np.all(
        np.isfinite(reflectivity) & (reflectivity >= 0) & (reflectivity <= 255)
        & (reflectivity == np.rint(reflectivity))
    ):
        raise ValueError(f"recorded intensity cannot be represented as Livox reflectivity at {base}")

    converted = [
        CustomPoint(
            x=x, y=y, z=z, reflectivity=int(intensity), tag=tag, line=line, offset_time=offset,
        )
        for (x, y, z, intensity, tag, line, _), offset
        in zip(points.tolist(), offsets.astype(np.uint32).tolist())
    ]
    return (
        CustomMsg(header=cloud.header, timebase=base, point_num=len(converted), points=converted),
        int(offsets.min()),
        int(offsets.max()),
    )


def bag_messages(connection):
    ids = {}
    for msg_type in (POINT_TYPE, IMU_TYPE):
        rows = connection.execute(
            "SELECT id, name, serialization_format FROM topics WHERE type = ?", (msg_type,)
        ).fetchall()
        if len(rows) != 1 or rows[0][2] != "cdr":
            raise ValueError(f"expected one CDR topic for {msg_type}, found {rows!r}")
        ids[msg_type] = rows[0][0]
        print(f"Replay {rows[0][1]} ({msg_type})", flush=True)
    rows = connection.execute(
        "SELECT topic_id, timestamp, data FROM messages "
        "WHERE topic_id IN (?, ?) ORDER BY timestamp, id",
        (ids[POINT_TYPE], ids[IMU_TYPE]),
    )
    return ids[POINT_TYPE], rows


def body_points(cloud):
    if (
        cloud.header.frame_id != "body"
        or cloud.width * cloud.height == 0
        or cloud.point_step <= 0
        or cloud.row_step < cloud.width * cloud.point_step
        or len(cloud.data) != cloud.row_step * cloud.height
    ):
        raise ValueError("unexpected FAST-LIO body cloud frame or layout")
    fields = {field.name: field for field in cloud.fields}
    order = ">" if cloud.is_bigendian else "<"
    coordinates = []
    for name in ("x", "y", "z"):
        field = fields.get(name)
        if (
            field is None or field.datatype != PointField.FLOAT32 or field.count != 1
            or field.offset + 4 > cloud.point_step
        ):
            raise ValueError(f"missing float32 {name} in FAST-LIO body cloud")
        coordinates.append(np.ndarray(
            (cloud.height, cloud.width), dtype=order + "f4", buffer=cloud.data,
            offset=field.offset, strides=(cloud.row_step, cloud.point_step),
        ))
    return np.stack(coordinates, axis=-1).reshape(-1, 3)


class Outputs:
    def __init__(self, node):
        self.clouds = {}
        self.origins = {}
        self.tf_stamps = set()
        node.create_subscription(PointCloud2, CLOUD_TOPIC, self.on_cloud, qos_profile_sensor_data)
        node.create_subscription(PoseStamped, ORIGIN_TOPIC, self.on_origin, qos_profile_sensor_data)
        node.create_subscription(TFMessage, "/tf", self.on_tf, 100)

    def on_cloud(self, msg):
        self.clouds[stamp_ns(msg.header.stamp)] = body_points(msg)

    def on_origin(self, msg):
        point = np.array(xyz(msg.pose.position), dtype=np.float64)
        q = msg.pose.orientation
        if (
            msg.header.frame_id != "body"
            or not np.isfinite(point).all()
            or not np.isclose(np.linalg.norm([q.x, q.y, q.z, q.w]), 1.0, atol=1e-5)
        ):
            raise ValueError("invalid FAST-LIO LiDAR origin pose")
        self.origins[stamp_ns(msg.header.stamp)] = point

    def on_tf(self, msg):
        for transform in msg.transforms:
            if transform.header.frame_id == "camera_init" and transform.child_frame_id == "body":
                self.tf_stamps.add(stamp_ns(transform.header.stamp))

    def summarize(self):
        common = sorted(self.clouds.keys() & self.origins.keys() & self.tf_stamps)
        if len(common) < 10 or common[0] <= 0:
            raise RuntimeError(
                f"expected at least 10 exact cloud/origin/TF stamps, found {len(common)} "
                f"from {len(self.clouds)} clouds, {len(self.origins)} origins, "
                f"{len(self.tf_stamps)} TF stamps"
            )
        positions = np.array([self.origins[stamp] for stamp in common])
        ranges, finite_counts, in_range_counts = [], [], []
        for stamp in common:
            points = self.clouds[stamp]
            points = points[np.isfinite(points).all(axis=1)]
            distances = np.linalg.norm(points - self.origins[stamp], axis=1)
            ranges.append(distances)
            finite_counts.append(len(points))
            in_range_counts.append(int(np.count_nonzero(distances <= 3.0)))
        distances = np.concatenate(ranges)
        if not len(distances) or not sum(in_range_counts):
            raise RuntimeError("no finite LiDAR points within 3 m in the paired clouds")
        return {
            "body_cloud_stamps": len(self.clouds),
            "origin_stamps": len(self.origins),
            "camera_init_to_body_tf_stamps": len(self.tf_stamps),
            "exact_cloud_origin_pairs": len(self.clouds.keys() & self.origins.keys()),
            "exact_cloud_origin_tf_pairs": len(common),
            "first_exact_stamp_ns": common[0],
            "last_exact_stamp_ns": common[-1],
            "origin_first_m": positions[0].tolist(),
            "origin_last_m": positions[-1].tolist(),
            "unique_origin_positions": len(np.unique(positions, axis=0)),
            "max_origin_change_from_first_m": float(np.linalg.norm(positions - positions[0], axis=1).max()),
            "finite_points_per_frame_min": min(finite_counts),
            "finite_points_per_frame_max": max(finite_counts),
            "within_3m_points_per_frame_min": min(in_range_counts),
            "within_3m_points_per_frame_max": max(in_range_counts),
            "range_min_m": float(distances.min()),
            "range_p95_m": float(np.percentile(distances, 95)),
            "range_max_m": float(distances.max()),
        }


def spin_until(node, process, deadline):
    while (remaining := deadline - time.monotonic()) > 0:
        if process.poll() is not None:
            raise RuntimeError(f"FAST-LIO exited with code {process.returncode}")
        rclpy.spin_once(node, timeout_sec=min(remaining, 0.02))


def wait_for_endpoints(node, process, cloud_pub, imu_pub):
    deadline = time.monotonic() + 12.0
    while time.monotonic() < deadline:
        if (
            cloud_pub.get_subscription_count() == 1
            and imu_pub.get_subscription_count() == 1
            and node.count_publishers(CLOUD_TOPIC) == 1
            and node.count_publishers(ORIGIN_TOPIC) == 1
            and node.count_publishers("/tf") == 1
        ):
            return
        spin_until(node, process, time.monotonic() + 0.1)
    raise RuntimeError(
        "FAST-LIO ROS endpoints did not match the isolated test: "
        f"lidar subscribers={cloud_pub.get_subscription_count()}, "
        f"IMU subscribers={imu_pub.get_subscription_count()}, "
        f"body publishers={node.count_publishers(CLOUD_TOPIC)}, "
        f"origin publishers={node.count_publishers(ORIGIN_TOPIC)}, "
        f"TF publishers={node.count_publishers('/tf')}"
    )


def replay(node, process, connection, cloud_pub, imu_pub, rate):
    cloud_id, rows = bag_messages(connection)
    first_record = wall_start = None
    cloud_count = imu_count = 0
    offset_min, offset_max = 200_000_000, 0
    first_cloud = last_cloud = None
    pacing_slip = 0.0
    for topic_id, record_ns, data in rows:
        if topic_id == cloud_id:
            cloud, minimum, maximum = livox_message(deserialize_message(data, PointCloud2))
            publisher, message = cloud_pub, cloud
            offset_min, offset_max = min(offset_min, minimum), max(offset_max, maximum)
            last_cloud = stamp_ns(cloud.header.stamp)
            if first_cloud is None:
                first_cloud = last_cloud
            cloud_count += 1
        else:
            publisher, message = imu_pub, deserialize_message(data, Imu)
            imu_count += 1

        now = time.monotonic()
        if first_record is None:
            first_record, wall_start = record_ns, now
        scheduled = wall_start + (record_ns - first_record) / (rate * NANOSECONDS)
        # On a slow CPU avoid sending a burst of queued IMU after conversion.
        # Header stamps are preserved, including IMU inter-sample timing.
        if now - scheduled > 0.02:
            slip = now - scheduled
            wall_start += slip
            pacing_slip += slip
        else:
            spin_until(node, process, scheduled)
        if process.poll() is not None:
            raise RuntimeError(f"FAST-LIO exited with code {process.returncode}")
        publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.0)

    if not cloud_count or not imu_count:
        raise RuntimeError("recording is missing LiDAR or IMU messages")
    spin_until(node, process, time.monotonic() + 2.0)
    return {
        "recorded_clouds_replayed": cloud_count,
        "recorded_imus_replayed": imu_count,
        "first_input_cloud_stamp_ns": first_cloud,
        "last_input_cloud_stamp_ns": last_cloud,
        "minimum_input_point_offset_ns": offset_min,
        "maximum_input_point_offset_ns": offset_max,
        "target_replay_rate": rate,
        "pacing_slip_s": pacing_slip,
    }


def stop_child(process):
    # Use the group created by start_new_session, including the ros2 run child.
    for sig, timeout in ((signal.SIGINT, 8.0), (signal.SIGTERM, 3.0), (signal.SIGKILL, 2.0)):
        try:
            os.killpg(process.pid, sig)
        except ProcessLookupError:
            process.wait(timeout=timeout)
            return
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            process.poll()
            try:
                os.killpg(process.pid, 0)
            except ProcessLookupError:
                return
            time.sleep(0.05)
    raise RuntimeError("FAST-LIO process group did not exit")


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", type=Path, default=DEFAULT_BAG, help="completed SQLite rosbag file")
    parser.add_argument("--params-file", type=Path, required=True, help="mid360_manual_filtered.yaml")
    parser.add_argument("--output", type=Path, required=True)
    parser.add_argument("--rate", type=float, choices=(0.5, 1.0), default=0.5)
    args = parser.parse_args()
    if int(os.environ.get("ROS_DOMAIN_ID", "0")) < 70:
        parser.error("set an isolated ROS_DOMAIN_ID >= 70")
    if not args.params_file.is_file() or not args.bag.is_file():
        parser.error("--params-file and --bag must point to existing files")

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=True)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = str(output / "ros_logs")
    node = process = None
    rclpy.init(args=[])
    try:
        node = rclpy.create_node("manual_fastlio_origin_smoke")
        outputs = Outputs(node)
        cloud_pub = node.create_publisher(CustomMsg, "/livox/lidar", qos_profile_sensor_data)
        imu_pub = node.create_publisher(Imu, "/livox/imu", qos_profile_sensor_data)
        with (output / "fastlio_origin_smoke.log").open("w") as log, closing(
            sqlite3.connect(args.bag.resolve().as_uri() + "?mode=ro", uri=True)
        ) as connection:
            process = subprocess.Popen(
                [
                    "ros2", "run", "fast_lio", "fastlio_mapping", "--ros-args",
                    "--params-file", str(args.params_file.resolve()), "-p", "use_sim_time:=false",
                ],
                stdout=log, stderr=subprocess.STDOUT, cwd=output, start_new_session=True,
            )
            wait_for_endpoints(node, process, cloud_pub, imu_pub)
            inputs = replay(node, process, connection, cloud_pub, imu_pub, args.rate)
        result = {"passed": True, **inputs, **outputs.summarize()}
        payload = json.dumps(result, indent=2, allow_nan=False) + "\n"
        (output / "fastlio_origin_smoke.json").write_text(payload)
        print(payload, end="", flush=True)
    finally:
        try:
            if process is not None:
                stop_child(process)
        finally:
            if node is not None:
                node.destroy_node()
            rclpy.try_shutdown()


if __name__ == "__main__":
    main()
