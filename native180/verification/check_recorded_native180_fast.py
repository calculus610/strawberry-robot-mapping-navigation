#!/usr/bin/env python3
"""Replay a completed MID360 recording through an isolated native180 build."""

import argparse
from contextlib import closing
import hashlib
import json
import os
from pathlib import Path
import signal
import sqlite3
import struct
import subprocess
import sys
import time

import numpy as np
from check_native180_launch import (
    GROUND_TOPIC, MAP_TOPIC, MATCH_TOPIC, METRICS_TOPIC, NANOSECONDS,
    ORIGIN_TOPIC, SCAN_TOPIC, TREE_TOPIC, cloud_xyz, rotation, stamp_ns, xyz,
)
from check_saved_native180 import (
    REQUIRED_FILES, check, native_projection, read_pcd_xyz, verify_saved_session,
)


LIDAR_TOPIC = "/livox/lidar"
IMU_TOPIC = "/livox/imu"
ODOMETRY_TOPIC = "/Odometry"
CUSTOM_TYPE = "livox_ros_driver2/msg/CustomMsg"
CLOUD_TYPE = "sensor_msgs/msg/PointCloud2"
IMU_TYPE = "sensor_msgs/msg/Imu"
LIVOX_LAYOUT = {
    "x": (0, 7), "y": (4, 7), "z": (8, 7), "intensity": (12, 7),
    "tag": (16, 2), "line": (17, 2), "timestamp": (18, 8),
}
LIVOX_DTYPE = np.dtype(list(zip(LIVOX_LAYOUT, ("<f4", "<f4", "<f4", "<f4", "u1", "u1", "<f8"))))
CUSTOM_OFFSET_SAMPLES = 5


def bag_topics(connection):
    topics = {
        name: (topic_id, message_type, format_name)
        for topic_id, name, message_type, format_name
        in connection.execute("SELECT id, name, type, serialization_format FROM topics")
    }
    check(IMU_TOPIC in topics, f"Missing recorded {IMU_TOPIC}")
    imu_id, imu_type, imu_format = topics[IMU_TOPIC]
    check((imu_type, imu_format) == (IMU_TYPE, "cdr"), "IMU topic has an unexpected ROS type or encoding")
    cloud_name = next((name for name in (LIDAR_TOPIC, "/livox/points") if name in topics), None)
    check(cloud_name is not None, "Missing recorded /livox/lidar or /livox/points")
    cloud_id, cloud_type, cloud_format = topics[cloud_name]
    check(cloud_type in (CUSTOM_TYPE, CLOUD_TYPE) and cloud_format == "cdr", "LiDAR encoding unsupported")
    return cloud_id, cloud_name, cloud_type, imu_id


def livox_from_point_cloud(cloud, message_type, point_type):
    # Retain the original PointCloud2 header and the MID360 recorded per-point
    # timestamps. Absolute float64 nanoseconds have sub-microsecond quantization.
    fields = {field.name: (field.offset, field.datatype) for field in cloud.fields}
    check(
        fields == LIVOX_LAYOUT and all(field.count == 1 for field in cloud.fields)
        and not cloud.is_bigendian and cloud.height == 1 and cloud.width > 0
        and cloud.point_step == LIVOX_DTYPE.itemsize
        and cloud.row_step == cloud.width * cloud.point_step == len(cloud.data),
        "Recorded LiDAR cloud does not match the packed MID360 layout",
    )
    base = stamp_ns(cloud.header.stamp)
    check(base > 0, "LiDAR header stamp must be positive")
    points = np.frombuffer(cloud.data, dtype=LIVOX_DTYPE)
    offsets = np.rint(points["timestamp"] - np.float64(base))
    intensity = points["intensity"]
    check(
        np.all(np.isfinite(offsets) & (offsets >= 0) & (offsets < 200_000_000)),
        f"Point offsets outside [0, 0.2 s) at stamp {base}",
    )
    check(
        np.all(np.isfinite(intensity) & (intensity >= 0) & (intensity <= 255) & (intensity == np.rint(intensity))),
        f"Recorded intensity cannot be represented as Livox reflectivity at stamp {base}",
    )
    converted = [
        point_type(x=x, y=y, z=z, reflectivity=int(value), tag=tag, line=line, offset_time=offset)
        for (x, y, z, value, tag, line, _), offset in zip(points.tolist(), offsets.astype(np.uint32).tolist())
    ]
    message = message_type(header=cloud.header, timebase=base, point_num=len(converted), points=converted)
    return message, int(offsets.min()), int(offsets.max())


def custom_cdr_header(data):
    """Read the timestamp and point count of a plain CDR v1 CustomMsg."""
    check(isinstance(data, bytes) and len(data) >= 16, "Truncated CustomMsg CDR header")
    endian = {b"\x00\x00": ">", b"\x00\x01": "<"}.get(data[:2])
    check(endian is not None, "CustomMsg requires plain CDR v1 big or little endian encoding")
    sec, nanosec, frame_size = struct.unpack_from(endian + "iII", data, 4)
    check(nanosec < NANOSECONDS, "Invalid CustomMsg CDR header nanoseconds")
    frame_end = 16 + frame_size
    check(0 < frame_size and frame_end <= len(data) and data[frame_end - 1] == 0,
          "Invalid CustomMsg CDR frame ID length or terminator")

    # CDR aligns uint64 timebase relative to the payload after encapsulation.
    # Then come uint32 point_num, the 4 lidar ID/reserved bytes and the length
    # of the CustomPoint sequence. The points themselves stay as raw bytes.
    timebase_offset = 4 + ((frame_end - 4 + 7) & ~7)
    points_offset = timebase_offset + 20
    check(points_offset <= len(data), "Truncated CustomMsg CDR metadata")
    point_num, = struct.unpack_from(endian + "I", data, timebase_offset + 8)
    sequence_length, = struct.unpack_from(endian + "I", data, timebase_offset + 16)
    check(point_num == sequence_length > 0, "Recorded CustomMsg point count invalid")
    # Each CustomPoint has four 32-bit fields and three bytes. CDR aligns the
    # next element to four bytes, so only the last one may omit its pad byte.
    check(points_offset + 20 * (point_num - 1) + 19 <= len(data), "Truncated CustomMsg CDR points")
    return sec * NANOSECONDS + nanosec, point_num


def sample_custom_offsets(connection, cloud_id, deserialize):
    """Check a fixed, evenly spaced sample before starting the paced replay."""
    count, = connection.execute("SELECT count(*) FROM messages WHERE topic_id = ?", (cloud_id,)).fetchone()
    check(count > 0, "Recording contained no CustomMsg frames")
    sample_count = min(count, CUSTOM_OFFSET_SAMPLES)
    indices = [i * (count - 1) // max(1, sample_count - 1) for i in range(sample_count)]
    minimums, maximums, stamps = [], [], []
    for index in indices:
        data, = connection.execute(
            "SELECT data FROM messages WHERE topic_id = ? ORDER BY timestamp, id LIMIT 1 OFFSET ?",
            (cloud_id, index),
        ).fetchone()
        header_stamp, point_num = custom_cdr_header(data)
        message = deserialize(data)
        check(header_stamp == stamp_ns(message.header.stamp) and point_num == message.point_num == len(message.points),
              f"CustomMsg CDR header differs from standard deserialization in sample frame {index}")
        offsets = [point.offset_time for point in message.points]
        minimums.append(min(offsets))
        maximums.append(max(offsets))
        stamps.append(header_stamp)
    return minimums, maximums, stamps


def span(values):
    return [min(values), max(values)] if len(values) else None


def stats_ms(values):
    return (
        {"median_ms": float(np.median(values)), "p95_ms": float(np.percentile(values, 95)), "max_ms": float(max(values))}
        if len(values) else None
    )


def point_coordinates(message):
    return cloud_xyz(message) if message.width * message.height else np.empty((0, 3), dtype=np.float32)


def stop(process, child_groups):
    if process.poll() is not None:
        return
    process.send_signal(signal.SIGINT)
    try:
        process.wait(timeout=25)
    except subprocess.TimeoutExpired:
        for group in child_groups:
            try:
                if os.getpgid(group) == group:
                    os.killpg(group, signal.SIGKILL)
            except ProcessLookupError:
                continue
        process.kill()
        process.wait(timeout=5)
        raise RuntimeError("Supervisor failed to stop after SIGINT")


def replay(connection, spin_until, node, process, lidar, imu, rate):
    import rclpy
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import Imu, PointCloud2

    cloud_id, cloud_name, cloud_type, imu_id = bag_topics(connection)
    raw_custom = cloud_type == CUSTOM_TYPE
    preflight_start = time.monotonic()
    if raw_custom:
        offset_min, offset_max, offset_sample_stamps = sample_custom_offsets(
            connection, cloud_id, lambda data: deserialize_message(data, CustomMsg)
        )
    else:
        offset_min, offset_max, offset_sample_stamps = [], [], []
    preflight_ms = (time.monotonic() - preflight_start) * 1000
    rows = connection.execute(
        "SELECT topic_id, timestamp, data FROM messages "
        "WHERE topic_id IN (?, ?) ORDER BY timestamp, id", (cloud_id, imu_id),
    )
    lidar_stamps, imu_stamps, conversion_ms, lidar_publish_ms = [], [], [], []
    record_first = record_last = wall_start = None
    schedule_shift = 0.0
    for topic_id, recorded_ns, data in rows:
        if topic_id == imu_id:
            message = deserialize_message(data, Imu)
            imu_stamps.append(stamp_ns(message.header.stamp))
            publisher = imu
        else:
            converted_at = time.monotonic()
            if raw_custom:
                header_stamp, _ = custom_cdr_header(data)
                message = data  # Humble Publisher.publish(bytes) uses publish_raw.
            else:
                message, minimum, maximum = livox_from_point_cloud(
                    deserialize_message(data, PointCloud2), CustomMsg, CustomPoint
                )
                header_stamp = stamp_ns(message.header.stamp)
                offset_min.append(minimum)
                offset_max.append(maximum)
            conversion_ms.append((time.monotonic() - converted_at) * 1000)
            lidar_stamps.append(header_stamp)
            publisher = lidar

        now = time.monotonic()
        if record_first is None:
            record_first, wall_start = recorded_ns, now
        record_last = recorded_ns
        due = wall_start + schedule_shift + (recorded_ns - record_first) / (rate * NANOSECONDS)
        if now - due > 0.02:
            # Avoid bursting queued IMU after a slower PointCloud2 conversion.
            # None of the sensor header timestamps is changed.
            schedule_shift += now - due
        else:
            spin_until(due)
        check(process.poll() is None, "Native launch exited while replaying")
        if topic_id == cloud_id:
            publish_at = time.monotonic()
            publisher.publish(message)
            lidar_publish_ms.append((time.monotonic() - publish_at) * 1000)
        else:
            publisher.publish(message)
        rclpy.spin_once(node, timeout_sec=0.0)

    check(lidar_stamps and imu_stamps, "Recording did not contain both sensor streams")
    return {
        "bag_lidar_topic": cloud_name,
        "bag_lidar_type": cloud_type,
        "bag_imu_topic": IMU_TOPIC,
        "replay_lidar_topic": LIDAR_TOPIC,
        "replay_imu_topic": IMU_TOPIC,
        "lidar_frames_published": len(lidar_stamps),
        "imu_samples_published": len(imu_stamps),
        "lidar_header_stamp_ns": span(lidar_stamps),
        "imu_header_stamp_ns": span(imu_stamps),
        "lidar_point_offset_ns": [min(offset_min), max(offset_max)],
        "lidar_point_offset_validation": {
            "mode": "sampled_standard_deserialization" if raw_custom else "every_converted_frame",
            "frames": len(offset_min),
            "sampled_header_stamp_ns": offset_sample_stamps,
        },
        "lidar_publish_mode": "original_cdr_bytes" if raw_custom else "pointcloud2_to_custommsg",
        "lidar_cdr_headers_checked": len(lidar_stamps) if raw_custom else 0,
        "lidar_offset_sample_preflight_ms": preflight_ms if raw_custom else None,
        "recorded_elapsed_s": (record_last - record_first) / NANOSECONDS,
        "wall_replay_elapsed_s": time.monotonic() - wall_start,
        "target_replay_rate": rate,
        "replay_schedule_delay_s": schedule_shift,
        "lidar_decode_conversion": stats_ms(conversion_ms),
        "lidar_publish_call": stats_ms(lidar_publish_ms),
    }


def summarize_matching(stage, retention_percent, matches, metrics, raw, poses, receive_time):
    pairs = sorted(matches.keys() & metrics.keys())
    check(len(pairs) >= 5, f"Expected five matched diagnostic frames, received {len(pairs)}")
    check(len(raw.keys() & poses.keys()) or not raw, "Raw observation had no exact optical-origin pair")
    independently_counted = 0
    frame_counts, max_ranges = [], []
    eligible_counts, sampled_counts = [], []
    for stamp in pairs:
        row = metrics[stamp]
        cloud = matches[stamp]
        eligible, sampled = row["eligible_points"], row["sampled_points"]
        check(row["stage"] == stage, f"Wrong optimization stage at {stamp}")
        check(0 <= eligible <= row["forward_points"], f"Invalid eligible count at {stamp}")
        check(sampled == eligible * retention_percent // 100,
              f"Expected floor({retention_percent}% of eligible points) at {stamp}")
        check(row["matching_input_points"] == len(cloud) == sampled - row["smoothing_removed"],
              f"Matching cloud and counts disagree at {stamp}")
        if stage == 1:
            check(row["smoothing_moved"] == row["smoothing_removed"] == 0, f"Stage 1 ran geometry at {stamp}")
        check(row["matching_behind_points"] == 0, f"Metrics report rear matching points at {stamp}")
        if len(cloud):
            check(np.isfinite(cloud).all() and cloud[:, 0].min() >= -1e-5, f"Nonfinite or rear matching point at {stamp}")
            max_range = float(np.linalg.norm(cloud.astype(np.float64), axis=1).max())
            check(max_range <= 3.0 + 1e-5, f"Matching input exceeds 3 m at {stamp}: {max_range}")
            check(abs(max_range - row["matching_max_range_m"]) <= 1e-5, f"Matching radius metric mismatch at {stamp}")
            max_ranges.append(max_range)
        if stamp in raw:
            points = raw[stamp].astype(np.float64)
            squared = (points[:, 0] ** 2 + points[:, 1] ** 2) + points[:, 2] ** 2
            check(len(points) == row["forward_points"] and np.count_nonzero(squared <= 9.0) == eligible,
                  f"Independently counted raw frame disagrees with metrics at {stamp}")
            independently_counted += 1
        frame_counts.append(len(cloud))
        eligible_counts.append(eligible)
        sampled_counts.append(sampled)

    nonempty = sum(count > 0 for count in frame_counts)
    check(nonempty >= 5, f"Only {nonempty} nonempty matching clouds were verified")
    order = np.array(sorted(receive_time[stamp] for stamp in pairs))
    stamps = np.asarray(pairs, dtype="<i8")
    return {
        "retention_percent_expected": retention_percent,
        "verified_matching_frames": len(pairs),
        "verified_nonempty_matching_frames": nonempty,
        "matching_only_frames_received": len(matches),
        "metric_frames_received": len(metrics),
        "independent_raw_eligible_count_frames": independently_counted,
        "first_matching_stamp_ns": pairs[0],
        "last_matching_stamp_ns": pairs[-1],
        "matching_stamp_sequence_sha256": hashlib.sha256(stamps.tobytes()).hexdigest(),
        "eligible_points_min_max": span(eligible_counts),
        "sampled_points_min_max": span(sampled_counts),
        "matching_cloud_points_min_max": span(frame_counts),
        "matching_max_range_m": max(max_ranges),
        "matching_stamp_interval": stats_ms(np.diff(stamps) / 1_000_000),
        "matching_receive_interval": stats_ms(np.diff(order) * 1000),
        "native_admitted_reported": sum(entry["native_admitted"] for entry in metrics.values()),
        "native_removed_reported": sum(entry["native_removed"] for entry in metrics.values()),
        "stage3_temporal_status_counts": {
            str(status): sum(entry.get("temporal_status") == status for entry in metrics.values())
            for status in sorted({entry["temporal_status"] for entry in metrics.values() if "temporal_status" in entry})
        },
    }


def summarize_odometry(positions):
    stamps = sorted(positions)
    summary = {"frames_received": len(stamps)}
    if stamps:
        points = np.asarray([positions[stamp] for stamp in stamps])
        check(np.isfinite(points).all(), "Nonfinite odometry position")
        displacement = np.linalg.norm(points - points[0], axis=1)
        steps = np.linalg.norm(np.diff(points, axis=0), axis=1)
        summary.update({
            "first_stamp_ns": stamps[0],
            "last_stamp_ns": stamps[-1],
            "stamp_sequence_sha256": hashlib.sha256(np.asarray(stamps, dtype="<i8").tobytes()).hexdigest(),
            "max_displacement_from_first_m": float(displacement.max()),
            "final_displacement_from_first_m": float(displacement[-1]),
            "max_step_m": float(steps.max()) if len(steps) else None,
        })
    return summary


def run(args):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from livox_ros_driver2.msg import CustomMsg
    from nav_msgs.msg import OccupancyGrid, Odometry
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, qos_profile_sensor_data
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import String

    output = args.output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = str(output / "ros_logs")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    command = [
        sys.executable, str(args.runner.resolve()), str(output), "start_lidar:=false",
        "start_rviz:=false", f"optimization_stage:={args.stage}",
    ]
    matches, raw, poses, trees, grids, metrics, receive_time = {}, {}, {}, {}, {}, {}, {}
    odometry_positions = {}
    references = []
    errors = []
    report = {
        "stage": args.stage,
        "bag": str(args.bag.resolve()),
        "input_contract_verified": False,
        "map_saved_and_reconstructed": False,
    }

    def cloud_receiver(target, frame, record_wall=False):
        def receive(message):
            stamp = stamp_ns(message.header.stamp)
            try:
                check(message.header.frame_id == frame, f"Unexpected cloud frame {message.header.frame_id}")
                target[stamp] = point_coordinates(message)
                if record_wall:
                    receive_time[stamp] = time.monotonic()
            except (KeyError, ValueError) as error:
                errors.append(str(error))
        return receive

    def metrics_receiver(message):
        try:
            row = json.loads(message.data)
            stamp = row["stamp_ns"]
            check(isinstance(stamp, int) and not isinstance(stamp, bool), "Metric stamp is not integer ns")
            metrics[stamp] = row
        except (TypeError, KeyError, ValueError) as error:
            errors.append(str(error))

    def odometry_receiver(message):
        odometry_positions[stamp_ns(message.header.stamp)] = xyz(message.pose.pose.position)

    rclpy.init(args=[])
    node = rclpy.create_node("native180_recorded_contract_source")
    latched = QoSProfile(
        depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    best = QoSProfile(depth=20, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(PointCloud2, MATCH_TOPIC, cloud_receiver(matches, "native_scan_lidar", True), best)
    node.create_subscription(PointCloud2, SCAN_TOPIC, cloud_receiver(raw, "native_scan_lidar"), best)
    node.create_subscription(PointCloud2, TREE_TOPIC, cloud_receiver(trees, "camera_init"), best)
    node.create_subscription(PoseStamped, ORIGIN_TOPIC, lambda m: poses.update({stamp_ns(m.header.stamp): m}), best)
    node.create_subscription(PoseStamped, GROUND_TOPIC, references.append, latched)
    node.create_subscription(OccupancyGrid, MAP_TOPIC, lambda m: grids.update({stamp_ns(m.header.stamp): m}), latched)
    node.create_subscription(String, METRICS_TOPIC, metrics_receiver, best)
    node.create_subscription(Odometry, ODOMETRY_TOPIC, odometry_receiver, 20)
    lidar = node.create_publisher(CustomMsg, LIDAR_TOPIC, qos_profile_sensor_data)
    imu = node.create_publisher(Imu, IMU_TOPIC, qos_profile_sensor_data)

    try:
        with (output / "launch.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            children_file = Path(f"/proc/{process.pid}/task/{process.pid}/children")
            groups = []

            def spin_until(deadline, predicate=None):
                while time.monotonic() < deadline:
                    check(process.poll() is None, f"Launch exited early with {process.returncode}")
                    if predicate is not None and predicate():
                        return
                    rclpy.spin_once(node, timeout_sec=max(0.0, min(deadline - time.monotonic(), 0.02)))
                check(predicate is None or predicate(), "Timed out discovering isolated mapper subscriptions")

            def ready():
                if children_file.is_file():
                    groups[:] = [int(child) for child in children_file.read_text().split()]
                return (
                    lidar.get_subscription_count() == imu.get_subscription_count() == 1
                    and node.count_publishers(LIDAR_TOPIC) == node.count_publishers(IMU_TOPIC) == 1
                    and node.count_publishers(MATCH_TOPIC) == node.count_publishers(METRICS_TOPIC) == 1
                )

            try:
                spin_until(time.monotonic() + 20, ready)
                with closing(sqlite3.connect(args.bag.resolve().as_uri() + "?mode=ro", uri=True)) as bag:
                    report["replay"] = replay(bag, spin_until, node, process, lidar, imu, args.rate)
                spin_until(time.monotonic() + 3.0)
            finally:
                stop(process, groups)

        log_text = (output / "launch.log").read_text()
        check("livox_ros_driver2_node" not in log_text, "Hardware driver appeared in the offline replay launch")
        check(not errors, f"Invalid diagnostic packets: {errors[:5]}")
        if args.stage > 0:
            report["matching"] = summarize_matching(
                args.stage, args.retention_percent, matches, metrics, raw, poses, receive_time,
            )
            report["input_contract_verified"] = True
        else:
            report["matching"] = None
            report["input_contract_verified"] = None
            report["control_note"] = "Original stage0 intentionally emits no optimization diagnostics"
        report["odometry"] = summarize_odometry(odometry_positions)
        report["no_effective_points_messages"] = log_text.count("No Effective Points!")
        report["supervisor_returncode"] = process.returncode
        report["fixed_reference_published"] = bool(references)
        report["fixed_reference_logged_ready"] = "NATIVE_HEIGHT_REFERENCE_READY" in log_text
        report["native_tree_messages"] = len(trees)
        report["occupancy_grid_messages"] = len(grids)
        report["exact_native_tree_grid_pairs"] = len(trees.keys() & grids.keys())
        report["raw_forward_frames_received"] = len(raw)
        report["exact_free_scan_optical_origin_pairs"] = len(raw.keys() & poses.keys())
        report["missing_required_files"] = [name for name in REQUIRED_FILES if not (output / name).is_file()]

        for stamp in raw.keys() & poses.keys():
            check(poses[stamp].header.frame_id == "camera_init", "Wrong optical origin frame")
            check(np.isfinite(xyz(poses[stamp].pose.position)).all(), "Invalid scan-end optical origin")
            check(np.isfinite(raw[stamp]).all() and (raw[stamp][:, 0] >= -1e-5).all(), "Invalid front observation")
        for grid in grids.values():
            check(grid.header.frame_id == "native_projection", "Wrong final occupancy frame")
        for message in references:
            check(message.header.frame_id == "camera_init", "Wrong fixed ground frame")
            check(message.pose == references[0].pose, "Fixed ground reference changed during replay")
        if references:
            reference = references[0].pose
            checked_maps = 0
            for stamp in trees.keys() & grids.keys():
                if not len(trees[stamp]):
                    continue
                grid = grids[stamp]
                occupied, _, _ = native_projection(trees[stamp], xyz(reference.position), rotation(reference.orientation))
                actual = np.asarray(grid.data).reshape(grid.info.height, grid.info.width)
                check(np.array_equal(occupied, actual == 100), "Live black mask differs from its native snapshot")
                checked_maps += 1
            report["live_native_black_masks_verified"] = checked_maps

        reference_ready = report["fixed_reference_published"] or report["fixed_reference_logged_ready"]
        if reference_ready and not report["missing_required_files"]:
            report["saved"] = verify_saved_session(output)
            saved_stamp = report["saved"]["map_stamp_ns"]
            source = read_pcd_xyz(output / "projection_source_camera_init.pcd")
            if saved_stamp in trees:
                check(np.array_equal(source, trees[saved_stamp]), "Saved projection source differs from its native snapshot")
                report["saved_source_matches_captured_native_snapshot"] = True
            report["map_saved_and_reconstructed"] = True
            report["outcome"] = "matching_and_saved_map_verified"
            check(process.returncode == 0, "Save files exist but the supervisor reported abnormal shutdown")
        elif reference_ready:
            report["outcome"] = "reference_ready_but_no_complete_saved_map"
        else:
            check(not grids and not (output / "map.pgm").exists(), "A map appeared without a fixed reference")
            report["outcome"] = "matching_only_no_fixed_reference"
        if not report["map_saved_and_reconstructed"]:
            check(
                process.returncode == 1 and "SESSION_SAVE_INCOMPLETE:" in log_text,
                "Incomplete save did not end with the supervisor's expected missing-files result",
            )
    except (RuntimeError, ValueError, KeyError, TypeError, OSError, sqlite3.Error) as error:
        report["error"] = str(error)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        payload = json.dumps(report, indent=2, allow_nan=False) + "\n"
        (output / "recorded_native180_result.json").write_text(payload)
        print(payload, end="", flush=True)
    return 1 if "error" in report else 0


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True)
    parser.add_argument("--bag", type=Path, required=True, help="completed rosbag2 SQLite .db3")
    parser.add_argument("--output", type=Path, required=True, help="new output directory")
    parser.add_argument("--stage", type=int, choices=(0, 1, 2, 3), default=3)
    parser.add_argument("--retention-percent", type=int, default=100,
                        help="expected retention in the mapper profile (1-100, default: %(default)s)")
    parser.add_argument("--rate", type=float, choices=(0.5, 1.0), default=0.5)
    args = parser.parse_args()
    if not 1 <= args.retention_percent <= 100:
        parser.error("--retention-percent must be in [1, 100]")
    if os.environ.get("ROS_DOMAIN_ID") not in {"83", "84"}:
        parser.error("choose isolated ROS_DOMAIN_ID=83 or 84")
    if not args.runner.is_file() or not args.bag.is_file():
        parser.error("--runner and --bag must point to existing files")
    raise SystemExit(run(args))


if __name__ == "__main__":
    main()
