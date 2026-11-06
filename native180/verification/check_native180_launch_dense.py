#!/usr/bin/env python3
"""Run native180 with an optional dense reveal of the background wall."""

import argparse
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
from check_saved_native180 import (
    RESOLUTION,
    check,
    native_projection,
    read_pcd_xyz,
    verify_saved_session,
)


NANOSECONDS = 1_000_000_000
MAP_TOPIC = "/map"
TREE_TOPIC = "/fastlio/native_map"
SCAN_TOPIC = "/fastlio/free_scan"
ORIGIN_TOPIC = "/fastlio/free_scan_pose"
GROUND_TOPIC = "/fastlio/native_height_reference"
MATCH_TOPIC = "/fastlio/matching_input"
METRICS_TOPIC = "/fastlio/optimization_metrics"
SPARSE_REVEAL_RESULT = (
    "The original sparse reveal retained six panel points. After the unchanged "
    "every-third-point preprocessing, the estimated closest background rays "
    "passed 20.34–33.85 mm from those points. Points without supported ray "
    "crossings are retained. The dense reveal exercises clearing with coverage."
)


def stamp_ns(stamp):
    return stamp.sec * NANOSECONDS + stamp.nanosec


def xyz(vector):
    return np.array([vector.x, vector.y, vector.z])


def rotation(orientation):
    x, y, z, w = (
        orientation.x, orientation.y, orientation.z, orientation.w
    )
    check(abs(x*x + y*y + z*z + w*w - 1.0) <= 1e-5, "Reference quaternion is not normalized")
    return np.array([
        [1 - 2*(y*y + z*z), 2*(x*y - z*w), 2*(x*z + y*w)],
        [2*(x*y + z*w), 1 - 2*(x*x + z*z), 2*(y*z - x*w)],
        [2*(x*z - y*w), 2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


def cloud_xyz(message):
    check(not message.is_bigendian, "Expected little-endian synthetic test cloud")
    fields = {field.name: field for field in message.fields}
    xyz_fields = [fields[axis] for axis in ("x", "y", "z")]
    check(
        all(field.datatype == 7 and field.count == 1 for field in xyz_fields),
        "Expected scalar float32 XYZ",
    )
    return np.column_stack([
        np.ndarray(
            (message.height, message.width), buffer=message.data, dtype="<f4",
            offset=field.offset, strides=(message.row_step, message.point_step),
        ).ravel()
        for field in xyz_fields
    ])


def fixture(panel_present):
    """Three-axis support for registration, with a removable panel and rear returns."""
    points = []
    for x in (-1.85, 2.55, 4.0):
        for y in np.arange(-2.0, 2.01, 0.08):
            for z in np.arange(-0.217, 1.40, 0.075):
                offset = 0.005 * np.sin(71*y + 107*z) if x == 2.55 else 0.0
                if panel_present and x == 2.55 and -0.4 < y < 0.7 and 0.13 < z < 1.0:
                    # Occlude the background with hits on the same lines of sight.
                    scale = 1.38 / x
                    points.append((1.38, float(y * scale), float(z * scale)))
                else:
                    points.append((float(x + offset), float(y), float(z)))
    for y in (-1.65, 1.65):
        for x in np.arange(0.27, 3.90, 0.11):
            for z in np.arange(-0.217, 1.35, 0.12):
                points.append((float(x), y, float(z)))
    for x in np.arange(0.27, 3.90, 0.11):
        for y in np.arange(-1.56, 1.57, 0.11):
            points.append((float(x), float(y), -0.247))
    return points


def dense_reveal_fixture():
    points = fixture(False)
    # The old fixed scan order and point_filter_num=3 left 20.34–33.85 mm gaps
    # to the closest rays at the six retained panel points. Keep the production
    # 2 cm threshold. Add first returns on the same x=2.55 m wall surface only
    # after the panel is gone, with a border around its projected footprint.
    # Appending preserves every pre-existing point's preprocessing index.
    for y in np.arange(-0.51, 0.82, 0.02):
        for z in np.arange(0.025, 1.115, 0.015):
            x = 2.55 + 0.005 * np.sin(71*y + 107*z)
            points.append((float(x), float(y), float(z)))
    return points


def panel_points(cloud):
    return int(np.count_nonzero(
        (1.16 < cloud[:, 0]) & (cloud[:, 0] < 1.60)
        & (-0.34 < cloud[:, 1]) & (cloud[:, 1] < 0.51)
        & (0.02 < cloud[:, 2]) & (cloud[:, 2] < 0.72)
    ))


def write_temporal_capture(out, stage, phases, panel_stamp, scans, poses, trees, metrics, invalid_metrics):
    counts = [
        {"stamp_ns": stamp, "panel_points": panel_points(cloud_xyz(trees[stamp]))}
        for stamp in sorted(trees)
    ]
    report = {
        "stage": stage,
        "first_lidar_input_stamp_ns": phases,
        "panel_native_stamp_ns": panel_stamp,
        "metrics": [metrics[stamp] for stamp in sorted(metrics)],
        "invalid_metrics": invalid_metrics,
        "native_panel_counts": counts,
        "sparse_baseline": SPARSE_REVEAL_RESULT,
    }
    (out / "metrics.json").write_text(json.dumps(report, indent=2) + "\n")

    paired = sorted(scans.keys() & poses.keys())
    keep = set(paired[-10:])
    if "dense_reveal" in phases:
        keep.update(stamp for stamp in paired if stamp >= phases["dense_reveal"])
    stamps = sorted(keep)
    clouds, translations, quaternions = [], [], []
    offsets = [0]
    for stamp in stamps:
        cloud = cloud_xyz(scans[stamp])
        pose = poses[stamp].pose
        q = pose.orientation
        clouds.append(cloud)
        translations.append(xyz(pose.position))
        quaternions.append((q.x, q.y, q.z, q.w))
        offsets.append(offsets[-1] + len(cloud))

    empty = np.empty((0, 3), dtype=np.float32)
    latest = max(trees, default=-1)
    np.savez_compressed(
        out / "temporal_capture.npz",
        raw_stamp_ns=np.asarray(stamps, dtype=np.int64),
        raw_xyz=np.concatenate(clouds, axis=0) if clouds else empty,
        raw_offsets=np.asarray(offsets, dtype=np.int64),
        raw_pose_translation=np.asarray(translations, dtype=np.float64).reshape(-1, 3),
        raw_pose_quat_xyzw=np.asarray(quaternions, dtype=np.float64).reshape(-1, 4),
        latest_native_stamp_ns=np.int64(latest),
        latest_native_xyz=cloud_xyz(trees[latest]) if latest in trees else empty,
        panel_native_stamp_ns=np.int64(panel_stamp if panel_stamp is not None else -1),
        panel_native_xyz=cloud_xyz(trees[panel_stamp]) if panel_stamp in trees else empty,
    )


def run(args):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from livox_ros_driver2.msg import CustomMsg, CustomPoint
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import Imu, PointCloud2
    from std_msgs.msg import String
    from std_srvs.srv import Trigger
    from tf2_msgs.msg import TFMessage

    out = args.output.resolve()
    out.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = str(out / "ros_logs")
    os.environ.setdefault("OPENBLAS_NUM_THREADS", "1")
    os.environ.setdefault("OMP_NUM_THREADS", "1")
    command = [sys.executable, str(args.runner.resolve()), str(out),
               "start_lidar:=false", "start_rviz:=false"]
    if args.stage != "original":
        command.append(f"optimization_stage:={args.stage}")

    maps, trees, scans, poses, matches, metrics = {}, {}, {}, {}, {}, {}
    refs = []
    projection_transforms = []
    invalid_metrics = []

    def collect(target):
        def receive(message):
            target[stamp_ns(message.header.stamp)] = message
        return receive

    def collect_metrics(message):
        try:
            entry = json.loads(message.data)
            key = entry["stamp_ns"] if "stamp_ns" in entry else entry["stamp"]
            check(isinstance(key, int) and not isinstance(key, bool), "Metric stamp must be integer ns")
            metrics[key] = entry
        except (ValueError, KeyError, TypeError) as error:
            invalid_metrics.append(str(error))

    def collect_static_tf(message):
        projection_transforms.extend(
            transform for transform in message.transforms if transform.child_frame_id == "native_projection"
        )

    rclpy.init(args=[])
    node = rclpy.create_node("native180_contract_source")
    latched = QoSProfile(
        depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    node.create_subscription(OccupancyGrid, MAP_TOPIC, collect(maps), latched)
    node.create_subscription(PointCloud2, TREE_TOPIC, collect(trees), best)
    node.create_subscription(PoseStamped, GROUND_TOPIC, refs.append, latched)
    node.create_subscription(PointCloud2, SCAN_TOPIC, collect(scans), best)
    node.create_subscription(PoseStamped, ORIGIN_TOPIC, collect(poses), best)
    node.create_subscription(PointCloud2, MATCH_TOPIC, collect(matches), best)
    node.create_subscription(String, METRICS_TOPIC, collect_metrics, best)
    node.create_subscription(TFMessage, "/tf_static", collect_static_tf, latched)
    imu = node.create_publisher(Imu, "/livox/imu", 200)
    lidar = node.create_publisher(CustomMsg, "/livox/lidar", 5)
    save = node.create_client(Trigger, "/native_map_save")
    fixture_points = {"initial": fixture(False)}
    if args.exercise_dynamic:
        fixture_points["panel"] = fixture(True)
        fixture_points["dense_reveal"] = dense_reveal_fixture()

    scene = {}
    for name, points in fixture_points.items():
        packet = CustomMsg()
        packet.header.frame_id = "livox_frame"
        packet.point_num = len(points)
        packet.points = [
            CustomPoint(
                x=x, y=y, z=z, reflectivity=80, tag=0x10, line=i % 4,
                offset_time=i * 95_000_000 // (len(points) - 1),
            )
            for i, (x, y, z) in enumerate(points)
        ]
        scene[name] = packet

    phase = "initial"
    first_input_stamps = {}
    panel_stamp = None

    def publish_imu():
        message = Imu()
        message.header.stamp = node.get_clock().now().to_msg()
        message.header.frame_id = "livox_frame"
        message.linear_acceleration.z = 1.0
        imu.publish(message)

    def publish_lidar():
        message = scene[phase]
        stamp = node.get_clock().now().nanoseconds - 110_000_000
        message.header.stamp.sec, message.header.stamp.nanosec = divmod(stamp, NANOSECONDS)
        message.timebase = stamp
        first_input_stamps.setdefault(phase, stamp)
        lidar.publish(message)

    with (out / "launch.log").open("w") as log:
        process = None

        def spin_until(predicate, seconds, failure):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
                check(process.poll() is None, f"Launch exited early ({process.returncode}); see {out / 'launch.log'}")
                if predicate():
                    return
            raise ValueError(f"{failure}; see {out / 'launch.log'}")

        child_groups = []
        try:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            # Each child is the launch session created by run_session.py. Record
            # it only for cleanup if the supervisor fails to handle SIGINT.
            children_file = Path(f"/proc/{process.pid}/task/{process.pid}/children")

            def ready():
                if children_file.is_file():
                    child_groups[:] = [int(child) for child in children_file.read_text().split()]
                return lidar.get_subscription_count() == 1 and imu.get_subscription_count() == 1

            spin_until(ready, 15, "Synthetic sensors did not discover exactly one subscriber each")
            imu_timer = node.create_timer(0.005, publish_imu)
            lidar_timer = node.create_timer(0.1, publish_lidar)
            spin_until(
                lambda: bool(refs) and bool(projection_transforms) and len(maps.keys() & trees.keys()) >= 5
                and len(scans.keys() & poses.keys()) >= 5,
                45,
                "Missing fixed reference, five native map/grid pairs, or five scan/optical pose pairs",
            )
            dynamic = None
            if args.exercise_dynamic:
                initial_panel_count = panel_points(cloud_xyz(trees[max(trees)]))
                check(initial_panel_count == 0, "Dynamic fixture panel ROI was occupied before its introduction")
                panel_since = max(trees)
                phase = "panel"
                spin_until(
                    lambda: max(trees) > panel_since and panel_points(cloud_xyz(trees[max(trees)])) > 0,
                    15,
                    "Temporarily stationary panel never entered the native tree",
                )
                panel_stamp = max(trees)
                admitted_panel_count = panel_points(cloud_xyz(trees[panel_stamp]))
                phase = "dense_reveal"
                spin_until(
                    lambda: max(trees) > panel_stamp and panel_points(cloud_xyz(trees[max(trees)])) == 0
                    and any(entry["native_removed"] > 0 for stamp, entry in metrics.items() if stamp > panel_stamp),
                    15,
                    "The panel did not leave the native tree after dense background observations",
                )
                dynamic = {
                    "native_panel_points_before": admitted_panel_count,
                    "native_panel_points_after": 0,
                    "clear_fixture": "dense_reveal",
                    "sparse_baseline": SPARSE_REVEAL_RESULT,
                }

            paired = maps.keys() & trees.keys()
            raw_pairs = scans.keys() & poses.keys()
            for topic in (MAP_TOPIC, TREE_TOPIC, SCAN_TOPIC, ORIGIN_TOPIC, GROUND_TOPIC):
                check(node.count_publishers(topic) == 1, f"Expected exactly one publisher on {topic}")
            for topic in ("/manual_mapping/map", "/manual_mapping/filtered_map", "/manual_mapping/lidar_origin_body"):
                check(node.count_publishers(topic) == 0, f"Parallel legacy output path present: {topic}")

            ref = refs[0].pose
            ground, basis = xyz(ref.position), rotation(ref.orientation)
            check(all(message.header.frame_id == "camera_init" for message in refs), "Wrong reference frame")
            for message in refs:
                check(np.array_equal(ground, xyz(message.pose.position)), "The ground reference moved")
                check(np.array_equal(basis, rotation(message.pose.orientation)), "The ground orientation moved")
            for transform in projection_transforms:
                check(transform.header.frame_id == "camera_init", "Wrong native_projection TF parent")
                check(np.array_equal(ground, xyz(transform.transform.translation)), "Static TF origin differs")
                check(np.array_equal(basis, rotation(transform.transform.rotation)), "Static TF orientation differs")
            for stamp in sorted(paired):
                tree, grid = trees[stamp], maps[stamp]
                check(tree.header.frame_id == "camera_init", "Native tree was published in the wrong frame")
                check(grid.header.frame_id == "native_projection", "/map is not in native_projection")
                check(abs(grid.info.resolution - RESOLUTION) <= 1e-8, "/map resolution changed")
                cloud = cloud_xyz(tree)
                check(cloud[:, 0].min() > -0.5, "Rear wall entered the native tree")
                occupied, minimum, count = native_projection(cloud, ground, basis)
                actual = np.asarray(grid.data, dtype=np.int8).reshape(grid.info.height, grid.info.width)
                check(set(np.unique(actual)) <= {-1, 0, 100}, "Unexpected occupancy values")
                check(count > 0 and np.array_equal(occupied, actual == 100), "Black cells differ from the native tree")
                check(
                    np.allclose(
                        [grid.info.origin.position.x, grid.info.origin.position.y],
                        minimum * RESOLUTION, atol=1e-8, rtol=0,
                    ),
                    "Grid canvas differs from all-height native source bounds",
                )
            raw_counts = []
            for stamp in sorted(raw_pairs):
                scan, pose = scans[stamp], poses[stamp]
                check(scan.header.frame_id == "native_scan_lidar", "Wrong raw observation frame")
                check(pose.header.frame_id == "camera_init", "Wrong optical origin frame")
                cloud = cloud_xyz(scan)
                check(np.isfinite(cloud).all() and cloud[:, 0].min() >= -1e-5, "Rear/invalid raw point survived")
                check(np.linalg.norm(cloud, axis=1).max() > 3.5, "Raw rays were cropped to the matching radius")
                height = ((xyz(pose.pose.position) - ground) @ basis)[2]
                check(abs(height - 0.247) < 0.05, f"Optical origin height disagrees with ground: {height}")
                raw_counts.append(len(cloud))

            optimization = None
            if args.stage in ("1", "2", "3"):
                common = sorted(set(raw_pairs) & matches.keys() & metrics.keys())
                check(not invalid_metrics, f"Malformed metric messages: {invalid_metrics[:3]}")
                check(len(common) >= 5, "Missing five exact raw/optical-origin/matching-input/metrics pairs")
                sampled, eligible, moved, removed = [], [], 0, 0
                for stamp in common:
                    row = metrics[stamp]
                    raw, match = cloud_xyz(scans[stamp]), cloud_xyz(matches[stamp])
                    eligible_xyz = raw[np.linalg.norm(raw.astype(np.float64), axis=1) <= 3.0]
                    count = len(eligible_xyz)
                    check(matches[stamp].header.frame_id == "native_scan_lidar", "Unexpected matching frame")
                    check(row["stage"] == int(args.stage), "Wrong optimization stage")
                    check(row["forward_points"] == len(raw), "Original raw observation and forward count differ")
                    check(row["eligible_points"] == count and 0 < count < len(raw), "3 m candidate count differs")
                    check(row["sampled_points"] == count * 70 // 100, "Expected floor(0.70 * N) points")
                    check(row["matching_input_points"] == len(match) > 0, "Matching input count differs")
                    check(np.isfinite(match).all(), "Nonfinite matching input")
                    max_range = float(np.linalg.norm(match.astype(np.float64), axis=1).max())
                    check(max_range <= 3.0 + 1e-5, f"Matching input exceeds 3 m: {max_range}")
                    check(abs(row["matching_max_range_m"] - max_range) <= 1e-5, "Matching radius differs from metrics")
                    check(row["matching_behind_points"] == 0 and match[:, 0].min() >= -1e-5, "Rear matching input")
                    if args.stage == "1":
                        check(len(match) == row["sampled_points"], "Stage 1 unexpectedly changed sampled count")
                        check(row["smoothing_moved"] == row["smoothing_removed"] == 0, "Stage 1 ran geometry")
                        sample_count = row["sampled_points"]
                        indices = ((2 * np.arange(sample_count) + 1) * count) // (2 * sample_count)
                        check(np.array_equal(match, eligible_xyz[indices]), "Stage 1 changed coordinates or sample order")
                    else:
                        check(
                            len(match) == row["sampled_points"] - row["smoothing_removed"],
                            "Geometry did not account for every sampled point",
                        )
                    sampled.append(row["sampled_points"])
                    eligible.append(count)
                    moved += row["smoothing_moved"]
                    removed += row["smoothing_removed"]
                optimization = {
                    "exact_diagnostic_pairs": len(common),
                    "eligible_min_max": [min(eligible), max(eligible)],
                    "sampled_min_max": [min(sampled), max(sampled)],
                    "smoothing_moved_sum": moved,
                    "smoothing_removed_sum": removed,
                    "native_admitted_sum": sum(row["native_admitted"] for row in metrics.values()),
                    "native_removed_sum": sum(row["native_removed"] for row in metrics.values()),
                }

            node.destroy_timer(imu_timer)
            node.destroy_timer(lidar_timer)
            spin_until(save.service_is_ready, 5, "/native_map_save unavailable")
            future = save.call_async(Trigger.Request())
            spin_until(future.done, 10, "/native_map_save did not reply")
            reply = future.result()
            pcd = out / "native_map_final_camera_init.pcd"
            check(reply.success and pcd.is_file(), f"Native save service failed: {reply.message}")
            service_cloud = read_pcd_xyz(pcd)
            check(len(service_cloud) > 0 and np.isfinite(service_cloud).all(), "Native save produced an invalid PCD")
            pcd.unlink()  # Final shutdown must create the file independently of the service.
        finally:
            try:
                if process is not None and process.poll() is None:
                    process.send_signal(signal.SIGINT)
                    try:
                        process.wait(timeout=20)
                    except subprocess.TimeoutExpired:
                        for group in child_groups:
                            try:
                                if os.getpgid(group) == group:
                                    os.killpg(group, signal.SIGKILL)
                            except ProcessLookupError:
                                continue
                        process.kill()
                        process.wait(timeout=5)
                        raise RuntimeError(f"Supervisor failed to stop; see {out / 'launch.log'}")
            finally:
                try:
                    write_temporal_capture(out, args.stage, first_input_stamps, panel_stamp,
                                           scans, poses, trees, metrics, invalid_metrics)
                finally:
                    node.destroy_node()
                    rclpy.shutdown()

    check(process.returncode == 0, f"Shutdown failed with {process.returncode}; see {out / 'launch.log'}")
    text = (out / "launch.log").read_text()
    for message in ("FORWARD180", "PROJECTION_READY", "NATIVE_MAP_SAVED", "SESSION_SAVED:"):
        check(message in text, f"Missing {message} in launch log")
    check("SESSION_SAVE_INCOMPLETE" not in text and "livox_ros_driver2_node" not in text,
          "Session save was incomplete or real driver was started")
    saved = verify_saved_session(out)
    source_stamp = saved["map_stamp_ns"]
    check(source_stamp in trees, "No captured native tree at the saved projection stamp")
    saved_source = read_pcd_xyz(out / "projection_source_camera_init.pcd")
    check(np.array_equal(saved_source, cloud_xyz(trees[source_stamp])), "Saved projection source differs from live native tree")
    check(saved["occupied_cells"] > 0, "Fixture produced no obstacle cells")
    if dynamic is not None:
        final = read_pcd_xyz(out / "native_map_final_camera_init.pcd")
        for name, cloud in (("service PCD", service_cloud), ("projection source", saved_source), ("final PCD", final)):
            check(panel_points(cloud) == 0, f"Cleared panel survived in {name}")
    summary = {
        "stage": args.stage,
        "session": str(out),
        "ground": ground.tolist(),
        "rotation": basis.tolist(),
        "raw_points_min_max": [min(raw_counts), max(raw_counts)],
        "exact_raw_pose_pairs": len(raw_pairs),
        "exact_native_map_pairs": len(paired),
        "native_save_service_points": len(service_cloud),
        "saved": saved,
        "optimization": optimization,
        "dynamic": dynamic,
    }
    (out / "contract_result.json").write_text(json.dumps(summary, indent=2) + "\n")
    print(json.dumps(summary, indent=2))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--runner", type=Path, required=True, help="run_session.py in a built isolated workspace copy")
    parser.add_argument("--stage", required=True, choices=("original", "0", "1", "2", "3"))
    parser.add_argument("--output", type=Path, required=True, help="new session directory")
    parser.add_argument("--exercise-dynamic", action="store_true", help="temporarily occlude a wall for stage 3")
    args = parser.parse_args()
    if not args.runner.is_file():
        parser.error("--runner must point to an existing supervisor")
    if os.environ.get("ROS_DOMAIN_ID") not in {"83", "84"}:
        parser.error("choose isolated ROS_DOMAIN_ID=83 or 84 and source only the intended workspace")
    if args.exercise_dynamic and args.stage != "3":
        parser.error("--exercise-dynamic requires --stage 3")
    run(args)


if __name__ == "__main__":
    main()
