#!/usr/bin/env python3
"""Isolated native projector contract with known free rays and mismatched stamps."""

import argparse
from array import array
import json
import os
from pathlib import Path
import signal
import subprocess
import sys
import time

import numpy as np
from check_saved_native180 import RESOLUTION, check, native_projection, read_pgm


def run(projector, output):
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from nav_msgs.msg import OccupancyGrid
    from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
    from sensor_msgs.msg import PointCloud2, PointField

    output = output.resolve()
    output.mkdir(parents=True, exist_ok=False)
    os.environ["ROS_LOCALHOST_ONLY"] = "1"
    os.environ["ROS_LOG_DIR"] = str(output / "ros_logs")

    angle = np.deg2rad(15)
    quaternion = (0.0, float(np.sin(angle / 2)), 0.0, float(np.cos(angle / 2)))
    basis = np.array([[np.cos(angle), 0, np.sin(angle)], [0, 1, 0], [-np.sin(angle), 0, np.cos(angle)]])
    ground = np.array([-.011, -.02329, -.20288])
    optical = np.array([.025, .025, .247])
    native = np.array([[.025, -.475, 0.0], [5.025, .475, 0.0], [5.025, .025, 2.0], [2.025, .025, .60]])
    beams = np.array([[4.025, .025, .247 + 4 * (z - .247)] for z in np.arange(.15, 1.0, .1)])
    obstacle = np.array([[1.025, .025, .60]])

    def pose(stamp, point):
        message = PoseStamped()
        message.header.frame_id = "camera_init"
        message.header.stamp.sec = stamp
        message.pose.position.x, message.pose.position.y, message.pose.position.z = map(float, point)
        (message.pose.orientation.x, message.pose.orientation.y,
         message.pose.orientation.z, message.pose.orientation.w) = quaternion
        return message

    def cloud(points, stamp, frame):
        xyz = np.asarray(points, dtype="<f4").reshape(-1, 3)
        message = PointCloud2()
        message.header.frame_id = frame
        message.header.stamp.sec = stamp
        message.height, message.width = 1, len(xyz)
        message.point_step, message.row_step = 12, 12 * len(xyz)
        message.fields = [
            PointField(name=name, offset=index * 4, datatype=PointField.FLOAT32, count=1)
            for index, name in enumerate(("x", "y", "z"))
        ]
        message.data = array("B", xyz.tobytes())
        return message

    maps = {}
    rclpy.init(args=[])
    node = rclpy.create_node("native180_free_contract_source")
    latched = QoSProfile(
        depth=5, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL
    )
    best = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
    ref = node.create_publisher(PoseStamped, "/fastlio/native_height_reference", latched)
    tree = node.create_publisher(PointCloud2, "/fastlio/native_map", 5)
    scan = node.create_publisher(PointCloud2, "/fastlio/free_scan", best)
    optical_pose = node.create_publisher(PoseStamped, "/fastlio/free_scan_pose", best)
    node.create_subscription(
        OccupancyGrid, "/map", lambda message: maps.update({message.header.stamp.sec: message}), latched
    )
    with (output / "projector.log").open("w") as log:
        process = subprocess.Popen(
            [sys.executable, str(projector.resolve()), "--ros-args", "-p", f"output_dir:={output}"],
            stdout=log, stderr=subprocess.STDOUT,
        )

        def spin(seconds, predicate=None):
            end = time.monotonic() + seconds
            while time.monotonic() < end:
                rclpy.spin_once(node, timeout_sec=0.02)
                check(process.poll() is None, f"Projector exited early ({process.returncode})")
                if predicate is not None and predicate():
                    return
            check(predicate is None, "Projector expectation timed out")

        def snapshot(stamp, points=native):
            tree.publish(cloud(points @ basis.T + ground, stamp, "camera_init"))
            spin(8, lambda: stamp in maps)
            result = maps[stamp]
            check(result.header.frame_id == "native_projection", "Wrong grid frame")
            return result

        def rays(stamp, points):
            scan.publish(cloud(points - optical, stamp, "native_scan_lidar"))

        def value(grid, x):
            ix = int(np.floor(x / RESOLUTION) - round(grid.info.origin.position.x / RESOLUTION))
            iy = int(np.floor(.025 / RESOLUTION) - round(grid.info.origin.position.y / RESOLUTION))
            return grid.data[iy * grid.info.width + ix]

        def send_optical_pose(stamp):
            optical_pose.publish(pose(stamp, optical @ basis.T + ground))

        try:
            spin(10, lambda: all(pub.get_subscription_count() == 1 for pub in (ref, tree, scan, optical_pose)))
            ref.publish(pose(0, ground))
            spin(.25)
            first = snapshot(10)
            check(value(first, 1.025) == -1 and value(first, 2.025) == 100, "Wrong initial native obstacle mask")

            rays(11, beams[:-1])
            send_optical_pose(12)
            spin(.25)
            check(value(snapshot(13), 1.025) == -1, "Mismatched scan and pose supplied evidence")
            send_optical_pose(11)
            spin(.25)
            check(value(snapshot(14), 1.025) == -1, "Eight of nine layers produced white")
            rays(12, beams[-1:])
            spin(.25)
            check(value(snapshot(15), 1.025) == 0, "Nine measured layers did not produce white")

            rays(16, obstacle)
            send_optical_pose(16)
            spin(.25)
            check(value(snapshot(17), 1.025) == -1, "A raw endpoint did not reset its column")
            rays(18, beams)
            send_optical_pose(18)
            spin(.25)
            check(value(snapshot(19), 1.025) == 0, "Fresh evidence did not restore white")
            check(value(snapshot(20, np.vstack((native, obstacle))), 1.025) == 100, "White overrode a native obstacle")
            check(value(snapshot(21), 1.025) == -1, "Stale free evidence returned after native obstacle removal")

            rays(22, beams)
            send_optical_pose(22)
            spin(.25)
            final = snapshot(23)
            check(value(final, 1.025) == 0 and value(final, 2.025) == 100, "Expected fresh white and native black")
            expected, minimum, _ = native_projection((native @ basis.T + ground).astype(np.float32), ground, basis)
            actual = np.asarray(final.data, dtype=np.int8).reshape(final.info.height, final.info.width)
            check(np.array_equal(expected, actual == 100), "Free rays altered the native black mask")
            check(np.allclose(
                [final.info.origin.position.x, final.info.origin.position.y], minimum * RESOLUTION,
                atol=1e-8, rtol=0,
            ), "Free rays altered the native canvas")
        finally:
            if process.poll() is None:
                process.send_signal(signal.SIGINT)
                try:
                    process.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait()
                    raise
            node.destroy_node()
            rclpy.shutdown()

    check(process.returncode == 0, f"Projector exit code {process.returncode}")
    pixels = read_pgm(output / "map.pgm")
    check(np.array_equal(pixels, np.where(actual == 100, 0, np.where(actual == 0, 254, 205))),
          "PGM disagrees with last live grid")
    check(set(np.unique(pixels)) == {0, 205, 254}, "PGM did not preserve all three occupancy states")
    print(json.dumps({
        "passed": True,
        "checks": ["tilted fixed frame", "exact scan and origin pairing", "nine layers to white",
                   "raw endpoint resets a column", "native obstacle priority", "fresh evidence after reset",
                   "native canvas", "last live grid equals saved PGM"],
    }))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--projector", type=Path, required=True, help="installed native_height_projector.py")
    parser.add_argument("--output", type=Path, required=True, help="new session directory")
    args = parser.parse_args()
    if os.environ.get("ROS_DOMAIN_ID") not in {"83", "84"}:
        parser.error("choose isolated ROS_DOMAIN_ID=83 or 84")
    if not args.projector.is_file():
        parser.error("--projector must point to an installed projector")
    run(args.projector, args.output)


if __name__ == "__main__":
    main()
