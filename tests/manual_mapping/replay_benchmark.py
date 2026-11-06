"""Benchmark the manual mapping stages using a read-only ROS 2 SQLite bag.

Source ROS 2 and set PYTHONPATH to the directory containing manual_cloud.
The positional bag is optional. Without it, only the synthetic test runs.
"""

import argparse
from collections import Counter
from contextlib import closing
import json
from pathlib import Path
import sqlite3
from time import perf_counter

import numpy as np

from manual_cloud.sampling import read_xyz, sample_frame
from manual_cloud.temporal import TemporalMap, TemporalOptions


_ORIGIN = np.zeros(3)
_RANGE_M = 3.0
_RETENTION_PERCENT = 70
_PREPROCESS_STRIDE = 3


def _stats(values):
    if not values:
        return None
    p50, p95 = np.percentile(values, [50, 95])
    return {"min": float(min(values)), "p50": float(p50), "p95": float(p95), "max": float(max(values))}


def _cloud_layout(cloud):
    return {
        "frame_id": cloud.header.frame_id,
        "height": cloud.height,
        "width": cloud.width,
        "point_step": cloud.point_step,
        "row_step": cloud.row_step,
        "is_bigendian": cloud.is_bigendian,
        "fields": [dict(name=f.name, offset=f.offset, datatype=f.datatype, count=f.count) for f in cloud.fields],
    }


def benchmark_bag(path, topic, frame_limit, max_clear_rays):
    from rclpy.serialization import deserialize_message
    from sensor_msgs.msg import PointCloud2
    from manual_cloud.smoothing import smooth_and_filter

    raw_counts, stride_counts, range_counts, selected_counts = [], [], [], []
    sample_times, smooth_times, temporal_times = [], [], []
    sample_stats, smooth_stats, temporal_status, temporal_stats = Counter(), Counter(), Counter(), Counter()
    min_selected, max_selected, first_stamp, last_stamp, layout = None, None, None, None, None
    radius_count_mismatches = selection_count_mismatches = selected_outside_range = 0
    temporal = TemporalMap({"max_voxels": 500_000, "max_clear_rays": max_clear_rays})

    uri = path.expanduser().resolve().as_uri() + "?mode=ro"
    with closing(sqlite3.connect(uri, uri=True)) as connection:
        matches = connection.execute(
            "SELECT id, type, serialization_format FROM topics WHERE name=?", (topic,),
        ).fetchall()
        if len(matches) != 1 or matches[0][1:] != ("sensor_msgs/msg/PointCloud2", "cdr"):
            raise ValueError(f"{topic} must be a single CDR sensor_msgs/msg/PointCloud2 topic in the bag")
        topic_id = matches[0][0]
        bag_count = connection.execute("SELECT count(*) FROM messages WHERE topic_id=?", (topic_id,)).fetchone()[0]
        rows = connection.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp, id LIMIT ?", (topic_id, frame_limit),
        )
        for (payload,) in rows:
            cloud = deserialize_message(payload, PointCloud2)
            if layout is None:
                layout = _cloud_layout(cloud)
            stamp = cloud.header.stamp.sec * 1_000_000_000 + cloud.header.stamp.nanosec
            if first_stamp is None:
                first_stamp = stamp
            last_stamp = stamp

            complete = read_xyz(cloud)
            raw = complete[::_PREPROCESS_STRIDE]
            raw_counts.append(len(complete))
            stride_counts.append(len(raw))

            started = perf_counter()
            frame = sample_frame(raw, _ORIGIN, _RANGE_M, _RETENTION_PERCENT)
            sample_times.append((perf_counter() - started) * 1000)
            sample_stats.update(frame.counts)
            range_counts.append(len(frame.in_range_points))
            selected_counts.append(len(frame.selected_points))
            valid = raw[np.isfinite(raw).all(axis=1)]
            expected_in_range = int(np.count_nonzero(np.linalg.norm(valid, axis=1) <= _RANGE_M))
            radius_count_mismatches += len(frame.in_range_points) != expected_in_range
            selection_count_mismatches += len(frame.selected_points) != expected_in_range * _RETENTION_PERCENT // 100
            if len(frame.selected_points):
                selected_distances = np.linalg.norm(frame.selected_points, axis=1)
                selected_outside_range += int(np.count_nonzero(selected_distances > _RANGE_M))
                low, high = float(selected_distances.min()), float(selected_distances.max())
                min_selected = low if min_selected is None else min(min_selected, low)
                max_selected = high if max_selected is None else max(max_selected, high)

            started = perf_counter()
            cleaned, counts = smooth_and_filter(frame.selected_points, frame.in_range_points)
            smooth_times.append((perf_counter() - started) * 1000)
            within_range = np.linalg.norm(cleaned, axis=1) <= _RANGE_M
            counts["outside_range_after_smoothing"] = int(np.count_nonzero(~within_range))
            cleaned = cleaned[within_range]
            smooth_stats.update(counts)

            started = perf_counter()
            metrics = temporal.update(cleaned, frame.finite_points, _ORIGIN,
                                      (stamp - first_stamp) / 1_000_000_000, allow_clear=False)
            temporal_times.append((perf_counter() - started) * 1000)
            temporal_status[metrics["status"]] += 1
            temporal_stats.update({key: metrics[key] for key in ("admitted", "removed", "clear_rays")})

    if not raw_counts:
        raise ValueError(f"the bag has no messages for {topic}")
    return {
        "scope": (
            "Input distribution and stage timings only. Raw sensor coordinates were used with identity pose. "
            "No deskew, FAST-LIO registration, or true map replay. Raw index stride approximates point_filter_num=3 "
            "without FAST-LIO tag, line, blind-range, or duplicate filtering. Clearing was disabled."
        ),
        "bag": str(path),
        "topic": topic,
        "bag_topic_messages": bag_count,
        "frames_read": len(raw_counts),
        "first_cloud_layout": layout,
        "header_span_s": (last_stamp - first_stamp) / 1_000_000_000,
        "preprocess_stride": _PREPROCESS_STRIDE,
        "range_m": _RANGE_M,
        "retention_percent": _RETENTION_PERCENT,
        "raw_points_per_frame": _stats(raw_counts),
        "stride_points_per_frame": _stats(stride_counts),
        "v1": {
            "counts_total": dict(sample_stats),
            "eligible_per_frame": _stats(range_counts),
            "selected_per_frame": _stats(selected_counts),
            "radius_count_mismatches": radius_count_mismatches,
            "selection_count_mismatches": selection_count_mismatches,
            "selected_outside_range": selected_outside_range,
            "min_selected_distance_m": min_selected,
            "max_selected_distance_m": max_selected,
            "sample_ms": _stats(sample_times),
        },
        "v2": {"counts_total": dict(smooth_stats), "smooth_and_filter_ms": _stats(smooth_times)},
        "v3": {
            "allow_clear": False,
            "max_clear_rays": max_clear_rays,
            "update_statuses": dict(temporal_status),
            "totals": dict(temporal_stats),
            "map_voxels": len(temporal),
            "update_ms": _stats(temporal_times),
        },
    }


def benchmark_synthetic(point_count, frames, max_clear_rays):
    """Time temporal updates with clearing in a fixed, ray-sampled room."""
    temporal = TemporalMap({"max_voxels": 500_000, "max_clear_rays": max_clear_rays})
    rng = np.random.default_rng(42)
    lower = np.array([-2.0, -1.8, -0.42])
    upper = np.array([4.5, 2.5, 1.8])
    updates, available, rays, skipped, removals = [], [], [], [], 0
    for frame_index in range(frames):
        az = rng.uniform(-np.pi, np.pi, point_count)
        el = rng.uniform(np.radians(-7), np.radians(52), point_count)
        direction = np.column_stack((np.cos(el) * np.cos(az), np.cos(el) * np.sin(az), np.sin(el)))
        distance = np.divide(np.where(direction > 0, upper, lower), direction,
                             out=np.full_like(direction, np.inf), where=direction != 0).min(axis=1)
        raw = direction * distance[:, None]
        frame = sample_frame(raw, _ORIGIN, _RANGE_M, _RETENTION_PERCENT)

        started = perf_counter()
        metrics = temporal.update(frame.selected_points, frame.finite_points, _ORIGIN,
                                  frame_index / 10, allow_clear=True)
        elapsed_ms = (perf_counter() - started) * 1000
        if frame_index >= 3:
            updates.append(elapsed_ms)
            available.append(metrics["available_clear_rays"])
            rays.append(metrics["clear_rays"])
            skipped.append(metrics["budget_skipped"])
        removals += metrics["removed"]

    return {
        "scope": "Fixed origin and fixed room with seeded varying ray directions. Timings exclude generation and sampling.",
        "allow_clear": True,
        "max_clear_rays": max_clear_rays,
        "raw_points_per_frame": point_count,
        "frames": frames,
        "warmup_frames_excluded": 3,
        "available_clear_rays_per_frame": _stats(available),
        "clear_rays_per_frame": _stats(rays),
        "budget_skipped_per_frame": _stats(skipped),
        "update_ms": _stats(updates),
        "map_voxels": len(temporal),
        "removed_voxels": removals,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", nargs="?", type=Path, help="read-only ROS 2 SQLite .db3 recording")
    parser.add_argument("--topic", default="/livox/lidar")
    parser.add_argument("--frames", type=int, default=60, help="maximum real frames to read (1–60)")
    parser.add_argument("--synthetic-frames", type=int, default=24, help="synthetic frames (4–60)")
    parser.add_argument("--synthetic-rays", type=int, help="returns per frame, defaults to the bag's stride-count median")
    parser.add_argument("--max-clear-rays", type=int, default=TemporalOptions().max_clear_rays,
                        help="maximum traced rays per frame; 0 is unlimited (default: %(default)s)")
    args = parser.parse_args()
    if not 1 <= args.frames <= 60 or not 4 <= args.synthetic_frames <= 60:
        parser.error("--frames must be 1–60 and --synthetic-frames must be 4–60")
    if args.synthetic_rays is not None and not 1 <= args.synthetic_rays <= 40_000:
        parser.error("--synthetic-rays must be 1–40000")
    if args.max_clear_rays < 0:
        parser.error("--max-clear-rays must be nonnegative")

    bag = benchmark_bag(args.bag, args.topic, args.frames, args.max_clear_rays) if args.bag else None
    default_rays = max(1, int(bag["stride_points_per_frame"]["p50"])) if bag else 6000
    rays = args.synthetic_rays if args.synthetic_rays is not None else min(default_rays, 40_000)
    synthetic = benchmark_synthetic(rays, args.synthetic_frames, args.max_clear_rays)
    print(json.dumps({"recorded_input": bag, "synthetic_static_room": synthetic}, indent=2, allow_nan=False))


if __name__ == "__main__":
    main()
