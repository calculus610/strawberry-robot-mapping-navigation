#!/usr/bin/env python3
"""Compare original and stage-0 results from the same stationary ROS fixture."""

import argparse
import json
from pathlib import Path

import numpy as np
from check_saved_native180 import (
    RESOLUTION,
    check,
    ground_coordinates,
    read_pcd_xyz,
    read_pgm,
    verify_saved_session,
)
import yaml


def nearest_distances(first, second):
    distances = []
    for start in range(0, len(first), 128):
        delta = first[start:start + 128, None, :] - second[None, :, :]
        distances.extend(np.sqrt(np.min(np.sum(delta * delta, axis=2), axis=1)))
    return np.asarray(distances)


def grid_cells(directory):
    image = read_pgm(directory / "map.pgm")
    metadata = yaml.safe_load((directory / "map.yaml").read_text())
    index = np.rint(np.asarray(metadata["origin"][:2]) / RESOLUTION).astype(np.int64)
    rows, columns = np.where(image == 0)
    return {(int(x), int(y)) for x, y in np.column_stack((columns, rows)) + index}


def covered(source, target):
    nearby = {(x + dx, y + dy) for x, y in target for dx in (-1, 0, 1) for dy in (-1, 0, 1)}
    return len(source & nearby) / len(source)


def compare(original, restored):
    sessions = [verify_saved_session(directory) for directory in (original, restored)]
    reports = [json.loads((directory / "contract_result.json").read_text()) for directory in (original, restored)]
    check([report["stage"] for report in reports] == ["original", "0"], "Expected original then Stage 0")
    check(reports[0]["raw_points_min_max"] == reports[1]["raw_points_min_max"], "Raw forward scan density changed")

    points = []
    for directory, report in zip((original, restored), reports):
        cloud = read_pcd_xyz(directory / "projection_source_camera_init.pcd")
        check(0 < len(cloud) <= 4000, "Comparator expects the small stationary synthetic fixture")
        points.append(ground_coordinates(cloud, report["ground"], report["rotation"]))
    counts = list(map(len, points))
    check(abs(counts[0] - counts[1]) / max(counts) <= 0.05, f"Native point counts differ: {counts}")

    distances = [nearest_distances(first, second) for first, second in (points, points[::-1])]
    p95 = [float(np.percentile(values, 95)) for values in distances]
    check(max(p95) <= 0.04, f"Native point geometry differs: directed P95 distances {p95}")
    cells = [grid_cells(directory) for directory in (original, restored)]
    check(all(cells), "Both fixtures need occupied cells")
    coverage = [covered(first, second) for first, second in (cells, cells[::-1])]
    check(min(coverage) >= 0.98, f"Occupied cells differ: one-cell directed coverage {coverage}")

    ground = [np.asarray(report["ground"]) for report in reports]
    difference = float(np.linalg.norm(ground[0] - ground[1]))
    check(difference <= 0.02, f"Stationary fixed references differ by {difference} m")
    return {
        "passed": True,
        "point_counts": counts,
        "native_nearest_p95_m": p95,
        "occupied_cells": list(map(len, cells)),
        "occupied_exact_coordinate_overlap": len(cells[0] & cells[1]) / len(cells[0] | cells[1]),
        "occupied_one_cell_coverage": coverage,
        "ground_position_difference_m": difference,
        "each_saved_map_reconstructed_exactly": len(sessions) == 2,
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("original", type=Path)
    parser.add_argument("restored", type=Path)
    args = parser.parse_args()
    print(json.dumps(compare(args.original, args.restored), indent=2))


if __name__ == "__main__":
    main()
