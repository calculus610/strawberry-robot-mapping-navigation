#!/usr/bin/env python3
"""Reconstruct a saved native180 occupancy grid using only its session artifacts."""

import argparse
import json
from pathlib import Path

import numpy as np
import yaml


RESOLUTION = 0.05
LOW = 0.10
HIGH = 1.00
LAYER_STEP = 0.10
LAYERS = 9
COMPLETE = (1 << LAYERS) - 1
REQUIRED_FILES = (
    "map.yaml",
    "map.pgm",
    "projection_source_camera_init.pcd",
    "native_map_final_camera_init.pcd",
    "free_evidence.npz",
)


def check(condition, message):
    if not condition:
        raise ValueError(message)


def read_pcd_xyz(path):
    """Read XYZ from the binary uncompressed projector/PCL files."""
    metadata = {}
    with path.open("rb") as stream:
        for raw in stream:
            line = raw.strip().decode("ascii")
            if not line or line.startswith("#"):
                continue
            key, *values = line.split()
            metadata[key] = values
            if key == "DATA":
                break
        check(metadata.get("DATA") == ["binary"], f"{path}: expected binary PCD")
        payload = stream.read()

    fields = metadata["FIELDS"]
    sizes = list(map(int, metadata["SIZE"]))
    counts = list(map(int, metadata.get("COUNT", [1] * len(fields))))
    types = metadata["TYPE"]
    check(len(fields) == len(sizes) == len(counts) == len(types), f"{path}: malformed fields")
    points, = map(int, metadata["POINTS"])
    width, = map(int, metadata["WIDTH"])
    height, = map(int, metadata["HEIGHT"])
    check(points == width * height, f"{path}: inconsistent point count")
    field_bytes = np.asarray(sizes) * counts
    stride = int(field_bytes.sum())
    check(len(payload) == points * stride, f"{path}: wrong PCD payload length")
    offsets = np.r_[0, np.cumsum(field_bytes[:-1])]
    xyz = np.empty((points, 3), dtype=np.float32)
    for index, name in enumerate(("x", "y", "z")):
        field = fields.index(name)
        check(
            sizes[field] == 4 and counts[field] == 1 and types[field] == "F",
            f"{path}: {name} is not scalar float32",
        )
        if points:
            xyz[:, index] = np.ndarray(
                (points,), dtype="<f4", buffer=payload, offset=int(offsets[field]), strides=(stride,)
            )
    return xyz


def read_pgm(path):
    with path.open("rb") as stream:
        check(stream.readline().strip() == b"P5", f"{path}: expected P5")
        width, height = map(int, stream.readline().split())
        check(stream.readline().strip() == b"255", f"{path}: expected 8-bit PGM")
        payload = stream.read()
    check(width > 0 and height > 0 and len(payload) == width * height, f"{path}: invalid shape")
    return np.frombuffer(payload, dtype=np.uint8).reshape(height, width)[::-1]


def ground_coordinates(points, ground, basis):
    ground = np.asarray(ground, dtype=np.float64)
    basis = np.asarray(basis, dtype=np.float64)
    check(ground.shape == (3,) and np.isfinite(ground).all(), "Invalid reference origin")
    check(
        basis.shape == (3, 3)
        and np.isfinite(basis).all()
        and np.allclose(basis.T @ basis, np.eye(3), atol=1e-7, rtol=0)
        and np.isclose(np.linalg.det(basis), 1.0, atol=1e-7, rtol=0),
        "Invalid reference rotation",
    )
    xyz = points[np.isfinite(points).all(axis=1)].astype(np.float64)
    return (xyz - ground) @ basis


def native_projection(points, ground, basis):
    local = ground_coordinates(points, ground, basis)
    check(len(local) > 0, "Fixture must have a nonempty native snapshot")
    minimum = np.floor((np.min(local[:, :2], axis=0) - 0.5) / RESOLUTION).astype(np.int64)
    maximum = np.floor((np.max(local[:, :2], axis=0) + 0.5) / RESOLUTION).astype(np.int64)
    width, height = maximum - minimum + 1
    check(0 < width * height <= 10_000_000, "Native canvas exceeds limit")
    selected = local[(LOW - 1e-6 <= local[:, 2]) & (local[:, 2] <= HIGH + 1e-6)]
    occupied = np.zeros((height, width), dtype=bool)
    indices = np.floor(selected[:, :2] / RESOLUTION).astype(np.int64) - minimum
    occupied[indices[:, 1], indices[:, 0]] = True
    return occupied, minimum, len(selected)


def verify_saved_session(directory):
    """Verify all five session files, using projection_stats for the ground transform."""
    for name in REQUIRED_FILES:
        path = directory / name
        check(path.is_file() and path.stat().st_size > 0, f"Missing or empty result: {path}")

    stats = json.loads((directory / "projection_stats.json").read_text())
    check(stats["frame"] == "native_projection", "Saved projection frame changed")
    check(stats["source_frame"] == "camera_init", "Saved source frame changed")
    check(
        (stats["min_height"], stats["max_height"], stats["resolution"], stats["free_height_step"], stats["free_layers"])
        == (LOW, HIGH, RESOLUTION, LAYER_STEP, LAYERS),
        "Saved height band, resolution or layers changed",
    )

    source = read_pcd_xyz(directory / "projection_source_camera_init.pcd")
    final = read_pcd_xyz(directory / "native_map_final_camera_init.pcd")
    check(np.isfinite(source).all() and np.isfinite(final).all() and len(final), "PCD invalid or empty")
    occupied, minimum, selected = native_projection(source, stats["ground"], stats["rotation"])
    check(stats["native_points"] == len(source), "Native source count differs from projection statistics")
    check(stats["selected_points"] == selected, "Height-band point count differs from projection statistics")

    metadata = yaml.safe_load((directory / "map.yaml").read_text())
    check(metadata["image"] == "map.pgm", "Map metadata does not reference its own PGM")
    check(metadata["resolution"] == RESOLUTION, "Map metadata resolution changed")
    check(
        np.allclose(metadata["origin"], [*(minimum * RESOLUTION), 0.0], atol=1e-8, rtol=0),
        "Map metadata origin disagrees with the native cloud canvas",
    )

    with np.load(directory / "free_evidence.npz", allow_pickle=False) as evidence:
        check(
            tuple(float(evidence[k]) for k in ("resolution", "low", "high", "height_step"))
            == (RESOLUTION, LOW, HIGH, LAYER_STEP),
            "Free-evidence geometry differs from the map",
        )
        check(int(evidence["map_stamp_ns"]) == stats["stamp_ns"], "Saved map/cache timestamps differ")
        check(
            int(evidence["observation_stamp_ns"]) == stats["free_observation_stamp_ns"],
            "Saved cache/stats observation timestamps differ",
        )
        xy = evidence["xy"]
        masks = evidence["masks"]
    check(xy.shape == (len(masks), 2) and masks.ndim == 1, "Malformed free-evidence arrays")
    check(xy.dtype.kind in "iu" and masks.dtype.kind in "iu", "Free-evidence arrays must use integer indices")
    check(np.all((masks > 0) & (masks <= COMPLETE)), "Invalid nine-layer free-evidence mask")
    check(len(masks) == stats["free_cache_columns"], "Cache column count differs from statistics")

    local_indices = xy.astype(np.int64) - minimum
    height, width = occupied.shape
    check(
        np.all((local_indices >= 0) & (local_indices < [width, height])),
        "Free-evidence cache extends outside the native canvas",
    )
    columns = local_indices[:, 1] * width + local_indices[:, 0]
    check(len(np.unique(columns)) == len(columns), "Duplicate free-evidence columns")
    check(not occupied.ravel()[columns].any(), "A native obstacle did not invalidate its old free evidence")
    expected = np.full((height, width), 205, dtype=np.uint8)
    expected[occupied] = 0
    expected.ravel()[columns[masks == COMPLETE]] = 254

    pixels = read_pgm(directory / "map.pgm")
    check(pixels.shape == expected.shape, "Saved PGM shape differs from the native canvas")
    differences = np.count_nonzero(pixels != expected)
    check(differences == 0, f"{differences} PGM cells differ from native reprojection and nine-layer evidence")
    check(
        (stats["width"], stats["height"], stats["occupied_cells"], stats["free_cells"], stats["unknown_cells"])
        == (width, height, np.count_nonzero(expected == 0), np.count_nonzero(expected == 254), np.count_nonzero(expected == 205)),
        "Projection statistics disagree with saved PGM",
    )
    return {
        "session": str(directory),
        "map_stamp_ns": stats["stamp_ns"],
        "width": int(width),
        "height": int(height),
        "native_source_points": len(source),
        "native_final_points": len(final),
        "height_band_points": selected,
        "occupied_cells": stats["occupied_cells"],
        "free_cells": stats["free_cells"],
        "unknown_cells": stats["unknown_cells"],
        "verified_required_files": len(REQUIRED_FILES),
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("sessions", type=Path, nargs="+")
    args = parser.parse_args()
    print(json.dumps([verify_saved_session(path) for path in args.sessions], indent=2))


if __name__ == "__main__":
    main()
