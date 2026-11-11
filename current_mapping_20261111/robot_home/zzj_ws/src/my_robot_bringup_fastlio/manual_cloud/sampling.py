"""NumPy-only point-cloud decoding, sampling, and static accumulation."""

from dataclasses import dataclass

import numpy as np


def read_xyz(cloud):
    """Decode a PointCloud2-compatible object to a float64 (N, 3) array."""
    height, width = cloud.height, cloud.width
    point_step, row_step = cloud.point_step, cloud.row_step
    data = memoryview(cloud.data)
    if (
        min(height, width, row_step) < 0
        or (height == 0 and width != 0)
        or point_step <= 0
        or row_step < width * point_step
        or not data.c_contiguous
        or data.nbytes != height * row_step
    ):
        raise ValueError("invalid PointCloud2 dimensions, strides, or data length")

    byte_order = ">" if cloud.is_bigendian else "<"
    formats = {7: byte_order + "f4", 8: byte_order + "f8"}  # PointField FLOAT32/64
    xyz = {}
    for field in cloud.fields:
        if field.name not in ("x", "y", "z"):
            continue
        dtype = formats.get(field.datatype)
        if dtype is None or field.count != 1 or field.name in xyz:
            raise ValueError(f"invalid PointCloud2 field {field.name}")
        dtype = np.dtype(dtype)
        if field.offset < 0 or field.offset + dtype.itemsize > point_step:
            raise ValueError(f"PointCloud2 field {field.name} exceeds point_step")
        xyz[field.name] = (field.offset, dtype)

    if set(xyz) != {"x", "y", "z"}:
        raise ValueError("PointCloud2 requires x, y and z fields")
    intervals = sorted((offset, offset + dtype.itemsize) for offset, dtype in xyz.values())
    if intervals[0][1] > intervals[1][0] or intervals[1][1] > intervals[2][0]:
        raise ValueError("PointCloud2 coordinate fields overlap")

    points = np.empty((height, width, 3), dtype=np.float64)
    if height and width:
        for column, name in enumerate(("x", "y", "z")):
            offset, dtype = xyz[name]
            points[:, :, column] = np.ndarray(
                shape=(height, width), dtype=dtype, buffer=data,
                offset=offset, strides=(row_step, point_step),
            )
    return points.reshape((-1, 3))


def distances_from(points, origin):
    with np.errstate(over="ignore"):
        x, y, z = (points - origin).T
        return np.hypot(np.hypot(x, y), z)


@dataclass(frozen=True)
class SampledFrame:
    finite_points: np.ndarray
    in_range_points: np.ndarray
    selected_points: np.ndarray
    counts: dict
    selected_range_min_m: float | None
    selected_range_max_m: float | None


def sample_frame(points, origin, radius_m=3.0, retention_percent=70):
    """Keep floor(retention_percent * N / 100) of the N eligible returns."""
    points = np.asarray(points, dtype=np.float64)
    origin = np.asarray(origin, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError("points must have shape (N, 3)")
    if origin.shape != (3,) or not np.isfinite(origin).all():
        raise ValueError("LiDAR origin must be a finite XYZ vector")
    if not np.isfinite(radius_m) or radius_m <= 0:
        raise ValueError("radius_m must be finite and positive")
    if not isinstance(retention_percent, (int, np.integer)) or not 0 <= retention_percent <= 100:
        raise ValueError("retention_percent must be an integer between 0 and 100")

    finite = points[np.isfinite(points).all(axis=1)]
    distances = distances_from(finite, origin)
    in_range = distances <= radius_m
    eligible = finite[in_range]
    eligible_ranges = distances[in_range]
    count = len(eligible) * retention_percent // 100

    # Take the midpoint of each equal interval over the input order.
    if count:
        indices = ((2 * np.arange(count, dtype=np.int64) + 1) * len(eligible)) // (2 * count)
        selected_ranges = eligible_ranges[indices]
        minimum, maximum = float(selected_ranges.min()), float(selected_ranges.max())
    else:
        indices = np.empty(0, dtype=np.int64)
        minimum = maximum = None

    return SampledFrame(
        finite_points=finite,
        in_range_points=eligible,
        selected_points=eligible[indices],
        counts={
            "input": len(points),
            "nonfinite": len(points) - len(finite),
            "finite": len(finite),
            "outside_range": len(finite) - len(eligible),
            "in_range": len(eligible),
            "sampled": count,
        },
        selected_range_min_m=minimum,
        selected_range_max_m=maximum,
    )


class StaticVoxelMap:
    """Store the first observation in each voxel, up to a fixed capacity."""

    def __init__(self, voxel_size, max_voxels):
        self.voxel_size = voxel_size
        self.max_voxels = max_voxels
        self._occupied = set()
        self._points = np.empty((max_voxels, 3), dtype="<f4")

    def __len__(self):
        return len(self._occupied)

    def points(self):
        return self._points[:len(self)]

    def update(self, points):
        with np.errstate(over="ignore"):
            grid = np.floor(points / self.voxel_size)
        if not np.isfinite(grid).all() or np.any(np.abs(grid) >= 2**63):
            raise ValueError("map points exceed the voxel index range")
        keys, representatives = np.unique(grid.astype(np.int64), axis=0, return_index=True)
        before = len(self)
        rejected = 0
        # On a partially full map, give priority to the first point in scan order.
        for row in np.argsort(representatives):
            key = tuple(keys[row])
            if key in self._occupied:
                continue
            if len(self) >= self.max_voxels:
                rejected += 1
                continue
            self._points[len(self)] = points[representatives[row]]
            self._occupied.add(key)
        return {"admitted": len(self) - before, "dropped_at_capacity": rejected, "map_voxels": len(self)}
