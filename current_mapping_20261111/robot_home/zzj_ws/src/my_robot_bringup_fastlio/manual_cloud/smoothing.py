"""Conservative geometric cleanup for a world-frame LiDAR mapping cloud.

The weighted local tangent-plane fit is implemented with SciPy and NumPy,
following the neighborhood-fit-project approach of PCL MLS. The k-neighbor
distance statistic follows PCL StatisticalOutlierRemoval. Neither stage uses
PCL code. Outlier rejection also requires absolute isolation and consults the
original pre-sampling frame to avoid deleting points isolated by sampling.

https://pointclouds.org/documentation/classpcl_1_1_moving_least_squares.html
https://pointclouds.org/documentation/classpcl_1_1_statistical_outlier_removal.html

The fit only moves XY for well-supported, nearly vertical surfaces. Sparse
features and discontinuities usually lack that support. One scan cannot
distinguish an isolated true reflector from an isolated false return, or
correct registration error or grid quantization. Tune on recorded scans.
"""

from dataclasses import dataclass
from math import radians, sin

import numpy as np
from scipy.spatial import cKDTree


@dataclass(frozen=True)
class SmoothingOptions:
    smooth_enabled: bool = True
    smooth_radius: float = 0.12
    smooth_min_neighbors: int = 10
    smooth_max_neighbors: int = 32
    smooth_max_thickness_ratio: float = 0.10
    smooth_min_span_ratio: float = 0.10
    smooth_max_normal_z: float = sin(radians(15))
    smooth_min_side_span: float = 0.02
    smooth_max_shift: float = 0.015
    outlier_enabled: bool = True
    outlier_mean_k: int = 10
    outlier_stddev: float = 3.0
    outlier_support_radius: float = 0.10
    outlier_min_distance: float = 0.15

    def __post_init__(self):
        if not (3 <= self.smooth_min_neighbors <= self.smooth_max_neighbors):
            raise ValueError("require 3 <= smooth_min_neighbors <= smooth_max_neighbors")
        if not (0.0 < self.smooth_min_side_span < self.smooth_radius < np.inf):
            raise ValueError("require 0 < smooth_min_side_span < smooth_radius")
        if not (0.0 < self.smooth_max_shift < np.inf):
            raise ValueError("smooth_max_shift must be finite and positive")
        if not (0.0 < self.smooth_max_thickness_ratio <= 1.0
                and 0.0 < self.smooth_min_span_ratio <= 1.0
                and 0.0 <= self.smooth_max_normal_z < 1.0):
            raise ValueError("invalid smoothing plane thresholds")
        if self.outlier_mean_k < 1 or not (0.0 < self.outlier_stddev < np.inf):
            raise ValueError("invalid outlier statistics thresholds")
        if not (0.0 < self.outlier_support_radius < np.inf
                and 0.0 < self.outlier_min_distance < np.inf):
            raise ValueError("outlier distances must be finite and positive")


def _finite_xyz(points, name):
    points = np.asarray(points, dtype=np.float64)
    if points.ndim != 2 or points.shape[1] != 3:
        raise ValueError(f"{name} must have shape (N, 3)")
    return points, np.isfinite(points).all(axis=1)


def _remove_outliers(points, unsmoothed_points, support_points, options):
    """Reject high mean-distance points with no other pre-sampling return nearby."""
    k = options.outlier_mean_k
    if len(points) <= k or support_points is None or len(support_points) == 0:
        return points, 0, 0

    distances, _ = cKDTree(points).query(points, k=k + 1)
    mean_distance = distances[:, 1:].mean(axis=1)
    threshold = mean_distance.mean() + options.outlier_stddev * mean_distance.std(ddof=1)
    candidates = np.flatnonzero(
        (mean_distance > threshold) & (distances[:, 1] >= options.outlier_min_distance)
    )
    if len(candidates) == 0:
        return points, 0, 0

    raw_distances, _ = cKDTree(support_points).query(unsmoothed_points[candidates], k=2)
    # Disregard one coincident return (normally the sampled point itself).
    # Any second return, or a noncoincident nearest return, can protect it.
    other_distance = np.where(raw_distances[:, 0] <= 1e-6,
                              raw_distances[:, 1], raw_distances[:, 0])
    remove = candidates[other_distance > options.outlier_support_radius]
    keep = np.ones(len(points), dtype=bool)
    keep[remove] = False
    return points[keep], len(candidates), len(remove)


def _smooth_xy(points, options, crop_origin, crop_radius):
    """Fit a weighted local plane and project only well-supported points in XY."""
    if len(points) <= options.smooth_min_neighbors:
        return points, 0

    result = points.copy()
    radius = options.smooth_radius
    tree = cKDTree(points)
    k = min(len(points), options.smooth_max_neighbors + 1)
    smoothed_count = 0

    # Bound the temporary neighborhood arrays independently of the frame size.
    for start in range(0, len(points), 1024):
        rows = np.arange(start, min(start + 1024, len(points)))
        distances, neighbors = tree.query(points[rows], k=k, distance_upper_bound=radius)
        valid = (neighbors < len(points)) & (neighbors != rows[:, None])
        enough = valid.sum(axis=1) >= options.smooth_min_neighbors
        if not np.any(enough):
            continue

        rows = rows[enough]
        distances, neighbors, valid = distances[enough], neighbors[enough], valid[enough]
        offsets = points[np.where(valid, neighbors, rows[:, None])] - points[rows, None, :]
        weights = np.where(valid, np.exp(-np.square(distances / radius)), 0.0)
        weights /= weights.sum(axis=1, keepdims=True)
        centroid = np.einsum("nk,nki->ni", weights, offsets)
        centered = offsets - centroid[:, None, :]
        covariance = centered.transpose(0, 2, 1) @ (centered * weights[:, :, None])
        eigenvalues, eigenvectors = np.linalg.eigh(covariance)
        normals = eigenvectors[:, :, 0]

        planar = (
            (eigenvalues[:, 1] > 0.0)
            & (eigenvalues[:, 0] <= options.smooth_max_thickness_ratio * eigenvalues[:, 1])
            & (eigenvalues[:, 1] >= options.smooth_min_span_ratio * eigenvalues[:, 2])
            & (np.abs(normals[:, 2]) <= options.smooth_max_normal_z)
        )
        if not np.any(planar):
            continue

        rows, valid, offsets = rows[planar], valid[planar], offsets[planar]
        normals, centroid = normals[planar], centroid[planar]
        normal_xy = normals[:, :2]
        normal_xy_squared = np.einsum("ni,ni->n", normal_xy, normal_xy)
        tangent_xy = np.column_stack((-normal_xy[:, 1], normal_xy[:, 0]))
        tangent_xy /= np.sqrt(normal_xy_squared)[:, None]
        tangent_distances = np.einsum("nki,ni->nk", offsets[:, :, :2], tangent_xy)
        # Points on a wall edge or at a gap should not be shifted by one-sided support.
        span = options.smooth_min_side_span
        left = (valid & (tangent_distances <= -span)).sum(axis=1)
        right = (valid & (tangent_distances >= span)).sum(axis=1)
        scale = np.einsum("ni,ni->n", centroid, normals) / normal_xy_squared
        correction = normal_xy * scale[:, None]
        shift = np.linalg.norm(correction, axis=1)
        accepted = (left >= 2) & (right >= 2) & (shift > 1e-9) & (shift <= options.smooth_max_shift)
        if crop_origin is not None:
            proposed = points[rows]
            proposed[:, :2] += correction
            x, y, z = (proposed - crop_origin).T
            accepted &= np.hypot(np.hypot(x, y), z) <= crop_radius
        result[rows[accepted], :2] += correction[accepted]
        smoothed_count += int(np.count_nonzero(accepted))

    return result, smoothed_count


def smooth_and_filter(points, support_points, options=None, *, crop_origin=None, crop_radius=None):
    """Return finite float64 Nx3 points in input order and stage counts.

    Both clouds are in the same world frame. ``support_points`` holds the
    original pre-sampling returns. Pass ``None`` to run without outlier removal.
    Input arrays are unmodified. Smoothing retains each surviving point and Z.
    Supply both crop arguments to preserve already cropped points when a proposed
    correction would put them outside the input range.
    """
    options = SmoothingOptions() if options is None else options
    points, finite = _finite_xyz(points, "points")
    input_count = len(points)
    unsmoothed_points = points[finite]
    finite_count = len(unsmoothed_points)
    result = unsmoothed_points
    candidates = removed = smoothed = 0

    if crop_origin is not None or crop_radius is not None:
        if crop_origin is None or crop_radius is None:
            raise ValueError("crop_origin and crop_radius must be supplied together")
        crop_origin = np.asarray(crop_origin, dtype=np.float64)
        crop_radius = float(crop_radius)
        if crop_origin.shape != (3,) or not np.isfinite(crop_origin).all():
            raise ValueError("crop_origin must be a finite XYZ vector")
        if not 0.0 < crop_radius < np.inf:
            raise ValueError("crop_radius must be finite and positive")
    if options.smooth_enabled:
        result, smoothed = _smooth_xy(result, options, crop_origin, crop_radius)
    if options.outlier_enabled and support_points is not None:
        support_points, finite_support = _finite_xyz(support_points, "support_points")
        result, candidates, removed = _remove_outliers(
            result, unsmoothed_points, support_points[finite_support], options
        )

    return result, {
        "input_count": input_count,
        "finite_count": finite_count,
        "nonfinite_count": input_count - finite_count,
        "smoothed_count": smoothed,
        "outlier_candidates": candidates,
        "outlier_vetoed": candidates - removed,
        "outlier_removed": removed,
        "output_count": len(result),
    }
