"""Geometry only. This module never accumulates point clouds or 2D history."""
import numpy as np


def rotation_matrix(q):
    q = np.asarray(q, dtype=np.float64)
    if not np.isfinite(q).all() or np.linalg.norm(q) < 1e-9:
        raise ValueError('Invalid reference quaternion')
    x, y, z, w = q / np.linalg.norm(q)
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
        [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)],
    ])


def project_snapshot(xyz, ground, rotation, low=0.10, high=1.00,
                     resolution=0.05, padding=0.50, max_cells=10_000_000):
    if not (np.isfinite([low, high, resolution, padding]).all() and
            low < high and resolution > 0 and padding >= 0):
        raise ValueError('Invalid projection settings')
    xyz = np.asarray(xyz, dtype=np.float64)
    xyz = xyz[np.isfinite(xyz).all(axis=1)]
    # Reference basis columns are the ground-frame axes in camera_init.
    local = (xyz - np.asarray(ground)) @ rotation
    if len(local) == 0:
        raise ValueError('Native snapshot contains no finite points')
    bounds_min = np.floor((local[:, :2].min(axis=0) - padding) / resolution).astype(np.int64)
    bounds_max = np.floor((local[:, :2].max(axis=0) + padding) / resolution).astype(np.int64)
    width, height = (int(v) for v in bounds_max - bounds_min + 1)
    if width < 1 or height < 1 or width * height > max_cells:
        raise ValueError(f'Grid {width}x{height} exceeds {max_cells} cells; snapshot rejected')
    # The tiny tolerance covers float32 serialization at the height boundary.
    keep = (local[:, 2] >= low - 1e-6) & (local[:, 2] <= high + 1e-6)
    selected = local[keep]
    grid = np.full((height, width), -1, dtype=np.int8)
    if len(selected):
        indices = np.floor(selected[:, :2] / resolution).astype(np.int64) - bounds_min
        grid[indices[:, 1], indices[:, 0]] = 100
    stats = dict(native_points=len(local), selected_points=len(selected),
                 below_points=int(np.count_nonzero(local[:, 2] < low - 1e-6)),
                 above_points=int(np.count_nonzero(local[:, 2] > high + 1e-6)),
                 occupied_cells=int(np.count_nonzero(grid == 100)),
                 free_cells=0, unknown_cells=int(np.count_nonzero(grid == -1)),
                 width=width, height=height,
                 selected_min_height=float(selected[:, 2].min()) if len(selected) else None,
                 selected_max_height=float(selected[:, 2].max()) if len(selected) else None)
    return grid, bounds_min * resolution, stats
