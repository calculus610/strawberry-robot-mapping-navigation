"""Conservative, pose-aligned voxel accumulation for the manual map."""

from dataclasses import dataclass
from math import ceil, isfinite, pi, sqrt

import numpy as np


_KEY_BITS = 21
_KEY_BIAS = 1 << (_KEY_BITS - 1)
_KEY_STRIDES = np.array([1 << (2 * _KEY_BITS), 1 << _KEY_BITS, 1], dtype=np.int64)
_ANGULAR_STEP = 0.01  # rad; only used to make clearing more conservative
_TIME_EPSILON = 1e-6  # accommodates ROS epoch stamps represented as float seconds
_WORLD_RANGE_EPSILON = 1e-9  # rigid transforms can round an in-range point outward


@dataclass(frozen=True)
class TemporalOptions:
    dynamic: bool = True
    voxel_size: float = 0.05
    max_range: float = 3.0
    endpoint_margin: float = 0.15
    hit_frames: int = 3
    hit_seconds: float = 0.2
    free_frames: int = 4
    free_seconds: float = 0.3
    candidate_gap: float = 0.5
    max_voxels: int = 250_000
    max_frame_points: int = 40_000
    min_clear_range: float = 0.5
    max_clear_rays: int = 2048  # zero leaves ray traversal uncapped

    def __post_init__(self):
        positive = (self.voxel_size, self.max_range, self.endpoint_margin, self.candidate_gap)
        nonnegative = (self.hit_seconds, self.free_seconds, self.min_clear_range)
        if not all(isfinite(value) and value > 0 for value in positive):
            raise ValueError("voxel size, range, endpoint margin and candidate gap must be positive")
        if not all(isfinite(value) and value >= 0 for value in nonnegative):
            raise ValueError("minimum range and time thresholds must be nonnegative")
        if self.max_range / self.voxel_size > 128 or self.min_clear_range >= self.max_range:
            raise ValueError("ray range must be greater than the minimum and at most 128 voxels")
        counts = (self.hit_frames, self.free_frames, self.max_voxels, self.max_frame_points)
        if not all(isinstance(value, int) and not isinstance(value, bool) and value > 0 for value in counts):
            raise ValueError("frame counts and capacity limits must be positive integers")
        if (not isinstance(self.max_clear_rays, int) or isinstance(self.max_clear_rays, bool)
                or self.max_clear_rays < 0):
            raise ValueError("max_clear_rays must be a nonnegative integer")


@dataclass(slots=True)
class _Evidence:
    first: float
    last: float
    count: int

    def add(self, stamp, gap):
        if stamp - self.last > gap + _TIME_EPSILON:
            self.first = stamp
            self.count = 0
        self.last = stamp
        self.count += 1

    def meets(self, frames, seconds):
        return self.count >= frames and self.last - self.first + _TIME_EPSILON >= seconds


def _as_points(points):
    array = np.asarray(points, dtype=np.float64)
    if array.shape == (0,):
        return array.reshape(0, 3)
    if array.ndim != 2 or array.shape[1] != 3:
        raise ValueError("point clouds must have shape (N, 3)")
    return array


def _pack(cells):
    return (cells + _KEY_BIAS) @ _KEY_STRIDES


def _contains(sorted_keys, query):
    if not sorted_keys.size:
        return np.zeros(query.shape, dtype=bool)
    indices = np.searchsorted(sorted_keys, query)
    return (indices < sorted_keys.size) & (sorted_keys[np.minimum(indices, sorted_keys.size - 1)] == query)


class TemporalMap:
    """Map points from filtered endpoints and clear them with observed free space.

    Inputs share a world frame. ``raw_world`` must contain actual, preferably
    deskewed, first returns before thinning or coordinate smoothing. Rays stop
    before the nearest return, so unobserved and occluded voxels do not supply
    free evidence. ``stamp`` is a strictly increasing time in seconds. A time
    reset must be declared with ``reset_evidence()`` before the next frame.
    """

    def __init__(self, options=None):
        if options is None or isinstance(options, dict):
            options = TemporalOptions(**(options or {}))
        if not isinstance(options, TemporalOptions):
            raise TypeError("options must be TemporalOptions or a dict")
        self.options = options
        self._occupied = {}
        self._pending = {}
        self._free = {}
        self._last_stamp = None
        self._capacity_drops = 0
        self._ray_phase = 0
        self._neighbor_offsets = np.array(
            [(x, y, z) for x in (-1, 0, 1) for y in (-1, 0, 1) for z in (-1, 0, 1)],
            dtype=np.int64,
        ) @ _KEY_STRIDES

    def __len__(self):
        return len(self._occupied)

    def reset_evidence(self):
        """Start a new timestamp epoch without changing the confirmed map."""
        self._pending.clear()
        self._free.clear()
        self._last_stamp = None
        self._ray_phase = 0

    def points(self):
        return np.asarray([self._occupied[key] for key in sorted(self._occupied)], dtype=np.float32).reshape(-1, 3)

    def _metrics(self):
        return dict(status="ok", input_points=0, raw_points=0, hit_voxels=0, raw_hit_voxels=0,
                    available_clear_rays=0, clear_rays=0, budget_skipped=0,
                    free_voxels=0, admitted=0, removed=0, expired=0,
                    dropped_at_capacity=0, capacity_drops_total=self._capacity_drops,
                    pending_voxels=len(self._pending), map_voxels=len(self._occupied))

    def _valid_points(self, points, origin):
        points = points[np.isfinite(points).all(axis=1)]
        # Keep a spare voxel at each edge for the endpoint protection halo.
        world_limit = (_KEY_BIAS - 2) * self.options.voxel_size
        points = points[(np.abs(points) < world_limit).all(axis=1)]
        distances = np.linalg.norm(points - origin, axis=1)
        return points, distances

    def _keys(self, points):
        return _pack(np.floor(points / self.options.voxel_size).astype(np.int64))

    def _ray_directions(self, offsets, distances):
        if not distances.size:
            return np.empty((0, 3)), np.empty(0)

        azimuth_bins = ceil(2 * pi / _ANGULAR_STEP)
        elevation_bins = ceil(pi / _ANGULAR_STEP)
        azimuth = np.arctan2(offsets[:, 1], offsets[:, 0])
        elevation = np.arctan2(offsets[:, 2], np.hypot(offsets[:, 0], offsets[:, 1]))
        az = np.floor((azimuth + pi) * azimuth_bins / (2 * pi)).astype(np.int64) % azimuth_bins
        el = np.minimum(np.floor((elevation + pi / 2) * elevation_bins / pi).astype(np.int64), elevation_bins - 1)
        angular_keys = el * azimuth_bins + az

        # Pick one actual nearest ray per small angular cell. A nearby closer
        # return may shorten the ray, including across the -pi/+pi seam.
        order = np.lexsort((offsets[:, 2], offsets[:, 1], offsets[:, 0], distances, angular_keys))
        sorted_keys = angular_keys[order]
        nearest = order[np.r_[True, sorted_keys[1:] != sorted_keys[:-1]]]
        lookup = np.full(azimuth_bins * elevation_bins, np.inf)
        lookup[angular_keys[nearest]] = distances[nearest]
        near_az = az[nearest]
        near_el = el[nearest]
        bound = distances[nearest].copy()
        for dy in (-1, 0, 1):
            for dx in (-1, 0, 1):
                neighbor = np.clip(near_el + dy, 0, elevation_bins - 1) * azimuth_bins + (near_az + dx) % azimuth_bins
                np.minimum(bound, lookup[neighbor], out=bound)

        length = np.minimum(self.options.max_range, bound - self.options.endpoint_margin)
        usable = length > self.options.min_clear_range
        indices = nearest[usable]
        return offsets[indices] / distances[indices, None], length[usable]

    def _free_keys(self, origin, directions, limits, hit_cells):
        if not limits.size:
            return np.empty(0, dtype=np.int64)

        # Batched Amanatides-Woo traversal. Only record fully traversed cells,
        # skipping the origin cell and contacts that barely graze a voxel.
        voxel = self.options.voxel_size
        cell = np.floor(origin / voxel).astype(np.int64)
        radius = ceil(self.options.max_range / voxel) + 1
        diameter = 2 * radius + 1
        strides = np.array([diameter * diameter, diameter, 1], dtype=np.int64)
        # A bounded scratch grid deduplicates rays without storing or sorting
        # a key for every crossing. The persistent map stays sparse.
        unseen, free, hit = 0, 1, 2
        seen = np.full(diameter ** 3, unseen, dtype=np.uint8)
        local_hits = hit_cells - cell + radius
        inside = ((local_hits >= 0) & (local_hits < diameter)).all(axis=1)
        seen[local_hits[inside] @ strides] = hit
        center = radius * strides.sum()
        if seen[center] == hit:
            return np.empty(0, dtype=np.int64)
        steps = np.sign(directions).astype(np.int64)
        absolute = np.abs(directions)
        delta = np.divide(voxel, absolute, out=np.full_like(directions, np.inf), where=absolute > 0)
        border = (cell + (steps > 0)) * voxel
        to_border = np.divide(np.abs(border - origin), absolute,
                              out=np.full_like(directions, np.inf), where=absolute > 0)
        packed = np.full(limits.size, center, dtype=np.int64)
        key_step = steps * strides
        while limits.size:
            rows = np.arange(limits.size)
            axes = np.argmin(to_border, axis=1)
            entry = to_border[rows, axes]
            packed += key_step[rows, axes]
            to_border[rows, axes] += delta[rows, axes]
            exit_distance = to_border.min(axis=1)
            # A farther ray crossing a voxel hit elsewhere in this frame
            # must not be used as evidence for the occluded region behind it.
            blocked = seen[packed] == hit
            full = (entry >= self.options.min_clear_range) & (exit_distance <= limits)
            full &= (exit_distance - entry >= voxel * 0.5) & ~blocked
            seen[packed[full]] = free
            active = (exit_distance < limits) & ~blocked
            if active.all():
                continue
            limits = limits[active]
            to_border = to_border[active]
            delta = delta[active]
            packed = packed[active]
            key_step = key_step[active]
        local = np.flatnonzero(seen == free)
        x, yz = np.divmod(local, strides[0])
        y, z = np.divmod(yz, diameter)
        return _pack(np.column_stack((x, y, z)) + cell - radius)

    def _apply_free(self, keys, protected, stamp, metrics):
        options = self.options
        for key in keys[~protected].tolist():
            if key in self._pending:
                del self._pending[key]
            if key not in self._occupied:
                continue
            evidence = self._free.setdefault(key, _Evidence(stamp, stamp, 0))
            evidence.add(stamp, options.candidate_gap)
            if evidence.meets(options.free_frames, options.free_seconds):
                del self._occupied[key]
                del self._free[key]
                metrics["removed"] += 1

    def _apply_hits(self, keys, points, stamp, metrics):
        options = self.options
        for key, point in zip(keys.tolist(), points):
            self._free.pop(key, None)
            if key in self._occupied:
                continue
            if key not in self._pending and len(self._occupied) + len(self._pending) >= options.max_voxels:
                metrics["dropped_at_capacity"] += 1
                continue
            if options.dynamic:
                evidence = self._pending.setdefault(key, _Evidence(stamp, stamp, 0))
                evidence.add(stamp, options.candidate_gap)
                if not evidence.meets(options.hit_frames, options.hit_seconds):
                    continue
                del self._pending[key]
            self._occupied[key] = tuple(point.tolist())
            metrics["admitted"] += 1

    def update(self, points_world, raw_world, sensor_origin_world, stamp, allow_clear=True):
        metrics = self._metrics()
        try:
            stamp = float(stamp)
        except (TypeError, ValueError, OverflowError):
            stamp = np.nan
        if not isfinite(stamp):
            metrics["status"] = "invalid_stamp"
            return metrics
        if self._last_stamp is not None and stamp <= self._last_stamp:
            metrics["status"] = "nonincreasing_stamp"
            return metrics

        try:
            points = _as_points(points_world)
            raw = _as_points(raw_world)
        except (TypeError, ValueError, OverflowError):
            metrics["status"] = "invalid_cloud"
            return metrics
        try:
            origin = np.asarray(sensor_origin_world, dtype=np.float64)
        except (TypeError, ValueError, OverflowError):
            metrics["status"] = "invalid_origin"
            return metrics
        options = self.options
        # Leave room for ray traversal and the one-voxel protection halo.
        world_limit = (_KEY_BIAS - 2) * options.voxel_size - options.max_range
        if origin.shape != (3,) or not np.isfinite(origin).all() or np.any(np.abs(origin) >= world_limit):
            metrics["status"] = "invalid_origin"
            return metrics
        if max(len(points), len(raw)) > options.max_frame_points:
            metrics["status"] = "frame_capacity"
            return metrics

        self._last_stamp = stamp
        points, distances = self._valid_points(points, origin)
        points = points[(distances > 0) & (distances <= options.max_range + _WORLD_RANGE_EPSILON)]
        metrics["input_points"] = len(points)
        hit_keys = self._keys(points)
        order = np.lexsort((points[:, 2], points[:, 1], points[:, 0], hit_keys))
        hit_keys = hit_keys[order]
        unique = np.r_[True, hit_keys[1:] != hit_keys[:-1]][:len(hit_keys)]
        hit_keys, points = hit_keys[unique], points[order[unique]]
        metrics["hit_voxels"] = len(hit_keys)

        if options.dynamic:
            expired = [key for key, evidence in self._pending.items()
                       if stamp - evidence.last > options.candidate_gap + _TIME_EPSILON]
            for key in expired:
                del self._pending[key]
            metrics["expired"] = len(expired)

            raw, distances = self._valid_points(raw, origin)
            valid = distances > 0
            raw, distances = raw[valid], distances[valid]
            metrics["raw_points"] = len(raw)
            near = distances <= options.max_range + 2 * sqrt(3) * options.voxel_size
            hit_cells = np.floor(raw[near] / options.voxel_size).astype(np.int64)
            raw_hits = np.unique(_pack(hit_cells))
            metrics["raw_hit_voxels"] = len(raw_hits)

            if self._free:
                active = np.fromiter(self._free, dtype=np.int64)
                protected = _contains(raw_hits, active[:, None] + self._neighbor_offsets).any(axis=1)
                for key in active[protected].tolist():
                    del self._free[key]

            if allow_clear and (self._occupied or self._pending):
                directions, limits = self._ray_directions(raw - origin, distances)
                available = len(limits)
                metrics["available_clear_rays"] = available
                budget = options.max_clear_rays
                if 0 < budget < available:
                    # Spread the budget across sorted angular cells, then
                    # advance one slot per over-budget frame. For a fixed
                    # ray list, ceil(available / budget) frames cover it.
                    selected = (np.arange(budget) * available // budget + self._ray_phase) % available
                    self._ray_phase = (self._ray_phase + 1) % available
                    directions, limits = directions[selected], limits[selected]
                metrics["clear_rays"] = len(limits)
                metrics["budget_skipped"] = available - len(limits)
                free_keys = self._free_keys(origin, directions, limits, hit_cells)
                # Intersect the ray result with persistent state before paying
                # for the 27-neighbor endpoint protection test.
                existing = np.fromiter((key for key in free_keys.tolist()
                                        if key in self._occupied or key in self._pending), dtype=np.int64)
                protected = _contains(hit_keys, existing)
                protected |= _contains(raw_hits, existing[:, None] + self._neighbor_offsets).any(axis=1)
                metrics["free_voxels"] = int(np.count_nonzero(~protected))
                self._apply_free(existing, protected, stamp, metrics)

        self._apply_hits(hit_keys, points, stamp, metrics)
        self._capacity_drops += metrics["dropped_at_capacity"]
        metrics["capacity_drops_total"] = self._capacity_drops
        metrics["pending_voxels"] = len(self._pending)
        metrics["map_voxels"] = len(self._occupied)
        return metrics
