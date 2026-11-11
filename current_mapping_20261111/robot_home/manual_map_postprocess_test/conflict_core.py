"""Bounded endpoint evidence and three-state decisions for the offline experiment."""

import math
import operator

import numpy as np


FREE = 254
UNKNOWN = 205
OCCUPIED = 0
CLUSTERS_PER_CELL = 4
CELL_CLUSTER_OVERFLOW = 1
CLUSTER_NEAR_GROUND = 1
CLUSTER_HEIGHT_BOUNDARY = 2
CLUSTER_HEIGHT_ASSIGNMENT = 4
CLUSTER_STATION_BOUNDARY = 8

DEFAULT_POSITIVE_PARAMS = {
    "height_span_m": 0.04,
    "station_min_frames": 3,
    "station_min_distance_m": 0.30,
    "station_min_bearing_deg": 20.0,
    "station_min_gap_ns": 1_000_000_000,
    "station_radius_m": 0.10,
    "station_bearing_radius_deg": 5.0,
    "low_height_m": 0.20,
    "station_distance_margin_m": None,
    "station_bearing_margin_deg": None,
    "station_time_margin_ns": None,
}

UINT32_MAX = np.iinfo(np.uint32).max
UINT64_MAX = np.iinfo(np.uint64).max
INT64_MAX = np.iinfo(np.int64).max
FLOAT32_MAX = np.finfo(np.float32).max
ROUNDING_FACTOR = 8 * np.finfo(np.float64).eps
STATION_LENGTH_ROUNDING = ROUNDING_FACTOR * 2
STATION_ANGLE_ROUNDING = ROUNDING_FACTOR * 360


def _integer(value, name, lower, upper):
    if isinstance(value, (bool, np.bool_)):
        raise ValueError(f"{name} must be an integer")
    try:
        result = operator.index(value)
    except TypeError as exc:
        raise ValueError(f"{name} must be an integer") from exc
    if not lower <= result <= upper:
        raise ValueError(f"{name} outside [{lower}, {upper}]")
    return result


def _positive_parameters(params):
    supplied = {} if params is None else dict(params)
    unsupported = supplied.keys() - DEFAULT_POSITIVE_PARAMS.keys()
    if unsupported:
        raise ValueError(f"unsupported positive parameters: {sorted(unsupported)}")
    result = DEFAULT_POSITIVE_PARAMS | supplied
    result["station_min_frames"] = _integer(
        result["station_min_frames"], "station_min_frames", 3, UINT32_MAX
    )
    result["station_min_gap_ns"] = _integer(
        result["station_min_gap_ns"], "station_min_gap_ns", 1, INT64_MAX
    )
    if result["station_time_margin_ns"] is not None:
        result["station_time_margin_ns"] = _integer(
            result["station_time_margin_ns"], "station_time_margin_ns", 0, INT64_MAX
        )

    for name in (
        "height_span_m", "station_min_distance_m", "station_min_bearing_deg",
        "station_radius_m", "station_bearing_radius_deg", "low_height_m",
        "station_distance_margin_m", "station_bearing_margin_deg",
    ):
        value = result[name]
        if name.endswith("_margin_m") or name.endswith("_margin_deg"):
            if value is None:
                continue
            lower = 0.0
        else:
            lower = np.finfo(np.float64).tiny
        upper = 180.0 if name.endswith("_deg") else FLOAT32_MAX
        if isinstance(value, (bool, np.bool_)) or not isinstance(
            value, (int, float, np.integer, np.floating)
        ) or not lower <= value <= upper:
            raise ValueError(f"{name} outside [{lower}, {upper}]")
        result[name] = float(value)
    return result


def _bearing_difference_degrees(dx, dy, ax, ay):
    delta = math.atan2(dy, dx) - math.atan2(ay, ax)
    return math.degrees(abs(math.remainder(delta, math.tau)))


def _upper_float32(value):
    rounded = np.float32(value)
    if float(rounded) < value:
        return np.nextafter(rounded, np.float32(np.inf))
    return rounded


class PositiveAccumulator:
    """Keep four greedy height clusters and one ordered station pair per cell.

    Input frames are ordered by timestamp, frame ID and source offset. Points
    are processed in raw point order within each cell. A station uses its first
    admissible hit as anchor, and station A closes when station B opens. This
    retains a sufficient geometric witness and can miss alternative pairs.
    """

    def __init__(self, cell_xy, params=None):
        self.cell_xy = np.array(cell_xy, dtype=np.float64, copy=True)
        if (
            self.cell_xy.ndim != 2 or self.cell_xy.shape[1] != 2
            or not np.isfinite(self.cell_xy).all() or np.any(np.abs(self.cell_xy) > FLOAT32_MAX)
        ):
            raise ValueError("cell_xy must contain bounded, finite [N,2] cell centers")
        self.params = _positive_parameters(params)
        distance_margin = self.params["station_distance_margin_m"]
        bearing_margin = self.params["station_bearing_margin_deg"]
        time_margin = self.params["station_time_margin_ns"]
        self._has_margins = None not in (distance_margin, bearing_margin, time_margin)
        self._station_distance = self.params["station_min_distance_m"]
        self._station_bearing = self.params["station_min_bearing_deg"]
        self._station_gap_ns = self.params["station_min_gap_ns"]
        if self._has_margins:
            self._station_distance += distance_margin
            self._station_bearing += bearing_margin
            self._station_gap_ns += time_margin

        n_cells = len(self.cell_xy)
        shape = (n_cells, CLUSTERS_PER_CELL)
        self.xyz_min = np.full((*shape, 3), np.nan, dtype=np.float64)
        self.xyz_max = np.full((*shape, 3), np.nan, dtype=np.float64)
        self.representatives = np.full((*shape, 4, 3), np.nan, dtype=np.float32)
        self.representative_offsets = np.full((*shape, 4), UINT64_MAX, dtype=np.uint64)
        self.representative_indices = np.full((*shape, 4), UINT32_MAX, dtype=np.uint32)
        self.cluster_support = np.zeros(shape, dtype=np.uint32)
        self.cell_support = np.zeros(n_cells, dtype=np.uint32)
        self.first_stamp_ns = np.zeros(shape, dtype=np.int64)
        self.last_stamp_ns = np.zeros(shape, dtype=np.int64)
        self.station_counts = np.zeros((*shape, 2), dtype=np.uint32)
        self.station_anchor_xy = np.full((*shape, 2, 2), np.nan, dtype=np.float64)
        self.cluster_flags = np.zeros(shape, dtype=np.uint8)
        self.cell_flags = np.zeros(n_cells, dtype=np.uint8)

        # Station A's maximum distance and bearing from its first view, rounded
        # upward so subtracting these bounds from separation stays conservative.
        self._station_a_extent = np.zeros((*shape, 2), dtype=np.float32)
        self._station_a_last_ns = np.zeros(shape, dtype=np.int64)
        self._last_stamp = 0
        self._last_id = -1
        self._last_offset = -1
        self._failed = False
        self._result = None

    @property
    def payload_bytes(self):
        """Bytes of resident NumPy payloads, excluding the current frame workspace."""
        return sum(
            array.nbytes for array in vars(self).values() if isinstance(array, np.ndarray)
        )

    def _increment(self, counts, index, kind):
        if counts[index] == UINT32_MAX:
            self._failed = True
            raise OverflowError(f"{kind} exceeds uint32 capacity at {index}")
        counts[index] += 1

    def _select_cluster(self, cell, z):
        height_limit = self.params["height_span_m"]
        matches = []
        first_empty = None
        near_limit = False
        for cluster in range(CLUSTERS_PER_CELL):
            if self.cluster_support[cell, cluster] == 0:
                first_empty = cluster
                break
            z_low = float(self.xyz_min[cell, cluster, 2])
            z_high = float(self.xyz_max[cell, cluster, 2])
            span = max(z_high, z) - min(z_low, z)
            rounding = max(abs(z), abs(z_low), abs(z_high), height_limit) * ROUNDING_FACTOR
            if abs(span - height_limit) <= rounding:
                self.cluster_flags[cell, cluster] |= CLUSTER_HEIGHT_BOUNDARY
                near_limit = True
            if span <= height_limit:
                matches.append((span, cluster))
        if len(matches) > 1:
            for _, cluster in matches:
                self.cluster_flags[cell, cluster] |= CLUSTER_HEIGHT_ASSIGNMENT
        if matches:
            _, selected = min(matches)
            if near_limit:
                self.cluster_flags[cell, selected] |= CLUSTER_HEIGHT_BOUNDARY
            return selected
        if first_empty is None:
            self.cell_flags[cell] |= CELL_CLUSTER_OVERFLOW
            return None
        if near_limit:
            self.cluster_flags[cell, first_empty] |= CLUSTER_HEIGHT_BOUNDARY
        return first_empty

    def _observe_station(self, cell, cluster, point, raw_index, sensor_xy, stamp, offset):
        center = self.cell_xy[cell]
        dx, dy = sensor_xy - center
        if dx == 0 and dy == 0:
            # This hit supports the cluster but has undefined planar bearing.
            return
        counts = self.station_counts[cell, cluster]
        group = 0
        anchor = self.station_anchor_xy[cell, cluster, 0]
        if counts[0]:
            ax, ay = anchor - center
            distance_a = math.hypot(*(sensor_xy - anchor))
            bearing_a = _bearing_difference_degrees(dx, dy, ax, ay)
            if counts[1] or not self._inside_station(distance_a, bearing_a):
                if counts[0] < self.params["station_min_frames"]:
                    return
                time_gap = stamp - int(self._station_a_last_ns[cell, cluster])
                if time_gap < self._station_gap_ns:
                    return
                extent_a = self._station_a_extent[cell, cluster]
                distance_gap = distance_a - float(extent_a[0])
                bearing_gap = bearing_a - float(extent_a[1])
                if (
                    abs(distance_gap - self._station_distance) <= STATION_LENGTH_ROUNDING
                    or abs(bearing_gap - self._station_bearing) <= STATION_ANGLE_ROUNDING
                ) and (
                    distance_gap >= self._station_distance - STATION_LENGTH_ROUNDING
                    and bearing_gap >= self._station_bearing - STATION_ANGLE_ROUNDING
                ):
                    self.cluster_flags[cell, cluster] |= CLUSTER_STATION_BOUNDARY
                if distance_gap < self._station_distance or bearing_gap < self._station_bearing:
                    return
                group = 1
                anchor = self.station_anchor_xy[cell, cluster, 1]
                if counts[1]:
                    bx, by = anchor - center
                    distance_b = math.hypot(*(sensor_xy - anchor))
                    bearing_b = _bearing_difference_degrees(dx, dy, bx, by)
                    if not self._inside_station(distance_b, bearing_b):
                        return
            else:
                extent_a = self._station_a_extent[cell, cluster]
                extent_a[0] = max(extent_a[0], _upper_float32(distance_a))
                extent_a[1] = max(extent_a[1], _upper_float32(bearing_a))

        first = counts[group] == 0
        if first:
            anchor[:] = sensor_xy
        if group == 0:
            self._station_a_last_ns[cell, cluster] = stamp

        self._increment(counts, group, "station support")
        # Each group's first and last references may refer to the same hit
        # when only one member exists. The XYZ is a float32 cache of that hit.
        first_slot = 2 * group
        slots = slice(first_slot if first else first_slot + 1, first_slot + 2)
        self.representatives[cell, cluster, slots] = point
        self.representative_offsets[cell, cluster, slots] = offset
        self.representative_indices[cell, cluster, slots] = raw_index

    def _inside_station(self, distance, bearing):
        return (
            distance <= self.params["station_radius_m"]
            and bearing <= self.params["station_bearing_radius_deg"]
        )

    def update_frame(self, cell_ids, points, raw_indices, sensor_xyz, stamp_ns, frame_id, frame_offset):
        """Accumulate a frame of already selected endpoints in fixed ground XYZ."""
        if self._failed or self._result is not None:
            raise RuntimeError("positive accumulator is closed")
        stamp = _integer(stamp_ns, "stamp_ns", 1, INT64_MAX)
        frame_id = _integer(frame_id, "frame_id", 0, UINT64_MAX)
        offset = _integer(frame_offset, "frame_offset", 0, UINT64_MAX - 1)
        if stamp <= self._last_stamp or frame_id <= self._last_id or offset <= self._last_offset:
            raise ValueError("positive frame timestamps, IDs and offsets must increase")

        cell_ids = np.asarray(cell_ids)
        raw_indices = np.asarray(raw_indices)
        points = np.asarray(points, dtype=np.float64)
        sensor_xyz = np.asarray(sensor_xyz, dtype=np.float64)
        if (
            cell_ids.ndim != 1 or cell_ids.dtype.kind not in "iu"
            or raw_indices.shape != cell_ids.shape or raw_indices.dtype.kind not in "iu"
            or points.shape != (len(cell_ids), 3)
            or sensor_xyz.shape != (3,) or not np.isfinite(sensor_xyz).all()
            or np.any(np.abs(sensor_xyz) > FLOAT32_MAX)
        ):
            raise ValueError("invalid positive frame array shapes, types or sensor coordinates")
        if (
            np.any(cell_ids < 0) or np.any(cell_ids >= len(self.cell_xy))
            or np.any(raw_indices < 0) or np.any(raw_indices >= UINT32_MAX)
            or not np.isfinite(points).all() or np.any(np.abs(points) > FLOAT32_MAX)
        ):
            raise ValueError("positive frame contains out-of-range point or index")
        self._last_stamp, self._last_id, self._last_offset = stamp, frame_id, offset
        last_cell = -1
        for i in np.lexsort((raw_indices, cell_ids)):
            cell = int(cell_ids[i])
            if cell != last_cell:
                self._increment(self.cell_support, cell, "cell support")
                last_cell = cell
            point = points[i]
            z = float(point[2])
            cluster = self._select_cluster(cell, z)
            if cluster is None:
                continue
            idx = (cell, cluster)
            if not self.cluster_support[idx]:
                self.xyz_min[idx] = self.xyz_max[idx] = point
                self.first_stamp_ns[idx] = stamp
            else:
                np.minimum(self.xyz_min[idx], point, out=self.xyz_min[idx])
                np.maximum(self.xyz_max[idx], point, out=self.xyz_max[idx])
            if z <= self.params["low_height_m"]:
                self.cluster_flags[idx] |= CLUSTER_NEAR_GROUND
            if self.last_stamp_ns[idx] != stamp:
                self._increment(self.cluster_support, idx, "cluster support")
                self.last_stamp_ns[idx] = stamp
                self._observe_station(cell, cluster, point, raw_indices[i], sensor_xyz[:2], stamp, offset)

    def finalize(self):
        """Close accumulation and return the bounded evidence arrays."""
        if self._failed:
            raise RuntimeError("positive accumulator failed")
        if self._result is None:
            del self._station_a_extent, self._station_a_last_ns
            self.cluster_active = self.cluster_support > 0
            self.positive_witness = np.all(
                self.station_counts >= self.params["station_min_frames"], axis=2
            )
            self.qualified_positive = self.positive_witness & self._has_margins
            self._result = {
                name: array for name, array in vars(self).items()
                if isinstance(array, np.ndarray)
            }
        return self._result


def classify_cell(
    cluster_active, positive_witness, qualified_positive, cluster_flags,
    cell_flags, assessed, credible_conflict, *, calibration_valid=False,
    confirmed_invalid=False,
):
    """Return (PGM value, reason) for a cell already in the frozen candidate set.

    A calibrated clean witness may establish occupancy at its own height.
    `confirmed_invalid` represents independent evidence covering all hits in
    the cell. The default archive supplies no such evidence or calibration.
    """
    active, nominal, qualified, flags, assessed, conflict = (
        np.asarray(values) for values in (
            cluster_active, positive_witness, qualified_positive, cluster_flags,
            assessed, credible_conflict,
        )
    )
    if any(
        values.shape != (CLUSTERS_PER_CELL,)
        for values in (active, nominal, qualified, flags, assessed, conflict)
    ):
        raise ValueError("decision requires one value per height cluster")
    if cell_flags:
        return UNKNOWN, "height_cluster_overflow"
    if not np.any(active):
        return FREE, "no_positive_support"
    if confirmed_invalid:
        return FREE, "independent_invalid_hit_evidence"

    active = active.astype(bool)
    conflict = conflict.astype(bool)
    clean_witness = active & nominal.astype(bool) & (flags == 0)
    if calibration_valid and np.any(
        clean_witness & qualified.astype(bool) & assessed.astype(bool) & ~conflict
    ):
        return OCCUPIED, "calibrated_positive_no_credible_conflict"
    if np.any(active & conflict):
        return UNKNOWN, "credible_ray_conflict"
    if np.any(clean_witness):
        if not calibration_valid or not np.any(clean_witness & qualified.astype(bool)):
            return UNKNOWN, "positive_uncalibrated"
        return UNKNOWN, "ray_geometry_unassessed"
    if np.any(active & ((flags & CLUSTER_NEAR_GROUND) != 0)):
        return UNKNOWN, "near_ground_ambiguous"
    if np.any(active & (flags != 0)):
        return UNKNOWN, "cluster_ambiguity"
    return UNKNOWN, "insufficient_separated_support"
