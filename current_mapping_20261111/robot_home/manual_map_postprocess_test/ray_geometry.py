"""Bounded diagnostics for rays drawn from recorded scan-end origins."""

import numpy as np


BLIND_M = 0.50
ENDPOINT_GUARD_M = 0.10
PAIR_CHUNK_LIMIT = 4096
EPS_M = 1e-9
EPS_RAD = 1e-12
TAU = 2.0 * np.pi
DEFAULT_RAY_PARAMS = {
    "blind_m": BLIND_M,
    "endpoint_guard_m": ENDPOINT_GUARD_M,
    "proxy_query_padding_m": 0.05,
    "proxy_association_m": 0.01,
    "ray_error_bound_m": None,
    "pair_chunk_limit": PAIR_CHUNK_LIMIT,
}

_PROXY_KINDS = ("near_support", "below_support", "unassociated_box",
                "padded_box", "protected_range")


def _parameters(params):
    padding = float(params.get("proxy_query_padding_m", DEFAULT_RAY_PARAMS["proxy_query_padding_m"]))
    association = float(params.get("proxy_association_m", DEFAULT_RAY_PARAMS["proxy_association_m"]))
    error = params.get("ray_error_bound_m", DEFAULT_RAY_PARAMS["ray_error_bound_m"])
    for name in ("blind_m", "endpoint_guard_m", "pair_chunk_limit"):
        if params.get(name, DEFAULT_RAY_PARAMS[name]) != DEFAULT_RAY_PARAMS[name]:
            raise ValueError(f"{name} is a fixed experiment parameter")
    if not np.isfinite(padding) or not np.isfinite(association) or not 0 < association <= padding:
        raise ValueError("Require 0 < proxy_association_m <= proxy_query_padding_m")
    if error is not None and (not np.isfinite(error) or error <= 0):
        raise ValueError("ray_error_bound_m must be a positive finite calibrated bound or null")
    return padding, association, error


def _slab_intervals(origin, directions, lower, upper, starts, ends):
    """Intersect with three closed slabs in units of distance along each ray."""
    offset_low = lower - origin
    offset_high = upper - origin
    parallel = directions == 0
    parallel_outside = np.any(parallel & ((offset_low > 0) | (offset_high < 0)), axis=-1)
    with np.errstate(divide="ignore", invalid="ignore", over="ignore"):
        t_low = offset_low / directions
        t_high = offset_high / directions
    enter = np.maximum(starts, np.max(np.where(parallel, -np.inf, np.minimum(t_low, t_high)), axis=-1))
    leave = np.minimum(ends, np.min(np.where(parallel, np.inf, np.maximum(t_low, t_high)), axis=-1))
    return np.where(parallel_outside, np.inf, enter), leave


def clip_segment_to_aabb(origin, endpoint, lower, upper,
                         blind_m=BLIND_M, endpoint_guard_m=ENDPOINT_GUARD_M):
    """Return distances delimiting a positive-length intersection, or None."""
    origin, endpoint, lower, upper = (
        np.asarray(value, dtype=np.float64) for value in (origin, endpoint, lower, upper)
    )
    if any(value.shape != (3,) or not np.all(np.isfinite(value))
           for value in (origin, endpoint, lower, upper)):
        raise ValueError("Ray coordinates must be finite XYZ triples")
    if (np.any(lower > upper) or not np.isfinite(blind_m) or not np.isfinite(endpoint_guard_m)
            or blind_m < 0 or endpoint_guard_m < 0):
        raise ValueError("Invalid box or protected range")
    direction = endpoint - origin
    length = np.linalg.norm(direction)
    if not np.isfinite(length) or length <= blind_m + endpoint_guard_m:
        return None
    enter, leave = _slab_intervals(
        origin, direction / length, lower, upper, blind_m, length - endpoint_guard_m
    )
    return (float(enter), float(leave)) if leave > enter + EPS_M else None


def _azimuth_ranges(deltas, origin, padded_lower, padded_upper):
    """Index rays by bearing in the XY plane, conservatively for the padded boxes.

    Each projected box lies inside its circumscribed disk. Outside that disk
    its bearing interval is bounded by the two tangents. Inside it we query
    every bearing. Repeating the sorted order at +2pi avoids wraparound gaps.
    """
    angles = np.arctan2(deltas[:, 1], deltas[:, 0])
    order = np.argsort(angles, kind="stable")
    sorted_angles = angles[order]
    extended_angles = np.concatenate((sorted_angles, sorted_angles + TAU))

    center = (padded_lower[:, :2] + padded_upper[:, :2]) / 2 - origin[:2]
    radii = np.hypot(*((padded_upper[:, :2] - padded_lower[:, :2]) / 2).T)
    center_distances = np.hypot(*center.T)
    all_bearings = center_distances <= radii + EPS_M
    ratio = np.divide(radii, center_distances, out=np.ones_like(radii), where=~all_bearings)
    half_width = np.arcsin(np.clip(ratio, 0.0, 1.0)) + EPS_RAD
    start = np.arctan2(center[:, 1], center[:, 0]) - half_width
    start = (start + np.pi) % TAU - np.pi
    left = np.searchsorted(extended_angles, start, side="left")
    right = np.searchsorted(extended_angles, start + 2 * half_width, side="right")
    left[all_bearings] = 0
    right[all_bearings] = len(order)
    return order, left, right


def _check_support(origin, unit, length, lower, upper, representatives,
                   padded_enter, padded_leave, association, padding):
    """Relate each proxy to actual support points within its valid range."""
    valid_end = length - ENDPOINT_GUARD_M
    enter = np.maximum(padded_enter, BLIND_M)
    leave = np.minimum(padded_leave, valid_end)
    valid_box = leave > enter + EPS_M
    raw_enter, raw_leave = _slab_intervals(origin, unit, lower, upper, BLIND_M, valid_end)
    raw_touch = raw_enter <= raw_leave + EPS_M
    on_box_plane = np.any((unit == 0) & ((lower == origin) | (upper == origin)), axis=1)
    graze = raw_touch & ((raw_leave <= raw_enter + EPS_M) | on_box_plane)

    relative = representatives - origin
    projection = np.einsum("ijk,ik->ij", relative, unit)
    valid_projection = (projection > BLIND_M + EPS_M) & (projection < valid_end[:, None] - EPS_M)
    reference_t = np.clip(projection, 0.0, length[:, None])
    valid_t = np.clip(projection, BLIND_M, valid_end[:, None])
    valid_distance = valid_box[:, None] & valid_projection
    closest_t = np.where(valid_distance, valid_t, reference_t)
    separation = relative - closest_t[:, :, None] * unit[:, None, :]
    distance2 = np.einsum("ijk,ijk->ij", separation, separation)
    distance2 = np.where(np.isfinite(distance2), distance2, np.inf)
    association2 = (association + EPS_M) ** 2
    near_support = valid_distance & (distance2 <= association2)
    near = np.any(near_support, axis=1)
    protected_support = ~valid_projection & (distance2 <= association2)
    protected = ~valid_box | (np.any(protected_support, axis=1) & ~near)

    xy_unit = unit[:, :2]
    xy_denominator = np.einsum("ij,ij->i", xy_unit, xy_unit)[:, None]
    xy_dot = np.einsum("ijk,ik->ij", relative[:, :, :2], xy_unit)
    xy_t = np.divide(xy_dot, xy_denominator, out=np.full_like(xy_dot, np.nan),
                     where=xy_denominator > 0)
    xy_separation = relative[:, :, :2] - xy_t[:, :, None] * xy_unit[:, None, :]
    xy_distance2 = np.einsum("ijk,ijk->ij", xy_separation, xy_separation)
    z_above_ray = relative[:, :, 2] - xy_t * unit[:, None, 2]
    below_support = (
        (xy_t > BLIND_M + EPS_M) & (xy_t < valid_end[:, None] - EPS_M)
        & (xy_distance2 <= association2)
        & (z_above_ray > association) & (z_above_ray <= padding + EPS_M)
    )
    below = valid_box & ~near & np.any(below_support, axis=1)
    unassociated = raw_touch & ~near
    kind = np.select((near, below, unassociated, protected), (0, 1, 2, 4), default=3)
    eligible_reference = np.any(valid_distance, axis=1)
    use_reference = np.where(eligible_reference[:, None], valid_distance, True)
    use_reference[below] = below_support[below]
    protected_with_reference = (kind == 4) & np.any(protected_support, axis=1)
    use_reference[protected_with_reference] = protected_support[protected_with_reference]
    support_index = np.argmin(np.where(use_reference, distance2, np.inf), axis=1)
    row = np.arange(len(unit))
    return {
        "proxy_bbox_count": raw_touch & ~graze,
        "proxy_near_count": near,
        "proxy_below_count": below,
        "proxy_unassociated_count": unassociated,
        "proxy_guard_count": protected,
        "proxy_graze_count": graze,
        "enter": enter,
        "leave": leave,
        "raw_enter": raw_enter,
        "raw_leave": raw_leave,
        "support_index": support_index,
        "support_t": closest_t[row, support_index],
        "distance2": distance2[row, support_index],
        "kind": kind,
    }


def describe_proxy_witness(sensor_xyz, endpoint_xyz, xyz_min, xyz_max, representatives, params):
    """Recompute one stored proxy from its raw return and the cluster summary."""
    padding, association, _ = _parameters(params)
    origin, endpoint, lower, upper, representatives = (
        np.asarray(value, dtype=np.float64)
        for value in (sensor_xyz, endpoint_xyz, xyz_min, xyz_max, representatives)
    )
    direction = endpoint - origin
    length = float(np.linalg.norm(direction))
    if length == 0.0 or not np.isfinite(length):
        raise ValueError("Witness return must have finite positive range")
    unit = direction[None, :] / length
    padded_range = padding + EPS_M
    full_enter, full_leave = _slab_intervals(
        origin, unit, lower - padded_range, upper + padded_range, 0.0, length
    )
    if full_enter[0] > full_leave[0] + EPS_M:
        raise ValueError("Witness ray misses the diagnostic query box")
    result = _check_support(
        origin, unit, np.array([length]), lower[None, :], upper[None, :],
        representatives[None, :, :], full_enter, full_leave, association, padding,
    )

    def interval(entry, exit):
        start, stop = float(entry[0]), float(exit[0])
        if start > stop + EPS_M:
            return None
        return {
            "enter_xyz": (origin + start * unit[0]).tolist(),
            "leave_xyz": (origin + stop * unit[0]).tolist(),
            "length_m": max(0.0, stop - start),
            "endpoint_after_leave_m": length - stop,
        }

    distance2 = float(result["distance2"][0])
    support = None
    if np.isfinite(distance2):
        index = int(result["support_index"][0])
        t = float(result["support_t"][0])
        closest = origin + t * unit[0]
        support = {
            "index": index,
            "xyz": representatives[index].tolist(),
            "closest_proxy_xyz": closest.tolist(),
            "distance_m": np.sqrt(distance2),
            "height_above_proxy_m": float(representatives[index, 2] - closest[2]),
            "distance_from_origin_m": t,
            "distance_to_return_m": length - t,
        }

    return {
        "kind": _PROXY_KINDS[result["kind"][0]],
        "origin_xyz": origin.tolist(),
        "endpoint_xyz": endpoint.tolist(),
        "full_query_interval": interval(full_enter, full_leave),
        "effective_query_interval": interval(result["enter"], result["leave"]),
        "effective_support_box_interval": interval(result["raw_enter"], result["raw_leave"]),
        "nearest_support": support,
    }


_COUNT_FIELDS = (
    "query_pair_count", "proxy_padded_box_count", "proxy_bbox_count", "proxy_near_count",
    "proxy_below_count", "proxy_unassociated_count", "proxy_guard_count", "proxy_graze_count",
    "proxy_event_switches", "proxy_hit_to_near_switches", "proxy_near_to_hit_switches",
    "sync_ambiguous_frames",
)


class RayDiagnostics:
    """Stream actual returns against a bounded batch of support clusters.

    The padded box is a diagnostic query range. It bounds these proxy checks,
    while the source record provides no bound on per-return origin error or
    complete support-surface geometry for a certified absence of conflict.
    """

    def __init__(self, snapshot, params):
        self.padding, self.association, self.error = _parameters(params)
        self.active = np.asarray(snapshot["cluster_active"], dtype=bool)
        if self.active.ndim != 2 or self.active.shape[1] != 4:
            raise ValueError("Require cluster_active[N,4]")
        self.shape = self.active.shape
        self.flat_active = np.flatnonzero(self.active)
        self.lower = np.asarray(snapshot["xyz_min"], dtype=np.float64)
        self.upper = np.asarray(snapshot["xyz_max"], dtype=np.float64)
        self.lower_active = self.lower.reshape(-1, 3)[self.flat_active]
        self.upper_active = self.upper.reshape(-1, 3)[self.flat_active]
        self.padded_lower = self.lower_active - (self.padding + EPS_M)
        self.padded_upper = self.upper_active + (self.padding + EPS_M)
        self.representatives = np.asarray(snapshot["representatives"])
        self.active_reps = self.representatives.reshape(-1, 4, 3)[self.flat_active]
        self.first_hit = np.asarray(snapshot["first_stamp_ns"]).reshape(-1)[self.flat_active]
        self.last_hit = np.asarray(snapshot["last_stamp_ns"]).reshape(-1)[self.flat_active]
        if (not np.all(np.isfinite(self.lower_active)) or not np.all(np.isfinite(self.upper_active))
                or np.any(self.lower_active > self.upper_active)):
            raise ValueError("Active cluster has invalid XYZ range")

        count = len(self.flat_active)
        self.counts = {name: np.zeros(count, dtype=np.uint32) for name in _COUNT_FIELDS}
        self.before_hit = np.zeros(count, dtype=bool)
        self.after_hit = np.zeros(count, dtype=bool)
        self.same_frame_hit = np.zeros(count, dtype=bool)
        self.last_event = np.zeros(count, dtype=np.uint8)
        self.first_proxy_stamp = np.full(count, -1, dtype=np.int64)
        self.last_proxy_stamp = np.full(count, -1, dtype=np.int64)
        self.witness_kind = np.full(count, len(_PROXY_KINDS), dtype=np.uint8)
        self.witness_dist2 = np.full(count, np.inf)
        self.witness_offset = np.full(count, -1, dtype=np.int64)
        self.witness_index = np.full(count, -1, dtype=np.int64)
        self.witness_frame = np.full(count, -1, dtype=np.int64)
        self.last_processed_stamp = -1
        self.stats = {
            "frames": 0,
            "raw_points": 0,
            "zero_length_rays": 0,
            "peak_index_array_payload_bytes": 0,
            "peak_azimuth_pairs_per_frame": 0,
            "peak_pair_chunk": 0,
            "pair_chunk_limit": PAIR_CHUNK_LIMIT,
        }

    def _add_counts(self, name, increment):
        old = self.counts[name]
        total = old.astype(np.uint64) + increment.astype(np.uint64)
        if np.any(total > np.iinfo(old.dtype).max):
            raise OverflowError(f"P3 {name} counter overflow")
        old[:] = total

    def _sync_hits(self, positive_cell_ids, positive_points):
        hits = np.zeros(self.shape, dtype=bool)
        ambiguous = np.zeros(self.shape, dtype=bool)
        if positive_cell_ids is not None:
            ids = np.asarray(positive_cell_ids, dtype=np.intp)
            xyz = np.asarray(positive_points, dtype=np.float64)[:, None, :]
            matches = (self.active[ids] & np.all(xyz >= self.lower[ids] - EPS_M, axis=-1)
                       & np.all(xyz <= self.upper[ids] + EPS_M, axis=-1))
            unique = np.count_nonzero(matches, axis=1) == 1
            np.logical_or.at(hits, ids, matches & unique[:, None])
            np.logical_or.at(ambiguous, ids, matches & ~unique[:, None])
        self._add_counts("sync_ambiguous_frames", ambiguous.reshape(-1)[self.flat_active])
        return hits.reshape(-1)[self.flat_active]

    def _record_witnesses(self, cids, indices, result, frame_id, offset):
        kinds = result["kind"]
        distances = result["distance2"]
        ranking = np.lexsort((indices, distances, kinds, cids))
        ranked_clusters = cids[ranking]
        first_in_cluster = np.r_[True, ranked_clusters[1:] != ranked_clusters[:-1]]
        winners = ranking[first_in_cluster]
        cluster = cids[winners]
        distance2 = distances[winners]
        kind = kinds[winners]
        tie = (kind == self.witness_kind[cluster]) & (distance2 == self.witness_dist2[cluster])
        better = ((kind < self.witness_kind[cluster])
                  | ((kind == self.witness_kind[cluster]) & (distance2 < self.witness_dist2[cluster]))
                  | (tie & (self.witness_offset[cluster] == offset)
                     & (indices[winners] < self.witness_index[cluster])))
        cluster = cluster[better]
        self.witness_kind[cluster] = kind[better]
        self.witness_dist2[cluster] = distance2[better]
        self.witness_offset[cluster] = offset
        self.witness_frame[cluster] = frame_id
        self.witness_index[cluster] = indices[winners[better]]

    def _frame_proxies(self, all_points, origin, frame_id, offset):
        count = len(self.flat_active)
        frame_near = np.zeros(count, dtype=bool)
        if count == 0 or not len(all_points):
            return frame_near

        deltas = np.asarray(all_points, dtype=np.float64) - origin
        length = np.linalg.norm(deltas, axis=1)
        if not np.all(np.isfinite(length)):
            raise ValueError("P3 ray length overflow or non-finite coordinate")
        nonzero = length > 0.0
        self.stats["zero_length_rays"] += int(np.count_nonzero(~nonzero))
        unit = np.divide(deltas, length[:, None], out=np.zeros_like(deltas), where=nonzero[:, None])
        order, left, right = _azimuth_ranges(deltas, origin, self.padded_lower, self.padded_upper)
        counts = right - left
        ends = np.cumsum(counts, dtype=np.int64)
        starts = ends - counts
        self.stats["peak_index_array_payload_bytes"] = max(
            self.stats["peak_index_array_payload_bytes"],
            sum(array.nbytes for array in (deltas, length, nonzero, unit, order,
                                           left, right, counts, ends, starts)),
        )
        total_pairs = int(ends[-1])
        self.stats["peak_azimuth_pairs_per_frame"] = max(
            self.stats["peak_azimuth_pairs_per_frame"], total_pairs
        )
        self._add_counts("query_pair_count", counts)
        frame_counts = {name: np.zeros(count, dtype=np.int64)
                        for name in _COUNT_FIELDS if name.startswith("proxy_") and "switches" not in name}
        for start in range(0, total_pairs, PAIR_CHUNK_LIMIT):
            pairs = np.arange(start, min(start + PAIR_CHUNK_LIMIT, total_pairs))
            self.stats["peak_pair_chunk"] = max(self.stats["peak_pair_chunk"], len(pairs))
            cids = np.searchsorted(ends, pairs, side="right")
            indices = order[(left[cids] + pairs - starts[cids]) % len(order)]
            full_enter, full_leave = _slab_intervals(
                origin, unit[indices], self.padded_lower[cids], self.padded_upper[cids],
                0.0, length[indices],
            )
            intersects = nonzero[indices] & (full_enter <= full_leave + EPS_M)
            if not np.any(intersects):
                continue

            cids, indices = cids[intersects], indices[intersects]
            result = _check_support(
                origin, unit[indices], length[indices], self.lower_active[cids], self.upper_active[cids],
                self.active_reps[cids], full_enter[intersects], full_leave[intersects],
                self.association, self.padding,
            )
            frame_counts["proxy_padded_box_count"] += np.bincount(cids, minlength=count)
            for name in frame_counts:
                if name in result:
                    frame_counts[name] += np.bincount(cids[result[name]], minlength=count)
            self._record_witnesses(cids, indices, result, frame_id, offset)

        for name, increment in frame_counts.items():
            self._add_counts(name, increment)
        return frame_counts["proxy_near_count"] > 0

    def update_frame(self, all_points, sensor_xyz, stamp_ns, frame_id, frame_offset,
                     positive_cell_ids=None, positive_points=None):
        if stamp_ns <= self.last_processed_stamp:
            raise ValueError("P3 frames must have strictly increasing timestamps")
        if (positive_cell_ids is None) != (positive_points is None):
            raise ValueError("P3 synchronous hit checking requires both cell indices and XYZ")
        self.last_processed_stamp = stamp_ns
        self.stats["frames"] += 1
        self.stats["raw_points"] += len(all_points)
        origin = np.asarray(sensor_xyz, dtype=np.float64)
        hit = self._sync_hits(positive_cell_ids, positive_points)
        near = self._frame_proxies(all_points, origin, frame_id, frame_offset)
        self.before_hit |= near & (stamp_ns < self.last_hit)
        self.after_hit |= near & (self.first_hit > 0) & (stamp_ns > self.first_hit)
        self.same_frame_hit |= hit & near
        self.first_proxy_stamp[near & (self.first_proxy_stamp < 0)] = stamp_ns
        self.last_proxy_stamp[near] = stamp_ns

        event = hit.astype(np.uint8) + 2 * near.astype(np.uint8)
        observed = event > 0
        self._add_counts("proxy_event_switches", observed & (self.last_event > 0) & (event != self.last_event))
        self._add_counts("proxy_hit_to_near_switches", (self.last_event == 1) & (event == 2))
        self._add_counts("proxy_near_to_hit_switches", (self.last_event == 2) & (event == 1))
        self.last_event[observed] = event[observed]

    def finalize(self):
        def restore(values, default):
            result = np.full(self.shape, default, dtype=values.dtype)
            result.reshape(-1)[self.flat_active] = values
            return result

        result = {name: restore(values, 0) for name, values in self.counts.items()}
        # The current source format has no independently verified support geometry.
        result.update(
            assessed=np.zeros(self.shape, dtype=bool),
            credible_conflict=np.zeros(self.shape, dtype=bool),
            assessment_reason=("missing_error_bound" if self.error is None else "insufficient_surface_coverage"),
            proxy_before_hit=restore(self.before_hit, False),
            proxy_after_hit=restore(self.after_hit, False),
            proxy_same_frame_hit=restore(self.same_frame_hit, False),
            first_proxy_stamp_ns=restore(self.first_proxy_stamp, -1),
            last_proxy_stamp_ns=restore(self.last_proxy_stamp, -1),
            nearest_proxy_distance_m=restore(
                np.where(np.isfinite(self.witness_dist2), np.sqrt(self.witness_dist2), np.nan), np.nan
            ),
            proxy_frame_offset=restore(self.witness_offset, -1),
            proxy_frame_id=restore(self.witness_frame, -1),
            proxy_point_index=restore(self.witness_index, -1),
            credible_frame_offset=np.full(self.shape, -1, dtype=np.int64),
            credible_point_index=np.full(self.shape, -1, dtype=np.int64),
        )
        return result
