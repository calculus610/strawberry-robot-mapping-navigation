"""Geometry for an uncalibrated, user-reviewed offline cleanup candidate."""

from dataclasses import dataclass

import numpy as np


OCCUPIED, UNKNOWN, FREE = 0, 205, 254
RESOLUTION = 0.05
HEIGHT_BAND = (0.10 - 1e-6, 1.0 + 1e-6)
EPS = 1e-10


@dataclass(frozen=True)
class Policy:
    # Assumed allowances, NOT measured or certified calibration bounds.
    coverage_radius_m: float = 0.015
    height_tolerance_m: float = 0.015
    beam_error_m: float = 0.007
    hit_error_m: float = 0.003
    stamp_error_ns: int = 20_000_000
    hit_match_m: float = 0.015
    range_min_m: float = 0.50
    range_max_m: float = 5.0
    endpoint_guard_m: float = 0.15
    min_stations: int = 3
    station_distance_m: float = 0.30
    station_angle_deg: float = 20.0
    station_gap_ns: int = 1_000_000_000
    max_translation_m_s: float = 0.02
    max_rotation_rad_s: float = 0.01
    stationary_dwell_ns: int = 500_000_000
    persistent_hit_frames: int = 5
    persistent_span_ns: int = 2_000_000_000

    def __post_init__(self):
        values = (self.coverage_radius_m, self.height_tolerance_m, self.beam_error_m,
                  self.hit_error_m, self.hit_match_m, self.range_min_m, self.range_max_m,
                  self.endpoint_guard_m, self.station_distance_m, self.station_angle_deg)
        if (not np.isfinite(values).all()
                or not 0 <= self.beam_error_m + self.hit_error_m
                    < min(self.coverage_radius_m, self.height_tolerance_m)
                or self.beam_error_m < 0
                or not 0 <= self.hit_error_m < self.hit_match_m
                or type(self.stamp_error_ns) is not int or self.stamp_error_ns < 0
                or not 0 < self.range_min_m < self.range_max_m
                or self.endpoint_guard_m <= 0
                or self.station_distance_m <= 0
                or not 0 < self.station_angle_deg < 180
                or type(self.min_stations) is not int or self.min_stations < 2
                or type(self.station_gap_ns) is not int or self.station_gap_ns <= 0
                or not np.isfinite([self.max_translation_m_s, self.max_rotation_rad_s]).all()
                or self.max_translation_m_s <= 0 or self.max_rotation_rad_s <= 0
                or type(self.stationary_dwell_ns) is not int or self.stationary_dwell_ns <= 0
                or type(self.persistent_hit_frames) is not int or self.persistent_hit_frames < 2
                or type(self.persistent_span_ns) is not int or self.persistent_span_ns <= 0):
            raise ValueError("Invalid refresh policy")


def vector(xyz):
    value = np.asarray(xyz, dtype=np.float64)
    if value.shape != (3,) or not np.isfinite(value).all():
        raise ValueError("Expected finite XYZ coordinates")
    return value


def measure_ray(relative, unit, length, extent, policy):
    """Vectorized test of an actual endpoint's guarded, near-height ray segment."""
    along = np.einsum("ij,ij->i", relative, unit)
    delta = relative - along[:, None] * unit
    distance = np.linalg.norm(delta, axis=1)
    # The volume encloses every observed position represented by a subregion.
    margin = extent + policy.beam_error_m + policy.hit_error_m
    lateral = distance + margin
    height = np.abs(delta[:, 2]) + margin
    closest_height = relative[:, 2] - delta[:, 2]
    # The caller accounts for the optical origin's world height separately.
    valid = ((lateral <= policy.coverage_radius_m + EPS)
             & (height <= policy.height_tolerance_m + EPS)
             & (along > policy.range_min_m + margin + EPS)
             & (length - along > policy.endpoint_guard_m + margin + EPS)
             & (length + policy.beam_error_m <= policy.range_max_m)
             & (along + margin <= policy.range_max_m))
    return valid, distance, delta[:, 2], along, closest_height


def in_guarded_height(closest_height, policy):
    lower, upper = HEIGHT_BAND
    return ((closest_height >= lower + policy.beam_error_m)
            & (closest_height <= upper - policy.beam_error_m))


def ray_witness(origin_xyz, endpoint_xyz, target_xyz, policy, target_extent_m=0.0):
    """Return one geometric witness, including the actual farther return, or None."""
    origin, endpoint, target = map(vector, (origin_xyz, endpoint_xyz, target_xyz))
    if not np.isfinite(target_extent_m) or target_extent_m < 0:
        raise ValueError("Invalid target extent")
    direction = endpoint - origin
    length = np.linalg.norm(direction)
    if length == 0 or not HEIGHT_BAND[0] <= target[2] <= HEIGHT_BAND[1]:
        return None
    result = measure_ray((target - origin)[None, :], (direction / length)[None, :],
                         np.array([length]), np.array([target_extent_m]), policy)
    valid, distance, height_delta, along, closest_height = (item[0] for item in result)
    if not valid or not in_guarded_height(closest_height + origin[2], policy):
        return None
    return {
        "origin_xyz": origin.tolist(),
        "endpoint_xyz": endpoint.tolist(),
        "target_xyz": target.tolist(),
        "distance_m": float(distance),
        "target_minus_ray_height_m": float(height_delta),
        "distance_from_origin_m": float(along),
        "distance_to_return_m": float(length - along),
    }


def independent(stamp, origin, bearing, other_stamp, other_origin, other_bearing, policy):
    """Require a distinct time, optical center and bearing against each stored vote."""
    separation = np.linalg.norm(origin - other_origin, axis=-1)
    alignment = np.einsum("...i,...i->...", bearing, other_bearing)
    return ((stamp - other_stamp >= policy.station_gap_ns + 2 * policy.stamp_error_ns)
            & (separation >= policy.station_distance_m + 2 * policy.beam_error_m)
            & (alignment <= np.cos(np.deg2rad(policy.station_angle_deg)) + EPS))


class Evidence:
    """Scalar form of a target's chronological hit and separated-miss rule."""

    def __init__(self, policy=None):
        self.policy = policy if policy is not None else Policy()
        self.last_hit_ns = 0
        self.last_event_ns = 0
        self.stations = []

    def hit(self, stamp_ns):
        if stamp_ns < self.last_event_ns or stamp_ns <= 0:
            raise ValueError("Hits must be chronological")
        self.last_hit_ns = self.last_event_ns = stamp_ns
        self.stations.clear()

    def miss(self, stamp_ns, origin_xyz, target_xyz):
        """Consume a miss whose endpoint has already passed ray_witness."""
        if stamp_ns < self.last_event_ns or stamp_ns <= 0:
            raise ValueError("Misses must be chronological")
        self.last_event_ns = stamp_ns
        if self.last_hit_ns == 0 or stamp_ns - self.last_hit_ns <= 2 * self.policy.stamp_error_ns or self.eligible:
            return False
        origin, target = map(vector, (origin_xyz, target_xyz))
        horizontal = target[:2] - origin[:2]
        length = np.linalg.norm(horizontal)
        if length <= EPS:
            return False
        bearing = horizontal / length
        if any(not independent(stamp_ns, origin, bearing, *old, self.policy)
               for old in self.stations):
            return False
        self.stations.append((stamp_ns, origin.copy(), bearing))
        return True

    @property
    def eligible(self):
        return len(self.stations) >= self.policy.min_stations


def compose_map(baseline, candidate_mask):
    """Write withdrawals only in an independent UNVALIDATED candidate."""
    if (baseline.ndim != 2 or baseline.dtype != np.uint8 or not baseline.size
            or not np.isin(baseline, (OCCUPIED, UNKNOWN, FREE)).all()
            or candidate_mask.shape != baseline.shape or candidate_mask.dtype != np.bool_
            or np.any(candidate_mask & (baseline != OCCUPIED))):
        raise ValueError("Invalid baseline or Occupied-to-Unknown mask")
    result = baseline.copy()
    result[candidate_mask] = UNKNOWN
    return result
