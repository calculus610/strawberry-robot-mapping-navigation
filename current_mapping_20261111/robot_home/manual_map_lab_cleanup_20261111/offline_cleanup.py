#!/usr/bin/env python3
"""Create an independent, unvalidated old-occupancy cleanup candidate."""

import argparse
from collections import Counter
from dataclasses import asdict
import csv
import hashlib
import json
import math
import os
from pathlib import Path
import re
import resource
import signal
import struct
import sys
import tempfile
import shutil
import uuid
from datetime import datetime
import time
from types import ModuleType

import numpy as np

from cleanup_core import (
    EPS, FREE, HEIGHT_BAND, OCCUPIED, RESOLUTION, UNKNOWN, Policy, compose_map,
    independent, in_guarded_height, measure_ray,
)


HERE = Path(__file__).resolve().parent
LOCAL_BASELINE = HERE / "baseline_tools"
LIVE_SESSIONS = Path("/home/wheeltec/maps/manual_postprocess_test")
ALGORITHM = "lab_old_occupancy_withdrawal_v1"
MEMORY_LIMIT_BYTES = 1536 * 1024 * 1024
CODE_SHA = {
    "conflict_verify.py": "59878f9a7cb6566a99cacd7729c3634d5d098f4cb1059439f4711347e9775dfe",
    "offline_conflict.py": "89957b41b590908fdaefcf9c861b0f8761cb4f4960d3b8c5d474c91879e5db75",
    "ray_geometry.py": "1927f6fafb8aff77103d56fc828065013fc23977adcdd98c236f8eb31b5b6e9c",
}
FRAME = struct.Struct("<qI7d")
MAGIC = b"A005OBS1"
MAX_POINTS = 200_000
MAX_CELLS = 1_000_000
MAX_REGIONS = 2_000_000
MAX_SUPPORTS = 160_000
PAIR_BATCH = 8192
REGION_M = 0.025
Z_SLOTS = 37
SLOTS_PER_CELL = 4 * Z_SLOTS
PCD_SOURCE = "original/projection_source_camera_init.pcd"
C_DIR = "enhanced_conflict"
SHA_PATTERN = re.compile("[0-9a-f]{64}")


def require(ok, message):
    if not ok:
        raise ValueError(message)


def hash_file(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1 << 20), b""):
            digest.update(block)
    return digest.hexdigest()


def read_object(path, max_bytes=2 << 20):
    with path.open("rb") as stream:
        data = stream.read(max_bytes + 1)
    require(len(data) <= max_bytes, f"Oversized JSON: {path.name}")
    result = json.loads(data)
    require(isinstance(result, dict), f"Expected JSON object: {path.name}")
    return result


def load_baseline(path):
    """Use the prior release's verified source readers and atomic rename."""
    modules = []
    for filename, digest in CODE_SHA.items():
        file = path / filename
        source = file.read_bytes()
        require(hashlib.sha256(source).hexdigest() == digest, f"Unpinned baseline tool: {file}")
        name = file.stem
        module = ModuleType(name)
        module.__file__ = str(file)
        sys.modules[name] = module
        exec(compile(source, str(file), "exec"), module.__dict__)
        modules.append(module)
    return modules


def read_projection_pcd(path, expected_points):
    """Read the exact XYZ-only projection snapshot emitted by native_height_projector."""
    require(type(expected_points) is int and 0 < expected_points <= 1_000_000,
            "Invalid native snapshot point count")
    metadata = {}
    with path.open("rb") as stream:
        for _ in range(32):
            line = stream.readline(512).strip()
            require(line, "Missing binary PCD header")
            if line.startswith(b"#"):
                continue
            name, *values = line.split()
            require(name not in metadata, "Duplicate PCD header field")
            metadata[name] = values
            if name == b"DATA":
                break
        require(metadata.get(b"DATA") == [b"binary"]
                and metadata.get(b"FIELDS") == [b"x", b"y", b"z"]
                and metadata.get(b"SIZE") == [b"4"] * 3
                and metadata.get(b"TYPE") == [b"F"] * 3
                and metadata.get(b"COUNT") == [b"1"] * 3,
                "Unexpected projection PCD point format")
        require(int(metadata[b"POINTS"][0]) == expected_points
                and int(metadata[b"WIDTH"][0]) * int(metadata[b"HEIGHT"][0]) == expected_points,
                "Projection PCD point count mismatch")
        payload = stream.read(expected_points * 12 + 1)
    require(len(payload) == expected_points * 12, "Bad projection PCD payload length")
    xyz = np.frombuffer(payload, dtype="<f4").reshape(-1, 3).astype(np.float64)
    require(np.isfinite(xyz).all() and np.max(np.abs(xyz)) <= 1_000_000, "Unsafe PCD coordinates")
    return xyz


class Source:
    def __init__(self, session, tools, expected_pcd=None, expected_c_report=None):
        self.session = session.resolve(strict=True)
        self.verify, self.worker, self.rays = tools
        # Reject oversized metadata/geometry before the legacy reader allocates.
        metadata = read_object(self.file("original/projection_stats.json"))
        require(type(metadata.get("height")) is int and type(metadata.get("width")) is int
                and 0 < metadata["height"] * metadata["width"] <= MAX_CELLS,
                "Map exceeds cleanup cell budget")
        for name in self.verify.A005_HASHED_FILES:
            if name.endswith(".json"):
                read_object(self.file(name))
            elif name.endswith(".yaml"):
                require(self.file(name).stat().st_size <= 65536, "Oversized map YAML")
        a005 = self.verify.load_a005(self.session)
        self.a005 = a005
        self.shape = a005.original.shape
        require(a005.original.size <= MAX_CELLS, "Refresh map budget exceeded")
        c_report_name = f"{C_DIR}/report.json"
        c_report_path = self.file(c_report_name)
        report_sha = hash_file(c_report_path)
        require(expected_c_report is None or report_sha == expected_c_report, "C report SHA mismatch")
        c_report = read_object(c_report_path)
        checkpoint_name = "a005_checkpoint.json"
        checkpoint = read_object(self.file(checkpoint_name))
        require(c_report["status"] == "complete" and c_report["algorithm"] == self.verify.ALGORITHM
                and c_report["calibration_status"] == "uncalibrated"
                and c_report["committed_directory_name"] == C_DIR
                and c_report["input_session"] == checkpoint["session"] == a005.report["session"]
                and checkpoint["status"] == "complete" and checkpoint["phase"] == "a005_complete"
                and checkpoint["fixed_original_save_verified"] is True, "Invalid A005/C seal")
        require(all(checkpoint[key] == a005.hashes[name]
                    for key, name in self.verify.CHECKPOINT_HASHES.items())
                and all(checkpoint["source_sha256"].get(name) == digest
                        and c_report["code_sha256"].get(name) == digest
                        for name, digest in CODE_SHA.items()), "Checkpoint/C binding mismatch")
        self.recording = a005.recording
        self.observation_path = self.file("observations.bin")
        require(checkpoint["observations_sha256"] == self.recording["sha256"]
                and self.observation_path.stat().st_size == self.recording["bytes"],
                "Observation seal size or SHA declaration mismatch")
        self.hashes = dict(a005.hashes)
        self.hashes.update({
            "observations.bin": self.recording["sha256"],
            checkpoint_name: hash_file(self.file(checkpoint_name)),
            "original/free_evidence.npz": hash_file(self.file("original/free_evidence.npz")),
        })
        require(c_report["inputs_sha256"] == self.hashes, "C input SHA mismatch")
        self.hashes[c_report_name] = report_sha
        required = {"map.pgm", "map.yaml", "selection_mask.pgm", "positive_only_mask.pgm",
                    "cells.csv", "candidate_indices.u32", "excluded_hits.csv",
                    "cluster_witnesses.jsonl", "parameters.json"}
        required.update(f"batches/{i:04d}.npz" for i in range(len(c_report["batches"])))
        artifacts = c_report["artifacts_sha256"]
        require(artifacts.keys() == required, "Unexpected C artifact list")
        for name, expected in artifacts.items():
            relative = f"{C_DIR}/{name}"
            self.hashes[relative] = hash_file(self.file(relative))
            require(self.hashes[relative] == expected, f"C artifact SHA mismatch: {name}")
        require(c_report["final_map_pgm_sha256"] == artifacts["map.pgm"]
                and c_report["final_map_yaml_sha256"] == artifacts["map.yaml"], "C map SHA mismatch")
        self.yaml = self.file(f"{C_DIR}/map.yaml").read_bytes()
        self.grid = self.verify.read_pgm(self.file(f"{C_DIR}/map.pgm"), self.shape)
        selection = self.verify.read_pgm(self.file(f"{C_DIR}/selection_mask.pgm"), self.shape, (0, 255)) == 255
        invariants = self.verify.verify_uncalibrated_maps(
            a005, self.grid, selection, self.yaml, self.file(f"{C_DIR}/cells.csv"))
        require(invariants == c_report["invariants"], "C map invariants differ from its seal")
        self.stamp = a005.stats["stamp_ns"]
        common_end = min(self.stamp, a005.stats["free_observation_stamp_ns"],
                         self.recording["last_stamp_ns"])
        require(self.recording["first_stamp_ns"] <= common_end
                and c_report["common_time_window_ns"] == [self.recording["first_stamp_ns"], common_end]
                and str(self.stamp) in checkpoint["final_save_evidence"]["map_stamps_after_stop"],
                "Final projection and replay time window disagree")
        self.hashes[PCD_SOURCE] = hash_file(self.file(PCD_SOURCE))
        require(expected_pcd is None or expected_pcd == self.hashes[PCD_SOURCE],
                "Projection snapshot SHA mismatch")
        xyz = read_projection_pcd(self.file(PCD_SOURCE), a005.stats["native_points"])
        xyz = (xyz - a005.ground) @ a005.basis
        minimum = np.floor((xyz[:, :2].min(axis=0) - 0.50) / RESOLUTION).astype(np.int64)
        maximum = np.floor((xyz[:, :2].max(axis=0) + 0.50) / RESOLUTION).astype(np.int64)
        require(np.array_equal(minimum, a005.lower)
                and tuple(maximum - minimum + 1) == self.shape[::-1],
                "Projection snapshot does not reproduce saved map bounds")
        self.native_xyz = xyz[in_band(xyz)]
        ids, inside, _ = self.worker.grid_indices(self.native_xyz, a005.lower, self.shape)
        require(inside.all() and len(ids) == a005.stats["selected_points"]
                and np.array_equal(np.unique(ids), np.flatnonzero(a005.original.ravel() == OCCUPIED)),
                "Projection snapshot does not reproduce saved Occupied cells")
        order = np.argsort(ids, kind="stable")
        self.native_cells = ids[order]
        self.native_xyz = self.native_xyz[order]
        self.stream_report = None
        self.baseline_name = f"{C_DIR}/map.pgm"
        self.select_final_map()

    def select_final_map(self):
        """Use the actually selected completed C/D output, never a guessed map."""
        name = "session.json"
        state = read_object(self.file(name))
        require(state.get("status") == "complete" and state.get("phase") == "complete",
                "The source session must have finished before cleanup")
        reported = Path(state["session"])
        require(str(reported) == self.a005.report["session"], "Session identity mismatch")
        final = Path(state["final_map"])
        require(final in (reported / C_DIR / "map.yaml", reported / "enhanced_scatter/map.yaml"),
                "Cleanup requires a completed conflict or scatter final map")
        self.hashes[name] = hash_file(self.file(name))
        if final.parent.name == "enhanced_scatter":
            folder = "enhanced_scatter"
            report_name = f"{folder}/report.json"
            report = read_object(self.file(report_name))
            require(report.get("status") == "complete"
                    and report.get("algorithm") == "experimental_isolated_occupied_components_v1"
                    and report.get("committed_directory_name") == folder
                    and report.get("input_session") == str(reported), "Invalid scatter seal")
            require(all(self.hashes.get(key) == digest for key, digest in report["inputs_sha256"].items()),
                    "Scatter input hashes disagree with C")
            artifacts = report["artifacts_sha256"]
            require(set(artifacts) == {"map.pgm", "map.yaml", "components.csv", "changes.csv"},
                    "Unexpected scatter artifact set")
            for short, digest in artifacts.items():
                relative = f"{folder}/{short}"
                require(hash_file(self.file(relative)) == digest, "Scatter artifact hash mismatch")
                self.hashes[relative] = digest
            self.hashes[report_name] = hash_file(self.file(report_name))
            grid = self.verify.read_pgm(self.file(f"{folder}/map.pgm"), self.shape)
            changed = grid != self.grid
            require(self.file(f"{folder}/map.yaml").read_bytes() == self.yaml
                    and not np.any(changed & ((self.grid != OCCUPIED) | (grid != UNKNOWN)))
                    and int(changed.sum()) == report["invariants"]["changed_cells"]
                    and report["final_map_pgm_sha256"] == artifacts["map.pgm"]
                    and report["final_map_yaml_sha256"] == artifacts["map.yaml"],
                    "Scatter geometry/transition mismatch")
            self.grid = grid
            self.baseline_name = f"{folder}/map.pgm"
        # Native source may also contain cells already withdrawn by stage D.
        keep = self.grid.ravel()[self.native_cells] == OCCUPIED
        self.native_cells = self.native_cells[keep]
        self.native_xyz = self.native_xyz[keep]

    def file(self, name):
        return self.verify.session_file(self.session, name)

    def frames(self, deadline, policy=None):
        digest = hashlib.sha256()
        first = last = None
        frames = points = inside = max_gap = gaps = after_snapshot = 0
        previous = None
        still_since = None
        self.frame_motion_ok = False
        with self.observation_path.open("rb") as stream:
            magic = stream.read(len(MAGIC))
            require(magic == MAGIC, "Invalid observation format")
            digest.update(magic)
            while True:
                check_time(deadline)
                offset = stream.tell()
                header = stream.read(FRAME.size)
                if not header:
                    break
                require(len(header) == FRAME.size, "Truncated observation header")
                stamp, count, *pose = FRAME.unpack(header)
                require(stamp > 0 and (last is None or stamp > last), "Non-increasing scan stamps")
                require(0 < count <= MAX_POINTS and frames < self.recording["frames"]
                        and points + count <= self.recording["points"], "Observation exceeds sealed counts")
                payload = stream.read(count * 12)
                require(len(payload) == count * 12, "Truncated observation payload")
                digest.update(header)
                digest.update(payload)
                if last is not None:
                    gap = stamp - last
                    max_gap = max(max_gap, gap)
                    gaps += gap > 250_000_000
                frames += 1
                points += count
                first = stamp if first is None else first
                last = stamp
                position = self.verify.finite(pose[:3], (3,), "sensor position")
                quaternion = self.verify.finite(pose[3:], (4,), "sensor quaternion")
                require(abs(np.linalg.norm(quaternion) - 1.0) <= 1e-3, "Invalid sensor quaternion norm")
                raw = np.frombuffer(payload, dtype="<f4").reshape(-1, 3)
                require(np.isfinite(raw).all() and np.max(np.abs(raw)) <= 1_000_000,
                        "Unsafe raw observation coordinates")
                # The A005/final map includes tail frames after the PCD timestamp.
                # ALL sealed endpoints protect the map, even when moving.
                inside += 1
                after_snapshot += stamp > self.stamp
                rotation = self.worker.rotation(quaternion)
                xyz = (raw.astype(np.float64) @ rotation.T + position - self.a005.ground) @ self.a005.basis
                origin = (position - self.a005.ground) @ self.a005.basis
                self.frame_motion_ok = False
                if policy is not None and previous is not None:
                    old_stamp, old_position, old_rotation = previous
                    dt = (stamp - old_stamp) / 1e9
                    angle = np.arccos(np.clip((np.trace(old_rotation.T @ rotation) - 1) / 2, -1, 1))
                    slow = (0 < dt <= .25
                            and np.linalg.norm(position - old_position) / dt <= policy.max_translation_m_s
                            and angle / dt <= policy.max_rotation_rad_s)
                    still_since = (old_stamp if still_since is None else still_since) if slow else None
                    self.frame_motion_ok = still_since is not None and stamp - still_since >= policy.stationary_dwell_ns
                previous = (stamp, position.copy(), rotation.copy())
                yield frames - 1, offset, stamp, xyz, origin
            actual = dict(frames=frames, points=points, bytes=stream.tell(), sha256=digest.hexdigest(),
                          first_stamp_ns=first, last_stamp_ns=last, max_gap_ns=max_gap,
                          gaps_over_250ms=gaps)
        require(inside > 0 and all(self.recording.get(key) == val for key, val in actual.items()),
                "Full observation stream differs from the seal")
        require(actual["gaps_over_250ms"] == 0, "Recording has observation gaps over 250 ms")
        self.stream_report = dict(actual, frames_in_window=inside, frames_excluded=frames - inside,
                                  frames_after_projection_stamp=after_snapshot)

    def recheck(self, deadline):
        for name, expected in self.hashes.items():
            check_time(deadline)
            require(hash_file(self.file(name)) == expected, f"Source changed during refresh: {name}")


def in_band(xyz):
    return (xyz[:, 2] >= HEIGHT_BAND[0]) & (xyz[:, 2] <= HEIGHT_BAND[1])


def check_time(deadline):
    if time.monotonic() > deadline:
        raise TimeoutError("Refresh time budget exceeded")


def chunk_pairs(left, right):
    """Enumerate interval pairs in bounded vectors even for a dense target."""
    end = np.cumsum(right - left)
    if not len(end):
        return
    start = end - (right - left)
    for offset in range(0, int(end[-1]), PAIR_BATCH):
        number = np.arange(offset, min(int(end[-1]), offset + PAIR_BATCH))
        row = np.searchsorted(end, number, side="right")
        yield row, left[row] + number - start[row]


def capture_supports(source, policy, deadline):
    """Group recorded in-band endpoints by 3D subregion and associate native points."""
    black = np.flatnonzero(source.grid.ravel() == OCCUPIED)
    region_count = len(black) * SLOTS_PER_CELL
    require(region_count <= MAX_REGIONS, "Recording-support region budget exceeded")
    compact = np.full(source.grid.size, -1, dtype=np.int32)
    compact[black] = np.arange(len(black), dtype=np.int32)
    minimum = np.full((region_count, 3), np.inf)
    maximum = np.full((region_count, 3), -np.inf)
    region_last = np.zeros(region_count, np.int64)
    region_first = np.zeros(region_count, np.int64)
    region_hit_frames = np.zeros(region_count, np.int32)
    native_match = np.zeros(len(source.native_cells), np.int64)
    native_latest = np.zeros_like(native_match)
    native_first = np.zeros_like(native_match)
    native_hit_frames = np.zeros(len(native_match), np.int32)

    for _, _, stamp, xyz, _ in source.frames(deadline):
        xyz = xyz[in_band(xyz)]
        ids, valid, world = source.worker.grid_indices(xyz, source.a005.lower, source.shape)
        ids, xyz, world = ids[valid], xyz[valid], world[valid]
        is_black = compact[ids] >= 0
        ids, xyz, world = ids[is_black], xyz[is_black], world[is_black]
        half = np.clip(np.floor((xyz[:, :2] - world * RESOLUTION) / REGION_M).astype(np.int64), 0, 1)
        slot_z = np.clip(np.floor((xyz[:, 2] - 0.10) / REGION_M).astype(np.int64), 0, Z_SLOTS - 1)
        region = compact[ids] * SLOTS_PER_CELL + (half[:, 0] * 2 + half[:, 1]) * Z_SLOTS + slot_z
        np.minimum.at(minimum, region, xyz)
        np.maximum.at(maximum, region, xyz)
        seen = np.unique(region)
        region_first[seen[region_first[seen] == 0]] = stamp
        region_last[seen] = stamp
        region_hit_frames[seen] += 1

        left = np.searchsorted(source.native_cells, ids, side="left")
        right = np.searchsorted(source.native_cells, ids, side="right")
        frame_hit = np.zeros(len(native_latest), bool)
        for row, native in chunk_pairs(left, right):
            delta = xyz[row] - source.native_xyz[native]
            distance2 = np.einsum("ij,ij->i", delta, delta)
            strong = native[distance2 <= (policy.hit_match_m - policy.hit_error_m + EPS) ** 2]
            veto = native[distance2 <= (policy.hit_match_m + policy.hit_error_m + EPS) ** 2]
            native_match[strong] = stamp
            native_latest[veto] = stamp
            frame_hit[veto] = True
        native_first[frame_hit & (native_first == 0)] = stamp
        native_hit_frames[frame_hit] += 1

    seen = np.flatnonzero(region_last)
    require(len(source.native_xyz) + len(seen) <= MAX_SUPPORTS, "Refresh support budget exceeded")
    center = (minimum[seen] + maximum[seen]) / 2
    extent = np.linalg.norm(maximum[seen] - minimum[seen], axis=1) / 2
    cells = np.r_[source.native_cells, black[seen // SLOTS_PER_CELL]]
    order = np.argsort(cells, kind="stable")
    result = {
        "cell": cells,
        "native": np.r_[np.ones(len(native_match), bool), np.zeros(len(seen), bool)],
        "xyz": np.concatenate((source.native_xyz, center)),
        "extent": np.r_[np.zeros(len(native_match)), extent],
        "matched": np.r_[native_match, region_last[seen]],
        "last_hit": np.r_[native_latest, region_last[seen]],
        "first_hit": np.r_[native_first, region_first[seen]],
        "hit_frames": np.r_[native_hit_frames, region_hit_frames[seen]],
    }
    result = {name: values[order] for name, values in result.items()}
    # Temporal persistence is a protection heuristic, not a semantic table label.
    result["persistent"] = ((result["hit_frames"] >= policy.persistent_hit_frames)
                            & (result["last_hit"] - result["first_hit"] >= policy.persistent_span_ns))
    return result


def uncertain_hit_indices(supports, xyz, lower, shape, stamp, policy, deadline=None):
    """Veto later returns that could overlap support after endpoint error."""
    eligible = np.flatnonzero((supports["matched"] > 0) & (stamp > supports["last_hit"]))
    if not len(eligible) or not len(xyz):
        return np.empty(0, dtype=np.int64)
    radii = supports["extent"][eligible] + policy.hit_match_m + policy.hit_error_m
    margin = float(radii.max())
    require(margin < RESOLUTION, "Endpoint uncertainty exceeds the neighboring-cell search bound")
    xyz = xyz[(xyz[:, 2] >= HEIGHT_BAND[0] - margin) & (xyz[:, 2] <= HEIGHT_BAND[1] + margin)]
    if not len(xyz):
        return np.empty(0, dtype=np.int64)
    xy = np.floor(xyz[:, :2] / RESOLUTION).astype(np.int64) - lower
    height, width = shape
    cells = supports["cell"][eligible]
    veto = np.zeros(len(eligible), bool)
    for dx in (-1, 0, 1):
        for dy in (-1, 0, 1):
            col, y = xy[:, 0] + dx, xy[:, 1] + dy
            rows = np.flatnonzero((col >= 0) & (col < width) & (y >= 0) & (y < height))
            flat = (height - 1 - y[rows]) * width + col[rows]
            left = np.searchsorted(cells, flat, side="left")
            right = np.searchsorted(cells, flat, side="right")
            for point, support in chunk_pairs(left, right):
                if deadline is not None:
                    check_time(deadline)
                delta = xyz[rows[point]] - supports["xyz"][eligible[support]]
                distance2 = np.einsum("ij,ij->i", delta, delta)
                veto[support[distance2 <= (radii[support] + EPS) ** 2]] = True
    return eligible[veto]


class MissReplay:
    """Stream guarded farther returns against each support after its latest hit."""

    def __init__(self, supports, policy, rays):
        self.supports = supports
        self.policy = policy
        self.rays = rays
        count = len(supports["xyz"])
        shape = (count, policy.min_stations)
        self.votes = np.zeros(count, np.int32)
        self.ray_frames = np.zeros(count, np.int32)
        self.veto_frames = np.zeros(count, np.int32)
        self.stamp = np.zeros(shape, np.int64)
        self.frame = np.full(shape, -1, np.int32)
        self.offset = np.full(shape, -1, np.int64)
        self.raw_index = np.full(shape, -1, np.int32)
        self.origins = np.zeros((*shape, 3))
        self.bearings = np.zeros((*shape, 2))
        self.endpoints = np.zeros((*shape, 3))
        self.max_pair_count = 0
        self.motion_rejected_frames = 0
        self.eligible_motion_frames = 0
        self.occlusion_vetoes = 0

    def step(self, frame, offset, stamp, xyz, origin, deadline, grid_geometry, motion_ok):
        policy = self.policy
        supports = self.supports
        if grid_geometry is not None:
            lower, shape = grid_geometry
            veto = uncertain_hit_indices(supports, xyz, lower, shape, stamp, policy, deadline)
            supports["last_hit"][veto] = stamp
            self.veto_frames[veto] += 1
            self.votes[veto] = 0
            self.ray_frames[veto] = 0
            self.stamp[veto] = 0
        if not motion_ok:
            self.motion_rejected_frames += 1
            return
        self.eligible_motion_frames += 1
        margin = supports["extent"] + policy.beam_error_m + policy.hit_error_m
        active = np.flatnonzero((supports["matched"] > 0) & ~supports["persistent"]
                                & (stamp - supports["last_hit"] > 2 * policy.stamp_error_ns)
                                & (self.votes < policy.min_stations)
                                & (margin <= min(policy.coverage_radius_m, policy.height_tolerance_m)))
        if not len(active):
            return
        targets = supports["xyz"][active]
        relative = targets - origin
        horizontal_length = np.linalg.norm(relative[:, :2], axis=1)
        # The bounds require the supported closest point to lie in front of the
        # blind region, and a bearing independent of a pure vertical ray.
        target_length = np.linalg.norm(relative, axis=1)
        valid_target = ((horizontal_length > EPS)
                        & (target_length > policy.range_min_m)
                        & (target_length <= policy.range_max_m))
        active = active[valid_target]
        if not len(active):
            return
        targets, relative = targets[valid_target], relative[valid_target]
        bearing = relative[:, :2] / horizontal_length[valid_target, None]

        delta = xyz - origin
        length = np.linalg.norm(delta, axis=1)
        raw = np.flatnonzero((length > policy.range_min_m)
                             & (length + policy.beam_error_m <= policy.range_max_m))
        if not len(raw):
            return
        length = length[raw]
        delta = delta[raw]
        unit = delta / length[:, None]
        padding = (supports["extent"][active] + policy.coverage_radius_m
                   + policy.beam_error_m + policy.hit_error_m + EPS)[:, None]
        order, left, right = self.rays._azimuth_ranges(delta, origin, targets - padding, targets + padding)
        self.max_pair_count = max(self.max_pair_count, int(np.sum(right - left)))
        occluded = np.zeros(len(active), dtype=bool)
        best = np.full(len(active), np.inf)
        best_ray = np.full(len(active), -1, np.int32)
        for row, angle in chunk_pairs(left, right):
            check_time(deadline)
            index = order[angle % len(order)]
            valid, distance, _, along, height = measure_ray(
                relative[row], unit[index], length[index], supports["extent"][active[row]], policy)
            # Nearby foreground returns veto ambiguous farther rays in this frame.
            obstruction_radius = (supports["extent"][active[row]] + policy.coverage_radius_m
                                  + policy.beam_error_m + policy.hit_error_m)
            blocked = ((distance <= obstruction_radius) & (along > policy.range_min_m)
                       & (length[index] < along + policy.endpoint_guard_m))
            occluded[row[blocked]] = True
            valid &= in_guarded_height(height + origin[2], policy)
            if not valid.any():
                continue
            row, distance, index = row[valid], distance[valid], index[valid]
            ranking = np.lexsort((index, distance, row))
            ranked_rows = row[ranking]
            first = ranking[np.r_[True, ranked_rows[1:] != ranked_rows[:-1]]]
            first = first[(distance[first] < best[row[first]])
                          | ((distance[first] == best[row[first]]) & (index[first] < best_ray[row[first]]))]
            winners = row[first]
            best[winners] = distance[first]
            best_ray[winners] = index[first]

        self.occlusion_vetoes += int(np.count_nonzero(occluded & (best_ray >= 0)))
        present = (best_ray >= 0) & ~occluded
        candidate = active[present]
        self.ray_frames[candidate] += 1
        bearing = bearing[present]
        chosen_raw = raw[best_ray[present]]
        allowed = independent(stamp, origin, bearing[:, None, :],
                              self.stamp[candidate], self.origins[candidate],
                              self.bearings[candidate], policy)
        take = np.all(allowed | (self.stamp[candidate] == 0), axis=1)
        candidate = candidate[take]
        slot = self.votes[candidate]
        self.stamp[candidate, slot] = stamp
        self.frame[candidate, slot] = frame
        self.offset[candidate, slot] = offset
        self.raw_index[candidate, slot] = chosen_raw[take]
        self.origins[candidate, slot] = origin
        self.bearings[candidate, slot] = bearing[take]
        self.endpoints[candidate, slot] = xyz[chosen_raw[take]]
        self.votes[candidate] += 1

    def witnesses(self, index):
        for slot in range(self.votes[index]):
            yield {
                "stamp_ns": int(self.stamp[index, slot]),
                "frame": int(self.frame[index, slot]),
                "frame_offset": int(self.offset[index, slot]),
                "raw_index": int(self.raw_index[index, slot]),
                "origin_xyz": self.origins[index, slot].tolist(),
                "endpoint_xyz": self.endpoints[index, slot].tolist(),
            }


CELL_FIELDS = (
    "pgm_row", "pgm_col", "in_review_roi", "original_value", "baseline_value", "output_value",
    "native_points", "unmatched_native_points", "recorded_subregions", "eligible_supports",
    "min_stations", "max_stations", "latest_hit_ns", "policy_candidate", "changed", "reason",
)


def review_roi(value):
    result = tuple(int(entry) for entry in value.split(","))
    if len(result) != 4 or not (0 <= result[0] < result[1] and 0 <= result[2] < result[3]):
        raise argparse.ArgumentTypeError("ROI must be r0,r1,c0,c1 with increasing half-open bounds")
    return result


def classify(source, supports, replay):
    count = source.grid.size
    flat = supports["cell"]
    eligible = replay.votes >= replay.policy.min_stations
    matched = supports["matched"] > 0
    native = supports["native"]
    total = np.bincount(flat, minlength=count)
    native_count = np.bincount(flat[native], minlength=count)
    unmatched_count = np.bincount(flat[native & ~matched], minlength=count)
    eligible_count = np.bincount(flat[eligible], minlength=count)
    possible = ((total > 0) & (eligible_count == total) & (source.grid.ravel() == OCCUPIED)
                & ~source.protected.ravel())
    return total, native_count, unmatched_count, eligible_count, possible.reshape(source.shape)


def write_evidence(stage, source, supports, replay, output, classification, roi, deadline):
    total, native_count, unmatched_count, eligible_count, possible = classification
    reasons = Counter()
    roi_reasons = Counter()
    _, width = source.shape
    black = np.flatnonzero(source.grid.ravel() == OCCUPIED)
    left = np.searchsorted(supports["cell"], black, side="left")
    right = np.searchsorted(supports["cell"], black, side="right")
    with ((stage / "cells.csv").open("x", newline="", encoding="utf-8") as cell_file,
          (stage / "changes.csv").open("x", newline="", encoding="utf-8") as change_file,
          (stage / "supports.csv").open("x", newline="", encoding="utf-8") as support_file):
        cells = csv.writer(cell_file)
        changes = csv.writer(change_file)
        evidence = csv.writer(support_file)
        cells.writerow(CELL_FIELDS)
        changes.writerow(CELL_FIELDS)
        evidence.writerow(("pgm_row", "pgm_col", "support_index", "kind", "x_m", "y_m", "z_m",
                           "extent_m", "matched_hit_ns", "latest_hit_ns", "hit_frames", "persistent_protection",
                           "later_uncertain_veto_frames", "valid_ray_frames_until_quorum",
                           "stations", "station_witnesses_json"))
        for cell, start, stop in zip(black, left, right):
            check_time(deadline)
            row, col = divmod(int(cell), width)
            in_roi = roi is not None and roi[0] <= row < roi[1] and roi[2] <= col < roi[3]
            candidate = bool(possible[row, col])
            new = int(output[row, col])
            stations = replay.votes[start:stop]
            if source.protected[row, col]:
                reason = "explicit_protected_roi"
            elif np.any(supports["persistent"][start:stop]):
                reason = "persistent_structure_support"
            elif not total[cell]:
                reason = "no_source_support"
            elif unmatched_count[cell]:
                reason = "unmatched_native_support"
            elif candidate:
                reason = "all_supports_reached_station_quorum"
            elif not np.any(replay.ray_frames[start:stop]):
                reason = "no_eligible_farther_return_after_latest_hit"
            else:
                reason = "insufficient_independent_stations_for_all_supports"
            reasons[reason] += 1
            if in_roi:
                roi_reasons[reason] += 1
            changed = new != OCCUPIED
            record = (row, col, int(in_roi), int(source.a005.original[row, col]), OCCUPIED, new,
                      int(native_count[cell]), int(unmatched_count[cell]), int(total[cell] - native_count[cell]),
                      int(eligible_count[cell]), int(stations.min()) if len(stations) else 0,
                      int(stations.max()) if len(stations) else 0,
                      int(supports["last_hit"][start:stop].max()) if len(stations) else 0,
                      int(candidate), int(changed), reason)
            cells.writerow(record)
            if changed:
                changes.writerow(record)
            for index in range(start, stop):
                evidence.writerow((
                    row, col, index, "native_point" if supports["native"][index] else "recorded_endpoint_subregion",
                    *supports["xyz"][index], supports["extent"][index], supports["matched"][index],
                    supports["last_hit"][index], supports["hit_frames"][index], int(supports["persistent"][index]), replay.veto_frames[index],
                    replay.ray_frames[index],
                    replay.votes[index], json.dumps(list(replay.witnesses(index)), separators=(",", ":")),
                ))
    return reasons, roi_reasons


def counts(grid):
    return {name: int(np.count_nonzero(grid == value))
            for name, value in (("occupied", OCCUPIED), ("unknown", UNKNOWN), ("free", FREE))}


def latest_session(root):
    root = root.resolve(strict=True)
    candidates = sorted(root.glob("*/session.json"), key=lambda p: p.stat().st_mtime, reverse=True)
    for path in candidates:
        try:
            state = read_object(path)
            final = Path(state.get("final_map", ""))
            if (state.get("status") == "complete" and state.get("phase") == "complete"
                    and final.name == "map.yaml"
                    and final.parent.name in (C_DIR, "enhanced_scatter")):
                return path.parent
        except (OSError, ValueError):
            continue
    raise ValueError("No completed session with a final conflict/scatter map found")


def execute(args):
    from preview_cleanup import make_preview

    started = time.monotonic()
    deadline = started + args.max_seconds
    code_names = ("offline_cleanup.py", "cleanup_core.py", "preview_cleanup.py", "run_cleanup.sh")
    code_sha = {name: hash_file(HERE / name) for name in code_names}
    tools = load_baseline(LOCAL_BASELINE.resolve(strict=True))
    session = args.source_session if args.source_session else latest_session(args.sessions_root)
    print(f"SOURCE_SESSION={session}", flush=True)
    source = Source(session, tools, args.projection_source_sha256, None)
    source.protected = np.zeros(source.shape, dtype=bool)
    for r0, r1, c0, c1 in args.protect_roi:
        require(r1 <= source.shape[0] and c1 <= source.shape[1], "Protected ROI exceeds source grid")
        source.protected[r0:r1, c0:c1] = True
    roi = args.review_roi
    if roi:
        require(roi[1] <= source.shape[0] and roi[3] <= source.shape[1], "Review ROI exceeds grid")
    if args.output_dir:
        output_dir = args.output_dir.absolute()
    else:
        tag = datetime.now().strftime("%Y%m%d_%H%M%S") + "_" + uuid.uuid4().hex[:8]
        output_dir = args.output_root.absolute() / (source.session.name + "_" + tag)
    require(not output_dir.is_relative_to(source.session)
            and not os.path.lexists(output_dir), "Output must be new and outside the source session")
    # Resolve ancestors before creating anything, so a symlink cannot redirect
    # output into the source session or an unrelated directory.
    parent = output_dir.parent
    require(parent.resolve() == parent and not parent.is_relative_to(source.session),
            "Output ancestors must be canonical and outside the source session")
    parent.mkdir(parents=True, exist_ok=True)
    require(shutil.disk_usage(parent).free >= 512 * 1024 * 1024, "Need at least 512 MiB free for output")
    policy = Policy()
    print(f"BASELINE={source.baseline_name}", flush=True)
    print("CLEANUP 1/3: capture every sealed frame and protect persistent supports", flush=True)
    supports = capture_supports(source, policy, deadline)
    first_pass = source.stream_report
    captured_at = time.monotonic()
    replay = MissReplay(supports, policy, source.rays)
    print("CLEANUP 2/3: replay later hits and stationary background observations", flush=True)
    for frame, offset, stamp, xyz, origin in source.frames(deadline, policy):
        replay.step(frame, offset, stamp, xyz, origin, deadline,
                    (source.a005.lower, source.shape), source.frame_motion_ok)
        if (frame + 1) % 200 == 0:
            print(f"REPLAY_FRAMES={frame + 1}/{source.recording['frames']}", flush=True)
    require(source.stream_report == first_pass, "Observation seal differs across replays")
    replayed_at = time.monotonic()
    classification = classify(source, supports, replay)
    possible = classification[-1]
    output = compose_map(source.grid, possible)
    changed = output != source.grid
    require(np.array_equal(changed, possible)
            and not np.any(changed & ((source.grid != OCCUPIED) | (output != UNKNOWN)))
            and np.array_equal(output[source.grid != OCCUPIED], source.grid[source.grid != OCCUPIED])
            and not np.any(changed & source.protected), "Invalid cleanup transitions")
    print("CLEANUP 3/3: write independent candidate, comparison and evidence", flush=True)
    with tempfile.TemporaryDirectory(prefix=f".{output_dir.name}.incomplete-", dir=parent) as name:
        stage = Path(name)
        (stage / "before").mkdir()
        (stage / "before/map.pgm").write_bytes(source.file(source.baseline_name).read_bytes())
        (stage / "before/map.yaml").write_bytes(source.yaml)
        source.worker.write_pgm(stage / "map.pgm", output)
        (stage / "map.yaml").write_bytes(source.yaml)
        source.worker.write_pgm(stage / "withdrawn_mask.pgm", possible.astype(np.uint8) * 255)
        source.worker.write_pgm(stage / "protected_mask.pgm", source.protected.astype(np.uint8) * 255)
        reasons, roi_reasons = write_evidence(stage, source, supports, replay, output,
                                            classification, roi, deadline)
        make_preview(source.grid, output, stage / "comparison.png", {"resolution_m": RESOLUTION})
        require(np.array_equal(source.verify.read_pgm(stage / "map.pgm", source.shape), output)
                and (stage / "map.yaml").read_bytes() == source.yaml,
                "Saved candidate differs from calculated result")
        source.recheck(deadline)
        peak_rss = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss * (1 if sys.platform == "darwin" else 1024)
        require(peak_rss <= MEMORY_LIMIT_BYTES, "Cleanup process exceeded its 1536 MiB RSS budget")
        artifacts = {str(path.relative_to(stage)): hash_file(path)
                     for path in sorted(stage.rglob("*")) if path.is_file()}
        report = {
            "status": "complete", "algorithm": ALGORITHM,
            "mode": "independent_experimental_candidate", "calibration_status": "uncalibrated",
            "acceptance_status": "pending_user_test", "automatic_navigation_selection": False,
            "source_session": str(source.session), "baseline": source.baseline_name,
            "output_dir": str(output_dir), "projection_stamp_ns": source.stamp,
            "projection_source": PCD_SOURCE, "inputs_sha256": source.hashes,
            "code_sha256": code_sha, "baseline_tools_sha256": CODE_SHA,
            "artifacts_sha256": artifacts, "observations": source.stream_report,
            "recording_warnings": source.recording["warnings"], "policy": asdict(policy),
            "policy_note": "All geometric allowances and persistence thresholds are uncalibrated engineering assumptions.",
            "protect_rois_pgm": args.protect_roi, "review_roi": roi,
            "reasons": dict(reasons), "review_roi_reasons": dict(roi_reasons),
            "support_counts": {
                "native_points": int(supports["native"].sum()),
                "unmatched_native_points": int(np.count_nonzero(supports["native"] & (supports["matched"] == 0))),
                "recorded_subregions": int(np.count_nonzero(~supports["native"])),
                "persistent_protected": int(np.count_nonzero(supports["persistent"])),
                "with_station_quorum": int(np.count_nonzero(replay.votes >= policy.min_stations)),
                "with_later_uncertain_hit": int(np.count_nonzero(replay.veto_frames)),
            },
            "invariants": {
                "passed": True, "changed_cells": int(changed.sum()), "new_free_cells": 0,
                "nonoccupied_preserved": True, "yaml_bytes_identical": True,
                "explicit_protected_cells_preserved": True, "shape_rows_cols": list(source.shape),
                "counts": {"before": counts(source.grid), "candidate": counts(output)},
            },
            "timing_seconds": {"capture": captured_at - started, "replay": replayed_at - captured_at,
                               "elapsed_before_commit": time.monotonic() - started},
            "resources": {"peak_rss_bytes": int(peak_rss), "rss_commit_limit_bytes": MEMORY_LIMIT_BYTES,
                          "effective_address_space_cap_bytes": (resource.getrlimit(resource.RLIMIT_AS)[0]
                                                                if sys.platform.startswith("linux") else None),
                          "pair_chunk_cap": PAIR_BATCH, "max_frame_azimuth_pairs": replay.max_pair_count,
                          "stream_passes": 2},
            "observation_gates": {"motion_rejected_frames": replay.motion_rejected_frames,
                                  "stationary_frames": replay.eligible_motion_frames,
                                  "foreground_occlusion_vetoes": replay.occlusion_vetoes},
            "limits": [
                "Only Occupied to Unknown. Neither Unknown nor Free is changed; no passage is certified.",
                "All native and recorded 3D supports of a cell must be refuted. Unmatched or persistent supports protect it.",
                "Every sealed frame, including frames after the projection snapshot, contributes endpoint protection.",
                "Only stationary scan-end proxy rays vote. Endpoints in moving frames still protect/revoke votes.",
                "Repeated native snapshots are not fresh sensor hits. The full input seal is reread for every run.",
                "Persistence is not semantic recognition: stationary people and repeatable noise may remain protected.",
                "The 4 cm rod and table preservation are not validated. Camera color does not determine LiDAR reflectivity.",
                "A005OBS1 has no per-point optical poses; BEST_EFFORT recording cannot prove no simultaneous message loss.",
                "Protected ROIs are optional explicit user geometry, never inferred from the review ROI.",
                "No original files, FAST-LIO map, live RViz map or navigation configuration are modified.",
            ],
        }
        (stage / "report.json").write_text(json.dumps(report, indent=2, ensure_ascii=False, allow_nan=False) + "\n",
                                           encoding="utf-8")
        require(all(hash_file(HERE / name) == digest for name, digest in code_sha.items())
                and all(hash_file(LOCAL_BASELINE / name) == digest for name, digest in CODE_SHA.items()),
                "Cleanup code changed during execution")
        check_time(deadline)
        source.worker.exclusive_rename(stage, output_dir)
    return report


def valid_sha(value):
    if not SHA_PATTERN.fullmatch(value):
        raise argparse.ArgumentTypeError("Expected a lowercase SHA-256")
    return value


def alarm(_signal, _frame):
    raise TimeoutError("Cleanup time budget exceeded; no candidate committed")


def interrupt(_signal, _frame):
    raise KeyboardInterrupt


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source-session", type=Path, help="Default: newest completed session")
    parser.add_argument("--sessions-root", type=Path, default=LIVE_SESSIONS)
    parser.add_argument("--output-dir", type=Path, help="Explicit NEW result directory")
    parser.add_argument("--output-root", type=Path, default=Path("/home/wheeltec/maps/manual_lab_cleanup"))
    parser.add_argument("--protect-roi", type=review_roi, action="append", default=[],
                        help="Keep this zero-based PGM r0,r1,c0,c1 rectangle; repeatable")
    parser.add_argument("--review-roi", type=review_roi, help="Label statistics only; does NOT protect cells")
    parser.add_argument("--max-seconds", default=600.0, type=float)
    parser.add_argument("--projection-source-sha256", type=valid_sha,
                        help="Optional externally verified projection PCD hash")
    args = parser.parse_args()
    if not 0 < args.max_seconds <= 3600:
        parser.error("--max-seconds must be in (0, 3600]")
    if sys.platform.startswith("linux"):
        soft, hard = resource.getrlimit(resource.RLIMIT_AS)
        cap = min([MEMORY_LIMIT_BYTES] + [n for n in (soft, hard) if n != resource.RLIM_INFINITY])
        resource.setrlimit(resource.RLIMIT_AS, (cap, hard))
    signal.signal(signal.SIGTERM, interrupt)
    signal.signal(signal.SIGALRM, alarm)
    signal.setitimer(signal.ITIMER_REAL, args.max_seconds)
    try:
        report = execute(args)
    except KeyboardInterrupt:
        parser.exit(130, "Cleanup interrupted; original files unchanged.\n")
    except (OSError, ValueError, KeyError, TypeError, OverflowError, MemoryError, RuntimeError) as exc:
        parser.exit(1, f"Cleanup failed: {type(exc).__name__}: {exc}\n")
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0)
    print(f"CLEANUP_CANDIDATE_SAVED={report['output_dir']}")
    print(f"CHANGED_CELLS={report['invariants']['changed_cells']}")
    print(f"COMPARISON={report['output_dir']}/comparison.png")
    print("ACCEPTANCE=pending_user_test; zero changes can mean insufficient evidence, not a failed save")


if __name__ == "__main__":
    main()


