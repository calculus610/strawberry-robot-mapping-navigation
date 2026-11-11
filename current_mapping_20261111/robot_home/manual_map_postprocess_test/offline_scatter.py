#!/usr/bin/env python3
"""Experimentally return isolated, single-frame C components to Unknown."""

import argparse
import csv
from dataclasses import dataclass
import hashlib
from itertools import zip_longest
import math
import os
from pathlib import Path
import shutil
import signal
import sys
import tempfile
import time

import numpy as np
from scipy import ndimage
from scipy.spatial.transform import Rotation

import conflict_verify as conflict
from offline_conflict import allowed_session, dump, exclusive_rename, grid_indices, peak_rss_bytes, write_pgm
from offline_enhance import FRAME, MAGIC, MAX_COORDINATE_M, MAX_FRAME_POINTS
from stream_kernel import RES, selected_band


HERE = Path(__file__).resolve().parent
OUTPUT_NAME = "enhanced_scatter"
ALGORITHM = "experimental_isolated_occupied_components_v1"
DEFAULT_MAX_SECONDS = 120.
OCCUPIED, UNKNOWN, FREE = conflict.OCCUPIED, conflict.UNKNOWN, conflict.FREE
CODE_FILES = ("offline_scatter.py", "offline_conflict.py", "conflict_verify.py",
              "offline_enhance.py", "stream_kernel.py")
ARTIFACTS = ("map.pgm", "map.yaml", "components.csv", "changes.csv")
OBSERVATIONS = "observations.bin"
C_ROOT = conflict.OUTPUT_NAME
C_FILES = tuple(f"{C_ROOT}/{name}" for name in ("report.json", "map.pgm", "map.yaml"))
RADIUS_CELLS = 4
MAX_AREA = 3
RING = tuple((row, col) for row in range(-1, 2) for col in range(-1, 2)
             if (row, col) != (0, 0))
CLOSE = tuple((row, col) for row in range(-3, 4) for col in range(-3, 4)
              if 0 < row * row + col * col < RADIUS_CELLS ** 2)
PARAMETERS = dict(connectivity=8, max_component_cells=MAX_AREA, resolution_m=RES,
                  min_external_occupied_center_distance_m=RADIUS_CELLS * RES,
                  boundary_margin_cells=RADIUS_CELLS, neighbor_ring_cells=1,
                  required_distinct_support_frames=1)
REASONS = ("single_frame_isolated", "unobserved_in_recording", "multiple_support_frames",
           "neighbor_hit_in_other_frame")
COMPONENT_FIELDS = ("component_id", "area_cells", "pgm_row_col_cells", "original_occupied_cells",
                    "a005_added_cells", "distinct_support_frames", "first_support_stamp_ns",
                    "first_ring_stamp_ns", "ring_hit_in_multiple_frames", "decision", "reason")
CHANGE_FIELDS = ("component_id", "pgm_row", "pgm_col", "world_grid_x", "world_grid_y",
                 "original_pgm", "a005_pgm", "c_pgm", "d_pgm", "reason")
LIMITS = [
    "This is an experimental filter. Single-frame evidence does not establish physical noise.",
    "Original Occupied cells can come from the native projection's accumulated map. "
    "Zero recording support preserves the full component.",
    "The recording and the native projection have different observation sources.",
    "The original and C maps are retained. Navigation performance is unmeasured.",
]


def check_deadline(deadline):
    if time.monotonic() > deadline:
        raise TimeoutError("Scatter time budget exceeded")


def file_identity(stat):
    return stat.st_dev, stat.st_ino, stat.st_size, stat.st_mtime_ns, stat.st_ctime_ns


@dataclass
class Source:
    session: Path
    a005: conflict.A005
    c_map: np.ndarray
    hashes: dict
    code: dict
    observation_identity: tuple

    @classmethod
    def load(cls, session):
        report_path = conflict.session_file(session, C_FILES[0])
        report_hash = conflict.sha(report_path)
        c_report = conflict.verify_result(session)
        a005 = conflict.load_a005(session)
        c_map_path = conflict.session_file(session, C_FILES[1])
        c_yaml_path = conflict.session_file(session, C_FILES[2])
        c_map = conflict.read_pgm(c_map_path, a005.original.shape)
        hashes = dict(c_report["inputs_sha256"])
        hashes.update(zip(C_FILES, (report_hash, c_report["final_map_pgm_sha256"],
                                    c_report["final_map_yaml_sha256"])))
        conflict.require(a005.hashes == {name: hashes[name] for name in a005.hashes}
                         and c_yaml_path.read_bytes() == a005.yaml, "C/A005 source changed")
        obs = conflict.session_file(session, OBSERVATIONS)
        code = {name: conflict.sha(conflict.session_file(HERE, name)) for name in CODE_FILES}
        return cls(session, a005, c_map, hashes, code, file_identity(obs.stat()))

    def recheck(self, deadline):
        for name, digest in self.hashes.items():
            check_deadline(deadline)
            path = conflict.session_file(self.session, name)
            if name == OBSERVATIONS:
                conflict.require(file_identity(path.stat()) == self.observation_identity,
                                 "Observation file changed during scatter processing")
            else:
                conflict.require(conflict.sha(path) == digest, "Scatter source changed: " + name)
        for name, digest in self.code.items():
            conflict.require(conflict.sha(conflict.session_file(HERE, name)) == digest,
                             "Scatter worker source changed: " + name)


@dataclass
class Geometry:
    owners: np.ndarray
    cells: np.ndarray
    starts: np.ndarray
    totals: dict

    def component(self, index):
        return self.cells[self.starts[index]:self.starts[index + 1]]

    @property
    def count(self):
        return len(self.starts) - 1


def find_geometry(c_map, deadline=math.inf):
    """Label C's entire black mask, then reject whole components by geometry."""
    shape = c_map.shape
    height, width = shape
    labels, count = ndimage.label(c_map == OCCUPIED, structure=np.ones((3, 3), dtype=bool))
    labels = labels.ravel()
    sizes = np.bincount(labels, minlength=count + 1)
    small = (sizes >= 1) & (sizes <= MAX_AREA)
    small[0] = False
    n_small = int(small.sum())
    flats = np.flatnonzero(small[labels])
    row, col = np.divmod(flats, width)
    boundary = ((row < RADIUS_CELLS) | (row >= height - RADIUS_CELLS)
                | (col < RADIUS_CELLS) | (col >= width - RADIUS_CELLS))
    small[labels[flats[boundary]]] = False
    n_interior = int(small.sum())
    flats = flats[small[labels[flats]]]
    own_labels = labels[flats]
    if len(flats):
        for dr, dc in CLOSE:
            check_deadline(deadline)
            neighbor = labels[flats + dr * width + dc]
            small[own_labels[(neighbor != 0) & (neighbor != own_labels)]] = False
    n_candidates = int(small.sum())
    flats = flats[small[own_labels]]
    cells = flats[np.argsort(labels[flats], kind="stable")]
    starts = np.concatenate(([0], np.cumsum(sizes[small])))
    owners = np.zeros(c_map.size, dtype=np.int32) if n_candidates else np.empty(0, dtype=np.int32)
    if n_candidates:
        component_ids = np.repeat(np.arange(1, n_candidates + 1, dtype=np.int32), sizes[small])
        owners[cells] = component_ids
        # Rings of different qualifying components cannot overlap, since their
        # closest member cells would then be less than four cells apart.
        for dr, dc in RING:
            adjacent = cells + dr * width + dc
            on_ring = labels[adjacent] == 0
            owners[adjacent[on_ring]] = -component_ids[on_ring]
    totals = dict(occupied_components=count, area_eligible_components=n_small,
                  protected_boundary_components=n_small - n_interior,
                  protected_nearby_occupied_components=n_interior - n_candidates,
                  geometry_candidates=n_candidates)
    return Geometry(owners, cells, starts, totals)


@dataclass
class Support:
    count: np.ndarray
    first: np.ndarray
    ring_first: np.ndarray
    ring_multiple: np.ndarray

    @classmethod
    def empty(cls, size):
        return cls(*(np.zeros(size, dtype=np.int64) for _ in range(3)),
                   np.zeros(size, dtype=bool))

    def add(self, owners, stamp):
        hits = owners[owners > 0] - 1
        ring = -owners[owners < 0] - 1
        new_hits = hits[self.count[hits] == 0]
        self.first[new_hits] = stamp
        self.count[hits] += 1
        new_ring = self.ring_first[ring] == 0
        self.ring_first[ring[new_ring]] = stamp
        self.ring_multiple[ring[~new_ring]] = True

    def decisions(self):
        # Result codes index REASONS. Across the ring, any distinct supporting
        # frame is conservative protection for a single-frame component.
        result = np.zeros(len(self.count), dtype=np.uint8)
        other_ring = (self.ring_first != 0) & ((self.ring_first != self.first) | self.ring_multiple)
        result[other_ring] = 3
        result[self.count == 0] = 1
        result[self.count >= 2] = 2
        return result


def read_support(source, geometry, deadline):
    """Read one sealed stream, transforming only when some component qualifies."""
    path = conflict.session_file(source.session, OBSERVATIONS)
    a005 = source.a005
    meta = a005.recording
    digest = hashlib.sha256()
    support = Support.empty(geometry.count)
    frames = points = first = last = later = max_points = 0
    with path.open("rb") as stream:
        conflict.require(file_identity(os.fstat(stream.fileno())) == source.observation_identity,
                         "Observation file changed before scatter processing")
        signature = stream.read(len(MAGIC))
        conflict.require(signature == MAGIC, "Invalid observation signature")
        digest.update(signature)
        while header := stream.read(FRAME.size):
            check_deadline(deadline)
            conflict.require(len(header) == FRAME.size, "Truncated observation frame header")
            stamp, n, *pose = FRAME.unpack(header)
            conflict.require(stamp > last, "Non-increasing observation frame stamp")
            conflict.require(1 <= n <= MAX_FRAME_POINTS and frames < meta["frames"]
                             and points + n <= meta["points"], "Unsafe or undeclared frame point count")
            block = stream.read(12 * n)
            conflict.require(len(block) == 12 * n, "Truncated observation frame payload")
            digest.update(header)
            digest.update(block)
            if geometry.count:
                position = conflict.finite(pose[:3], (3,), "observation position")
                quaternion = np.asarray(pose[3:], dtype=np.float64)
                norm = np.linalg.norm(quaternion)
                conflict.require(np.isfinite(norm) and norm > 0, "Invalid observation quaternion")
                cloud = np.frombuffer(block, dtype="<f4").reshape(n, 3)
                conflict.require(np.isfinite(cloud).all() and np.abs(cloud).max() <= MAX_COORDINATE_M,
                                 "Nonfinite or unsafe observation coordinates")
                band, _ = selected_band(cloud, position, Rotation.from_quat(quaternion).as_matrix(),
                                        a005.ground, a005.basis)
                flats, valid, _ = grid_indices(band, a005.lower, source.c_map.shape)
                support.add(np.unique(geometry.owners[flats[valid]]), stamp)
            frames += 1
            points += n
            first = first or stamp
            last = stamp
            later += stamp > a005.stats["stamp_ns"]
            max_points = max(max_points, n)
        observed = dict(frames=frames, points=points, bytes=stream.tell(), sha256=digest.hexdigest(),
                        first_stamp_ns=first, last_stamp_ns=last)
        conflict.require(file_identity(os.fstat(stream.fileno())) == source.observation_identity,
                         "Observation file changed while reading scatter evidence")
    for key, value in observed.items():
        conflict.require(meta[key] == value, "Scatter observation mismatch: " + key)
    observed.update(status="evaluated_all_recorded_frames" if geometry.count else "skipped_no_candidates",
                    frames_after_final_native_stamp=later, max_frame_points=max_points)
    return support, observed


def compose(source, geometry, decisions):
    c_map = source.c_map
    removed = geometry.cells[np.repeat(decisions == 0, np.diff(geometry.starts))]
    final = c_map.copy()
    final.ravel()[removed] = UNKNOWN
    return final


def map_invariants(source, final, geometry, decisions, yaml_bytes):
    original, baseline, c_map = source.a005.original, source.a005.baseline, source.c_map
    changed = final != c_map
    conflict.require(yaml_bytes == source.a005.yaml, "Scatter YAML differs from C")
    conflict.require(np.all(~changed | ((c_map == OCCUPIED) & (final == UNKNOWN))),
                     "Scatter may only change C Occupied to Unknown")
    conflict.require(np.array_equal(compose(source, geometry, decisions), final),
                     "Scatter map disagrees with the full-component decisions")
    original_removed = int(np.count_nonzero(changed & (original == OCCUPIED)))
    a005_removed = int(np.count_nonzero(changed & (original == UNKNOWN) & (baseline == OCCUPIED)))
    n_changed = int(changed.sum())
    conflict.require(original_removed + a005_removed == n_changed, "Unexpected changed-cell origin")
    return dict(passed=True, shape_rows_cols=list(final.shape), yaml_bytes_identical=True,
                c_free_and_unknown_preserved=True, new_free_cells=0, changed_cells=n_changed,
                changed_original_occupied_cells=original_removed, changed_a005_added_cells=a005_removed,
                changed_components=int(np.count_nonzero(decisions == 0)))


def report_rows(source, geometry, support, decisions, deadline, changes=False):
    width = source.c_map.shape[1]
    original, a005 = source.a005.original.ravel(), source.a005.baseline.ravel()
    x_lower, y_lower = map(int, source.a005.lower)
    height = source.c_map.shape[0]
    for index, decision in enumerate(decisions):
        check_deadline(deadline)
        cells = geometry.component(index)
        reason = REASONS[decision]
        if changes:
            if decision == 0:
                for flat in cells:
                    row, col = divmod(int(flat), width)
                    yield (index + 1, row, col, x_lower + col, y_lower + height - 1 - row,
                           int(original[flat]), int(a005[flat]), OCCUPIED, UNKNOWN, reason)
        else:
            coordinates = " ".join(f"{flat // width}:{flat % width}" for flat in cells)
            n_original = int(np.count_nonzero(original[cells] == OCCUPIED))
            yield (index + 1, len(cells), coordinates, n_original, len(cells) - n_original,
                   support.count[index], support.first[index] or "", support.ring_first[index] or "",
                   int(support.ring_multiple[index]), "remove" if decision == 0 else "retain", reason)


def csv_rows(source, geometry, support, decisions, deadline):
    yield "components.csv", COMPONENT_FIELDS, report_rows(source, geometry, support, decisions, deadline)
    yield "changes.csv", CHANGE_FIELDS, report_rows(source, geometry, support, decisions, deadline, changes=True)


def check_csv(path, fields, rows):
    with path.open(newline="", encoding="utf-8") as stream:
        reader = csv.reader(stream)
        conflict.require(next(reader, []) == list(fields), f"Invalid CSV header: {path.name}")
        for actual, expected in zip_longest(reader, rows):
            conflict.require(expected is not None and actual == list(map(str, expected)),
                             f"Scatter evidence CSV mismatch: {path.name}")


def decisions_report(geometry, decisions):
    return dict(geometry.totals, reasons={reason: int(np.count_nonzero(decisions == i))
                                        for i, reason in enumerate(REASONS)})


def artifact_hashes(root):
    return {name: conflict.sha(conflict.session_file(root, name)) for name in ARTIFACTS}


def verify_result(session):
    """Recompute geometry and evidence, and verify the exact published map."""
    session = Path(session).resolve(strict=True)
    root = session / OUTPUT_NAME
    conflict.require(root.resolve(strict=True) == root and root.is_dir(), "Invalid scatter result directory")
    conflict.require({p.name for p in root.iterdir()} == {*ARTIFACTS, "report.json"},
                     "Unexpected scatter artifacts")
    report = conflict.read_json(conflict.session_file(root, "report.json"))
    conflict.require(report["status"] == "complete" and report["algorithm"] == ALGORITHM
                     and report["input_session"] == str(session)
                     and report["committed_directory_name"] == OUTPUT_NAME
                     and report["parameters"] == PARAMETERS and report["limits"] == LIMITS,
                     "Invalid scatter result report")
    source = Source.load(session)
    conflict.require(report["inputs_sha256"] == source.hashes and report["code_sha256"] == source.code,
                     "Scatter input or source SHA mismatch")
    hashes = artifact_hashes(root)
    conflict.require(report["artifacts_sha256"] == hashes
                     and report["final_map_pgm_sha256"] == hashes["map.pgm"]
                     and report["final_map_yaml_sha256"] == hashes["map.yaml"],
                     "Scatter artifact SHA mismatch")
    geometry = find_geometry(source.c_map)
    support, observed = read_support(source, geometry, math.inf)
    decisions = support.decisions()
    for name, fields, rows in csv_rows(source, geometry, support, decisions, math.inf):
        check_csv(root / name, fields, rows)
    final = conflict.read_pgm(root / "map.pgm", source.c_map.shape)
    checks = map_invariants(source, final, geometry, decisions, (root / "map.yaml").read_bytes())
    conflict.require(report["observations"] == observed and report["selection"] == decisions_report(geometry, decisions)
                     and report["invariants"] == checks, "Scatter verification does not match report")
    source.recheck(math.inf)
    return report


def build(source, stage, started, deadline):
    validated = time.monotonic()
    check_deadline(deadline)
    geometry = find_geometry(source.c_map, deadline)
    geometry_done = time.monotonic()
    support, observed = read_support(source, geometry, deadline)
    streamed = time.monotonic()
    decisions = support.decisions()
    write_pgm(stage / "map.pgm", compose(source, geometry, decisions))
    (stage / "map.yaml").write_bytes(source.a005.yaml)
    for name, fields, rows in csv_rows(source, geometry, support, decisions, deadline):
        with (stage / name).open("w", newline="", encoding="utf-8") as stream:
            writer = csv.writer(stream)
            writer.writerow(fields)
            writer.writerows(rows)
    for name, fields, rows in csv_rows(source, geometry, support, decisions, deadline):
        check_csv(stage / name, fields, rows)
    saved = conflict.read_pgm(stage / "map.pgm", source.c_map.shape)
    checks = map_invariants(source, saved, geometry, decisions, (stage / "map.yaml").read_bytes())
    hashes = artifact_hashes(stage)
    source.recheck(deadline)
    staged = time.monotonic()
    return dict(status="complete", algorithm=ALGORITHM, committed_directory_name=OUTPUT_NAME,
                input_session=str(source.session), inputs_sha256=source.hashes, code_sha256=source.code,
                parameters=PARAMETERS, selection=decisions_report(geometry, decisions),
                observations=observed, invariants=checks, limits=LIMITS,
                artifacts_sha256=hashes, final_map_pgm_sha256=hashes["map.pgm"],
                final_map_yaml_sha256=hashes["map.yaml"],
                timing_seconds=dict(input_validation=validated - started, geometry=geometry_done - validated,
                                    stream_sha_and_evidence=streamed - geometry_done,
                                    stage_verify_and_recheck=staged - streamed,
                                    elapsed_before_commit=staged - started),
                resources=dict(core_peak_rss_bytes=peak_rss_bytes(), observation_passes=1,
                               observation_bytes=observed["bytes"], retained_cloud_frames=0))


def process(session, max_seconds=DEFAULT_MAX_SECONDS):
    """Commit a complete scatter result without replacing an existing result."""
    conflict.require(math.isfinite(max_seconds) and max_seconds > 0, "Invalid scatter time limit")
    started = time.monotonic()
    deadline = started + max_seconds
    session = Path(session).resolve(strict=True)
    output = session / OUTPUT_NAME
    conflict.require(not os.path.lexists(output), "Scatter output already exists")
    lock = session / f".{OUTPUT_NAME}.lock"
    stage = None
    identity = None
    descriptor = os.open(lock, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
    try:
        try:
            stat = os.fstat(descriptor)
            identity = stat.st_dev, stat.st_ino
            os.write(descriptor, f"{os.getpid()}\n".encode("ascii"))
        finally:
            os.close(descriptor)
        check_deadline(deadline)
        source = Source.load(session)
        stage = Path(tempfile.mkdtemp(dir=session, prefix=f".{OUTPUT_NAME}.incomplete-"))
        report = build(source, stage, started, deadline)
        dump(stage / "report.json", report)
        check_deadline(deadline)
        exclusive_rename(stage, output)
        return report
    finally:
        try:
            if stage is not None and stage.is_dir():
                shutil.rmtree(stage)
        finally:
            try:
                stat = lock.lstat()
                if (stat.st_dev, stat.st_ino) == identity:
                    lock.unlink()
            except FileNotFoundError:
                pass


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS)
    args = parser.parse_args()

    def stop(signum, _frame):
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, stop)
    try:
        session = allowed_session(args.session)
        process(session, args.max_seconds)
    except (Exception, KeyboardInterrupt) as error:
        print(f"SCATTER_MAP_FAILED {type(error).__name__}: {error}", file=sys.stderr, flush=True)
        return 1
    try:
        print(f"SCATTER_MAP_SAVED {session / OUTPUT_NAME / 'map.yaml'}", flush=True)
    except BrokenPipeError:
        sys.stdout = open(os.devnull, "w")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
