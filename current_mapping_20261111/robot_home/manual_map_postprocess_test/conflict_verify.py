"""Read and verify A005 inputs and the committed Free-conflict result."""

import csv
from dataclasses import dataclass
import hashlib
import json
import math
import operator
from pathlib import Path
import re

import numpy as np
import yaml


OCCUPIED = 0
UNKNOWN = 205
FREE = 254
STATES = {"occupied": OCCUPIED, "unknown": UNKNOWN, "free": FREE}


def verify_maps(
    original,
    baseline,
    candidate,
    selectionmask,
    original_yaml_bytes,
    baseline_yaml_bytes,
    candidate_yaml_bytes,
    changed_records=None,
):
    """Raise ValueError on any invalid map, transition, or supplied record.

    The input reader validates the original YAML geometry. Byte equality and
    array shape equality here require the candidate to inherit that geometry.
    Records use zero-based PGM row and column and may also include selected
    cells whose state stayed unchanged.
    """
    for name, grid in (("original", original), ("baseline", baseline), ("candidate", candidate)):
        if not isinstance(grid, np.ndarray) or grid.dtype != np.uint8 or grid.ndim != 2:
            raise ValueError(f"{name} must be a two-dimensional uint8 grid")
        if grid.shape != original.shape or grid.size == 0:
            raise ValueError(f"{name} grid geometry differs from original or is empty")
        if np.any((grid != OCCUPIED) & (grid != UNKNOWN) & (grid != FREE)):
            raise ValueError(f"{name} contains a value outside the trinary PGM palette")
    if (
        not isinstance(selectionmask, np.ndarray)
        or selectionmask.dtype != np.bool_
        or selectionmask.shape != original.shape
    ):
        raise ValueError("selectionmask must be a bool grid with the original geometry")
    if not all(
        isinstance(value, bytes) and value
        for value in (original_yaml_bytes, baseline_yaml_bytes, candidate_yaml_bytes)
    ) or not original_yaml_bytes == baseline_yaml_bytes == candidate_yaml_bytes:
        raise ValueError("original, baseline and candidate YAML bytes must match")
    if np.any((baseline != original) & ((original != UNKNOWN) | (baseline != OCCUPIED))):
        raise ValueError("A005 baseline changes a state other than original Unknown to Occupied")
    if np.any(selectionmask & ((original != FREE) | (baseline != FREE))):
        raise ValueError("selected cells must be Free in both original and A005")

    changed = candidate != baseline
    if np.any(changed & (baseline == OCCUPIED)):
        raise ValueError("candidate changes an existing A005 Occupied cell")
    if np.any(changed & (baseline == UNKNOWN)):
        raise ValueError("candidate changes an existing A005 Unknown cell")
    if np.any(changed & (candidate == FREE)):
        raise ValueError("candidate creates a Free cell")
    if np.any(changed & ~selectionmask):
        raise ValueError("candidate changes a cell outside the selected original Free set")
    n_changed = int(np.count_nonzero(changed))
    records_verified = None
    if changed_records is not None:
        seen = np.zeros(original.shape, dtype=bool)
        height, width = original.shape
        changes_reported = 0
        records_verified = 0
        for record in changed_records:
            row, col, before, after = _record_integers(
                record, ("pgm_row", "pgm_col", "A_pgm", "candidate_pgm")
            )
            if not (0 <= row < height and 0 <= col < width and selectionmask[row, col]):
                raise ValueError(f"record lies outside selected cells at PGM ({col}, {row})")
            if seen[row, col]:
                raise ValueError(f"duplicate record at PGM ({col}, {row})")
            if before != baseline[row, col] or after != candidate[row, col]:
                raise ValueError(f"record values differ from maps at PGM ({col}, {row})")
            seen[row, col] = True
            changes_reported += before != after
            records_verified += 1
        if changes_reported != n_changed:
            raise ValueError("change records do not account for every changed cell")

    selected = candidate[selectionmask]
    return {
        "passed": True,
        "shape_rows_cols": list(original.shape),
        "yaml_bytes_identical": True,
        "all_a005_occupied_and_unknown_preserved": True,
        "changed_cells_outside_selection": 0,
        "new_free_cells": 0,
        "selected_cells": int(selected.size),
        "changed_cells": n_changed,
        "changed_records_verified": records_verified,
        "counts": {
            "original": _state_counts(original),
            "baseline": _state_counts(baseline),
            "candidate": _state_counts(candidate),
            "selected_after": _state_counts(selected),
        },
    }


def _record_integers(record, keys):
    try:
        return tuple(
            int(record[key]) if isinstance(record[key], str) else operator.index(record[key])
            for key in keys
        )
    except (KeyError, TypeError, ValueError) as exc:
        raise ValueError(f"invalid integer field in record (expected {', '.join(keys)})") from exc


def _state_counts(grid):
    return {name: int(np.count_nonzero(grid == value)) for name, value in STATES.items()}


HERE = Path(__file__).resolve().parent
OUTPUT_NAME = "enhanced_conflict"
ALGORITHM = "free_conflict_v1_uncalibrated"
A005_ALGORITHM = "A005_first_return_sensor_voxel_then_fixed_height_band_hit_union"
RES = .05
MAX_CELLS = 10_000_000
MAX_COORD = 1_000_000
MAGIC = b"A005OBS1"
A005_INPUT_NAMES = ("original/map.pgm", "original/map.yaml", "original/projection_stats.json",
                    "recording.json")
A005_HASHED_FILES = (*A005_INPUT_NAMES, "enhanced/map.pgm", "enhanced/map.yaml", "report.json")
CONFLICT_INPUT_NAMES = (*A005_HASHED_FILES, "original/free_evidence.npz", "a005_checkpoint.json",
                        "observations.bin")
CODE_FILES = ("offline_conflict.py", "conflict_verify.py", "conflict_core.py", "ray_geometry.py")
CHECKPOINT_HASHES = {
    "report_sha256": "report.json",
    "original_map_sha256": "original/map.pgm",
    "a005_map_sha256": "enhanced/map.pgm",
    "a005_yaml_sha256": "enhanced/map.yaml",
    "recording_sha256": "recording.json",
}


def require(ok, message):
    if not ok:
        raise ValueError(message)


def session_file(session, name):
    path = session / name
    require(path.resolve(strict=True) == path and path.is_file(), f"Expected a session file: {name}")
    return path


def sha(path):
    h = hashlib.sha256()
    with path.open("rb") as f:
        for block in iter(lambda: f.read(1 << 20), b""):
            h.update(block)
    return h.hexdigest()


def read_json(path):
    return json.loads(path.read_text(encoding="utf-8"))


def positive_int(value, name):
    require(type(value) is int and value > 0, name + " must be a positive integer")
    return value


def finite(value, shape, name):
    array = np.asarray(value, dtype=np.float64)
    require(array.shape == shape and np.isfinite(array).all() and np.abs(array).max() <= MAX_COORD,
            "Invalid " + name)
    return array


def read_pgm(path, shape, values=(0, 205, 254)):
    height, width = shape
    header = f"P5\n{width} {height}\n255\n".encode("ascii")
    with path.open("rb") as f:
        require(f.read(len(header)) == header, "Invalid PGM header: " + path.name)
        payload = f.read(height * width + 1)
    require(len(payload) == height * width, "Invalid PGM payload size: " + path.name)
    grid = np.frombuffer(payload, np.uint8).reshape(shape)
    require(np.isin(grid, values).all(), "Invalid PGM palette: " + path.name)
    return grid


@dataclass(frozen=True)
class A005:
    hashes: dict
    report: dict
    stats: dict
    recording: dict
    yaml: bytes
    lower: np.ndarray
    ground: np.ndarray
    basis: np.ndarray
    original: np.ndarray
    baseline: np.ndarray


def load_a005(session):
    """Validate fixed A005 files, including the sealed recording declaration."""
    paths = {name: session_file(session, name) for name in A005_HASHED_FILES}
    hashes = {name: sha(path) for name, path in paths.items()}
    stats = read_json(paths["original/projection_stats.json"])
    recording = read_json(paths["recording.json"])
    report = read_json(paths["report.json"])
    require(report["status"] == recording["status"] == "complete", "Incomplete A005 source")
    require(report["algorithm"] == A005_ALGORITHM, "Unexpected A005 algorithm")
    require(hashes["enhanced/map.pgm"] == report["candidate_pgm_sha256"]
            and hashes["enhanced/map.yaml"] == report["candidate_yaml_sha256"], "A005 hash mismatch")
    # Keys in a relocated report still point to its original location. Only the
    # names and hashes are used, and every file is opened under this session.
    reported_session = Path(report["session"])
    require(reported_session.is_absolute(), "Invalid reported A005 session path")
    for name in A005_INPUT_NAMES:
        require(report["original_inputs_sha256"].get(str(reported_session / name)) == hashes[name],
                "A005 input hash mismatch: " + name)
    yaml_bytes = paths["original/map.yaml"].read_bytes()
    require(yaml_bytes == paths["enhanced/map.yaml"].read_bytes(), "Changed A005 YAML")
    meta = yaml.safe_load(yaml_bytes)
    require(meta["image"] == "map.pgm" and meta["mode"] == "trinary" and meta["negate"] == 0,
            "Unexpected map mode")
    require(meta["resolution"] == stats["resolution"] == RES
            and 1/255 < meta["free_thresh"] < 50/255 < meta["occupied_thresh"] < 1,
            "Unexpected map resolution/thresholds")
    origin = finite(meta["origin"], (3,), "origin")
    lower = np.rint(origin[:2] / RES).astype(np.int64)
    require(origin[2] == 0 and np.allclose(lower * RES, origin[:2], atol=1e-8, rtol=0),
            "Non-lattice origin")
    shape = (positive_int(stats["height"], "height"), positive_int(stats["width"], "width"))
    require(math.prod(shape) <= MAX_CELLS, "Map cell budget exceeded")
    original = read_pgm(paths["original/map.pgm"], shape)
    baseline = read_pgm(paths["enhanced/map.pgm"], shape)
    require(_state_counts(original) == report["original_counts"]
            == {name: stats[name + "_cells"] for name in STATES}, "Original count mismatch")
    require(_state_counts(baseline) == report["candidate_counts"], "A005 count mismatch")
    require(np.all((original == baseline) | ((original == UNKNOWN) & (baseline == OCCUPIED))),
            "Invalid A005 transition")
    require(stats["frame"] == "native_projection" and stats["source_frame"] == "camera_init"
            and stats["min_height"] == .1 and stats["max_height"] == 1., "Projection frame/band changed")
    positive_int(stats["stamp_ns"], "native stamp")
    ground = finite(stats["ground"], (3,), "ground")
    basis = finite(stats["rotation"], (3, 3), "fixed basis")
    require(np.allclose(basis.T @ basis, np.eye(3), atol=1e-8, rtol=0)
            and np.isclose(np.linalg.det(basis), 1, atol=1e-8, rtol=0), "Improper fixed reference")
    ref = recording["reference"]
    require(ref["frame_id"] == "camera_init", "Recording frame changed")
    positive_int(ref["stamp_ns"], "reference stamp")
    require(np.allclose(finite(ref["ground"], (3,), "recorded ground"), ground, rtol=0, atol=1e-9)
            and np.allclose(finite(ref["rotation"], (3, 3), "recorded basis"), basis, rtol=0, atol=1e-9),
            "Recording reference mismatch")
    require(recording["format"] == MAGIC.decode("ascii"), "Unsupported recording format")
    metrics = ("frames", "points", "bytes", "first_stamp_ns", "last_stamp_ns")
    for key in metrics:
        positive_int(recording[key], key)
    require(recording["first_stamp_ns"] <= recording["last_stamp_ns"], "Invalid recording time range")
    for key in ("errors", "pending_scan_stamps", "pending_pose_stamps"):
        require(recording[key] == [], "Unsealed recording: " + key)
    require(isinstance(recording["warnings"], list), "Invalid recorder warnings")
    require(recording["stats"]["received_scan"] == recording["stats"]["received_pose"]
            == recording["frames"], "Pair count mismatch")
    require(isinstance(recording["sha256"], str) and re.fullmatch("[0-9a-f]{64}", recording["sha256"]),
            "Invalid recording SHA")
    require(all(report["observations"][key] == recording[key] for key in (*metrics, "sha256")),
            "A005 recording metadata mismatch")
    return A005(hashes, report, stats, recording, yaml_bytes, lower, ground, basis, original, baseline)


def verify_a005(session):
    """Verify the saved A005 result and recorder metadata without rereading the binary."""
    return load_a005(Path(session).resolve(strict=True)).report


def source_hashes():
    return {name: sha(session_file(HERE, name)) for name in CODE_FILES}


def load_conflict_source(session):
    """Verify the checkpoint and fixed file hashes, except the observation payload."""
    paths = {name: session_file(session, name) for name in CONFLICT_INPUT_NAMES}
    a005 = load_a005(session)
    hashes = dict(a005.hashes)
    for name in ("original/free_evidence.npz", "a005_checkpoint.json"):
        hashes[name] = sha(paths[name])
    checkpoint = read_json(paths["a005_checkpoint.json"])
    require(checkpoint["session"] == str(session) and checkpoint["status"] == "complete"
            and checkpoint["phase"] == "a005_complete"
            and checkpoint["fixed_original_save_verified"] is True, "Invalid A005 checkpoint")
    for field, name in CHECKPOINT_HASHES.items():
        require(checkpoint[field] == hashes[name], "A005 checkpoint hash mismatch: " + name)
    recording = a005.recording
    require(checkpoint["observations_sha256"] == recording["sha256"], "A005 checkpoint observation SHA mismatch")
    require(paths["observations.bin"].stat().st_size == recording["bytes"], "Observation size mismatch")
    code = source_hashes()
    require(all(checkpoint["source_sha256"].get(name) == digest for name, digest in code.items()),
            "A005 checkpoint worker source SHA mismatch")
    hashes["observations.bin"] = recording["sha256"]
    return a005, hashes, code


def _selected_records(path):
    with path.open(newline="", encoding="utf-8") as f:
        for record in csv.DictReader(f):
            (support,) = _record_integers(record, ("support_frames",))
            require(support >= 3, "Selected record has fewer than three distinct frames")
            yield record


def verify_uncalibrated_maps(a005, candidate, selection, candidate_yaml, cells_path):
    verification = verify_maps(a005.original, a005.baseline, candidate, selection,
                               a005.yaml, a005.yaml, candidate_yaml, _selected_records(cells_path))
    require(verification["selected_cells"] == verification["changed_cells"]
            == verification["changed_records_verified"]
            == verification["counts"]["selected_after"]["unknown"],
            "Uncalibrated v1 must change each selected Free cell to Unknown")
    return verification


def verify_result(session):
    """Verify the named result and fixed provenance before presenting its map.

    The worker checks the entire observation stream before committing. Readers
    verify its SHA declaration, size, and report binding without rescanning it.
    """
    session = Path(session).resolve(strict=True)
    root = session / OUTPUT_NAME
    require(root.resolve(strict=True) == root and root.is_dir(), "Expected a committed conflict directory")
    report = read_json(session_file(root, "report.json"))
    require(report["status"] == "complete" and report["committed_directory_name"] == OUTPUT_NAME
            and report["input_session"] == str(session) and report["algorithm"] == ALGORITHM
            and report["calibration_status"] == "uncalibrated", "Invalid conflict result report")
    a005, hashes, code = load_conflict_source(session)
    require(report["inputs_sha256"] == hashes and report["code_sha256"] == code, "Conflict source hash mismatch")
    required = {"map.pgm", "map.yaml", "selection_mask.pgm", "positive_only_mask.pgm", "cells.csv",
                "candidate_indices.u32", "excluded_hits.csv", "cluster_witnesses.jsonl", "parameters.json"}
    required.update(f"batches/{i:04d}.npz" for i in range(len(report["batches"])))
    files = {}
    for path in root.rglob("*"):
        require(not path.is_symlink(), "Unexpected result symlink")
        if path == root / "report.json":
            continue
        if path.is_file():
            files[path.relative_to(root).as_posix()] = path
        else:
            require(path == root / "batches" and path.is_dir(), "Unexpected result entry")
    artifacts = report["artifacts_sha256"]
    require(files.keys() == artifacts.keys() == required, "Conflict artifact list does not match disk")
    for name, path in files.items():
        require(sha(path) == artifacts[name], "Conflict artifact SHA mismatch: " + name)
    require(report["final_map_pgm_sha256"] == artifacts["map.pgm"]
            and report["final_map_yaml_sha256"] == artifacts["map.yaml"], "Final map hash mismatch")
    require(report["parameters"] == read_json(files["parameters.json"]), "Conflict parameters mismatch")
    shape = a005.original.shape
    candidate = read_pgm(files["map.pgm"], shape)
    selection = read_pgm(files["selection_mask.pgm"], shape, (0, 255)) == 255
    positive = read_pgm(files["positive_only_mask.pgm"], shape, (0, 255)) == 255
    require(not np.any(positive & ~selection) and report["nominal_positive_cells"] == int(positive.sum()),
            "Invalid nominal-positive mask")
    verification = verify_uncalibrated_maps(a005, candidate, selection, files["map.yaml"].read_bytes(),
                                           files["cells.csv"])
    require(verification == report["invariants"]
            and report["selection"]["selected_cells"] == verification["selected_cells"],
            "Conflict transition/count verification mismatch")
    return report


