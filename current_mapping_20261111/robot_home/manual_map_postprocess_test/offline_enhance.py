#!/usr/bin/env python3
"""Build an offline A005 derivative from a finished, isolated map test session."""
import argparse
import csv
import hashlib
import json
import os
from pathlib import Path
import resource
import struct
import sys
import tempfile
import time
from datetime import datetime, timezone

import numpy as np
from PIL import Image, ImageDraw
from scipy.spatial.transform import Rotation
import yaml

from stream_kernel import HIGH, LOW, RES, HitTiles, selected_band


MAGIC = b"A005OBS1"
FRAME = struct.Struct("<qI7d")
REFERENCE_ATOL = 1e-9
MAX_FRAME_POINTS = 200_000
MAX_MAP_CELLS = 10_000_000
MAX_COORDINATE_M = 1_000_000
OCCUPIED, UNKNOWN, FREE = 0, 205, 254
REASON = "recorded_height_band_endpoint_in_original_unknown_cell"
LIVE_ROOT = Path("/home/wheeltec/maps/manual_postprocess_test")
FIXTURE_ROOT = Path(__file__).resolve().parent / "verification"


class ExistingOutputError(ValueError):
    pass


def sha(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1 << 20), b""):
            digest.update(chunk)
    return digest.hexdigest()


def now_utc():
    return datetime.now(timezone.utc).isoformat()


def exists(path):
    return os.path.lexists(path)


def finite_array(value, shape, name):
    array = np.asarray(value, dtype=np.float64)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"Invalid {name}")
    return array


def positive_int(value, name):
    if type(value) is not int or value < 1:
        raise ValueError(f"{name} must be a positive integer")
    return value


def reference(ground_value, rotation_value, name):
    ground = finite_array(ground_value, (3,), name + " ground")
    basis = finite_array(rotation_value, (3, 3), name + " rotation")
    if np.max(np.abs(ground)) > MAX_COORDINATE_M:
        raise ValueError(f"{name} ground exceeds numeric safety limit")
    if (not np.allclose(basis.T @ basis, np.eye(3), rtol=0, atol=1e-8)
            or not np.isclose(np.linalg.det(basis), 1, rtol=0, atol=1e-8)):
        raise ValueError(f"{name} rotation is not proper orthogonal")
    return ground, basis


def load_original(session):
    folder = session / "original"
    yaml_path, stats_path, pgm_path = (folder / name for name in
                                      ("map.yaml", "projection_stats.json", "map.pgm"))
    yaml_bytes = yaml_path.read_bytes()
    stats_bytes = stats_path.read_bytes()
    meta = yaml.safe_load(yaml_bytes)
    stats = json.loads(stats_bytes)
    if (meta["image"] != "map.pgm" or meta["mode"] != "trinary" or meta["negate"] != 0
            or meta["resolution"] != RES or stats["resolution"] != RES):
        raise ValueError("Expected the original nonnegated 0.05m trinary PGM")
    free, occupied = float(meta["free_thresh"]), float(meta["occupied_thresh"])
    if not (1 / 255 < free < 50 / 255 < occupied < 1):
        raise ValueError("YAML thresholds do not preserve the three PGM cell classes")
    origin = finite_array(meta["origin"], (3,), "map origin")
    if origin[2] != 0 or np.max(np.abs(origin[:2])) > MAX_COORDINATE_M:
        raise ValueError("Expected an axis-aligned original map with bounded origin")
    lower = np.rint(origin[:2] / RES).astype(np.int64)
    if not np.allclose(lower * RES, origin[:2], rtol=0, atol=1e-8):
        raise ValueError("Original map origin is outside the 0.05m grid lattice")
    if (stats["frame"] != "native_projection" or stats["source_frame"] != "camera_init"
            or stats["min_height"] != 0.10 or stats["max_height"] != 1.00):
        raise ValueError("Unexpected original projection frame or height band")
    ground, basis = reference(stats["ground"], stats["rotation"], "Original projection")
    native_stamp = positive_int(stats["stamp_ns"], "Original projection stamp")
    h, w = (positive_int(stats[key], key) for key in ("height", "width"))
    if h * w > MAX_MAP_CELLS:
        raise ValueError("Original grid exceeds cell safety limit")
    original_digest = hashlib.sha256()
    with pgm_path.open("rb") as stream:
        header = f"P5\n{w} {h}\n255\n".encode("ascii")
        if stream.read(len(header)) != header:
            raise ValueError("Invalid original binary PGM header or geometry")
        payload = stream.read(h * w)
        if len(payload) != h * w or stream.read(1):
            raise ValueError("Original PGM payload length does not match its geometry")
        original_digest.update(header)
        original_digest.update(payload)
    original = np.frombuffer(payload, dtype=np.uint8).reshape(h, w)
    counts = np.bincount(original.ravel(), minlength=256)
    if any(counts[v] != 0 for v in set(range(256)) - {OCCUPIED, UNKNOWN, FREE}):
        raise ValueError("Original PGM contains pixels outside 0/205/254")
    original_counts = {}
    for value, name in ((OCCUPIED, "occupied"), (UNKNOWN, "unknown"), (FREE, "free")):
        original_counts[name] = int(counts[value])
        if original_counts[name] != stats[name + "_cells"]:
            raise ValueError(f"Original PGM and projection stats disagree on {name} cells")
    inputs = {str(pgm_path): original_digest.hexdigest(),
              str(yaml_path): hashlib.sha256(yaml_bytes).hexdigest(),
              str(stats_path): hashlib.sha256(stats_bytes).hexdigest()}
    return original, yaml_bytes, lower, stats, (ground, basis), native_stamp, original_counts, inputs


def load_recording(session, expected_reference):
    path = session / "recording.json"
    data = path.read_bytes()
    meta = json.loads(data)
    if meta["status"] != "complete" or meta["format"] != MAGIC.decode("ascii"):
        raise ValueError("Observation recording is incomplete or its format is unsupported")
    for key in ("errors", "pending_scan_stamps", "pending_pose_stamps"):
        if not isinstance(meta[key], list) or meta[key]:
            raise ValueError(f"Observation recording contains {key}")
    if not isinstance(meta["warnings"], list):
        raise ValueError("Invalid recorder warnings")
    for name in ("frames", "points", "bytes", "first_stamp_ns", "last_stamp_ns"):
        positive_int(meta[name], "Recording " + name)
    if meta["first_stamp_ns"] > meta["last_stamp_ns"]:
        raise ValueError("Invalid declared observation stamp range")
    for stream in ("scan", "pose"):
        received = positive_int(meta["stats"]["received_" + stream], f"Recorder {stream} receives")
        if received != meta["frames"]:
            raise ValueError(f"Recorder {stream} receives differ from declared complete pairs")
    ref = meta["reference"]
    if ref["frame_id"] != "camera_init":
        raise ValueError("Recording reference is outside camera_init")
    positive_int(ref["stamp_ns"], "Recording reference stamp")
    ground, basis = reference(ref["ground"], ref["rotation"], "Recording reference")
    ground_diff = float(np.max(np.abs(ground - expected_reference[0])))
    rotation_diff = float(np.max(np.abs(basis - expected_reference[1])))
    if ground_diff > REFERENCE_ATOL or rotation_diff > REFERENCE_ATOL:
        raise ValueError("Recording reference differs from the original projection reference")
    ref_check = {"reference_stamp_ns": ref["stamp_ns"], "absolute_tolerance": REFERENCE_ATOL,
                 "max_ground_difference_m": ground_diff, "max_rotation_element_difference": rotation_diff,
                 "computation_uses_original_projection_reference": True}
    return meta, ref_check, hashlib.sha256(data).hexdigest()


def stream_hits(path, meta, fixed_reference, lower, shape, native_stamp):
    if path.stat().st_size != meta["bytes"]:
        raise ValueError("Recorded observations byte length does not match recording metadata")
    digest = hashlib.sha256()
    tiles = HitTiles()
    h, w = shape
    frames = points = voxels = band_count = outside_count = later = 0
    first_stamp = last_stamp = None
    max_points = 0
    with path.open("rb") as stream:
        if stream.read(len(MAGIC)) != MAGIC:
            raise ValueError("Invalid observations binary signature")
        digest.update(MAGIC)
        while header := stream.read(FRAME.size):
            if len(header) != FRAME.size:
                raise ValueError("Truncated observation frame header")
            digest.update(header)
            stamp, n, px, py, pz, qx, qy, qz, qw = FRAME.unpack(header)
            if stamp <= 0 or (last_stamp is not None and stamp <= last_stamp):
                raise ValueError(f"Non-increasing observation stamp at frame {frames + 1}")
            if not 1 <= n <= MAX_FRAME_POINTS or frames >= meta["frames"] or points + n > meta["points"]:
                raise ValueError(f"Unsafe or undeclared point count at frame {frames + 1}")
            position = finite_array((px, py, pz), (3,), "optical position")
            quat = finite_array((qx, qy, qz, qw), (4,), "optical quaternion")
            norm = np.linalg.norm(quat)
            if not np.isfinite(norm) or norm == 0 or np.max(np.abs(position)) > MAX_COORDINATE_M:
                raise ValueError(f"Invalid observation pose at frame {frames + 1}")
            block = stream.read(n * 12)
            if len(block) != n * 12:
                raise ValueError(f"Truncated observation point payload at frame {frames + 1}")
            digest.update(block)
            cloud = np.frombuffer(block, dtype="<f4").reshape(n, 3)
            if not np.isfinite(cloud).all() or np.max(np.abs(cloud)) > MAX_COORDINATE_M:
                raise ValueError(f"Nonfinite or unsafe point coordinates at frame {frames + 1}")
            band, retained = selected_band(cloud, position, Rotation.from_quat(quat).as_matrix(),
                                           *fixed_reference)
            cells = np.floor(band[:, :2] / RES).astype(np.int64)
            local = cells - lower
            in_bounds = ((local[:, 0] >= 0) & (local[:, 0] < w)
                         & (local[:, 1] >= 0) & (local[:, 1] < h))
            tiles.add_cells(cells[in_bounds])
            frames += 1
            points += n
            voxels += retained
            band_count += len(band)
            outside_count += len(band) - int(in_bounds.sum())
            later += stamp > native_stamp
            max_points = max(max_points, n)
            if first_stamp is None:
                first_stamp = stamp
            last_stamp = stamp
        actual = {"frames": frames, "points": points, "bytes": stream.tell(),
                  "sha256": digest.hexdigest(), "first_stamp_ns": first_stamp,
                  "last_stamp_ns": last_stamp}
    for key, value in actual.items():
        if meta[key] != value:
            raise ValueError(f"Recorded observations {key} does not match recording metadata")
    actual.update(first_return_voxels=voxels, height_band_points=band_count,
                  height_band_points_outside_final_grid=outside_count,
                  frames_after_final_native_stamp=later, max_frame_points=max_points,
                  tiles=len(tiles.tiles), tile_bit_payload_bytes=sum(x.nbytes for x in tiles.tiles.values()))
    return tiles, actual


def write_map(folder, candidate, yaml_bytes):
    folder.mkdir()
    h, w = candidate.shape
    pgm = folder / "map.pgm"
    header = f"P5\n{w} {h}\n255\n".encode("ascii")
    with pgm.open("wb") as output:
        output.write(header)
        output.write(candidate.tobytes())
    (folder / "map.yaml").write_bytes(yaml_bytes)
    with pgm.open("rb") as stream:
        if stream.read(len(header)) != header:
            raise IOError("Staged candidate PGM header verification failed")
        payload = np.frombuffer(stream.read(), dtype=np.uint8)
        if not np.array_equal(payload, candidate.ravel()):
            raise IOError("Staged candidate PGM pixel verification failed")
    if (folder / "map.yaml").read_bytes() != yaml_bytes:
        raise IOError("Staged candidate YAML byte verification failed")


def write_changes(path, mask, lower):
    rows = 0
    h = mask.shape[0]
    with path.open("w", newline="") as output:
        writer = csv.writer(output)
        writer.writerow(["column", "row", "world_grid_x", "world_grid_y",
                         "center_x_m", "center_y_m", "before", "after", "reason"])
        for row in np.flatnonzero(mask.any(axis=1)):
            iy = int(lower[1]) + h - 1 - int(row)
            for col in np.flatnonzero(mask[row]):
                ix = int(lower[0]) + int(col)
                writer.writerow([int(col), int(row), ix, iy,
                                 f"{(ix + .5) * RES:.3f}", f"{(iy + .5) * RES:.3f}",
                                 UNKNOWN, OCCUPIED, REASON])
                rows += 1
    return rows


def comparison_image(path, original, candidate, changes):
    h, w = original.shape
    informative = candidate != UNKNOWN
    rows = np.flatnonzero(informative.any(axis=1))
    cols = np.flatnonzero(informative.any(axis=0))
    y0, y1 = (max(0, int(rows[0]) - 12), min(h, int(rows[-1]) + 13)) if len(rows) else (0, h)
    x0, x1 = (max(0, int(cols[0]) - 12), min(w, int(cols[-1]) + 13)) if len(cols) else (0, w)
    height, width = y1 - y0, x1 - x0
    scale = min(2, 740 / max(height, width))
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    nearest = getattr(Image, "Resampling", Image).NEAREST
    source_view = Image.fromarray(original[y0:y1, x0:x1]).resize(size, nearest).convert("RGB")
    candidate_view = Image.fromarray(candidate[y0:y1, x0:x1]).resize(size, nearest).convert("RGB")
    diff_view = np.array(source_view)
    ys, xs = np.nonzero(changes[y0:y1, x0:x1])
    # Pool the marker by display pixel so sparse changes remain visible at reduced scale.
    dy = np.minimum((ys * size[1] // height), size[1] - 1)
    dx = np.minimum((xs * size[0] // width), size[0] - 1)
    diff_view[dy, dx] = (255, 117, 24)
    panel_w = max(size[0], 135)
    margin, header = 12, 26
    canvas = Image.new("RGB", (3 * panel_w + 4 * margin, size[1] + header + margin), (34, 39, 45))
    draw = ImageDraw.Draw(canvas)
    for index, (label, picture) in enumerate([
            ("Original map", source_view), ("A005 candidate", candidate_view),
            ("Orange: added cells", Image.fromarray(diff_view))]):
        x = margin + index * (panel_w + margin)
        draw.text((x, 7), label, fill=(245, 245, 245))
        canvas.paste(picture, (x, header))
    canvas.save(path, format="PNG")
    with Image.open(path) as verify:
        verify.verify()
    return {"crop_col_row_bounds_half_open": [x0, y0, x1, y1],
            "panel_display_width_height": list(size), "difference_rgb": [255, 117, 24]}


def json_text(data):
    return json.dumps(data, indent=2, ensure_ascii=False, allow_nan=False) + "\n"


def write_failed_report(session, phase, error, started):
    report = {"status": "failed", "session": str(session), "stage": phase,
              "error_type": type(error).__name__, "error": str(error),
              "reported_utc": now_utc(), "elapsed_s": time.perf_counter() - started}
    with tempfile.TemporaryDirectory(dir=session, prefix=".offline-failure-") as scratch:
        path = Path(scratch) / "report.json"
        path.write_text(json_text(report))
        os.replace(path, session / "report.json")


def check_outputs(session):
    for name in ("enhanced", "comparison.png", "changes.csv"):
        if exists(session / name):
            raise ExistingOutputError(f"Refusing to overwrite {session / name}")
    report = session / "report.json"
    if exists(report):
        if report.is_symlink() or not report.is_file() or json.loads(report.read_text()).get("status") != "failed":
            raise ExistingOutputError(f"Refusing to overwrite {report}")


def enhance(session):
    started = time.perf_counter()
    lock = session / ".offline_enhance.lock"
    try:
        descriptor = os.open(lock, os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
    except FileExistsError as error:
        raise ExistingOutputError(f"Another enhancement holds {lock}") from error
    os.close(descriptor)
    phase = "check_outputs"
    installed_sidecars = []
    try:
        check_outputs(session)
        phase = "validate_inputs"
        original, yaml_bytes, lower, stats, fixed_ref, native_stamp, counts, inputs = load_original(session)
        meta, reference_check, metadata_sha = load_recording(session, fixed_ref)
        inputs[str(session / "recording.json")] = metadata_sha
        validated_at = time.perf_counter()
        phase = "stream_observations"
        tiles, observed = stream_hits(session / "observations.bin", meta, fixed_ref, lower,
                                      original.shape, native_stamp)
        streamed_at = time.perf_counter()
        phase = "verify_derivative"
        candidate, additions, hits = tiles.compose(original, lower)
        added = int(additions.sum())
        checks = {"only_original_unknown_to_occupied": bool(
                    np.array_equal(candidate != original, additions)
                    and np.all(original[additions] == UNKNOWN)
                    and np.all(candidate[additions] == OCCUPIED)),
                  "original_occupied_retained": bool(np.all(candidate[original == OCCUPIED] == OCCUPIED)),
                  "original_free_mask_unchanged": bool(np.array_equal(candidate == FREE, original == FREE)),
                  "new_free_cells": int(((candidate == FREE) & (original != FREE)).sum())}
        if not all(checks[name] for name in ("only_original_unknown_to_occupied",
                                              "original_occupied_retained", "original_free_mask_unchanged")):
            raise AssertionError("Candidate violated trinary transition invariants")
        derived_at = time.perf_counter()
        phase = "stage_outputs"
        with tempfile.TemporaryDirectory(dir=session, prefix=".offline-stage-",
                                         ignore_cleanup_errors=True) as scratch:
            staging = Path(scratch)
            write_map(staging / "enhanced", candidate, yaml_bytes)
            if write_changes(staging / "changes.csv", additions, lower) != added:
                raise AssertionError("Change CSV disagrees with exact candidate difference")
            preview = comparison_image(staging / "comparison.png", original, candidate, additions)
            if any(sha(Path(path)) != digest for path, digest in inputs.items()):
                raise RuntimeError("Original map or recording metadata changed during processing")
            phase = "commit_outputs"
            original_obs_frames = stats.get("free_observation_frames")
            output_pgm = staging / "enhanced" / "map.pgm"
            report = {
                "status": "complete", "session": str(session), "completed_utc": now_utc(),
                "algorithm": "A005_first_return_sensor_voxel_then_fixed_height_band_hit_union",
                "resolution_m": RES, "sensor_frame_voxel_m": RES,
                "height_band_m_with_tolerance": [LOW, HIGH],
                "max_frame_points_allowed": MAX_FRAME_POINTS,
                "reference_check": reference_check,
                "original_inputs_sha256": inputs,
                "candidate_pgm_sha256": sha(output_pgm),
                "candidate_yaml_sha256": sha(staging / "enhanced" / "map.yaml"),
                "yaml_exact_original_bytes": True, "added_cells": added,
                "original_counts": counts,
                "candidate_counts": {"occupied": counts["occupied"] + added,
                                     "unknown": counts["unknown"] - added, "free": counts["free"]},
                "observations": observed, "recording_warnings": meta["warnings"],
                "recording_received": meta["stats"],
                "original_native_map_stamp_ns": native_stamp,
                "original_projector_free_observation_stamp_ns": stats.get("free_observation_stamp_ns"),
                "original_projector_free_observation_frames": original_obs_frames,
                "recorded_pairs_minus_original_projector_free_frames": (
                    observed["frames"] - original_obs_frames if type(original_obs_frames) is int else None),
                "last_recorded_stamp_minus_final_native_ns": observed["last_stamp_ns"] - native_stamp,
                "original_value_of_in_grid_hit_cells": {
                    str(v): int((hits & (original == v)).sum()) for v in (OCCUPIED, UNKNOWN, FREE)},
                "transition_checks": checks, "comparison_preview": preview,
                "timing_seconds": {
                    "input_validation": validated_at - started,
                    "stream_read_hash_and_accumulation": streamed_at - validated_at,
                    "candidate_compose_and_checks": derived_at - streamed_at,
                    "stage_outputs_and_input_recheck": time.perf_counter() - derived_at,
                    "elapsed_before_commit": time.perf_counter() - started,
                },
                "process_peak_rss_kib_linux": resource.getrusage(resource.RUSAGE_SELF).ru_maxrss,
                "measurement_limits": [
                    "Recorded observations are independently paired by the recorder and may differ from original projector consumption.",
                    "Complete recording means no detected mismatch in the saved streams, without a guarantee of DDS delivery.",
                    "RSS covers this process and its runtime, without measuring whole-system memory.",
                    "Added cells are measured endpoint candidates, without physical object labels or navigation validation.",
                ],
            }
            (staging / "report.json").write_text(json_text(report))
            (staging / "report.pending.json").write_text(json_text({
                "status": "failed", "session": str(session), "stage": "commit_outputs",
                "error_type": "CommitIncomplete", "error": "Final report commit did not complete",
                "reported_utc": now_utc(),
            }))
            check_outputs(session)
            for name in ("comparison.png", "changes.csv"):
                target = session / name
                os.link(staging / name, target)
                installed_sidecars.append(target)
            os.replace(staging / "report.pending.json", session / "report.json")
            if exists(session / "enhanced"):
                raise ExistingOutputError("Enhanced map appeared while preparing outputs")
            os.rename(staging / "enhanced", session / "enhanced")
            os.replace(staging / "report.json", session / "report.json")
    except (Exception, KeyboardInterrupt) as error:
        # The failed marker remains if the verified candidate directory exists
        # but committing its complete report was interrupted.
        if not exists(session / "enhanced"):
            for target in installed_sidecars:
                target.unlink(missing_ok=True)
            if not isinstance(error, ExistingOutputError) or installed_sidecars:
                write_failed_report(session, phase, error, started)
        raise
    finally:
        lock.unlink(missing_ok=True)
    return session / "enhanced" / "map.yaml"


def allowed_session(argument):
    if not argument.is_absolute():
        raise ValueError("--session must be an absolute robot session path")
    path = argument.resolve(strict=True)
    allowed = any(path != root and path.is_relative_to(root)
                  for root in (LIVE_ROOT.resolve(), FIXTURE_ROOT.resolve()))
    if not allowed or not path.is_dir():
        raise ValueError("--session must be under the dedicated live-session or verification root")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    args = parser.parse_args()
    try:
        candidate = enhance(allowed_session(args.session))
    except (Exception, KeyboardInterrupt) as error:
        print(f"OFFLINE_ENHANCEMENT_FAILED {type(error).__name__}: {error}", file=sys.stderr)
        return 1
    try:
        print(f"ENHANCED_MAP_SAVED {candidate}", flush=True)
    except BrokenPipeError:
        # Artifacts are committed even when the caller's terminal disappears.
        sys.stdout = open(os.devnull, "w")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
