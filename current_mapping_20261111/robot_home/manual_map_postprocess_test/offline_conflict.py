#!/usr/bin/env python3
"""Build the bounded Free-conflict v1 map from a verified A005 checkpoint."""
import argparse
import csv
import ctypes
import hashlib
import json
import math
import os
from pathlib import Path
import platform
import resource
import shutil
import signal
import struct
import sys
import tempfile
import time

import numpy as np

from conflict_verify import (
    ALGORITHM, HERE, MAGIC, MAX_COORD, OUTPUT_NAME, RES, finite, load_conflict_source,
    positive_int, read_pgm, require, session_file, sha, source_hashes,
    verify_uncalibrated_maps,
)


LIVE_ROOT = Path("/home/wheeltec/maps/manual_postprocess_test")
FIXTURE_ROOT = HERE / "verification"
FRAME = struct.Struct("<qI7d")
MAX_POINTS = 200_000
BATCH_CELLS = 4096
MAX_TILES = 1024
MAX_CANDIDATES = 1_000_000
DEFAULT_MAX_SECONDS = 600.


def dump(path, value):
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n",
                    encoding="utf-8")


def rotation(quaternion):
    q = finite(quaternion, (4,), "quaternion")
    norm = np.linalg.norm(q)
    require(np.isfinite(norm) and norm > 0, "Zero/nonfinite quaternion")
    x, y, z, w = q / norm
    # Same normalized XYZW arithmetic as the existing independent reference.
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


def selected(all_local, raw):
    voxels = np.floor(raw.astype(np.float64) / RES).astype(np.int64)
    _, first = np.unique(voxels, axis=0, return_index=True)
    first = np.sort(first)
    z = all_local[first, 2]
    first = first[(z >= .1 - 1e-6) & (z <= 1.0 + 1e-6)]
    return all_local[first], first


def grid_indices(points, lower, shape):
    cells = np.floor(points[:, :2] / RES).astype(np.int64)
    xy = cells - lower
    h, w = shape
    valid = (xy[:, 0] >= 0) & (xy[:, 0] < w) & (xy[:, 1] >= 0) & (xy[:, 1] < h)
    flat = (h - 1 - xy[:, 1]) * w + xy[:, 0]
    return flat, valid, cells


def write_pgm(path, grid):
    h, w = grid.shape
    with Path(path).open("xb") as f:
        f.write(f"P5\n{w} {h}\n255\n".encode("ascii"))
        for row in grid:
            f.write(row.tobytes())


class Inputs:
    def __init__(self, session):
        self.session = Path(session).resolve(strict=True)
        self.a005, self.hashes, self.code = load_conflict_source(self.session)
        source = self.a005
        s, m = source.stats, source.recording
        self.recording = m
        self.yaml = source.yaml
        self.lower = source.lower
        self.original = source.original
        self.baseline = source.baseline
        self.shape = self.original.shape
        self.ground = source.ground
        self.basis = source.basis
        self.observation_path = session_file(self.session, "observations.bin")
        require(sha(self.observation_path) == self.hashes["observations.bin"], "Observation hash mismatch")
        native = positive_int(s["stamp_ns"], "native stamp")
        free = positive_int(s["free_observation_stamp_ns"], "free observation stamp")
        with np.load(session_file(self.session, "original/free_evidence.npz"), allow_pickle=False) as z:
            require(z["map_stamp_ns"].shape == z["observation_stamp_ns"].shape == () and z["map_stamp_ns"].dtype.kind in 'iu' and z["observation_stamp_ns"].dtype.kind in 'iu', "Invalid free evidence timestamp types")
            require(int(z["map_stamp_ns"]) == native and int(z["observation_stamp_ns"]) == free, "Free evidence timestamp mismatch")
            require(float(z["resolution"]) == RES and float(z["low"]) == .1 and float(z["high"]) == 1. and float(z["height_step"]) == .1, "Free evidence grid mismatch")
            xy, masks = z["xy"], z["masks"]
            require(xy.ndim == 2 and xy.shape[1] == 2 and xy.dtype.kind in 'iu' and masks.shape == (len(xy),) and masks.dtype.kind in 'iu' and np.all((masks >= 1) & (masks <= 511)), "Invalid free evidence masks")
            require(len(np.unique(xy,axis=0)) == len(xy), 'Duplicate free evidence coordinates')
            relative = xy.astype(np.int64)-self.lower
            require(np.all((relative >= 0) & (relative < np.array(self.shape[::-1]))), 'Free evidence outside map bounds')
            fr,fc=self.shape[0]-1-relative[:,1],relative[:,0]
            free_full=masks == 511
            require(np.all(self.original[fr[free_full],fc[free_full]] == 254) and int(free_full.sum()) == s['free_cells'], 'Full free masks disagree with original Free cells')
        self.t_start = m["first_stamp_ns"]
        self.t_end = min(native, free, m["last_stamp_ns"])
        require(0 < self.t_start <= self.t_end <= m["last_stamp_ns"], "Empty/invalid common time window")

    def recheck(self):
        require(all(sha(session_file(self.session, name)) == digest for name, digest in self.hashes.items()),
                "Source changed during conflict processing")
        require(self.code == source_hashes(), "Worker code changed during conflict processing")

    def frames(self, deadline=None):
        m = self.recording
        digest = hashlib.sha256()
        frames = points = inside = 0
        first_stamp = last_stamp = None
        with self.observation_path.open("rb") as f:
            magic = f.read(8)
            require(magic == MAGIC, "Bad recording magic")
            digest.update(magic)
            while True:
                if deadline is not None and time.monotonic() > deadline:
                    raise TimeoutError("Conflict time budget exceeded")
                offset = f.tell()
                header = f.read(FRAME.size)
                if not header:
                    break
                require(len(header) == FRAME.size, "Truncated frame header")
                digest.update(header)
                stamp,n,*pose = FRAME.unpack(header)
                require(stamp > 0 and (last_stamp is None or stamp > last_stamp), "Non-increasing frame timestamp")
                require(1 <= n <= MAX_POINTS and frames < m["frames"] and points+n <= m["points"], "Frame point/count limit exceeded")
                pos = finite(pose[:3], (3,), "frame position")
                rot = rotation(pose[3:])
                payload = f.read(n*12)
                require(len(payload) == n*12, "Truncated frame points")
                digest.update(payload)
                raw = np.frombuffer(payload,dtype="<f4").reshape(n,3)
                require(np.isfinite(raw).all() and np.max(np.abs(raw)) <= MAX_COORD, "Nonfinite/unsafe points")
                frames += 1
                points += n
                if first_stamp is None:
                    first_stamp = stamp
                last_stamp = stamp
                if self.t_start <= stamp <= self.t_end:
                    inside += 1
                    # Preserve reference arithmetic order before sensor sampling.
                    local = (raw.astype(np.float64) @ rot.T + pos - self.ground) @ self.basis
                    sensor = (pos-self.ground) @ self.basis
                    yield frames-1,offset,stamp,raw,local,sensor
            actual = dict(frames=frames,points=points,bytes=f.tell(),sha256=digest.hexdigest(),first_stamp_ns=first_stamp,last_stamp_ns=last_stamp)
        require(inside > 0, "No frames in common time window")
        require(all(actual[k] == m[k] for k in actual), "Full-stream metadata/SHA mismatch, including tail")
        self.last_pass = dict(actual,frames_in_window=inside,frames_excluded=frames-inside)


class SaturatingTiles:
    """P1 exact frame-presence threshold, uint8 saturation deliberately at three."""
    def __init__(self, max_tiles=1024):
        self.max_tiles = max_tiles
        self.tiles = {}

    def update(self, world_cells):
        cells = np.unique(world_cells, axis=0)
        if not len(cells):
            return
        tiles, inverse = np.unique(cells//64,axis=0,return_inverse=True)
        for j, key_arr in enumerate(tiles):
            key = tuple(map(int,key_arr))
            if key not in self.tiles:
                require(len(self.tiles) < self.max_tiles, "P1 tile budget exceeded")
                self.tiles[key] = np.zeros(4096,np.uint8)
            local = cells[inverse == j] - key_arr*64
            ids = local[:,1]*64+local[:,0]
            arr = self.tiles[key]
            arr[ids] = np.minimum(arr[ids]+1,3)

    def export(self, stage, inputs, max_candidates):
        n = weak = 0
        h,w = inputs.shape
        with (stage/"candidate_indices.u32").open("xb") as f, (stage/"excluded_hits.csv").open("x",newline="",encoding="utf-8") as e:
            writer = csv.writer(e)
            writer.writerow(("pgm_col","pgm_row","support_frames","reason"))
            for (tx,ty),counts in sorted(self.tiles.items()):
                idx = np.flatnonzero(counts)
                cols = tx*64+idx%64-inputs.lower[0]
                rows = h-1-(ty*64+idx//64-inputs.lower[1])
                take = counts[idx] == 3
                out = (rows[take]*w+cols[take]).astype("<u4")
                n += len(out)
                require(n <= max_candidates, "Candidate index budget exceeded")
                f.write(out.tobytes())
                for col,row,s in zip(cols[~take],rows[~take],counts[idx][~take]):
                    writer.writerow((int(col),int(row),int(s),"one_or_two_hits_outside_frozen_selection"))
                    weak += 1
        result = dict(selected_cells=n,one_or_two_hit_cells=weak,hit_free_cells=n+weak,tiles=len(self.tiles),tile_numeric_payload_bytes=len(self.tiles)*4112)
        self.tiles.clear()
        if n:
            index = np.memmap(stage/"candidate_indices.u32",dtype="<u4",mode="r+")
            index.sort(kind="quicksort")
            index.flush()
            del index
        return result


def exclusive_rename(source, target):
    """Atomic no-replace directory commit; fail closed on unsupported platforms."""
    libc = ctypes.CDLL(None,use_errno=True)
    if sys.platform == "darwin":
        fn = libc.renamex_np
        fn.argtypes = (ctypes.c_char_p,ctypes.c_char_p,ctypes.c_uint)
        result = fn(os.fsencode(source),os.fsencode(target),4)  # RENAME_EXCL
    elif sys.platform.startswith("linux"):
        fn = libc.renameat2
        fn.argtypes = (ctypes.c_int,ctypes.c_char_p,ctypes.c_int,ctypes.c_char_p,ctypes.c_uint)
        result = fn(-100,os.fsencode(source),-100,os.fsencode(target),1)  # RENAME_NOREPLACE
    else:
        raise RuntimeError("No tested exclusive directory rename on this platform")
    if result != 0:
        e = ctypes.get_errno()
        raise OSError(e,os.strerror(e),str(target))


class Transaction:
    def __init__(self, session):
        self.output = session / OUTPUT_NAME
        require(not os.path.lexists(self.output), "Conflict output already exists")
        self.lock = session / f".{OUTPUT_NAME}.lock"
        fd = os.open(self.lock, os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        self.stage = None
        try:
            try:
                st = os.fstat(fd)
                self.lock_identity = st.st_dev, st.st_ino
                os.write(fd, f"{os.getpid()}\n".encode("ascii"))
            finally:
                os.close(fd)
            self.stage = Path(tempfile.mkdtemp(prefix=f".{OUTPUT_NAME}.incomplete-", dir=session))
        except BaseException:
            self.close()
            raise

    def commit(self):
        exclusive_rename(self.stage, self.output)

    def close(self):
        try:
            if self.stage is not None and self.stage.is_dir():
                shutil.rmtree(self.stage)
        finally:
            try:
                st = self.lock.lstat()
                if (st.st_dev, st.st_ino) == self.lock_identity:
                    self.lock.unlink()
            except FileNotFoundError:
                pass


def peak_rss_bytes():
    n = resource.getrusage(resource.RUSAGE_SELF).ru_maxrss
    return int(n if sys.platform == "darwin" else n*1024)


def batch_hits(band, raw_indices, inputs, keys):
    flat,valid,_ = grid_indices(band,inputs.lower,inputs.shape)
    pos = np.searchsorted(keys,flat)
    in_batch = valid & (pos < len(keys))
    accepted = np.flatnonzero(in_batch)
    accepted = accepted[keys[pos[accepted]] == flat[accepted]]
    return pos[accepted],band[accepted],raw_indices[accepted]


def resolve_reference(stream, inputs, offset, point_index):
    """Reconstruct a bounded witness from its actual immutable frame and raw point."""
    require(8 <= offset <= inputs.recording["bytes"]-FRAME.size, "Invalid witness offset")
    stream.seek(offset)
    header = stream.read(FRAME.size)
    require(len(header) == FRAME.size, "Truncated witness frame")
    stamp,n,*pose = FRAME.unpack(header)
    require(inputs.t_start <= stamp <= inputs.t_end and 0 <= point_index < n <= MAX_POINTS, "Invalid witness index/time")
    stream.seek(offset+FRAME.size+point_index*12)
    raw = np.frombuffer(stream.read(12),dtype='<f4').astype(np.float64)
    require(raw.shape == (3,), "Truncated witness point")
    pos = np.asarray(pose[:3])
    local = (raw @ rotation(pose[3:]).T + pos-inputs.ground) @ inputs.basis
    sensor = (pos-inputs.ground) @ inputs.basis
    return {"frame_offset":int(offset),"raw_point_index":int(point_index),"stamp_ns":int(stamp),
            "xyz":local.tolist(),"sensor_xyz":sensor.tolist()}


def algorithm_run(session, max_seconds, tx):
    from conflict_core import PositiveAccumulator, DEFAULT_POSITIVE_PARAMS, classify_cell
    from ray_geometry import RayDiagnostics, DEFAULT_RAY_PARAMS, describe_proxy_witness

    started = time.monotonic()
    deadline = started + max_seconds
    stage = tx.stage
    inputs = Inputs(session)
    params = dict(version=ALGORITHM, selection_frames=3, resolution_m=RES,
                  sensor_first_return_voxel_m=RES, height_band_with_tolerance=[.099999, 1.000001],
                  positive=DEFAULT_POSITIVE_PARAMS, ray=DEFAULT_RAY_PARAMS, calibration_status="uncalibrated",
                  ground_label_exception_enabled=False, automatic_deployment=False,
                  batch_cells=BATCH_CELLS, max_tiles=MAX_TILES, max_candidates=MAX_CANDIDATES,
                  max_seconds=max_seconds)
    dump(stage / "parameters.json", params)
    require(shutil.disk_usage(session).free >= (1 << 30) + inputs.original.size * 10,
            "Insufficient disk reserve")
    timing = {"input_validation":time.monotonic()-started}
    t=time.monotonic()
    tiles=SaturatingTiles(MAX_TILES)
    for frame_id,offset,stamp,raw,local,sensor in inputs.frames(deadline):
        band,_=selected(local,raw)
        flat,valid,cells=grid_indices(band,inputs.lower,inputs.shape)
        pick=np.flatnonzero(valid)
        pick=pick[inputs.original.ravel()[flat[pick]] == 254]
        tiles.update(cells[pick])
    selection_stats=tiles.export(stage,inputs,MAX_CANDIDATES)
    p1_pass=inputs.last_pass.copy()
    del tiles
    timing["P1_selection"]=time.monotonic()-t
    n=selection_stats["selected_cells"]
    require(shutil.disk_usage(session).free >= (1<<30)+n*16000+inputs.original.size*10, "Insufficient staging output disk budget")
    keys_all=np.memmap(stage/"candidate_indices.u32",dtype='<u4',mode='r') if n else np.empty(0,dtype='<u4')
    selection=np.zeros(inputs.shape,dtype=bool)
    selection.ravel()[keys_all]=True
    candidate=inputs.baseline.copy()
    positive_mask=np.zeros(inputs.shape,dtype=np.uint8)
    (stage/"batches").mkdir()
    columns=["pgm_col","pgm_row","x_m_native_projection","y_m_native_projection","A_pgm","candidate_pgm",
             "support_frames","cluster_count","nominal_positive_witness","qualified_positive","cell_flags","reason",
             "ray_assessed","credible_conflict","proxy_near_count","proxy_below_count","proxy_unassociated_count"]
    batch_reports=[]
    reasons={}
    total_nominal=0
    with (stage/"cells.csv").open('x',newline='',encoding='utf-8') as cell_file, (stage/"cluster_witnesses.jsonl").open('x',encoding='utf-8') as witness_file:
        writer=csv.DictWriter(cell_file,fieldnames=columns)
        writer.writeheader()
        for bnum,start in enumerate(range(0,n,BATCH_CELLS)):
            keys=np.asarray(keys_all[start:start+BATCH_CELLS],dtype=np.int64)
            rows,cols=np.divmod(keys,inputs.shape[1])
            cell_xy=(np.column_stack((cols,inputs.shape[0]-1-rows))+inputs.lower+.5)*RES
            positive=PositiveAccumulator(cell_xy,params["positive"])
            t=time.monotonic()
            for fid,offset,stamp,raw,local,sensor in inputs.frames(deadline):
                band,raw_indices=selected(local,raw)
                ids,points,ri=batch_hits(band,raw_indices,inputs,keys)
                positive.update_frame(ids,points,ri,sensor,stamp,fid,offset)
            snap=positive.finalize()
            positive_seconds=time.monotonic()-t
            require(np.all(snap['cell_support'] >= 3), "P1/P2 support inconsistency")
            # Recover exact float64 support coordinates before ray comparisons; compact cache remains in core.
            exact_representatives=np.full(snap['representatives'].shape,np.nan,dtype=np.float64)
            with inputs.observation_path.open("rb") as refstream:
                for ci,cl,ri in np.argwhere(np.isfinite(snap['representatives']).all(axis=-1)):
                    ref=resolve_reference(refstream,inputs,int(snap['representative_offsets'][ci,cl,ri]),int(snap['representative_indices'][ci,cl,ri]))
                    exact_representatives[ci,cl,ri]=ref['xyz']
            ray_snapshot=dict(snap,representatives=exact_representatives)
            rays=RayDiagnostics(ray_snapshot,params["ray"])
            t=time.monotonic()
            for fid,offset,stamp,raw,local,sensor in inputs.frames(deadline):
                band,raw_indices=selected(local,raw)
                ids,points,_=batch_hits(band,raw_indices,inputs,keys)
                rays.update_frame(local,sensor,stamp,fid,offset,positive_cell_ids=ids,positive_points=points)
            neg=rays.finalize()
            ray_seconds=time.monotonic()-t
            with inputs.observation_path.open("rb") as refstream:
                for j,(row,col) in enumerate(zip(rows,cols)):
                    pixel,reason=classify_cell(snap['cluster_active'][j],snap['positive_witness'][j],snap['qualified_positive'][j],
                                               snap['cluster_flags'][j],snap['cell_flags'][j],
                                               neg['assessed'][j],neg['credible_conflict'][j],
                                               calibration_valid=False)
                    candidate[row,col]=pixel
                    nominal=bool(np.any(snap['positive_witness'][j]))
                    positive_mask[row,col]=255 if nominal else 0
                    total_nominal+=int(nominal)
                    reasons[reason]=reasons.get(reason,0)+1
                    writer.writerow(dict(pgm_col=int(col),pgm_row=int(row),x_m_native_projection=float(cell_xy[j,0]),
                                         y_m_native_projection=float(cell_xy[j,1]),A_pgm=254,candidate_pgm=int(pixel),
                                         support_frames=int(snap['cell_support'][j]),cluster_count=int(snap['cluster_active'][j].sum()),
                                         nominal_positive_witness=int(nominal),qualified_positive=int(np.any(snap['qualified_positive'][j])),
                                         cell_flags=int(snap['cell_flags'][j]),reason=reason,
                                         ray_assessed=int(np.any(neg['assessed'][j])),credible_conflict=int(np.any(neg['credible_conflict'][j])),
                                         proxy_near_count=int(neg['proxy_near_count'][j].sum()),proxy_below_count=int(neg['proxy_below_count'][j].sum()),
                                         proxy_unassociated_count=int(neg['proxy_unassociated_count'][j].sum())))
                    for cl in np.flatnonzero(snap['cluster_active'][j]):
                        witnesses=[]
                        for ri in range(4):
                            if np.isfinite(snap['representatives'][j,cl,ri]).all():
                                witnesses.append(resolve_reference(refstream,inputs,int(snap['representative_offsets'][j,cl,ri]),int(snap['representative_indices'][j,cl,ri])))
                        rw=None
                        ray_offset=int(neg['proxy_frame_offset'][j,cl])
                        if ray_offset >= 8 and ray_offset < inputs.recording['bytes']:
                            rw=resolve_reference(refstream,inputs,ray_offset,int(neg['proxy_point_index'][j,cl]))
                            rw['geometry']=describe_proxy_witness(np.asarray(rw['sensor_xyz']),np.asarray(rw['xyz']),snap['xyz_min'][j,cl],snap['xyz_max'][j,cl],exact_representatives[j,cl],params['ray'])
                        record=dict(pgm_col=int(col),pgm_row=int(row),cluster=int(cl),support_frames=int(snap['cluster_support'][j,cl]),
                                    nominal_positive_witness=bool(snap['positive_witness'][j,cl]),qualified_positive=bool(snap['qualified_positive'][j,cl]),
                                    station_counts=snap['station_counts'][j,cl].tolist(),flags=int(snap['cluster_flags'][j,cl]),
                                    xyz_min=snap['xyz_min'][j,cl].tolist(),xyz_max=snap['xyz_max'][j,cl].tolist(),
                                    positive_witnesses=witnesses,ray_proxy_witness=rw,ray_assessed=bool(neg['assessed'][j,cl]),
                                    credible_conflict=bool(neg['credible_conflict'][j,cl]),warning="scan-end proxy; uncalibrated; representatives are not a solid surface")
                        witness_file.write(json.dumps(record,ensure_ascii=False,allow_nan=False)+'\n')
            numeric_neg={k:v for k,v in neg.items() if isinstance(v,np.ndarray)}
            np.savez_compressed(stage/'batches'/f'{bnum:04d}.npz',cell_flat_indices=keys,
                                **{f'positive_{k}':v for k,v in snap.items() if isinstance(v,np.ndarray)},
                                **{f'ray_{k}':v for k,v in numeric_neg.items()})
            metrics=dict(batch=bnum,cells=len(keys),P2_seconds=positive_seconds,P3_seconds=ray_seconds,
                         positive_payload_bytes=int(positive.payload_bytes),exact_representative_cache_bytes=exact_representatives.nbytes,
                         ray_diagnostics={k:v for k,v in neg.items() if not isinstance(v,np.ndarray)},
                         ray_engine_stats=rays.stats,
                         proxy_near_pairs=int(neg['proxy_near_count'].sum()),proxy_below_pairs=int(neg['proxy_below_count'].sum()),
                         assessed_clusters=int(neg['assessed'].sum()),nominal_positive_clusters=int(snap['positive_witness'].sum()))
            batch_reports.append(metrics)
            print(json.dumps(dict(phase="batch_complete",**metrics),ensure_ascii=False),flush=True)
            del positive,snap,ray_snapshot,exact_representatives,rays,neg,numeric_neg
    write_pgm(stage / "map.pgm", candidate)
    (stage / "map.yaml").write_bytes(inputs.yaml)
    write_pgm(stage / "selection_mask.pgm", selection.astype(np.uint8) * 255)
    write_pgm(stage / "positive_only_mask.pgm", positive_mask)
    saved = read_pgm(stage / "map.pgm", inputs.shape)
    saved_selection = read_pgm(stage / "selection_mask.pgm", inputs.shape, (0, 255)) == 255
    require(np.array_equal(saved, candidate) and np.array_equal(saved_selection, selection),
            "Staged conflict map or selection differs from the computed pixels")
    verification = verify_uncalibrated_maps(inputs.a005, saved, saved_selection,
                                           (stage / "map.yaml").read_bytes(), stage / "cells.csv")
    artifacts = {p.relative_to(stage).as_posix(): sha(p) for p in stage.rglob("*") if p.is_file()}
    t = time.monotonic()
    inputs.recheck()
    timing["source_recheck"] = time.monotonic() - t
    timing["elapsed_before_commit"] = time.monotonic() - started
    require(time.monotonic() <= deadline, "Time budget exceeded before commit")
    passes = 1 + 2 * len(batch_reports)
    report = dict(status="complete", algorithm=ALGORITHM, committed_directory_name=OUTPUT_NAME,
                  input_session=str(inputs.session), inputs_sha256=inputs.hashes, code_sha256=inputs.code,
                  parameters=params, calibration_status="uncalibrated",
                  common_time_window_ns=[inputs.t_start, inputs.t_end], observations=p1_pass,
                  selection=selection_stats, nominal_positive_cells=total_nominal, reasons=reasons,
                  batches=batch_reports, invariants=verification, timing_seconds=timing,
                  final_map_pgm_sha256=artifacts["map.pgm"], final_map_yaml_sha256=artifacts["map.yaml"],
                  artifacts_sha256=artifacts,
                  resources=dict(platform=platform.platform(), core_peak_rss_bytes=peak_rss_bytes(),
                                 main_sequential_passes=passes,
                                 main_sequential_bytes=inputs.recording["bytes"] * passes,
                                 additional_reads="source SHA before/after and bounded witness seeks"),
                  limits=["Ray diagnostics use scan-end proxies with uncalibrated firing/pose uncertainty.",
                          "Nominal positive diagnostics are measurement evidence only.",
                          "Physical accuracy and navigation performance are unmeasured.",
                          "Atomic visibility uses an exclusive rename. Power-loss durability is unmeasured."])
    dump(stage / "report.json", report)
    require(time.monotonic() <= deadline, "Time budget exceeded at commit")
    tx.commit()
    return report


def process(session, max_seconds=DEFAULT_MAX_SECONDS):
    """Run v1 on a verified session; the CLI enforces the operational path scope."""
    require(math.isfinite(max_seconds) and max_seconds > 0, "Invalid conflict time limit")
    session = Path(session).resolve(strict=True)
    tx = Transaction(session)
    try:
        return algorithm_run(session, max_seconds, tx)
    finally:
        tx.close()


def allowed_session(argument):
    require(argument.is_absolute(), "--session must be an absolute path")
    path = argument.resolve(strict=True)
    roots = (LIVE_ROOT.resolve(), FIXTURE_ROOT.resolve())
    require(path.is_dir() and any(path != root and path.is_relative_to(root) for root in roots),
            "--session must be below the live-session or verification root")
    return path


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--max-seconds", type=float, default=DEFAULT_MAX_SECONDS)
    args = parser.parse_args()

    def stop(signum, frame):
        raise KeyboardInterrupt(f"signal {signum}")

    signal.signal(signal.SIGTERM, stop)
    try:
        session = allowed_session(args.session)
        process(session, args.max_seconds)
    except (Exception, KeyboardInterrupt) as error:
        print(f"CONFLICT_MAP_FAILED {type(error).__name__}: {error}", file=sys.stderr, flush=True)
        return 1
    try:
        print(f"CONFLICT_MAP_SAVED {session / OUTPUT_NAME / 'map.yaml'}", flush=True)
    except BrokenPipeError:
        sys.stdout = open(os.devnull, "w")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
