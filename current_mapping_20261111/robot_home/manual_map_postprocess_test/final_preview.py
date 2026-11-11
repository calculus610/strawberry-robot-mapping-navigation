#!/usr/bin/env python3
"""Create the final saved-map comparison and optionally request a local viewer."""

import argparse
from dataclasses import dataclass
import hashlib
import json
import math
import os
from pathlib import Path
import re
import subprocess
import sys
import tempfile

from PIL import Image, ImageDraw, ImageFont
import yaml


OCCUPIED, UNKNOWN, FREE = 0, 205, 254
PALETTE = bytes((OCCUPIED, UNKNOWN, FREE))
MAX_MAP_CELLS = 10_000_000
BLUE, PURPLE, ORANGE = (79, 149, 255), (177, 58, 212), (255, 151, 61)
NEAREST = getattr(Image, "Resampling", Image).NEAREST
PNG_SIGNATURE = b"\x89PNG\r\n\x1a\n"
VIEWER = "/usr/bin/xdg-open"


@dataclass(frozen=True)
class SavedMap:
    size: tuple
    pixels: bytes
    yaml_bytes: bytes


def _checked_file(path):
    if path.is_symlink() or not path.is_file():
        raise ValueError(f"Expected a regular file at {path}")
    return path


def _read_map(folder, pgm_sha, yaml_sha):
    if folder.is_symlink() or not folder.is_dir():
        raise ValueError(f"Expected a saved map directory at {folder}")
    yaml_path = _checked_file(folder / "map.yaml")
    pgm_path = _checked_file(folder / "map.pgm")
    metadata = yaml_path.read_bytes()
    if hashlib.sha256(metadata).hexdigest() != yaml_sha:
        raise ValueError(f"YAML SHA-256 mismatch at {yaml_path}")
    with pgm_path.open("rb") as stream:
        magic = stream.readline(4)
        dimensions = stream.readline(64)
        maximum = stream.readline(5)
        match = re.fullmatch(rb"([1-9][0-9]*) ([1-9][0-9]*)\n", dimensions)
        if magic != b"P5\n" or maximum != b"255\n" or match is None:
            raise ValueError(f"Invalid binary 8-bit PGM header at {pgm_path}")
        width, height = map(int, match.groups())
        cells = width * height
        if cells > MAX_MAP_CELLS:
            raise ValueError(f"PGM exceeds the cell limit at {pgm_path}")
        pixels = stream.read(cells + 1)
    if len(pixels) != cells:
        raise ValueError(f"PGM payload length differs from its geometry at {pgm_path}")
    digest = hashlib.sha256(magic + dimensions + maximum)
    digest.update(pixels)
    if digest.hexdigest() != pgm_sha:
        raise ValueError(f"PGM SHA-256 mismatch at {pgm_path}")
    if pixels.translate(None, PALETTE):
        raise ValueError(f"PGM contains pixels outside the trinary palette at {pgm_path}")
    return SavedMap((width, height), pixels, metadata)


def _verify_geometry(original, *stages):
    if any(original.size != stage.size for stage in stages):
        raise ValueError("Saved stage PGM dimensions differ")
    if any(original.yaml_bytes != stage.yaml_bytes for stage in stages):
        raise ValueError("Saved stage YAML bytes differ")
    meta = yaml.safe_load(original.yaml_bytes)
    if (meta["image"] != "map.pgm" or meta["mode"] != "trinary" or meta["negate"] != 0
            or not math.isfinite(meta["resolution"]) or meta["resolution"] <= 0
            or len(meta["origin"]) != 3 or not all(map(math.isfinite, meta["origin"]))
            or not 1 / 255 < meta["free_thresh"] < 50 / 255 < meta["occupied_thresh"] < 1):
        raise ValueError("Invalid trinary map metadata")


def _inspect_cells(original, a005, conflict, scatter):
    width, height = original.size
    left, top, right, bottom = width, height, 0, 0
    added = demoted = removed = 0
    unknown = bytes((UNKNOWN,))
    for y in range(height):
        start = y * width
        stop = start + width
        a005_row = a005.pixels[start:stop]
        original_row = original.pixels[start:stop]
        conflict_row = conflict.pixels[start:stop]
        if a005_row != original_row:
            for before, after in zip(original_row, a005_row):
                if before != after:
                    if (before, after) != (UNKNOWN, OCCUPIED):
                        raise ValueError("A005 changes a cell outside Unknown to Occupied")
                    added += 1
        if conflict_row != a005_row:
            for before, after in zip(a005_row, conflict_row):
                if before != after:
                    if (before, after) != (FREE, UNKNOWN):
                        raise ValueError("Conflict map changes a cell outside A005 Free to Unknown")
                    demoted += 1
        if scatter is not None:
            scatter_row = scatter.pixels[start:stop]
            if scatter_row != conflict_row:
                for before, after in zip(conflict_row, scatter_row):
                    if before != after:
                        if (before, after) != (OCCUPIED, UNKNOWN):
                            raise ValueError("Scatter map changes a cell outside C Occupied to Unknown")
                        removed += 1
        # Verified transitions make the A005 bounds contain the visible cells in all maps.
        from_left = a005_row.lstrip(unknown)
        if from_left:
            left = min(left, width - len(from_left))
            right = max(right, len(a005_row.rstrip(unknown)))
            top = min(top, y)
            bottom = y + 1
    if bottom:
        margin = 12
        crop = (max(0, left - margin), max(0, top - margin),
                min(width, right + margin), min(height, bottom + margin))
    else:
        crop = (0, 0, width, height)
    return crop, added, demoted, removed


def _thumbnail(saved_map, crop, size):
    with Image.frombytes("L", saved_map.size, saved_map.pixels) as grayscale:
        return grayscale.resize(size, NEAREST, box=crop)


def _display_interval(cell, scale):
    # Nearest-neighbor sampling uses display pixel centers.
    first = math.ceil(cell * scale - 0.5)
    after = math.ceil((cell + 1) * scale - 0.5)
    if first == after:
        pooled = int((cell + 0.5) * scale)
        return pooled, pooled
    return first, after - 1


def _overlay_changes(picture, before, after, crop, color):
    if before is after:
        return
    left, top, right, bottom = crop
    width, _ = before.size
    scale_x = picture.width / (right - left)
    scale_y = picture.height / (bottom - top)
    draw = ImageDraw.Draw(picture)
    for y in range(top, bottom):
        start = y * width
        previous = before.pixels[start + left:start + right]
        current = after.pixels[start + left:start + right]
        if previous == current:
            continue
        y0, y1 = _display_interval(y - top, scale_y)
        # Pool onto the display so isolated changed cells remain visible after reduction.
        for x, (old, new) in enumerate(zip(previous, current)):
            if old != new:
                x0, x1 = _display_interval(x, scale_x)
                draw.rectangle((x0, y0, x1, y1), fill=color)


def _font(size):
    for name in ("DejaVuSans.ttf", "Ubuntu-R.ttf"):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    try:
        return ImageFont.load_default(size=size)
    except TypeError:  # Pillow versions whose bitmap fallback has a fixed size
        return ImageFont.load_default()


def _render(original, a005, conflict, scatter, fallback, crop, added, demoted, removed):
    width, height = crop[2] - crop[0], crop[3] - crop[1]
    columns = 2 if scatter is None else 3
    scale = min(2, (720 if scatter is None else 520) / width, 410 / height)
    size = (max(1, round(width * scale)), max(1, round(height * scale)))
    thumbnails = [_thumbnail(saved_map, crop, size) for saved_map in (original, a005, conflict)]
    difference = thumbnails[0].convert("RGB")
    _overlay_changes(difference, original, a005, crop, BLUE)
    _overlay_changes(difference, a005, conflict, crop, PURPLE)
    if scatter is not None:
        thumbnails.append(_thumbnail(scatter, crop, size))
        _overlay_changes(difference, conflict, scatter, crop, ORANGE)
    panel_width = max(335, size[0])
    padding, panel_header = 16, 24
    title_height = 70
    panel_height = panel_header + size[1]
    legend_y = title_height + 2 * panel_height + 2 * padding
    canvas_height = legend_y + 52
    if scatter is not None:
        legend_y = title_height + panel_height + padding
        canvas_height = legend_y + max(panel_height, panel_header + 6 * 23) + padding
    canvas_width = columns * panel_width + (columns + 1) * padding
    canvas = Image.new("RGB", (canvas_width, canvas_height), (31, 36, 45))
    draw = ImageDraw.Draw(canvas)
    white = (247, 249, 252)
    heading, label, small = _font(24), _font(16), _font(14)
    draw.text((padding, 10), "Saved map comparison", font=heading, fill=white)
    caption = "Using A005 fallback" if fallback else "Using Free-conflict candidate"
    titles = ("Original", "A005", "Final (A005 fallback)" if fallback else "Final candidate",
              "Changes from original")
    if scatter is not None:
        caption = f"Experimental scatter filter ({removed:,} changed cells)"
        titles = ("Original", "A005", "C (Free conflict)", "Final (scatter)", "Changes by stage")
    draw.text((padding, 42), caption, font=label, fill=white)
    for index, (title, image) in enumerate(zip(titles, thumbnails + [difference])):
        row, col = divmod(index, columns)
        x = padding + col * (panel_width + padding)
        y = title_height + row * (panel_height + padding)
        draw.text((x, y), title, font=label, fill=white)
        canvas.paste(image, (x, y + panel_header))

    entries = [
        ("Occupied", (OCCUPIED,) * 3),
        ("Free", (FREE,) * 3),
        ("Unknown", (UNKNOWN,) * 3),
        (f"A005 Unknown to Occupied ({added:,} cells)", BLUE),
        (f"Conflict Free to Unknown ({demoted:,} cells)", PURPLE),
    ]
    positions = [(padding, legend_y), (padding + 108, legend_y), (padding + 185, legend_y),
                 (padding, legend_y + 23), (2 * padding + panel_width, legend_y + 23)]
    if scatter is not None:
        entries.append((f"Scatter Occupied to Unknown ({removed:,} cells)", ORANGE))
        legend_x = padding + 2 * (panel_width + padding)
        draw.text((legend_x, legend_y), "Legend", font=label, fill=white)
        positions = [(legend_x, legend_y + panel_header + index * 23)
                     for index in range(len(entries))]
    for (x, y), (name, fill) in zip(positions, entries):
        draw.rectangle((x, y + 3, x + 12, y + 15), fill=fill, outline=(157, 165, 177))
        draw.text((x + 19, y), name, font=small, fill=white)
    return canvas, size


def _save_new(canvas, output):
    temporary = None
    try:
        with tempfile.NamedTemporaryFile(prefix=".preview-", suffix=".tmp",
                                         dir=output.parent, delete=False) as stream:
            temporary = Path(stream.name)
            canvas.save(stream, format="PNG")
            stream.flush()
            os.fsync(stream.fileno())
        with Image.open(temporary) as check:
            check.verify()
        os.link(temporary, output)
    finally:
        if temporary is not None:
            temporary.unlink()


def create_preview(session, selected_map, out=None):
    """Verify a completed session map and publish a new comparison PNG."""
    session = Path(session)
    selected_map = Path(selected_map)
    output = session / "final_comparison.png" if out is None else Path(out)
    if not session.is_absolute() or session.is_symlink() or not session.is_dir():
        raise ValueError("Session must be an absolute path to a saved session directory")
    a005_path = session / "enhanced" / "map.yaml"
    conflict_path = session / "enhanced_conflict" / "map.yaml"
    scatter_path = session / "enhanced_scatter" / "map.yaml"
    if selected_map not in (a005_path, conflict_path, scatter_path):
        raise ValueError("Selected map must be the session's A005, committed conflict map or committed scatter map")
    if output.parent != session or output.suffix != ".png" or output.name.startswith("."):
        raise ValueError("Output must be a new PNG directly inside the session directory")
    if os.path.lexists(output):
        raise FileExistsError(output)

    fallback = selected_map == a005_path
    conflict_report = scatter_report = None
    if selected_map == scatter_path:
        from offline_scatter import verify_result as verify_scatter_result

        scatter_report = verify_scatter_result(session)
        if scatter_report["status"] != "complete":
            raise ValueError("Scatter report is incomplete")
        # Scatter verification also verifies the committed C result. Bind the
        # copies used to render to those exact verified input bytes.
        conflict_report_path = _checked_file(conflict_path.parent / "report.json")
        conflict_report_bytes = conflict_report_path.read_bytes()
        source_hashes = scatter_report["inputs_sha256"]
        if hashlib.sha256(conflict_report_bytes).hexdigest() != source_hashes["enhanced_conflict/report.json"]:
            raise ValueError("Conflict report changed after scatter verification")
        conflict_report = json.loads(conflict_report_bytes)
        for name in ("map.pgm", "map.yaml"):
            if conflict_report["artifacts_sha256"][name] != source_hashes[f"enhanced_conflict/{name}"]:
                raise ValueError(f"Conflict {name} differs from the verified scatter input")
    elif selected_map == conflict_path:
        from conflict_verify import verify_result

        conflict_report = verify_result(session)
    a005_report_bytes = _checked_file(session / "report.json").read_bytes()
    if conflict_report is not None:
        if hashlib.sha256(a005_report_bytes).hexdigest() != conflict_report["inputs_sha256"]["report.json"]:
            raise ValueError("A005 report changed after conflict verification")
    a005_report = json.loads(a005_report_bytes)
    if a005_report.get("status") != "complete":
        raise ValueError("A005 report is incomplete")
    recorded_root = Path(a005_report["session"])
    recorded_hashes = a005_report["original_inputs_sha256"]
    original = _read_map(session / "original",
                         recorded_hashes[str(recorded_root / "original/map.pgm")],
                         recorded_hashes[str(recorded_root / "original/map.yaml")])
    a005 = _read_map(a005_path.parent, a005_report["candidate_pgm_sha256"],
                     a005_report["candidate_yaml_sha256"])
    conflict = a005
    if conflict_report is not None:
        hashes = conflict_report["artifacts_sha256"]
        conflict = _read_map(conflict_path.parent, hashes["map.pgm"], hashes["map.yaml"])
    scatter = None
    stages = [a005, conflict]
    if scatter_report is not None:
        scatter = _read_map(scatter_path.parent, scatter_report["final_map_pgm_sha256"],
                            scatter_report["final_map_yaml_sha256"])
        stages.append(scatter)
    _verify_geometry(original, *stages)
    crop, added, demoted, removed = _inspect_cells(original, a005, conflict, scatter)
    if scatter_report is not None and scatter_report["invariants"]["changed_cells"] != removed:
        raise ValueError("Scatter changed-cell count differs from the verified report")
    canvas, size = _render(original, a005, conflict, scatter, fallback, crop, added, demoted, removed)
    _save_new(canvas, output)
    result = {
        "path": str(output),
        "map": str(selected_map),
        "fallback": fallback,
        "a005_added_occupied": added,
        "conflict_demoted_free": demoted,
        "crop_col_row_bounds_half_open": list(crop),
        "panel_display_width_height": list(size),
    }
    if scatter is not None:
        result["scatter_changed_cells"] = removed
    return result


def _local_display(environment):
    match = re.fullmatch(r"(?:unix/?)?:([0-9]+)(?:\.[0-9]+)?", environment.get("DISPLAY", ""))
    x11 = match is not None and (Path("/tmp/.X11-unix") / f"X{match[1]}").is_socket()
    wayland = False
    if environment.get("WAYLAND_DISPLAY"):
        socket = Path(environment["WAYLAND_DISPLAY"])
        runtime = Path(environment.get("XDG_RUNTIME_DIR", ""))
        wayland = socket.is_socket() if socket.is_absolute() else (
            runtime.is_absolute() and (runtime / socket).is_socket()
        )
    if not x11:
        environment.pop("DISPLAY", None)
    if not wayland:
        environment.pop("WAYLAND_DISPLAY", None)
    return x11 or wayland


def request_local_preview(path, env=None):
    """Request a detached local viewer and report the launch outcome."""
    path = Path(path)
    result = {"path": str(path)}
    environment = dict(os.environ if env is None else env)
    try:
        if not path.is_absolute() or path.suffix != ".png":
            raise ValueError("Preview must be an absolute PNG path")
        with _checked_file(path).open("rb") as stream:
            if stream.read(len(PNG_SIGNATURE)) != PNG_SIGNATURE:
                raise ValueError("Preview has an invalid PNG signature")
        if not _local_display(environment):
            return dict(result, status="disabled", reason="No local desktop display socket")
        environment.pop("POSTPROCESS_OWNER_TOKEN", None)
        viewer = subprocess.Popen(
            [VIEWER, str(path)], shell=False, env=environment, start_new_session=True,
            stdin=subprocess.DEVNULL, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
            close_fds=True,
        )
        exit_code = viewer.poll()
        if exit_code not in (None, 0):
            return dict(result, status="failed", reason=f"xdg-open exited with code {exit_code}")
        return dict(result, status="requested", pid=viewer.pid)
    except (OSError, ValueError) as exc:
        return dict(result, status="failed", reason=str(exc))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--map", dest="selected_map", type=Path, required=True)
    parser.add_argument("--out", type=Path)
    args = parser.parse_args(argv)
    try:
        report = create_preview(args.session, args.selected_map, args.out)
    except (OSError, ValueError, TypeError, KeyError, RuntimeError, yaml.YAMLError) as exc:
        print(f"FINAL_PREVIEW_FAILED {exc}", file=sys.stderr)
        return 1
    print("FINAL_PREVIEW_SAVED " + report["path"], flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
