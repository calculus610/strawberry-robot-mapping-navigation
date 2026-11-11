"""Render a candidate cleanup comparison from two aligned trinary PGM grids."""

from collections.abc import Mapping
from math import isfinite
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont


OCCUPIED = 0
UNKNOWN = 205
FREE = 254
ORANGE_INDEX = 1
ORANGE = (255, 151, 61)
NEAREST = getattr(Image, "Resampling", Image).NEAREST

BACKGROUND = "#20252e"
FOREGROUND = "#f7f8fa"
SUBTITLE = "#c5cbd3"
BORDER = "#7b8592"
MARGIN = 24
GAP = 16
TOP = 114


def _font(size):
    for name in (
        "DejaVuSans.ttf",
        "/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf",
        "/System/Library/Fonts/Supplemental/Arial.ttf",
    ):
        try:
            return ImageFont.truetype(name, size)
        except OSError:
            continue
    return ImageFont.load_default()


def _validate_maps(before, after):
    if before.ndim != 2 or not before.size or before.shape != after.shape:
        raise ValueError("Before and After must be nonempty 2D maps of the same shape")
    if before.dtype != np.uint8 or after.dtype != np.uint8:
        raise ValueError("Before and After must use uint8 PGM pixels")

    allowed = np.zeros(256, dtype=bool)
    allowed[[OCCUPIED, UNKNOWN, FREE]] = True
    if not allowed[before].all() or not allowed[after].all():
        raise ValueError("Expected only PGM values 0, 205, and 254")

    changed = before != after
    if np.any(before[changed] != OCCUPIED) or np.any(after[changed] != UNKNOWN):
        raise ValueError("Cleanup may only change occupied pixels to unknown")
    return changed


def _panel_size(size, bounds, max_scale):
    width, height = size
    scale = min(bounds[0] / width, bounds[1] / height, max_scale)
    return max(1, round(width * scale)), max(1, round(height * scale))


def _change_image(before, changed):
    codes = before.copy()
    codes[changed] = ORANGE_INDEX
    palette = [channel for value in range(256) for channel in (value, value, value)]
    palette[ORANGE_INDEX * 3 : ORANGE_INDEX * 3 + 3] = ORANGE
    image = Image.fromarray(codes)
    image.putpalette(palette)
    return image


def _draw_panels(canvas, draw, panels, display_size, panel_width, y, crop=None):
    width, height = display_size
    for i, panel in enumerate(panels):
        x = MARGIN + i * (panel_width + GAP) + (panel_width - width) // 2
        thumbnail = panel.resize(display_size, NEAREST, box=crop).convert("RGB")
        canvas.paste(thumbnail, (x, y))
        draw.rectangle((x - 1, y - 1, x + width, y + height), outline=BORDER)


def make_preview(
    before: np.ndarray,
    after: np.ndarray,
    output_path: str | Path,
    stats: Mapping | None = None,
) -> Path:
    """Create a new PNG preview. Counts come from the supplied PGM arrays.

    Optional stats may provide resolution_m or resolution in meters per cell.
    The output directory must already exist.
    """
    changed = _validate_maps(before, after)
    withdrawn = int(np.count_nonzero(changed))
    occupied_before = int(np.count_nonzero(before == OCCUPIED))
    occupied_after = occupied_before - withdrawn
    height, width = before.shape
    overview_size = _panel_size((width, height), (600, 820), max_scale=8)
    panel_width = max(285, overview_size[0])
    legend_y = TOP + overview_size[1] + 18

    crop = None
    canvas_height = legend_y + 60
    if withdrawn:
        row, col = divmod(int(np.argmax(changed)), width)
        crop = (max(0, col - 16), max(0, row - 16), min(width, col + 17), min(height, row + 17))
        left, top, right, bottom = crop
        zoom_size = _panel_size((right - left, bottom - top), (220, 220), max_scale=6)
        zoom_y = legend_y + 88
        canvas_height = zoom_y + zoom_size[1] + MARGIN

    canvas_width = 2 * MARGIN + 3 * panel_width + 2 * GAP
    canvas = Image.new("RGB", (canvas_width, canvas_height), BACKGROUND)
    draw = ImageDraw.Draw(canvas)
    title_font, heading_font, body_font = _font(26), _font(19), _font(14)

    draw.text((MARGIN, 12), "Occupancy cleanup preview", font=title_font, fill=FOREGROUND)
    draw.text((MARGIN, 47), "Candidate map | Unvalidated", font=body_font, fill=ORANGE)
    map_detail = f"{width:,} x {height:,} cells"
    if stats is not None:
        resolution = stats.get("resolution_m", stats.get("resolution"))
        if isinstance(resolution, (int, float)) and isfinite(resolution) and resolution > 0:
            map_detail += f" | {resolution:g} m per input cell"
    draw.text((canvas_width // 2, 47), map_detail, font=body_font, fill=SUBTITLE)

    headings = (
        ("Before", f"{occupied_before:,} occupied cells"),
        ("After", f"{occupied_after:,} occupied cells"),
        ("Changes", f"{withdrawn:,} withdrawn cells"),
    )
    for i, (title, count) in enumerate(headings):
        x = MARGIN + i * (panel_width + GAP)
        draw.text((x, 72), title, font=heading_font, fill=FOREGROUND)
        draw.text((x, 94), count, font=body_font, fill=SUBTITLE)

    panels = (Image.fromarray(before), Image.fromarray(after), _change_image(before, changed))
    _draw_panels(canvas, draw, panels, overview_size, panel_width, TOP)

    legend = (
        ("Occupied", (OCCUPIED,) * 3),
        ("Free", (FREE,) * 3),
        ("Unknown", (UNKNOWN,) * 3),
        ("Withdrawn occupancy", ORANGE),
    )
    for i, (label, color) in enumerate(legend):
        x = MARGIN + 160 * i
        draw.rectangle((x, legend_y, x + 13, legend_y + 13), fill=color, outline=BORDER)
        draw.text((x + 21, legend_y - 2), label, font=body_font, fill=FOREGROUND)
    if overview_size[0] < width or overview_size[1] < height:
        scale_note = "Overview reduced by nearest neighbor. Counts use the full grid."
    else:
        scale_note = "Display uses nearest neighbor. Counts use the input grid."
    draw.text((MARGIN, legend_y + 23), scale_note, font=body_font, fill=SUBTITLE)

    if crop is not None:
        draw.text(
            (MARGIN, legend_y + 50),
            f"Example change | Row {row:,}, column {col:,} (zero-based PGM coordinates)",
            font=heading_font,
            fill=FOREGROUND,
        )
        _draw_panels(canvas, draw, panels, zoom_size, panel_width, zoom_y, crop)

    destination = Path(output_path)
    with destination.open("xb") as stream:
        canvas.save(stream, format="PNG")
    return destination
