# SPDX-FileCopyrightText: Copyright (c) 2025 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
"""Mixed-reality overlay utilities for the UR10 Robotiq Cortex example.

This module provides a minimal, *non-intrusive* path to start visualizing
simulated objects (the cubes placed on the Isaac Sim floor) on top of a
real-world reference image.

The goal is to keep the main example behavior unchanged while exposing a
simple function that can be called from tools, notebooks, or future UI
buttons to generate an overlay image.

Assumptions (first iteration):
	* The reference image is a single RGB image of the real workspace.
	* We choose a fixed pixel on the image as a point on the floor.
	* We use a crude linear mapping from world (x, y) to image (u, v).

This is **not** a calibrated projection. It is intentionally simple so that
you can iterate on camera intrinsics/extrinsics later by replacing the
mapping logic in :func:`project_world_to_image`.
"""

from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from typing import Iterable, List, Mapping, Sequence, Tuple

import numpy as np

try:  # Pillow is usually available in Isaac Python env; fallback is graceful.
	from PIL import Image, ImageDraw
except Exception:  # pragma: no cover - runtime-only dependency
	Image = None  # type: ignore
	ImageDraw = None  # type: ignore


@dataclass
class MRCube:
	"""Simple description of a cube used for MR overlay.

	Attributes
	----------
	name:
		Name of the cube (e.g. "RedCube").
	position:
		World position as (x, y, z) in meters.
	size:
		Cube edge length in meters.
	color:
		RGB triple in [0, 1].
	"""

	name: str
	position: Tuple[float, float, float]
	size: float
	color: Tuple[float, float, float]


def _coerce_cubes(layout: Iterable[Mapping]) -> List[MRCube]:
	"""Convert a layout list of dicts into :class:`MRCube` objects.

	The UR10 example populates ``_mr_cube_layout`` with a list of dicts of the
	form::

		{
			"name": str,
			"color": [r, g, b],
			"size": float,
			"position": [x, y, z],
		}

	This helper keeps the overlay code decoupled from the exact structure on
	the Cortex side.
	"""

	cubes: List[MRCube] = []
	for item in layout:
		name = str(item.get("name", "Cube"))
		pos = item.get("position", [0.0, 0.0, 0.0])
		size = float(item.get("size", 0.05))
		color = item.get("color", [1.0, 1.0, 1.0])
		if len(pos) != 3:
			continue
		if len(color) != 3:
			color = [1.0, 1.0, 1.0]
		cubes.append(
			MRCube(
				name=name,
				position=(float(pos[0]), float(pos[1]), float(pos[2])),
				size=size,
				color=(float(color[0]), float(color[1]), float(color[2])),
			)
		)
	return cubes


def project_world_to_image(
	position: Sequence[float],
	image_size: Tuple[int, int],
	floor_world_ref: Tuple[float, float] = (0.45, 0.3),
	floor_pixel_ref: Tuple[int, int] = (640, 720),
	scale_x: float = 900.0,
	scale_y: float = -900.0,
) -> Tuple[int, int]:
	"""Project a world-space point (x, y, z) to image pixel coordinates.

	This is a deliberately simple mapping used to *start* experimenting with
	MR overlays. It is **not** physically correct.

	The mapping treats a chosen world (x, y) point as mapping to a chosen
	pixel on the image (``floor_pixel_ref``) and then scales x/y offsets by
	fixed factors (``scale_x``, ``scale_y``).

	Parameters
	----------
	position:
		World position (x, y, z) in meters.
	image_size:
		(width, height) of the image in pixels.
	floor_world_ref:
		World (x, y) that is assumed to lie on the visible floor in the
		image and maps exactly to ``floor_pixel_ref``.
	floor_pixel_ref:
		Pixel (u, v) chosen on the image that corresponds to
		``floor_world_ref``.
	scale_x, scale_y:
		Scaling from world meters to image pixels along x and y. ``scale_y``
		is typically negative to account for image coordinates increasing
		downward.
	"""

	x, y, _z = position
	w, h = image_size

	wx0, wy0 = floor_world_ref
	u0, v0 = floor_pixel_ref

	du = (x - wx0) * scale_x
	dv = (y - wy0) * scale_y

	u = int(round(u0 + du))
	v = int(round(v0 + dv))

	# Clamp to image bounds
	u = max(0, min(w - 1, u))
	v = max(0, min(h - 1, v))

	return u, v


def draw_cubes_on_image(
	image,
	cubes: Sequence[MRCube],
	radius_pixels: int = 15,
):
	"""Draw simple markers for cubes onto an image.

	Each cube becomes a filled circle (or square, if desired) at the
	projected pixel location. The color is taken from the cube's RGB, scaled
	to 0–255.
	"""

	if ImageDraw is None:
		raise RuntimeError("Pillow (PIL) is not available; cannot draw MR overlay.")

	draw = ImageDraw.Draw(image, "RGBA")
	w, h = image.size

	for cube in cubes:
		u, v = project_world_to_image(cube.position, (w, h))

		r = radius_pixels
		bbox = (u - r, v - r, u + r, v + r)

		rgb = tuple(int(255 * max(0.0, min(1.0, c))) for c in cube.color)

		# Semi-transparent fill to keep the real image visible underneath.
		fill = rgb + (140,)
		outline = rgb + (255,)

		draw.ellipse(bbox, fill=fill, outline=outline, width=2)

	return image


def generate_mr_overlay(
	input_image_path: str,
	cube_layout: Iterable[Mapping],
	output_image_path: str | None = None,
) -> str:
	"""Generate a mixed-reality overlay image for the provided layout.

	Parameters
	----------
	input_image_path:
		Path to the real-world reference image onto which virtual objects
		will be overlaid.
	cube_layout:
		Iterable of mappings describing cubes (see ``_mr_cube_layout`` in
		``ur10_robotiq_cortex.py`` for the expected keys).
	output_image_path:
		Optional explicit output path. If omitted, a ``*_mr_overlay.png``
		image is written next to ``input_image_path``.

	Returns
	-------
	str
		Path to the written overlay image.
	"""

	if Image is None:
		raise RuntimeError(
			"Pillow (PIL) is not available in this Python environment; "
			"cannot generate MR overlay image."
		)

	in_path = Path(input_image_path)
	if not in_path.is_file():
		raise FileNotFoundError(f"Input MR image not found: {in_path}")

	img = Image.open(in_path).convert("RGBA")

	cubes = _coerce_cubes(cube_layout)
	img = draw_cubes_on_image(img, cubes)

	if output_image_path is None:
		output_image_path = str(in_path.with_name(in_path.stem + "_mr_overlay.png"))

	out_path = Path(output_image_path)
	out_path.parent.mkdir(parents=True, exist_ok=True)
	img.save(out_path)

	return str(out_path)

__all__ = [
	"MRCube",
	"project_world_to_image",
	"draw_cubes_on_image",
	"generate_mr_overlay",
]
