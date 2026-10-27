"""Strict reader for Cube Shadow Studio's layout_schema.json handoff."""

import json
import math
import re
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Tuple

from .models import Placement, Pose, Scene


class LayoutError(ValueError):
    pass


@dataclass(frozen=True)
class Layout:
    schema_version: str
    method: str
    scene: Scene
    cube_cap: int
    placements: Tuple[Placement, ...]
    target: Dict[str, Any]
    metrics: Dict[str, Any]
    validation: Dict[str, Any]
    source_path: Path


def _finite(value: Any, label: str) -> float:
    if isinstance(value, bool):
        raise LayoutError(f"{label} must be a finite number")
    try:
        number = float(value)
    except (TypeError, ValueError) as exc:
        raise LayoutError(f"{label} must be a finite number") from exc
    if not math.isfinite(number):
        raise LayoutError(f"{label} must be a finite number")
    return number


def _numbers(value: Any, count: int, label: str) -> Tuple[float, ...]:
    if not isinstance(value, (list, tuple)) or len(value) != count:
        raise LayoutError(f"{label} must contain {count} numbers")
    return tuple(_finite(v, f"{label}[{i}]") for i, v in enumerate(value))


def load_layout(path: str) -> Layout:
    source = Path(path).expanduser().resolve()
    try:
        raw = json.loads(source.read_text(encoding="utf-8"))
    except (OSError, json.JSONDecodeError) as exc:
        raise LayoutError(f"Cannot read layout JSON: {exc}") from exc
    if not isinstance(raw, dict):
        raise LayoutError("Layout root must be an object")
    required = {"schema_version", "method", "coordinate_frame", "scene", "cube_cap", "placements", "target", "metrics", "validation"}
    missing = sorted(required - raw.keys())
    if missing:
        raise LayoutError(f"Missing required fields: {', '.join(missing)}")
    if raw["schema_version"] != "1.0":
        raise LayoutError(f"Unsupported schema_version {raw['schema_version']!r}; expected '1.0'")
    frame = raw["coordinate_frame"]
    if frame.get("unit") != "mm" or frame.get("yaw_unit") != "degrees":
        raise LayoutError("Only millimetres and degree yaw are supported")
    scene_raw = raw["scene"]
    cube_size = _finite(scene_raw.get("cube_size"), "scene.cube_size")
    if cube_size <= 0:
        raise LayoutError("scene.cube_size must be positive")
    image = scene_raw.get("image_size")
    if not isinstance(image, list) or len(image) != 2 or any(isinstance(v, bool) or not isinstance(v, int) or v <= 0 for v in image):
        raise LayoutError("scene.image_size must contain two positive integers")
    build = scene_raw.get("build_bounds")
    scene = Scene(
        cube_size=cube_size,
        light=_numbers(scene_raw.get("light"), 3, "scene.light"),
        wall_y=_finite(scene_raw.get("wall_y"), "scene.wall_y"),
        wall_bounds=_numbers(scene_raw.get("wall_bounds"), 4, "scene.wall_bounds"),
        image_size=(image[0], image[1]),
        build_bounds=None if build is None else _numbers(build, 4, "scene.build_bounds"),
    )
    cap = raw["cube_cap"]
    if isinstance(cap, bool) or not isinstance(cap, int) or cap < 0:
        raise LayoutError("cube_cap must be a nonnegative integer")
    items = raw["placements"]
    if not isinstance(items, list):
        raise LayoutError("placements must be an array")
    if len(items) > cap:
        raise LayoutError(f"Layout has {len(items)} cubes but cube_cap is {cap}")
    placements = []
    seen = set()
    for index, item in enumerate(items, 1):
        if not isinstance(item, dict):
            raise LayoutError(f"placements[{index - 1}] must be an object")
        cube_id = item.get("id")
        if not isinstance(cube_id, str) or not re.fullmatch(r"cube_[0-9]{3}", cube_id):
            raise LayoutError(f"Invalid cube id at placement {index}: {cube_id!r}")
        if cube_id in seen:
            raise LayoutError(f"Duplicate cube id {cube_id}")
        if item.get("step") != index:
            raise LayoutError(f"{cube_id}: steps must be consecutive and match array order")
        contacts = item.get("contacts_below_at_placement")
        if not isinstance(contacts, list) or any(not isinstance(v, str) for v in contacts):
            raise LayoutError(f"{cube_id}: contacts_below_at_placement must be a string array")
        if len(set(contacts)) != len(contacts):
            raise LayoutError(f"{cube_id}: duplicate support ids")
        unknown = [v for v in contacts if v not in seen]
        if unknown:
            raise LayoutError(f"{cube_id}: supports must refer to earlier cubes: {unknown}")
        seen.add(cube_id)
        grounded = item.get("grounded")
        if not isinstance(grounded, bool):
            raise LayoutError(f"{cube_id}: grounded must be Boolean")
        placements.append(Placement(
            id=cube_id,
            step=index,
            pose=Pose(*(_finite(item.get(axis), f"{cube_id}.{axis}") for axis in ("x", "y", "z", "yaw"))),
            grounded=grounded,
            contacts_below=tuple(contacts),
        ))
    target = raw["target"]
    if not isinstance(target, dict) or target.get("image_size") != image:
        raise LayoutError("target.image_size must match scene.image_size")
    return Layout(
        schema_version="1.0",
        method=str(raw["method"]),
        scene=scene,
        cube_cap=cap,
        placements=tuple(placements),
        target=dict(target),
        metrics=dict(raw["metrics"]),
        validation=dict(raw["validation"]),
        source_path=source,
    )

