"""Preparation helpers for the dual-arm U -> S -> T showcase."""

from __future__ import annotations

import json
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

import numpy as np

from .config import WorkcellConfig
from .curobo_runner import CuroboPlan, StaticCubeObstacle
from .layout import Layout
from .transforms import matrix, matrix_pose, quaternion_matrix


@dataclass(frozen=True)
class SequenceRun:
    label: str
    arm: str
    layout: Layout
    plan: CuroboPlan
    config: WorkcellConfig
    output_dir: Path
    target_image: Optional[str] = None

    @property
    def base_to_tabletop(self) -> np.ndarray:
        tabletop_to_base = matrix(
            self.config.tabletop_translation_m,
            self.config.tabletop_quaternion_wxyz,
        )
        return np.linalg.inv(tabletop_to_base)


def arm_config(
    base: WorkcellConfig,
    arm: str,
    separation_m: float,
) -> WorkcellConfig:
    """Return a calibrated parallel-arm transform derived from one workcell.

    The two bases are translated along the tabletop Y axis. Their orientation
    and all robot-specific calibration values remain unchanged. Each arm gets
    a feeder directly in front of its own base.
    """
    if arm not in {"left", "right"}:
        raise ValueError("arm must be 'left' or 'right'")
    base_y_tabletop = (-0.5 if arm == "left" else 0.5) * separation_m
    delta_tabletop = np.asarray([0.0, base_y_tabletop, 0.0], dtype=float)
    rotation = quaternion_matrix(base.tabletop_quaternion_wxyz)
    translated = np.asarray(base.tabletop_translation_m) - rotation @ delta_tabletop
    source_x, source_y, source_z = base.source_origin_mm
    return replace(
        base,
        tabletop_translation_m=tuple(float(v) for v in translated),
        source_origin_mm=(
            source_x,
            source_y + base_y_tabletop * 1000.0,
            source_z,
        ),
    )


def write_shifted_layout(
    source_path: str,
    destination: str | Path,
    x_offset_mm: float,
    combined_x_bounds: tuple[float, float],
) -> Path:
    """Write a sequence-specific handoff while preserving Studio metadata."""
    source = Path(source_path).expanduser().resolve()
    destination = Path(destination).expanduser().resolve()
    raw = json.loads(source.read_text(encoding="utf-8"))
    for placement in raw["placements"]:
        placement["x"] = float(placement["x"]) + x_offset_mm

    scene = raw["scene"]
    build = list(scene.get("build_bounds") or [-200.0, 200.0, -250.0, 150.0])
    build[0], build[1] = combined_x_bounds
    scene["build_bounds"] = build
    wall = list(scene["wall_bounds"])
    wall[0] = min(wall[0], combined_x_bounds[0])
    wall[1] = max(wall[1], combined_x_bounds[1])
    scene["wall_bounds"] = wall
    raw["sequence"] = {
        "source_layout": str(source),
        "x_offset_mm": x_offset_mm,
        "order": ["U", "S", "T"],
    }
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text(
        json.dumps(raw, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    return destination


def completed_cube_obstacles(
    label: str,
    layout: Layout,
    completed_steps: Optional[int] = None,
) -> list[StaticCubeObstacle]:
    placements = layout.placements
    if completed_steps is not None:
        placements = placements[:completed_steps]
    return [
        StaticCubeObstacle(
            name=f"completed_{label}_{placement.id}",
            pose=placement.pose,
            side_mm=layout.scene.cube_size,
        )
        for placement in placements
    ]


def base_pose_in_tabletop(config: WorkcellConfig):
    transform = np.linalg.inv(
        matrix(config.tabletop_translation_m, config.tabletop_quaternion_wxyz)
    )
    return matrix_pose(transform)
