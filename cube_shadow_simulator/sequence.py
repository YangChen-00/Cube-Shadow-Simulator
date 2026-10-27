"""Preparation helpers for the dual-arm U -> S -> T showcase."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass, replace
from pathlib import Path
from typing import Optional

import numpy as np

from .config import WorkcellConfig
from .curobo_runner import CuroboPlan, StaticCubeObstacle
from .layout import Layout
from .transforms import matrix, matrix_pose


@dataclass(frozen=True)
class SequenceRun:
    label: str
    arm: str
    layout: Layout
    plan: CuroboPlan
    config: WorkcellConfig
    output_dir: Path
    target_image: Optional[str] = None
    stage: int = 0

    @property
    def base_to_tabletop(self) -> np.ndarray:
        tabletop_to_base = matrix(
            self.config.tabletop_translation_m,
            self.config.tabletop_quaternion_wxyz,
        )
        return np.linalg.inv(tabletop_to_base)


# Pickup sits inward of each base, but downstage of S (which reaches y=-224 mm).
FEEDER_INWARD_M = 0.30
FEEDER_DOWNSTAGE_M = 0.28


def arm_base_tabletop(
    arm: str,
    separation_m: float,
    lateral_m: float,
) -> tuple[np.ndarray, float]:
    """Return the base position in tabletop metres and its yaw.

    The arms stand on opposite sides of the letter row and face inward, so the
    left arm reaches U from -X and the right arm reaches T from +X.
    """
    if arm not in {"left", "right"}:
        raise ValueError("arm must be 'left' or 'right'")
    sign = -1.0 if arm == "left" else 1.0
    base = np.asarray([sign * 0.5 * separation_m, lateral_m, 0.0], dtype=float)
    yaw = 0.0 if arm == "left" else math.pi
    return base, yaw


def _tabletop_in_robot_base(base_xyz: np.ndarray, yaw: float):
    rotation = np.array(
        [
            [math.cos(yaw), -math.sin(yaw), 0.0],
            [math.sin(yaw), math.cos(yaw), 0.0],
            [0.0, 0.0, 1.0],
        ],
        dtype=float,
    )
    translation = -rotation.T @ np.asarray(base_xyz, dtype=float)
    half = -0.5 * yaw
    quaternion = (math.cos(half), 0.0, 0.0, math.sin(half))
    return tuple(float(v) for v in translation), quaternion


def arm_config(
    base: WorkcellConfig,
    arm: str,
    separation_m: float,
    lateral_m: float = -0.20,
) -> WorkcellConfig:
    """Place one arm on the left or right flank of the shared tabletop.

    Tool calibration is preserved. Only the base pose and that arm's feeder
    move, so each gripper works on its own side of U, S, and T.
    """
    base_xyz, yaw = arm_base_tabletop(arm, separation_m, lateral_m)
    translation, quaternion = _tabletop_in_robot_base(base_xyz, yaw)
    inward = FEEDER_INWARD_M if arm == "left" else -FEEDER_INWARD_M
    feeder = base_xyz + np.asarray([inward, -FEEDER_DOWNSTAGE_M, 0.0], dtype=float)
    _, _, source_z = base.source_origin_mm
    return replace(
        base,
        tabletop_translation_m=translation,
        tabletop_quaternion_wxyz=quaternion,
        source_origin_mm=(
            float(feeder[0] * 1000.0),
            float(feeder[1] * 1000.0),
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
