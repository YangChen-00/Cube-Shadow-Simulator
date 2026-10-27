"""Deterministic task-level pick/place phases passed to cuRobo."""

import math
from dataclasses import dataclass
from typing import List

from .config import WorkcellConfig
from .geometry import footprint_gap
from .layout import Layout
from .models import Pose


@dataclass(frozen=True)
class TaskPhase:
    name: str
    cube_id: str
    goal: Pose
    vertical_offset_m: float
    holding_cube: bool


@dataclass(frozen=True)
class CubeTask:
    cube_id: str
    step: int
    source: Pose
    target: Pose
    supports: tuple[str, ...]
    place_contacts: tuple[str, ...]
    phases: tuple[TaskPhase, ...]


def source_pose(index: int, side: float, cfg: WorkcellConfig) -> Pose:
    if cfg.source_mode == "feeder":
        x, y, z = cfg.source_origin_mm
        return Pose(x, y, z, 0.0)
    spacing = side + cfg.source_gap_mm
    row, column = divmod(index, cfg.source_columns)
    x, y, z = cfg.source_origin_mm
    return Pose(x + column * spacing, y - row * spacing, z, 0.0)


def make_tasks(layout: Layout, cfg: WorkcellConfig) -> List[CubeTask]:
    side = layout.scene.cube_size
    result = []
    for index, cube in enumerate(layout.placements):
        source = source_pose(index, side, cfg)
        target = cube.pose
        place_contacts = list(cube.contacts_below) or ["table"]
        # Four spheres conservatively enclose the carried cube. Their XY
        # envelope extends beyond the exact square by this amount, so a real
        # non-overlapping neighbour can otherwise make the final vertical
        # descent report a false attached-object collision.
        sphere_overhang_mm = (math.sqrt(6.0) - 1.0) * side / 4.0
        activation_margin_mm = cfg.collision_activation_distance_m * 1000.0
        lateral_tolerance_mm = max(
            1.0,
            sphere_overhang_mm + activation_margin_mm,
        )
        for previous in layout.placements[:index]:
            vertical_overlap = side - abs(target.z - previous.pose.z)
            if (
                vertical_overlap > 1e-6
                and footprint_gap(target, previous.pose, side, side)
                <= lateral_tolerance_mm
                and previous.id not in place_contacts
            ):
                place_contacts.append(previous.id)
        result.append(CubeTask(
            cube_id=cube.id,
            step=cube.step,
            source=source,
            target=target,
            supports=cube.contacts_below,
            place_contacts=tuple(place_contacts),
            phases=(
                TaskPhase("pre_pick", cube.id, source, cfg.approach_height_m, False),
                TaskPhase("pick", cube.id, source, 0.0, False),
                TaskPhase("lift", cube.id, source, cfg.lift_height_m, True),
                TaskPhase("pre_place", cube.id, target, cfg.approach_height_m, True),
                TaskPhase("place", cube.id, target, 0.0, True),
                TaskPhase("retreat", cube.id, target, cfg.approach_height_m, False),
            ),
        ))
    return result
