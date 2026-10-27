"""Deterministic task-level pick/place phases passed to cuRobo."""

from dataclasses import dataclass
from typing import List

from .config import WorkcellConfig
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
    phases: tuple[TaskPhase, ...]


def source_pose(index: int, side: float, cfg: WorkcellConfig) -> Pose:
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
        result.append(CubeTask(
            cube_id=cube.id,
            step=cube.step,
            source=source,
            target=target,
            supports=cube.contacts_below,
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
