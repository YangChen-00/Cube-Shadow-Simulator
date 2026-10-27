"""Remote workcell configuration. No robot geometry is guessed in code."""

from __future__ import annotations

import json
import math
from dataclasses import dataclass
from pathlib import Path
from typing import Any, Dict, Optional, Tuple


def _tuple(raw: Dict[str, Any], key: str, length: int) -> Tuple[float, ...]:
    value = raw.get(key)
    if not isinstance(value, list) or len(value) != length:
        raise ValueError(f"{key} must be an array of {length} numbers")
    result = tuple(float(v) for v in value)
    if not all(math.isfinite(v) for v in result):
        raise ValueError(f"{key} must contain finite numbers")
    return result


@dataclass(frozen=True)
class WorkcellConfig:
    robot_yaml: str
    tabletop_translation_m: Tuple[float, float, float]
    tabletop_quaternion_wxyz: Tuple[float, float, float, float]
    tool_down_quaternion_tabletop_wxyz: Tuple[float, float, float, float]
    tcp_to_cube_center_m: float
    approach_height_m: float
    lift_height_m: float
    attached_link_name: str
    initial_joint_positions: Optional[Tuple[float, ...]]
    source_origin_mm: Tuple[float, float, float]
    source_columns: int
    source_gap_mm: float
    table_thickness_m: float
    table_margin_m: float
    wall_thickness_m: float
    position_tolerance_m: float
    orientation_tolerance_rad: float
    collision_activation_distance_m: float
    max_attempts: int
    warmup_iterations: int
    interpolation_dt: float
    viser_host: str
    viser_port: int
    playback_hz: float
    light_intensity: float

    @classmethod
    def load(cls, path: str | Path) -> "WorkcellConfig":
        source = Path(path).expanduser().resolve()
        raw = json.loads(source.read_text(encoding="utf-8"))
        frame = raw["tabletop_in_robot_base"]
        source_cfg = raw["cube_source"]
        planner = raw.get("planner", {})
        viewer = raw.get("viewer", {})
        initial = raw.get("initial_joint_positions")
        return cls(
            robot_yaml=str(raw.get("robot_yaml", "franka.yml")),
            tabletop_translation_m=_tuple(frame, "translation_m", 3),
            tabletop_quaternion_wxyz=_normalise(_tuple(frame, "quaternion_wxyz", 4)),
            tool_down_quaternion_tabletop_wxyz=_normalise(
                _tuple(raw, "tool_down_quaternion_tabletop_wxyz", 4)
            ),
            tcp_to_cube_center_m=float(raw.get("tcp_to_cube_center_m", 0.105)),
            approach_height_m=float(raw.get("approach_height_m", 0.10)),
            lift_height_m=float(raw.get("lift_height_m", 0.14)),
            attached_link_name=str(raw.get("attached_link_name", "attached_object")),
            initial_joint_positions=None if initial is None else tuple(float(v) for v in initial),
            source_origin_mm=_tuple(source_cfg, "origin_mm", 3),
            source_columns=int(source_cfg.get("columns", 4)),
            source_gap_mm=float(source_cfg.get("gap_mm", 20.0)),
            table_thickness_m=float(raw.get("table_thickness_m", 0.04)),
            table_margin_m=float(raw.get("table_margin_m", 0.12)),
            wall_thickness_m=float(raw.get("wall_thickness_m", 0.01)),
            position_tolerance_m=float(planner.get("position_tolerance_m", 0.005)),
            orientation_tolerance_rad=float(planner.get("orientation_tolerance_rad", 0.05)),
            collision_activation_distance_m=float(
                planner.get("collision_activation_distance_m", 0.01)
            ),
            max_attempts=int(planner.get("max_attempts", 5)),
            warmup_iterations=int(planner.get("warmup_iterations", 5)),
            interpolation_dt=float(planner.get("interpolation_dt", 0.025)),
            viser_host=str(viewer.get("host", "127.0.0.1")),
            viser_port=int(viewer.get("port", 8080)),
            playback_hz=float(viewer.get("playback_hz", 40.0)),
            light_intensity=float(viewer.get("light_intensity", 35.0)),
        )


def _normalise(q: Tuple[float, ...]) -> Tuple[float, ...]:
    norm = math.sqrt(sum(v * v for v in q))
    if norm < 1e-9:
        raise ValueError("Quaternion must not be zero")
    return tuple(v / norm for v in q)
