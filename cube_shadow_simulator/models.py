"""Data exchanged between Studio, the cuRobo planner, and Viser."""

from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional, Tuple


@dataclass(frozen=True)
class Pose:
    x: float
    y: float
    z: float
    yaw: float = 0.0

    def metres(self) -> Tuple[float, float, float]:
        return self.x / 1000.0, self.y / 1000.0, self.z / 1000.0


@dataclass(frozen=True)
class Placement:
    id: str
    step: int
    pose: Pose
    grounded: bool
    contacts_below: Tuple[str, ...]


@dataclass(frozen=True)
class Scene:
    cube_size: float
    light: Tuple[float, float, float]
    wall_y: float
    wall_bounds: Tuple[float, float, float, float]
    image_size: Tuple[int, int]
    build_bounds: Optional[Tuple[float, float, float, float]]


@dataclass(frozen=True)
class Issue:
    code: str
    message: str
    cube_id: Optional[str] = None
    step: Optional[int] = None
    severity: str = "error"


@dataclass
class FeasibilityReport:
    feasible: bool
    checks: Dict[str, bool]
    issues: List[Issue]
    cube_count: int
    max_height_mm: float
    supported_pixels: str = "up to 16x16 target silhouettes (layout-driven)"

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)


@dataclass
class ExecutionReport:
    success: bool
    completed_steps: int
    total_steps: int
    curobo_version: str = "unknown"
    failed_cube: Optional[str] = None
    failed_phase: Optional[str] = None
    failure_status: Optional[str] = None
    phase_waypoints: Dict[str, int] = field(default_factory=dict)
    notes: List[str] = field(default_factory=list)

    def to_dict(self) -> Dict[str, Any]:
        return asdict(self)

