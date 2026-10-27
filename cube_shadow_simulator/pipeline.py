"""Studio layout -> structural gate -> cuRobo plan -> optional Viser demo."""

from __future__ import annotations

import hashlib
import json
from dataclasses import dataclass
from pathlib import Path
from typing import Optional

import numpy as np

from .config import WorkcellConfig
from .curobo_runner import CuroboAssemblyPlanner, CuroboPlan
from .feasibility import assess_layout
from .layout import Layout, load_layout
from .models import ExecutionReport, FeasibilityReport


@dataclass
class PipelineResult:
    layout: Layout
    structural: FeasibilityReport
    plan: Optional[CuroboPlan]
    engine: Optional[CuroboAssemblyPlanner]
    output_dir: Path

    @property
    def success(self) -> bool:
        return bool(self.structural.feasible and self.plan and self.plan.report.success)


def run_pipeline(
    layout_path: str,
    config_path: str,
    output_dir: str = "runs/latest",
    warmup: bool = True,
) -> PipelineResult:
    layout = load_layout(layout_path)
    config = WorkcellConfig.load(config_path)
    output = Path(output_dir).expanduser().resolve()
    output.mkdir(parents=True, exist_ok=True)
    source_bytes = layout.source_path.read_bytes()
    structural = assess_layout(layout)
    (output / "structural_feasibility.json").write_text(
        json.dumps(structural.to_dict(), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    manifest = {
        "schema_version": "2.0",
        "layout_path": str(layout.source_path),
        "layout_sha256": hashlib.sha256(source_bytes).hexdigest(),
        "studio_method": layout.method,
        "studio_shadow_metrics": layout.metrics,
        "workcell_config": str(Path(config_path).expanduser().resolve()),
        "execution_authority": "cuRoboV2 MotionPlanner",
        "viewer": "cuRobo ViserVisualizer",
        "coordinate_contract": (
            "Studio tabletop millimetres transformed by tabletop_in_robot_base; "
            "cuRobo scene uses metres and wxyz quaternions"
        ),
    }
    (output / "manifest.json").write_text(
        json.dumps(manifest, indent=2, ensure_ascii=False) + "\n", encoding="utf-8"
    )
    if not structural.feasible:
        report = ExecutionReport(
            success=False,
            completed_steps=0,
            total_steps=len(layout.placements),
            failed_phase="structural_gate",
            failure_status="; ".join(issue.code for issue in structural.issues),
            notes=["cuRobo was not called because the layout violates support/gravity constraints."],
        )
        plan = CuroboPlan(report, np.asarray([], dtype=np.float32))
        _write_plan(output, plan)
        return PipelineResult(layout, structural, plan, None, output)

    engine = CuroboAssemblyPlanner(layout, config, warmup=warmup)
    plan = engine.plan()
    _write_plan(output, plan)
    return PipelineResult(layout, structural, plan, engine, output)


def _write_plan(output: Path, plan: CuroboPlan) -> None:
    (output / "curobo_plan.json").write_text(
        json.dumps(plan.summary(), indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    arrays = {phase.key: phase.joint_positions for phase in plan.phases}
    arrays["initial_joint_positions"] = plan.initial_joint_positions
    for cube_id, transform in plan.tool_to_cube.items():
        arrays[f"tool_to_cube_{cube_id}"] = transform
    np.savez_compressed(output / "curobo_trajectories.npz", **arrays)
