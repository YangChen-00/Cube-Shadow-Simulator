"""Remote-server command line entry point."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

from .layout import LayoutError
from .pipeline import run_pipeline
from .studio_bridge import generate_layout


DEFAULT_CONFIG = Path(__file__).resolve().parents[1] / "config" / "franka_remote.json"


def parser() -> argparse.ArgumentParser:
    root = argparse.ArgumentParser(
        description="Plan Cube Shadow assembly with cuRoboV2 and display it in Viser"
    )
    sub = root.add_subparsers(dest="command", required=True)
    for name, help_text in (
        ("demo", "plan with cuRobo and keep a Viser server open"),
        ("plan", "plan with cuRobo and write evidence files without opening Viser"),
    ):
        cmd = sub.add_parser(name, help=help_text)
        cmd.add_argument("layout", nargs="?", help="Cube-Shadow-Studio layout JSON")
        cmd.add_argument("--drawing", help="run Cube-Shadow-Studio on this silhouette first")
        cmd.add_argument("--studio-root", default="../Cube-Shadow-Studio")
        cmd.add_argument("--budget", type=int, default=32)
        cmd.add_argument("--seconds", type=float, default=5.0)
        cmd.add_argument("--config", default=str(DEFAULT_CONFIG), help="calibrated remote workcell JSON")
        cmd.add_argument("--output", default="runs/latest")
        cmd.add_argument("--target-image", help="optional target silhouette shown in the Viser panel")
        cmd.add_argument("--skip-warmup", action="store_true", help="skip CUDA graph warmup")
        cmd.add_argument("--debug", action="store_true", help="show the full remote traceback")
    sequence = sub.add_parser(
        "sequence",
        help="plan and display a dual-arm U -> S -> T showcase",
    )
    sequence.add_argument("layouts", nargs=3, metavar=("U_JSON", "S_JSON", "T_JSON"))
    sequence.add_argument("--target-images", nargs=3, metavar=("U_PNG", "S_PNG", "T_PNG"))
    sequence.add_argument("--config", default=str(DEFAULT_CONFIG))
    sequence.add_argument("--output", default="runs/UST")
    sequence.add_argument("--arm-separation", type=float, default=0.56, metavar="METRES")
    sequence.add_argument("--letter-spacing", type=float, default=130.0, metavar="MM")
    sequence.add_argument("--skip-warmup", action="store_true")
    sequence.add_argument("--debug", action="store_true")
    return root


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    try:
        if args.command == "sequence":
            return _run_sequence(args)
        if bool(args.layout) == bool(args.drawing):
            raise ValueError("Provide exactly one of layout or --drawing")
        layout_path = args.layout
        target_image = args.target_image
        if args.drawing:
            layout_path = str(generate_layout(
                args.drawing,
                args.studio_root,
                str(Path(args.output) / "studio"),
                args.budget,
                args.seconds,
            ))
            target_image = target_image or args.drawing
        result = run_pipeline(
            layout_path,
            args.config,
            args.output,
            warmup=not args.skip_warmup,
        )
        report = result.plan.report if result.plan else None
        summary = {
            "success": result.success,
            "structurally_feasible": result.structural.feasible,
            "curobo_success": bool(report and report.success),
            "completed_steps": 0 if report is None else report.completed_steps,
            "total_steps": len(result.layout.placements),
            "planned_phases": 0 if result.plan is None else len(result.plan.phases),
            "expected_phases": len(result.layout.placements) * 6 + 1,
            "failed_cube": None if report is None else report.failed_cube,
            "failed_phase": None if report is None else report.failed_phase,
            "failure_status": None if report is None else report.failure_status,
            "output_dir": str(result.output_dir),
        }
        print(json.dumps(summary, indent=2, ensure_ascii=False))
        if args.command == "demo" and result.engine is not None and result.plan is not None:
            from .config import WorkcellConfig
            from .viser_app import AssemblyViewer
            viewer = AssemblyViewer(
                result.engine,
                result.plan,
                result.layout,
                WorkcellConfig.load(args.config),
                target_image,
                result.output_dir,
            )
            viewer.serve_forever()
        elif result.engine is not None:
            result.engine.planner.destroy()
        return 0 if result.success else 1
    except (LayoutError, OSError, RuntimeError, ValueError, json.JSONDecodeError) as exc:
        if args.debug:
            raise
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2


def _run_sequence(args) -> int:
    from dataclasses import replace

    from .config import WorkcellConfig
    from .curobo_runner import StaticCubeObstacle
    from .sequence import (
        SequenceRun,
        arm_config,
        completed_cube_obstacles,
        write_shifted_layout,
    )
    from .sequence_viewer import DualArmSequenceViewer

    if args.arm_separation <= 0.0:
        raise ValueError("--arm-separation must be greater than zero")
    if args.letter_spacing <= 0.0:
        raise ValueError("--letter-spacing must be greater than zero")

    labels = ("U", "S", "T")
    arms = ("left", "right", "left")
    offsets = (-args.letter_spacing, 0.0, args.letter_spacing)
    output = Path(args.output).expanduser().resolve()
    combined_bounds = (
        min(offsets) - 200.0,
        max(offsets) + 200.0,
    )
    base_config = WorkcellConfig.load(args.config)
    arm_configs = {
        arm: arm_config(base_config, arm, args.arm_separation)
        for arm in ("left", "right")
    }
    images = args.target_images or (None, None, None)
    completed: list[StaticCubeObstacle] = []
    runs = []
    summaries = []
    viewer_engine = None

    for label, arm, offset, layout_path, target_image in zip(
        labels, arms, offsets, args.layouts, images
    ):
        shifted = write_shifted_layout(
            layout_path,
            output / "handoff" / f"{label}.json",
            offset,
            combined_bounds,
        )
        if viewer_engine is not None:
            viewer_engine.planner.destroy()
            viewer_engine = None
        result = run_pipeline(
            str(shifted),
            args.config,
            str(output / label),
            warmup=not args.skip_warmup,
            config_override=arm_configs[arm],
            static_cubes=tuple(completed),
        )
        if result.engine is None or result.plan is None:
            raise RuntimeError(
                f"{label} failed the structural gate and cannot enter the dual-arm sequence"
            )
        viewer_engine = result.engine
        runs.append(SequenceRun(
            label=label,
            arm=arm,
            layout=result.layout,
            plan=result.plan,
            config=arm_configs[arm],
            output_dir=result.output_dir,
            target_image=target_image,
        ))
        terminal = (
            result.engine.current_state.position.detach().cpu().numpy().reshape(-1)
        )
        arm_configs[arm] = replace(
            arm_configs[arm],
            initial_joint_positions=tuple(float(value) for value in terminal),
        )
        completed.extend(completed_cube_obstacles(
            label,
            result.layout,
            result.plan.report.completed_steps,
        ))
        summaries.append({
            "shape": label,
            "arm": arm,
            "success": result.success,
            "completed_steps": result.plan.report.completed_steps,
            "total_steps": len(result.layout.placements),
            "planned_phases": len(result.plan.phases),
            "expected_phases": len(result.layout.placements) * 6 + 1,
            "return_home_planned": bool(
                result.plan.phases and result.plan.phases[-1].name == "home"
            ),
            "failed_cube": result.plan.report.failed_cube,
            "failed_phase": result.plan.report.failed_phase,
            "failure_status": result.plan.report.failure_status,
            "output_dir": str(result.output_dir),
        })

    sequence_summary = {
        "schema_version": "1.0",
        "sequence": "U -> S -> T",
        "arm_assignment": {"U": "left", "S": "right", "T": "left"},
        "execution": "strictly serialized",
        "arm_separation_m": args.arm_separation,
        "letter_spacing_mm": args.letter_spacing,
        "completed_cube_collision_obstacles": len(completed),
        "runs": summaries,
        "output_dir": str(output),
    }
    output.mkdir(parents=True, exist_ok=True)
    (output / "sequence_manifest.json").write_text(
        json.dumps(sequence_summary, indent=2, ensure_ascii=False) + "\n",
        encoding="utf-8",
    )
    print(json.dumps(sequence_summary, indent=2, ensure_ascii=False))
    viewer = DualArmSequenceViewer(viewer_engine, runs, output)
    viewer.serve_forever()
    return 0 if all(item["success"] for item in summaries) else 1
