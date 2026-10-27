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
    return root


def main(argv=None) -> int:
    args = parser().parse_args(argv)
    try:
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
            "expected_phases": len(result.layout.placements) * 6,
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
