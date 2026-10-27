"""Invoke the sibling Studio without importing or duplicating its internals."""

import subprocess
import sys
from pathlib import Path


def generate_layout(drawing: str, studio_root: str, output_dir: str, budget: int = 32, seconds: float = 5.0) -> Path:
    root = Path(studio_root).expanduser().resolve()
    if not (root / "cube_shadow/__main__.py").exists():
        raise RuntimeError(f"Not a Cube-Shadow-Studio checkout: {root}")
    image = Path(drawing).expanduser().resolve()
    if not image.is_file():
        raise RuntimeError(f"Drawing does not exist: {image}")
    destination = Path(output_dir).expanduser().resolve()
    destination.mkdir(parents=True, exist_ok=True)
    candidates = [root / ".venv/bin/python", root / ".venv/Scripts/python.exe"]
    python = next((candidate for candidate in candidates if candidate.exists()), Path(sys.executable))
    command = [
        str(python), "-B", "-m", "cube_shadow", str(image),
        "--budget", str(budget), "--seconds", str(seconds), "--output", str(destination),
    ]
    completed = subprocess.run(command, cwd=root, text=True, capture_output=True)
    if completed.returncode:
        detail = (completed.stderr or completed.stdout).strip().splitlines()
        tail = "\n".join(detail[-12:])
        raise RuntimeError(f"Studio layout generation failed (exit {completed.returncode}):\n{tail}")
    layout = destination / "layout.json"
    if not layout.exists():
        raise RuntimeError(f"Studio finished without producing {layout}")
    return layout

