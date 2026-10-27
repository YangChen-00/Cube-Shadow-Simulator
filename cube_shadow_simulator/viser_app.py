"""Viser playback of the joint trajectories produced by cuRoboV2."""

from __future__ import annotations

import threading
import time
from pathlib import Path
from typing import Optional

import numpy as np

from .config import WorkcellConfig
from .curobo_runner import CuroboAssemblyPlanner, CuroboPlan
from .layout import Layout
from .transforms import cube_pose_in_base, matrix, matrix_pose, tabletop_point_mm


class AssemblyViewer:
    def __init__(
        self,
        engine: CuroboAssemblyPlanner,
        plan: CuroboPlan,
        layout: Layout,
        config: WorkcellConfig,
        target_image: Optional[str] = None,
    ):
        try:
            from curobo.types import ContentPath, JointState
            from curobo.viewer import ViserVisualizer
        except ImportError as exc:
            raise RuntimeError("cuRoboV2 Viser support is unavailable on this server") from exc
        self.engine = engine
        self.plan = plan
        self.layout = layout
        self.config = config
        self.JointState = JointState
        self.visualizer = ViserVisualizer(
            content_path=ContentPath(robot_config_file=config.robot_yaml),
            connect_ip=config.viser_host,
            connect_port=config.viser_port,
            add_control_frames=False,
            visualize_robot_spheres=False,
        )
        self.server = self.visualizer._server
        self.obstacle_handles = self._add_scene_boxes()
        self._playing = False
        self._stop = False
        self._add_lighting()
        self._add_gui(target_image)
        self._reset_scene()

    def _add_scene_boxes(self):
        """Create movable Viser nodes for every cuRobo cuboid.

        cuRobo's ``add_scene(..., add_control_frames=False)`` intentionally
        returns an empty dictionary: handles are only returned for interactive
        control frames.  Assembly playback needs ordinary movable scene-node
        handles instead, so mirror the same collision cuboids through Viser's
        public primitive API.
        """
        handles = {}
        for cuboid in self.engine.scene.cuboid:
            pose = np.asarray(cuboid.pose, dtype=float).reshape(-1)
            dims = np.asarray(cuboid.dims, dtype=float).reshape(-1)
            raw_color = cuboid.color
            if raw_color is None:
                raw_color = [0.75, 0.75, 0.75, 1.0]
            rgb = tuple(
                int(round(255.0 * value)) if float(value) <= 1.0
                else int(round(float(value)))
                for value in raw_color[:3]
            )
            handles[cuboid.name] = self.server.scene.add_box(
                name=f"/obstacles/{cuboid.name}",
                dimensions=tuple(dims[:3]),
                color=rgb,
                position=tuple(pose[:3]),
                wxyz=tuple(pose[3:7]),
                cast_shadow=True,
                receive_shadow=True,
            )
        return handles

    def _add_lighting(self) -> None:
        self.server.scene.configure_default_lights(enabled=False)
        light_position = tabletop_point_mm(self.layout.scene.light, self.config)
        self.server.scene.add_icosphere(
            "/fixed_light/marker", radius=0.025, color=(255, 225, 120),
            position=tuple(light_position), cast_shadow=False,
        )
        self.server.scene.add_light_point(
            "/fixed_light/source", color=(255, 238, 190),
            intensity=self.config.light_intensity, cast_shadow=True,
            position=tuple(light_position),
        )
        self.server.scene.add_light_ambient(
            "/ambient_light", color=(210, 220, 235), intensity=0.35
        )

    def _add_gui(self, target_image: Optional[str]) -> None:
        outcome = "可执行" if self.plan.report.success else "规划失败"
        self.status = self.server.gui.add_markdown(
            f"## Cube Shadow\ncuRoboV2 判定：**{outcome}**\n\n等待播放。"
        )
        self.speed = self.server.gui.add_slider(
            "播放速度", min=0.25, max=4.0, step=0.25, initial_value=1.0
        )
        play = self.server.gui.add_button("播放 cuRobo 轨迹")
        reset = self.server.gui.add_button("重置")

        @play.on_click
        def _(_) -> None:
            if not self._playing:
                threading.Thread(target=self._play, daemon=True).start()

        @reset.on_click
        def _(_) -> None:
            self._stop = True
            while self._playing:
                time.sleep(0.01)
            self._reset_scene()

        if target_image:
            path = Path(target_image).expanduser().resolve()
            if path.is_file():
                from PIL import Image
                image = np.asarray(Image.open(path).convert("RGB"))
                self.server.gui.add_image(image, label="目标阴影", format="png")

    def _joint_state(self, q: np.ndarray, joint_names):
        tensor = self.engine.torch.as_tensor(
            q, device="cuda", dtype=self.engine.torch.float32
        ).unsqueeze(0)
        return self.JointState.from_position(
            tensor, joint_names=joint_names
        )

    def _set_robot(self, q: np.ndarray, joint_names) -> None:
        self.visualizer.set_joint_state(self._joint_state(q, joint_names).squeeze(0))

    def _tool_matrix(self, q: np.ndarray, joint_names) -> np.ndarray:
        plan_state = self.engine.joint_state_from_waypoint(q, list(joint_names))
        state = self.engine.planner.compute_kinematics(plan_state)
        tool = state.tool_poses.get_link_pose(self.engine.planner.tool_frames[0])
        position = tool.position.reshape(-1, 3)[0].detach().cpu().numpy()
        quaternion = tool.quaternion.reshape(-1, 4)[0].detach().cpu().numpy()
        return matrix(position, quaternion)

    def _set_cube(self, cube_id: str, transform: np.ndarray) -> None:
        position, quaternion = matrix_pose(transform)
        handle = self.obstacle_handles[cube_id]
        handle.position = tuple(position)
        handle.wxyz = tuple(quaternion)

    def _reset_scene(self) -> None:
        self._stop = False
        self._set_robot(
            self.plan.initial_joint_positions, self.plan.initial_joint_names
        )
        for task in self.plan.tasks:
            position, quaternion = cube_pose_in_base(task.source, self.config)
            self._set_cube(task.cube_id, matrix(position, quaternion))
        self.status.content = (
            "## Cube Shadow\ncuRoboV2 已生成碰撞检查轨迹。\n\n点击“播放 cuRobo 轨迹”。"
            if self.plan.report.success else
            f"## Cube Shadow\n规划在 `{self.plan.report.failed_cube}/"
            f"{self.plan.report.failed_phase}` 失败。可播放已完成部分。"
        )

    def _play(self) -> None:
        self._playing = True
        self._stop = False
        self._reset_scene()
        targets = {task.cube_id: task for task in self.plan.tasks}
        try:
            for phase_index, phase in enumerate(self.plan.phases, 1):
                self.status.content = (
                    f"## Cube Shadow\n步骤 `{phase_index}/{len(self.plan.phases)}`："
                    f"`{phase.cube_id} · {phase.name}`"
                )
                for q in phase.joint_positions:
                    if self._stop:
                        return
                    self._set_robot(q, phase.joint_names)
                    if phase.holding_cube:
                        transform = self._tool_matrix(q, phase.joint_names) @ self.plan.tool_to_cube[phase.cube_id]
                        self._set_cube(phase.cube_id, transform)
                    delay = 1.0 / (self.config.playback_hz * float(self.speed.value))
                    time.sleep(delay)
                if phase.name == "place":
                    task = targets[phase.cube_id]
                    position, quaternion = cube_pose_in_base(task.target, self.config)
                    self._set_cube(phase.cube_id, matrix(position, quaternion))
            self.status.content = "## Cube Shadow\n**装配播放完成。** 固定点光源正在墙面投射最终阴影。"
        finally:
            self._playing = False

    def serve_forever(self) -> None:
        print(
            f"Viser is serving on http://{self.config.viser_host}:{self.config.viser_port} "
            "(use SSH port forwarding when running remotely)."
        )
        while True:
            time.sleep(10.0)
