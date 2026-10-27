"""Viser playback of the joint trajectories produced by cuRoboV2."""

from __future__ import annotations

import threading
import time
from collections import Counter
from datetime import datetime
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
        output_dir: Optional[str | Path] = None,
    ):
        try:
            from curobo.types import ContentPath
            from curobo.viewer import ViserVisualizer
        except ImportError as exc:
            raise RuntimeError("cuRoboV2 Viser support is unavailable on this server") from exc
        self.engine = engine
        self.plan = plan
        self.layout = layout
        self.config = config
        self.output_dir = Path(output_dir or "runs/latest").expanduser().resolve()
        self.visualizer = ViserVisualizer(
            content_path=ContentPath(robot_config_file=config.robot_yaml),
            connect_ip=config.viser_host,
            connect_port=config.viser_port,
            add_control_frames=False,
            visualize_robot_spheres=False,
        )
        self._cache_viser_joint_defaults()
        self.server = self.visualizer._server
        self.obstacle_handles = self._add_scene_boxes()
        self._playing = False
        self._recording = False
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
        play_label = "播放完整装配" if self.plan.report.success else "播放失败前轨迹"
        self.play_button = self.server.gui.add_button(play_label)
        reset = self.server.gui.add_button("重置")

        self.server.gui.add_markdown("### 视频导出")
        self.export_speed = self.server.gui.add_slider(
            "下载视频速度", min=0.25, max=4.0, step=0.25, initial_value=1.0
        )
        self.export_resolution = self.server.gui.add_dropdown(
            "视频清晰度",
            options=("1280×720", "960×540", "640×360"),
            initial_value="960×540",
        )
        record_label = (
            "录制并下载完整装配视频"
            if self.plan.report.success
            else "录制并下载失败前视频"
        )
        self.record_button = self.server.gui.add_button(record_label)
        self.server.gui.add_markdown(
            "导出使用当前浏览器视角，生成 30 FPS MP4；"
            "滑块选择的速度会直接改变视频时长。"
        )

        @self.play_button.on_click
        def _(_) -> None:
            if not self._playing and not self._recording:
                self._playing = True
                self.play_button.disabled = True
                self.record_button.disabled = True
                threading.Thread(target=self._play, daemon=True).start()

        @self.record_button.on_click
        def _(event) -> None:
            if self._playing or self._recording or event.client is None:
                return
            self._recording = True
            self.play_button.disabled = True
            self.record_button.disabled = True
            threading.Thread(
                target=self._record_video,
                args=(event.client,),
                daemon=True,
            ).start()

        @reset.on_click
        def _(_) -> None:
            self._stop = True
            while self._playing or self._recording:
                time.sleep(0.01)
            self._reset_scene()

        if target_image:
            path = Path(target_image).expanduser().resolve()
            if path.is_file():
                from PIL import Image
                image = np.asarray(Image.open(path).convert("RGB"))
                self.server.gui.add_image(image, label="目标阴影", format="png")

    def _cache_viser_joint_defaults(self) -> None:
        """Cache a complete display configuration, including locked joints."""
        kinematics = self.visualizer._kinematics
        full_default = kinematics.get_full_js(
            kinematics.default_joint_state.clone()
        )
        self._viser_joint_names = tuple(self.visualizer._viser_joint_names)
        self._viser_joint_indices = {
            name: index for index, name in enumerate(self._viser_joint_names)
        }
        full_default.reindex(self._viser_joint_names)
        self._viser_default_positions = (
            full_default.position.detach().cpu().numpy().reshape(-1).copy()
        )

        if self._viser_default_positions.size != len(self._viser_joint_names):
            raise RuntimeError(
                "Viser robot model returned an inconsistent default joint state"
            )

    def _set_robot(self, q: np.ndarray, joint_names) -> None:
        names = tuple(joint_names)
        values = np.asarray(q, dtype=float).reshape(-1)
        if values.size != len(names):
            raise ValueError(
                f"Trajectory has {values.size} values for {len(names)} joint names"
            )

        # Avoid ViserVisualizer.set_joint_state() here: its get_full_js() path
        # cannot accept a waypoint that already contains only some locked joints.
        # plan_grasp waypoints also include gripper joints (panda_finger_joint1/2)
        # that the exported display URDF does not actuate. Those keep their defaults.
        position = self._viser_default_positions.copy()
        applied = 0
        for name, value in zip(names, values):
            index = self._viser_joint_indices.get(name)
            if index is None:
                continue
            position[index] = value
            applied += 1
        if applied == 0:
            raise ValueError(
                "Trajectory joints do not overlap the Viser model: "
                f"{list(names)} vs {list(self._viser_joint_names)}"
            )
        self.visualizer._viser_urdf.update_cfg(position)

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
        for task_index, task in enumerate(self.plan.tasks):
            position, quaternion = cube_pose_in_base(task.source, self.config)
            self._set_cube(task.cube_id, matrix(position, quaternion))
            if self.config.source_mode == "feeder":
                self.obstacle_handles[task.cube_id].visible = task_index == 0
        self.status.content = (
            "## Cube Shadow\ncuRoboV2 已生成碰撞检查轨迹。\n\n点击“播放完整装配”。"
            if self.plan.report.success else
            f"## Cube Shadow\n规划在 `{self.plan.report.failed_cube}/"
            f"{self.plan.report.failed_phase}` 失败："
            f"`{self.plan.report.failure_status}`\n\n可播放失败前轨迹。"
        )

    def _play(self) -> None:
        self._playing = True
        self._stop = False
        self._reset_scene()
        targets = {task.cube_id: task for task in self.plan.tasks}
        try:
            for phase_index, phase in enumerate(self.plan.phases, 1):
                if phase.name == "pre_pick":
                    self.obstacle_handles[phase.cube_id].visible = True
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
            if self.plan.report.success:
                self.status.content = (
                    "## Cube Shadow\n**完整装配播放完成。** "
                    "固定点光源正在墙面投射最终阴影。"
                )
            else:
                self.status.content = (
                    "## Cube Shadow\n已播放全部可用轨迹，但装配未完成。\n\n"
                    f"cuRobo 在 `{self.plan.report.failed_cube}/"
                    f"{self.plan.report.failed_phase}` 规划失败："
                    f"`{self.plan.report.failure_status}`"
                )
        finally:
            self._playing = False
            self.play_button.disabled = False
            self.record_button.disabled = False

    def _video_sample_schedule(self, output_fps: float, speed: float) -> Counter:
        """Map source waypoint indices to the number of MP4 frames to write."""
        source_times = []
        source_time = 0.0
        fallback_dt = 1.0 / self.config.playback_hz
        for phase in self.plan.phases:
            dt = float(phase.dt) if float(phase.dt) > 0.0 else fallback_dt
            for _ in phase.joint_positions:
                source_times.append(source_time)
                source_time += dt

        # The extra state is the finished layout after the final place snap.
        source_times.append(source_time)
        duration = source_time if source_time > 0.0 else 1.0
        output_count = max(2, int(np.ceil(duration / speed * output_fps)) + 1)
        requested_times = np.linspace(0.0, source_time, output_count)
        source_times_array = np.asarray(source_times, dtype=float)
        right = np.searchsorted(source_times_array, requested_times, side="left")
        right = np.clip(right, 0, len(source_times_array) - 1)
        left = np.clip(right - 1, 0, len(source_times_array) - 1)
        use_left = (
            np.abs(requested_times - source_times_array[left])
            <= np.abs(source_times_array[right] - requested_times)
        )
        selected = np.where(use_left, left, right)
        return Counter(int(index) for index in selected)

    def _video_render_size(self, client) -> tuple[int, int]:
        requested = {
            "1280×720": (1280, 720),
            "960×540": (960, 540),
            "640×360": (640, 360),
        }[str(self.export_resolution.value)]
        viewport_width = max(2, int(client.camera.image_width))
        viewport_height = max(2, int(client.camera.image_height))
        scale = min(
            1.0,
            viewport_width / requested[0],
            viewport_height / requested[1],
        )
        width = max(2, int(requested[0] * scale) // 2 * 2)
        height = max(2, int(requested[1] * scale) // 2 * 2)
        return width, height

    def _record_video(self, client) -> None:
        """Render the cuRobo trajectory in the connected browser and download MP4."""
        try:
            import imageio.v2 as imageio
        except ImportError:
            self.status.content = (
                "## Cube Shadow\n**视频导出不可用：** 请重新执行 `pip install -e .` "
                "安装 imageio-ffmpeg。"
            )
            self._recording = False
            self.play_button.disabled = False
            self.record_button.disabled = False
            return

        self._stop = False
        writer = None
        partial_path = None
        try:
            self._reset_scene()
            self._stop = False
            self.output_dir.mkdir(parents=True, exist_ok=True)
            speed = float(self.export_speed.value)
            output_fps = 30.0
            schedule = self._video_sample_schedule(output_fps, speed)
            output_frames = sum(schedule.values())
            width, height = self._video_render_size(client)
            speed_tag = f"{speed:g}".replace(".", "p")
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            filename = f"cube-shadow-{speed_tag}x-{stamp}.mp4"
            final_path = self.output_dir / filename
            partial_path = self.output_dir / f".{filename}.partial.mp4"
            writer = imageio.get_writer(
                str(partial_path),
                format="FFMPEG",
                mode="I",
                fps=output_fps,
                codec="libx264",
                macro_block_size=2,
                ffmpeg_log_level="error",
                output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
            )
            targets = {task.cube_id: task for task in self.plan.tasks}
            written = 0
            source_index = 0

            def capture_current_state() -> None:
                nonlocal written
                repeats = schedule.get(source_index, 0)
                if repeats == 0:
                    return
                frame = client.get_render(
                    height=height,
                    width=width,
                    transport_format="jpeg",
                    timeout=30.0,
                )
                if frame.shape[-1] == 4:
                    frame = frame[..., :3]
                for _ in range(repeats):
                    writer.append_data(frame)
                written += repeats
                self.status.content = (
                    "## Cube Shadow\n**正在录制 MP4…** "
                    f"`{written}/{output_frames}` 帧 · `{speed:g}×` · "
                    f"`{width}×{height}`"
                )

            for phase_index, phase in enumerate(self.plan.phases, 1):
                if phase.name == "pre_pick":
                    self.obstacle_handles[phase.cube_id].visible = True
                for q in phase.joint_positions:
                    if self._stop:
                        raise InterruptedError("用户停止了录制")
                    self._set_robot(q, phase.joint_names)
                    if phase.holding_cube:
                        transform = (
                            self._tool_matrix(q, phase.joint_names)
                            @ self.plan.tool_to_cube[phase.cube_id]
                        )
                        self._set_cube(phase.cube_id, transform)
                    capture_current_state()
                    source_index += 1
                if phase.name == "place":
                    task = targets[phase.cube_id]
                    position, quaternion = cube_pose_in_base(task.target, self.config)
                    self._set_cube(phase.cube_id, matrix(position, quaternion))
                self.status.content = (
                    "## Cube Shadow\n**正在录制 MP4…** "
                    f"轨迹 `{phase_index}/{len(self.plan.phases)}`"
                )

            capture_current_state()
            writer.close()
            writer = None
            partial_path.replace(final_path)
            client.send_file_download(
                final_path.name,
                final_path.read_bytes(),
                save_immediately=True,
            )
            self.status.content = (
                "## Cube Shadow\n**视频已导出并开始下载。**\n\n"
                f"`{final_path.name}` · `{speed:g}×` · `{width}×{height}` · 30 FPS\n\n"
                f"服务器备份：`{final_path}`"
            )
        except InterruptedError:
            self.status.content = "## Cube Shadow\n录制已停止，未完成的视频已丢弃。"
        except Exception as exc:
            self.status.content = f"## Cube Shadow\n**视频导出失败：** `{exc}`"
            client.add_notification(
                title="视频导出失败",
                body=str(exc),
                color="red",
                auto_close_seconds=None,
            )
        finally:
            if writer is not None:
                try:
                    writer.close()
                except Exception:
                    pass
            if partial_path is not None and partial_path.exists():
                try:
                    partial_path.unlink()
                except OSError:
                    pass
            self._recording = False
            self.play_button.disabled = False
            self.record_button.disabled = False

    def serve_forever(self) -> None:
        print(
            "Viser is ready; use the HTTP address in the Viser banner above "
            "(and forward that actual port over SSH when running remotely)."
        )
        while True:
            time.sleep(10.0)
