"""Dual-arm Viser playback for the U -> S -> T cuRobo showcase."""

from __future__ import annotations

import threading
import time
from collections import Counter
from datetime import datetime
from pathlib import Path

import numpy as np

from .curobo_runner import CuroboAssemblyPlanner
from .sequence import SequenceRun, base_pose_in_tabletop
from .transforms import matrix, matrix_pose, yaw_quaternion


class DualArmSequenceViewer:
    """Render two fixed-base arms while executing U, S, and T serially."""

    def __init__(
        self,
        engine: CuroboAssemblyPlanner,
        runs: list[SequenceRun],
        output_dir: str | Path,
    ):
        if not runs:
            raise ValueError("At least one sequence run is required")
        try:
            from curobo.types import ContentPath
            from curobo.viewer import ViserVisualizer
            from viser.extras import ViserUrdf
        except ImportError as exc:
            raise RuntimeError("cuRoboV2 dual-arm Viser support is unavailable") from exc

        self.engine = engine
        self.runs = runs
        self.config = runs[0].config
        self.output_dir = Path(output_dir).expanduser().resolve()
        self.visualizer = ViserVisualizer(
            content_path=ContentPath(robot_config_file=self.config.robot_yaml),
            connect_ip=self.config.viser_host,
            connect_port=self.config.viser_port,
            add_control_frames=False,
            visualize_robot_spheres=False,
            add_robot_to_scene=False,
        )
        self.server = self.visualizer._server
        # ViserVisualizer currently constructs one root URDF even when
        # add_robot_to_scene=False. Hide it instead of recursively removing it;
        # some Viser versions warn once per child during recursive removal.
        self.visualizer._viser_urdf.show_visual = False
        self._cache_viser_joint_defaults()
        self._playing = False
        self._recording = False
        self._stop = False

        self.arm_roots = {}
        self.arm_urdfs = {}
        for arm in ("left", "right"):
            run = next((item for item in runs if item.arm == arm), None)
            if run is None:
                raise ValueError(f"Sequence has no run assigned to {arm} arm")
            position, quaternion = base_pose_in_tabletop(run.config)
            self.arm_roots[arm] = self.server.scene.add_frame(
                f"/arms/{arm}",
                position=tuple(position),
                wxyz=tuple(quaternion),
                show_axes=False,
            )
            self.arm_urdfs[arm] = ViserUrdf(
                self.server,
                self.visualizer._urdf,
                root_node_name=f"/arms/{arm}",
            )

        self.cube_handles = self._add_environment_and_cubes()
        self._add_lighting()
        self._add_gui()
        self._reset_scene()

    def _add_environment_and_cubes(self):
        config = self.config
        all_x = [
            placement.pose.x
            for run in self.runs
            for placement in run.layout.placements
        ]
        all_y = [
            placement.pose.y
            for run in self.runs
            for placement in run.layout.placements
        ]
        all_x.extend(task.source.x for run in self.runs for task in run.plan.tasks)
        all_y.extend(task.source.y for run in self.runs for task in run.plan.tasks)
        margin = config.table_margin_m
        x0 = min(all_x) / 1000.0 - margin
        x1 = max(all_x) / 1000.0 + margin
        y0 = min(all_y) / 1000.0 - margin
        y1 = max(all_y) / 1000.0 + margin
        self.server.scene.add_box(
            "/environment/table",
            dimensions=(x1 - x0, y1 - y0, config.table_thickness_m),
            position=((x0 + x1) / 2.0, (y0 + y1) / 2.0, -config.table_thickness_m / 2.0),
            color=(107, 87, 66),
            cast_shadow=True,
            receive_shadow=True,
        )
        wall_x0 = min(run.layout.scene.wall_bounds[0] for run in self.runs) / 1000.0
        wall_x1 = max(run.layout.scene.wall_bounds[1] for run in self.runs) / 1000.0
        wall_z0 = min(run.layout.scene.wall_bounds[2] for run in self.runs) / 1000.0
        wall_z1 = max(run.layout.scene.wall_bounds[3] for run in self.runs) / 1000.0
        wall_y = self.runs[0].layout.scene.wall_y / 1000.0
        self.server.scene.add_box(
            "/environment/projection_wall",
            dimensions=(wall_x1 - wall_x0, config.wall_thickness_m, wall_z1 - wall_z0),
            position=((wall_x0 + wall_x1) / 2.0, wall_y, (wall_z0 + wall_z1) / 2.0),
            color=(240, 240, 235),
            cast_shadow=True,
            receive_shadow=True,
        )

        palette = {"U": (55, 126, 224), "S": (235, 151, 45), "T": (74, 170, 103)}
        handles = {}
        for run in self.runs:
            side = run.layout.scene.cube_size / 1000.0
            color = palette.get(run.label, (225, 122, 31))
            for task in run.plan.tasks:
                transform = self._pose_in_tabletop(task.source)
                position, quaternion = matrix_pose(transform)
                handle = self.server.scene.add_box(
                    f"/assemblies/{run.label}/{task.cube_id}",
                    dimensions=(side, side, side),
                    position=tuple(position),
                    wxyz=tuple(quaternion),
                    color=color,
                    cast_shadow=True,
                    receive_shadow=True,
                    visible=False,
                )
                handles[(run.label, task.cube_id)] = handle
        return handles

    def _add_lighting(self) -> None:
        self.server.scene.configure_default_lights(enabled=False)
        light = np.asarray(self.runs[0].layout.scene.light, dtype=float) / 1000.0
        self.server.scene.add_icosphere(
            "/fixed_light/marker",
            radius=0.025,
            color=(255, 225, 120),
            position=tuple(light),
            cast_shadow=False,
        )
        self.server.scene.add_light_point(
            "/fixed_light/source",
            color=(255, 238, 190),
            intensity=self.config.light_intensity,
            cast_shadow=True,
            position=tuple(light),
        )
        self.server.scene.add_light_ambient(
            "/ambient_light", color=(210, 220, 235), intensity=0.35
        )

    def _add_gui(self) -> None:
        labels = " → ".join(run.label for run in self.runs)
        self.status = self.server.gui.add_markdown(
            f"## Dual-arm Cube Shadow\n顺序：**{labels}**\n\n等待播放。"
        )
        self.speed = self.server.gui.add_slider(
            "播放速度", min=0.25, max=4.0, step=0.25, initial_value=1.0
        )
        self.play_button = self.server.gui.add_button("播放 U → S → T 双臂装配")
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
        self.record_button = self.server.gui.add_button("录制并下载 U-S-T 完整视频")

        for run in self.runs:
            if not run.target_image:
                continue
            path = Path(run.target_image).expanduser().resolve()
            if path.is_file():
                from PIL import Image

                image = np.asarray(Image.open(path).convert("RGB"))
                self.server.gui.add_image(image, label=f"{run.label} 目标阴影", format="png")

        @self.play_button.on_click
        def _(_) -> None:
            if self._playing or self._recording:
                return
            self._playing = True
            self._set_controls_disabled(True)
            threading.Thread(target=self._play, daemon=True).start()

        @self.record_button.on_click
        def _(event) -> None:
            if self._playing or self._recording or event.client is None:
                return
            self._recording = True
            self._set_controls_disabled(True)
            threading.Thread(target=self._record_video, args=(event.client,), daemon=True).start()

        @reset.on_click
        def _(_) -> None:
            self._stop = True
            while self._playing or self._recording:
                time.sleep(0.01)
            self._reset_scene()

    def _set_controls_disabled(self, disabled: bool) -> None:
        self.play_button.disabled = disabled
        self.record_button.disabled = disabled

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

    def _set_robot(self, arm: str, q: np.ndarray, joint_names) -> None:
        names = tuple(joint_names)
        values = np.asarray(q, dtype=float).reshape(-1)
        if values.size != len(names):
            raise ValueError(
                f"Trajectory has {values.size} values for {len(names)} joint names"
            )

        # A saved trajectory may contain some (but not all) locked joints. Calling
        # cuRobo get_full_js() on that mixed state raises because it tries to add
        # those joints a second time. Merge by name into a known-complete default
        # configuration instead; absent locked joints retain their URDF defaults.
        position = self._viser_default_positions.copy()
        unknown = [name for name in names if name not in self._viser_joint_indices]
        if unknown:
            raise ValueError(f"Trajectory contains joints absent from Viser: {unknown}")
        for name, value in zip(names, values):
            position[self._viser_joint_indices[name]] = value
        self.arm_urdfs[arm].update_cfg(position)

    def _tool_matrix(self, q: np.ndarray, joint_names) -> np.ndarray:
        plan_state = self.engine.joint_state_from_waypoint(q, list(joint_names))
        state = self.engine.planner.compute_kinematics(plan_state)
        tool = state.tool_poses.get_link_pose(self.engine.planner.tool_frames[0])
        position = tool.position.reshape(-1, 3)[0].detach().cpu().numpy()
        quaternion = tool.quaternion.reshape(-1, 4)[0].detach().cpu().numpy()
        return matrix(position, quaternion)

    @staticmethod
    def _pose_in_tabletop(pose) -> np.ndarray:
        return matrix(pose.metres(), yaw_quaternion(pose.yaw))

    def _set_cube(self, run: SequenceRun, cube_id: str, transform: np.ndarray) -> None:
        position, quaternion = matrix_pose(transform)
        handle = self.cube_handles[(run.label, cube_id)]
        handle.position = tuple(position)
        handle.wxyz = tuple(quaternion)

    def _reset_scene(self) -> None:
        self._stop = False
        for arm in ("left", "right"):
            run = next(item for item in self.runs if item.arm == arm)
            self._set_robot(arm, run.plan.initial_joint_positions, run.plan.initial_joint_names)
        for run in self.runs:
            for task in run.plan.tasks:
                self._set_cube(run, task.cube_id, self._pose_in_tabletop(task.source))
                self.cube_handles[(run.label, task.cube_id)].visible = False
        self.status.content = (
            "## Dual-arm Cube Shadow\n两台机械臂已就位。\n\n"
            "点击“播放 U → S → T 双臂装配”。"
        )

    def _activate_run(self, run: SequenceRun) -> None:
        self._set_robot(run.arm, run.plan.initial_joint_positions, run.plan.initial_joint_names)
        for index, task in enumerate(run.plan.tasks):
            self._set_cube(run, task.cube_id, self._pose_in_tabletop(task.source))
            self.cube_handles[(run.label, task.cube_id)].visible = (
                run.config.source_mode != "feeder" or index == 0
            )

    def _apply_waypoint(self, run: SequenceRun, phase, q: np.ndarray) -> None:
        self._set_robot(run.arm, q, phase.joint_names)
        if phase.holding_cube:
            transform = (
                run.base_to_tabletop
                @ self._tool_matrix(q, phase.joint_names)
                @ run.plan.tool_to_cube[phase.cube_id]
            )
            self._set_cube(run, phase.cube_id, transform)

    def _finish_place(self, run: SequenceRun, cube_id: str) -> None:
        task = next(task for task in run.plan.tasks if task.cube_id == cube_id)
        self._set_cube(run, cube_id, self._pose_in_tabletop(task.target))

    def _play(self) -> None:
        self._stop = False
        self._reset_scene()
        total_phases = sum(len(run.plan.phases) for run in self.runs)
        phase_counter = 0
        try:
            for run_index, run in enumerate(self.runs, 1):
                if self._stop:
                    return
                self._activate_run(run)
                for phase in run.plan.phases:
                    phase_counter += 1
                    if phase.name == "pre_pick":
                        self.cube_handles[(run.label, phase.cube_id)].visible = True
                    self.status.content = (
                        f"## Dual-arm Cube Shadow\n**{run.label} · {run.arm} arm** "
                        f"`{run_index}/{len(self.runs)}`\n\n"
                        f"轨迹 `{phase_counter}/{total_phases}`："
                        f"`{phase.cube_id} · {phase.name}`"
                    )
                    for q in phase.joint_positions:
                        if self._stop:
                            return
                        self._apply_waypoint(run, phase, q)
                        time.sleep(1.0 / (self.config.playback_hz * float(self.speed.value)))
                    if phase.name == "place":
                        self._finish_place(run, phase.cube_id)
                if not run.plan.report.success:
                    self.status.content = (
                        f"## Dual-arm Cube Shadow\n{run.label} 在 "
                        f"`{run.plan.report.failed_cube}/{run.plan.report.failed_phase}` "
                        f"规划失败，继续下一字母。"
                    )
                    time.sleep(1.0)
            succeeded = [run.label for run in self.runs if run.plan.report.success]
            self.status.content = (
                "## Dual-arm Cube Shadow\n**U → S → T 序列播放完成。**\n\n"
                f"cuRobo 成功字母：`{', '.join(succeeded) or '无'}`"
            )
        finally:
            self._playing = False
            self._set_controls_disabled(False)

    def _video_sample_schedule(self, output_fps: float, speed: float) -> Counter:
        times = []
        current = 0.0
        fallback_dt = 1.0 / self.config.playback_hz
        for run in self.runs:
            for phase in run.plan.phases:
                dt = float(phase.dt) if float(phase.dt) > 0.0 else fallback_dt
                for _ in phase.joint_positions:
                    times.append(current)
                    current += dt
        times.append(current)
        duration = current if current > 0.0 else 1.0
        output_count = max(2, int(np.ceil(duration / speed * output_fps)) + 1)
        requested = np.linspace(0.0, current, output_count)
        available = np.asarray(times, dtype=float)
        right = np.clip(np.searchsorted(available, requested, side="left"), 0, len(available) - 1)
        left = np.clip(right - 1, 0, len(available) - 1)
        selected = np.where(
            np.abs(requested - available[left]) <= np.abs(available[right] - requested),
            left,
            right,
        )
        return Counter(int(index) for index in selected)

    def _video_render_size(self, client) -> tuple[int, int]:
        requested = {
            "1280×720": (1280, 720),
            "960×540": (960, 540),
            "640×360": (640, 360),
        }[str(self.export_resolution.value)]
        scale = min(
            1.0,
            max(2, int(client.camera.image_width)) / requested[0],
            max(2, int(client.camera.image_height)) / requested[1],
        )
        return (
            max(2, int(requested[0] * scale) // 2 * 2),
            max(2, int(requested[1] * scale) // 2 * 2),
        )

    def _record_video(self, client) -> None:
        try:
            import imageio.v2 as imageio
        except ImportError:
            self.status.content = "## Dual-arm Cube Shadow\n请先重新执行 `pip install -e .`。"
            self._recording = False
            self._set_controls_disabled(False)
            return

        writer = None
        partial_path = None
        self._stop = False
        try:
            self._reset_scene()
            self.output_dir.mkdir(parents=True, exist_ok=True)
            speed = float(self.export_speed.value)
            fps = 30.0
            schedule = self._video_sample_schedule(fps, speed)
            total = sum(schedule.values())
            width, height = self._video_render_size(client)
            speed_tag = f"{speed:g}".replace(".", "p")
            stamp = datetime.now().strftime("%Y%m%d-%H%M%S-%f")
            filename = f"cube-shadow-U-S-T-{speed_tag}x-{stamp}.mp4"
            final_path = self.output_dir / filename
            partial_path = self.output_dir / f".{filename}.partial.mp4"
            writer = imageio.get_writer(
                str(partial_path), format="FFMPEG", mode="I", fps=fps,
                codec="libx264", macro_block_size=2, ffmpeg_log_level="error",
                output_params=["-pix_fmt", "yuv420p", "-movflags", "+faststart"],
            )
            source_index = 0
            written = 0

            def capture() -> None:
                nonlocal written
                repeats = schedule.get(source_index, 0)
                if not repeats:
                    return
                frame = client.get_render(
                    height=height, width=width, transport_format="jpeg", timeout=30.0
                )
                if frame.shape[-1] == 4:
                    frame = frame[..., :3]
                for _ in range(repeats):
                    writer.append_data(frame)
                written += repeats
                self.status.content = (
                    f"## Dual-arm Cube Shadow\n**正在导出 U-S-T MP4…** "
                    f"`{written}/{total}` 帧 · `{speed:g}×`"
                )

            for run in self.runs:
                self._activate_run(run)
                for phase in run.plan.phases:
                    if phase.name == "pre_pick":
                        self.cube_handles[(run.label, phase.cube_id)].visible = True
                    for q in phase.joint_positions:
                        if self._stop:
                            raise InterruptedError
                        self._apply_waypoint(run, phase, q)
                        capture()
                        source_index += 1
                    if phase.name == "place":
                        self._finish_place(run, phase.cube_id)
            capture()
            writer.close()
            writer = None
            partial_path.replace(final_path)
            client.send_file_download(final_path.name, final_path.read_bytes(), save_immediately=True)
            self.status.content = (
                "## Dual-arm Cube Shadow\n**U-S-T 视频已导出并开始下载。**\n\n"
                f"`{final_path.name}` · `{speed:g}×` · 30 FPS"
            )
        except InterruptedError:
            self.status.content = "## Dual-arm Cube Shadow\n录制已停止。"
        except Exception as exc:
            self.status.content = f"## Dual-arm Cube Shadow\n**视频导出失败：** `{exc}`"
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
            self._set_controls_disabled(False)

    def serve_forever(self) -> None:
        print("Dual-arm U-S-T Viser is ready; use the HTTP address in the Viser banner above.")
        while True:
            time.sleep(10.0)
