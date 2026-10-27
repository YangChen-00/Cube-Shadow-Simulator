"""cuRoboV2 motion planning for the complete cube assembly sequence."""

from __future__ import annotations

import math
from dataclasses import asdict, dataclass, field
from typing import Any, Dict, List, Optional

import numpy as np

from .config import WorkcellConfig
from .layout import Layout
from .models import ExecutionReport
from .planner import CubeTask, TaskPhase, make_tasks
from .transforms import cube_pose_in_base, tabletop_point_mm, tool_pose_over_cube


@dataclass
class PlannedPhase:
    key: str
    cube_id: str
    name: str
    holding_cube: bool
    joint_positions: np.ndarray
    joint_names: List[str]
    dt: float


@dataclass
class CuroboPlan:
    report: ExecutionReport
    initial_joint_positions: np.ndarray
    initial_joint_names: List[str] = field(default_factory=list)
    phases: List[PlannedPhase] = field(default_factory=list)
    tasks: List[CubeTask] = field(default_factory=list)
    tool_to_cube: Dict[str, np.ndarray] = field(default_factory=dict)

    def summary(self) -> Dict[str, Any]:
        return {
            "schema_version": "2.0",
            "planner": "cuRoboV2 MotionPlanner",
            "report": self.report.to_dict(),
            "initial_joint_positions": self.initial_joint_positions.tolist(),
            "initial_joint_names": self.initial_joint_names,
            "phases": [
                {
                    "key": p.key,
                    "cube_id": p.cube_id,
                    "name": p.name,
                    "holding_cube": p.holding_cube,
                    "waypoints": int(p.joint_positions.shape[0]),
                    "joint_names": p.joint_names,
                    "dt": p.dt,
                }
                for p in self.phases
            ],
            "tasks": [
                {
                    "cube_id": task.cube_id,
                    "step": task.step,
                    "source_mm_deg": asdict(task.source),
                    "target_mm_deg": asdict(task.target),
                    "allowed_place_contacts": list(task.supports) or ["table"],
                }
                for task in self.tasks
            ],
        }


class CuroboAssemblyPlanner:
    """Owns one cuRobo planner and its changing collision scene."""

    def __init__(self, layout: Layout, config: WorkcellConfig, warmup: bool = True):
        self.layout = layout
        self.config = config
        try:
            import torch
            import curobo
            from curobo.motion_planner import MotionPlanner, MotionPlannerCfg
            from curobo.scene import Cuboid, Scene
            from curobo.types import GoalToolPose, JointState, Pose
        except ImportError as exc:
            raise RuntimeError(
                "cuRoboV2 is not installed. Install NVlabs/cuRobo v0.8+ on the CUDA server."
            ) from exc
        if not torch.cuda.is_available():
            raise RuntimeError("cuRobo planning requires CUDA; run this command on the remote GPU server.")

        version = getattr(curobo, "__version__", "unknown")
        if version != "unknown" and _version_pair(version) < (0, 8):
            raise RuntimeError(f"cuRobo {version} is too old; this project uses the v0.8+ public API.")

        self.torch = torch
        self.CuroboPose = Pose
        self.GoalToolPose = GoalToolPose
        self.JointState = JointState
        self.version = version
        self.scene = self._make_scene(Scene, Cuboid)
        planner_cfg = MotionPlannerCfg.create(
            robot=config.robot_yaml,
            scene_model=self.scene,
            collision_cache={"cuboid": len(self.scene.cuboid) + 8},
            position_tolerance=config.position_tolerance_m,
            orientation_tolerance=config.orientation_tolerance_rad,
            optimizer_collision_activation_distance=config.collision_activation_distance_m,
            interpolation_dt=config.interpolation_dt,
        )
        self.planner = MotionPlanner(planner_cfg)
        self.plan_joint_names = list(self.planner.kinematics.joint_names)
        self.articulated_joint_names = list(
            getattr(self.planner.kinematics, "all_articulated_joint_names", []) or []
        )
        if warmup:
            self.planner.warmup(
                enable_graph=True,
                num_warmup_iterations=config.warmup_iterations,
            )
        self.current_state = self._initial_state()

    def _initial_state(self):
        if self.config.initial_joint_positions is None:
            position = self.planner.default_joint_state.position
        else:
            position = self.torch.tensor(
                self.config.initial_joint_positions,
                device="cuda",
                dtype=self.torch.float32,
            )
        position = position.reshape(-1)
        position = self._project_to_plan_space(position)
        return self.JointState.from_position(
            position.unsqueeze(0), joint_names=self.plan_joint_names
        )

    def _project_to_plan_space(self, position, source_names=None):
        """Drop locked joints from a full cuRobo waypoint before the next solve."""
        if position.shape[-1] == len(self.plan_joint_names):
            return position
        names = list(source_names or self.articulated_joint_names)
        if position.shape[-1] != len(names) or not all(
            name in names for name in self.plan_joint_names
        ):
            raise RuntimeError(
                f"Cannot map {position.shape[-1]} joints to cuRobo plan space "
                f"{self.plan_joint_names}; source roster is {names}"
            )
        indices = [names.index(name) for name in self.plan_joint_names]
        index = self.torch.tensor(indices, device=position.device, dtype=self.torch.long)
        return position.index_select(-1, index)

    def joint_state_from_waypoint(self, position: np.ndarray, joint_names: List[str]):
        tensor = self.torch.as_tensor(
            position, device="cuda", dtype=self.torch.float32
        ).reshape(-1)
        tensor = self._project_to_plan_space(tensor, joint_names)
        return self.JointState.from_position(
            tensor.unsqueeze(0), joint_names=self.plan_joint_names
        )

    def _make_scene(self, Scene, Cuboid):
        side = self.layout.scene.cube_size
        tasks = make_tasks(self.layout, self.config)
        xs = [p.pose.x for p in self.layout.placements] + [t.source.x for t in tasks]
        ys = [p.pose.y for p in self.layout.placements] + [t.source.y for t in tasks]
        build = self.layout.scene.build_bounds
        if build:
            xs += [build[0], build[1]]
            ys += [build[2], build[3]]
        margin_mm = self.config.table_margin_m * 1000.0
        x0, x1 = min(xs, default=-300) - margin_mm, max(xs, default=300) + margin_mm
        y0, y1 = min(ys, default=-300) - margin_mm, max(ys, default=300) + margin_mm
        table_center = tabletop_point_mm(
            ((x0 + x1) / 2, (y0 + y1) / 2, -self.config.table_thickness_m * 500),
            self.config,
        )
        tabletop_q = list(self.config.tabletop_quaternion_wxyz)
        cuboids = [Cuboid(
            name="table",
            dims=[(x1-x0)/1000.0, (y1-y0)/1000.0, self.config.table_thickness_m],
            pose=[*table_center.tolist(), *tabletop_q],
            color=[0.42, 0.34, 0.26, 1.0],
        )]

        wall_x0, wall_x1, wall_z0, wall_z1 = self.layout.scene.wall_bounds
        wall_center = tabletop_point_mm(
            ((wall_x0+wall_x1)/2, self.layout.scene.wall_y, (wall_z0+wall_z1)/2),
            self.config,
        )
        cuboids.append(Cuboid(
            name="projection_wall",
            dims=[(wall_x1-wall_x0)/1000.0, self.config.wall_thickness_m,
                  (wall_z1-wall_z0)/1000.0],
            pose=[*wall_center.tolist(), *tabletop_q],
            color=[0.94, 0.94, 0.92, 1.0],
        ))
        for task in tasks:
            position, quaternion = cube_pose_in_base(task.source, self.config)
            cuboids.append(Cuboid(
                name=task.cube_id,
                dims=[side/1000.0] * 3,
                pose=[*position.tolist(), *quaternion.tolist()],
                color=[0.90, 0.48, 0.12, 1.0],
            ))
        return Scene(cuboid=cuboids)

    def _goal(self, phase: TaskPhase):
        position, quaternion = tool_pose_over_cube(
            phase.goal, self.config, phase.vertical_offset_m
        )
        pos = self.torch.tensor(position, device="cuda", dtype=self.torch.float32)
        quat = self.torch.tensor(quaternion, device="cuda", dtype=self.torch.float32)
        return self.GoalToolPose(
            tool_frames=self.planner.tool_frames,
            position=pos.reshape(1, 1, 1, 1, 3),
            quaternion=quat.reshape(1, 1, 1, 1, 4),
        )

    def _plan_phase(
        self, phase: TaskPhase
    ) -> tuple[Optional[np.ndarray], List[str], str]:
        result = self.planner.plan_pose(
            self._goal(phase),
            self.current_state,
            max_attempts=self.config.max_attempts,
        )
        if result is None or result.success is None or not bool(result.success.any().item()):
            return None, [], str(getattr(result, "status", "plan_pose returned no solution"))
        trajectory = result.get_interpolated_plan()
        values = trajectory.position.detach().cpu().numpy()
        values = np.asarray(values).reshape(-1, values.shape[-1])
        trajectory_names = list(
            getattr(trajectory, "joint_names", None)
            or self.articulated_joint_names
            or self.plan_joint_names
        )
        if len(trajectory_names) != values.shape[-1]:
            if len(self.articulated_joint_names) == values.shape[-1]:
                trajectory_names = self.articulated_joint_names
            elif len(self.plan_joint_names) == values.shape[-1]:
                trajectory_names = self.plan_joint_names
            else:
                raise RuntimeError(
                    f"cuRobo returned {values.shape[-1]} trajectory joints, but the "
                    f"available joint rosters have {len(self.plan_joint_names)} plan "
                    f"joints and {len(self.articulated_joint_names)} articulated joints"
                )
        self.current_state = self.joint_state_from_waypoint(values[-1], trajectory_names)
        return values, trajectory_names, "success"

    def _cube_curobo_pose(self, task: CubeTask, target: bool = False):
        item = task.target if target else task.source
        position, quaternion = cube_pose_in_base(item, self.config)
        return self.CuroboPose.from_numpy(position, quaternion)

    def _attach_cube(self, task: CubeTask) -> np.ndarray:
        side = self.layout.scene.cube_size / 1000.0
        # franka.yml reserves four spheres for attached_object. Four spheres
        # centred on the XY quadrants conservatively enclose the full cube.
        radius = math.sqrt(6.0) * side / 4.0
        centres = [
            [sx*side/4, sy*side/4, 0.0, radius]
            for sx in (-1, 1) for sy in (-1, 1)
        ]
        spheres = self.torch.tensor(centres, device="cuda", dtype=self.torch.float32)
        self.planner.attachment_manager.update(
            spheres,
            self.current_state,
            link_name=self.config.attached_link_name,
            world_objects_pose_offset=self._cube_curobo_pose(task),
        )
        return self._tool_to_cube(task)

    def _tool_to_cube(self, task: CubeTask) -> np.ndarray:
        state = self.planner.compute_kinematics(self.current_state)
        tool = state.tool_poses.get_link_pose(self.planner.tool_frames[0])
        tool_position = tool.position.reshape(-1, 3)[0].detach().cpu().numpy()
        tool_quaternion = tool.quaternion.reshape(-1, 4)[0].detach().cpu().numpy()
        from .transforms import matrix
        cube_position, cube_quaternion = cube_pose_in_base(task.source, self.config)
        return np.linalg.inv(matrix(tool_position, tool_quaternion)) @ matrix(
            cube_position, cube_quaternion
        )

    def plan(self) -> CuroboPlan:
        tasks = make_tasks(self.layout, self.config)
        initial = self.current_state.position.detach().cpu().numpy().reshape(-1)
        phases: List[PlannedPhase] = []
        tool_to_cube: Dict[str, np.ndarray] = {}
        counts: Dict[str, int] = {}
        pick_table_relaxations: List[str] = []
        completed = 0
        failure: Optional[tuple[str, str, str]] = None

        for task in tasks:
            disabled_obstacles: set[str] = set()
            attached = False

            def disable_obstacle(name: str) -> None:
                if name not in disabled_obstacles:
                    self.planner.scene_collision_checker.enable_obstacle(
                        name, enable=False, env_idx=0
                    )
                    disabled_obstacles.add(name)

            def enable_obstacle(name: str) -> None:
                if name in disabled_obstacles:
                    self.planner.scene_collision_checker.enable_obstacle(
                        name, enable=True, env_idx=0
                    )
                    disabled_obstacles.remove(name)

            for phase in task.phases:
                if phase.name == "pick":
                    # Grasp contact is intentional. The cube is represented by
                    # attached spheres immediately after this phase succeeds.
                    disable_obstacle(task.cube_id)
                if phase.name == "lift":
                    # At the first lift waypoint the attached cube still rests
                    # on the supply table. Allow that contact until it is clear.
                    disable_obstacle("table")
                if phase.name == "place":
                    # The final descent intentionally ends in contact with the
                    # table or with cubes declared as supports by Studio.
                    for name in (list(task.supports) or ["table"]):
                        disable_obstacle(name)

                values, trajectory_names, status = self._plan_phase(phase)
                if (
                    values is None
                    and phase.name == "pick"
                    and "table" not in disabled_obstacles
                ):
                    # Prefer a fully collision-checked grasp. If the Franka
                    # hand's conservative collision spheres touch the support
                    # margin at the low grasp pose, retry only this contact
                    # segment with the supply table allowed. The table stays
                    # allowed through lift start and is restored after lift.
                    full_collision_status = status
                    disable_obstacle("table")
                    values, trajectory_names, status = self._plan_phase(phase)
                    if values is not None:
                        pick_table_relaxations.append(task.cube_id)
                    else:
                        status = (
                            f"full collision check: {full_collision_status}; "
                            f"supply-table contact retry: {status}"
                        )
                if values is None:
                    failure = (task.cube_id, phase.name, status)
                    break
                key = f"{task.step:03d}_{phase.name}"
                phases.append(PlannedPhase(
                    key, task.cube_id, phase.name, phase.holding_cube,
                    values, trajectory_names, self.config.interpolation_dt,
                ))
                counts[key] = int(values.shape[0])

                if phase.name == "pick":
                    tool_to_cube[task.cube_id] = self._attach_cube(task)
                    attached = True
                elif phase.name == "lift":
                    enable_obstacle("table")
                elif phase.name == "place":
                    self.planner.scene_collision_checker.update_obstacle_pose(
                        task.cube_id, self._cube_curobo_pose(task, target=True), env_idx=0
                    )
                    self.planner.attachment_manager.detach(
                        link_name=self.config.attached_link_name
                    )
                    attached = False
                    for name in (list(task.supports) or ["table"]):
                        enable_obstacle(name)
                    # Keep this cube disabled while the fingers withdraw from
                    # the grasp pose. It becomes an obstacle after retreat.
                elif phase.name == "retreat":
                    enable_obstacle(task.cube_id)
            if failure:
                if attached:
                    self.planner.attachment_manager.detach(
                        link_name=self.config.attached_link_name
                    )
                for name in tuple(disabled_obstacles):
                    enable_obstacle(name)
                break
            completed += 1

        notes = [
            "All robot feasibility claims come from cuRoboV2 plan_pose results.",
            "Source-table contact is allowed only for a low pick retry and the initial lift; target supports are allowed only during placement.",
            "A placed cube becomes a collision obstacle immediately after the gripper retreat.",
            "The carried cube is represented by four conservative attached collision spheres.",
        ]
        if pick_table_relaxations:
            notes.append(
                "Supply-table contact relaxation was required at pick for: "
                + ", ".join(pick_table_relaxations)
            )

        report = ExecutionReport(
            success=failure is None,
            completed_steps=completed,
            total_steps=len(tasks),
            curobo_version=self.version,
            failed_cube=None if failure is None else failure[0],
            failed_phase=None if failure is None else failure[1],
            failure_status=None if failure is None else failure[2],
            phase_waypoints=counts,
            notes=notes,
        )
        return CuroboPlan(
            report=report,
            initial_joint_positions=initial,
            initial_joint_names=self.plan_joint_names,
            phases=phases,
            tasks=tasks,
            tool_to_cube=tool_to_cube,
        )


def _version_pair(value: str) -> tuple[int, int]:
    try:
        parts = value.split("+")[0].split(".")
        return int(parts[0]), int(parts[1])
    except (ValueError, IndexError):
        return 999, 999
