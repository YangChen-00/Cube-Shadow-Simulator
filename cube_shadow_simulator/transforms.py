"""Small rigid-transform helpers using cuRobo's wxyz quaternion convention."""

from __future__ import annotations

import math
from typing import Iterable, Tuple

import numpy as np

from .config import WorkcellConfig
from .models import Pose


def quaternion_multiply(a: Iterable[float], b: Iterable[float]) -> np.ndarray:
    aw, ax, ay, az = a
    bw, bx, by, bz = b
    return np.asarray([
        aw * bw - ax * bx - ay * by - az * bz,
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
    ], dtype=np.float64)


def yaw_quaternion(degrees: float) -> np.ndarray:
    half = math.radians(degrees) / 2.0
    return np.asarray([math.cos(half), 0.0, 0.0, math.sin(half)])


def quaternion_matrix(q: Iterable[float]) -> np.ndarray:
    w, x, y, z = np.asarray(tuple(q), dtype=np.float64)
    return np.asarray([
        [1 - 2 * (y*y + z*z), 2 * (x*y - z*w), 2 * (x*z + y*w)],
        [2 * (x*y + z*w), 1 - 2 * (x*x + z*z), 2 * (y*z - x*w)],
        [2 * (x*z - y*w), 2 * (y*z + x*w), 1 - 2 * (x*x + y*y)],
    ])


def tabletop_point_mm(point: Iterable[float], cfg: WorkcellConfig) -> np.ndarray:
    local = np.asarray(tuple(point), dtype=np.float64) / 1000.0
    rotation = quaternion_matrix(cfg.tabletop_quaternion_wxyz)
    return np.asarray(cfg.tabletop_translation_m) + rotation @ local


def cube_pose_in_base(pose: Pose, cfg: WorkcellConfig) -> Tuple[np.ndarray, np.ndarray]:
    position = tabletop_point_mm((pose.x, pose.y, pose.z), cfg)
    quaternion = quaternion_multiply(cfg.tabletop_quaternion_wxyz, yaw_quaternion(pose.yaw))
    return position, quaternion


def tool_pose_over_cube(
    pose: Pose, cfg: WorkcellConfig, vertical_offset_m: float = 0.0
) -> Tuple[np.ndarray, np.ndarray]:
    local = np.asarray([
        pose.x / 1000.0,
        pose.y / 1000.0,
        pose.z / 1000.0 + cfg.tcp_to_cube_center_m + vertical_offset_m,
    ])
    position = np.asarray(cfg.tabletop_translation_m) + quaternion_matrix(
        cfg.tabletop_quaternion_wxyz
    ) @ local
    orientation_local = quaternion_multiply(
        yaw_quaternion(pose.yaw), cfg.tool_down_quaternion_tabletop_wxyz
    )
    quaternion = quaternion_multiply(cfg.tabletop_quaternion_wxyz, orientation_local)
    return position, quaternion


def matrix(position: Iterable[float], quaternion: Iterable[float]) -> np.ndarray:
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = quaternion_matrix(quaternion)
    result[:3, 3] = np.asarray(tuple(position))
    return result


def matrix_pose(value: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    rotation = value[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        q = np.asarray([
            0.25 * s,
            (rotation[2, 1] - rotation[1, 2]) / s,
            (rotation[0, 2] - rotation[2, 0]) / s,
            (rotation[1, 0] - rotation[0, 1]) / s,
        ])
    else:
        index = int(np.argmax(np.diag(rotation)))
        if index == 0:
            s = math.sqrt(1 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2
            q = np.asarray([(rotation[2, 1]-rotation[1, 2])/s, .25*s,
                            (rotation[0, 1]+rotation[1, 0])/s,
                            (rotation[0, 2]+rotation[2, 0])/s])
        elif index == 1:
            s = math.sqrt(1 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2
            q = np.asarray([(rotation[0, 2]-rotation[2, 0])/s,
                            (rotation[0, 1]+rotation[1, 0])/s, .25*s,
                            (rotation[1, 2]+rotation[2, 1])/s])
        else:
            s = math.sqrt(1 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2
            q = np.asarray([(rotation[1, 0]-rotation[0, 1])/s,
                            (rotation[0, 2]+rotation[2, 0])/s,
                            (rotation[1, 2]+rotation[2, 1])/s, .25*s])
    return value[:3, 3].copy(), q / np.linalg.norm(q)
