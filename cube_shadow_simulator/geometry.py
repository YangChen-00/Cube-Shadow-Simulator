"""Convex 2-D footprint and simple 3-D collision helpers."""

import math
from typing import Iterable, List, Sequence, Tuple

from .models import Pose

Point = Tuple[float, float]


def square(pose: Pose, side: float) -> List[Point]:
    half = side / 2.0
    c, s = math.cos(math.radians(pose.yaw)), math.sin(math.radians(pose.yaw))
    result = []
    for x, y in ((-half, -half), (half, -half), (half, half), (-half, half)):
        result.append((pose.x + c * x - s * y, pose.y + s * x + c * y))
    return result


def polygon_area(poly: Sequence[Point]) -> float:
    return abs(sum(a[0] * b[1] - a[1] * b[0] for a, b in zip(poly, poly[1:] + poly[:1]))) / 2.0 if len(poly) >= 3 else 0.0


def _cross(a: Point, b: Point, p: Point) -> float:
    return (b[0] - a[0]) * (p[1] - a[1]) - (b[1] - a[1]) * (p[0] - a[0])


def clip(subject: Sequence[Point], boundary: Sequence[Point]) -> List[Point]:
    output = list(subject)
    for a, b in zip(boundary, boundary[1:] + boundary[:1]):
        source, output = output, []
        if not source:
            break
        previous = source[-1]
        prev_cross = _cross(a, b, previous)
        for current in source:
            curr_cross = _cross(a, b, current)
            if (curr_cross >= -1e-8) != (prev_cross >= -1e-8):
                denom = prev_cross - curr_cross
                ratio = prev_cross / denom if abs(denom) > 1e-12 else 0.0
                output.append((previous[0] + ratio * (current[0] - previous[0]), previous[1] + ratio * (current[1] - previous[1])))
            if curr_cross >= -1e-8:
                output.append(current)
            previous, prev_cross = current, curr_cross
    return output


def convex_hull(points: Iterable[Point]) -> List[Point]:
    pts = sorted(set(points))
    if len(pts) <= 1:
        return pts
    def turn(o: Point, a: Point, b: Point) -> float:
        return (a[0] - o[0]) * (b[1] - o[1]) - (a[1] - o[1]) * (b[0] - o[0])
    lower: List[Point] = []
    for p in pts:
        while len(lower) >= 2 and turn(lower[-2], lower[-1], p) <= 1e-9:
            lower.pop()
        lower.append(p)
    upper: List[Point] = []
    for p in reversed(pts):
        while len(upper) >= 2 and turn(upper[-2], upper[-1], p) <= 1e-9:
            upper.pop()
        upper.append(p)
    return lower[:-1] + upper[:-1]


def point_inside_with_margin(point: Point, polygon: Sequence[Point], margin: float) -> bool:
    if len(polygon) < 3:
        return False
    for a, b in zip(polygon, polygon[1:] + polygon[:1]):
        edge_length = math.hypot(b[0] - a[0], b[1] - a[1])
        if _cross(a, b, point) < margin * edge_length - 1e-7:
            return False
    return True


def footprints_overlap(a: Pose, b: Pose, side_a: float, side_b: float, tolerance_area: float = 1e-5) -> bool:
    return polygon_area(clip(square(a, side_a), square(b, side_b))) > tolerance_area


def volumes_overlap(a: Pose, b: Pose, side: float) -> bool:
    vertical = min(a.z + side / 2, b.z + side / 2) - max(a.z - side / 2, b.z - side / 2)
    return vertical > 1e-6 and footprints_overlap(a, b, side, side)


def footprint_in_bounds(pose: Pose, side: float, bounds: Tuple[float, float, float, float]) -> bool:
    xmin, xmax, ymin, ymax = bounds
    return all(xmin - 1e-7 <= x <= xmax + 1e-7 and ymin - 1e-7 <= y <= ymax + 1e-7 for x, y in square(pose, side))

