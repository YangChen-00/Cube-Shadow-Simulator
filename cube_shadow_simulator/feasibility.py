"""Fast structural gate; robot feasibility is decided only by cuRobo."""

from typing import Dict, List

from .geometry import clip, convex_hull, footprint_in_bounds, point_inside_with_margin, polygon_area, square, volumes_overlap
from .layout import Layout
from .models import FeasibilityReport, Issue


def assess_layout(layout: Layout, support_margin_mm: float = 4.0) -> FeasibilityReport:
    scene, cubes = layout.scene, layout.placements
    issues: List[Issue] = []
    categories: Dict[str, bool] = {
        "schema": True,
        "build_area": True,
        "static_collision": True,
        "support_and_gravity": True,
        "assembly_order": True,
    }
    side = scene.cube_size
    by_id = {cube.id: cube for cube in cubes}
    placed = []
    for cube in cubes:
        pose = cube.pose
        if scene.build_bounds and not footprint_in_bounds(pose, side, scene.build_bounds):
            categories["build_area"] = False
            issues.append(Issue("OUTSIDE_BUILD_AREA", "Rotated cube footprint leaves the permitted tabletop area.", cube.id, cube.step))
        expected_ground_z = side / 2.0
        if cube.grounded and abs(pose.z - expected_ground_z) > 1e-5:
            categories["support_and_gravity"] = False
            issues.append(Issue("BAD_GROUND_HEIGHT", f"Grounded cube centre must be z={expected_ground_z:g} mm.", cube.id, cube.step))
        if not cube.grounded and not cube.contacts_below:
            categories["support_and_gravity"] = False
            issues.append(Issue("FLOATING_CUBE", "Non-grounded cube has no support; gravity would make it fall.", cube.id, cube.step))
        for previous in placed:
            if volumes_overlap(pose, previous.pose, side):
                categories["static_collision"] = False
                issues.append(Issue("CUBE_OVERLAP", f"Final volume overlaps {previous.id}.", cube.id, cube.step))

        support_patches = []
        for support_id in cube.contacts_below:
            support = by_id[support_id]
            if support.step >= cube.step:
                categories["assembly_order"] = False
                issues.append(Issue("LATE_SUPPORT", f"Support {support_id} is not placed earlier.", cube.id, cube.step))
                continue
            expected_z = support.pose.z + side
            if abs(pose.z - expected_z) > 1e-5:
                categories["support_and_gravity"] = False
                issues.append(Issue("SUPPORT_HEIGHT_MISMATCH", f"Support {support_id} top does not meet the cube bottom.", cube.id, cube.step))
                continue
            patch = clip(square(pose, side), square(support.pose, side))
            if polygon_area(patch) <= 1e-5:
                categories["support_and_gravity"] = False
                issues.append(Issue("NO_CONTACT_AREA", f"Declared support {support_id} has no contact area.", cube.id, cube.step))
            else:
                support_patches.extend(patch)
        if support_patches:
            support_hull = convex_hull(support_patches)
            if not point_inside_with_margin((pose.x, pose.y), support_hull, support_margin_mm):
                categories["support_and_gravity"] = False
                issues.append(Issue("UNSTABLE_CENTRE", "Cube centre is outside the support polygon after the safety margin.", cube.id, cube.step))

        placed.append(cube)

    # Propagate the load of every already planned cube down the support graph.
    # This catches a leaning tower whose individual cubes appear locally valid
    # but whose combined centre of mass tips a lower contact.
    masses = {cube.id: 1.0 for cube in cubes}
    moments = {cube.id: [cube.pose.x, cube.pose.y] for cube in cubes}
    for cube in reversed(cubes):
        mass = masses[cube.id]
        centre = (moments[cube.id][0] / mass, moments[cube.id][1] / mass)
        if cube.grounded:
            support_region = square(cube.pose, side)
        else:
            contact_vertices = []
            for support_id in cube.contacts_below:
                contact_vertices.extend(clip(square(cube.pose, side), square(by_id[support_id].pose, side)))
            support_region = convex_hull(contact_vertices)
        if support_region and not point_inside_with_margin(centre, support_region, support_margin_mm):
            categories["support_and_gravity"] = False
            issues.append(Issue("UNSTABLE_SUBTREE", "Combined centre of mass above this contact leaves the support polygon.", cube.id, cube.step))
        if cube.contacts_below:
            share = 1.0 / len(cube.contacts_below)
            for support_id in cube.contacts_below:
                masses[support_id] += mass * share
                moments[support_id][0] += moments[cube.id][0] * share
                moments[support_id][1] += moments[cube.id][1] * share

    max_height = max((c.pose.z + side / 2.0 for c in cubes), default=0.0)
    return FeasibilityReport(
        feasible=not issues,
        checks=categories,
        issues=issues,
        cube_count=len(cubes),
        max_height_mm=max_height,
    )

