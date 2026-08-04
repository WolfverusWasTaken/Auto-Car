import math
from dataclasses import dataclass

import shapely
import numpy as np

SWERVE_DIRECTION = 1.0
SWERVE_LATERAL_OFFSET = 1.0
SWERVE_OBSTACLE_RADIUS = 8.0
SWERVE_MIN_GAP_WIDTH = 1.2
SWERVE_SPEED_LIMIT = 1.5


@dataclass
class SwervePlan:
    start_distance: float
    return_distance: float
    direction: float
    lateral_offset: float
    obstacle_count: int


def make_swerve_plan(path_msg, path_linestring, collision_points, collision_point_distances, target_idx,
                     ego_distance, distance_to_car_front, obstacle_radius=SWERVE_OBSTACLE_RADIUS,
                     lateral_offset=SWERVE_LATERAL_OFFSET, fallback_direction=SWERVE_DIRECTION,
                     min_gap_width=SWERVE_MIN_GAP_WIDTH):
    obstacle_indices = get_obstacle_group(collision_points, collision_point_distances, target_idx, ego_distance,
                                          path_linestring.length, obstacle_radius)

    if len(obstacle_indices) == 0:
        return None

    target_lateral_offset = choose_target_lateral_offset(path_linestring, collision_points, collision_point_distances,
                                                         obstacle_indices, collision_point_distances[target_idx],
                                                         lateral_offset, fallback_direction, min_gap_width)

    first_obstacle_distance = np.min(collision_point_distances[obstacle_indices])
    last_obstacle_distance = np.max(collision_point_distances[obstacle_indices])
    start_distance = max(ego_distance, first_obstacle_distance - obstacle_radius)
    return_distance = min(path_linestring.length, last_obstacle_distance + obstacle_radius)

    if return_distance <= start_distance + distance_to_car_front:
        return None

    apply_swerve_path(path_msg, path_linestring, start_distance, first_obstacle_distance,
                      last_obstacle_distance, return_distance, target_lateral_offset)

    return SwervePlan(start_distance=start_distance, return_distance=return_distance,
                      direction=np.sign(target_lateral_offset), lateral_offset=target_lateral_offset,
                      obstacle_count=len(obstacle_indices))


def get_obstacle_group(collision_points, collision_point_distances, target_idx, ego_distance, path_length, obstacle_radius):
    if collision_points[target_idx]["category"] not in (3, 4):
        return []

    obstacle_mask = (
        ((collision_points["category"] == 3) | (collision_points["category"] == 4)) &
        (collision_point_distances >= ego_distance) &
        (collision_point_distances <= path_length)
    )

    obstacle_indices = np.where(obstacle_mask)[0]
    if len(obstacle_indices) == 0:
        return []

    group_start = collision_point_distances[target_idx] - obstacle_radius
    group_end = collision_point_distances[target_idx] + obstacle_radius

    changed = True
    while changed:
        changed = False
        for idx in obstacle_indices:
            obstacle_start = collision_point_distances[idx] - obstacle_radius
            obstacle_end = collision_point_distances[idx] + obstacle_radius
            if obstacle_start <= group_end and obstacle_end >= group_start:
                new_start = min(group_start, obstacle_start)
                new_end = max(group_end, obstacle_end)
                changed = changed or new_start != group_start or new_end != group_end
                group_start = new_start
                group_end = new_end

    return [
        idx for idx in obstacle_indices
        if collision_point_distances[idx] - obstacle_radius <= group_end and
        collision_point_distances[idx] + obstacle_radius >= group_start
    ]


def choose_target_lateral_offset(path_linestring, collision_points, collision_point_distances, obstacle_indices,
                                 target_distance, lateral_offset, fallback_direction, min_gap_width):
    obstacle_sides = []

    for idx in obstacle_indices:
        obstacle_point = shapely.Point(collision_points[idx]["x"], collision_points[idx]["y"], collision_points[idx]["z"])
        obstacle_sides.append(get_obstacle_side(path_linestring, obstacle_point, collision_point_distances[idx]))

    obstacle_sides = np.array(obstacle_sides)
    left_sides = obstacle_sides[obstacle_sides > 0.1]
    right_sides = obstacle_sides[obstacle_sides < -0.1]

    if len(left_sides) > 0 and len(right_sides) > 0:
        nearest_left = np.min(left_sides)
        nearest_right = np.max(right_sides)
        gap_width = nearest_left - nearest_right

        if gap_width >= min_gap_width:
            gap_center = (nearest_left + nearest_right) / 2.0
            return float(np.clip(gap_center, -lateral_offset, lateral_offset))

    side_sum = 0.0

    for side, idx in zip(obstacle_sides, obstacle_indices):
        weight = 1.0 / (abs(collision_point_distances[idx] - target_distance) + 1.0)
        side_sum += side * weight

    if math.isclose(side_sum, 0.0, abs_tol=0.1):
        return fallback_direction * lateral_offset

    return -lateral_offset if side_sum > 0 else lateral_offset


def get_obstacle_side(path_linestring, obstacle_point, obstacle_distance):
    center_point = path_linestring.interpolate(obstacle_distance)
    heading = get_heading_at_distance(path_linestring, obstacle_distance)
    dx = obstacle_point.x - center_point.x
    dy = obstacle_point.y - center_point.y
    return math.cos(heading) * dy - math.sin(heading) * dx


def apply_swerve_path(path_msg, path_linestring, start_distance, first_obstacle_distance,
                      last_obstacle_distance, return_distance, target_lateral_offset):
    for wp in path_msg.waypoints:
        waypoint_point = shapely.Point(wp.position.x, wp.position.y, wp.position.z)
        waypoint_distance = path_linestring.project(waypoint_point)

        if waypoint_distance < start_distance or waypoint_distance > return_distance:
            continue

        offset = get_lateral_offset(waypoint_distance, start_distance, first_obstacle_distance,
                                    last_obstacle_distance, return_distance, target_lateral_offset)

        if math.isclose(offset, 0.0):
            continue

        heading = get_heading_at_distance(path_linestring, waypoint_distance)
        wp.position.x += -math.sin(heading) * offset
        wp.position.y += math.cos(heading) * offset


def get_lateral_offset(waypoint_distance, start_distance, first_obstacle_distance,
                       last_obstacle_distance, return_distance, lateral_offset):
    if waypoint_distance <= first_obstacle_distance:
        denominator = max(first_obstacle_distance - start_distance, 0.001)
        progress = (waypoint_distance - start_distance) / denominator
        return lateral_offset * math.sin(progress * math.pi / 2.0)

    if waypoint_distance <= last_obstacle_distance:
        return lateral_offset

    denominator = max(return_distance - last_obstacle_distance, 0.001)
    progress = (waypoint_distance - last_obstacle_distance) / denominator
    return lateral_offset * math.cos(progress * math.pi / 2.0)


def get_heading_at_distance(linestring, distance):
    point_after = linestring.interpolate(distance + 0.1)
    point_before = linestring.interpolate(max(0, distance - 0.1))
    return math.atan2(point_after.y - point_before.y, point_after.x - point_before.x)
