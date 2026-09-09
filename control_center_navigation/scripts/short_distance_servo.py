#!/usr/bin/env python3
"""Pure helpers for safe, deterministic short-distance chassis servoing."""

import math


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def clamp(value: float, lower: float, upper: float) -> float:
    return max(lower, min(upper, value))


def body_frame_error(current_pose, target_pose):
    """Return target X/Y error in the robot frame and shortest yaw error."""
    current_x, current_y, current_yaw = current_pose
    target_x, target_y, target_yaw = target_pose
    dx = target_x - current_x
    dy = target_y - current_y
    cos_yaw = math.cos(current_yaw)
    sin_yaw = math.sin(current_yaw)
    error_x = cos_yaw * dx + sin_yaw * dy
    error_y = -sin_yaw * dx + cos_yaw * dy
    return error_x, error_y, normalize_angle(target_yaw - current_yaw)


def apply_body_offset(reference_pose, body_offset):
    """Apply an SE(2) body-frame offset to a pose in an arbitrary frame."""
    reference_x, reference_y, reference_yaw = reference_pose
    offset_x, offset_y, offset_yaw = body_offset
    cos_yaw = math.cos(reference_yaw)
    sin_yaw = math.sin(reference_yaw)
    return (
        reference_x + cos_yaw * offset_x - sin_yaw * offset_y,
        reference_y + sin_yaw * offset_x + cos_yaw * offset_y,
        normalize_angle(reference_yaw + offset_yaw),
    )


def reproject_target_pose(current_reference_pose, current_map_pose, target_map_pose):
    """Express a fixed map target in a smooth local reference frame.

    `current_reference_pose` is normally odom->base_link.  The target is
    derived from the current map error, so map->odom corrections can be
    admitted deliberately instead of entering the velocity loop as a jump.
    """
    return apply_body_offset(
        current_reference_pose,
        body_frame_error(current_map_pose, target_map_pose),
    )


def bounded_pose_update(current_pose, desired_pose, *, gain, max_xy_step, max_yaw_step):
    """Move a local-frame target toward a map-derived target with hard bounds."""
    current_x, current_y, current_yaw = current_pose
    desired_x, desired_y, desired_yaw = desired_pose
    dx = desired_x - current_x
    dy = desired_y - current_y
    distance = math.hypot(dx, dy)
    step = min(max_xy_step, max(0.0, gain) * distance)
    if distance > 0.0 and step < distance:
        scale = step / distance
        dx *= scale
        dy *= scale
    yaw_step = clamp(
        max(0.0, gain) * normalize_angle(desired_yaw - current_yaw),
        -max_yaw_step,
        max_yaw_step,
    )
    return (
        current_x + dx,
        current_y + dy,
        normalize_angle(current_yaw + yaw_step),
    )


def proportional_command(
    error_x: float,
    error_y: float,
    yaw_error: float,
    *,
    translation_locked: bool,
    xy_gain: float,
    yaw_gain: float,
    max_linear_speed: float,
    max_angular_speed: float,
    min_linear_speed: float,
    min_angular_speed: float,
    yaw_tolerance: float,
):
    if translation_locked:
        linear_x = 0.0
        linear_y = 0.0
    else:
        distance = math.hypot(error_x, error_y)
        speed = min(max_linear_speed, xy_gain * distance)
        if distance > 0.0:
            speed = max(min_linear_speed, speed)
            linear_x = speed * error_x / distance
            linear_y = speed * error_y / distance
        else:
            linear_x = 0.0
            linear_y = 0.0

    if abs(yaw_error) <= yaw_tolerance:
        angular_z = 0.0
    else:
        angular_z = clamp(yaw_gain * yaw_error, -max_angular_speed, max_angular_speed)
        if abs(angular_z) < min_angular_speed:
            angular_z = math.copysign(min_angular_speed, angular_z)
    return linear_x, linear_y, angular_z


def rate_limit(previous: float, desired: float, maximum_delta: float) -> float:
    return previous + clamp(desired - previous, -maximum_delta, maximum_delta)


def axis_command(
    error: float,
    *,
    gain: float,
    maximum_speed: float,
    minimum_speed: float,
    tolerance: float,
) -> float:
    """Return a signed one-axis proportional command with a dead band."""
    if abs(error) <= tolerance:
        return 0.0
    speed = min(maximum_speed, gain * abs(error))
    speed = max(minimum_speed, speed)
    return math.copysign(speed, error)


def corridor_is_clear(
    data,
    width: int,
    height: int,
    resolution: float,
    origin_pose,
    start_pose,
    target_pose,
    *,
    half_length: float,
    half_width: float,
    lethal_cost: int = 253,
    front_extension: float = 0.0,
):
    """Check the full swept rectangular footprint along a straight SE(2) move.

    The center follows a straight line while yaw follows the shortest angular
    interpolation about base_link, not the shifted rectangle center.
    front_extension enlarges positive body X only. Outside-map cells are unsafe.
    """
    if width <= 0 or height <= 0 or resolution <= 0.0:
        return False
    if len(data) != width * height:
        return False
    if not all(math.isfinite(v) and v >= 0.0 for v in (half_length, half_width, front_extension)):
        return False
    front_length = half_length + front_extension
    footprint_radius = math.hypot(front_length, half_width)

    start_x, start_y, start_yaw = start_pose
    target_x, target_y, target_yaw = target_pose
    distance = math.hypot(target_x - start_x, target_y - start_y)
    yaw_delta = normalize_angle(target_yaw - start_yaw)
    linear_samples = math.ceil(distance / max(0.02, resolution * 0.5))
    # Bound corner travel too, including the longer front during rotation.
    angular_samples = max(
        math.ceil(abs(yaw_delta) / math.radians(5.0)),
        math.ceil(abs(yaw_delta) * footprint_radius / max(0.005, resolution * 0.5)),
    )
    sample_count = max(1, linear_samples, angular_samples)

    origin_x, origin_y, origin_yaw = origin_pose
    origin_cos = math.cos(origin_yaw)
    origin_sin = math.sin(origin_yaw)
    cell_radius = math.ceil(footprint_radius / resolution) + 1

    for sample_index in range(sample_count + 1):
        ratio = sample_index / sample_count
        pose_x = start_x + ratio * (target_x - start_x)
        pose_y = start_y + ratio * (target_y - start_y)
        pose_yaw = normalize_angle(start_yaw + ratio * yaw_delta)

        dx_origin = pose_x - origin_x
        dy_origin = pose_y - origin_y
        grid_x = (origin_cos * dx_origin + origin_sin * dy_origin) / resolution
        grid_y = (-origin_sin * dx_origin + origin_cos * dy_origin) / resolution
        center_x = math.floor(grid_x)
        center_y = math.floor(grid_y)
        min_x = center_x - cell_radius
        max_x = center_x + cell_radius
        min_y = center_y - cell_radius
        max_y = center_y + cell_radius
        if min_x < 0 or min_y < 0 or max_x >= width or max_y >= height:
            return False

        pose_cos = math.cos(pose_yaw)
        pose_sin = math.sin(pose_yaw)
        for cell_y in range(min_y, max_y + 1):
            local_grid_y = (cell_y + 0.5) * resolution
            for cell_x in range(min_x, max_x + 1):
                if int(data[cell_y * width + cell_x]) < lethal_cost:
                    continue
                local_grid_x = (cell_x + 0.5) * resolution
                world_x = origin_x + origin_cos * local_grid_x - origin_sin * local_grid_y
                world_y = origin_y + origin_sin * local_grid_x + origin_cos * local_grid_y
                dx = world_x - pose_x
                dy = world_y - pose_y
                footprint_x = pose_cos * dx + pose_sin * dy
                footprint_y = -pose_sin * dx + pose_cos * dy
                if -half_length <= footprint_x <= front_length and abs(footprint_y) <= half_width:
                    return False
    return True
