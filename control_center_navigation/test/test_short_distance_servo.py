import math

from short_distance_servo import (
    apply_body_offset,
    axis_command,
    body_frame_error,
    bounded_pose_update,
    corridor_is_clear,
    proportional_command,
    rate_limit,
    reproject_target_pose,
)


def make_grid(width=100, height=100):
    return [0] * (width * height)


def set_cost(grid, x, y, value=254, width=100):
    grid[y * width + x] = value


def test_body_error_stays_world_direct_during_rotation():
    error_x, error_y, yaw_error = body_frame_error(
        (0.0, 0.0, math.pi / 2.0),
        (1.0, 0.0, 0.0),
    )
    assert abs(error_x) < 1e-9
    assert abs(error_y + 1.0) < 1e-9
    assert abs(yaw_error + math.pi / 2.0) < 1e-9


def test_map_target_is_reprojected_into_odom_without_changing_body_error():
    current_map = (4.0, -2.0, math.radians(30.0))
    current_odom = (1.0, 3.0, math.radians(-20.0))
    target_map = apply_body_offset(current_map, (0.8, -0.35, 0.4))
    target_odom = reproject_target_pose(current_odom, current_map, target_map)
    odom_error = body_frame_error(current_odom, target_odom)
    assert abs(odom_error[0] - 0.8) < 1e-9
    assert abs(odom_error[1] + 0.35) < 1e-9
    assert abs(odom_error[2] - 0.4) < 1e-9


def test_map_correction_is_bounded_before_entering_odom_loop():
    updated = bounded_pose_update(
        (0.0, 0.0, 0.0),
        (0.30, 0.40, 0.5),
        gain=1.0,
        max_xy_step=0.02,
        max_yaw_step=0.03,
    )
    assert abs(math.hypot(updated[0], updated[1]) - 0.02) < 1e-9
    assert abs(updated[2] - 0.03) < 1e-9


def test_translation_lock_only_allows_yaw():
    command = proportional_command(
        0.2,
        -0.1,
        0.4,
        translation_locked=True,
        xy_gain=1.0,
        yaw_gain=1.0,
        max_linear_speed=0.2,
        max_angular_speed=0.5,
        min_linear_speed=0.02,
        min_angular_speed=0.05,
        yaw_tolerance=0.06,
    )
    assert command == (0.0, 0.0, 0.4)


def test_rate_limit():
    assert rate_limit(0.0, 1.0, 0.1) == 0.1
    assert rate_limit(0.0, -1.0, 0.1) == -0.1


def test_axis_command_has_dead_band_and_preserves_direction():
    parameters = {
        'gain': 1.0,
        'maximum_speed': 0.16,
        'minimum_speed': 0.025,
        'tolerance': 0.025,
    }
    assert axis_command(0.02, **parameters) == 0.0
    assert axis_command(0.04, **parameters) == 0.04
    assert axis_command(-0.40, **parameters) == -0.16


def test_free_corridor_is_clear():
    grid = make_grid()
    assert corridor_is_clear(
        grid,
        100,
        100,
        0.05,
        (-2.5, -2.5, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        half_length=0.30,
        half_width=0.225,
    )


def test_obstacle_inside_swept_footprint_blocks():
    grid = make_grid()
    set_cost(grid, 60, 50)
    assert not corridor_is_clear(
        grid,
        100,
        100,
        0.05,
        (-2.5, -2.5, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        half_length=0.30,
        half_width=0.225,
    )


def test_obstacle_outside_swept_footprint_does_not_block():
    grid = make_grid()
    set_cost(grid, 60, 60)
    assert corridor_is_clear(
        grid,
        100,
        100,
        0.05,
        (-2.5, -2.5, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        half_length=0.30,
        half_width=0.225,
    )


def test_short_mode_ignores_inflation_but_still_blocks_lethal_cells():
    inflated_grid = make_grid()
    set_cost(inflated_grid, 60, 50, value=253)
    assert corridor_is_clear(
        inflated_grid,
        100,
        100,
        0.05,
        (-2.5, -2.5, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        half_length=0.25,
        half_width=0.175,
        lethal_cost=254,
    )

    lethal_grid = make_grid()
    set_cost(lethal_grid, 60, 50, value=254)
    assert not corridor_is_clear(
        lethal_grid,
        100,
        100,
        0.05,
        (-2.5, -2.5, 0.0),
        (0.0, 0.0, 0.0),
        (1.0, 0.0, 0.0),
        half_length=0.25,
        half_width=0.175,
        lethal_cost=254,
    )
