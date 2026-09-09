"""Offline fake-clock/pose regressions: never publish to ROS or hardware."""
import pytest
from test_crowd_refinement import final_node
from test_staged_short_servo_logic import Stamp, last_command


def near_node(yaw=0.0):
    n = final_node()
    n._short_target_map_pose = (0.03, 0.0, yaw)
    n._short_odom_target = n._short_target_map_pose
    n._short_xy_tolerance = 0.04
    n._short_yaw_tolerance = 0.05
    n._short_axis_tolerance = 0.02
    n._short_settle_time = n._short_final_map_settle_time = 0.5
    return n


def test_radial_arrival_stops_without_chasing_tighter_axis_threshold():
    n = near_node()
    for t in (0.1, 0.3, 0.59):
        n._clock.seconds = t
        n._run_short_distance_servo()
        assert last_command(n) == (0, 0, 0)
        assert not n.completed
    n._clock.seconds = 0.61
    n._run_short_distance_servo()
    assert len(n.completed) == 1
    assert n.completed[0][0] <= 0.04


@pytest.mark.parametrize('index', range(4))
def test_each_odom_map_position_yaw_error_resets_continuous_window(index):
    n = near_node()
    good = [0.03, 0.02, 0.03, 0.02]
    assert n._settle_final_refine(Stamp(0), *good)
    bad = good.copy()
    bad[index] = 0.1
    assert not n._settle_final_refine(Stamp(0.4), *bad)
    assert n._settle_final_refine(Stamp(0.6), *good)
    assert not n.completed
    assert n._settle_final_refine(Stamp(1.11), *good)
    assert len(n.completed) == 1


def test_position_in_tolerance_goes_to_yaw_not_extra_translation():
    n = near_node(yaw=0.2)
    n._clock.seconds = 0.1
    n._run_short_distance_servo()
    assert n._short_stage == 'yaw'
    assert last_command(n) == (0, 0, 0)
    n._clock.seconds = 0.35
    n._run_short_distance_servo()
    assert last_command(n)[:2] == (0, 0)
    assert last_command(n)[2] > 0
    assert not n.completed


def test_obstacle_discards_arrival_window_even_at_exact_goal():
    n = near_node()
    n._clock.seconds = 0.1
    n._run_short_distance_servo()
    n._short_stage_path_is_clear = lambda *args: (False, 'person')
    n._clock.seconds = 0.7
    n._run_short_distance_servo()
    assert not n.completed
    assert n._short_arrival_since is None
    assert last_command(n) == (0, 0, 0)


def test_deactivation_discards_previous_goal_stability():
    n = near_node()
    n._short_arrival_since = Stamp(0)
    n._deactivate_short_distance()
    assert n._short_arrival_since is None
