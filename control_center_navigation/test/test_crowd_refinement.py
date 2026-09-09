"""Offline regression tests. No ROS publishers, services, or hardware goals."""
from types import SimpleNamespace
import math
import pytest
from waypoint_manager_node import WaypointManagerNode
from test_staged_short_servo_logic import make_node, Stamp, last_command


def final_node():
    node = make_node()
    node._short_final_refine = True
    node._short_blocked_since = None
    node._short_clear_since = None
    node._refine_clear_settle = 0.6
    node._refine_obstacle_wait = 20.0
    node._active_goal_id = 1
    node._publish_waypoint_markers = lambda: None
    node.stopped = []
    node._stop_sequence = lambda: node.stopped.append(True)
    node.statuses = []
    node._publish_navigation_status = lambda state, *args, **kwargs: node.statuses.append(state)
    node._fallback_short_distance_to_nav2 = WaypointManagerNode._fallback_short_distance_to_nav2.__get__(node)
    node._start_navigation = lambda *args, **kwargs: pytest.fail('must not restart Nav2')
    return node


def test_blocked_corridor_stops_and_resumes_same_goal_only_after_stable_clear():
    n = final_node()
    n._short_stage_path_is_clear = lambda *args: (False, 'person')
    for t in (0.1, 1.0, 4.0):
        n._clock.seconds = t
        n._run_short_distance_servo()
        assert last_command(n) == (0, 0, 0)
        assert not n.completed and not n.stopped
    assert n._active_goal_name == 'test'
    n._short_stage_path_is_clear = lambda *args: (True, 'clear')
    for t in (4.2, 4.5):
        n._clock.seconds = t
        n._run_short_distance_servo()
        assert last_command(n) == (0, 0, 0)
    n._clock.seconds = 4.9
    n._run_short_distance_servo()
    assert last_command(n)[1] > 0
    assert n._active_goal_name == 'test'


def test_flickering_obstacle_cannot_reset_wait_budget_or_resume_on_one_frame():
    n = final_node()
    for t, clear in ((0.1, False), (8, True), (8.2, False), (19.9, True), (20.2, False)):
        n._clock.seconds = t
        assert n._wait_for_refine_corridor(clear, 'person', n._clock.now())
        assert last_command(n) == (0, 0, 0)
    assert not n._short_servo_active
    assert n._failed_goal_name == 'test'
    assert n.statuses[-1] == 'failed'
    assert not n.completed


@pytest.mark.parametrize('reason', ['no progress', 'timeout', 'AMCL failed to converge', 'TF stale'])
def test_final_failures_never_loop_back_to_nav2_or_advance(reason):
    n = final_node()
    n._fallback_short_distance_to_nav2(reason)
    assert last_command(n) == (0, 0, 0)
    assert not n.completed and n.stopped
    assert n._failed_goal_name == 'test'


def test_cancel_rejected_retains_nav2_handle_and_never_starts_servo():
    n = final_node()
    n._arrival_refine_pending = True
    handle = object()
    n._active_goal_handle = handle
    n._check_refine_cancel(SimpleNamespace(result=lambda: SimpleNamespace(return_code=1)), 1)
    assert not n._arrival_refine_pending
    assert n._active_goal_handle is handle


def test_stale_action_result_cannot_resume_old_waypoint():
    n = final_node()
    n._arrival_refine_pending = True
    n._finish_arrival_refine_handover = lambda *args: pytest.fail('stale handoff')
    n._handle_navigation_result(SimpleNamespace(result=lambda: SimpleNamespace(status=5)), 'test', 0)
    assert n._active_goal_name == 'test'


def test_nav2_success_cannot_bypass_failed_refinement_or_missing_tf():
    n = final_node()
    n._arrival_refine_pending = False
    n._far_navigation_auto_refine_enabled = True
    n._try_start_short_distance = lambda *args, **kwargs: (False, None)
    n._handle_navigation_result(SimpleNamespace(result=lambda: SimpleNamespace(status=4)), 'test', 1)
    assert not n.completed and n.statuses[-1] == 'failed'
    assert n._failed_goal_name == 'test'


def test_close_but_not_precise_position_never_succeeds():
    n = final_node()
    n._map_pose[:] = [0.85, 0.5, 0.5]
    n._odom_pose[:] = [0.85, 0.5, 0.5]
    n._short_stage = 'yaw'
    for t in (0.1, 0.6, 1.0):
        n._clock.seconds = t
        n._run_short_distance_servo()
    assert not n.completed


def test_manual_pause_clears_wait_and_cannot_auto_resume():
    n = final_node()
    n._navigation_paused = False
    n._active_goal_handle = n._active_undock_goal_handle = n._active_dock_goal_handle = None
    n._wait_for_refine_corridor(False, 'person', Stamp(0))
    response = n._handle_pause_navigation(None, SimpleNamespace())
    assert response.success and n._navigation_paused
    assert n._paused_goal_name == 'test'
    count = len(n._short_cmd_vel_pub.messages)
    n._clock.seconds = 5
    n._run_short_distance_servo()
    assert len(n._short_cmd_vel_pub.messages) == count


@pytest.mark.parametrize('xy,yaw,expected', [(0.2, 0.1, True), (0.3, 0.1, False), (0.2, 0.6, False)])
def test_handoff_radius_is_not_arrival_tolerance(xy, yaw, expected):
    n = final_node()
    n._short_servo_active = False
    n._navigation_paused = False
    n._precise_docking_active = False
    n._arrival_refine_pending = False
    n._active_goal_handle = object()
    n._waypoints = {'test': {'x': xy, 'y': 0.0, 'yaw': yaw}}
    n._arrival_xy_tolerance = 0.06
    n._arrival_yaw_tolerance = 0.08
    n._arrival_settle_time = 0.6
    n._far_navigation_auto_refine_enabled = n._short_servo_enabled = True
    n._short_distance_threshold = 2.0
    n._refine_handoff_distance = 0.25
    n._refine_handoff_yaw = 0.35
    n._normalize_angle = lambda value: math.atan2(math.sin(value), math.cos(value))
    n._goal_reached_since = n._last_arrival_log_time = None
    calls = []
    n._handover_arrival_to_refine = lambda *args: calls.append(args)
    for t in (0, 0.7):
        n._clock.seconds = t
        n._check_active_goal_arrival()
    assert bool(calls) == expected
    assert not n.completed
