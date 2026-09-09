import math
from types import SimpleNamespace
from test_staged_short_servo_logic import Clock, Logger

from waypoint_manager_node import (
    TRANSIENT_MANUAL_GOAL_NAME,
    WaypointManagerNode,
)


class ImmediateFuture:
    def result(self):
        return SimpleNamespace(return_code=0)

    def add_done_callback(self, callback):
        callback(self)


class GoalHandle:
    def __init__(self):
        self.cancel_count = 0

    def cancel_goal_async(self):
        self.cancel_count += 1
        return ImmediateFuture()


class Response:
    success = False
    message = ''


class Request:
    x = 1.2
    y = -0.4
    yaw = 3.5
    frame_id = 'map'


def test_manual_goal_is_transient_and_forces_regular_nav2_leg():
    node = WaypointManagerNode.__new__(WaypointManagerNode)
    node._map_frame = 'map'
    node._waypoints = {
        '吧台': {'x': 0.0, 'y': 0.0, 'yaw': 0.0},
    }
    node._completed_waypoints = set()
    node._failed_goal_name = None
    node._stop_sequence = lambda: None
    calls = []
    node._start_navigation = lambda name, **kwargs: (
        calls.append((name, kwargs)) or (True, 'started')
    )

    response = node._handle_navigate_to_pose_refined(Request(), Response())

    assert response.success
    assert calls == [(TRANSIENT_MANUAL_GOAL_NAME, {'force_normal': True})]
    assert node._public_waypoint_names() == ['吧台']
    assert TRANSIENT_MANUAL_GOAL_NAME not in node._persistent_waypoints()
    target = node._waypoints[TRANSIENT_MANUAL_GOAL_NAME]
    assert math.isclose(target['yaw'], math.atan2(math.sin(3.5), math.cos(3.5)))


def test_arrival_tolerance_cancels_nav2_before_starting_final_refinement():
    node = WaypointManagerNode.__new__(WaypointManagerNode)
    handle = GoalHandle()
    node._far_navigation_auto_refine_enabled = True
    node._short_servo_enabled = True
    node._active_goal_handle = handle
    node._active_goal_name = TRANSIENT_MANUAL_GOAL_NAME
    node._active_goal_id = 7
    node._goal_reached_since = object()
    node._arrival_refine_pending = False
    node.get_clock = lambda: Clock()
    node.get_logger = lambda: Logger()
    statuses = []
    node._publish_navigation_status = lambda state, *_args, **_kwargs: statuses.append(state)
    node._try_start_short_distance = lambda name, announce_departure=False, final_refine=False: (
        name == TRANSIENT_MANUAL_GOAL_NAME and not announce_departure,
        0.04,
    )

    node._handover_arrival_to_refine(
        TRANSIENT_MANUAL_GOAL_NAME, 0.04, 0.02
    )

    assert handle.cancel_count == 1
    assert node._active_goal_id == 7
    assert node._arrival_refine_pending
    assert node._active_goal_handle is handle
    assert statuses == ['far_navigation_refine_pending']
    # An acknowledged cancel is not terminal: only the action result transfers control.
    node._handle_navigation_result(SimpleNamespace(result=lambda: SimpleNamespace(status=5)), TRANSIENT_MANUAL_GOAL_NAME, 7)
    assert not node._arrival_refine_pending
    assert statuses[-1] == 'far_navigation_refining'


def test_refinement_unavailable_must_not_claim_arrival():
    node = WaypointManagerNode.__new__(WaypointManagerNode)
    node._arrival_refine_pending = True
    node._active_goal_name = TRANSIENT_MANUAL_GOAL_NAME
    node._try_start_short_distance = lambda *_args, **_kwargs: (False, 0.05)
    statuses = []
    node._publish_navigation_status = lambda state, *args, **kwargs: statuses.append(state)
    node._stop_sequence = lambda: None
    completed = []
    node._complete_goal_by_tolerance = (
        lambda name, xy, yaw: completed.append((name, xy, yaw))
    )

    node._finish_arrival_refine_handover(TRANSIENT_MANUAL_GOAL_NAME)

    assert not completed
    assert statuses == ['failed']
