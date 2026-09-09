import math

from waypoint_manager_node import WaypointManagerNode


class Duration:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds


class Stamp:
    def __init__(self, seconds):
        self.nanoseconds = int(seconds * 1e9)

    def __sub__(self, other):
        return Duration(self.nanoseconds - other.nanoseconds)


class Clock:
    def __init__(self):
        self.seconds = 0.0

    def now(self):
        return Stamp(self.seconds)


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass


def make_node():
    node = WaypointManagerNode.__new__(WaypointManagerNode)
    node._clock = Clock()
    node.get_clock = lambda: node._clock
    node.get_logger = lambda: Logger()
    node._map_pose = [0.0, 0.0, 0.0]
    node._odom_pose = [0.0, 0.0, 0.0]
    node._map_frame = 'map'
    node._odom_frame = 'odom'
    node._robot_pose_in_frame = lambda frame: tuple(
        node._odom_pose if frame == node._odom_frame else node._map_pose
    )
    node._pose_in_frame = lambda _target, _frame: (1.0, 0.5, 0.5)
    node._short_stage_path_is_clear = lambda *_args: (True, 'clear')
    node._publish_navigation_status = lambda *_args, **_kwargs: None
    node._fallback_short_distance_to_nav2 = lambda reason: (_ for _ in ()).throw(
        AssertionError(reason)
    )
    node.completed = []
    node._complete_short_distance = lambda distance, yaw: node.completed.append(
        (distance, yaw)
    )
    node._short_cmd_vel_pub = Publisher()
    node._short_servo_active = True
    node._short_final_refine = False
    node._active_goal_name = 'test'
    node._short_target = {'frame_id': 'map', 'x': 1.0, 'y': 0.5, 'yaw': 0.5}
    node._short_target_map_pose = (1.0, 0.5, 0.5)
    node._short_odom_target = (1.0, 0.5, 0.5)
    node._active_goal_start_time = Stamp(0.0)
    node._short_max_duration = 35.0
    node._short_map_correction_gain = 0.08
    node._short_max_map_xy_correction_step = 0.002
    node._short_max_map_yaw_correction_step = 0.004
    node._short_final_map_settle_time = 0.35
    node._short_max_final_correction_passes = 3
    node._short_axis_tolerance = 0.025
    node._short_xy_tolerance = 0.05
    node._short_yaw_tolerance = 0.06
    node._short_yaw_exit_tolerance = 0.09
    node._short_settle_time = 0.4
    node._short_stage_settle_time = 0.18
    node._short_progress_epsilon = 0.015
    node._short_progress_timeout = 3.0
    node._short_path_check_period = 0.1
    node._short_xy_gain = 0.9
    node._short_yaw_gain = 1.2
    node._short_max_linear_speed = 0.16
    node._short_max_angular_speed = 0.45
    node._short_min_linear_speed = 0.025
    node._short_min_angular_speed = 0.06
    node._short_max_linear_acceleration = 0.5
    node._short_max_angular_acceleration = 1.0
    node._short_servo_period = 0.05
    node._short_stage = 'lateral'
    node._short_stage_ready_since = None
    node._short_translation_locked = False
    node._short_stable_since = None
    node._short_best_distance = math.hypot(1.0, 0.5)
    node._short_last_progress_time = Stamp(0.0)
    node._short_last_path_check_time = Stamp(0.0)
    node._short_last_status_time = None
    node._short_last_command = (0.0, 0.0, 0.0)
    node._short_correction_pass = 0
    node._short_final_verify_since = None
    return node


def last_command(node):
    message = node._short_cmd_vel_pub.messages[-1]
    return message.linear.x, message.linear.y, message.angular.z


def test_stages_never_mix_axes_and_yaw_drift_reenters_translation():
    node = make_node()

    node._clock.seconds = 0.1
    node._run_short_distance_servo()
    x, y, yaw = last_command(node)
    assert x == 0.0 and y > 0.0 and yaw == 0.0

    node._map_pose[:] = [0.0, 0.5, 0.0]
    node._odom_pose[:] = [0.0, 0.5, 0.0]
    node._clock.seconds = 0.2
    node._run_short_distance_servo()
    assert node._short_stage == 'longitudinal'
    assert last_command(node) == (0.0, 0.0, 0.0)

    node._clock.seconds = 0.4
    node._run_short_distance_servo()
    x, y, yaw = last_command(node)
    assert x > 0.0 and y == 0.0 and yaw == 0.0

    node._map_pose[:] = [1.0, 0.5, 0.0]
    node._odom_pose[:] = [1.0, 0.5, 0.0]
    node._clock.seconds = 0.5
    node._run_short_distance_servo()
    assert node._short_stage == 'yaw'
    assert last_command(node) == (0.0, 0.0, 0.0)

    node._clock.seconds = 0.7
    node._run_short_distance_servo()
    x, y, yaw = last_command(node)
    assert x == 0.0 and y == 0.0 and yaw > 0.0

    # Simulate the real chassis drifting 15 cm while final yaw is corrected.
    node._map_pose[:] = [1.15, 0.5, 0.5]
    node._odom_pose[:] = [1.15, 0.5, 0.5]
    node._clock.seconds = 0.8
    node._run_short_distance_servo()
    node._clock.seconds = 1.3
    node._run_short_distance_servo()
    node._clock.seconds = 1.7
    node._run_short_distance_servo()
    assert not node.completed
    assert node._short_stage == 'lateral'
    assert node._short_correction_pass == 1
