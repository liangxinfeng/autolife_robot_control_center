#!/usr/bin/env python3
import importlib.util
import math
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.parameter import Parameter
from sensor_msgs.msg import LaserScan


SCRIPT = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'proximity_voice_alert_node.py'
)


def _scan(point_angles=(), distance=0.60):
    message = LaserScan()
    message.angle_min = -math.pi
    message.angle_increment = math.radians(1.0)
    message.angle_max = math.pi
    message.range_min = 0.05
    message.range_max = 10.0
    message.ranges = [float('inf')] * 361
    for angle_degrees in point_angles:
        index = int(round(angle_degrees + 180.0))
        message.ranges[index] = float(distance)
    return message


def main():
    spec = importlib.util.spec_from_file_location(
        'proximity_voice_alert_under_test', SCRIPT
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    node = module.ProximityVoiceAlertNode(parameter_overrides=[
        Parameter('startup_grace_sec', value=0.0),
        Parameter('repeat_while_near_sec', value=999.0),
        Parameter('trigger_consecutive_scans', value=3),
        Parameter('clear_consecutive_scans', value=3),
    ])
    published = []

    class PublisherRecorder:
        def publish(self, message):
            published.append(message.data)

    node._voice_publisher = PublisherRecorder()
    near = _scan((-179, 179, 180))
    clear = _scan()
    side = _scan((58, 60, 62))

    # A close object while the navigation action is idle must stay silent.
    for _ in range(5):
        node._handle_scan(near)
    assert published == []

    active_status = GoalStatusArray()
    active_goal = GoalStatus()
    active_goal.status = GoalStatus.STATUS_EXECUTING
    active_status.status_list = [active_goal]
    node._handle_navigation_status(active_status)

    for _ in range(3):
        node._handle_scan(near)
    assert published == ['前方有人或障碍物，请注意避让']

    for _ in range(8):
        node._handle_scan(near)
    assert len(published) == 1, 'cooldown did not suppress repeated alerts'

    for _ in range(3):
        node._handle_scan(clear)
    for _ in range(5):
        node._handle_scan(side)
    assert len(published) == 1, 'points outside the front sector triggered an alert'

    for _ in range(3):
        node._handle_scan(near)
    assert len(published) == 2, 'clear-to-near transition did not re-arm alert'

    # A terminal/empty action status disables and resets the detector.
    node._handle_navigation_status(GoalStatusArray())
    for _ in range(5):
        node._handle_scan(near)
    assert len(published) == 2, 'an idle robot emitted a proximity alert'

    node.destroy_node()
    rclpy.shutdown()
    print('proximity_voice_alert_smoke_ok')


if __name__ == '__main__':
    main()
