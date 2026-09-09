#!/usr/bin/env python3
"""Render the complete standalone navigation window in an isolated domain."""

import importlib.util
import math
from pathlib import Path
import sys
import time

from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'navigation_ui_node.py'
)
spec = importlib.util.spec_from_file_location('navigation_ui_window', SOURCE)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def sample_map():
    message = OccupancyGrid()
    message.info.width = 150
    message.info.height = 110
    message.info.resolution = 0.05
    message.info.origin = Pose()
    message.info.origin.position.x = -3.75
    message.info.origin.position.y = -2.75
    message.info.origin.orientation.w = 1.0
    data = [0] * (150 * 110)
    for y in range(110):
        for x in range(150):
            wall = (
                x in (3, 146) or y in (3, 106)
                or (x in range(45, 49) and y in range(20, 88))
                or (x in range(93, 97) and y in range(35, 104))
            )
            if wall:
                data[y * 150 + x] = 100
    message.data = data
    return message


def main():
    rclpy.init()
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    node = module.NavigationRosNode()
    window = module.NavigationWindow(node)
    received_maps = []
    node.signals.map_received.connect(received_maps.append)
    node.set_map_updates_enabled(False)
    node._on_map(sample_map())
    assert not received_maps
    node.set_map_updates_enabled(True)
    node._on_map(sample_map())
    assert len(received_maps) == 1
    print('PASS: inactive navigation canvas rejects shared SLAM map frames')
    tf_poses = []
    node.signals.pose_received.connect(tf_poses.append)
    node._last_cov_x = 0.03
    node._last_cov_y = 0.02
    mock_transform = type('TransformStamped', (), {
        'transform': type('Transform', (), {
            'translation': type(
                'Translation', (), {'x': 1.25, 'y': -0.40, 'z': 0.0}
            )(),
            'rotation': type('Rotation', (), {
                'x': 0.0,
                'y': 0.0,
                'z': math.sin(math.radians(35) * 0.5),
                'w': math.cos(math.radians(35) * 0.5),
            })(),
        })(),
    })()

    class MockTfBuffer:
        def lookup_transform(self, *_args):
            return mock_transform

    node.tf_buffer = MockTfBuffer()
    node._publish_display_pose_from_tf()
    assert tf_poses
    assert abs(tf_poses[-1]['x'] - 1.25) < 1e-9
    assert abs(tf_poses[-1]['y'] + 0.40) < 1e-9
    assert abs(tf_poses[-1]['yaw'] - math.radians(35)) < 1e-9
    print('PASS: robot display pose is refreshed from map-to-base TF')

    window.resize(1500, 900)
    window._map_received(sample_map())
    window._pose_received({
        'x': 0.5, 'y': 0.3, 'yaw': math.radians(32),
        'cov_x': 0.025, 'cov_y': 0.018,
    })
    window._markers_received([
        {'name': '充电桩', 'x': -2.3, 'y': -1.5, 'yaw': 0.0},
        {'name': '吧台', 'x': 2.2, 'y': 1.6, 'yaw': math.radians(90)},
        {'name': '客房', 'x': 2.6, 'y': -1.5, 'yaw': math.radians(-135)},
        {'name': '洗衣房', 'x': -1.9, 'y': 1.7, 'yaw': math.radians(35)},
    ])
    window._set_path(type('PathMessage', (), {
        'poses': [
            type('Stamped', (), {'pose': type('PoseValue', (), {
                'position': type('Position', (), {'x': x, 'y': y})()
            })()})()
            for x, y in ((-2.3, -1.5), (-0.6, -0.3), (0.5, 0.3), (2.2, 1.6))
        ]
    })(), False)
    window._laser_received([
        (
            0.5 + math.cos(index * 0.1) * 1.15,
            0.3 + math.sin(index * 0.1) * 1.15,
        )
        for index in range(63)
    ])
    window._relocalization_status({
        'state': 'matching', 'detail': '正在执行全地图相关性搜索',
        'attempt': 2, 'score': 0.641,
    })
    window._navigation_status({
        'state': 'running', 'message': '正在前往吧台，剩余 3.42 m',
        'active_waypoint': '吧台',
    })
    window.waypoint_list.addItems(['充电桩', '吧台', '客房', '洗衣房'])
    window.show()
    application.processEvents()

    press_w = QtGui.QKeyEvent(
        QtCore.QEvent.KeyPress, QtCore.Qt.Key_W, QtCore.Qt.NoModifier
    )
    press_a = QtGui.QKeyEvent(
        QtCore.QEvent.KeyPress, QtCore.Qt.Key_A, QtCore.Qt.NoModifier
    )
    release_a = QtGui.QKeyEvent(
        QtCore.QEvent.KeyRelease, QtCore.Qt.Key_A, QtCore.Qt.NoModifier
    )
    press_s = QtGui.QKeyEvent(
        QtCore.QEvent.KeyPress, QtCore.Qt.Key_S, QtCore.Qt.NoModifier
    )
    release_s = QtGui.QKeyEvent(
        QtCore.QEvent.KeyRelease, QtCore.Qt.Key_S, QtCore.Qt.NoModifier
    )
    release_w = QtGui.QKeyEvent(
        QtCore.QEvent.KeyRelease, QtCore.Qt.Key_W, QtCore.Qt.NoModifier
    )
    window.keyPressEvent(press_w)
    window.keyPressEvent(press_a)
    assert node._velocity == (0.2, 0.0, 0.8)
    window.keyReleaseEvent(release_a)
    assert node._velocity == (0.2, 0.0, 0.0)
    window.keyPressEvent(press_s)
    assert node._velocity == (0.0, 0.0, 0.0)
    window.keyReleaseEvent(release_s)
    assert node._velocity == (0.2, 0.0, 0.0)
    window.keyReleaseEvent(release_w)
    assert node._velocity == (0.0, 0.0, 0.0)
    print('PASS: simultaneous keyboard motion and safe opposite-key handling')

    published = []
    original_publish = node._publish_velocity_values

    def record_publish(values):
        published.append((time.monotonic(), tuple(values)))
        original_publish(values)

    node._publish_velocity_values = record_publish
    window.keyPressEvent(QtGui.QKeyEvent(
        QtCore.QEvent.KeyPress, QtCore.Qt.Key_W, QtCore.Qt.NoModifier
    ))
    time.sleep(0.16)
    window.keyReleaseEvent(QtGui.QKeyEvent(
        QtCore.QEvent.KeyRelease, QtCore.Qt.Key_W, QtCore.Qt.NoModifier
    ))
    node._publish_velocity_values = original_publish
    moving_heartbeats = [
        sample for sample in published if abs(sample[1][0]) > 1e-6
    ]
    assert len(moving_heartbeats) >= 5
    print('PASS: manual velocity heartbeat continues while Qt is blocked')

    output = Path('/tmp/navigation_native_ui_window_smoke.png')
    assert window.grab().save(str(output))
    print(f'PASS: full navigation window rendering: {output}')
    window.close()
    application.processEvents()
    node.destroy_node()
    rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
