#!/usr/bin/env python3
"""Save an edited map and verify nav2_map_server can load it."""

import importlib.util
import math
import os
from pathlib import Path
import signal
import subprocess
import sys
import tempfile
import time

from lifecycle_msgs.msg import Transition
from lifecycle_msgs.srv import ChangeState
from nav2_msgs.srv import LoadMap
import numpy as np
import rclpy
from rclpy.node import Node


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'navigation_ui_node.py'
)
spec = importlib.util.spec_from_file_location(
    'navigation_map_editor_load_test', SOURCE
)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def call(node, client, request, timeout=5.0):
    assert client.wait_for_service(timeout_sec=timeout)
    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future, timeout_sec=timeout)
    assert future.done()
    return future.result()


def main():
    with tempfile.TemporaryDirectory() as directory:
        grid = np.zeros((40, 60), dtype=np.int16)
        grid[5:35, 28:31] = 100
        metadata = {
            'resolution': 0.05,
            'origin_x': -1.5,
            'origin_y': -1.0,
            'origin_yaw': math.radians(8.0),
        }
        success, message, yaml_path = module.save_map_files(
            directory, 'map_server_load_test', grid, metadata
        )
        assert success, message

        environment = os.environ.copy()
        process = subprocess.Popen(
            [
                'ros2', 'run', 'nav2_map_server', 'map_server',
                '--ros-args',
                '-r', '__node:=map_editor_test_server',
                '-p', f'yaml_filename:={yaml_path}',
            ],
            env=environment,
            start_new_session=True,
        )
        rclpy.init()
        node = Node('map_editor_load_test_client')
        try:
            change_state = node.create_client(
                ChangeState, '/map_editor_test_server/change_state'
            )
            configure = ChangeState.Request()
            configure.transition.id = Transition.TRANSITION_CONFIGURE
            assert call(node, change_state, configure).success
            activate = ChangeState.Request()
            activate.transition.id = Transition.TRANSITION_ACTIVATE
            assert call(node, change_state, activate).success

            load_map = node.create_client(
                LoadMap, '/map_editor_test_server/load_map'
            )
            request = LoadMap.Request()
            request.map_url = str(Path(yaml_path).resolve())
            response = call(node, load_map, request)
            assert int(response.result) == 0, (
                f'LoadMap returned result={int(response.result)}'
            )
            assert int(response.map.info.width) == 60
            assert int(response.map.info.height) == 40
            assert abs(float(response.map.info.resolution) - 0.05) < 1e-6
        finally:
            node.destroy_node()
            rclpy.shutdown()
            try:
                os.killpg(process.pid, signal.SIGTERM)
            except ProcessLookupError:
                pass
            try:
                process.wait(timeout=2.0)
            except subprocess.TimeoutExpired:
                try:
                    os.killpg(process.pid, signal.SIGKILL)
                except ProcessLookupError:
                    pass
                process.wait(timeout=2.0)

    print('PASS: saved edited map is accepted by nav2_map_server LoadMap')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
