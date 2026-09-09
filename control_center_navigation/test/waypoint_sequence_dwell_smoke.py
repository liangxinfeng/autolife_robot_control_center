#!/usr/bin/env python3
"""Verify that sequence dwell is asynchronous and advances once."""

import importlib.util
from pathlib import Path
import sys
import time

import rclpy


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'waypoint_manager_node.py'
)
spec = importlib.util.spec_from_file_location(
    'waypoint_manager_dwell_test', SOURCE
)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def main():
    rclpy.init()
    node = module.WaypointManagerNode()
    node._active_sequence = ['a', 'b']
    node._active_sequence_dwell_times = [0.08, 0.0]
    node._active_sequence_index = 0
    advanced = []
    node._advance_sequence_now = lambda: advanced.append(time.monotonic())

    started = time.monotonic()
    node._advance_sequence_after_arrival('a')
    call_elapsed = time.monotonic() - started
    assert call_elapsed < 0.04
    assert node._sequence_dwell_timer is not None
    assert not advanced

    deadline = time.monotonic() + 0.5
    while not advanced and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.02)

    assert len(advanced) == 1
    assert advanced[0] - started >= 0.06
    node._stop_sequence()
    node.destroy_node()
    rclpy.shutdown()
    print('PASS: per-waypoint dwell is non-blocking and advances once')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
