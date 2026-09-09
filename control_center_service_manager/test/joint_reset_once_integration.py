#!/usr/bin/env python3
"""Verify that the one-shot helper discovers and publishes exactly once."""

import importlib.util
from pathlib import Path
import time

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from std_msgs.msg import String


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'control_center_service_manager'
    / 'joint_reset_once.py'
)


def main():
    spec = importlib.util.spec_from_file_location(
        'joint_reset_once_under_test', SOURCE
    )
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    rclpy.init()
    receiver = Node('joint_reset_test_receiver')
    received = []
    receiver.create_subscription(
        String,
        '/control_reset_reset_test',
        lambda message: received.append(message.data),
        10,
    )
    sender = module.JointResetOnce(parameter_overrides=[
        Parameter('topic_suffix', value='reset_test'),
        Parameter('discovery_timeout_sec', value=2.0),
    ])
    assert sender.publish_once()
    deadline = time.monotonic() + 2.0
    while not received and time.monotonic() < deadline:
        rclpy.spin_once(receiver, timeout_sec=0.05)
    assert received == [''], received
    sender.destroy_node()
    receiver.destroy_node()
    rclpy.shutdown()
    print('joint_reset_once_integration_ok')


if __name__ == '__main__':
    main()
