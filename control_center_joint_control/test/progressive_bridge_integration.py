"""Isolated ROS integration check for large-goal progressive following."""

import json
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rcl_interfaces.msg import ParameterType, ParameterValue, SetParametersResult
from rcl_interfaces.srv import GetParameters, SetParameters
from std_msgs.msg import String
from std_srvs.srv import SetBool

from control_center_joint_control.joint_bridge import JointBridge
from control_center_joint_control.joint_definitions import (
    FEEDBACK_KEYS,
    GRIPPER_FEEDBACK_KEYS,
    GROUPS,
    TARGET_FEEDBACK_KEYS,
    group_joint_names,
)


def feedback_message(left_inner):
    data = {}
    for group_name in ('leg_waist', 'left_arm', 'right_arm', 'neck'):
        names = group_joint_names(group_name)
        positions = [0.0] * len(names)
        if group_name == 'left_arm':
            positions[0] = float(left_inner)
        data[FEEDBACK_KEYS[group_name]] = {
            'position': positions,
            'speed': [0.0] * len(names),
            'torque': [0.0] * len(names),
            'temperature': [30.0] * len(names),
            'communication_lost': [False] * len(names),
        }
        data[TARGET_FEEDBACK_KEYS[group_name]] = list(positions)
    for key in GRIPPER_FEEDBACK_KEYS.values():
        data[key] = {
            'position': [10.0],
            'speed': [0.0],
            'torque': [0.0],
            'temperature': [30.0],
            'communication_lost': [False],
        }
    return String(data=json.dumps(data))


def spin_for(executor, seconds):
    deadline = time.monotonic() + seconds
    while time.monotonic() < deadline:
        executor.spin_once(timeout_sec=0.01)


def main():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'enforce_single_instance:=false',
        '-p', 'feedback_topic:=/progressive_test/feedback',
        '-p', 'whole_body_command_topic:=/progressive_test/body_command',
        '-p', 'gripper_command_topic:=/progressive_test/gripper_command',
        '-p', 'feedback_timeout_sec:=10.0',
        '-p', 'hardware_prepare_timeout_sec:=6.0',
        '-p', 'hardware_reset_min_wait_sec:=0.0',
        '-p', 'hardware_reset_stable_sec:=0.1',
        '-p', 'topic_suffix:=progressive_test',
    ])
    bridge = JointBridge()
    io = Node('progressive_bridge_test_io')
    feedback_pub = io.create_publisher(
        String, '/progressive_test/feedback', 10
    )
    apply_pub = io.create_publisher(
        String, '/control_center_joint_control/apply_command', 10
    )
    commands = []
    clear_messages = []
    enable_messages = []
    reset_messages = []
    mode_requests = []
    io.create_subscription(
        String,
        '/progressive_test/body_command',
        lambda message: commands.append(json.loads(message.data)),
        10,
    )
    io.create_subscription(
        String,
        '/topic_arm_joints_clear_error_progressive_test',
        lambda message: clear_messages.append(json.loads(message.data)),
        10,
    )
    io.create_subscription(
        String,
        '/topic_arm_joints_set_enable_state_progressive_test',
        lambda message: enable_messages.append(json.loads(message.data)),
        10,
    )
    io.create_subscription(
        String,
        '/control_reset_progressive_test',
        lambda message: reset_messages.append(message.data),
        10,
    )

    def set_mode(request, response):
        response.results = [
            SetParametersResult(successful=True, reason='')
            for _parameter in request.parameters
        ]
        return response

    def get_mode(request, response):
        mode_requests.extend(request.names)
        response.values = [
            ParameterValue(
                type=ParameterType.PARAMETER_STRING,
                string_value='position',
            )
            for _name in request.names
        ]
        return response

    for node_name in (
        'left_arm',
        'right_arm',
        'neck',
        'waist_leg',
    ):
        io.create_service(
            SetParameters,
            f'/node_mod_motor_{node_name}_control_progressive_test/set_parameters',
            set_mode,
        )
        io.create_service(
            GetParameters,
            f'/node_mod_motor_{node_name}_control_progressive_test/get_parameters',
            get_mode,
        )
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(io)
    try:
        spin_for(executor, 0.1)
        feedback_pub.publish(feedback_message(0.0))
        spin_for(executor, 0.1)
        assert bridge._is_feedback_fresh()

        response = bridge._set_hardware_enabled(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert response.success
        deadline = time.monotonic() + 5.0
        while not bridge._hardware_ready and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert bridge._hardware_ready
        spin_for(executor, 0.1)
        assert reset_messages == ['']
        assert not mode_requests
        assert not clear_messages
        assert not enable_messages

        bridge._lock('test mode switch')
        feedback_pub.publish(feedback_message(17.0))
        spin_for(executor, 0.1)
        no_reset_response = bridge._set_hardware_enabled_without_reset(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert no_reset_response.success
        assert bridge._hardware_enabled
        assert not bridge._hardware_ready
        assert bridge._hardware_prepare is not None
        assert bridge._hardware_unlock_mode == 'without_reset'
        deadline = time.monotonic() + 5.5
        while not bridge._hardware_ready and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert bridge._hardware_ready
        assert reset_messages == ['']
        assert len(mode_requests) == 4
        assert not clear_messages
        assert len(enable_messages) == 1
        assert set(enable_messages[0]) == {
            joint
            for group_name in ('neck', 'left_arm', 'right_arm', 'leg_waist')
            for joint in group_joint_names(group_name)
        }
        assert commands
        assert abs(commands[0]['left_arm_target_joints_position'][0] - 17.0) < 1e-6
        assert all(
            abs(value) < 1e-6
            for key, values in commands[0].items()
            for index, value in enumerate(values)
            if not (key == 'left_arm_target_joints_position' and index == 0)
        )
        print('PASS: no-reset unlock holds current pose, prepares hardware and sends no reset')

        targets = {
            name: (80.0 if index == 0 else 0.0)
            for index, name in enumerate(group_joint_names('left_arm'))
        }
        apply_pub.publish(String(data=json.dumps({
            'group': 'left_arm',
            'targets_deg': targets,
        })))
        spin_for(executor, 0.15)
        assert 'left_arm' in bridge._active_goals
        assert commands

        current = 17.0
        for _ in range(30):
            commanded = commands[-1][GROUPS['left_arm']['command_key']][0]
            assert commanded >= current - 1e-6
            assert commanded - current <= 5.001
            current = commanded
            feedback_pub.publish(feedback_message(current))
            spin_for(executor, 0.12)
            if 'left_arm' not in bridge._active_goals:
                break

        assert abs(current - 80.0) <= 1.0
        assert 'left_arm' not in bridge._active_goals
        assert bridge._hardware_enabled

        # A 20% preset speed must reduce the body lead from 5 degrees to
        # approximately 1 degree per feedback cycle.
        targets[group_joint_names('left_arm')[0]] = 70.0
        apply_pub.publish(String(data=json.dumps({
            'group': 'left_arm',
            'targets_deg': targets,
            'speed_scale': 0.2,
        })))
        spin_for(executor, 0.15)
        assert 'left_arm' in bridge._active_goals
        for _ in range(20):
            commanded = commands[-1][GROUPS['left_arm']['command_key']][0]
            assert commanded <= current + 1e-6
            assert current - commanded <= 1.001
            current = commanded
            feedback_pub.publish(feedback_message(current))
            spin_for(executor, 0.12)
            if 'left_arm' not in bridge._active_goals:
                break
        assert abs(current - 70.0) <= 1.0
        assert 'left_arm' not in bridge._active_goals
        command_count_at_rest = len(commands)
        for noisy_current in (70.4, 69.6, 70.2, 69.8):
            feedback_pub.publish(feedback_message(noisy_current))
            spin_for(executor, 0.12)
        assert len(commands) == command_count_at_rest

        # The pose library's new 200% ceiling must be accepted by the bridge
        # and increase the body lead from 5 degrees to 10 degrees per cycle.
        current = 70.0
        feedback_pub.publish(feedback_message(current))
        spin_for(executor, 0.12)
        targets[group_joint_names('left_arm')[0]] = 85.0
        apply_pub.publish(String(data=json.dumps({
            'group': 'left_arm',
            'targets_deg': targets,
            'speed_scale': 2.0,
        })))
        spin_for(executor, 0.15)
        assert 'left_arm' in bridge._active_goals
        for _ in range(10):
            commanded = commands[-1][GROUPS['left_arm']['command_key']][0]
            assert commanded >= current - 1e-6
            assert commanded - current <= 10.001
            current = commanded
            feedback_pub.publish(feedback_message(current))
            spin_for(executor, 0.12)
            if 'left_arm' not in bridge._active_goals:
                break
        assert abs(current - 85.0) <= 1.0
        assert 'left_arm' not in bridge._active_goals
        print(
            f'PASS: 20%, 100%, and 200% progressive goals stayed within their '
            f'lead limits; feedback noise caused no extra commands '
            f'({len(commands)} messages)'
        )
    finally:
        executor.remove_node(io)
        executor.remove_node(bridge)
        io.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
