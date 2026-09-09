"""Isolated ROS integration test for keyframe execution and cancellation."""

import json
import time

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String
from std_srvs.srv import SetBool

from control_center_joint_control.action_library import action_joint_names
from control_center_joint_control.joint_bridge import JointBridge
from control_center_joint_control.joint_definitions import (
    FEEDBACK_KEYS,
    GRIPPER_FEEDBACK_KEYS,
    GROUPS,
    TARGET_FEEDBACK_KEYS,
    group_joint_names,
)


def feedback_message(values):
    data = {}
    for group_name in ('leg_waist', 'left_arm', 'right_arm', 'neck'):
        names = group_joint_names(group_name)
        positions = [float(values.get(name, 0.0)) for name in names]
        data[FEEDBACK_KEYS[group_name]] = {
            'position': positions,
            'speed': [0.0] * len(names),
            'torque': [0.0] * len(names),
            'temperature': [30.0] * len(names),
            'communication_lost': [False] * len(names),
        }
        data[TARGET_FEEDBACK_KEYS[group_name]] = list(positions)
    for name, key in GRIPPER_FEEDBACK_KEYS.items():
        data[key] = {
            'position': [float(values.get(name, 10.0))],
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


def frame(first_value, transition=0.25, hold=0.05):
    names = action_joint_names(['right_arm'])
    return {
        'transition_sec': transition,
        'hold_sec': hold,
        'easing': 'smooth',
        'joints_deg': {
            name: (float(first_value) if index == 0 else 0.0)
            for index, name in enumerate(names)
        },
    }


def main():
    rclpy.init(args=[
        '--ros-args',
        '-p', 'enforce_single_instance:=false',
        '-p', 'feedback_topic:=/action_test/feedback',
        '-p', 'whole_body_command_topic:=/action_test/body_command',
        '-p', 'gripper_command_topic:=/action_test/gripper_command',
        '-p', 'feedback_timeout_sec:=10.0',
        '-p', 'hardware_prepare_timeout_sec:=6.0',
        '-p', 'hardware_reset_min_wait_sec:=0.0',
        '-p', 'hardware_reset_stable_sec:=0.05',
        '-p', 'command_update_rate_hz:=20.0',
        '-p', 'topic_suffix:=action_test',
    ])
    bridge = JointBridge()
    io = Node('action_bridge_test_io')
    feedback_pub = io.create_publisher(String, '/action_test/feedback', 10)
    action_pub = io.create_publisher(
        String, '/control_center_joint_control/action_command', 10
    )
    reset_messages = []
    commands = []
    io.create_subscription(
        String,
        '/control_reset_action_test',
        lambda message: reset_messages.append(message.data),
        10,
    )
    io.create_subscription(
        String,
        '/action_test/body_command',
        lambda message: commands.append(json.loads(message.data)),
        10,
    )
    executor = SingleThreadedExecutor()
    executor.add_node(bridge)
    executor.add_node(io)
    values = {}
    try:
        feedback_pub.publish(feedback_message(values))
        spin_for(executor, 0.15)
        response = bridge._set_hardware_enabled(
            SetBool.Request(data=True), SetBool.Response()
        )
        assert response.success
        deadline = time.monotonic() + 3.0
        while not bridge._hardware_ready and time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.02)
        assert bridge._hardware_ready
        assert reset_messages == ['']

        no_motion_action = {
            'groups': ['right_arm'],
            'loop_count': 1,
            'keyframes': [frame(0.0)],
        }
        action_pub.publish(String(data=json.dumps({
            'command': 'start',
            'name': 'empty_action',
            'action': no_motion_action,
        })))
        spin_for(executor, 0.15)
        assert bridge._action_state is None
        assert not commands
        assert '基本相同' in bridge._last_event

        slow_feedback_action = {
            'groups': ['right_arm'],
            'loop_count': 1,
            'speed_percent': 100,
            'keyframes': [frame(6.0, transition=0.2, hold=0.0)],
        }
        action_pub.publish(String(data=json.dumps({
            'command': 'start',
            'name': 'wait_for_real_feedback',
            'action': slow_feedback_action,
        })))
        spin_for(executor, 0.7)
        assert bridge._action_state is not None
        assert '等待实际到达' in bridge._action_state['stage_label']
        deadline = time.monotonic() + 3.0
        while bridge._action_state is not None and time.monotonic() < deadline:
            body = commands[-1]
            for group_name in ('leg_waist', 'left_arm', 'right_arm', 'neck'):
                names = group_joint_names(group_name)
                command_values = body[GROUPS[group_name]['command_key']]
                for name, value in zip(names, command_values):
                    values[name] = float(value)
            feedback_pub.publish(feedback_message(values))
            spin_for(executor, 0.06)
        assert bridge._action_state is None
        assert '已完成' in bridge._last_event
        commands.clear()

        action = {
            'groups': ['right_arm'],
            'loop_count': 2,
            'speed_percent': 200,
            'keyframes': [frame(12.0), frame(-12.0)],
        }
        action_pub.publish(String(data=json.dumps({
            'command': 'start',
            'name': 'integration_wave',
            'action': action,
        })))
        observed = []
        deadline = time.monotonic() + 8.0
        started = False
        while time.monotonic() < deadline:
            executor.spin_once(timeout_sec=0.01)
            if bridge._action_state is not None:
                started = True
            if commands:
                body = commands[-1]
                for group_name in ('leg_waist', 'left_arm', 'right_arm', 'neck'):
                    names = group_joint_names(group_name)
                    command_values = body[GROUPS[group_name]['command_key']]
                    for name, value in zip(names, command_values):
                        values[name] = float(value)
                observed.append(values[action_joint_names(['right_arm'])[0]])
                feedback_pub.publish(feedback_message(values))
            if started and bridge._action_state is None:
                break
        assert started
        assert bridge._action_state is None
        assert '已完成' in bridge._last_event
        assert max(observed) >= 11.5
        assert min(observed) <= -11.5
        assert max(
            abs(after - before)
            for before, after in zip(observed, observed[1:])
        ) <= 10.001

        long_action = {
            'groups': ['right_arm'],
            'loop_count': 1,
            'keyframes': [frame(40.0, transition=2.0, hold=0.0)],
        }
        action_pub.publish(String(data=json.dumps({
            'command': 'start',
            'name': 'cancel_test',
            'action': long_action,
        })))
        spin_for(executor, 0.15)
        assert bridge._action_state is not None
        command_count_before_cancel = len(commands)
        action_pub.publish(String(data=json.dumps({'command': 'cancel'})))
        spin_for(executor, 0.15)
        assert bridge._action_state is None
        assert '操作员停止' in bridge._last_event
        assert len(commands) > command_count_before_cancel
        right_command = commands[-1][GROUPS['right_arm']['command_key']]
        assert right_command[0] == bridge._current[
            action_joint_names(['right_arm'])[0]
        ]
        print(
            f'PASS: two-loop keyframe action completed and cancel worked '
            f'with {len(commands)} safe body commands'
        )
    finally:
        executor.remove_node(io)
        executor.remove_node(bridge)
        io.destroy_node()
        bridge.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
