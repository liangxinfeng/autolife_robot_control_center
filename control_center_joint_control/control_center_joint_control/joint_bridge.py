#!/usr/bin/env python3
"""Convert vendor JSON feedback to ROS JointState and guard position commands."""

import json
import math
import os
import time
import fcntl

import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from rcl_interfaces.srv import GetParameters, SetParameters
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from std_srvs.srv import SetBool

from .action_library import (
    action_joint_names,
    interpolate_joint_values,
    normalize_action_record,
)
from .body_height import BODY_HEIGHT_JOINTS, matches_body_height_profile
from .operator_targets import resolve_operator_targets
from .joint_definitions import (
    ALL_JOINTS,
    BASE_MODEL_JOINTS,
    BODY_JOINTS,
    FEEDBACK_KEYS,
    GRIPPER_FEEDBACK_KEYS,
    GRIPPER_JOINTS,
    GROUPS,
    GROUP_ORDER,
    JOINT_LIMITS,
    JOINT_TO_GROUP,
    TARGET_FEEDBACK_KEYS,
    clamp_joint,
    group_joint_names,
)


class JointBridge(Node):
    def __init__(self):
        super().__init__('whole_body_joint_bridge')
        self.declare_parameter('robot_id', 345)
        self.declare_parameter('topic_suffix', '')
        self.declare_parameter('feedback_timeout_sec', 0.6)
        self.declare_parameter('hardware_unlock_timeout_sec', 180.0)
        self.declare_parameter('require_operator_heartbeat', False)
        self.declare_parameter('operator_heartbeat_topic',
                               '/control_center/operator_heartbeat')
        self.declare_parameter('operator_heartbeat_timeout_sec', 0.50)
        self.declare_parameter('max_single_step_deg', 5.0)
        self.declare_parameter('max_gripper_command_lead_deg', 30.0)
        self.declare_parameter('goal_tolerance_deg', 1.0)
        self.declare_parameter('gripper_goal_tolerance_deg', 2.0)
        self.declare_parameter('command_update_rate_hz', 10.0)
        self.declare_parameter('action_timeout_factor', 4.0)
        self.declare_parameter('action_timeout_margin_sec', 10.0)
        self.declare_parameter('action_stall_timeout_sec', 30.0)
        self.declare_parameter('action_progress_epsilon_deg', 0.2)
        self.declare_parameter('prepare_hardware_on_unlock', True)
        self.declare_parameter('hardware_prepare_timeout_sec', 45.0)
        self.declare_parameter('hardware_reset_min_wait_sec', 3.0)
        self.declare_parameter('hardware_reset_stable_sec', 1.0)
        self.declare_parameter('hardware_reset_position_tolerance_deg', 3.0)
        self.declare_parameter('hardware_reset_speed_tolerance_deg_s', 2.0)
        self.declare_parameter('hardware_enable_group_interval_sec', 0.6)
        self.declare_parameter('hardware_hold_before_enable_sec', 0.3)
        self.declare_parameter('hardware_clear_settle_sec', 1.0)
        self.declare_parameter('joint_state_rate_hz', 30.0)
        self.declare_parameter('status_rate_hz', 10.0)
        self.declare_parameter('enforce_single_instance', True)
        self.declare_parameter('feedback_topic', '')
        self.declare_parameter('whole_body_command_topic', '')
        self.declare_parameter('gripper_command_topic', '')

        suffix = str(self.get_parameter('topic_suffix').value).strip()
        if not suffix:
            suffix = f"{os.environ.get('ROS_DOMAIN_ID', '0')}_{int(self.get_parameter('robot_id').value)}"
        self._instance_lock_handle = None
        self._acquire_instance_lock(suffix)
        feedback_topic = self._topic_or_default(
            'feedback_topic',
            f'/topic_arm_whole_body_and_gripper_current_joints_status_{suffix}',
        )
        whole_body_command_topic = self._topic_or_default(
            'whole_body_command_topic',
            f'/topic_arm_whole_body_target_joints_position_{suffix}',
        )
        gripper_command_topic = self._topic_or_default(
            'gripper_command_topic',
            f'/topic_arm_gripper_target_joints_position_{suffix}',
        )

        self._feedback_topic = feedback_topic
        self._whole_body_command_topic = whole_body_command_topic
        self._gripper_command_topic = gripper_command_topic
        self._current = {}
        self._controller_target = {}
        self._preview = {}
        self._details = {}
        self._feedback_monotonic = None
        self._hardware_enabled = False
        self._hardware_ready = False
        self._hardware_unlock_mode = 'locked'
        self._hardware_prepare = None
        self._leg_waist_enabled = False
        self._last_operator_activity = time.monotonic()
        self._last_event = '等待机器人关节反馈'
        self._last_event_level = 'info'
        self._recent_own_payloads = {}
        self._last_external_command_at = None
        self._operator_heartbeat_required = bool(
            self.get_parameter('require_operator_heartbeat').value
        )
        self._operator_heartbeat_timeout = max(
            0.15,
            float(self.get_parameter('operator_heartbeat_timeout_sec').value),
        )
        self._last_operator_heartbeat = None
        self._operator_session = ''
        self._operator_heartbeat_lost = False
        self._active_goals = {}
        self._goal_sent_groups = set()
        self._action_state = None

        self._joint_state_pub = self.create_publisher(
            JointState, '/control_center_joint_control/joint_states', 10
        )
        self._target_joint_state_pub = self.create_publisher(
            JointState, '/control_center_joint_control/target_joint_states', 10
        )
        self._status_pub = self.create_publisher(
            String, '/control_center_joint_control/status', 10
        )
        self._whole_body_pub = self.create_publisher(
            String, whole_body_command_topic, 10
        )
        self._gripper_pub = self.create_publisher(
            String, gripper_command_topic, 10
        )
        # Reset unlock uses the vendor state machine. No-reset unlock uses the
        # vendor's documented mode/clear/enable interfaces, but first latches
        # the measured pose and enables one group at a time so startup does not
        # jump to the reset pose or overload heartbeat protection.
        self._control_reset_pub = self.create_publisher(
            String, f'/control_reset_{suffix}', 10
        )
        self._joint_clear_error_pub = self.create_publisher(
            String, f'/topic_arm_joints_clear_error_{suffix}', 10
        )
        self._joint_enable_pub = self.create_publisher(
            String, f'/topic_arm_joints_set_enable_state_{suffix}', 10
        )
        self._mode_clients = {
            'left_arm': (
                self.create_client(
                    SetParameters,
                    f'/node_mod_motor_left_arm_control_{suffix}/set_parameters',
                ),
                'param_arm_left_right_arm_joint_control_mode',
            ),
            'right_arm': (
                self.create_client(
                    SetParameters,
                    f'/node_mod_motor_right_arm_control_{suffix}/set_parameters',
                ),
                'param_arm_left_right_arm_joint_control_mode',
            ),
            'neck': (
                self.create_client(
                    SetParameters,
                    f'/node_mod_motor_neck_control_{suffix}/set_parameters',
                ),
                'param_arm_neck_joint_control_mode',
            ),
            'waist_leg': (
                self.create_client(
                    SetParameters,
                    f'/node_mod_motor_waist_leg_control_{suffix}/set_parameters',
                ),
                'param_arm_leg_waist_joint_control_mode',
            ),
        }
        self._mode_state_clients = {
            name: (
                self.create_client(
                    GetParameters,
                    client.srv_name.rsplit('/', 1)[0] + '/get_parameters',
                ),
                parameter_name,
            )
            for name, (client, parameter_name) in self._mode_clients.items()
        }

        self.create_subscription(String, feedback_topic, self._on_feedback, 50)
        self.create_subscription(
            String, '/control_center_joint_control/preview_command', self._on_preview, 10
        )
        self.create_subscription(
            String, '/control_center_joint_control/apply_command', self._on_apply, 10
        )
        self.create_subscription(
            String,
            '/control_center_joint_control/action_command',
            self._on_action_command,
            10,
        )
        self.create_subscription(
            String, whole_body_command_topic, self._on_whole_body_target_observed, 10
        )
        self.create_subscription(
            String,
            str(self.get_parameter('operator_heartbeat_topic').value),
            self._on_operator_heartbeat,
            10,
        )
        self.create_service(
            SetBool, '/control_center_joint_control/set_hardware_enabled',
            self._set_hardware_enabled,
        )
        self.create_service(
            SetBool,
            '/control_center_joint_control/set_hardware_enabled_without_reset',
            self._set_hardware_enabled_without_reset,
        )
        self.create_service(
            SetBool, '/control_center_joint_control/set_leg_waist_enabled',
            self._set_leg_waist_enabled,
        )

        joint_rate = max(1.0, float(self.get_parameter('joint_state_rate_hz').value))
        status_rate = max(1.0, float(self.get_parameter('status_rate_hz').value))
        command_rate = max(
            1.0, float(self.get_parameter('command_update_rate_hz').value)
        )
        self.create_timer(1.0 / joint_rate, self._publish_joint_states)
        self.create_timer(1.0 / status_rate, self._publish_status)
        self.create_timer(1.0 / command_rate, self._advance_motion)
        self.create_timer(0.1, self._advance_hardware_prepare)
        self.create_timer(0.25, self._check_auto_lock)
        self.get_logger().info(
            f'Monitoring {feedback_topic}; command output is LOCKED by default'
        )

    def _acquire_instance_lock(self, suffix):
        if not bool(self.get_parameter('enforce_single_instance').value):
            self.get_logger().warning('Single-instance protection is disabled')
            return
        safe_suffix = ''.join(
            character if character.isalnum() or character in ('-', '_') else '_'
            for character in suffix
        )
        path = f'/tmp/control_center_joint_control_{safe_suffix}.lock'
        handle = open(path, 'a+', encoding='utf-8')
        try:
            fcntl.flock(handle.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as error:
            handle.close()
            raise RuntimeError(
                f'Another control_center_joint_control bridge already owns {path}. '
                'Close the existing launch before starting a second one.'
            ) from error
        handle.seek(0)
        handle.truncate()
        handle.write(f'{os.getpid()}\n')
        handle.flush()
        self._instance_lock_handle = handle

    def _topic_or_default(self, parameter_name, default):
        value = str(self.get_parameter(parameter_name).value).strip()
        return value or default

    def _event(self, text, level='info'):
        self._last_event = str(text)
        self._last_event_level = level
        # rclpy keys log call sites by source line and rejects changing the
        # severity at one call site, so keep each severity on its own line.
        if level == 'error':
            self.get_logger().error(self._last_event)
        elif level in ('warn', 'warning'):
            self.get_logger().warning(self._last_event)
        else:
            self.get_logger().info(self._last_event)

    def _on_feedback(self, message):
        try:
            data = json.loads(message.data)
            current = {}
            details = {}
            controller_target = {}
            for group_name in ('leg_waist', 'left_arm', 'right_arm', 'neck'):
                joint_names = group_joint_names(group_name)
                state = data[FEEDBACK_KEYS[group_name]]
                positions = state['position']
                if len(positions) != len(joint_names):
                    raise ValueError(
                        f'{FEEDBACK_KEYS[group_name]} positions: '
                        f'expected {len(joint_names)}, got {len(positions)}'
                    )
                target_values = data.get(TARGET_FEEDBACK_KEYS[group_name], positions)
                for index, name in enumerate(joint_names):
                    current[name] = float(positions[index])
                    controller_target[name] = float(target_values[index])
                    details[name] = {
                        'speed': self._array_value(state, 'speed', index),
                        'torque': self._array_value(state, 'torque', index),
                        'temperature': self._array_value(state, 'temperature', index),
                        'communication_lost': bool(
                            self._array_value(state, 'communication_lost', index, False)
                        ),
                    }
            for name, key in GRIPPER_FEEDBACK_KEYS.items():
                state = data[key]
                current[name] = float(state['position'][0])
                controller_target[name] = current[name]
                details[name] = {
                    'speed': self._array_value(state, 'speed', 0),
                    'torque': self._array_value(state, 'torque', 0),
                    'temperature': self._array_value(state, 'temperature', 0),
                    'communication_lost': bool(
                        self._array_value(state, 'communication_lost', 0, False)
                    ),
                }
            first_feedback = not self._current
            self._current = current
            self._controller_target = controller_target
            self._details = details
            self._feedback_monotonic = time.monotonic()
            if first_feedback:
                self._preview = dict(current)
                self._event(f'已连接机器人，收到 {len(current)} 个关节状态')
        except Exception as error:
            self._event(f'关节反馈解析失败：{error}', 'error')

    @staticmethod
    def _array_value(state, key, index, default=0.0):
        values = state.get(key, [])
        return values[index] if index < len(values) else default

    def _on_preview(self, message):
        if self._action_state is not None:
            return
        try:
            group_name, targets, _speed_scale, _motion_profile = (
                self._parse_operator_command(message.data)
            )
            self._preview.update(targets)
            self._last_operator_activity = time.monotonic()
        except Exception as error:
            self._event(f'预览目标无效：{error}', 'warn')

    def _on_apply(self, message):
        try:
            group_name, targets, speed_scale, motion_profile = (
                self._parse_operator_command(message.data)
            )
            single_joint = json.loads(message.data).get('single_joint')
        except Exception as error:
            self._event(f'发送目标无效：{error}', 'warn')
            return
        if self._action_state is not None:
            self._event('关键帧动作正在运行，已拒绝单组关节目标', 'warn')
            return
        if not self._is_feedback_fresh():
            self._lock('机器人反馈已断开，已拒绝发送并锁定')
            return
        lost_joints = self._communication_lost_joints(targets)
        if lost_joints:
            self._lock(
                f'目标组存在 {len(lost_joints)} 个通信离线关节，'
                '已拒绝发送并锁定真机控制'
            )
            return
        if not self._hardware_enabled:
            self._event('真机控制仍为锁定状态，仅保留目标预览', 'warn')
            return
        if not self._hardware_ready:
            self._event('厂家电机正在切换位置模式并使能，请稍候', 'warn')
            return
        if group_name == 'leg_waist' and not self._leg_waist_enabled:
            self._event('腰腿控制未二次解锁，已拒绝发送', 'warn')
            return

        if (
            group_name == 'leg_waist'
            and single_joint is None
            and not motion_profile
            and matches_body_height_profile(targets)
            and abs(
                float(targets['Joint_Waist_Yaw'])
                - float(self._current['Joint_Waist_Yaw'])
            ) <= 1.0
        ):
            # Saved poses such as “默认” do not carry the UI motion-profile
            # tag. Detect their height-curve leg targets so returning from a
            # lowered pose still uses synchronized ankle/knee/waist progress.
            motion_profile = 'body_height'
        if motion_profile == 'body_height':
            # Height owns ankle, knee and waist pitch. Freeze waist yaw from
            # fresh feedback even if an older yaw preview exists.
            targets = dict(targets)
            targets['Joint_Waist_Yaw'] = float(
                self._current['Joint_Waist_Yaw']
            )
        self._preview.update(targets)
        self._last_operator_activity = time.monotonic()
        previous = self._active_goals.get(group_name, {})
        commanded = {
            joint: float(previous.get('commands', {}).get(
                joint, self._current[joint]
            ))
            for joint in targets
        }
        if single_joint is not None:
            # Cancel old goals for unedited members of this group. Preserve
            # the selected joint's progressive command for continuous dragging.
            for joint in targets:
                if joint != single_joint:
                    commanded[joint] = float(self._current[joint])
        if motion_profile == 'body_height':
            commanded['Joint_Waist_Yaw'] = float(
                self._current['Joint_Waist_Yaw']
            )
        self._active_goals[group_name] = {
            'targets': dict(targets),
            'speed_scale': speed_scale,
            'start': dict(commanded),
            'commands': dict(commanded),
            'motion_profile': motion_profile,
            'coordinated_progress': 0.0,
        }
        self._goal_sent_groups.discard(group_name)
        maximum_error = max(
            abs(targets[name] - self._current[name]) for name in targets
        )
        profile_text = '、踝膝腰同步' if motion_profile == 'body_height' else ''
        self._event(
            f"已接收“{GROUPS[group_name]['label']}”目标，"
            f"将以 {speed_scale * 100:.0f}% 速度渐进跟随{profile_text}"
            f"（最大偏差 {maximum_error:.1f}°）"
        )

    def _advance_motion(self):
        if self._action_state is not None:
            self._advance_action()
        else:
            self._advance_active_goals()

    def _on_action_command(self, message):
        try:
            data = json.loads(message.data)
            command = str(data.get('command', ''))
            if command == 'cancel':
                self._cancel_action('动作已由操作员停止')
                return
            if command != 'start':
                raise ValueError('command must be start or cancel')
            record = normalize_action_record(data.get('action', {}))
            name = str(data.get('name', '')).strip() or '未命名动作'
        except Exception as error:
            self._event(f'动作命令无效：{error}', 'warn')
            return

        if not self._is_feedback_fresh():
            self._lock('机器人反馈已断开，无法开始动作')
            return
        if not self._hardware_enabled:
            self._event('真机控制仍为锁定状态，无法开始动作', 'warn')
            return
        if not self._hardware_ready:
            self._event('厂家复位/使能尚未完成，无法开始动作', 'warn')
            return
        joints = action_joint_names(record['groups'])
        lost_joints = self._communication_lost_joints(joints)
        if lost_joints:
            self._lock(
                f'动作包含 {len(lost_joints)} 个通信离线关节，已拒绝执行'
            )
            return
        if 'leg_waist' in record['groups'] and not self._leg_waist_enabled:
            self._event('动作包含腰腿关节，请先完成腰腿二次解锁', 'warn')
            return
        maximum_change = max(
            abs(
                float(frame['joints_deg'][joint])
                - float(self._current[joint])
            )
            for frame in record['keyframes']
            for joint in joints
        )
        if maximum_change < 0.2:
            labels = '、'.join(
                GROUPS[group]['label'] for group in record['groups']
            )
            self._event(
                f'动作只记录了“{labels}”，所有关键帧与当前姿态'
                f'基本相同（最大差值 {maximum_change:.2f}°），未下发',
                'warn',
            )
            return

        self._active_goals.clear()
        self._goal_sent_groups.clear()
        now = time.monotonic()
        self._action_state = {
            'name': name,
            'record': record,
            'joint_names': joints,
            'frame_index': 0,
            'loop_index': 0,
            'stage': 'transition',
            'stage_label': '到达关键帧',
            'frame_started': now,
            'start_values': {
                joint: float(self._current[joint]) for joint in joints
            },
            'commanded_values': {
                joint: float(self._current[joint]) for joint in joints
            },
            'progress': 0.0,
        }
        self._prepare_action_frame(self._action_state, now)
        self._preview.update(record['keyframes'][0]['joints_deg'])
        self._last_operator_activity = now
        self._event(
            f'开始执行动作“{name}”：{len(record["keyframes"])} 个关键帧，'
            f'循环 {record["loop_count"]} 次，'
            f'速度 {record["speed_percent"]}%，'
            f'最大关节变化 {maximum_change:.1f}°',
            'warn',
        )

    def _publish_action_targets(self, state, desired, frame):
        groups = state['record']['groups']
        body_groups = ('leg_waist', 'left_arm', 'right_arm', 'neck')
        controlled_body = [group for group in body_groups if group in groups]
        speed_scale = state['record']['speed_percent'] / 100.0
        if controlled_body:
            lead = max(
                0.1, float(self.get_parameter('max_single_step_deg').value)
            ) * speed_scale
            payload = {}
            for group_name in body_groups:
                if group_name in controlled_body:
                    values = []
                    for joint in group_joint_names(group_name):
                        command = self._stable_progressive_target(
                            self._current[joint],
                            state['commanded_values'][joint],
                            desired[joint],
                            frame['joints_deg'][joint],
                            state['start_values'][joint],
                            lead,
                        )
                        state['commanded_values'][joint] = command
                        values.append(command)
                else:
                    values = [
                        self._controller_target[joint]
                        for joint in group_joint_names(group_name)
                    ]
                payload[GROUPS[group_name]['command_key']] = values
            self._publish_vendor(self._whole_body_pub, payload)

        if 'grippers' in groups:
            lead = max(
                0.1,
                float(self.get_parameter(
                    'max_gripper_command_lead_deg'
                ).value),
            ) * speed_scale
            left = self._stable_progressive_target(
                self._current['Joint_Left_Gripper'],
                state['commanded_values']['Joint_Left_Gripper'],
                desired['Joint_Left_Gripper'],
                frame['joints_deg']['Joint_Left_Gripper'],
                state['start_values']['Joint_Left_Gripper'],
                lead,
            )
            right = self._stable_progressive_target(
                self._current['Joint_Right_Gripper'],
                state['commanded_values']['Joint_Right_Gripper'],
                desired['Joint_Right_Gripper'],
                frame['joints_deg']['Joint_Right_Gripper'],
                state['start_values']['Joint_Right_Gripper'],
                lead,
            )
            state['commanded_values']['Joint_Left_Gripper'] = left
            state['commanded_values']['Joint_Right_Gripper'] = right
            self._publish_vendor(
                self._gripper_pub,
                {
                    'left_gripper_target_joints_position': [left],
                    'right_gripper_target_joints_position': [right],
                },
            )

    def _action_target_reached(self, targets):
        body_tolerance = max(
            0.0, float(self.get_parameter('goal_tolerance_deg').value)
        )
        gripper_tolerance = max(
            0.0,
            float(self.get_parameter('gripper_goal_tolerance_deg').value),
        )
        return all(
            abs(float(self._current[joint]) - float(target)) <= (
                gripper_tolerance
                if joint in GRIPPER_JOINTS else body_tolerance
            )
            for joint, target in targets.items()
        )

    def _advance_action(self):
        state = self._action_state
        if state is None:
            return
        if not self._hardware_enabled or not self._is_feedback_fresh():
            if self._hardware_enabled:
                self._lock('动作执行中反馈断开，已停止动作并锁定')
            return
        if (
            'leg_waist' in state['record']['groups']
            and not self._leg_waist_enabled
        ):
            self._cancel_action('腰腿二次解锁已关闭，动作已停止')
            return
        lost = self._communication_lost_joints(state['joint_names'])
        if lost:
            self._lock(
                f'动作执行中检测到 {len(lost)} 个关节通信离线，'
                '已停止动作并锁定'
            )
            return

        now = time.monotonic()
        self._last_operator_activity = now
        frame = state['record']['keyframes'][state['frame_index']]
        if state['stage'] == 'transition':
            speed_scale = state['record']['speed_percent'] / 100.0
            duration = float(frame['transition_sec']) / speed_scale
            progress = min(1.0, (now - state['frame_started']) / duration)
            desired = interpolate_joint_values(
                state['start_values'],
                frame['joints_deg'],
                progress,
                frame['easing'],
            )
            state['progress'] = progress
            self._preview.update(desired)
            self._publish_action_targets(state, desired, frame)
            maximum_error = self._action_frame_max_error(frame)
            epsilon = max(
                0.01,
                float(self.get_parameter(
                    'action_progress_epsilon_deg'
                ).value),
            )
            if maximum_error <= state['best_error_deg'] - epsilon:
                state['best_error_deg'] = maximum_error
                state['last_progress_at'] = now
            if progress < 1.0:
                return
            if self._action_target_reached(frame['joints_deg']):
                state['stage'] = 'hold'
                state['stage_label'] = '停留'
                state['hold_started'] = now
                state['progress'] = 1.0
                return
            state['stage_label'] = (
                f'等待实际到达，最大误差 {maximum_error:.1f}°'
            )
            stall_timeout = max(
                1.0,
                float(self.get_parameter(
                    'action_stall_timeout_sec'
                ).value),
            )
            if now - state['last_progress_at'] > stall_timeout:
                self._cancel_action(
                    f'关键帧 {state["frame_index"] + 1} 连续 '
                    f'{stall_timeout:.0f} 秒无到达进展，动作已停止'
                )
            elif now > state['frame_deadline']:
                allowed = state['frame_deadline'] - state['frame_started']
                self._cancel_action(
                    f'关键帧 {state["frame_index"] + 1} 已等待 '
                    f'{allowed:.1f} 秒仍未到达，超过动态宽限上限，'
                    '动作已停止'
                )
            return

        if now - state['hold_started'] < float(frame['hold_sec']):
            return
        self._advance_action_frame(now)

    def _advance_action_frame(self, now):
        state = self._action_state
        state['frame_index'] += 1
        frame_count = len(state['record']['keyframes'])
        if state['frame_index'] >= frame_count:
            state['loop_index'] += 1
            if state['loop_index'] >= state['record']['loop_count']:
                name = state['name']
                self._action_state = None
                self._event(f'动作“{name}”已完成')
                return
            state['frame_index'] = 0
        state['stage'] = 'transition'
        state['stage_label'] = '到达关键帧'
        state['frame_started'] = now
        state['start_values'] = {
            joint: float(self._current[joint])
            for joint in state['joint_names']
        }
        state['commanded_values'] = dict(state['start_values'])
        state['progress'] = 0.0
        self._prepare_action_frame(state, now)
        target = state['record']['keyframes'][state['frame_index']]['joints_deg']
        self._preview.update(target)

    def _action_frame_max_error(self, frame):
        return max(
            abs(float(self._current[joint]) - float(target))
            for joint, target in frame['joints_deg'].items()
        )

    def _prepare_action_frame(self, state, now):
        """Build a generous deadline from frame distance and selected speed."""
        frame = state['record']['keyframes'][state['frame_index']]
        speed_scale = state['record']['speed_percent'] / 100.0
        command_rate = max(
            1.0,
            float(self.get_parameter('command_update_rate_hz').value),
        )
        body_lead = max(
            0.1, float(self.get_parameter('max_single_step_deg').value)
        )
        gripper_lead = max(
            0.1,
            float(self.get_parameter(
                'max_gripper_command_lead_deg'
            ).value),
        )
        expected_by_joint = []
        for joint, target in frame['joints_deg'].items():
            lead = gripper_lead if joint in GRIPPER_JOINTS else body_lead
            rate = max(0.1, lead * speed_scale * command_rate)
            expected_by_joint.append(
                abs(float(target) - float(state['start_values'][joint])) / rate
            )
        scaled_transition = float(frame['transition_sec']) / speed_scale
        expected = max([scaled_transition] + expected_by_joint)
        factor = max(
            1.0, float(self.get_parameter('action_timeout_factor').value)
        )
        margin = max(
            0.0,
            float(self.get_parameter('action_timeout_margin_sec').value),
        )
        state['frame_deadline'] = now + expected * factor + margin
        state['best_error_deg'] = self._action_frame_max_error(frame)
        state['last_progress_at'] = now

    def _cancel_action(self, reason):
        if self._action_state is None:
            self._event(reason)
            return
        groups = list(self._action_state['record']['groups'])
        if self._is_feedback_fresh():
            body_groups = ('leg_waist', 'left_arm', 'right_arm', 'neck')
            if any(group in groups for group in body_groups):
                self._publish_vendor(
                    self._whole_body_pub,
                    {
                        GROUPS[group]['command_key']: [
                            self._current[joint]
                            for joint in group_joint_names(group)
                        ]
                        for group in body_groups
                    },
                )
            if 'grippers' in groups:
                self._publish_vendor(
                    self._gripper_pub,
                    {
                        'left_gripper_target_joints_position': [
                            self._current['Joint_Left_Gripper']
                        ],
                        'right_gripper_target_joints_position': [
                            self._current['Joint_Right_Gripper']
                        ],
                    },
                )
        self._action_state = None
        self._active_goals.clear()
        self._goal_sent_groups.clear()
        self._preview = dict(self._current)
        self._last_operator_activity = time.monotonic()
        self._event(reason, 'warn')

    def _advance_active_goals(self):
        """Advance commands monotonically so feedback noise cannot reverse them."""
        if not self._active_goals:
            return
        if not self._hardware_enabled or not self._is_feedback_fresh():
            if self._hardware_enabled:
                self._lock('机器人反馈已断开，渐进运动已停止并锁定')
            return
        # A commanded pose is active operator work. Do not let the inactivity
        # timeout interrupt a slow, feedback-bounded coordinated transition.
        self._last_operator_activity = time.monotonic()

        active_joint_names = [
            joint
            for record in self._active_goals.values()
            for joint in record['targets']
        ]
        lost_joints = self._communication_lost_joints(active_joint_names)
        if lost_joints:
            self._lock(
                f'运动中检测到 {len(lost_joints)} 个关节通信离线，'
                '已停止运动并锁定真机控制'
            )
            return

        body_tolerance = max(
            0.0, float(self.get_parameter('goal_tolerance_deg').value)
        )
        gripper_tolerance = max(
            0.0,
            float(self.get_parameter('gripper_goal_tolerance_deg').value),
        )
        body_lead = max(
            0.1, float(self.get_parameter('max_single_step_deg').value)
        )
        gripper_lead = max(
            0.1, float(self.get_parameter('max_gripper_command_lead_deg').value)
        )
        completed = []

        body_groups = ('leg_waist', 'left_arm', 'right_arm', 'neck')
        active_body_groups = [
            name for name in body_groups if name in self._active_goals
        ]
        if active_body_groups:
            payload = {}
            for group_name in body_groups:
                if group_name in self._active_goals:
                    record = self._active_goals[group_name]
                    goal = record['targets']
                    group_lead = body_lead * record['speed_scale']
                    if (
                        group_name == 'leg_waist'
                        and record.get('motion_profile') == 'body_height'
                    ):
                        synchronized, progress = (
                            self._coordinated_progressive_targets(
                                BODY_HEIGHT_JOINTS,
                                self._current,
                                record['commands'],
                                record['start'],
                                goal,
                                record.get('coordinated_progress', 0.0),
                                group_lead,
                            )
                        )
                        record['coordinated_progress'] = progress
                        record['commands'].update(synchronized)
                        # Waist yaw is held while ankle, knee and waist pitch
                        # follow one shared 0..100 percent trajectory.
                        record['commands']['Joint_Waist_Yaw'] = (
                            record['start']['Joint_Waist_Yaw']
                        )
                        values = [
                            record['commands'][joint]
                            for joint in group_joint_names(group_name)
                        ]
                    else:
                        values = []
                        for joint in group_joint_names(group_name):
                            command = self._stable_progressive_target(
                                self._current[joint],
                                record['commands'][joint],
                                goal[joint],
                                goal[joint],
                                record['start'][joint],
                                group_lead,
                            )
                            record['commands'][joint] = command
                            values.append(command)
                else:
                    values = [
                        self._controller_target[joint]
                        for joint in group_joint_names(group_name)
                    ]
                payload[GROUPS[group_name]['command_key']] = values
            self._publish_vendor(self._whole_body_pub, payload)
            self._goal_sent_groups.update(active_body_groups)

        if 'grippers' in self._active_goals:
            record = self._active_goals['grippers']
            goal = record['targets']
            group_lead = gripper_lead * record['speed_scale']
            left = self._stable_progressive_target(
                self._current['Joint_Left_Gripper'],
                record['commands']['Joint_Left_Gripper'],
                goal['Joint_Left_Gripper'],
                goal['Joint_Left_Gripper'],
                record['start']['Joint_Left_Gripper'],
                group_lead,
            )
            right = self._stable_progressive_target(
                self._current['Joint_Right_Gripper'],
                record['commands']['Joint_Right_Gripper'],
                goal['Joint_Right_Gripper'],
                goal['Joint_Right_Gripper'],
                record['start']['Joint_Right_Gripper'],
                group_lead,
            )
            record['commands']['Joint_Left_Gripper'] = left
            record['commands']['Joint_Right_Gripper'] = right
            self._publish_vendor(
                self._gripper_pub,
                {
                    'left_gripper_target_joints_position': [left],
                    'right_gripper_target_joints_position': [right],
                },
            )
            self._goal_sent_groups.add('grippers')

        for group_name, record in self._active_goals.items():
            if group_name not in self._goal_sent_groups:
                continue
            goal = record['targets']
            tolerance = (
                gripper_tolerance
                if group_name == 'grippers' else body_tolerance
            )
            if all(
                abs(self._current[joint] - target) <= tolerance
                for joint, target in goal.items()
            ):
                completed.append(group_name)
        for group_name in completed:
            del self._active_goals[group_name]
            self._goal_sent_groups.discard(group_name)
        if completed:
            labels = '、'.join(GROUPS[name]['label'] for name in completed)
            self._event(
                f'{labels}已进入目标容差并停止重复微调'
                f'（身体 ±{body_tolerance:.1f}°）'
            )

    @staticmethod
    def _stable_progressive_target(
        current, previous, trajectory_target, final_target, start, lead
    ):
        """Bound lead from feedback while never reversing due to sensor noise."""
        current = float(current)
        previous = float(previous)
        trajectory_target = float(trajectory_target)
        final_target = float(final_target)
        start = float(start)
        lead = max(0.1, float(lead))
        candidate = min(
            current + lead,
            max(current - lead, trajectory_target),
        )
        if final_target >= start:
            return min(final_target, max(previous, candidate))
        return max(final_target, min(previous, candidate))

    @staticmethod
    def _coordinated_progressive_targets(
        joint_names, current, previous, start, final, previous_progress, lead
    ):
        """Advance multiple joints with one feedback-bounded progress value."""
        lead = max(0.1, float(lead))
        previous_progress = min(1.0, max(0.0, float(previous_progress)))
        deltas = {
            name: float(final[name]) - float(start[name])
            for name in joint_names
        }
        moving = {
            name: delta for name, delta in deltas.items()
            if abs(delta) > 1e-9
        }
        if not moving:
            return ({name: float(final[name]) for name in joint_names}, 1.0)

        largest_delta = max(abs(delta) for delta in moving.values())
        nominal_progress = min(
            1.0,
            previous_progress + lead / largest_delta,
        )
        feedback_progress = 1.0
        for name, delta in moving.items():
            observed_progress = (
                float(current[name]) - float(start[name])
            ) / delta
            feedback_progress = min(
                feedback_progress,
                observed_progress + lead / abs(delta),
            )
        progress = max(
            previous_progress,
            min(1.0, nominal_progress, feedback_progress),
        )
        commands = {
            name: float(start[name]) + progress * deltas[name]
            for name in joint_names
        }
        return commands, progress

    def _parse_operator_command(self, payload):
        data = json.loads(payload)
        group_name = str(data['group'])
        if group_name not in GROUPS:
            raise ValueError(f'unknown group {group_name}')
        requested = data.get('targets_deg', {})
        single_joint = data.get('single_joint')
        targets = resolve_operator_targets(
            group_name, requested, self._current, single_joint
        )
        speed_scale = float(data.get('speed_scale', 1.0))
        if not math.isfinite(speed_scale) or not 0.1 <= speed_scale <= 2.0:
            raise ValueError('speed_scale must be between 0.1 and 2.0')
        motion_profile = str(data.get('motion_profile', '')).strip()
        if motion_profile not in ('', 'body_height'):
            raise ValueError(f'unknown motion profile {motion_profile}')
        if motion_profile == 'body_height' and group_name != 'leg_waist':
            raise ValueError('body_height motion profile requires leg_waist group')
        if single_joint is not None and motion_profile:
            raise ValueError('single_joint cannot run a coordinated motion profile')
        return group_name, targets, speed_scale, motion_profile

    def _publish_vendor(self, publisher, payload):
        text = json.dumps(payload, ensure_ascii=False, separators=(',', ':'))
        now = time.monotonic()
        self._prune_recent_own_payloads(now)
        self._recent_own_payloads[text] = now
        publisher.publish(String(data=text))

    def _on_whole_body_target_observed(self, message):
        now = time.monotonic()
        self._prune_recent_own_payloads(now)
        if message.data in self._recent_own_payloads:
            return
        # The official vendor reset publishes joint targets of its own.
        # Those messages are expected during the preparation state machine.
        if self._hardware_prepare is not None:
            return
        self._last_external_command_at = now
        if self._hardware_enabled:
            self._lock('检测到其他节点下发全身关节命令，为避免冲突已自动锁定')

    def _on_operator_heartbeat(self, message):
        session = str(message.data).strip()
        if not session:
            return
        if not self._operator_session:
            self._operator_session = session
        elif session != self._operator_session:
            if not self._operator_heartbeat_lost:
                self._operator_heartbeat_lost = True
                self.get_logger().error(
                    'Different control-center session detected; this joint '
                    'bridge remains locked to its original owner'
                )
                active = bool(self._active_goals or self._action_state)
                if active:
                    self._hold_active_joint_targets_once()
                if self._hardware_enabled:
                    self._lock(
                        '控制中心会话已更换，旧关节桥接已停止并锁定'
                    )
            return
        self._last_operator_heartbeat = time.monotonic()

    def _hold_active_joint_targets_once(self):
        if not self._hardware_enabled or not self._is_feedback_fresh():
            return False
        if (
            self._last_external_command_at is not None
            and time.monotonic() - self._last_external_command_at < 0.50
        ):
            self._event(
                '检测到其他关节控制端刚发送命令；不发送保持目标，已停止本桥接后续输出',
                'warn',
            )
            return False

        groups = set(self._active_goals) | set(self._goal_sent_groups)
        if self._action_state is not None:
            groups.update(self._action_state['record'].get('groups', []))
        if not groups:
            return False

        body_payload = {}
        body_groups = ('leg_waist', 'left_arm', 'right_arm', 'neck')
        if groups.intersection(body_groups):
            for group_name in body_groups:
                body_payload[GROUPS[group_name]['command_key']] = [
                    self._current[joint]
                    for joint in group_joint_names(group_name)
                ]
        if 'grippers' in groups:
            self._publish_vendor(
                self._gripper_pub,
                {
                    'left_gripper_target_joints_position': [
                        self._current['Joint_Left_Gripper']
                    ],
                    'right_gripper_target_joints_position': [
                        self._current['Joint_Right_Gripper']
                    ],
                },
            )
        if body_payload:
            self._publish_vendor(self._whole_body_pub, body_payload)
        if body_payload or 'grippers' in groups:
            self._event('控制界面失联，已发送一次当前位置保持目标', 'warn')
            return True
        return False

    def _prune_recent_own_payloads(self, now):
        self._recent_own_payloads = {
            payload: stamp
            for payload, stamp in self._recent_own_payloads.items()
            if now - stamp < 1.5
        }

    def _set_hardware_enabled(self, request, response):
        prepare = bool(self.get_parameter('prepare_hardware_on_unlock').value)
        return self._set_hardware_enabled_mode(
            request, response, prepare_reset=prepare
        )

    def _set_hardware_enabled_without_reset(self, request, response):
        return self._set_hardware_enabled_mode(
            request, response, prepare_reset=False
        )

    def _set_hardware_enabled_mode(self, request, response, prepare_reset):
        if not request.data:
            self._lock('真机控制已手动锁定')
            response.success = True
            response.message = self._last_event
            return response
        if not self._is_feedback_fresh():
            response.success = False
            response.message = '机器人关节反馈未连接或已超时'
            self._lock(response.message)
            return response
        lost_joints = self._communication_lost_joints(BODY_JOINTS)
        if lost_joints:
            response.success = False
            response.message = (
                f'检测到 {len(lost_joints)} 个身体关节通信离线；'
                '请先恢复厂家电机服务'
            )
            self._lock(response.message)
            return response
        self._hardware_enabled = True
        self._hardware_ready = False
        self._hardware_unlock_mode = (
            'reset' if prepare_reset else 'without_reset'
        )
        self._leg_waist_enabled = False
        self._last_operator_activity = time.monotonic()
        self._preview = dict(self._current)
        if prepare_reset:
            self._hardware_prepare = {
                'started': time.monotonic(),
                'stage': 'wait_reset_subscriber',
                'published_at': None,
                'stable_since': None,
            }
            response.message = '正在执行厂家官方复位与使能流程；机器人可能移动'
        else:
            # A fresh vendor arm service starts with its motors disabled. A
            # software-only unlock accepts commands but cannot move it. Latch
            # the measured pose, then prepare the hardware without invoking
            # the vendor reset pose.
            now = time.monotonic()
            self._hardware_prepare = {
                'started': now,
                'stage': 'check_modes',
                'futures': {},
                'next_publish': now,
                'enable_group_index': 0,
                'stable_since': None,
                'hold_targets': {
                    name: float(self._current[name])
                    for name in BODY_JOINTS
                },
            }
            response.message = (
                '正在锁存当前姿态、检查位置模式并同步使能；不会清错或前往复位姿态'
            )
        response.success = True
        self._event(response.message, 'warn')
        return response

    def _advance_hardware_prepare(self):
        prepare = self._hardware_prepare
        if not self._hardware_enabled or prepare is None:
            return
        now = time.monotonic()
        timeout = float(self.get_parameter('hardware_prepare_timeout_sec').value)
        if now - prepare['started'] > timeout:
            self._lock('厂家电机位置模式/使能准备超时，已自动锁定')
            return

        if prepare['stage'] == 'wait_reset_subscriber':
            if self._control_reset_pub.get_subscription_count() == 0:
                return
            # Match the official vendor example exactly. Its callback owns
            # position-mode selection, clear-error, enable and PID sequencing.
            self._control_reset_pub.publish(String(data=''))
            prepare['published_at'] = now
            prepare['stage'] = 'wait_reset_stable'
            self._event(
                '厂家复位命令已发送，正在等待关节通信正常并稳定',
                'warn',
            )
            return

        if prepare['stage'] == 'wait_reset_stable':
            minimum_wait = max(
                0.0,
                float(self.get_parameter('hardware_reset_min_wait_sec').value),
            )
            if now - prepare['published_at'] < minimum_wait:
                return

            if self._communication_lost_joints(BODY_JOINTS):
                prepare['stable_since'] = None
                return

            position_tolerance = max(
                0.1,
                float(self.get_parameter(
                    'hardware_reset_position_tolerance_deg'
                ).value),
            )
            speed_tolerance = max(
                0.1,
                float(self.get_parameter(
                    'hardware_reset_speed_tolerance_deg_s'
                ).value),
            )
            maximum_error = max(
                abs(
                    self._current.get(name, 0.0)
                    - self._controller_target.get(
                        name, self._current.get(name, 0.0)
                    )
                )
                for name in BODY_JOINTS
            )
            maximum_speed = max(
                abs(float(self._details.get(name, {}).get('speed', 0.0)))
                for name in BODY_JOINTS
            )
            if maximum_error > position_tolerance or maximum_speed > speed_tolerance:
                prepare['stable_since'] = None
                return

            if prepare['stable_since'] is None:
                prepare['stable_since'] = now
                return
            stable_duration = max(
                0.0,
                float(self.get_parameter('hardware_reset_stable_sec').value),
            )
            if now - prepare['stable_since'] < stable_duration:
                return

            self._hardware_prepare = None
            self._hardware_ready = True
            self._last_operator_activity = now
            self._preview = dict(self._current)
            self._event('厂家复位与使能已完成，可以控制真机关节')
            return

        if prepare['stage'] == 'check_modes':
            for name, (client, parameter_name) in self._mode_state_clients.items():
                if name in prepare['futures'] or not client.service_is_ready():
                    continue
                request = GetParameters.Request()
                request.names = [parameter_name]
                prepare['futures'][name] = client.call_async(request)
            if len(prepare['futures']) != len(self._mode_state_clients):
                return
            if not all(future.done() for future in prepare['futures'].values()):
                return
            try:
                invalid_modes = []
                for name, future in prepare['futures'].items():
                    result = future.result()
                    value = (
                        '' if result is None or not result.values
                        else result.values[0].string_value
                    )
                    if value != 'position':
                        invalid_modes.append(f'{name}={value or "unknown"}')
                if invalid_modes:
                    self._lock(
                        '控制器不是位置模式，拒绝无复位使能：'
                        + '、'.join(invalid_modes)
                    )
                    return
            except Exception as error:
                self._lock(f'读取厂家控制模式异常：{error}')
                return
            prepare['stage'] = 'enable_hold'
            prepare['next_publish'] = now

        if prepare['stage'] == 'enable_hold' and now >= prepare['next_publish']:
            if self._joint_enable_pub.get_subscription_count() == 0:
                return
            # Do not clear errors or rewrite mode parameters here: either can
            # briefly remove holding torque. Publish the measured hold target
            # first, then enable every body joint atomically in one message.
            self._publish_hardware_hold_pose(prepare)
            self._publish_joint_boolean_command(
                self._joint_enable_pub, BODY_JOINTS
            )
            prepare['stage'] = 'verify_hold'
            prepare['stable_since'] = None
            prepare['next_publish'] = now + 0.1
            self._event('已同步使能全部关节，正在确认当前姿态稳定', 'warn')
            return

        if prepare['stage'] == 'modes':
            for name, (client, parameter_name) in self._mode_clients.items():
                if name in prepare['futures'] or not client.service_is_ready():
                    continue
                request = SetParameters.Request()
                request.parameters = [
                    Parameter(
                        parameter_name,
                        Parameter.Type.STRING,
                        'position',
                    ).to_parameter_msg()
                ]
                prepare['futures'][name] = client.call_async(request)
            if len(prepare['futures']) != len(self._mode_clients):
                return
            if not all(future.done() for future in prepare['futures'].values()):
                return
            try:
                failed = []
                for name, future in prepare['futures'].items():
                    result = future.result()
                    if (
                        result is None
                        or not result.results
                        or not all(item.successful for item in result.results)
                    ):
                        failed.append(name)
                if failed:
                    self._lock(
                        '厂家位置模式设置失败：' + '、'.join(failed)
                    )
                    return
            except Exception as error:
                self._lock(f'厂家位置模式设置异常：{error}')
                return
            prepare['stage'] = 'hold'
            prepare['next_publish'] = now

        if prepare['stage'] == 'hold' and now >= prepare['next_publish']:
            self._publish_hardware_hold_pose(prepare)
            prepare['stage'] = 'clear'
            prepare['next_publish'] = now + max(
                0.0,
                float(self.get_parameter(
                    'hardware_hold_before_enable_sec'
                ).value),
            )
            self._event('已锁存当前姿态，正在清除厂家电机错误', 'warn')
            return

        if prepare['stage'] == 'clear' and now >= prepare['next_publish']:
            if self._joint_clear_error_pub.get_subscription_count() == 0:
                return
            self._publish_hardware_hold_pose(prepare)
            self._publish_joint_boolean_command(self._joint_clear_error_pub)
            prepare['stage'] = 'enable'
            prepare['next_publish'] = now + max(
                0.1,
                float(self.get_parameter('hardware_clear_settle_sec').value),
            )
            return

        if prepare['stage'] == 'enable' and now >= prepare['next_publish']:
            if self._joint_enable_pub.get_subscription_count() == 0:
                return
            enable_groups = ('neck', 'left_arm', 'right_arm', 'leg_waist')
            group_index = prepare['enable_group_index']
            already_enabled = [
                joint
                for group_name in enable_groups[:group_index]
                for joint in group_joint_names(group_name)
            ]
            lost_joints = self._communication_lost_joints(already_enabled)
            if lost_joints:
                self._lock(
                    f'分组使能期间检测到 {len(lost_joints)} 个关节通信离线，'
                    '已停止并锁定真机控制'
                )
                return
            group_name = enable_groups[group_index]
            self._publish_hardware_hold_pose(prepare)
            self._publish_joint_boolean_command(
                self._joint_enable_pub,
                group_joint_names(group_name),
            )
            prepare['enable_group_index'] += 1
            interval = max(
                0.5,
                float(self.get_parameter(
                    'hardware_enable_group_interval_sec'
                ).value),
            )
            prepare['next_publish'] = now + interval
            if prepare['enable_group_index'] >= len(enable_groups):
                prepare['stage'] = 'verify_hold'
                prepare['stable_since'] = None
                self._event('厂家电机已分组使能，正在确认当前姿态稳定', 'warn')
            return

        if prepare['stage'] == 'verify_hold' and now >= prepare['next_publish']:
            lost_joints = self._communication_lost_joints(BODY_JOINTS)
            if lost_joints:
                self._lock(
                    f'无复位使能后检测到 {len(lost_joints)} 个关节通信离线，'
                    '已停止并锁定真机控制'
                )
                return
            self._publish_hardware_hold_pose(prepare)
            position_tolerance = max(
                0.1,
                float(self.get_parameter(
                    'hardware_reset_position_tolerance_deg'
                ).value),
            )
            speed_tolerance = max(
                0.1,
                float(self.get_parameter(
                    'hardware_reset_speed_tolerance_deg_s'
                ).value),
            )
            maximum_error = max(
                abs(self._current[name] - prepare['hold_targets'][name])
                for name in BODY_JOINTS
            )
            maximum_speed = max(
                abs(float(self._details.get(name, {}).get('speed', 0.0)))
                for name in BODY_JOINTS
            )
            if maximum_error > position_tolerance or maximum_speed > speed_tolerance:
                prepare['stable_since'] = None
                prepare['next_publish'] = now + 0.2
                return
            if prepare['stable_since'] is None:
                prepare['stable_since'] = now
                prepare['next_publish'] = now + 0.1
                return
            stable_duration = max(
                0.0,
                float(self.get_parameter('hardware_reset_stable_sec').value),
            )
            if now - prepare['stable_since'] < stable_duration:
                prepare['next_publish'] = now + 0.1
                return
            self._hardware_prepare = None
            self._hardware_ready = True
            self._last_operator_activity = now
            self._preview = dict(self._current)
            self._event('无复位使能已完成，机器人保持当前姿态，可以控制真机关节')

    def _publish_hardware_hold_pose(self, prepare):
        hold_targets = prepare['hold_targets']
        payload = {
            GROUPS[group_name]['command_key']: [
                hold_targets[joint]
                for joint in group_joint_names(group_name)
            ]
            for group_name in ('leg_waist', 'left_arm', 'right_arm', 'neck')
        }
        self._publish_vendor(self._whole_body_pub, payload)

    @staticmethod
    def _joint_boolean_payload(joint_names=BODY_JOINTS):
        return {
            name: True
            for name in joint_names
        }

    def _publish_joint_boolean_command(self, publisher, joint_names=BODY_JOINTS):
        publisher.publish(String(data=json.dumps(
            self._joint_boolean_payload(joint_names),
            ensure_ascii=False,
            separators=(',', ':'),
        )))

    def _set_leg_waist_enabled(self, request, response):
        if request.data and not self._hardware_enabled:
            response.success = False
            response.message = '请先解锁真机控制'
            self._leg_waist_enabled = False
            return response
        if request.data and not self._hardware_ready:
            response.success = False
            response.message = '厂家电机仍在准备中，请稍候'
            self._leg_waist_enabled = False
            return response
        self._leg_waist_enabled = bool(request.data)
        if not request.data:
            if (
                self._action_state is not None
                and 'leg_waist' in self._action_state['record']['groups']
            ):
                self._cancel_action('腰腿二次解锁已关闭，动作已停止')
            self._active_goals.pop('leg_waist', None)
            self._goal_sent_groups.discard('leg_waist')
        self._last_operator_activity = time.monotonic()
        response.success = True
        response.message = (
            '腰腿高风险控制已解锁' if request.data else '腰腿高风险控制已锁定'
        )
        self._event(response.message, 'warn' if request.data else 'info')
        return response

    def _lock(self, reason):
        self._hardware_enabled = False
        self._hardware_ready = False
        self._hardware_unlock_mode = 'locked'
        self._hardware_prepare = None
        self._leg_waist_enabled = False
        self._active_goals.clear()
        self._goal_sent_groups.clear()
        self._action_state = None
        self._event(reason, 'warn')

    def _is_feedback_fresh(self):
        if self._feedback_monotonic is None:
            return False
        timeout = float(self.get_parameter('feedback_timeout_sec').value)
        return time.monotonic() - self._feedback_monotonic <= timeout

    def _communication_lost_joints(self, joint_names):
        return [
            name
            for name in joint_names
            if self._details.get(name, {}).get('communication_lost', True)
        ]

    def _check_auto_lock(self):
        if self._operator_heartbeat_required:
            heartbeat_stale = (
                self._last_operator_heartbeat is None
                or time.monotonic() - self._last_operator_heartbeat
                > self._operator_heartbeat_timeout
            )
            if heartbeat_stale:
                if not self._operator_heartbeat_lost:
                    self._operator_heartbeat_lost = True
                    active = bool(self._active_goals or self._action_state)
                    if active:
                        self._hold_active_joint_targets_once()
                    if self._hardware_enabled:
                        self._lock(
                            '控制中心界面心跳超时，关节目标已停止并自动锁定；请重新解锁'
                        )
                return
        if not self._hardware_enabled:
            return
        if self._active_goals or self._action_state is not None:
            self._last_operator_activity = time.monotonic()
            return
        timeout = float(self.get_parameter('hardware_unlock_timeout_sec').value)
        if time.monotonic() - self._last_operator_activity >= timeout:
            self._lock(f'{timeout:.0f} 秒无操作，真机控制已自动锁定')

    def _publish_joint_states(self):
        # Never keep emitting a cached pose after its source has gone stale.
        # This also prevents an intentionally non-exclusive test bridge from
        # fighting a healthy instance with old data.
        if not self._current or not self._is_feedback_fresh():
            return
        current = dict(self._current)
        preview = dict(self._preview or self._current)
        self._joint_state_pub.publish(self._make_model_joint_state(current))
        self._target_joint_state_pub.publish(self._make_model_joint_state(preview))

    def destroy_node(self):
        # On orderly bridge shutdown, hold only the groups this bridge was
        # actively moving, then stop publishing.  A heartbeat timeout uses
        # the same bounded handoff for a GUI crash.
        if self._hardware_enabled and (self._active_goals or self._action_state):
            self._hold_active_joint_targets_once()
            self._lock('关节控制桥接正在关闭，控制输出已锁定')
        return super().destroy_node()

    def _make_model_joint_state(self, values_deg):
        message = JointState()
        message.header.stamp = self.get_clock().now().to_msg()
        message.name = list(BASE_MODEL_JOINTS) + list(ALL_JOINTS)
        message.position = [0.0] * len(BASE_MODEL_JOINTS)
        message.velocity = [0.0] * len(BASE_MODEL_JOINTS)
        message.effort = [0.0] * len(BASE_MODEL_JOINTS)
        for name in ALL_JOINTS:
            # The vendor gripper feedback is motor rotation (10..360 deg), not
            # the URDF wrist-to-tool joint. Keep that model joint neutral.
            position = 0.0 if name in GRIPPER_JOINTS else math.radians(values_deg[name])
            detail = self._details.get(name, {})
            velocity = (
                0.0 if name in GRIPPER_JOINTS
                else math.radians(float(detail.get('speed', 0.0)))
            )
            message.position.append(position)
            message.velocity.append(velocity)
            message.effort.append(float(detail.get('torque', 0.0)))
        return message

    def _publish_status(self):
        now = time.monotonic()
        age = None if self._feedback_monotonic is None else now - self._feedback_monotonic
        unlock_timeout = float(self.get_parameter('hardware_unlock_timeout_sec').value)
        remaining = (
            max(0.0, unlock_timeout - (now - self._last_operator_activity))
            if self._hardware_enabled else 0.0
        )
        joints = {}
        for name in ALL_JOINTS:
            detail = self._details.get(name, {})
            joints[name] = {
                'current_deg': self._current.get(name),
                'preview_deg': self._preview.get(name),
                'controller_target_deg': self._controller_target.get(name),
                'speed_deg_s': detail.get('speed'),
                'torque': detail.get('torque'),
                'temperature_c': detail.get('temperature'),
                'communication_lost': detail.get('communication_lost', True),
                'lower_deg': JOINT_LIMITS[name][0],
                'upper_deg': JOINT_LIMITS[name][1],
            }
        payload = {
            'connected': self._is_feedback_fresh(),
            'feedback_age_sec': age,
            'joint_count': len(self._current),
            'hardware_enabled': self._hardware_enabled,
            'hardware_ready': self._hardware_ready,
            'hardware_unlock_mode': self._hardware_unlock_mode,
            'hardware_prepare_stage': (
                None if self._hardware_prepare is None
                else self._hardware_prepare['stage']
            ),
            'leg_waist_enabled': self._leg_waist_enabled,
            'auto_lock_remaining_sec': remaining,
            'active_goal_groups': list(self._active_goals),
            'active_goal_error_deg': {
                group_name: max(
                    abs(self._current.get(joint, target) - target)
                    for joint, target in record['targets'].items()
                )
                for group_name, record in self._active_goals.items()
            },
            'active_goal_speed_percent': {
                group_name: record['speed_scale'] * 100.0
                for group_name, record in self._active_goals.items()
            },
            'action_running': self._action_state is not None,
            'action_name': (
                '' if self._action_state is None
                else self._action_state['name']
            ),
            'action_frame_index': (
                0 if self._action_state is None
                else self._action_state['frame_index']
            ),
            'action_frame_count': (
                0 if self._action_state is None
                else len(self._action_state['record']['keyframes'])
            ),
            'action_loop_index': (
                0 if self._action_state is None
                else self._action_state['loop_index']
            ),
            'action_loop_count': (
                1 if self._action_state is None
                else self._action_state['record']['loop_count']
            ),
            'action_speed_percent': (
                100 if self._action_state is None
                else self._action_state['record']['speed_percent']
            ),
            'action_progress': (
                0.0 if self._action_state is None
                else self._action_state['progress']
            ),
            'action_stage': (
                '' if self._action_state is None
                else self._action_state['stage_label']
            ),
            'event': self._last_event,
            'event_level': self._last_event_level,
            'feedback_topic': self._feedback_topic,
            'whole_body_command_topic': self._whole_body_command_topic,
            'gripper_command_topic': self._gripper_command_topic,
            'whole_body_command_publisher_count': self.count_publishers(
                self._whole_body_command_topic
            ),
            'joints': joints,
        }
        self._status_pub.publish(
            String(data=json.dumps(payload, ensure_ascii=False, separators=(',', ':')))
        )


def main(args=None):
    rclpy.init(args=args)
    node = JointBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except KeyboardInterrupt:
            pass
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except KeyboardInterrupt:
                pass


if __name__ == '__main__':
    main()
