"""Export keyframe actions in the vendor recorded-trajectory pickle format."""

import json
import math
import os
from pathlib import Path
import pickle
import time

from .action_library import (
    easing_fraction,
    normalize_action_record,
)
from .joint_definitions import (
    ALL_JOINTS,
    FEEDBACK_KEYS,
    GRIPPER_FEEDBACK_KEYS,
    GRIPPER_JOINTS,
    TARGET_FEEDBACK_KEYS,
    group_joint_names,
)
from .pose_library import normalize_joint_values


DEFAULT_SAMPLE_RATE_HZ = 100.0
MAX_EXPORT_SAMPLES = 30000
BODY_GROUPS = ('leg_waist', 'left_arm', 'right_arm', 'neck')


def _state_payload(timestamp, positions, velocities):
    payload = {'timestamp': float(timestamp)}
    for group_name in ('leg_waist', 'left_arm', 'right_arm'):
        names = group_joint_names(group_name)
        payload[FEEDBACK_KEYS[group_name]] = {
            'position': [positions[name] for name in names],
            'speed': [velocities[name] for name in names],
            'torque': [0.0] * len(names),
            'temperature': [0.0] * len(names),
            'communication_lost': [False] * len(names),
        }
    for name in GRIPPER_JOINTS:
        payload[GRIPPER_FEEDBACK_KEYS[name]] = {
            'position': [positions[name]],
            'speed': [velocities[name]],
            'torque': [0.0],
            'temperature': [0.0],
            'communication_lost': [False],
        }
    names = group_joint_names('neck')
    payload[FEEDBACK_KEYS['neck']] = {
        'position': [positions[name] for name in names],
        'speed': [velocities[name] for name in names],
        'torque': [0.0] * len(names),
        'temperature': [0.0] * len(names),
        'communication_lost': [False] * len(names),
    }
    for group_name in BODY_GROUPS:
        names = group_joint_names(group_name)
        payload[TARGET_FEEDBACK_KEYS[group_name]] = [
            positions[name] for name in names
        ]
    return payload


def build_vendor_trajectory(
    record,
    baseline_joints_deg,
    sample_rate_hz=DEFAULT_SAMPLE_RATE_HZ,
    timestamp_base=None,
):
    record = normalize_action_record(record)
    baseline = normalize_joint_values(baseline_joints_deg)
    sample_rate_hz = float(sample_rate_hz)
    if not math.isfinite(sample_rate_hz) or sample_rate_hz <= 0.0:
        raise ValueError('采样频率必须大于0')
    speed_scale = record['speed_percent'] / 100.0
    segment_specs = []
    total_samples = 1
    for _loop in range(record['loop_count']):
        for frame in record['keyframes']:
            transition = float(frame['transition_sec']) / speed_scale
            transition_steps = max(1, round(transition * sample_rate_hz))
            hold_steps = max(0, round(
                float(frame['hold_sec']) * sample_rate_hz
            ))
            total_samples += transition_steps + hold_steps
            segment_specs.append((
                frame, transition, transition_steps, hold_steps
            ))
    if total_samples > MAX_EXPORT_SAMPLES:
        duration = sum(
            transition + hold_steps / sample_rate_hz
            for _frame, transition, _steps, hold_steps in segment_specs
        )
        raise ValueError(
            f'动作导出后约 {duration:.1f} 秒、{total_samples} 个采样点，'
            f'超过兼容PKL上限 {MAX_EXPORT_SAMPLES} 点'
        )

    timestamp_base = (
        time.time() if timestamp_base is None else float(timestamp_base)
    )
    current = dict(baseline)
    elapsed = 0.0
    zero_velocity = {name: 0.0 for name in ALL_JOINTS}
    trajectory = [(
        0.0,
        json.dumps(
            _state_payload(timestamp_base, current, zero_velocity),
            ensure_ascii=False,
        ),
    )]
    for frame, transition, transition_steps, hold_steps in segment_specs:
        start = dict(current)
        step_duration = transition / transition_steps
        for step_index in range(1, transition_steps + 1):
            fraction = easing_fraction(
                step_index / transition_steps,
                frame['easing'],
            )
            next_positions = dict(current)
            for name, target in frame['joints_deg'].items():
                next_positions[name] = (
                    start[name] + (float(target) - start[name]) * fraction
                )
            velocities = {
                name: (
                    next_positions[name] - current[name]
                ) / step_duration
                for name in ALL_JOINTS
            }
            elapsed += step_duration
            current = next_positions
            trajectory.append((
                elapsed,
                json.dumps(
                    _state_payload(
                        timestamp_base + elapsed,
                        current,
                        velocities,
                    ),
                    ensure_ascii=False,
                ),
            ))
        hold_step_duration = 1.0 / sample_rate_hz
        for _ in range(hold_steps):
            elapsed += hold_step_duration
            trajectory.append((
                elapsed,
                json.dumps(
                    _state_payload(
                        timestamp_base + elapsed,
                        current,
                        zero_velocity,
                    ),
                    ensure_ascii=False,
                ),
            ))
    return trajectory


def export_action_pickle(
    path,
    record,
    baseline_joints_deg,
    sample_rate_hz=DEFAULT_SAMPLE_RATE_HZ,
):
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    trajectory = build_vendor_trajectory(
        record,
        baseline_joints_deg,
        sample_rate_hz=sample_rate_hz,
    )
    temporary = path.with_name(path.name + '.tmp')
    try:
        with temporary.open('wb') as stream:
            pickle.dump(trajectory, stream, protocol=5)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if temporary.exists():
            temporary.unlink()
    return {
        'path': path,
        'sample_count': len(trajectory),
        'duration_sec': float(trajectory[-1][0]),
        'sample_rate_hz': float(sample_rate_hz),
    }
