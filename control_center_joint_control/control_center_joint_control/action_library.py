"""Persistent keyframe action library and interpolation helpers."""

from datetime import datetime
import math
import os
from pathlib import Path
import re

import yaml

from .joint_definitions import GROUPS, GROUP_ORDER, clamp_joint, group_joint_names


ACTION_FILE_VERSION = 1
EASING_MODES = ('smooth', 'linear')
_VALID_NAME = re.compile(r'^[^/\\\x00-\x1f]{1,32}$')


def validate_action_name(name):
    name = str(name).strip()
    if not _VALID_NAME.match(name):
        raise ValueError('动作名称需为1到32个字符，且不能包含斜杠或控制字符')
    return name


def normalize_groups(groups):
    if not isinstance(groups, (list, tuple)):
        raise ValueError('groups 必须是列表')
    result = []
    for raw_group in groups:
        group = str(raw_group)
        if group not in GROUPS:
            raise ValueError(f'未知关节组：{group}')
        if group not in result:
            result.append(group)
    result.sort(key=GROUP_ORDER.index)
    if not result:
        raise ValueError('动作至少需要选择一个关节组')
    return result


def action_joint_names(groups):
    return [
        joint
        for group in normalize_groups(groups)
        for joint in group_joint_names(group)
    ]


def _finite_bounded(value, label, lower, upper):
    value = float(value)
    if not math.isfinite(value) or not lower <= value <= upper:
        raise ValueError(f'{label}必须在 {lower:g} 到 {upper:g} 之间')
    return value


def normalize_keyframe(frame, joint_names):
    if not isinstance(frame, dict):
        raise ValueError('关键帧必须是字典')
    values = frame.get('joints_deg', {})
    if not isinstance(values, dict):
        raise ValueError('关键帧 joints_deg 必须是字典')
    allowed = set(joint_names)
    if set(values) != allowed:
        missing = sorted(allowed - set(values))
        extra = sorted(set(values) - allowed)
        raise ValueError(f'关键帧关节不匹配；缺少={missing}，多余={extra}')
    easing = str(frame.get('easing', 'smooth'))
    if easing not in EASING_MODES:
        raise ValueError(f'未知平滑方式：{easing}')
    return {
        'transition_sec': _finite_bounded(
            frame.get('transition_sec', 1.5), '到达时间', 0.2, 30.0
        ),
        'hold_sec': _finite_bounded(
            frame.get('hold_sec', 0.2), '停留时间', 0.0, 30.0
        ),
        'easing': easing,
        'joints_deg': {
            name: clamp_joint(name, values[name])
            for name in joint_names
        },
    }


def normalize_action_record(record):
    if not isinstance(record, dict):
        raise ValueError('动作记录必须是字典')
    groups = normalize_groups(record.get('groups', []))
    joint_names = action_joint_names(groups)
    raw_frames = record.get('keyframes', [])
    if not isinstance(raw_frames, list) or not raw_frames:
        raise ValueError('动作至少需要一个关键帧')
    if len(raw_frames) > 200:
        raise ValueError('单个动作最多允许200个关键帧')
    loop_count = int(record.get('loop_count', 1))
    if not 1 <= loop_count <= 20:
        raise ValueError('循环次数必须在1到20之间')
    speed_percent = int(record.get('speed_percent', 100))
    if not 10 <= speed_percent <= 200:
        raise ValueError('真机执行速度必须在10%到200%之间')
    return {
        'saved_at': str(record.get('saved_at', '')),
        'groups': groups,
        'loop_count': loop_count,
        'speed_percent': speed_percent,
        'keyframes': [
            normalize_keyframe(frame, joint_names)
            for frame in raw_frames
        ],
    }


def make_action_record(groups, keyframes, loop_count=1, speed_percent=100):
    return normalize_action_record({
        'saved_at': datetime.now().astimezone().isoformat(timespec='seconds'),
        'groups': list(groups),
        'loop_count': int(loop_count),
        'speed_percent': int(speed_percent),
        'keyframes': list(keyframes),
    })


def load_action_library(path):
    path = Path(path)
    if not path.exists():
        return {}
    raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    actions = raw.get('actions', {})
    if not isinstance(actions, dict):
        raise ValueError('动作文件中的 actions 必须是字典')
    result = {}
    for raw_name, record in actions.items():
        try:
            name = validate_action_name(raw_name)
            result[name] = normalize_action_record(record)
        except (TypeError, ValueError):
            continue
    return result


def save_action_library(path, actions):
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    normalized = {}
    for raw_name, record in sorted(actions.items()):
        name = validate_action_name(raw_name)
        normalized[name] = normalize_action_record(record)
    document = {'version': ACTION_FILE_VERSION, 'actions': normalized}
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
        encoding='utf-8',
    )
    os.replace(temporary, path)


def easing_fraction(progress, mode='smooth'):
    progress = min(1.0, max(0.0, float(progress)))
    if mode == 'linear':
        return progress
    if mode != 'smooth':
        raise ValueError(f'未知平滑方式：{mode}')
    return progress * progress * (3.0 - 2.0 * progress)


def interpolate_joint_values(start, target, progress, mode='smooth'):
    fraction = easing_fraction(progress, mode)
    return {
        name: float(start[name]) + (
            float(value) - float(start[name])
        ) * fraction
        for name, value in target.items()
    }
