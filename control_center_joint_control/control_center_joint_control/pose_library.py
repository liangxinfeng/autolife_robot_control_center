"""Persistent full-body pose library helpers."""

from datetime import datetime
import os
from pathlib import Path
import re

import yaml

from .joint_definitions import ALL_JOINTS, clamp_joint


POSE_FILE_VERSION = 1
_VALID_NAME = re.compile(r'^[^/\\\x00-\x1f]{1,32}$')


def validate_pose_name(name):
    name = str(name).strip()
    if not _VALID_NAME.match(name):
        raise ValueError('姿态名称需为1到32个字符，且不能包含斜杠或控制字符')
    return name


def normalize_joint_values(values):
    if not isinstance(values, dict):
        raise ValueError('joints_deg must be an object')
    missing = [name for name in ALL_JOINTS if name not in values]
    if missing:
        raise ValueError(f'姿态缺少关节：{missing}')
    return {name: clamp_joint(name, values[name]) for name in ALL_JOINTS}


def load_pose_library(path):
    path = Path(path)
    if not path.exists():
        return {}
    raw = yaml.safe_load(path.read_text(encoding='utf-8')) or {}
    poses = raw.get('poses', {})
    if not isinstance(poses, dict):
        raise ValueError('姿态文件中的 poses 必须是字典')
    result = {}
    for raw_name, record in poses.items():
        try:
            name = validate_pose_name(raw_name)
            joints = normalize_joint_values(record.get('joints_deg', {}))
            result[name] = {
                'saved_at': str(record.get('saved_at', '')),
                'joints_deg': joints,
            }
        except (TypeError, ValueError):
            continue
    return result


def save_pose_library(path, poses):
    path = Path(path).resolve()
    path.parent.mkdir(parents=True, exist_ok=True)
    normalized = {}
    for raw_name, record in sorted(poses.items()):
        name = validate_pose_name(raw_name)
        normalized[name] = {
            'saved_at': str(record.get('saved_at', '')),
            'joints_deg': normalize_joint_values(record.get('joints_deg', {})),
        }
    document = {'version': POSE_FILE_VERSION, 'poses': normalized}
    temporary = path.with_name(path.name + '.tmp')
    temporary.write_text(
        yaml.safe_dump(document, allow_unicode=True, sort_keys=False),
        encoding='utf-8',
    )
    os.replace(temporary, path)


def make_pose_record(joints_deg):
    return {
        'saved_at': datetime.now().astimezone().isoformat(timespec='seconds'),
        'joints_deg': normalize_joint_values(joints_deg),
    }
