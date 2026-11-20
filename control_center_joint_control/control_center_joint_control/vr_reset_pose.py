"""Persist and apply the pose used by the vendor VR system reset.

The vendor arm service reads its home-pose JSON and height setting only when
the service starts.  This module updates every installed copy atomically and
keeps an immutable, per-file snapshot so the factory/system pose can always be
inspected and restored from the operator UI.
"""

from __future__ import annotations

from copy import deepcopy
from datetime import datetime
import json
import math
import os
from pathlib import Path
import re
import socket
import tempfile


LEFT_ARM_JOINTS = (
    'Joint_Left_Shoulder_Inner', 'Joint_Left_Shoulder_Outer',
    'Joint_Left_UpperArm', 'Joint_Left_Elbow', 'Joint_Left_Forearm',
    'Joint_Left_Wrist_Upper', 'Joint_Left_Wrist_Lower',
)
RIGHT_ARM_JOINTS = tuple(name.replace('Left', 'Right') for name in LEFT_ARM_JOINTS)
NECK_JOINTS = ('Joint_Neck_Roll', 'Joint_Neck_Pitch', 'Joint_Neck_Yaw')
LEG_JOINTS = ('Joint_Ankle', 'Joint_Knee', 'Joint_Waist_Pitch', 'Joint_Waist_Yaw')
VR_RESET_JOINTS = LEG_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + NECK_JOINTS

HOME_KEYS = (
    'LEFT_ARM_MOTOR_HOME_ANGLE',
    'RIGHT_ARM_MOTOR_HOME_ANGLE',
    'NECK_MOTOR_HOME_ANGLE',
)

DEFAULT_CONFIG_PATHS = (
    Path('/home/ubuntu/Documents/AutolifeRobotArm/configs/robot_v3_1.json'),
    Path('/home/ubuntu/autolife-v3/env/lib/python3.12/site-packages/'
         'autolife_robot_arm/configs/autolife_s1/robot_v3_1.json'),
)
DEFAULT_SETTINGS_PATHS = (
    Path('/home/ubuntu/Documents/AutolifeRobotArm/src/autolife_robot_arm/settings.toml'),
    Path('/home/ubuntu/autolife-v3/env/lib/python3.12/site-packages/'
         'autolife_robot_arm/settings.toml'),
)
DEFAULT_STATE_PATH = Path.home() / '.config' / 'autolife_robot_control_center' / 'vr_reset_pose.json'
HEIGHT_SECTION = 'app_settings.robot_control_settings.reset.height'


def _existing(paths):
    return [Path(path).resolve() for path in paths if Path(path).is_file()]


def _atomic_write(path, text):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    mode = path.stat().st_mode if path.exists() else None
    fd, temporary = tempfile.mkstemp(prefix=f'.{path.name}.', dir=str(path.parent))
    try:
        with os.fdopen(fd, 'w', encoding='utf-8', newline='') as stream:
            stream.write(text)
            stream.flush()
            os.fsync(stream.fileno())
        if mode is not None:
            os.chmod(temporary, mode)
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


def _read_json(path):
    with Path(path).open('r', encoding='utf-8') as stream:
        return json.load(stream)


def _write_json(path, value):
    _atomic_write(path, json.dumps(value, ensure_ascii=False, indent=2) + '\n')


def _height_values(text):
    section = _toml_section(text, HEIGHT_SECTION)
    values = {}
    for key in ('use_current_height', 'target_height_z'):
        match = re.search(rf'(?m)^\s*{re.escape(key)}\s*=\s*([^#\r\n]+)', section)
        if match:
            values[key] = match.group(1).strip()
    return values


def _toml_section(text, name):
    match = re.search(
        rf'(?ms)^\s*\[{re.escape(name)}\]\s*$.*?(?=^\s*\[|\Z)', text
    )
    if not match:
        raise ValueError(f'厂商设置中缺少 [{name}]')
    return match.group(0)


def _replace_height_values(text, values):
    match = re.search(
        rf'(?ms)^\s*\[{re.escape(HEIGHT_SECTION)}\]\s*$.*?(?=^\s*\[|\Z)', text
    )
    if not match:
        raise ValueError(f'厂商设置中缺少 [{HEIGHT_SECTION}]')
    section = match.group(0)
    for key, raw_value in values.items():
        pattern = rf'(?m)^(\s*{re.escape(key)}\s*=\s*)[^#\r\n]*(.*)$'
        if not re.search(pattern, section):
            raise ValueError(f'厂商身高设置中缺少 {key}')
        section = re.sub(pattern, rf'\g<1>{raw_value}\g<2>', section, count=1)
    return text[:match.start()] + section + text[match.end():]


def _robot_id():
    match = re.search(r'(\d+)$', socket.gethostname())
    return match.group(1) if match else socket.gethostname()


class VrResetPoseStore:
    """Manage the vendor reset-pose files and their immutable baseline."""

    def __init__(self, config_paths=None, settings_paths=None, state_path=None):
        self.config_paths = _existing(config_paths or DEFAULT_CONFIG_PATHS)
        self.settings_paths = _existing(settings_paths or DEFAULT_SETTINGS_PATHS)
        self.state_path = Path(state_path or DEFAULT_STATE_PATH).expanduser().resolve()
        if not self.config_paths:
            raise FileNotFoundError('未找到厂商 robot_v3_1.json')
        if not self.settings_paths:
            raise FileNotFoundError('未找到厂商 settings.toml')
        self._ensure_baseline()

    def _load_state(self):
        if not self.state_path.is_file():
            return None
        return _read_json(self.state_path)

    def _capture_original(self):
        configs = {}
        for path in self.config_paths:
            data = _read_json(path)
            configs[str(path)] = {key: deepcopy(data[key]) for key in HOME_KEYS}
        settings = {}
        for path in self.settings_paths:
            text = path.read_text(encoding='utf-8')
            settings[str(path)] = _height_values(text)
        return {'configs': configs, 'settings': settings}

    def _ensure_baseline(self):
        state = self._load_state()
        if state and state.get('original', {}).get('configs'):
            return state
        state = {
            'version': 1,
            'robot_id': _robot_id(),
            'mode': 'original',
            'created_at': datetime.now().astimezone().isoformat(timespec='seconds'),
            'original': self._capture_original(),
            'custom': None,
        }
        _write_json(self.state_path, state)
        return state

    def status(self):
        state = self._ensure_baseline()
        return {
            'robot_id': state.get('robot_id', ''),
            'mode': state.get('mode', 'original'),
            'custom_saved_at': (state.get('custom') or {}).get('saved_at', ''),
            'state_path': str(self.state_path),
        }

    @staticmethod
    def _validate_pose(joints):
        missing = [name for name in VR_RESET_JOINTS if name not in joints]
        if missing:
            raise ValueError('缺少关节实测值：' + '、'.join(missing))
        result = {}
        for name in VR_RESET_JOINTS:
            value = float(joints[name])
            if not math.isfinite(value):
                raise ValueError(f'{name} 不是有效数值')
            result[name] = value
        return result

    @staticmethod
    def _height_target(joints, config):
        geometry = config.get('LEG_GEOMETRY', {})
        thigh = float(geometry.get('THIGH_LENGTH', 0.37))
        shin = float(geometry.get('SHIN_LENGTH', 0.35))
        ankle = math.radians(joints['Joint_Ankle'])
        knee = math.radians(joints['Joint_Knee'])
        height = shin * math.cos(ankle) + thigh * math.cos(ankle - knee)
        return max(-0.56, min(0.0, height - shin - thigh))

    @staticmethod
    def _home_values(joints):
        leg = [joints[name] for name in LEG_JOINTS]
        left = leg + [joints[name] for name in LEFT_ARM_JOINTS]
        right = leg + [joints[name] for name in RIGHT_ARM_JOINTS]
        neck = leg + [joints[name] for name in NECK_JOINTS]
        return {
            'LEFT_ARM_MOTOR_HOME_ANGLE': {'sitting': left, 'standing': left},
            'RIGHT_ARM_MOTOR_HOME_ANGLE': {'sitting': right, 'standing': right},
            'NECK_MOTOR_HOME_ANGLE': {'zero': neck},
        }

    def save_current(self, joints):
        joints = self._validate_pose(joints)
        homes = self._home_values(joints)
        heights = []
        for path in self.config_paths:
            data = _read_json(path)
            for key, modes in homes.items():
                if key not in data:
                    raise ValueError(f'{path} 缺少 {key}')
                for mode, values in modes.items():
                    data[key][mode] = [round(value, 6) for value in values]
            heights.append(self._height_target(joints, data))
            _write_json(path, data)
        target_height = sum(heights) / len(heights)
        for path in self.settings_paths:
            text = path.read_text(encoding='utf-8')
            text = _replace_height_values(text, {
                'use_current_height': 'false',
                'target_height_z': f'{target_height:.6f}',
            })
            _atomic_write(path, text)
        state = self._ensure_baseline()
        state['mode'] = 'custom'
        state['custom'] = {
            'saved_at': datetime.now().astimezone().isoformat(timespec='seconds'),
            'joints_deg': joints,
            'target_height_z': target_height,
        }
        _write_json(self.state_path, state)
        return self.status()

    def restore_original(self):
        state = self._ensure_baseline()
        original = state['original']
        for path in self.config_paths:
            path_key = str(path)
            snapshot = original['configs'].get(path_key)
            if snapshot is None:
                # A newly installed duplicate receives the first preserved copy.
                snapshot = next(iter(original['configs'].values()))
            data = _read_json(path)
            for key in HOME_KEYS:
                data[key] = deepcopy(snapshot[key])
            _write_json(path, data)
        for path in self.settings_paths:
            path_key = str(path)
            values = original['settings'].get(path_key)
            if values is None:
                values = next(iter(original['settings'].values()))
            text = _replace_height_values(path.read_text(encoding='utf-8'), values)
            _atomic_write(path, text)
        state['mode'] = 'original'
        state['restored_at'] = datetime.now().astimezone().isoformat(timespec='seconds')
        _write_json(self.state_path, state)
        return self.status()

    def original_pose(self):
        state = self._ensure_baseline()
        configs = next(iter(state['original']['configs'].values()))
        left = configs['LEFT_ARM_MOTOR_HOME_ANGLE']['standing']
        right = configs['RIGHT_ARM_MOTOR_HOME_ANGLE']['standing']
        neck = configs['NECK_MOTOR_HOME_ANGLE']['zero']
        result = dict(zip(LEG_JOINTS, left[:4]))
        result.update(zip(LEFT_ARM_JOINTS, left[4:11]))
        result.update(zip(RIGHT_ARM_JOINTS, right[4:11]))
        result.update(zip(NECK_JOINTS, neck[4:7]))
        height_values = next(iter(state['original']['settings'].values()))
        raw_height = height_values.get('target_height_z', '0.0')
        try:
            target_height = float(raw_height)
        except ValueError:
            target_height = 0.0
        return result, target_height

    def current_pose(self):
        """Return the reset pose currently selected in persisted config state."""
        state = self._ensure_baseline()
        custom = state.get('custom') or {}
        if state.get('mode') == 'custom' and custom.get('joints_deg'):
            pose = self._validate_pose(custom['joints_deg'])
            try:
                target_height = float(custom.get('target_height_z', 0.0))
            except (TypeError, ValueError):
                target_height = 0.0
            return pose, target_height
        return self.original_pose()
