import json
import pickle

from control_center_joint_control.action_library import action_joint_names
from control_center_joint_control.joint_definitions import (
    ALL_JOINTS,
    FEEDBACK_KEYS,
    GRIPPER_FEEDBACK_KEYS,
    TARGET_FEEDBACK_KEYS,
)
from control_center_joint_control.motion_storage import (
    ACTION_FILE_NAME,
    POSE_FILE_NAME,
    action_pickle_path,
    initialize_motion_library,
    resolve_motion_library_dir,
)
from control_center_joint_control.trajectory_pickle import export_action_pickle


def _frame(value):
    names = action_joint_names(['left_arm'])
    return {
        'transition_sec': 1.0,
        'hold_sec': 0.1,
        'easing': 'smooth',
        'joints_deg': {
            name: (float(value) if index == 0 else 0.0)
            for index, name in enumerate(names)
        },
    }


def test_export_matches_vendor_recorded_pickle_shape(tmp_path):
    baseline = {
        name: (10.0 if 'Gripper' in name else 0.0)
        for name in ALL_JOINTS
    }
    record = {
        'groups': ['left_arm'],
        'loop_count': 1,
        'speed_percent': 200,
        'keyframes': [_frame(20.0), _frame(-20.0)],
    }
    output = tmp_path / 'wave.pkl'
    result = export_action_pickle(output, record, baseline)
    with output.open('rb') as stream:
        trajectory = pickle.load(stream)
    assert result['sample_count'] == len(trajectory)
    assert 100 <= len(trajectory) <= 130
    assert isinstance(trajectory, list)
    assert all(
        isinstance(item, tuple)
        and len(item) == 2
        and isinstance(item[0], float)
        and isinstance(item[1], str)
        for item in trajectory
    )
    assert trajectory[0][0] == 0.0
    assert trajectory[-1][0] > 1.1
    payload = json.loads(trajectory[-1][1])
    expected_keys = [
        'timestamp',
        FEEDBACK_KEYS['leg_waist'],
        FEEDBACK_KEYS['left_arm'],
        FEEDBACK_KEYS['right_arm'],
        GRIPPER_FEEDBACK_KEYS['Joint_Left_Gripper'],
        GRIPPER_FEEDBACK_KEYS['Joint_Right_Gripper'],
        FEEDBACK_KEYS['neck'],
        TARGET_FEEDBACK_KEYS['leg_waist'],
        TARGET_FEEDBACK_KEYS['left_arm'],
        TARGET_FEEDBACK_KEYS['right_arm'],
        TARGET_FEEDBACK_KEYS['neck'],
    ]
    assert list(payload) == expected_keys
    assert payload['left_arm_target_joint_state'][0] == -20.0
    assert payload['right_arm_target_joint_state'] == [0.0] * 7


def test_source_storage_resolution_and_initial_migration(tmp_path):
    workspace = tmp_path / 'ros2_ws'
    source_package = workspace / 'src' / 'control_center_joint_control'
    source_package.mkdir(parents=True)
    (source_package / 'package.xml').write_text('<package/>', encoding='utf-8')
    share = (
        workspace / 'install' / 'control_center_joint_control'
        / 'share' / 'control_center_joint_control'
    )
    legacy = share / 'config'
    legacy.mkdir(parents=True)
    (legacy / ACTION_FILE_NAME).write_text(
        'version: 1\nactions: {}\n', encoding='utf-8'
    )
    (legacy / POSE_FILE_NAME).write_text(
        'version: 1\nposes: {}\n', encoding='utf-8'
    )
    directory = resolve_motion_library_dir(share)
    assert directory == source_package / 'motion_library'
    initialize_motion_library(directory, legacy)
    assert (directory / ACTION_FILE_NAME).is_file()
    assert (directory / POSE_FILE_NAME).is_file()
    assert action_pickle_path(directory, '打招呼').name == '打招呼.pkl'
