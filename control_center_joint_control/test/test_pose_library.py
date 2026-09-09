from control_center_joint_control.joint_definitions import ALL_JOINTS
from control_center_joint_control.pose_library import (
    load_pose_library,
    make_pose_record,
    save_pose_library,
    validate_pose_name,
)

import pytest


def sample_pose():
    values = {name: 0.0 for name in ALL_JOINTS}
    values['Joint_Left_Gripper'] = 100.0
    values['Joint_Right_Gripper'] = 120.0
    return values


def test_pose_library_round_trip(tmp_path):
    path = tmp_path / 'poses.yaml'
    poses = {'站立': make_pose_record(sample_pose())}
    save_pose_library(path, poses)
    loaded = load_pose_library(path)
    assert list(loaded) == ['站立']
    assert loaded['站立']['joints_deg']['Joint_Left_Gripper'] == 100.0
    assert loaded['站立']['saved_at']


def test_invalid_pose_name_is_rejected():
    with pytest.raises(ValueError):
        validate_pose_name('')
    with pytest.raises(ValueError):
        validate_pose_name('folder/name')
