import math

import pytest

from control_center_joint_control.joint_definitions import (
    ALL_JOINTS,
    BODY_JOINTS,
    GROUPS,
    JOINT_TO_GROUP,
    clamp_joint,
    step_toward,
)


def test_joint_count_and_groups_are_unique():
    assert len(ALL_JOINTS) == 23
    assert len(set(ALL_JOINTS)) == 23
    assert len(BODY_JOINTS) == 21
    assert set(JOINT_TO_GROUP) == set(ALL_JOINTS)


def test_clamp_joint_uses_verified_limits():
    assert clamp_joint('Joint_Neck_Pitch', -100.0) == -45.0
    assert clamp_joint('Joint_Neck_Pitch', 100.0) == 30.0
    assert clamp_joint('Joint_Left_Elbow', 80.0) == 80.0
    assert clamp_joint('Joint_Left_Gripper', 0.0) == 10.0
    assert clamp_joint('Joint_Right_Gripper', 500.0) == 360.0


def test_clamp_rejects_non_finite():
    with pytest.raises(ValueError):
        clamp_joint('Joint_Knee', math.nan)


def test_leg_waist_is_only_high_risk_group():
    risky = [name for name, group in GROUPS.items() if group['high_risk']]
    assert risky == ['leg_waist']


def test_step_toward_limits_large_targets_without_rejecting_them():
    assert step_toward(10.0, 12.0, 5.0) == 12.0
    assert step_toward(10.0, 50.0, 5.0) == 15.0
    assert step_toward(10.0, -50.0, 5.0) == 5.0
    with pytest.raises(ValueError):
        step_toward(0.0, 10.0, 0.0)
