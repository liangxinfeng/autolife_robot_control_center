import inspect

import pytest

from control_center_joint_control.body_height import (
    BODY_HEIGHT_JOINTS,
    matches_body_height_profile,
)
from control_center_joint_control.joint_bridge import JointBridge


WATER = {
    'Joint_Ankle': 30.1286429413094,
    'Joint_Knee': 58.47622958096591,
    'Joint_Waist_Pitch': 28.729747425426137,
}


def coordinated_step(start, current, lead=5.0):
    helper = JointBridge._coordinated_progressive_targets
    arguments = {
        'joint_names': BODY_HEIGHT_JOINTS,
        'current': current,
        'start': start,
        'final': WATER,
        'previous_progress': 0.0,
        'lead': lead,
    }
    if 'previous' in inspect.signature(helper).parameters:
        arguments['previous'] = start
    return helper(**arguments)


def test_water_pose_is_a_height_profile():
    assert matches_body_height_profile(WATER)


def test_first_step_keeps_water_joint_ratio():
    start = {name: 0.0 for name in BODY_HEIGHT_JOINTS}
    commands, progress = coordinated_step(start, start)
    assert 0.0 < progress < 1.0
    assert commands['Joint_Knee'] == pytest.approx(5.0)
    assert (
        commands['Joint_Ankle'] / commands['Joint_Knee']
        == pytest.approx(WATER['Joint_Ankle'] / WATER['Joint_Knee'])
    )
    assert (
        commands['Joint_Waist_Pitch'] / commands['Joint_Knee']
        == pytest.approx(WATER['Joint_Waist_Pitch'] / WATER['Joint_Knee'])
    )


def test_slowest_joint_bounds_shared_progress():
    start = {name: 0.0 for name in BODY_HEIGHT_JOINTS}
    current = {
        'Joint_Ankle': 6.0,
        'Joint_Knee': 1.0,
        'Joint_Waist_Pitch': 6.0,
    }
    commands, _progress = coordinated_step(start, current, lead=2.0)
    assert commands['Joint_Knee'] <= 3.0 + 1e-9
    ratio = commands['Joint_Ankle'] / commands['Joint_Knee']
    assert ratio == pytest.approx(WATER['Joint_Ankle'] / WATER['Joint_Knee'])


def test_active_pose_refreshes_inactivity_timer():
    fake = type('FakeBridge', (), {
        '_hardware_enabled': True,
        '_active_goals': {'leg_waist': object()},
        '_action_state': None,
        '_last_operator_activity': 0.0,
    })()
    JointBridge._check_auto_lock(fake)
    assert fake._last_operator_activity > 0.0
