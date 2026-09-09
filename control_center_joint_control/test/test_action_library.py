from pathlib import Path

import pytest

from control_center_joint_control.action_library import (
    action_joint_names,
    easing_fraction,
    interpolate_joint_values,
    load_action_library,
    make_action_record,
    normalize_action_record,
    save_action_library,
)


def right_arm_frame(value, transition=0.5, hold=0.1):
    names = action_joint_names(['right_arm'])
    return {
        'transition_sec': transition,
        'hold_sec': hold,
        'easing': 'smooth',
        'joints_deg': {
            name: (float(value) if index == 0 else 0.0)
            for index, name in enumerate(names)
        },
    }


def test_action_library_round_trip(tmp_path):
    path = tmp_path / 'actions.yaml'
    record = make_action_record(
        ['right_arm'],
        [right_arm_frame(20.0), right_arm_frame(-20.0)],
        loop_count=3,
        speed_percent=45,
    )
    save_action_library(path, {'打招呼': record})
    loaded = load_action_library(path)
    assert list(loaded) == ['打招呼']
    assert loaded['打招呼']['loop_count'] == 3
    assert loaded['打招呼']['speed_percent'] == 45
    assert len(loaded['打招呼']['keyframes']) == 2


def test_action_requires_exact_selected_joint_set():
    frame = right_arm_frame(10.0)
    frame['joints_deg'].pop(next(iter(frame['joints_deg'])))
    with pytest.raises(ValueError, match='关节不匹配'):
        normalize_action_record({
            'groups': ['right_arm'],
            'loop_count': 1,
            'keyframes': [frame],
        })


def test_smooth_interpolation_has_zero_slope_ends_and_expected_midpoint():
    assert easing_fraction(0.0, 'smooth') == 0.0
    assert easing_fraction(1.0, 'smooth') == 1.0
    values = interpolate_joint_values(
        {'joint': -20.0},
        {'joint': 20.0},
        0.5,
        'smooth',
    )
    assert values['joint'] == pytest.approx(0.0)
    assert easing_fraction(0.01, 'smooth') < 0.01
    assert easing_fraction(0.99, 'smooth') > 0.99


def test_action_limits_loop_count_and_frame_timing():
    with pytest.raises(ValueError, match='循环次数'):
        normalize_action_record({
            'groups': ['right_arm'],
            'loop_count': 21,
            'keyframes': [right_arm_frame(0.0)],
        })
    with pytest.raises(ValueError, match='到达时间'):
        normalize_action_record({
            'groups': ['right_arm'],
            'loop_count': 1,
            'keyframes': [right_arm_frame(0.0, transition=0.01)],
        })
    with pytest.raises(ValueError, match='执行速度'):
        normalize_action_record({
            'groups': ['right_arm'],
            'loop_count': 1,
            'speed_percent': 5,
            'keyframes': [right_arm_frame(0.0)],
        })
    with pytest.raises(ValueError, match='执行速度'):
        normalize_action_record({
            'groups': ['right_arm'],
            'loop_count': 1,
            'speed_percent': 201,
            'keyframes': [right_arm_frame(0.0)],
        })


def test_old_action_defaults_to_full_speed():
    record = normalize_action_record({
        'groups': ['right_arm'],
        'loop_count': 1,
        'keyframes': [right_arm_frame(10.0)],
    })
    assert record['speed_percent'] == 100


def test_action_accepts_200_percent_speed():
    record = make_action_record(
        ['right_arm'],
        [right_arm_frame(10.0)],
        speed_percent=200,
    )
    assert record['speed_percent'] == 200
