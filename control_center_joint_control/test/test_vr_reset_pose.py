import json

from control_center_joint_control.vr_reset_pose import (
    LEFT_ARM_JOINTS,
    LEG_JOINTS,
    NECK_JOINTS,
    RIGHT_ARM_JOINTS,
    VrResetPoseStore,
)


def _config():
    return {
        'LEG_GEOMETRY': {'THIGH_LENGTH': 0.37, 'SHIN_LENGTH': 0.35},
        'LEFT_ARM_MOTOR_HOME_ANGLE': {
            'zero': [0] * 11, 'sitting': [1] * 11, 'standing': [1] * 11,
        },
        'RIGHT_ARM_MOTOR_HOME_ANGLE': {
            'zero': [0] * 11, 'sitting': [2] * 11, 'standing': [2] * 11,
        },
        'NECK_MOTOR_HOME_ANGLE': {'zero': [3] * 7},
    }


def _settings():
    return '''[app_settings.robot_control_settings.reset.height]\nuse_current_height = false\ntarget_height_z = 0.0\ntarget_speed = 15.0\n\n[following]\nvalue = 1\n'''


def test_save_keeps_immutable_original_and_restore(tmp_path):
    config_path = tmp_path / 'robot_v2_2.json'
    settings_path = tmp_path / 'settings.toml'
    state_path = tmp_path / 'state.json'
    config_path.write_text(json.dumps(_config()), encoding='utf-8')
    settings_path.write_text(_settings(), encoding='utf-8')
    store = VrResetPoseStore([config_path], [settings_path], state_path)

    initial_pose, initial_height = store.current_pose()
    assert list(initial_pose[name] for name in LEG_JOINTS) == [1, 1, 1, 1]
    assert initial_height == 0.0

    names = LEG_JOINTS + LEFT_ARM_JOINTS + RIGHT_ARM_JOINTS + NECK_JOINTS
    pose = {name: float(index + 10) for index, name in enumerate(names)}
    pose['Joint_Ankle'] = 20.0
    pose['Joint_Knee'] = 40.0
    store.save_current(pose)

    changed = json.loads(config_path.read_text(encoding='utf-8'))
    assert changed['LEFT_ARM_MOTOR_HOME_ANGLE']['standing'] == [
        pose[name] for name in LEG_JOINTS + LEFT_ARM_JOINTS
    ]
    assert changed['RIGHT_ARM_MOTOR_HOME_ANGLE']['sitting'] == [
        pose[name] for name in LEG_JOINTS + RIGHT_ARM_JOINTS
    ]
    assert changed['NECK_MOTOR_HOME_ANGLE']['zero'] == [
        pose[name] for name in LEG_JOINTS + NECK_JOINTS
    ]
    assert store.status()['mode'] == 'custom'
    current_pose, current_height = store.current_pose()
    assert current_pose == pose
    assert current_height < 0.0

    # A later save must never replace the original baseline.
    store.save_current({name: value + 5 for name, value in pose.items()})
    original_pose, original_height = store.original_pose()
    assert list(original_pose[name] for name in LEG_JOINTS) == [1, 1, 1, 1]
    assert original_height == 0.0

    store.restore_original()
    restored = json.loads(config_path.read_text(encoding='utf-8'))
    assert restored['LEFT_ARM_MOTOR_HOME_ANGLE']['standing'] == [1] * 11
    assert restored['RIGHT_ARM_MOTOR_HOME_ANGLE']['standing'] == [2] * 11
    assert restored['NECK_MOTOR_HOME_ANGLE']['zero'] == [3] * 7
    assert 'target_height_z = 0.0' in settings_path.read_text(encoding='utf-8')
    assert store.status()['mode'] == 'original'
    current_pose, current_height = store.current_pose()
    assert list(current_pose[name] for name in LEG_JOINTS) == [1, 1, 1, 1]
    assert current_height == 0.0
