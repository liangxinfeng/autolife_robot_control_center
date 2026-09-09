"""Regression tests for noise-resistant progressive joint commands."""

from control_center_joint_control.joint_bridge import JointBridge


def test_positive_command_never_reverses_with_noisy_feedback():
    command = 0.0
    commands = []
    for current in (0.0, 2.0, 4.5, 4.1, 6.0, 5.7, 8.0):
        command = JointBridge._stable_progressive_target(
            current=current,
            previous=command,
            trajectory_target=30.0,
            final_target=30.0,
            start=0.0,
            lead=5.0,
        )
        commands.append(command)
    assert commands == sorted(commands)
    assert all(command <= 30.0 for command in commands)


def test_negative_command_never_reverses_with_noisy_feedback():
    command = 10.0
    commands = []
    for current in (10.0, 8.0, 5.5, 5.9, 3.0, 3.4, 0.0):
        command = JointBridge._stable_progressive_target(
            current=current,
            previous=command,
            trajectory_target=-20.0,
            final_target=-20.0,
            start=10.0,
            lead=5.0,
        )
        commands.append(command)
    assert commands == sorted(commands, reverse=True)
    assert all(command >= -20.0 for command in commands)


def test_body_height_joints_share_one_progress_value_when_lowering():
    commands, progress = JointBridge._coordinated_progressive_targets(
        joint_names=('Joint_Ankle', 'Joint_Knee', 'Joint_Waist_Pitch'),
        current={
            'Joint_Ankle': 0.0, 'Joint_Knee': 0.0,
            'Joint_Waist_Pitch': 0.0,
        },
        previous={
            'Joint_Ankle': 0.0, 'Joint_Knee': 0.0,
            'Joint_Waist_Pitch': 0.0,
        },
        start={
            'Joint_Ankle': 0.0, 'Joint_Knee': 0.0,
            'Joint_Waist_Pitch': 0.0,
        },
        final={
            'Joint_Ankle': 76.0, 'Joint_Knee': 155.0,
            'Joint_Waist_Pitch': 81.0,
        },
        previous_progress=0.0,
        lead=5.0,
    )
    assert progress == 5.0 / 155.0
    assert commands['Joint_Knee'] == 5.0
    assert commands['Joint_Ankle'] == 76.0 * progress
    assert commands['Joint_Waist_Pitch'] == 81.0 * progress
    assert commands['Joint_Ankle'] > 0.0


def test_body_height_joints_share_one_progress_value_when_rising():
    commands, progress = JointBridge._coordinated_progressive_targets(
        joint_names=('Joint_Ankle', 'Joint_Knee', 'Joint_Waist_Pitch'),
        current={
            'Joint_Ankle': 76.0, 'Joint_Knee': 155.0,
            'Joint_Waist_Pitch': 81.0,
        },
        previous={
            'Joint_Ankle': 76.0, 'Joint_Knee': 155.0,
            'Joint_Waist_Pitch': 81.0,
        },
        start={
            'Joint_Ankle': 76.0, 'Joint_Knee': 155.0,
            'Joint_Waist_Pitch': 81.0,
        },
        final={
            'Joint_Ankle': 0.0, 'Joint_Knee': 0.0,
            'Joint_Waist_Pitch': 0.0,
        },
        previous_progress=0.0,
        lead=5.0,
    )
    assert progress == 5.0 / 155.0
    assert commands['Joint_Knee'] == 150.0
    assert commands['Joint_Ankle'] == 76.0 * (1.0 - progress)
    assert commands['Joint_Waist_Pitch'] == 81.0 * (1.0 - progress)
    assert commands['Joint_Ankle'] < 76.0


def test_slow_joint_holds_shared_progress_without_reversing_commands():
    start = {
        'Joint_Ankle': 0.0, 'Joint_Knee': 0.0,
        'Joint_Waist_Pitch': 0.0,
    }
    final = {
        'Joint_Ankle': 76.0, 'Joint_Knee': 155.0,
        'Joint_Waist_Pitch': 81.0,
    }
    previous_progress = 20.0 / 155.0
    previous = {
        'Joint_Ankle': 76.0 * previous_progress,
        'Joint_Knee': 20.0,
        'Joint_Waist_Pitch': 81.0 * previous_progress,
    }
    commands, progress = JointBridge._coordinated_progressive_targets(
        joint_names=('Joint_Ankle', 'Joint_Knee', 'Joint_Waist_Pitch'),
        current={
            'Joint_Ankle': 10.0, 'Joint_Knee': 5.0,
            'Joint_Waist_Pitch': 10.5,
        },
        previous=previous,
        start=start,
        final=final,
        previous_progress=previous_progress,
        lead=5.0,
    )
    assert progress == previous_progress
    assert commands['Joint_Ankle'] >= 76.0 * previous_progress
    assert commands['Joint_Knee'] >= previous['Joint_Knee']
    assert commands['Joint_Waist_Pitch'] >= previous['Joint_Waist_Pitch']
