"""Tests for the vendor-compatible coordinated body-height profile."""

import pytest

from control_center_joint_control.body_height import (
    MAX_BODY_LOWERING_M,
    body_height_targets,
    estimate_body_lowering,
    matches_body_height_profile,
)


def test_straight_and_lowest_endpoints():
    assert body_height_targets(0.0) == {
        'Joint_Ankle': 0.0,
        'Joint_Knee': 0.0,
        'Joint_Waist_Pitch': 0.0,
    }
    from control_center_joint_control.joint_definitions import JOINT_LIMITS
    scale=min(1., JOINT_LIMITS['Joint_Knee'][1]/155.)
    assert body_height_targets(MAX_BODY_LOWERING_M) == pytest.approx({
        'Joint_Ankle': 76.0*scale,
        'Joint_Knee': 155.0*scale,
        'Joint_Waist_Pitch': 81.0*scale,
    })


def test_profile_round_trip_and_vertical_upper_body():
    targets = body_height_targets(0.28)
    assert targets == {
        'Joint_Ankle': 38.0,
        'Joint_Knee': 77.5,
        'Joint_Waist_Pitch': 40.5,
    }
    # URDF axes are ankle -, knee + and waist -.  The vendor profile keeps
    # the upper body within one degree of vertical at half travel.
    assert -targets['Joint_Ankle'] + targets['Joint_Knee'] \
        - targets['Joint_Waist_Pitch'] == pytest.approx(-1.0)
    lowering, residual = estimate_body_lowering(targets)
    assert lowering == pytest.approx(0.28)
    assert residual == pytest.approx(0.0)


def test_requested_height_is_safely_clamped():
    assert body_height_targets(-1.0) == body_height_targets(0.0)
    assert body_height_targets(2.0) == body_height_targets(
        MAX_BODY_LOWERING_M
    )


def test_custom_pose_reports_profile_residual():
    lowering, residual = estimate_body_lowering({
        'Joint_Ankle': 20.0,
        'Joint_Knee': 20.0,
        'Joint_Waist_Pitch': 20.0,
    })
    assert 0.0 < lowering < MAX_BODY_LOWERING_M
    assert residual > 5.0
    assert not matches_body_height_profile({
        'Joint_Ankle': 20.0,
        'Joint_Knee': 20.0,
        'Joint_Waist_Pitch': 20.0,
    })


def test_default_pose_is_recognized_for_synchronized_return():
    default_pose = {
        'Joint_Ankle': 0.002950873256714876,
        'Joint_Knee': -0.000635572701446281,
        'Joint_Waist_Pitch': -0.002179106404958678,
    }
    assert matches_body_height_profile(default_pose)
