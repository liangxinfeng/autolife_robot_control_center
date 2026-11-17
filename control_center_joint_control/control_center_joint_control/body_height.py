"""Verified coordinated waist/leg profile for vertical body-height changes."""

import math

from .joint_definitions import JOINT_LIMITS


# Motor-coordinate ratios shared with the installed 345 V5 body_height.py.
# S3 model axes: ankle +Y, knee -Y, waist pitch +Y.
# ``lowering_m`` is a positive distance down from the straight 0-degree pose.
MAX_BODY_LOWERING_M = 0.56
MAX_REVERSE_BODY_LOWERING_M = 0.53
BODY_HEIGHT_JOINT_TARGETS_DEG = {
    'Joint_Ankle': 76.0,
    'Joint_Knee': 155.0,
    'Joint_Waist_Pitch': 81.0,
}
BODY_HEIGHT_JOINTS = tuple(BODY_HEIGHT_JOINT_TARGETS_DEG)


def body_height_targets(lowering_m, *, allow_reverse=False):
    """Return the coordinated joint targets for a requested lowering distance."""
    minimum = -MAX_REVERSE_BODY_LOWERING_M if allow_reverse else 0.0
    lowering_m = float(lowering_m)
    if not math.isfinite(lowering_m):
        raise ValueError('身体高度目标必须为有限数值')
    lowering_m = min(MAX_BODY_LOWERING_M, max(minimum, lowering_m))
    scale = lowering_m / MAX_BODY_LOWERING_M
    # Clip shared progress, not individual joints: retain ankle/knee/waist
    # geometry at the S3 knee limit (slightly below the nominal 155 degrees).
    low = max(JOINT_LIMITS[n][0] / v for n, v in BODY_HEIGHT_JOINT_TARGETS_DEG.items())
    high = min(JOINT_LIMITS[n][1] / v for n, v in BODY_HEIGHT_JOINT_TARGETS_DEG.items())
    scale = min(high, max(low, scale))
    return {
        name: maximum * scale
        for name, maximum in BODY_HEIGHT_JOINT_TARGETS_DEG.items()
    }


def estimate_body_lowering(joints_deg, *, signed=False):
    """Project three waist/leg angles onto the vendor height profile.

    Returns ``(lowering_m, residual_deg)``.  The residual is useful for showing
    that a manually edited waist/leg pose is only an approximate height pose.
    """
    values = {
        name: float(joints_deg[name])
        for name in BODY_HEIGHT_JOINTS
    }
    if not all(math.isfinite(v) for v in values.values()):
        raise ValueError('腰腿反馈必须为有限数值')
    denominator = sum(
        maximum * maximum
        for maximum in BODY_HEIGHT_JOINT_TARGETS_DEG.values()
    )
    scale = sum(
        values[name] * BODY_HEIGHT_JOINT_TARGETS_DEG[name]
        for name in BODY_HEIGHT_JOINTS
    ) / denominator
    minimum = -MAX_REVERSE_BODY_LOWERING_M / MAX_BODY_LOWERING_M if signed else 0.0
    scale = min(1.0, max(minimum, scale))
    residual = max(
        abs(values[name] - BODY_HEIGHT_JOINT_TARGETS_DEG[name] * scale)
        for name in BODY_HEIGHT_JOINTS
    )
    return scale * MAX_BODY_LOWERING_M, residual


def matches_body_height_profile(joints_deg, tolerance_deg=2.0):
    """Return whether the three coordinated joints lie on the height curve."""
    _lowering_m, residual = estimate_body_lowering(joints_deg, signed=True)
    return residual <= max(0.0, float(tolerance_deg))
