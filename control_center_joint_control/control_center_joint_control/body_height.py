"""Verified coordinated waist/leg profile for vertical body-height changes."""

from .joint_definitions import clamp_joint


# These values mirror the vendor's ``move_z.py`` profile on the S2 robot.
# ``lowering_m`` is a positive distance down from the straight 0-degree pose.
MAX_BODY_LOWERING_M = 0.56
BODY_HEIGHT_JOINT_TARGETS_DEG = {
    'Joint_Ankle': 76.0,
    'Joint_Knee': 155.0,
    'Joint_Waist_Pitch': 81.0,
}
BODY_HEIGHT_JOINTS = tuple(BODY_HEIGHT_JOINT_TARGETS_DEG)


def body_height_targets(lowering_m):
    """Return the coordinated joint targets for a requested lowering distance."""
    lowering_m = min(MAX_BODY_LOWERING_M, max(0.0, float(lowering_m)))
    scale = lowering_m / MAX_BODY_LOWERING_M
    return {
        name: clamp_joint(name, maximum * scale)
        for name, maximum in BODY_HEIGHT_JOINT_TARGETS_DEG.items()
    }


def estimate_body_lowering(joints_deg):
    """Project three waist/leg angles onto the vendor height profile.

    Returns ``(lowering_m, residual_deg)``.  The residual is useful for showing
    that a manually edited waist/leg pose is only an approximate height pose.
    """
    values = {
        name: float(joints_deg[name])
        for name in BODY_HEIGHT_JOINTS
    }
    denominator = sum(
        maximum * maximum
        for maximum in BODY_HEIGHT_JOINT_TARGETS_DEG.values()
    )
    scale = sum(
        values[name] * BODY_HEIGHT_JOINT_TARGETS_DEG[name]
        for name in BODY_HEIGHT_JOINTS
    ) / denominator
    scale = min(1.0, max(0.0, scale))
    residual = max(
        abs(values[name] - BODY_HEIGHT_JOINT_TARGETS_DEG[name] * scale)
        for name in BODY_HEIGHT_JOINTS
    )
    return scale * MAX_BODY_LOWERING_M, residual


def matches_body_height_profile(joints_deg, tolerance_deg=2.0):
    """Return whether the three coordinated joints lie on the height curve."""
    _lowering_m, residual = estimate_body_lowering(joints_deg)
    return residual <= max(0.0, float(tolerance_deg))
