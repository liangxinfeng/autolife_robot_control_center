"""Resolve intentional group moves versus isolated live joint edits."""
import math
from .joint_definitions import clamp_joint, group_joint_names


def resolve_operator_targets(group, requested, current, single_joint=None):
    if not isinstance(requested, dict):
        raise ValueError('targets_deg must be an object')
    allowed = set(group_joint_names(group))
    if single_joint is None:
        if set(requested) != allowed:
            raise ValueError('joint set mismatch')
        return {name: clamp_joint(name, requested[name]) for name in allowed}
    if single_joint not in allowed or set(requested) != {single_joint}:
        raise ValueError('single_joint must match the only requested joint')
    held = {}
    for name in allowed:
        value = float(current[name])
        if not math.isfinite(value):
            raise ValueError('current joint feedback must be finite')
        # Never clamp an untouched joint to a UI limit: hold its measured pose.
        held[name] = value
    held[single_joint] = clamp_joint(single_joint, requested[single_joint])
    return held
