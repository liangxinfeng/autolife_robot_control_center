"""Conservative near-tail suppression backed by another scanner's wall returns.

No ROS, map matching, replacement ranges, or temporal persistence is used here.
The caller must provide a fresh, time-correct transform into the other scan.
"""
import math
import numpy as np


def cross_checked_tail_indices(ranges, intensities, angle_min, increment,
                               other_ranges, other_intensities,
                               other_angle_min, other_increment, rotation, translation):
    r = np.asarray(ranges, dtype=float)
    strength = np.asarray(intensities, dtype=float)
    other = np.asarray(other_ranges, dtype=float)
    other_strength = np.asarray(other_intensities, dtype=float)
    if (not np.isfinite([angle_min, increment, other_angle_min, other_increment]).all() or
            len(r) < 80 or strength.shape != r.shape or len(other) < 80 or
            other_strength.shape != other.shape or not .005 < abs(increment) < .012 or
            not .005 < abs(other_increment) < .012):
        return []
    rotation = np.asarray(rotation, dtype=float)
    translation = np.asarray(translation, dtype=float)
    if (rotation.shape != (3, 3) or translation.shape != (3,) or
            not np.isfinite(rotation).all() or not np.isfinite(translation).all()):
        return []
    valid = np.isfinite(r) & (r >= .04) & (r <= 16)
    weak = valid & np.isfinite(strength) & (strength >= 0) & (strength <= 12)
    angles = angle_min + np.arange(len(r)) * increment
    with np.errstate(invalid='ignore'):
        xy = np.column_stack((r * np.cos(angles), r * np.sin(angles)))
        oa = other_angle_min + np.arange(len(other)) * other_increment
        other_xy = np.column_stack((other * np.cos(oa), other * np.sin(oa)))
    certificates = {}

    def corroborated(i):
        if i in certificates:
            return certificates[i]
        certificates[i] = False
        point = rotation @ np.r_[xy[i], 0.] + translation
        distance = np.linalg.norm(point[:2])
        if abs(point[2]) > .025 or not .20 < distance < 1.5:
            return False
        beam = int(round((math.atan2(point[1], point[0]) - other_angle_min) /
                         other_increment))
        if beam < 43 or beam + 43 >= len(other):
            return False
        support = np.arange(beam - 2, beam + 3)
        # Actual returns on five neighbouring rays must all be behind this point.
        # Infinity/NaN is not evidence of free space.
        if (not np.isfinite(other[support]).all() or
                not np.isfinite(other_strength[support]).all() or
                np.any(other[support] < distance + .06) or
                np.any(other[support] > 1.5) or np.any(other_strength[support] < 0)):
            return False
        for first in range(beam - 40, beam + 10, 2):
            ids = np.arange(first, first + 32)
            if (not np.isfinite(other[ids]).all() or np.any(other[ids] < .20) or
                    np.any(other[ids] > 1.5) or
                    not np.isfinite(other_strength[ids]).all() or
                    np.count_nonzero(other_strength[ids] >= 14) < 28):
                continue
            anchor = other_xy[ids]
            steps = np.linalg.norm(np.diff(anchor, axis=0), axis=1)
            span = np.linalg.norm(anchor[-1] - anchor[0])
            if np.max(steps) > .02 or not .10 <= span <= .35:
                continue
            centre = anchor.mean(axis=0)
            _, vectors = np.linalg.eigh((anchor - centre).T @ (anchor - centre))
            normal, tangent = vectors[:, 0], vectors[:, 1]
            residual = (anchor - centre) @ normal
            if np.sqrt(np.mean(residual ** 2)) > .003 or np.max(np.abs(residual)) > .008:
                continue
            # Corroborating rays must belong to this retained straight wall,
            # including at most 6 cm of extrapolation along the wall.
            projected = (other_xy[support] - centre) @ tangent
            extents = (anchor - centre) @ tangent
            if (np.max(np.abs((other_xy[support] - centre) @ normal)) > .020 or
                    projected.min() < extents.min() - .06 or
                    projected.max() > extents.max() + .06):
                continue
            signed_distance = (point[:2] - centre) @ normal
            scanner_distance = -centre @ normal
            if (signed_distance * scanner_distance <= 0 or
                    not .04 <= abs(signed_distance) <= .15):
                continue
            certificates[i] = True
            return True
        return False

    confirmed = []
    # Restrict this extra pass to the close weak fragments excluded by the
    # original 40 cm protection. Every removed point needs independent positive
    # ray evidence; a weak neighbour, missing ray, or map alone is insufficient.
    for i in np.flatnonzero(weak & (r >= .30) & (r <= .45)):
        lo, hi = max(3, i-24), min(len(r)-3, i+25)
        if not any(not np.any(valid[j:j+3]) for j in range(lo, hi)):
            continue
        neighbours = np.arange(max(0,i-32), min(len(r),i+33))
        strong = neighbours[valid[neighbours] & (strength[neighbours] >= 14)]
        if not len(strong) or np.min(np.linalg.norm(xy[strong]-xy[i],axis=1)) > .15:
            continue
        if corroborated(i):
            confirmed.append(int(i))
    return [i for i in confirmed if any(i != j and abs(i-j)<=8 and np.linalg.norm(xy[i]-xy[j])<=.05 for j in confirmed)]
