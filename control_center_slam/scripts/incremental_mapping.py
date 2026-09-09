"""Pure helpers for incremental mapping; never publish motion or modify a map."""
import math
import time
import re
import subprocess
from pathlib import Path
import numpy as np


def resolve_state_file(filename):
    path = Path(filename).expanduser().resolve()
    if path.suffix.lower() in ('.yaml', '.yml', '.pgm'):
        path = path.with_suffix('.pbstream')
    if path.suffix.lower() != '.pbstream' or not path.is_file() or path.stat().st_size < 16:
        raise ValueError('需要完整保存的 .pbstream 建图状态；仅有 YAML/PGM 无法续建。')
    validate_state_submaps(path)
    return path


def validate_state_submaps(path):
    """Use the shipped Cartographer reader: file size alone cannot prove a map exists."""
    try:
        result = subprocess.run(
            ['/opt/ros/jazzy/bin/cartographer_pbstream', 'info', str(path)],
            capture_output=True, text=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired) as error:
        raise ValueError('无法校验建图状态，请检查 Cartographer 读取工具或文件是否可读。') from error
    count = re.search(r'#submap_2d_grid:\s*(\d+)', result.stdout + result.stderr)
    if result.returncode != 0 or count is None:
        raise ValueError('建图状态无法解析或不完整；请重新保存完整 .pbstream 文件。')
    if int(count.group(1)) == 0:
        raise ValueError('该文件没有子地图栅格，无法增量建图。旧版最终保存可能漏存未封存子地图；'
                         'PGM/YAML 仍可用于导航，但不能代替完整建图状态。请使用完整快照或重新建图保存。')
    return int(count.group(1))


def relative_initial_pose(target, reference):
    """StartTrajectory's t=0 reference is the FIRST retained trajectory node."""
    x, y, yaw = target
    rx, ry, ryaw = reference
    c, s = math.cos(ryaw), math.sin(ryaw)
    return (c * (x-rx) + s * (y-ry), -s * (x-rx) + c * (y-ry),
            math.atan2(math.sin(yaw-ryaw), math.cos(yaw-ryaw)))


def scan_map_support(cells, info, base_points, pose, tolerance=.15):
    """Conservative seed validation, NOT proof of globally unique localization.

    Require many returns and high endpoint agreement. Ignore unavailable/unknown
    space; do not create evidence there. Uses an occupancy distance field.
    """
    from scipy.ndimage import distance_transform_edt
    points = np.asarray(base_points, dtype=float)
    if len(points) < 60 or not np.any(cells >= 65):
        return 0.0
    x, y, yaw = pose
    c, s = math.cos(yaw), math.sin(yaw)
    world = points @ np.array([[c, s], [-s, c]]) + [x, y]
    indices = np.floor((world - [info['origin_x'], info['origin_y']]) /
                       info['resolution']).astype(int)
    inside = ((indices[:, 0] >= 0) & (indices[:, 1] >= 0) &
              (indices[:, 0] < cells.shape[1]) & (indices[:, 1] < cells.shape[0]))
    if not inside.any():
        return 0.0
    padding = int(math.ceil(tolerance/info['resolution']))+1
    low = np.maximum(indices[inside].min(axis=0)-padding, 0)
    high = np.minimum(indices[inside].max(axis=0)+padding+1, [cells.shape[1], cells.shape[0]])
    local = cells[low[1]:high[1], low[0]:high[0]]
    if not np.any(local >= 65):
        return 0.0
    distance = distance_transform_edt(local < 65) * info['resolution']
    matched = np.zeros(len(points), dtype=bool)
    ix, iy = indices[inside, 0], indices[inside, 1]
    matched[inside] = (distance[iy-low[1], ix-low[0]] <= tolerance) & (cells[iy, ix] >= 0)
    return float(matched.mean())


def refine_initial_pose(cells, info, base_points, seed, radius=1.5,
                        angle_range=math.radians(45), cancelled=lambda: False):
    """Bounded coarse-to-fine correlative search; no ROS writes or motion.

    Preserve separate hypotheses to reject locally ambiguous corridors/repeated
    structures. Search tolerance is NOT a relaxed final acceptance threshold.
    """
    from scipy.ndimage import distance_transform_edt
    started = time.monotonic()
    seed = np.asarray(seed, dtype=float)
    points = np.asarray(base_points, dtype=float)
    resolution = float(info['resolution'])
    if (points.ndim != 2 or points.shape[1] != 2 or len(points) < 60 or
            not np.isfinite(points).all() or not np.isfinite(seed).all()):
        raise ValueError('有效激光点不足，请等待前后雷达数据。')
    if not .005 <= resolution <= .2 or abs(info.get('origin_yaw', 0)) > 1e-5:
        raise ValueError('地图分辨率或原点不支持本次局部匹配。')
    if not 0 < radius <= 3 or not 0 < angle_range <= math.pi/2:
        raise ValueError('局部搜索范围无效。')
    points = points[np.linalg.norm(points, axis=1) <= 8.1]
    if len(points) < 60:
        raise ValueError('近距离有效激光不足，未进行自动对齐。')
    if len(points) > 300:
        points = points[np.linspace(0, len(points)-1, 300, dtype=int)]
    # Crop work to the reachable scan envelope; never distance-transform an
    # entire large building just to resolve one seed.
    origin = np.array([info['origin_x'], info['origin_y']])
    extent = np.linalg.norm(points, axis=1).max() + radius + .5
    lo = np.maximum(np.floor((seed[:2]-extent-origin)/resolution).astype(int), 0)
    hi = np.minimum(np.ceil((seed[:2]+extent-origin)/resolution).astype(int)+1,
                    [cells.shape[1], cells.shape[0]])
    if np.any(hi <= lo) or np.prod(hi-lo) > 6000000:
        raise ValueError('光标不在有效地图内，或局部地图过大。')
    grid = cells[lo[1]:hi[1], lo[0]:hi[0]]
    origin = origin + lo*resolution
    if not np.any(grid >= 65):
        raise ValueError('光标附近没有足够已建图的墙体，请重新选大致位置。')
    distances = np.minimum(distance_transform_edt(grid < 65)*resolution, .6).astype(np.float32)
    footprint = np.array([[0, 0], [.25, .25], [.25, -.25], [-.25, .25], [-.25, -.25]])

    def check():
        if cancelled():
            raise ValueError('定位搜索已取消。')
        if time.monotonic()-started > 12:
            raise ValueError('局部搜索超时，请将光标再靠近实际位置后重试。')

    def evaluate(poses):
        output = []
        for offset in range(0, len(poses), 64):
            check()
            batch = poses[offset:offset+64]
            c, s = np.cos(batch[:, 2:3]), np.sin(batch[:, 2:3])
            wx = c*points[:, 0]-s*points[:, 1]+batch[:, 0:1]
            wy = s*points[:, 0]+c*points[:, 1]+batch[:, 1:2]
            ix = np.floor((wx-origin[0])/resolution).astype(int)
            iy = np.floor((wy-origin[1])/resolution).astype(int)
            inside = (ix >= 0) & (iy >= 0) & (ix < grid.shape[1]) & (iy < grid.shape[0])
            ix = np.clip(ix, 0, grid.shape[1]-1); iy = np.clip(iy, 0, grid.shape[0]-1)
            valid = inside & (grid[iy, ix] >= 0)
            distance = distances[iy, ix]
            score = (np.exp(-.5*(distance/.12)**2)*valid).mean(axis=1)
            support = ((distance <= .15) & valid).mean(axis=1)
            # A high wall endpoint score must not place the base in a wall or
            # unknown space. Rotate the conservative 50 cm square with yaw.
            bx = c*footprint[:, 0]-s*footprint[:, 1]+batch[:, 0:1]
            by = s*footprint[:, 0]+c*footprint[:, 1]+batch[:, 1:2]
            bx = np.floor((bx-origin[0])/resolution).astype(int)
            by = np.floor((by-origin[1])/resolution).astype(int)
            inside_base = (bx >= 0) & (by >= 0) & (bx < grid.shape[1]) & (by < grid.shape[0])
            base_cells = grid[np.clip(by, 0, grid.shape[0]-1), np.clip(bx, 0, grid.shape[1]-1)]
            valid_base = (inside_base & (base_cells >= 0) & (base_cells <= 25)).all(axis=1)
            output.append(np.column_stack((np.where(valid_base, score, -1), support)))
        return np.concatenate(output)

    def candidates(center, xy_offsets, yaw_offsets):
        offsets = np.stack(np.meshgrid(xy_offsets, xy_offsets, yaw_offsets, indexing='ij'), axis=-1).reshape(-1, 3)
        poses = center + offsets
        within = (np.abs(poses[:, :2]-seed[:2]) <= radius+1e-8).all(axis=1)
        within &= np.abs(poses[:, 2]-seed[2]) <= angle_range+1e-8
        return poses[within]

    poses = candidates(seed, np.linspace(-radius, radius, 21),
                       np.linspace(-angle_range, angle_range, 31))
    values = evaluate(poses)
    selected = []
    for i in np.argsort(values[:, 0])[::-1]:
        p = poses[i]
        if all(np.linalg.norm(p[:2]-q[:2]) > .30 or abs(p[2]-q[2]) > math.radians(8) for q in selected):
            selected.append(p)
        if len(selected) == 8:
            break
    refined = []
    for center in selected:
        local = candidates(center, np.linspace(-.15, .15, 11), np.deg2rad(np.arange(-3, 4)))
        scores = evaluate(local)
        best = int(np.argmax(scores[:, 0]))
        refined.append((local[best], scores[best]))
    refined.sort(key=lambda pair: pair[1][0], reverse=True)
    pose, quality = refined[0]
    if quality[0] < .55 or quality[1] < .60:
        raise ValueError(f'附近搜索仍未可靠匹配（端点吻合{quality[1]:.0%}），请重选大致区域或方向。')
    for other, score in refined[1:]:
        if np.linalg.norm(other[:2]-pose[:2]) > .35 or abs(other[2]-pose[2]) > math.radians(8):
            if quality[0]-score[0] < .035:
                raise ValueError('附近存在多个相似位置，暂不开始续建。请把光标靠近实际位置，或换到更有特征的位置。')
    if (np.max(np.abs(pose[:2]-seed[:2])) > radius-.04 or
            abs(pose[2]-seed[2]) > angle_range-math.radians(1)):
        raise ValueError('最佳匹配位于搜索边缘，请将光标更靠近实际位置或方向后重试。')
    return dict(pose=tuple(pose), support=float(quality[1]), score=float(quality[0]),
                elapsed=time.monotonic()-started)
