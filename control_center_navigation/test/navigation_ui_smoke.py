#!/usr/bin/env python3
"""Offscreen render and map-coordinate regression test."""

import importlib.util
import math
from pathlib import Path
import sys
import tempfile

import numpy as np
from geometry_msgs.msg import Pose
from nav_msgs.msg import OccupancyGrid
from PyQt5 import QtCore, QtGui, QtWidgets


SOURCE = (
    Path(__file__).resolve().parents[1]
    / 'scripts'
    / 'navigation_ui_node.py'
)
spec = importlib.util.spec_from_file_location('navigation_ui_node', SOURCE)
module = importlib.util.module_from_spec(spec)
sys.modules[spec.name] = module
spec.loader.exec_module(module)


def make_map():
    message = OccupancyGrid()
    message.info.width = 120
    message.info.height = 90
    message.info.resolution = 0.05
    message.info.origin = Pose()
    message.info.origin.position.x = -3.0
    message.info.origin.position.y = -2.25
    message.info.origin.orientation.w = 1.0
    data = [0] * (120 * 90)
    for y in range(90):
        for x in range(120):
            if x in (3, 116) or y in (3, 86):
                data[y * 120 + x] = 100
            elif 42 <= x <= 45 and 15 <= y <= 70:
                data[y * 120 + x] = 100
            elif 72 <= x <= 75 and 30 <= y <= 84:
                data[y * 120 + x] = 100
    message.data = data
    return message


def main():
    application = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    canvas = module.MapCanvas()
    canvas.resize(900, 650)
    canvas.set_map(make_map())
    original_grid = canvas.grid.copy()
    canvas.set_mode('draw_obstacle')
    canvas.set_brush_size_cm(5)
    canvas._undo_stack.append(canvas.grid.copy())
    assert canvas._paint_grid_cell(50, 20)
    canvas._render_grid()
    assert canvas.grid[50, 20] == 100
    canvas.undo_edit()
    assert np.array_equal(canvas.grid, original_grid)
    canvas.redo_edit()
    assert canvas.grid[50, 20] == 100
    canvas.set_mode('erase_obstacle')
    assert canvas._paint_grid_cell(50, 20)
    assert canvas.grid[50, 20] == 0
    print('PASS: obstacle brush, eraser, undo and redo')

    with tempfile.TemporaryDirectory() as temporary_directory:
        success, _message, saved_path = module.save_map_files(
            temporary_directory,
            '编辑地图_01',
            canvas.grid,
            canvas.map_meta,
        )
        assert success
        yaml_path = Path(saved_path)
        pgm_path = yaml_path.with_suffix('.pgm')
        assert yaml_path.is_file()
        assert pgm_path.is_file()
        yaml_before = yaml_path.read_bytes()
        pgm_before = pgm_path.read_bytes()
        duplicate_success, duplicate_message, _ = module.save_map_files(
            temporary_directory,
            '编辑地图_01',
            canvas.grid,
            canvas.map_meta,
        )
        assert not duplicate_success
        assert '禁止覆盖' in duplicate_message
        assert yaml_path.read_bytes() == yaml_before
        assert pgm_path.read_bytes() == pgm_before
    print('PASS: edited map saves as new files and never overwrites')

    fixed_map_snapshot = canvas.display_snapshot()
    fixed_grid = canvas.grid.copy()
    canvas.clear_map()
    assert canvas.image is None
    assert canvas.grid is None
    assert canvas.map_meta is None
    canvas.restore_display_snapshot(fixed_map_snapshot)
    assert np.array_equal(canvas.grid, fixed_grid)
    assert canvas.image is not None
    assert canvas.map_meta is not None
    print('PASS: fixed navigation map is restored after SLAM display cleanup')

    canvas.drag_start = QtCore.QPointF(12.0, 12.0)
    canvas.drag_world = (0.0, 0.0)
    canvas.restore_display_snapshot(fixed_map_snapshot)
    assert canvas.drag_start is None
    assert canvas.drag_world is None
    canvas.set_mode('initial')
    initial_poses = []
    canvas.initial_pose_requested.connect(
        lambda x, y, yaw: initial_poses.append((x, y, yaw))
    )
    start = QtCore.QPointF(canvas.width() * 0.48, canvas.height() * 0.52)
    end = start + QtCore.QPointF(55.0, -18.0)
    canvas.mousePressEvent(QtGui.QMouseEvent(
        QtCore.QEvent.MouseButtonPress,
        start,
        QtCore.Qt.LeftButton,
        QtCore.Qt.LeftButton,
        QtCore.Qt.NoModifier,
    ))
    canvas.mouseMoveEvent(QtGui.QMouseEvent(
        QtCore.QEvent.MouseMove,
        end,
        QtCore.Qt.NoButton,
        QtCore.Qt.LeftButton,
        QtCore.Qt.NoModifier,
    ))
    canvas.mouseReleaseEvent(QtGui.QMouseEvent(
        QtCore.QEvent.MouseButtonRelease,
        end,
        QtCore.Qt.LeftButton,
        QtCore.Qt.NoButton,
        QtCore.Qt.NoModifier,
    ))
    assert len(initial_poses) == 1
    print('PASS: initial-pose drag works after map restoration')

    canvas.robot = {
        'x': 0.7, 'y': 0.4, 'yaw': math.radians(28),
        'cov_x': 0.03, 'cov_y': 0.02,
    }
    canvas.goal = {'x': 1.8, 'y': 1.2, 'yaw': math.radians(-20)}
    canvas.global_path = [
        (-1.8, -1.2), (-0.5, -0.4), (0.7, 0.4), (1.8, 1.2)
    ]
    canvas.local_path = [(0.7, 0.4), (1.1, 0.65), (1.45, 0.9)]
    canvas.waypoints = [
        {'name': '充电桩', 'x': -1.8, 'y': -1.2, 'yaw': 0.0},
        {'name': '吧台', 'x': 1.7, 'y': 1.45, 'yaw': math.radians(90)},
        {'name': '客房', 'x': 2.1, 'y': -1.25, 'yaw': math.radians(-135)},
    ]
    canvas.laser_points = [
        (
            canvas.robot['x'] + math.cos(i * 0.12) * 1.1,
            canvas.robot['y'] + math.sin(i * 0.12) * 1.1,
        )
        for i in range(52)
    ]
    canvas.show()
    application.processEvents()

    for world in ((-2.0, -1.0), (0.0, 0.0), (2.2, 1.7)):
        screen = canvas.world_to_screen(*world)
        recovered = canvas.screen_to_world(screen)
        assert abs(recovered[0] - world[0]) < 1e-6
        assert abs(recovered[1] - world[1]) < 1e-6

    output = Path('/tmp/navigation_native_ui_map_smoke.png')
    assert canvas.grab().save(str(output))
    print(f'PASS: map transform and offscreen rendering: {output}')

    dialog = module.WaypointSequenceDialog(
        ['a', 'b', 'c', 'd'], ['a', 'b', 'a', 'c', 'd']
    )
    dwell_times = [1.0, 2.5, 4.0, 0.0, 8.0]
    for index, dwell_time in enumerate(dwell_times):
        dialog.route_list.item(index).setData(
            QtCore.Qt.UserRole + 1, dwell_time
        )
    dialog._renumber()
    assert dialog.sequence_names() == ['a', 'b', 'a', 'c', 'd']
    assert dialog.sequence_dwell_times() == dwell_times
    dialog.close()
    print('PASS: ordered waypoint route supports duplicates and dwell times')

    canvas.close()
    application.processEvents()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
