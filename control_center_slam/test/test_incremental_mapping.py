import math
import numpy as np
import pytest
from incremental_mapping import resolve_state_file, relative_initial_pose, scan_map_support


def test_state_selection_keeps_source(tmp_path, monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr('incremental_mapping.subprocess.run', lambda *a, **k:
        SimpleNamespace(returncode=0, stdout='', stderr='#submap_2d_grid: 2'))
    state = tmp_path / '旧地图.pbstream'
    state.write_bytes(b'x' * 100)
    assert resolve_state_file(tmp_path / '旧地图.yaml') == state
    assert state.read_bytes() == b'x' * 100


def test_empty_submap_state_is_rejected_before_launch(tmp_path, monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr('incremental_mapping.subprocess.run', lambda *a, **k:
        SimpleNamespace(returncode=0, stdout='', stderr='#submap_2d_grid: 0'))
    state = tmp_path / '1.pbstream'; state.write_bytes(b'x'*100)
    with pytest.raises(ValueError, match='没有子地图'):
        resolve_state_file(state)
    assert state.read_bytes() == b'x'*100


def test_invalid_state_is_rejected(tmp_path, monkeypatch):
    from types import SimpleNamespace
    monkeypatch.setattr('incremental_mapping.subprocess.run', lambda *a, **k:
        SimpleNamespace(returncode=1, stdout='', stderr='parse failed'))
    state = tmp_path / 'bad.pbstream'; state.write_bytes(b'x'*100)
    with pytest.raises(ValueError, match='无法解析'):
        resolve_state_file(state)


@pytest.mark.parametrize('final', [False, True])
def test_save_keeps_active_submaps_in_both_modes(final):
    from types import SimpleNamespace
    from pathlib import Path
    from slam_ui_node import SlamPage
    requests = []
    client = SimpleNamespace(service_is_ready=lambda: True, call_async=lambda req:
        (requests.append(req) or SimpleNamespace(add_done_callback=lambda callback: None)))
    page = SimpleNamespace(node=SimpleNamespace(write_client=client), _save_final=final,
        _map_prefix=Path('/tmp/test_only_no_file_write'),
        save_status=SimpleNamespace(setText=lambda text: None), _write_complete=lambda f: None)
    SlamPage._write_state(page)
    assert requests[0].include_unfinished_submaps is True


def test_picture_only_is_rejected(tmp_path):
    with pytest.raises(ValueError):
        resolve_state_file(tmp_path / '只有图片.yaml')


def test_relative_pose_with_rotated_reference():
    assert relative_initial_pose((2, 5, math.pi), (2, 3, math.pi/2)) == pytest.approx((2, 0, math.pi/2))


def test_alignment_rejects_wrong_pose_and_empty_scan():
    grid = np.zeros((200, 200), np.int16)
    grid[:, 100] = 100
    info = dict(origin_x=0, origin_y=0, resolution=.02)
    points = np.column_stack([np.full(80, 2.), np.linspace(.2, 3.5, 80)])
    assert scan_map_support(grid, info, points, (0, 0, 0)) == 1
    assert scan_map_support(grid, info, points, (.5, 0, 0)) == 0
    assert scan_map_support(grid, info, [], (0, 0, 0)) == 0


def test_unknown_space_does_not_count_as_alignment():
    grid = np.full((200, 200), -1, np.int16)
    grid[:, 100] = 100
    points = np.column_stack([np.full(80, 2.05), np.linspace(.2, 3.5, 80)])
    assert scan_map_support(grid, dict(origin_x=0, origin_y=0, resolution=.02), points, (0, 0, 0)) == 0


def test_offscreen_ui_pose_selection_and_save_guards():
    import rclpy
    from PyQt5 import QtWidgets, QtCore, QtTest
    from nav_msgs.msg import OccupancyGrid
    from slam_ui_node import SlamPage
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rclpy.init()
    page = SlamPage(resume_state='/tmp/old.pbstream')
    page.spin_timer.stop(); page.velocity_timer.stop(); page.health_timer.stop()
    try:
        assert not page.final_button.isEnabled()
        assert page._trajectory_id is None
        grid = OccupancyGrid()
        grid.info.width = 100; grid.info.height = 100; grid.info.resolution = .1
        grid.info.origin.orientation.w = 1
        grid.data = [0] * 10000
        page.canvas.set_map(grid)
        page.resize(1200, 850); page.show(); app.processEvents()
        center = page.canvas.rect().center()
        QtTest.QTest.mousePress(page.canvas, QtCore.Qt.LeftButton, pos=center)
        QtTest.QTest.mouseRelease(page.canvas, QtCore.Qt.LeftButton, pos=center+QtCore.QPoint(60, 0))
        assert page.canvas.initial_pose is not None
        assert page.canvas.initial_pose[2] == pytest.approx(0)
        page.grab().save('/tmp/slam328_incremental_ui.png')
    finally:
        page.shutdown()
        page.close()
        rclpy.shutdown()
