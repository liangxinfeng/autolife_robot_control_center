import math
import numpy as np
import pytest
from incremental_mapping import refine_initial_pose


def scene():
    grid = np.zeros((220, 260), dtype=np.int16)
    grid[:2, :] = 100; grid[-2:, :] = 100
    grid[:, :2] = 100; grid[:, -2:] = 100
    grid[40:82, 145:149] = 100
    grid[80:84, 148:209] = 100
    grid[138:159, 39:65] = 100
    grid[30:43, 30:41] = 100
    info = dict(origin_x=0., origin_y=0., origin_yaw=0., resolution=.05)
    pose = np.array([4.6, 4.8, .22])
    angles = np.linspace(-math.pi, math.pi, 300, endpoint=False)
    ranges = np.arange(.35, 8.01, .025)
    x = pose[0] + np.cos(angles[:, None]+pose[2])*ranges
    y = pose[1] + np.sin(angles[:, None]+pose[2])*ranges
    gx = np.clip((x/.05).astype(int), 0, grid.shape[1]-1)
    gy = np.clip((y/.05).astype(int), 0, grid.shape[0]-1)
    hits = grid[gy, gx] >= 65
    present = hits.any(axis=1)
    measured = ranges[hits.argmax(axis=1)][present]
    return grid, info, np.column_stack((np.cos(angles[present])*measured,
                                       np.sin(angles[present])*measured)), pose


def test_recovers_rough_position_and_direction():
    grid, info, points, pose = scene()
    result = refine_initial_pose(grid, info, points, pose+[.85, -.65, math.radians(24)])
    assert np.linalg.norm(np.array(result['pose'][:2])-pose[:2]) < .1
    assert abs(result['pose'][2]-pose[2]) < math.radians(2)
    assert result['support'] > .85
    print(result)


def test_already_close_pose_stays_close():
    grid, info, points, pose = scene()
    result = refine_initial_pose(grid, info, points, pose+[.05, -.03, .01])
    assert np.linalg.norm(np.array(result['pose'][:2])-pose[:2]) < .1


def test_corridor_ambiguity_is_rejected():
    grid = np.zeros((500, 200), dtype=np.int16)
    grid[:, 40] = 100; grid[:, 160] = 100
    points = np.column_stack((np.tile([-3., 3.], 70), np.repeat(np.linspace(-3., 3., 70), 2)))
    with pytest.raises(ValueError, match='相似'):
        refine_initial_pose(grid, dict(origin_x=0, origin_y=0, resolution=.05), points, (5, 10, 0))


def test_search_can_be_cancelled():
    grid, info, points, pose = scene()
    with pytest.raises(ValueError, match='取消'):
        refine_initial_pose(grid, info, points, pose, cancelled=lambda: True)


def test_wrong_area_is_not_accepted():
    grid, info, points, pose = scene()
    with pytest.raises(ValueError):
        refine_initial_pose(grid, info, points, pose+[5., 0., 0.])


def test_background_result_requires_confirmation(monkeypatch):
    import time
    from types import SimpleNamespace
    import rclpy
    from PyQt5 import QtWidgets
    from geometry_msgs.msg import Pose
    from slam_ui_node import SlamPage
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    rclpy.init()
    page = SlamPage(resume_state='/tmp/old.pbstream')
    page.spin_timer.stop(); page.velocity_timer.stop(); page.health_timer.stop()
    grid, info, points, pose = scene()
    page.canvas.map_cells = grid
    page.canvas.map_info = dict(info, width=grid.shape[1], height=grid.shape[0])
    page.canvas.initial_pose = tuple(pose+[.8, -.6, .3])
    feedback = Pose(); feedback.orientation.w = 1
    page.node.alignment_odometry.append((time.monotonic(), feedback))
    page.node.alignment_points = lambda: points
    def forbidden(*args):
        raise AssertionError('Rejecting the match must not start a trajectory')
    service = SimpleNamespace(service_is_ready=lambda: True, call_async=forbidden)
    page.node.states_client = service; page.node.start_client = service; page.node.query_client = service
    monkeypatch.setattr(QtWidgets.QMessageBox, 'question', lambda *a: QtWidgets.QMessageBox.No)
    try:
        page._begin_resume()
        assert page._resume_busy
        deadline = time.monotonic()+5
        while page._resume_busy and time.monotonic() < deadline:
            app.processEvents(); time.sleep(.005)
        assert not page._resume_busy
        assert page._trajectory_id is None
        assert page.canvas.select_initial_pose
        assert '保留修正光标' in page.resume_status.text()
        assert np.linalg.norm(np.asarray(page.canvas.initial_pose[:2])-pose[:2]) < .1
    finally:
        page.shutdown(); page.close(); app.processEvents(); rclpy.shutdown()
