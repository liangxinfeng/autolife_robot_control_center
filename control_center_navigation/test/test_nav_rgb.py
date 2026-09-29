"""Offline UI/keyboard regression. Fake chassis: never creates a ROS node."""
import importlib.util
import os
from pathlib import Path
import sys
import time
from types import SimpleNamespace

import numpy as np
from nav_msgs.msg import OccupancyGrid
from PyQt5 import QtCore, QtGui, QtWidgets, QtTest

ROOT = Path(__file__).resolve().parent
SCRIPTS = ROOT if (ROOT / 'navigation_ui_node.py').exists() else ROOT.parent / 'scripts'
SCRIPTS = Path(os.environ.get('NAV_RGB_SCRIPT_DIR', str(SCRIPTS)))
spec = importlib.util.spec_from_file_location('rgb_nav_test_ui', SCRIPTS / 'navigation_ui_node.py')
nav = importlib.util.module_from_spec(spec)
spec.loader.exec_module(nav)


class FakeNode:
    def __init__(self):
        self.signals = nav.RosSignals()
        self._loaded_map_path = ''
        self.commands = []
        self.map_age = self.pose_age = 0.0
        self.continuity_status = {}
        self._chassis_lease = SimpleNamespace(held=False, last_error='')

    def map_catalog(self):
        return []

    def refresh_waypoints(self):
        pass

    def cancel_direct_goal(self):
        self.commands.append('cancel')

    def set_velocity(self, *values):
        self.commands.append(values)

    def stop_immediate(self):
        self.commands.append('stop_now')

    def stop(self):
        self.commands.append('stop')


class FakeSource:
    def __init__(self):
        self.enabled = False
        self.latest = None
        self.closed = False

    def set_enabled(self, enabled):
        self.enabled = enabled

    def snapshot(self):
        return self.latest, ''

    def close(self):
        self.closed = True


app = QtWidgets.QApplication([])
host = QtWidgets.QMainWindow()
node = FakeNode()
win = nav.NavigationWindow(node)
win.setWindowFlags(QtCore.Qt.Widget)
win.setMinimumSize(0, 0)
host.setCentralWidget(win)
host.resize(1500, 900)
source = FakeSource()
win.rgb_panel.source.close()
win.rgb_panel.source = source
message = OccupancyGrid()
message.info.width, message.info.height, message.info.resolution = 150, 100, .05
message.info.origin.orientation.w = 1.0
grid = np.zeros((100, 150), dtype=np.int8)
grid[:3] = grid[-3:] = 100
grid[:, :3] = grid[:, -3:] = 100
grid[30:60, 60:65] = 100
message.data = grid.ravel().tolist()
win._map_received(message)
host.show()
app.processEvents()
assert not source.enabled and not win.rgb_panel.isVisible()
assert win.rgb_toggle.x() < win.mode_buttons['browse'].x()
baseline_width = win.map_canvas.width()
win.map_canvas.zoom = 1.4
original_grid = win.map_canvas.grid.copy()
QtTest.QTest.mouseClick(win.rgb_toggle, QtCore.Qt.LeftButton)
app.processEvents()
assert source.enabled and win.map_canvas.isVisible() and win.rgb_panel.isVisible()
assert win.map_canvas.width() >= 300 and win.rgb_panel.width() >= 300
frame = np.zeros((720, 1280, 3), dtype=np.uint8)
frame[:, :640] = (225, 95, 55)
frame[:, 640:] = (35, 160, 225)
source.latest = (frame, time.monotonic(), 1)
win.rgb_panel.refresh_frame()
app.processEvents()
host.grab().save(str(ROOT / 'rgb_split.png'))
QtTest.QTest.mouseClick(win.rgb_panel.enlarge_button, QtCore.Qt.LeftButton)
app.processEvents()
assert win._rgb_fullscreen and not win.map_canvas.isVisible()
assert win.rgb_panel.viewport.hasFocus()
assert win.rgb_panel.width() >= baseline_width - 10

# Send events to the focused RGB widget, NOT directly to the navigation window.
for key, expected in win.MOVES.items():
    node.commands.clear()
    QtTest.QTest.keyPress(win.rgb_panel.viewport, key)
    assert any(isinstance(command, tuple) and any(command) for command in node.commands), (key, node.commands)
    QtTest.QTest.keyRelease(win.rgb_panel.viewport, key)
    QtTest.QTest.qWait(160)
    assert not win._key_filter.pressed_keys(), (key, node.commands)
node.commands.clear()
QtTest.QTest.keyClick(win.rgb_panel.viewport, QtCore.Qt.Key_X)
assert 'stop_now' in node.commands
QtTest.QTest.keyClick(win.rgb_panel.viewport, QtCore.Qt.Key_Space)
assert 'cancel' in node.commands
host.grab().save(str(ROOT / 'rgb_full.png'))
node.commands.clear()
QtTest.QTest.keyPress(win.rgb_panel.viewport, QtCore.Qt.Key_W)
app.sendEvent(host, QtCore.QEvent(QtCore.QEvent.WindowDeactivate))
assert 'stop_now' in node.commands and not win._key_filter.pressed_keys()
win.map_name_edit.setFocus()
app.processEvents()
node.commands.clear()
QtTest.QTest.keyClicks(win.map_name_edit, 'wasd')
assert win.map_name_edit.text() == 'wasd'
assert not any(isinstance(command, tuple) and any(command) for command in node.commands)
win.rgb_panel.viewport.setFocus()
QtTest.QTest.keyClick(win.rgb_panel.viewport, QtCore.Qt.Key_Escape)
app.processEvents()
assert not win._rgb_fullscreen and win.map_canvas.isVisible()
source.latest = (frame, time.monotonic() - 2, 2)
win.rgb_panel.refresh_frame()
assert win.rgb_panel.viewport.image is None
QtTest.QTest.mouseClick(win.rgb_toggle, QtCore.Qt.LeftButton)
app.processEvents()
assert not source.enabled and not win.rgb_panel.isVisible()
assert win.map_canvas.width() == baseline_width
assert win.map_canvas.zoom == 1.4 and np.array_equal(win.map_canvas.grid, original_grid)
for _ in range(3):
    win.rgb_toggle.setChecked(True)
    win._toggle_rgb_fullscreen()
    win.rgb_toggle.setChecked(False)
    app.processEvents()
    assert win.map_canvas.isVisible() and not source.enabled
win.rgb_toggle.setChecked(True)
app.processEvents()
win.hide()
app.processEvents()
assert not source.enabled
win.show()
app.processEvents()
assert source.enabled
win.close()
assert source.closed
print('PASS: toggle, split/fullscreen, map preservation, all keyboard controls, edit focus, deactivation, stale frame, hide/close')

if '--live' in sys.argv:
    # Read camera only; FakeNode above prevents any real chassis/service calls.
    rgb_spec = importlib.util.spec_from_file_location('live_rgb', SCRIPTS / 'head_rgb_panel.py')
    rgb = importlib.util.module_from_spec(rgb_spec)
    rgb_spec.loader.exec_module(rgb)
    source = rgb.LatestHeadRGB()
    cpu = time.process_time()
    start = time.monotonic()
    source.set_enabled(True)
    frames = set()
    while time.monotonic() - start < 8:
        frame, error = source.snapshot()
        if frame:
            frames.add(frame[2])
        time.sleep(.03)
    elapsed = time.monotonic() - start
    cpu = time.process_time() - cpu
    assert len(frames) > 15, (len(frames), error)
    pixels = frame[0]
    image = QtGui.QImage(pixels.data, pixels.shape[1], pixels.shape[0], pixels.strides[0], QtGui.QImage.Format_RGB888).copy()
    image.save(str(ROOT / 'rgb_live.png'))
    source.set_enabled(False)
    time.sleep(.3)
    assert source.snapshot()[0] is None
    source.set_enabled(True)
    time.sleep(.7)
    assert source.snapshot()[0] is not None
    source.close()
    assert not source._thread.is_alive()
    print(f'PASS: live camera {len(frames)} unique frames/{elapsed:.2f}s, CPU {cpu:.3f}s; pause/resume/close')
