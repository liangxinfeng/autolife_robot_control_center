"""Real Qt key-stream tests for the manual driving pages.

Required by the 2026-09-10 acceptance report: synthesize a sustained hold with
periodic fake release/press pairs (the exact field waveform) through the real
Qt key handlers and assert the pages never publish an unexpected zero frame.
Calling ``publish_velocity()`` directly is not enough.

The page objects, their Qt timers and their key handlers are the real ones; the
chassis lease and the ROS publisher are replaced by recorders so nothing is put
on a real topic.  Time is advanced through an injected filter clock and the
pending-release flush is invoked directly (that is what the single-shot timer
does in the running application).
"""

import importlib.util
import math
import os
from pathlib import Path

import pytest

os.environ.setdefault('QT_QPA_PLATFORM', 'offscreen')

from PyQt5 import QtCore, QtGui, QtWidgets  # noqa: E402

from control_center_chassis_guard.manual_key_filter import ManualKeyFilter  # noqa: E402
from control_center_chassis_guard.velocity_ramp import (  # noqa: E402
    SmoothingConfig,
    VelocityRamp,
)

PACKAGE_ROOT = Path(__file__).resolve().parents[1]
CONTROL_CENTER_ROOT = PACKAGE_ROOT.parent
RELEASE_DELAY_SEC = 0.03
TICK = 0.02
LINEAR_LIMIT = 0.5
KEY_W = QtCore.Qt.Key_W
ZERO = (0.0, 0.0, 0.0)


def _ramp(clock):
    """Deterministic slew-rate limiter driven by the test clock."""
    return VelocityRamp(
        config=SmoothingConfig(
            linear_accel=LINEAR_LIMIT, linear_decel=LINEAR_LIMIT,
            angular_accel=1.0, angular_decel=1.0, update_hz=1.0 / TICK),
        clock=clock)


def _load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


class FakeClock:
    def __init__(self, start=5000.0):
        self.value = start

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(
            (message.linear.x, message.linear.y, message.angular.z))

    def zeros(self):
        return [message for message in self.messages if message == ZERO]

    def clear(self):
        self.messages.clear()


class RecordingLease:
    def __init__(self):
        self.acquired = 0
        self.released = 0
        self.held = False
        self.last_error = None

    def acquire(self, timeout_sec=None):
        self.acquired += 1
        self.held = True
        return True

    def release(self):
        self.released += 1
        self.held = False
        return True

    def shutdown(self):
        return None


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(['zcode-key-debounce-test'])
    return app


@pytest.fixture(scope='module')
def ros_context():
    rclpy = pytest.importorskip('rclpy')
    if not rclpy.ok():
        rclpy.init()
    yield rclpy


@pytest.fixture(scope='module')
def slam_module():
    return _load_or_skip(
        CONTROL_CENTER_ROOT / 'control_center_slam' / 'scripts' / 'slam_ui_node.py',
        'zcode_keydebounce_slam_ui')


@pytest.fixture(scope='module')
def nav_module():
    return _load_or_skip(
        CONTROL_CENTER_ROOT / 'control_center_navigation' / 'scripts'
        / 'navigation_ui_node.py',
        'zcode_keydebounce_nav_ui')


def _load_or_skip(path, name):
    try:
        return _load_module(path, name)
    except Exception as error:  # pragma: no cover - environment dependent
        pytest.skip('cannot import %s: %s' % (path.name, error))


def _key_event(pressed, key):
    # PyQt5 has no setAutoRepeat(); autorep is a constructor argument.  The
    # field waveform is a real (non-autorepeat) release+press pair.
    return QtGui.QKeyEvent(
        QtCore.QEvent.KeyPress if pressed else QtCore.QEvent.KeyRelease,
        key, QtCore.Qt.NoModifier, '', False, 1)


def _send(widget, pressed, key=KEY_W):
    QtWidgets.QApplication.sendEvent(widget, _key_event(pressed, key))


def _make_slam_page(slam_module, clock):
    page = slam_module.SlamPage(topic_suffix='0_306')
    page.velocity_timer.stop()
    recorder = RecordingPublisher()
    lease = RecordingLease()
    page.node.velocity_pub = recorder
    page.node._chassis_lease = lease
    page.node._velocity_ramp = _ramp(clock)
    page._chassis_lease = lease
    page._trajectory_id = 0
    page._key_filter = ManualKeyFilter(release_delay_sec=RELEASE_DELAY_SEC,
                                       clock=clock)
    page._release_timer.stop()
    return page, recorder, lease


def _make_nav_window(nav_module, clock):
    node = nav_module.NavigationRosNode()
    # Drive the control tick explicitly so the published trace is deterministic.
    node._velocity_shutdown.set()
    node._velocity_thread.join(timeout=1.0)
    window = nav_module.NavigationWindow(node)
    recorder = RecordingPublisher()
    lease = RecordingLease()
    node.velocity_publisher = recorder
    node._chassis_lease = lease
    node._velocity_ramp = _ramp(clock)
    window._key_filter = ManualKeyFilter(release_delay_sec=RELEASE_DELAY_SEC,
                                         clock=clock)
    window._release_timer.stop()
    return window, recorder, lease


def _sustained_hold_with_pairs(page, recorder, clock, publish_tick, iterations=334):
    """10 s of hold at the 30 ms X11 auto-repeat interval, as release+press."""
    for _ in range(iterations):
        _send(page, False)  # spurious X11 release (isAutoRepeat() == False)
        _send(page, True)   # immediate re-press of the same key
        clock.advance(0.030)
        page._flush_pending_release()
        publish_tick()


class TestSlamPageKeyStream:
    def test_sustained_hold_with_pairs_publishes_no_zero_frames(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, lease = _make_slam_page(slam_module, clock)
        try:
            _send(page, True)
            assert recorder.messages[-1] != ZERO
            recorder.clear()
            _sustained_hold_with_pairs(
                page, recorder, clock, page._publish_pressed_motion)
            assert recorder.messages, 'page published nothing during hold'
            assert recorder.zeros() == [], (
                '%d zero frames published during a sustained hold'
                % len(recorder.zeros()))
            assert lease.released == 0, 'lease released during a sustained hold'
        finally:
            page.shutdown()

    def test_real_release_stops_after_debounce(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam_page(slam_module, clock)
        try:
            _send(page, True)
            for _ in range(20):
                clock.advance(TICK)
                page._publish_pressed_motion()
            held = recorder.messages[-1]
            assert held[0] > 0.0
            recorder.clear()
            _send(page, False)  # genuine release, no re-press follows
            page._publish_pressed_motion()
            assert recorder.zeros() == [], 'stopped before the debounce elapsed'
            assert page._release_timer.isActive(), 'flush timer was not armed'
            clock.advance(RELEASE_DELAY_SEC + 0.001)
            page._flush_pending_release()
            assert recorder.messages[-1][0] < held[0], (
                'release did not start decelerating')
            assert recorder.messages[-1] != ZERO, (
                'a normal release must not step straight to zero')
            for _ in range(40):
                clock.advance(TICK)
                page._publish_pressed_motion()
            assert recorder.messages[-1] == ZERO, 'release did not stop the robot'
        finally:
            page.shutdown()

    def test_focus_out_stops_immediately_without_debounce(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam_page(slam_module, clock)
        try:
            _send(page, True)
            recorder.clear()
            QtWidgets.QApplication.sendEvent(
                page, QtGui.QFocusEvent(QtCore.QEvent.FocusOut))
            assert recorder.messages[-1] == ZERO, 'focus-out did not stop immediately'
            assert page._key_filter.pressed_keys() == frozenset()
        finally:
            page.shutdown()


class TestNavigationWindowKeyStream:
    def test_sustained_hold_with_pairs_publishes_no_zero_frames(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, lease = _make_nav_window(nav_module, clock)
        node = window.node
        try:
            _send(window, True)
            recorder.clear()
            _sustained_hold_with_pairs(
                window, recorder, clock, window._apply_pressed_motion)
            assert recorder.messages, 'window published nothing during hold'
            assert recorder.zeros() == [], (
                '%d zero frames published during a sustained hold'
                % len(recorder.zeros()))
            assert lease.released == 0, 'lease released during a sustained hold'
        finally:
            window._stop_motion()
            node.destroy_node()

    def test_real_release_stops_after_debounce(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav_window(nav_module, clock)
        node = window.node
        try:
            _send(window, True)
            for _ in range(30):
                clock.advance(TICK)
                window._apply_pressed_motion()
            held = recorder.messages[-1]
            assert held[0] > 0.0
            recorder.clear()
            _send(window, False)
            window._apply_pressed_motion()
            assert recorder.zeros() == [], 'stopped before the debounce elapsed'
            assert window._release_timer.isActive(), 'flush timer was not armed'
            clock.advance(RELEASE_DELAY_SEC + 0.001)
            window._flush_pending_release()
            assert recorder.messages[-1][0] < held[0], (
                'release did not start decelerating')
            assert recorder.messages[-1] != ZERO, (
                'a normal release must not step straight to zero')
            for _ in range(40):
                clock.advance(TICK)
                window._apply_pressed_motion()
            assert recorder.messages[-1] == ZERO, 'release did not stop the robot'
        finally:
            window._stop_motion()
            node.destroy_node()

    def test_x_key_stops_immediately_without_debounce(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav_window(nav_module, clock)
        node = window.node
        try:
            _send(window, True)
            recorder.clear()
            _send(window, True, key=QtCore.Qt.Key_X)
            assert recorder.messages[-1] == ZERO, 'X key did not stop immediately'
            assert window._key_filter.pressed_keys() == frozenset()
        finally:
            window._stop_motion()
            node.destroy_node()


def test_debounce_window_covers_x11_repeat_interval():
    """The confirm window must exceed the 30 ms X11 auto-repeat interval."""
    assert RELEASE_DELAY_SEC >= 0.025
    assert math.isclose(RELEASE_DELAY_SEC, 0.03, abs_tol=1e-9)
