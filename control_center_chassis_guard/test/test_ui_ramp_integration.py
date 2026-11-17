"""Ramp behaviour through the real Qt key handlers of both manual driving pages.

The 2026-09-15 teleop smoothing task must not be verified by calling the ramp
directly: the pages own the key stream, the release debounce and the periodic
tick, so these tests drive them exactly as the operator does.  Real key events
go through the real ``keyPressEvent``/``keyReleaseEvent`` handlers, and the tick
is driven by the same callback the production timer/heartbeat calls.

The chassis lease and the ROS publisher are recorders, so nothing is put on a
real topic.  Time is advanced through an injected clock, which is also injected
into the ramp, and the navigation node's heartbeat thread is stopped so the
trace of published commands is deterministic.
"""

import importlib.util
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

CONTROL_CENTER_ROOT = Path(__file__).resolve().parents[2]
NAVIGATION_ROOT = CONTROL_CENTER_ROOT.parent / 'control_center_navigation'
if not (NAVIGATION_ROOT / 'package.xml').is_file():
    NAVIGATION_ROOT = CONTROL_CENTER_ROOT / 'control_center_navigation'
SLAM_ROOT = CONTROL_CENTER_ROOT.parent / 'control_center_slam'
if not (SLAM_ROOT / 'package.xml').is_file():
    SLAM_ROOT = CONTROL_CENTER_ROOT / 'control_center_slam'
RELEASE_DELAY_SEC = 0.03
TICK = 0.02
LINEAR_ACCEL = 0.5
LINEAR_DECEL = 0.5
ANGULAR_ACCEL = 1.0
ANGULAR_DECEL = 1.0
# The navigation page's default linear slider is 0.20 m/s, the angular one
# 0.80 rad/s; the mapping page defaults to 0.18 m/s and 0.45 rad/s.
NAV_LINEAR = 0.20
NAV_ANGULAR = 0.80
SLAM_LINEAR = 0.18
SLAM_ANGULAR = 0.45
ZERO = (0.0, 0.0, 0.0)

KEY_W = QtCore.Qt.Key_W
KEY_S = QtCore.Qt.Key_S
KEY_A = QtCore.Qt.Key_A
KEY_D = QtCore.Qt.Key_D
KEY_X = QtCore.Qt.Key_X


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

    def clear(self):
        self.messages.clear()

    def last(self):
        return self.messages[-1]


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


def _load_module(path, name):
    spec = importlib.util.spec_from_file_location(name, path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def _load_or_skip(path, name):
    try:
        return _load_module(path, name)
    except Exception as error:  # pragma: no cover - environment dependent
        pytest.skip('cannot import %s: %s' % (path.name, error))


@pytest.fixture(scope='module')
def qt_app():
    app = QtWidgets.QApplication.instance()
    if app is None:
        app = QtWidgets.QApplication(['zcode-ramp-test'])
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
        SLAM_ROOT / 'scripts' / 'slam_ui_node.py',
        'zcode_ramp_slam_ui')


@pytest.fixture(scope='module')
def nav_module():
    return _load_or_skip(
        NAVIGATION_ROOT / 'scripts'
        / 'navigation_ui_node.py',
        'zcode_ramp_nav_ui')


def _limits():
    return SmoothingConfig(
        linear_accel=LINEAR_ACCEL, linear_decel=LINEAR_DECEL,
        angular_accel=ANGULAR_ACCEL, angular_decel=ANGULAR_DECEL,
        update_hz=1.0 / TICK)


def _key_event(pressed, key):
    return QtGui.QKeyEvent(
        QtCore.QEvent.KeyPress if pressed else QtCore.QEvent.KeyRelease,
        key, QtCore.Qt.NoModifier, '', False, 1)


def _send(widget, pressed, key=KEY_W):
    QtWidgets.QApplication.sendEvent(widget, _key_event(pressed, key))


def _make_nav(nav_module, clock):
    node = nav_module.NavigationRosNode()
    # The heartbeat is the production tick; the tests call the same step
    # synchronously so the published sequence has no background interleaving.
    node._velocity_shutdown.set()
    node._velocity_thread.join(timeout=1.0)
    window = nav_module.NavigationWindow(node)
    recorder = RecordingPublisher()
    lease = RecordingLease()
    node.velocity_publisher = recorder
    node._chassis_lease = lease
    node._velocity_ramp = VelocityRamp(config=_limits(), clock=clock)
    window._key_filter = ManualKeyFilter(
        release_delay_sec=RELEASE_DELAY_SEC, clock=clock)
    window._release_timer.stop()
    return window, recorder, lease


def _make_slam(slam_module, clock):
    page = slam_module.SlamPage(topic_suffix='0_345')
    page.velocity_timer.stop()
    recorder = RecordingPublisher()
    lease = RecordingLease()
    page.node.velocity_pub = recorder
    page.node._chassis_lease = lease
    page.node._velocity_ramp = VelocityRamp(config=_limits(), clock=clock)
    page._chassis_lease = lease
    page._trajectory_id = 0
    page._key_filter = ManualKeyFilter(
        release_delay_sec=RELEASE_DELAY_SEC, clock=clock)
    page._release_timer.stop()
    return page, recorder, lease


def nav_ticks(window, clock, count):
    for _ in range(count):
        clock.advance(TICK)
        window._apply_pressed_motion()


def slam_ticks(page, clock, count):
    for _ in range(count):
        clock.advance(TICK)
        page._publish_pressed_motion()


def _release_and_settle(widget, recorder, clock, flush, ticks, key=KEY_W):
    """Send a genuine release, let the debounce expire, then tick to rest.

    The flush frame carries the whole debounce interval as its dt, so the
    recorder is cleared after it and only uniform per-tick frames are left for
    the caller's step-bound assertions.
    """
    _send(widget, False, key=key)
    clock.advance(RELEASE_DELAY_SEC + 0.001)
    flush()
    recorder.clear()
    ticks(widget, clock, 60)


def _release_without_settling(widget, clock, flush, key=KEY_W):
    """Apply a genuine release so the next key press starts from the ramp.

    The flush frame carries the debounce interval as its dt, so callers that
    assert a per-tick bound should clear the recorder after this returns.
    """
    _send(widget, False, key=key)
    clock.advance(RELEASE_DELAY_SEC + 0.001)
    flush()


def _axis(messages, index):
    return [message[index] for message in messages]


def _assert_bounded_steps(values, limit, tick=TICK):
    for previous, current in zip(values, values[1:]):
        assert abs(current - previous) <= limit * tick + 1e-9, (
            'velocity changed by more than the configured limit in one tick')


class TestNavigationPageRamp:
    def test_case1_rest_to_forward_ramps_from_zero(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            first = recorder.last()
            assert 0.0 < first[0] < NAV_LINEAR, (
                'press must produce motion on the first frame, below target')
            nav_ticks(window, clock, 30)
            linear = _axis(recorder.messages, 0)
            assert linear == sorted(linear)
            _assert_bounded_steps(linear, LINEAR_ACCEL)
            assert recorder.last()[0] == pytest.approx(NAV_LINEAR)
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case2_release_decelerates_to_zero_and_keeps_the_lease(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            nav_ticks(window, clock, 30)
            assert recorder.last()[0] == pytest.approx(NAV_LINEAR)
            recorder.clear()

            _send(window, False)
            clock.advance(RELEASE_DELAY_SEC + 0.001)
            window._flush_pending_release()

            assert recorder.last()[0] < NAV_LINEAR, (
                'the ramp must decelerate on the release frame')
            assert lease.held, 'the lease must be held while decelerating'
            recorder.clear()

            nav_ticks(window, clock, 30)
            linear = _axis(recorder.messages, 0)
            assert linear == sorted(linear, reverse=True), 'stop must be monotonic'
            _assert_bounded_steps(linear, LINEAR_DECEL)
            assert recorder.last() == ZERO, 'the ramp must land on exactly zero'
            assert lease.released >= 1, 'the lease must be handed back at zero'
            assert not lease.held
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case3_and_4_rotation_ramps_up_and_back_to_zero(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav(nav_module, clock)
        try:
            _send(window, True, key=KEY_A)
            assert 0.0 < recorder.last()[2] < NAV_ANGULAR
            nav_ticks(window, clock, 70)
            angular = _axis(recorder.messages, 2)
            assert angular == sorted(angular)
            _assert_bounded_steps(angular, ANGULAR_ACCEL)
            assert recorder.last()[2] == pytest.approx(NAV_ANGULAR)
            recorder.clear()

            _release_and_settle(
                window, recorder, clock, window._flush_pending_release,
                nav_ticks, key=KEY_A)
            angular = _axis(recorder.messages, 2)
            assert angular == sorted(angular, reverse=True)
            _assert_bounded_steps(angular, ANGULAR_DECEL)
            assert recorder.last() == ZERO
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case5_forward_to_reverse_passes_through_zero(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            nav_ticks(window, clock, 30)
            assert recorder.last()[0] == pytest.approx(NAV_LINEAR)

            # A genuine release first: holding W and S together cancels out.
            _release_without_settling(
                window, clock, window._flush_pending_release)
            _send(window, True, key=KEY_S)
            recorder.clear()
            nav_ticks(window, clock, 80)

            linear = _axis(recorder.messages, 0)
            assert linear[0] < NAV_LINEAR, 'reversal must start by slowing down'
            assert any(value <= 0.0 for value in linear), (
                'the command must reach zero before reversing')
            previous = NAV_LINEAR
            for current in linear:
                if previous > 0.0 and current < 0.0:
                    assert abs(previous) <= LINEAR_DECEL * TICK + 1e-9, (
                        'the sign flipped without a zero crossing')
                previous = current
            assert recorder.last()[0] == pytest.approx(-NAV_LINEAR)
            _assert_bounded_steps(linear, LINEAR_DECEL)
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case6_quick_tap_does_not_run_on(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            nav_ticks(window, clock, 2)
            peak = max(abs(message[0]) for message in recorder.messages)
            assert 0.0 < peak < 0.05, 'a tap must not reach the slider speed'

            _release_and_settle(
                window, recorder, clock, window._flush_pending_release,
                nav_ticks)
            assert recorder.last() == ZERO
            assert peak <= LINEAR_ACCEL * 4 * TICK
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case7_long_hold_reaches_the_slider_speed(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            nav_ticks(window, clock, 300)
            assert recorder.last()[0] == pytest.approx(NAV_LINEAR)
            assert window.node._velocity_ramp.target[0] == pytest.approx(NAV_LINEAR)
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case8_combined_forward_and_turn_ramps_both_axes(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            _send(window, True, key=KEY_D)
            nav_ticks(window, clock, 90)
            assert recorder.last()[0] == pytest.approx(NAV_LINEAR)
            assert recorder.last()[2] == pytest.approx(-NAV_ANGULAR)
            _assert_bounded_steps(_axis(recorder.messages, 0), LINEAR_ACCEL)
            _assert_bounded_steps(_axis(recorder.messages, 2), ANGULAR_ACCEL)
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case9_x_key_stops_immediately_without_ramping(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            nav_ticks(window, clock, 30)
            assert recorder.last()[0] == pytest.approx(NAV_LINEAR)
            recorder.clear()

            _send(window, True, key=KEY_X)

            assert recorder.last() == ZERO, 'the X key must not be rate limited'
            assert window.node._velocity_ramp.idle
            assert lease.released >= 1
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case9_focus_loss_stops_immediately(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            nav_ticks(window, clock, 30)
            recorder.clear()

            QtWidgets.QApplication.sendEvent(
                window, QtGui.QFocusEvent(QtCore.QEvent.FocusOut))

            assert recorder.last() == ZERO
            assert window.node._velocity_ramp.idle
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case11_leaving_the_page_stops_immediately(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            nav_ticks(window, clock, 30)
            recorder.clear()

            # Switching to the mapping page hides this embedded page.
            QtWidgets.QApplication.sendEvent(window, QtGui.QHideEvent())

            assert recorder.last() == ZERO
            assert window.node._velocity_ramp.idle, (
                'no request may survive leaving the page')
        finally:
            window._stop_motion()
            window.node.destroy_node()

    def test_case9_shutdown_does_not_leave_a_command_outstanding(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, _lease = _make_nav(nav_module, clock)
        _send(window, True)
        nav_ticks(window, clock, 30)
        assert recorder.last()[0] > 0.0
        recorder.clear()

        window.node.destroy_node()

        assert recorder.last() == ZERO, (
            'shutdown must not leave the chassis driving')

    def test_no_zero_frames_are_inserted_while_a_key_is_held(
            self, qt_app, ros_context, nav_module):
        clock = FakeClock()
        window, recorder, lease = _make_nav(nav_module, clock)
        try:
            _send(window, True)
            recorder.clear()
            for _ in range(50):
                nav_ticks(window, clock, 1)
                assert recorder.last() != ZERO
            assert lease.released == 0
        finally:
            window._stop_motion()
            window.node.destroy_node()


class TestMappingPageRamp:
    def test_case1_rest_to_forward_ramps_from_zero(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam(slam_module, clock)
        try:
            _send(page, True)
            first = recorder.last()
            assert 0.0 < first[0] < SLAM_LINEAR
            slam_ticks(page, clock, 30)
            linear = _axis(recorder.messages, 0)
            assert linear == sorted(linear)
            _assert_bounded_steps(linear, LINEAR_ACCEL)
            assert recorder.last()[0] == pytest.approx(SLAM_LINEAR)
        finally:
            page.shutdown()

    def test_case2_release_decelerates_to_zero(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, lease = _make_slam(slam_module, clock)
        try:
            _send(page, True)
            slam_ticks(page, clock, 30)
            recorder.clear()

            _send(page, False)
            clock.advance(RELEASE_DELAY_SEC + 0.001)
            page._flush_pending_release()

            assert recorder.last()[0] < SLAM_LINEAR
            assert lease.held
            recorder.clear()

            slam_ticks(page, clock, 30)
            linear = _axis(recorder.messages, 0)
            assert linear == sorted(linear, reverse=True)
            _assert_bounded_steps(linear, LINEAR_DECEL)
            assert recorder.last() == ZERO
            assert lease.released >= 1
        finally:
            page.shutdown()

    def test_case3_and_4_rotation_ramps_up_and_back_to_zero(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam(slam_module, clock)
        try:
            _send(page, True, key=KEY_A)
            assert 0.0 < recorder.last()[2] < SLAM_ANGULAR
            slam_ticks(page, clock, 60)
            _assert_bounded_steps(_axis(recorder.messages, 2), ANGULAR_ACCEL)
            assert recorder.last()[2] == pytest.approx(SLAM_ANGULAR)
            recorder.clear()

            slam_ticks(page, clock, 40)
            # still held: no zero frames while the key is down
            assert all(message != ZERO for message in recorder.messages)

            _release_and_settle(
                page, recorder, clock, page._flush_pending_release,
                slam_ticks, key=KEY_A)
            angular = _axis(recorder.messages, 2)
            assert angular == sorted(angular, reverse=True)
            assert recorder.last() == ZERO
        finally:
            page.shutdown()

    def test_case5_direction_reversal_crosses_zero(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam(slam_module, clock)
        try:
            _send(page, True)
            slam_ticks(page, clock, 30)
            _release_without_settling(
                page, clock, page._flush_pending_release)
            _send(page, True, key=KEY_S)
            recorder.clear()
            slam_ticks(page, clock, 80)

            linear = _axis(recorder.messages, 0)
            assert any(value <= 0.0 for value in linear)
            assert recorder.last()[0] == pytest.approx(-SLAM_LINEAR)
        finally:
            page.shutdown()

    def test_case6_quick_tap_does_not_run_on(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam(slam_module, clock)
        try:
            _send(page, True)
            slam_ticks(page, clock, 2)
            peak = max(abs(message[0]) for message in recorder.messages)
            assert 0.0 < peak < 0.05

            _release_and_settle(
                page, recorder, clock, page._flush_pending_release, slam_ticks)
            assert recorder.last() == ZERO
        finally:
            page.shutdown()

    def test_case7_long_hold_reaches_the_spin_box_speed(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam(slam_module, clock)
        try:
            _send(page, True)
            slam_ticks(page, clock, 300)
            assert recorder.last()[0] == pytest.approx(SLAM_LINEAR)
        finally:
            page.shutdown()

    def test_case9_focus_loss_stops_immediately(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam(slam_module, clock)
        try:
            _send(page, True)
            slam_ticks(page, clock, 30)
            recorder.clear()

            QtWidgets.QApplication.sendEvent(
                page, QtGui.QFocusEvent(QtCore.QEvent.FocusOut))

            assert recorder.last() == ZERO
            assert page.node._velocity_ramp.idle
        finally:
            page.shutdown()

    def test_case9_shutdown_stops_immediately(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam(slam_module, clock)
        _send(page, True)
        slam_ticks(page, clock, 30)
        recorder.clear()

        page.shutdown()

        assert recorder.last() == ZERO
        assert page.node._velocity_ramp.idle

    def test_a_finished_trajectory_cannot_leave_a_command_running(
            self, qt_app, ros_context, slam_module):
        clock = FakeClock()
        page, recorder, _lease = _make_slam(slam_module, clock)
        try:
            _send(page, True)
            slam_ticks(page, clock, 30)
            recorder.clear()

            page._trajectory_id = None
            page._publish_pressed_motion()

            assert recorder.last() == ZERO
            assert page.node.velocity_idle
        finally:
            page.shutdown()


def test_the_shipped_limits_keep_the_default_speeds_operator_responsive():
    """Guard against a tuning change that reintroduces an obvious lag."""
    from control_center_chassis_guard.velocity_ramp import load_smoothing_config
    limits = load_smoothing_config()
    linear_time = NAV_LINEAR / limits.linear_accel
    angular_time = NAV_ANGULAR / limits.angular_accel
    assert linear_time <= 0.5, 'reaching the default linear speed must stay quick'
    assert angular_time <= 0.7, 'reaching the default yaw rate must stay quick'
    first_frame = limits.linear_accel * TICK
    assert first_frame >= 0.005, 'the first frame must be perceivable'
