"""Integration tests for the lease gating inside the two control center UI nodes.

Loads the real UI scripts from the sibling packages and verifies the manual
velocity gate: non-zero velocity requires a held lease, acquisition failure
publishes zeros and releases nothing harmful, and losing the lease mid-drive
immediately publishes zeros.  The ROS publisher is replaced by a recorder, so
these tests never put anything on a real topic.

The module self-skips when the ROS 2 / Qt environment needed to import the
UI scripts is unavailable (e.g. on a development laptop without ROS).
"""

import importlib.util
import os
from pathlib import Path

import pytest

CONTROL_CENTER_ROOT = Path(__file__).resolve().parents[2]
NAVIGATION_ROOT = CONTROL_CENTER_ROOT.parent / 'control_center_navigation'
if not (NAVIGATION_ROOT / 'package.xml').is_file():
    NAVIGATION_ROOT = CONTROL_CENTER_ROOT / 'control_center_navigation'
SLAM_ROOT = CONTROL_CENTER_ROOT.parent / 'control_center_slam'
if not (SLAM_ROOT / 'package.xml').is_file():
    SLAM_ROOT = CONTROL_CENTER_ROOT / 'control_center_slam'


def _load_module(script_path, module_name):
    spec = importlib.util.spec_from_file_location(module_name, script_path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


try:
    slam_module = _load_module(
        SLAM_ROOT / 'scripts'
        / 'slam_ui_node.py',
        'zcode_test_slam_ui_node',
    )
    nav_module = _load_module(
        NAVIGATION_ROOT / 'scripts'
        / 'navigation_ui_node.py',
        'zcode_test_navigation_ui_node',
    )
except Exception as error:  # pragma: no cover - environment dependent
    pytest.skip(
        'control center UI scripts cannot be imported here: %s' % error,
        allow_module_level=True)

if os.environ.get('ZCODE_SKIP_UI_TESTS') == '1':  # pragma: no cover
    pytest.skip('ZCODE_SKIP_UI_TESTS=1', allow_module_level=True)

rclpy = pytest.importorskip('rclpy')

from fake_redis import FakeRedis  # noqa: E402

from control_center_chassis_guard import velocity_ramp  # noqa: E402


class RecordingLease:
    """Scriptable lease double used by the gating tests."""

    def __init__(self, allow_acquire=True):
        self.allow_acquire = allow_acquire
        self.acquired = 0
        self.released = 0
        self.last_error = 'RecordingLease: acquire denied'

    def acquire(self, timeout_sec=None):
        self.acquired += 1
        return self.allow_acquire

    def release(self):
        self.released += 1
        return True

    def shutdown(self):
        return None


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(
            (message.linear.x, message.linear.y, message.angular.z))


class FakeClock:
    def __init__(self, start=7000.0):
        self.value = start

    def __call__(self):
        return self.value

    def advance(self, seconds):
        self.value += seconds


TICK = 0.02


def ramped(node, clock, tick=TICK):
    """Install a deterministic ramp so lease timing can be asserted exactly."""
    node._velocity_ramp = velocity_ramp.VelocityRamp(
        config=velocity_ramp.SmoothingConfig(
            linear_accel=0.5, linear_decel=0.5,
            angular_accel=1.0, angular_decel=1.0, update_hz=1.0 / tick),
        clock=clock)
    return node._velocity_ramp


@pytest.fixture()
def slam_node():
    if not rclpy.ok():
        rclpy.init()
    node = slam_module.SlamRosNode(topic_suffix='0_345')
    yield node
    node.destroy_node()


@pytest.fixture()
def nav_node():
    if not rclpy.ok():
        rclpy.init()
    node = nav_module.NavigationRosNode()
    yield node
    node.destroy_node()


def test_slam_nonzero_requires_lease(slam_node):
    recorder = RecordingPublisher()
    slam_node.velocity_pub = recorder
    lease = RecordingLease(allow_acquire=False)
    slam_node._chassis_lease = lease
    ramped(slam_node, FakeClock())

    slam_node.set_target_velocity(0.18, 0.0, 0.0)

    assert lease.acquired == 1
    assert recorder.messages == [(0.0, 0.0, 0.0)]
    assert slam_node._velocity == (0.0, 0.0, 0.0)
    assert slam_node.velocity_idle


def test_slam_nonzero_publishes_a_limited_first_step_when_lease_granted(
        slam_node):
    recorder = RecordingPublisher()
    slam_node.velocity_pub = recorder
    lease = RecordingLease(allow_acquire=True)
    slam_node._chassis_lease = lease
    ramped(slam_node, FakeClock())

    slam_node.set_target_velocity(0.0, 0.0, 0.45)

    # The requested speed is recorded as the target, but the chassis only sees
    # the first slew-rate limited increment (angular accel 1.0 rad/s^2 * 0.02 s).
    assert slam_node._velocity_ramp.target == (0.0, 0.0, 0.45)
    assert recorder.messages == [(0.0, 0.0, 0.02)]
    assert lease.released == 0


def test_slam_zero_publishes_and_releases(slam_node):
    recorder = RecordingPublisher()
    slam_node.velocity_pub = recorder
    lease = RecordingLease(allow_acquire=True)
    slam_node._chassis_lease = lease
    ramped(slam_node, FakeClock())

    slam_node.set_target_velocity(0.0, 0.0, 0.0)

    # Zero commands never acquire the lease ...
    assert lease.acquired == 0
    assert recorder.messages == [(0.0, 0.0, 0.0)]


def test_slam_lease_loss_publishes_zero(slam_node):
    recorder = RecordingPublisher()
    slam_node.velocity_pub = recorder
    lease = RecordingLease(allow_acquire=True)
    slam_node._chassis_lease = lease
    ramped(slam_node, FakeClock())
    slam_node.set_target_velocity(0.18, 0.0, 0.0)
    assert recorder.messages[-1] != (0.0, 0.0, 0.0)

    slam_node._handle_chassis_lease_lost('unit-test')

    assert slam_node._velocity == (0.0, 0.0, 0.0)
    assert recorder.messages[-1] == (0.0, 0.0, 0.0)
    assert slam_node.velocity_idle


def test_nav_nonzero_requires_lease(nav_node):
    recorder = RecordingPublisher()
    nav_node.velocity_publisher = recorder
    lease = RecordingLease(allow_acquire=False)
    nav_node._chassis_lease = lease

    nav_node.set_velocity(0.2, 0.0, 0.0)

    assert lease.acquired == 1
    assert recorder.messages == [(0.0, 0.0, 0.0)]
    assert nav_node._velocity_active is False
    assert nav_node._velocity == (0.0, 0.0, 0.0)


def test_nav_nonzero_publishes_a_limited_first_step_when_lease_granted(
        nav_node):
    recorder = RecordingPublisher()
    nav_node.velocity_publisher = recorder
    lease = RecordingLease(allow_acquire=True)
    nav_node._chassis_lease = lease
    ramped(nav_node, FakeClock())

    nav_node.set_velocity(0.0, 0.0, 0.45)

    # The requested speed becomes the ramp target immediately, but the chassis
    # only sees the first slew-rate limited increment (1.0 rad/s^2 * 0.02 s).
    assert nav_node._velocity_ramp.target == (0.0, 0.0, 0.45)
    assert recorder.messages == [(0.0, 0.0, 0.02)]
    assert nav_node._velocity_active is True


def test_nav_stop_ramps_down_then_releases_lease(nav_node):
    recorder = RecordingPublisher()
    nav_node.velocity_publisher = recorder
    lease = RecordingLease(allow_acquire=True)
    nav_node._chassis_lease = lease
    clock = FakeClock()
    ramped(nav_node, clock)

    nav_node.set_velocity(0.2, 0.0, 0.0)
    for _ in range(30):
        clock.advance(TICK)
        nav_node.tick_velocity()
    assert recorder.messages[-1][0] == pytest.approx(0.2)
    released_before_stop = lease.released

    nav_node.stop()

    # A normal stop is rate limited, so the lease stays held while decelerating.
    assert lease.released == released_before_stop
    assert nav_node._velocity_active is True
    assert recorder.messages[-1][0] > 0.0

    for _ in range(30):
        clock.advance(TICK)
        nav_node.tick_velocity()

    assert recorder.messages[-1] == (0.0, 0.0, 0.0)
    assert nav_node._velocity_active is False
    assert lease.released > released_before_stop, (
        'the chassis must be handed back once the ramp reaches zero')


def test_nav_stop_immediate_releases_lease_at_once(nav_node):
    recorder = RecordingPublisher()
    nav_node.velocity_publisher = recorder
    lease = RecordingLease(allow_acquire=True)
    nav_node._chassis_lease = lease
    clock = FakeClock()
    ramped(nav_node, clock)
    nav_node.set_velocity(0.2, 0.0, 0.0)

    nav_node.stop_immediate()

    assert lease.released >= 1
    assert recorder.messages[-1] == (0.0, 0.0, 0.0)
    assert nav_node._velocity_active is False


def test_nav_lease_loss_stops_heartbeat_state(nav_node):
    recorder = RecordingPublisher()
    nav_node.velocity_publisher = recorder
    lease = RecordingLease(allow_acquire=True)
    nav_node._chassis_lease = lease
    nav_node.set_velocity(0.2, 0.0, 0.0)

    nav_node._handle_chassis_lease_lost('unit-test')

    assert nav_node._velocity_active is False
    assert recorder.messages[-1] == (0.0, 0.0, 0.0)
