from pathlib import Path
from types import MethodType, SimpleNamespace

from geometry_msgs.msg import Twist
from std_msgs.msg import Bool


SCRIPT = Path(__file__).parents[1] / 'scripts' / 'cmd_vel_arbiter_node.py'


def load_arbiter_class():
    namespace = {'__name__': 'cmd_vel_arbiter_test_module', '__file__': str(SCRIPT)}
    exec(compile(SCRIPT.read_text(encoding='utf-8'), str(SCRIPT), 'exec'), namespace)
    return namespace['CmdVelArbiterNode']


class FakeClock:
    def __init__(self):
        self.value = 0.0

    def now(self):
        return self.value


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


def make_arbiter():
    cls = load_arbiter_class()
    clock = FakeClock()
    publisher = FakePublisher()
    node = SimpleNamespace(
        _clock=clock,
        _publisher=publisher,
        _manual_override_active=False,
        _last_manual_override_stamp=None,
        _manual_override_timeout=0.45,
        _last_nav_stamp=None,
        _nav_priority_timeout=0.5,
        _charging_hold_active=lambda: False,
        _continuity=SimpleNamespace(permits_navigation=lambda: True),
        _filter_nav_cmd_vel=lambda message: message,
        get_logger=lambda: SimpleNamespace(info=lambda _message: None),
    )
    for name in (
        '_manual_override_is_active', '_nav_has_priority',
        '_handle_manual_override', '_handle_nav_cmd_vel', '_handle_arm_cmd_vel',
    ):
        setattr(node, name, MethodType(getattr(cls, name), node))
    return node, clock, publisher


def test_manual_override_suppresses_navigation_and_passes_manual_velocity():
    node, _clock, publisher = make_arbiter()
    node._handle_manual_override(Bool(data=True))
    assert len(publisher.messages) == 1

    navigation = Twist()
    navigation.linear.x = 0.2
    node._handle_nav_cmd_vel(navigation)
    assert len(publisher.messages) == 1

    manual = Twist()
    manual.linear.y = 0.25
    node._handle_arm_cmd_vel(manual)
    assert publisher.messages[-1] is manual


def test_override_transition_publishes_zero_speed_handoff():
    node, clock, publisher = make_arbiter()
    node._handle_manual_override(Bool(data=True))
    assert publisher.messages[-1].linear.x == 0.0
    clock.value = 0.1
    node._handle_manual_override(Bool(data=False))
    assert len(publisher.messages) == 2
    assert publisher.messages[-1].angular.z == 0.0


def test_override_heartbeat_expires_without_refresh():
    node, clock, _publisher = make_arbiter()
    node._handle_manual_override(Bool(data=True))
    clock.value = 0.44
    assert node._manual_override_is_active() is True
    clock.value = 0.46
    assert node._manual_override_is_active() is False


def test_launch_wires_explicit_manual_override_topic():
    launch = (
        Path(__file__).parents[1] / 'launch' / 'bringup_navigation.launch.py'
    ).read_text(encoding='utf-8')
    assert "default_value='/manual_cmd_vel_active'" in launch
    assert "'manual_override_topic': manual_override_topic" in launch
