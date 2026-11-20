"""Regression checks for the control center /map ownership guard."""
from types import SimpleNamespace

import pytest
from control_center_shell import preflight
from control_center_shell.resource_observer import Endpoint
from control_center_shell.app import ControlCenterWindow, QtWidgets, _active_map_publishers


class FakeNode:
    def __init__(self, owners):
        self.owners = owners

    def get_parameter(self, key):
        return SimpleNamespace(value='0_367')

    def get_publishers_info_by_topic(self, topic):
        assert topic == '/map'
        return self.owners


@pytest.fixture(autouse=True)
def fake_graph(monkeypatch):
    init = FakeNode.__init__
    def attach(node, owners):
        init(node, owners)
        def read(topics, **kwargs):
            return [Endpoint('/map', f"{ep.node_namespace.rstrip('/')}/{ep.node_name}", str(i + 1))
                    for i, ep in enumerate(node.get_publishers_info_by_topic(topics[0]))]
        monkeypatch.setattr(preflight._observer.GRAPH, 'read', read)
    monkeypatch.setattr(FakeNode, '__init__', attach)


def test_empty_map_has_no_owner():
    assert _active_map_publishers(FakeNode([])) == []


def test_slam_toolbox_and_map_server_are_both_detected():
    owners = [
        SimpleNamespace(node_namespace='/', node_name='slam_toolbox'),
        SimpleNamespace(node_namespace='/', node_name='map_server'),
    ]
    assert sorted(_active_map_publishers(FakeNode(owners))) == ['/map_server', '/slam_toolbox']


def test_duplicate_map_endpoints_remain_visible():
    owner = SimpleNamespace(node_namespace='/mapping', node_name='slam_toolbox')
    assert _active_map_publishers(FakeNode([owner, owner])) == ['/mapping/slam_toolbox'] * 2


def test_gui_start_is_blocked_when_slam_toolbox_owns_map(monkeypatch):
    alerts, states = [], []
    monkeypatch.setattr(QtWidgets.QMessageBox, 'warning', lambda *args: alerts.append(args))
    owner = SimpleNamespace(node_namespace='/', node_name='slam_toolbox')
    window = SimpleNamespace(node=FakeNode([owner]), _append_log=lambda *args: None)
    host = SimpleNamespace(set_state=states.append)
    assert not ControlCenterWindow._require_unowned_map(window, 'slam', host)
    assert states == ['error']
    assert alerts


def test_gui_start_is_allowed_when_map_is_unowned():
    window = SimpleNamespace(node=FakeNode([]), _append_log=lambda *args: None)
    host = SimpleNamespace(set_state=lambda *_: None)
    assert ControlCenterWindow._require_unowned_map(window, 'navigation', host)
