#!/usr/bin/env python3
"""Offscreen integration test for the lazy service-manager tab."""

from pathlib import Path

from PyQt5 import QtTest, QtWidgets
import rclpy
from rclpy.parameter import Parameter

import control_center_shell.app as shell_app
from control_center_shell.app import ControlCenterNode, ControlCenterWindow


def main():
    rclpy.init()
    application = (
        QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    )
    node = ControlCenterNode()
    node.set_parameters([
        Parameter('start_navigation', Parameter.Type.BOOL, False),
    ])
    window = ControlCenterWindow(node)
    window.resize(1540, 940)
    window.show()
    application.processEvents()
    assert [window.tabs.tabText(i) for i in range(window.tabs.count())] == [
        '导航与航点', '地图构建', '全身关节', '系统服务', '系统状态', '运行日志']
    assert window.tabs.tabText(3) == '系统服务'

    window.tabs.setCurrentIndex(3)
    QtTest.QTest.qWait(1400)
    application.processEvents()
    panel = window._service_panel
    assert panel is not None
    assert window.stack.currentWidget() is panel
    assert len(panel._core_cards) == 4
    assert panel.service_table.rowCount() > 0
    assert all(
        card.state_label.text() != '检查中'
        for card in panel._core_cards.values()
    )
    assert window._service_action_guard(
        'start', 'vision-service.service'
    )[0]

    class FakeNavigationProcess:
        def __init__(self):
            self.running = True
            self.stop_count = 0
            self.process_group_id = None

        def stop(self):
            self.stop_count += 1
            self.running = False

    original_navigation_process = window.processes['navigation']
    fake_navigation = FakeNavigationProcess()
    window.processes['navigation'] = fake_navigation
    allowed, _message = window._service_action_guard(
        'start', 'gv-slam-service.service'
    )
    assert allowed
    assert fake_navigation.stop_count == 1
    assert not fake_navigation.running

    fake_navigation = FakeNavigationProcess()
    window.processes['navigation'] = fake_navigation
    original_service_probe = shell_app._service_is_active
    shell_app._service_is_active = lambda _unit: True
    window._service_state_observed(
        'gv-slam-service.service', 'active', 'enabled'
    )
    assert fake_navigation.stop_count == 1
    shell_app._service_is_active = original_service_probe
    window.processes['navigation'] = original_navigation_process
    print('PASS: vendor SLAM start and observation stop integrated navigation')
    output = Path('/tmp/control_center_service_manager_integrated.png')
    assert window.grab().save(str(output))
    print(
        'PASS: service manager lazy integration, live status refresh and render: '
        f'{output}'
    )

    window.close()
    application.processEvents()
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
