"""Load real Qt pages on an isolated ROS domain without launching hardware nodes."""
import os
import rclpy
from PyQt5 import QtTest, QtWidgets
from control_center_shell import app
from control_center_shell.system_status import _pid_description
from unittest.mock import patch

assert os.environ['ROS_DOMAIN_ID'] == '183'
qt = QtWidgets.QApplication([])
rclpy.init()
node = app.ControlCenterNode()
with patch.object(app.ManagedProcess,'start',side_effect=AssertionError('unexpected launch')):
    w=app.ControlCenterWindow(node)
    w.centralWidget().setEnabled(True)
    w._load_navigation_ui()
    assert w._navigation_window is not None
    assert w._load_slam_ui()
    assert w._load_joint_ui()
    for index in range(w.tabs.count()):
        w.tabs.setCurrentIndex(index)
        qt.processEvents()
        assert w.stack.currentIndex() == index
        print('rendered',w.tabs.tabText(index),flush=True)
    assert w._status_page is not None and w._service_panel is not None
    w.tabs.setCurrentIndex(4)
    QtTest.QTest.qWait(1250)
    assert str(os.getpid()) in _pid_description(os.getpid())
    assert all(not proc.running for proc in w.processes.values())
    w.close()
    qt.processEvents()
node.destroy_node()
rclpy.shutdown()
