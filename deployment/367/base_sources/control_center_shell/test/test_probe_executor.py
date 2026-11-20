import time
from concurrent.futures import ThreadPoolExecutor
import rclpy
from control_center_shell import preflight


def test_background_probes_do_not_share_or_shutdown_gui_executor():
    # Run only under isolated ROS_DOMAIN_ID, no robot services or motion.
    rclpy.init()
    node=rclpy.create_node('isolated_gui_probe_test')
    calls=[];node.create_timer(.01,lambda:calls.append(1))
    try:
        with ThreadPoolExecutor(max_workers=3) as pool:
            jobs=[pool.submit(preflight._map_publishers,.15) for _ in range(12)]
            while not all(f.done() for f in jobs):rclpy.spin_once(node,timeout_sec=.01)
            assert all(f.result()==[] for f in jobs)
        assert rclpy.ok() and len(calls)>5
    finally:
        node.destroy_node();rclpy.shutdown()


def test_navigation_placeholder_click_reaches_start_handler(monkeypatch):
    from types import SimpleNamespace
    from PyQt5 import QtWidgets, QtCore, QtTest
    from control_center_shell import app
    qt = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    calls = []
    monkeypatch.setattr(app, '_service_is_active', lambda _: False)
    monkeypatch.setattr(app.ControlCenterWindow, 'start_navigation', lambda _: calls.append('start'))
    parameters = {'robot_id': 300, 'topic_suffix': '0_345', 'start_navigation': False}
    node = SimpleNamespace(get_parameter=lambda key: SimpleNamespace(value=parameters[key]))
    window = app.ControlCenterWindow(node)
    window.centralWidget().setEnabled(True)
    placeholder = window.nav_host.findChild(app.EmptyModulePage)
    assert placeholder is not None
    button = placeholder.findChild(QtWidgets.QPushButton)
    QtTest.QTest.mouseClick(button, QtCore.Qt.LeftButton)
    assert calls == ['start']
    window._operator_heartbeat_timer.stop()
    window.deleteLater()
