"""Render pose-library row states under Xvfb."""

from pathlib import Path
import sys

from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy

from control_center_joint_control.control_panel import (
    ControlPanel,
    JointControlRosNode,
)


def main():
    output = Path(
        sys.argv[1] if len(sys.argv) > 1
        else '/tmp/whole_body_pose_library_test.png'
    )
    rclpy.init()
    app = QtWidgets.QApplication([])
    node = JointControlRosNode()
    panel = ControlPanel(node)
    panel.pose_page.set_poses(
        {
            '默认': {'saved_at': '2026-07-30T10:00:00'},
            '鞠躬': {'saved_at': '2026-07-30T10:01:00'},
            '打招呼': {'saved_at': '2026-07-30T10:02:00'},
        },
        Path('/tmp/saved_joint_poses.yaml'),
    )
    panel.tabs.setCurrentWidget(panel.pose_page)
    panel.pose_page.pose_list.setCurrentRow(2)
    panel.resize(1280, 760)
    panel.show()
    result = {'ok': False}

    def capture():
        pose_list = panel.pose_page.pose_list
        middle = pose_list.item(1)
        middle_rect = pose_list.visualItemRect(middle)
        image = panel.grab()
        alternate = pose_list.palette().color(QtGui.QPalette.AlternateBase)
        result['ok'] = (
            pose_list.currentRow() == 2
            and middle.text().startswith('鞠躬')
            and middle_rect.isValid()
            and alternate.lightness() < 100
            and not image.isNull()
            and image.save(str(output))
        )
        app.quit()

    QtCore.QTimer.singleShot(900, capture)
    app.exec_()
    panel.close()
    node.destroy_node()
    rclpy.shutdown()
    if not result['ok']:
        raise SystemExit('pose-library row style smoke test failed')
    print(f'PASS: rendered pose library row states to {output}')


if __name__ == '__main__':
    main()
