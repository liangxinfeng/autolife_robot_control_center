"""Render one embedded-view frame under Xvfb for visual smoke testing."""

from pathlib import Path
import math
import sys

from PyQt5 import QtCore, QtGui, QtWidgets

from control_center_joint_control.robot_view import RobotOpenGLWidget


def main():
    package_root = Path(__file__).resolve().parents[1]
    output = Path(
        sys.argv[1] if len(sys.argv) > 1
        else '/tmp/whole_body_joint_view_test.png'
    )
    app = QtWidgets.QApplication([])
    view = RobotOpenGLWidget(package_root / 'urdf' / 'robot_v3_1.urdf')
    view.resize(900, 700)
    view.set_joint_states(
        {'Joint_Left_Elbow': 20.0, 'Joint_Right_Elbow': -20.0},
        {'Joint_Left_Elbow': 75.0, 'Joint_Right_Elbow': -65.0},
    )
    view.show()
    result = {'ok': False}

    def capture():
        try:
            picked = ''
            for point in (
                QtCore.QPoint(view.width() // 2, view.height() // 2),
                QtCore.QPoint(view.width() // 2, view.height() * 2 // 5),
                QtCore.QPoint(view.width() * 3 // 5, view.height() // 2),
            ):
                picked = view.pick_joint(point)
                if picked:
                    break
            if not picked:
                raise RuntimeError('color picking did not resolve a robot joint')
            view.set_selected_joint(picked)
            view.set_interaction_mode('joint')
            yaw_before = view.yaw
            view.mousePressEvent(QtGui.QMouseEvent(
                QtCore.QEvent.MouseButtonPress,
                QtCore.QPointF(8, 8),
                QtCore.Qt.LeftButton,
                QtCore.Qt.LeftButton,
                QtCore.Qt.NoModifier,
            ))
            view.mouseMoveEvent(QtGui.QMouseEvent(
                QtCore.QEvent.MouseMove,
                QtCore.QPointF(70, 36),
                QtCore.Qt.NoButton,
                QtCore.Qt.LeftButton,
                QtCore.Qt.NoModifier,
            ))
            view.mouseReleaseEvent(QtGui.QMouseEvent(
                QtCore.QEvent.MouseButtonRelease,
                QtCore.QPointF(70, 36),
                QtCore.Qt.LeftButton,
                QtCore.Qt.NoButton,
                QtCore.Qt.NoModifier,
            ))
            if math.isclose(view.yaw, yaw_before):
                raise RuntimeError('joint mode empty-space drag did not orbit camera')
            image = view.grabFramebuffer()
            result['ok'] = (
                not view.render_error
                and not image.isNull()
                and image.width() >= 800
                and image.height() >= 600
                and image.save(str(output))
            )
            if view.render_error:
                print('render error:', view.render_error)
            print('picked joint:', picked)
        finally:
            app.quit()

    QtCore.QTimer.singleShot(1800, capture)
    app.exec_()
    if not result['ok']:
        raise SystemExit('embedded robot view render smoke test failed')
    print(f'PASS: rendered embedded robot view to {output}')


if __name__ == '__main__':
    main()
