"""Render the combined controls and native 3D viewer under Xvfb."""

from pathlib import Path
import sys

from PyQt5 import QtCore, QtWidgets
import rclpy

from control_center_joint_control.control_panel import (
    ControlPanel,
    JointControlRosNode,
)
from control_center_joint_control.joint_definitions import ALL_JOINTS


def main():
    output = Path(
        sys.argv[1] if len(sys.argv) > 1
        else '/tmp/whole_body_control_panel_test.png'
    )
    rclpy.init()
    app = QtWidgets.QApplication([])
    node = JointControlRosNode()
    panel = ControlPanel(node)
    joints = {}
    for name in ALL_JOINTS:
        current = 10.0 if 'Gripper' in name else 0.0
        preview = current
        if name == 'Joint_Left_Elbow':
            current, preview = 20.0, 70.0
        if name == 'Joint_Right_Elbow':
            current, preview = -20.0, -65.0
        joints[name] = {
            'current_deg': current,
            'preview_deg': preview,
            'controller_target_deg': current,
            'speed_deg_s': 0.0,
            'torque': 0.0,
            'temperature_c': 30.0,
            'communication_lost': False,
        }
    panel.on_status({
        'connected': True,
        'feedback_age_sec': 0.01,
        'joint_count': 23,
        'hardware_enabled': False,
        'hardware_unlock_mode': 'locked',
        'leg_waist_enabled': False,
        'auto_lock_remaining_sec': 0.0,
        'active_goal_error_deg': {},
        'event': '合并界面渲染测试',
        'event_level': 'info',
        'whole_body_command_publisher_count': 1,
        'joints': joints,
    })
    panel.robot_view.set_joint_mode()
    assert panel.unlock_reset_button.text() == '解锁并复位'
    assert panel.unlock_without_reset_button.text() == '解锁（不复位）'
    assert panel.unlock_reset_button.isEnabled()
    assert panel.unlock_without_reset_button.isEnabled()
    height_control = panel.pages['leg_waist'].body_height_control
    assert height_control is not None
    assert height_control.slider.isEnabled()
    original_yaw = panel.pages['leg_waist'].rows['Joint_Waist_Yaw'].target()
    height_control.target_box.setValue(28.0)
    assert panel.pages['leg_waist'].rows['Joint_Ankle'].target() == 38.0
    assert panel.pages['leg_waist'].rows['Joint_Knee'].target() == 77.5
    assert panel.pages['leg_waist'].rows['Joint_Waist_Pitch'].target() == 40.5
    assert panel.pages['leg_waist'].rows['Joint_Waist_Yaw'].target() == original_yaw
    panel.robot_view._joint_selected('Joint_Left_Elbow')
    assert panel.robot_view.direction_guide.isVisible() is False
    panel.robot_view.selected_box.setValue(55.0)
    assert panel.pages['left_arm'].rows['Joint_Left_Elbow'].target() == 55.0
    assert panel.tabs.currentWidget() is panel.pages['left_arm']
    panel.show()
    assert panel.robot_view.direction_guide.isVisible()
    panel.tabs.setCurrentWidget(panel.pages['leg_waist'])
    result = {'ok': False}

    def capture():
        image = panel.grab()
        result['ok'] = (
            not panel.robot_view.view.render_error
            and not image.isNull()
            and image.width() >= 1100
            and image.height() >= 650
            and image.save(str(output))
        )
        app.quit()

    QtCore.QTimer.singleShot(2000, capture)
    app.exec_()
    panel.close()
    node.destroy_node()
    rclpy.shutdown()
    if not result['ok']:
        raise SystemExit('combined control panel render smoke test failed')
    print(f'PASS: rendered combined control panel to {output}')


if __name__ == '__main__':
    main()
