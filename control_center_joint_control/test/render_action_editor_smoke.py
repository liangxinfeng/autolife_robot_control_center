"""Render the keyframe action editor and virtual preview under Xvfb."""

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
        else '/tmp/whole_body_action_editor_test.png'
    )
    rclpy.init()
    app = QtWidgets.QApplication([])
    node = JointControlRosNode()
    panel = ControlPanel(node)
    joints = {
        name: {
            'current_deg': 10.0 if 'Gripper' in name else 0.0,
            'preview_deg': 10.0 if 'Gripper' in name else 0.0,
            'controller_target_deg': 10.0 if 'Gripper' in name else 0.0,
            'speed_deg_s': 0.0,
            'torque': 0.0,
            'temperature_c': 30.0,
            'communication_lost': False,
        }
        for name in ALL_JOINTS
    }
    panel.on_status({
        'connected': True,
        'feedback_age_sec': 0.01,
        'joint_count': 23,
        'hardware_enabled': False,
        'hardware_ready': False,
        'leg_waist_enabled': False,
        'auto_lock_remaining_sec': 0.0,
        'active_goal_error_deg': {},
        'action_running': False,
        'event': '动作编辑器渲染测试',
        'event_level': 'info',
        'whole_body_command_publisher_count': 1,
        'joints': joints,
    })
    panel.tabs.setCurrentWidget(panel.action_page)
    panel.action_page.new_action()
    left_arm = panel.pages['left_arm']
    left_arm.rows['Joint_Left_Shoulder_Outer'].set_target(35.0, notify=True)
    assert panel.action_page.selected_groups() == ['left_arm']
    panel.capture_action_frame(False)
    try:
        panel.action_page.add_captured_frame(
            panel._action_target_values(
                panel.action_page.capture_joint_names()
            )
        )
    except ValueError as error:
        assert '上一帧完全相同' in str(error)
    else:
        raise AssertionError('duplicate action frame was not rejected')

    panel.action_page.new_action()
    panel.schedule_preview('right_arm')
    panel.action_page.name_edit.setText('打招呼')
    right_arm = panel.pages['right_arm']
    right_arm.rows['Joint_Right_Shoulder_Outer'].set_target(55.0)
    right_arm.rows['Joint_Right_Elbow'].set_target(-75.0)
    panel.capture_action_frame(False)
    panel.action_page.transition_box.setValue(0.3)
    right_arm.rows['Joint_Right_Forearm'].set_target(35.0)
    panel.capture_action_frame(False)
    right_arm.rows['Joint_Right_Forearm'].set_target(-35.0)
    panel.capture_action_frame(False)
    panel.action_page.loop_box.setValue(2)
    panel.action_page.speed_box.setValue(200)
    assert panel.pose_page.speed_slider.maximum() == 200
    assert panel.pose_page.speed_box.maximum() == 200
    panel.action_page.frame_list.setCurrentRow(1)
    panel.action_page.copy_frame()
    assert panel.action_page.frame_list.count() == 4
    assert panel.action_page.frame_list.currentRow() == 2
    panel.action_page.remove_frame()
    record = panel.action_page.build_record()
    assert len(record['keyframes']) == 3
    assert record['groups'] == ['right_arm']
    assert record['speed_percent'] == 200
    panel.preview_joint_action('打招呼', record)
    assert panel.action_preview_timer.isActive()
    assert panel.robot_view.view.target_opacity == 0.62
    assert panel.robot_view.target_toggle.isChecked()
    panel.resize(1500, 860)
    panel.show()
    result = {'ok': False}

    def capture():
        image = panel.grab()
        result['ok'] = (
            panel.action_page.frame_list.count() == 3
            and not image.isNull()
            and image.width() >= 1300
            and image.height() >= 760
            and image.save(str(output))
        )
        app.quit()

    QtCore.QTimer.singleShot(800, capture)
    app.exec_()
    panel.stop_action_preview(quiet=True)
    panel.close()
    node.destroy_node()
    rclpy.shutdown()
    if not result['ok']:
        raise SystemExit('action editor render smoke test failed')
    print(f'PASS: rendered keyframe action editor to {output}')


if __name__ == '__main__':
    main()
