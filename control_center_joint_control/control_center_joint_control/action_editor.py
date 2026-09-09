"""Compact keyframe action editor used by the joint-control panel."""

from copy import deepcopy
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from .action_library import (
    EASING_MODES,
    action_joint_names,
    make_action_record,
    normalize_action_record,
)
from .joint_definitions import GROUPS, GROUP_ORDER


class ActionLibraryPage(QtWidgets.QWidget):
    capture_requested = QtCore.pyqtSignal(bool)
    frame_selected = QtCore.pyqtSignal(object)
    save_requested = QtCore.pyqtSignal(str, object)
    delete_requested = QtCore.pyqtSignal(str)
    preview_requested = QtCore.pyqtSignal(str, object)
    stop_preview_requested = QtCore.pyqtSignal()
    execute_requested = QtCore.pyqtSignal(str, object)
    cancel_requested = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self._actions = {}
        self._frames = []
        self._changing = False
        self._path = ''
        self._group_selection_manual = False

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(12, 12, 12, 12)
        layout.setSpacing(8)

        note = QtWidgets.QLabel(
            '用当前关节“目标值”制作关键帧。先完整播放三维虚影，'
            '确认无碰撞后再执行真机动作。'
        )
        note.setObjectName('groupNote')
        note.setWordWrap(True)
        layout.addWidget(note)

        library_row = QtWidgets.QHBoxLayout()
        library_row.addWidget(QtWidgets.QLabel('动作'))
        self.action_combo = QtWidgets.QComboBox()
        self.action_combo.setObjectName('actionCombo')
        self.action_combo.currentIndexChanged.connect(self._load_selected_action)
        self.new_button = QtWidgets.QPushButton('新建')
        self.new_button.clicked.connect(self.new_action)
        self.delete_action_button = QtWidgets.QPushButton('删除动作')
        self.delete_action_button.clicked.connect(self._delete_action)
        library_row.addWidget(self.action_combo, 1)
        library_row.addWidget(self.new_button)
        library_row.addWidget(self.delete_action_button)
        layout.addLayout(library_row)

        name_row = QtWidgets.QHBoxLayout()
        name_row.addWidget(QtWidgets.QLabel('名称'))
        self.name_edit = QtWidgets.QLineEdit()
        self.name_edit.setPlaceholderText('例如：打招呼')
        self.name_edit.setMaxLength(32)
        self.save_button = QtWidgets.QPushButton('保存动作')
        self.save_button.clicked.connect(self._save_action)
        name_row.addWidget(self.name_edit, 1)
        name_row.addWidget(self.save_button)
        layout.addLayout(name_row)

        group_frame = QtWidgets.QFrame()
        group_frame.setObjectName('actionGroupSelector')
        group_row = QtWidgets.QHBoxLayout(group_frame)
        group_row.setContentsMargins(8, 5, 8, 5)
        group_row.addWidget(QtWidgets.QLabel('参与关节组'))
        self.group_checks = {}
        for group_name in GROUP_ORDER:
            check = QtWidgets.QCheckBox(GROUPS[group_name]['label'])
            check.setChecked(group_name == 'right_arm')
            check.toggled.connect(self._group_toggled)
            self.group_checks[group_name] = check
            group_row.addWidget(check)
        group_row.addStretch()
        layout.addWidget(group_frame)
        self.group_summary = QtWidgets.QLabel('')
        self.group_summary.setObjectName('safetyHint')
        self.group_summary.setWordWrap(True)
        layout.addWidget(self.group_summary)

        self.frame_list = QtWidgets.QListWidget()
        self.frame_list.setObjectName('actionFrameList')
        self.frame_list.setAlternatingRowColors(True)
        self.frame_list.setUniformItemSizes(True)
        self.frame_list.currentRowChanged.connect(self._frame_changed)
        layout.addWidget(self.frame_list, 1)

        edit_row = QtWidgets.QHBoxLayout()
        self.add_button = QtWidgets.QPushButton('＋ 添加当前目标')
        self.replace_button = QtWidgets.QPushButton('替换选中帧')
        self.copy_button = QtWidgets.QPushButton('复制帧')
        self.remove_button = QtWidgets.QPushButton('删除帧')
        self.up_button = QtWidgets.QPushButton('上移')
        self.down_button = QtWidgets.QPushButton('下移')
        self.add_button.clicked.connect(lambda: self.capture_requested.emit(False))
        self.replace_button.clicked.connect(lambda: self.capture_requested.emit(True))
        self.copy_button.clicked.connect(self.copy_frame)
        self.remove_button.clicked.connect(self.remove_frame)
        self.up_button.clicked.connect(lambda: self.move_frame(-1))
        self.down_button.clicked.connect(lambda: self.move_frame(1))
        for button in (
            self.add_button, self.replace_button, self.copy_button,
            self.remove_button,
            self.up_button, self.down_button,
        ):
            edit_row.addWidget(button)
        edit_row.addStretch()
        layout.addLayout(edit_row)

        timing = QtWidgets.QHBoxLayout()
        timing.addWidget(QtWidgets.QLabel('到达'))
        self.transition_box = QtWidgets.QDoubleSpinBox()
        self.transition_box.setRange(0.2, 30.0)
        self.transition_box.setSingleStep(0.1)
        self.transition_box.setValue(1.5)
        self.transition_box.setSuffix(' s')
        timing.addWidget(self.transition_box)
        timing.addWidget(QtWidgets.QLabel('停留'))
        self.hold_box = QtWidgets.QDoubleSpinBox()
        self.hold_box.setRange(0.0, 30.0)
        self.hold_box.setSingleStep(0.1)
        self.hold_box.setValue(0.2)
        self.hold_box.setSuffix(' s')
        timing.addWidget(self.hold_box)
        timing.addWidget(QtWidgets.QLabel('平滑'))
        self.easing_combo = QtWidgets.QComboBox()
        self.easing_combo.addItem('平滑加减速', 'smooth')
        self.easing_combo.addItem('线性', 'linear')
        timing.addWidget(self.easing_combo)
        timing.addStretch()
        timing.addWidget(QtWidgets.QLabel('循环'))
        self.loop_box = QtWidgets.QSpinBox()
        self.loop_box.setRange(1, 20)
        self.loop_box.setValue(1)
        self.loop_box.setSuffix(' 次')
        timing.addWidget(self.loop_box)
        layout.addLayout(timing)
        self.transition_box.valueChanged.connect(self._timing_changed)
        self.hold_box.valueChanged.connect(self._timing_changed)
        self.easing_combo.currentIndexChanged.connect(self._timing_changed)

        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel('真机执行速度'))
        speed_row.addWidget(QtWidgets.QLabel('慢'))
        self.speed_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.speed_slider.setRange(10, 200)
        self.speed_slider.setSingleStep(5)
        self.speed_slider.setPageStep(10)
        self.speed_slider.setValue(100)
        self.speed_slider.setTickPosition(QtWidgets.QSlider.TicksBelow)
        self.speed_slider.setTickInterval(10)
        self.speed_box = QtWidgets.QSpinBox()
        self.speed_box.setRange(10, 200)
        self.speed_box.setSingleStep(5)
        self.speed_box.setValue(100)
        self.speed_box.setSuffix('%')
        self.speed_box.setMinimumWidth(78)
        self.speed_slider.valueChanged.connect(self.speed_box.setValue)
        self.speed_box.valueChanged.connect(self.speed_slider.setValue)
        speed_row.addWidget(self.speed_slider, 1)
        speed_row.addWidget(QtWidgets.QLabel('快'))
        speed_row.addWidget(self.speed_box)
        self.speed_hint = QtWidgets.QLabel(
            '速度会缩放过渡时间；到时未到位会继续等待实际到达'
        )
        self.speed_hint.setObjectName('safetyHint')
        speed_row.addWidget(self.speed_hint)
        layout.addLayout(speed_row)

        playback = QtWidgets.QHBoxLayout()
        self.preview_button = QtWidgets.QPushButton('▶ 三维虚影预览')
        self.stop_preview_button = QtWidgets.QPushButton('■ 停止预览')
        self.execute_button = QtWidgets.QPushButton('让机器人执行')
        self.execute_button.setObjectName('applyButton')
        self.cancel_button = QtWidgets.QPushButton('立即停止动作')
        self.cancel_button.setObjectName('actionStopButton')
        self.preview_button.clicked.connect(self._preview)
        self.stop_preview_button.clicked.connect(self.stop_preview_requested.emit)
        self.execute_button.clicked.connect(self._execute)
        self.cancel_button.clicked.connect(self.cancel_requested.emit)
        playback.addWidget(self.preview_button)
        playback.addWidget(self.stop_preview_button)
        playback.addWidget(self.execute_button)
        playback.addWidget(self.cancel_button)
        playback.addStretch()
        layout.addLayout(playback)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, 1000)
        self.progress.setValue(0)
        self.progress.setFormat('动作未运行')
        layout.addWidget(self.progress)
        self.path_label = QtWidgets.QLabel('')
        self.path_label.setObjectName('safetyHint')
        self.path_label.setWordWrap(True)
        layout.addWidget(self.path_label)
        self._update_controls()

    def set_actions(self, actions, path, preferred=''):
        self._actions = deepcopy(actions)
        current = preferred or self.current_action_name()
        self._path = str(path)
        self._changing = True
        self.action_combo.clear()
        self.action_combo.addItem('— 新建未保存动作 —', '')
        for name in self._actions:
            self.action_combo.addItem(name, name)
        index = self.action_combo.findData(current)
        self.action_combo.setCurrentIndex(max(0, index))
        self._changing = False
        directory = Path(self._path).parent
        self.path_label.setText(
            f'统一保存目录：{directory}\n'
            f'动作YAML：{Path(self._path).name}　'
            f'兼容PKL：{directory / "action_pkls"}'
        )
        if index >= 0:
            self.load_action(current)
        elif not self._frames:
            self.new_action()

    def current_action_name(self):
        return str(self.action_combo.currentData() or '')

    def selected_groups(self):
        return [
            name for name in GROUP_ORDER
            if self.group_checks[name].isChecked()
        ]

    def new_action(self):
        self._changing = True
        self.action_combo.setCurrentIndex(0)
        self.name_edit.clear()
        self._frames = []
        self._group_selection_manual = False
        self.loop_box.setValue(1)
        self.speed_box.setValue(100)
        for name, check in self.group_checks.items():
            check.setChecked(name == 'right_arm')
        self._changing = False
        self.refresh_frames()

    def load_action(self, name):
        record = self._actions.get(str(name))
        if record is None:
            return
        self._changing = True
        self.name_edit.setText(str(name))
        for group, check in self.group_checks.items():
            check.setChecked(group in record['groups'])
        self.loop_box.setValue(int(record.get('loop_count', 1)))
        self.speed_box.setValue(int(record.get('speed_percent', 100)))
        self._frames = deepcopy(record['keyframes'])
        self._group_selection_manual = True
        self._changing = False
        self.refresh_frames()
        if self._frames:
            self.frame_list.setCurrentRow(0)

    def _load_selected_action(self):
        if self._changing:
            return
        name = self.current_action_name()
        if name:
            self.load_action(name)
        else:
            self.new_action()

    def capture_joint_names(self):
        return action_joint_names(self.selected_groups())

    def has_frames(self):
        return bool(self._frames)

    def suggest_groups(self, groups):
        """Follow the last edited body group until the user chooses manually."""
        if self._frames or self._group_selection_manual:
            return
        groups = [name for name in groups if name in self.group_checks]
        if not groups:
            return
        self._changing = True
        for name, check in self.group_checks.items():
            check.setChecked(name in groups)
        self._changing = False
        self._update_controls()

    def add_captured_frame(self, values, replace=False):
        groups = self.selected_groups()
        joint_names = action_joint_names(groups)
        frame = {
            'transition_sec': float(self.transition_box.value()),
            'hold_sec': float(self.hold_box.value()),
            'easing': str(self.easing_combo.currentData()),
            'joints_deg': {name: float(values[name]) for name in joint_names},
        }
        row = self.frame_list.currentRow()
        if replace and 0 <= row < len(self._frames):
            previous = self._frames[row]
            maximum_delta = max(
                abs(frame['joints_deg'][name] - previous['joints_deg'][name])
                for name in joint_names
            )
            if maximum_delta < 0.05:
                raise ValueError('新目标与选中关键帧完全相同，无需替换')
            self._frames[row] = frame
        else:
            if self._frames:
                previous = self._frames[-1]
                maximum_delta = max(
                    abs(frame['joints_deg'][name] - previous['joints_deg'][name])
                    for name in joint_names
                )
                if maximum_delta < 0.05:
                    labels = '、'.join(
                        GROUPS[name]['label'] for name in groups
                    )
                    raise ValueError(
                        f'新目标与上一帧完全相同。当前只记录：{labels}；'
                        '请先调整这些关节，或在第一帧前重新选择参与关节组'
                    )
            row = len(self._frames)
            self._frames.append(frame)
        self.refresh_frames()
        self.frame_list.setCurrentRow(row)

    def remove_frame(self):
        row = self.frame_list.currentRow()
        if 0 <= row < len(self._frames):
            del self._frames[row]
            self.refresh_frames()
            if self._frames:
                self.frame_list.setCurrentRow(min(row, len(self._frames) - 1))

    def copy_frame(self):
        row = self.frame_list.currentRow()
        if not 0 <= row < len(self._frames):
            return
        destination = row + 1
        self._frames.insert(destination, deepcopy(self._frames[row]))
        self.refresh_frames()
        self.frame_list.setCurrentRow(destination)

    def move_frame(self, offset):
        row = self.frame_list.currentRow()
        destination = row + int(offset)
        if not (0 <= row < len(self._frames) and 0 <= destination < len(self._frames)):
            return
        self._frames[row], self._frames[destination] = (
            self._frames[destination], self._frames[row]
        )
        self.refresh_frames()
        self.frame_list.setCurrentRow(destination)

    def refresh_frames(self):
        selected = self.frame_list.currentRow()
        self._changing = True
        self.frame_list.clear()
        for index, frame in enumerate(self._frames, 1):
            easing = '平滑' if frame.get('easing') == 'smooth' else '线性'
            if index == 1:
                delta_text = '首帧'
            else:
                previous = self._frames[index - 2]['joints_deg']
                maximum_delta = max(
                    abs(value - previous[name])
                    for name, value in frame['joints_deg'].items()
                )
                delta_text = f'变化 {maximum_delta:.1f}°'
            self.frame_list.addItem(
                f'关键帧 {index:02d}　到达 {frame["transition_sec"]:.1f}s　'
                f'停留 {frame["hold_sec"]:.1f}s　{easing}　{delta_text}'
            )
        self._changing = False
        if self._frames and selected >= 0:
            self.frame_list.setCurrentRow(min(selected, len(self._frames) - 1))
        self._update_controls()

    def _frame_changed(self, row):
        if self._changing or not 0 <= row < len(self._frames):
            self._update_controls()
            return
        frame = self._frames[row]
        self._changing = True
        self.transition_box.setValue(float(frame['transition_sec']))
        self.hold_box.setValue(float(frame['hold_sec']))
        index = self.easing_combo.findData(frame.get('easing', 'smooth'))
        self.easing_combo.setCurrentIndex(max(0, index))
        self._changing = False
        self.frame_selected.emit(dict(frame['joints_deg']))
        self._update_controls()

    def _timing_changed(self):
        if self._changing:
            return
        row = self.frame_list.currentRow()
        if not 0 <= row < len(self._frames):
            return
        self._frames[row]['transition_sec'] = float(self.transition_box.value())
        self._frames[row]['hold_sec'] = float(self.hold_box.value())
        self._frames[row]['easing'] = str(self.easing_combo.currentData())
        self.refresh_frames()
        self.frame_list.setCurrentRow(row)

    def build_record(self):
        return make_action_record(
            self.selected_groups(),
            deepcopy(self._frames),
            self.loop_box.value(),
            self.speed_box.value(),
        )

    def _save_action(self):
        try:
            record = self.build_record()
        except ValueError as error:
            QtWidgets.QMessageBox.warning(self, '动作不完整', str(error))
            return
        self.save_requested.emit(self.name_edit.text(), record)

    def _delete_action(self):
        name = self.current_action_name()
        if name:
            self.delete_requested.emit(name)

    def _preview(self):
        try:
            record = self.build_record()
        except ValueError as error:
            QtWidgets.QMessageBox.warning(self, '无法预览', str(error))
            return
        self.preview_requested.emit(self.name_edit.text().strip() or '未命名动作', record)

    def _execute(self):
        try:
            record = self.build_record()
        except ValueError as error:
            QtWidgets.QMessageBox.warning(self, '无法执行', str(error))
            return
        self.execute_requested.emit(self.name_edit.text().strip() or '未命名动作', record)

    def set_progress(self, running, name='', frame_index=0, frame_count=0,
                     loop_index=0, loop_count=1, progress=0.0, stage=''):
        if not running:
            self.progress.setValue(0)
            self.progress.setFormat('动作未运行')
            return
        self.progress.setValue(round(min(1.0, max(0.0, progress)) * 1000))
        self.progress.setFormat(
            f'{name} · 循环 {loop_index + 1}/{loop_count} · '
            f'关键帧 {frame_index + 1}/{frame_count} · {stage} · %p%'
        )

    def _update_controls(self):
        has_groups = any(check.isChecked() for check in self.group_checks.values())
        has_frames = bool(self._frames)
        row = self.frame_list.currentRow()
        selected = 0 <= row < len(self._frames)
        for check in self.group_checks.values():
            check.setEnabled(not has_frames)
        self.add_button.setEnabled(has_groups)
        self.replace_button.setEnabled(selected)
        self.copy_button.setEnabled(selected)
        self.remove_button.setEnabled(selected)
        self.up_button.setEnabled(selected and row > 0)
        self.down_button.setEnabled(selected and row < len(self._frames) - 1)
        self.preview_button.setEnabled(has_frames)
        self.execute_button.setEnabled(has_frames)
        self.save_button.setEnabled(has_frames)
        self.delete_action_button.setEnabled(bool(self.current_action_name()))
        selected_labels = [
            GROUPS[name]['label'] for name in self.selected_groups()
        ]
        if has_frames:
            prefix = '本动作固定记录'
            suffix = '；若选错，请新建动作后重新选择'
        else:
            prefix = '下一帧将记录'
            suffix = '；调整其他关节组时会自动跟随，也可在此手动多选'
        self.group_summary.setText(
            f'{prefix}：{"、".join(selected_labels) if selected_labels else "尚未选择"}'
            f'{suffix}'
        )

    def _group_toggled(self):
        if not self._changing:
            self._group_selection_manual = True
        self._update_controls()
