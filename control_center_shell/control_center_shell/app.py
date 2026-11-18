#!/usr/bin/env python3
import importlib.util
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from PyQt5 import QtCore, QtGui, QtWidgets


APP_TITLE = '机器人综合控制台'
VENDOR_SLAM_SERVICE = 'gv-slam-service.service'
LEGACY_JOINT_PROCESS_PATTERN = (
    'whole_body_joint_control.*(joint_bridge|joint_control_panel)'
)

STYLE = """
QMainWindow, QWidget { background: #242a33; color: #e6edf5; font-size: 14px; }
QFrame#browserBar {
  background: #171c24; border: 0; border-bottom: 1px solid #39424f;
}
QTabBar::tab {
  background: transparent; color: #aeb9c8; border: 0;
  border-bottom: 3px solid transparent;
  padding: 9px 16px 7px 16px; margin: 0 1px; min-width: 92px;
}
QTabBar::tab:selected {
  background: #242a33; color: #f3f7fb; font-weight: 700;
  border-bottom: 3px solid #2e91d1;
}
QTabBar::tab:hover:!selected { background: #252c37; color: #dbe4ee; }
QLabel#stateRunning, QLabel#stateStopped, QLabel#stateStarting, QLabel#stateError {
  border-radius: 9px; padding: 3px 9px; font-size: 12px; font-weight: 700;
}
QLabel#stateRunning { background: #173e31; color: #69d59b; }
QLabel#stateStopped { background: #303844; color: #aeb9c8; }
QLabel#stateStarting { background: #49391e; color: #f2c46d; }
QLabel#stateError { background: #48282b; color: #f19a9f; }
QPushButton {
  background: #303946; color: #dce6f1; border: 1px solid #465363;
  border-radius: 5px; padding: 5px 12px;
}
QPushButton:hover { background: #3a4655; }
QPushButton#startButton { background: #237fba; color: white; border-color: #2e91d1; }
QPushButton#startButton:hover { background: #2e91d1; }
QPushButton#stopButton { background: #44272a; color: #f0a0a5; border-color: #704045; }
QFrame#emptyCard { background: #242a33; border: 1px solid #394450; border-radius: 8px; }
QLabel#emptyTitle { font-size: 20px; font-weight: 700; color: #e6edf5; }
QLabel#emptyHint { color: #9da9b8; font-size: 14px; }
QPlainTextEdit {
  background: #111a28; color: #d6e2f2; border: 0;
  font-family: 'DejaVu Sans Mono'; font-size: 12px;
}
QStatusBar { background: #172338; color: #c9d4e5; }
"""


def _load_libexec_module(package_name, executable_name, module_name):
    path = (
        Path(get_package_prefix(package_name))
        / 'lib'
        / package_name
        / executable_name
    )
    if not path.is_file():
        raise RuntimeError(f'找不到模块入口：{path}')
    spec = importlib.util.spec_from_file_location(module_name, path)
    if spec is None or spec.loader is None:
        raise RuntimeError(f'无法加载模块：{path}')
    module = importlib.util.module_from_spec(spec)
    sys.modules[module_name] = module
    spec.loader.exec_module(module)
    return module


def _service_is_active(name):
    try:
        result = subprocess.run(
            ['systemctl', '--user', 'is-active', '--quiet', name],
            check=False,
            timeout=3.0,
        )
        return result.returncode == 0
    except Exception:
        return False


def _set_service_state(action, name):
    try:
        # gv-slam-service has TimeoutStopSec=20s and owns an entire ROS 2
        # process group.  Give systemd enough time to finish stopping all of
        # its children before checking for conflicting AMCL/TF publishers.
        timeout = 25.0 if action in ('stop', 'restart') else 8.0
        return subprocess.run(
            ['systemctl', '--user', action, name],
            check=False,
            capture_output=True,
            text=True,
            timeout=timeout,
        )
    except Exception as error:
        return error


def _process_matches(pattern):
    try:
        result = subprocess.run(
            ['pgrep', '-af', pattern],
            check=False,
            capture_output=True,
            text=True,
            timeout=2.0,
        )
        return [line for line in result.stdout.splitlines() if line.strip()]
    except Exception:
        return []


def _external_cartographer_processes():
    """Find both SLAM workers that can overwrite /map or map->odom."""
    # This is only called while the managed SLAM module is stopped. Therefore
    # even a worker left by an earlier control-center run is an external orphan.
    return _process_matches('cartographer_(node|occupancy_grid_node)')


def _legacy_joint_processes():
    """Return only processes belonging to the superseded standalone package."""
    return [
        line
        for line in _process_matches(LEGACY_JOINT_PROCESS_PATTERN)
        if 'control_center_joint_control' not in line
    ]


def _legacy_joint_pid_is_safe_to_stop(pid):
    """Revalidate a PID immediately before signalling it to avoid PID reuse."""
    try:
        command = Path(f'/proc/{int(pid)}/cmdline').read_bytes().replace(
            b'\0', b' '
        ).decode('utf-8', errors='replace')
    except (FileNotFoundError, PermissionError, OSError, ValueError):
        return False
    return (
        'whole_body_joint_control' in command
        and (
            'joint_bridge' in command
            or 'joint_control_panel' in command
        )
        and 'control_center_joint_control' not in command
    )


def _legacy_joint_pids():
    pids = []
    for line in _legacy_joint_processes():
        fields = line.strip().split(maxsplit=1)
        if not fields or not fields[0].isdigit():
            continue
        pid = int(fields[0])
        if pid != os.getpid() and _legacy_joint_pid_is_safe_to_stop(pid):
            pids.append(pid)
    return sorted(set(pids), reverse=True)


def _stop_legacy_joint_processes():
    """Stop orphaned standalone joint UI/bridge processes before replacement."""
    initial = _legacy_joint_processes()
    if not initial:
        return [], []

    signals_and_timeouts = (
        (signal.SIGINT, 2.5),
        (signal.SIGTERM, 1.5),
        (signal.SIGKILL, 0.5),
    )
    for stop_signal, timeout_sec in signals_and_timeouts:
        pids = _legacy_joint_pids()
        if not pids:
            break
        for pid in pids:
            if not _legacy_joint_pid_is_safe_to_stop(pid):
                continue
            try:
                os.kill(pid, stop_signal)
            except (ProcessLookupError, PermissionError):
                pass
        deadline = time.monotonic() + timeout_sec
        while time.monotonic() < deadline:
            if not _legacy_joint_pids():
                break
            time.sleep(0.05)

    return initial, _legacy_joint_processes()


class ControlCenterNode(Node):
    def __init__(self):
        super().__init__('control_center_shell')
        self.declare_parameter('robot_id', 283)
        self.declare_parameter('topic_suffix', '0_283')
        self.declare_parameter('start_navigation', False)


class ManagedProcess(QtCore.QObject):
    output = QtCore.pyqtSignal(str, str)
    state_changed = QtCore.pyqtSignal(str, str)

    def __init__(self, key, parent=None):
        super().__init__(parent)
        self.key = key
        self.process = QtCore.QProcess(self)
        self.process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        self.process.readyReadStandardOutput.connect(self._read_output)
        self.process.started.connect(self._on_started)
        self.process.errorOccurred.connect(self._on_error)
        self.process.finished.connect(self._on_finished)
        self.intentional_stop = False
        self.process_group_id = None
        self.started_with_setsid = False

    @property
    def running(self):
        return self.process.state() != QtCore.QProcess.NotRunning

    def start(self, arguments):
        if self.running:
            return
        executable = shutil.which('ros2') or '/opt/ros/jazzy/bin/ros2'
        setsid = shutil.which('setsid')
        self.started_with_setsid = bool(setsid)
        self.intentional_stop = False
        self.state_changed.emit(self.key, 'starting')
        self.output.emit(self.key, '$ ' + executable + ' ' + ' '.join(arguments))
        if setsid:
            self.process.start(setsid, [executable, *arguments])
        else:
            self.process.start(executable, arguments)

    def stop(self, timeout_ms=5000):
        if not self.running:
            self.state_changed.emit(self.key, 'stopped')
            return
        self.intentional_stop = True
        self.output.emit(self.key, '正在停止模块……')
        # QProcess.finished can clear self.process_group_id inside waitForFinished.
        # Capture the owned session now; a dead launch parent does not prove
        # its controller/costmap children have exited.
        owned_group = self.process_group_id if os.name == 'posix' else None
        if owned_group == os.getpgrp():
            owned_group = None
        deadline = time.monotonic() + timeout_ms / 1000.0
        if owned_group is not None:
            try:
                os.killpg(owned_group, signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            self.process.terminate()
        parent_exited = self.process.waitForFinished(timeout_ms)
        if owned_group is not None:
            def group_alive():
                try:
                    os.killpg(owned_group, 0)
                    return True
                except ProcessLookupError:
                    return False
            while group_alive() and time.monotonic() < deadline:
                time.sleep(0.05)
            for sig, grace in ((signal.SIGTERM, 1.8), (signal.SIGKILL, 0.5)):
                if not group_alive():
                    break
                self.output.emit(self.key, f'清理本模块残留进程组：{sig.name}。')
                try:
                    os.killpg(owned_group, sig)
                except ProcessLookupError:
                    break
                until = time.monotonic() + grace
                while group_alive() and time.monotonic() < until:
                    time.sleep(0.05)
            self.process.waitForFinished(100)
        elif not parent_exited:
            self.output.emit(self.key, '模块未及时退出，向启动进程发送 SIGTERM。')
            self.process.terminate()
            if not self.process.waitForFinished(1800):
                self.output.emit(self.key, '启动进程仍未退出，执行最终 SIGKILL。')
                self.process.kill()
                self.process.waitForFinished(1000)

    def _on_started(self):
        # The process is started through `setsid`, so its PID is also the
        # session/process-group id. Signalling the group is essential: killing
        # only `ros2 launch` can orphan Nav2, Cartographer or joint nodes.
        pid = int(self.process.processId())
        # started may be emitted before the setsid utility has called setsid().
        # QProcess children inherit our group, so setsid does not need to fork.
        self.process_group_id = pid if os.name == 'posix' and self.started_with_setsid else None
        self.state_changed.emit(self.key, 'running')

    def _read_output(self):
        raw = bytes(self.process.readAllStandardOutput())
        text = raw.decode('utf-8', errors='replace').rstrip()
        if text:
            self.output.emit(self.key, text)

    def _on_error(self, error):
        self.output.emit(self.key, f'进程错误：{error}')
        self.state_changed.emit(self.key, 'error')

    def _on_finished(self, exit_code, _status):
        if self.intentional_stop or exit_code == 0:
            state = 'stopped'
        else:
            state = 'error'
            self.output.emit(self.key, f'模块异常退出，退出码 {exit_code}')
        self.state_changed.emit(self.key, state)
        self.process_group_id = None


class EmptyModulePage(QtWidgets.QFrame):
    start_requested = QtCore.pyqtSignal()

    def __init__(self, title, hint, button_text='启动模块'):
        super().__init__()
        self.setObjectName('emptyCard')
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(48, 48, 48, 48)
        layout.addStretch()
        icon = QtWidgets.QLabel('◉')
        icon.setAlignment(QtCore.Qt.AlignCenter)
        icon.setStyleSheet('font-size: 52px; color: #75a9cf;')
        title_label = QtWidgets.QLabel(title)
        title_label.setObjectName('emptyTitle')
        title_label.setAlignment(QtCore.Qt.AlignCenter)
        hint_label = QtWidgets.QLabel(hint)
        hint_label.setObjectName('emptyHint')
        hint_label.setWordWrap(True)
        hint_label.setAlignment(QtCore.Qt.AlignCenter)
        button = QtWidgets.QPushButton(button_text)
        button.setObjectName('startButton')
        button.setFixedWidth(180)
        button.clicked.connect(self.start_requested)
        row = QtWidgets.QHBoxLayout()
        row.addStretch()
        row.addWidget(button)
        row.addStretch()
        layout.addWidget(icon)
        layout.addSpacing(8)
        layout.addWidget(title_label)
        layout.addWidget(hint_label)
        layout.addSpacing(20)
        layout.addLayout(row)
        layout.addStretch()


class ModuleHost(QtWidgets.QWidget):
    start_requested = QtCore.pyqtSignal()
    stop_requested = QtCore.pyqtSignal()

    def __init__(self, title, description, placeholder):
        super().__init__()
        self.title = title
        self.description = description
        self.state = 'stopped'
        self.content = None
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        # These controls retain the host API used by process-state handling.
        # The visible controls live in the single compact toolbar above all pages.
        self.state_label = QtWidgets.QLabel('未启动')
        self.state_label.setObjectName('stateStopped')
        self.start_button = QtWidgets.QPushButton('启动')
        self.start_button.setObjectName('startButton')
        self.stop_button = QtWidgets.QPushButton('停止')
        self.stop_button.setObjectName('stopButton')
        self.start_button.clicked.connect(self.start_requested)
        self.stop_button.clicked.connect(self.stop_requested)
        self.stop_button.setEnabled(False)

        self.content_layout = QtWidgets.QVBoxLayout()
        self.content_layout.setContentsMargins(0, 0, 0, 0)
        self.content_layout.addWidget(placeholder)
        self.content = placeholder
        outer.addLayout(self.content_layout, 1)

    def set_content(self, widget, delete_old=True):
        old = self.content
        if old is widget:
            return
        if old is not None:
            self.content_layout.removeWidget(old)
            old.hide()
            if delete_old:
                old.deleteLater()
        self.content = widget
        self.content_layout.addWidget(widget)
        widget.show()

    def set_state(self, state):
        self.state = state
        names = {
            'running': ('运行中', 'stateRunning'),
            'starting': ('启动中', 'stateStarting'),
            'stopped': ('未启动', 'stateStopped'),
            'error': ('异常', 'stateError'),
        }
        text, object_name = names.get(state, names['stopped'])
        self.state_label.setText(text)
        self.state_label.setObjectName(object_name)
        self.state_label.style().unpolish(self.state_label)
        self.state_label.style().polish(self.state_label)
        self.start_button.setEnabled(state in ('stopped', 'error'))
        self.stop_button.setEnabled(state in ('running', 'starting'))


class ControlCenterWindow(QtWidgets.QMainWindow):
    def __init__(self, node):
        super().__init__()
        self.node = node
        self.robot_id = int(node.get_parameter('robot_id').value)
        self.topic_suffix = str(node.get_parameter('topic_suffix').value)
        self.start_navigation_on_open = bool(
            node.get_parameter('start_navigation').value
        )
        self.setWindowTitle(APP_TITLE)
        self.resize(1540, 940)
        self.setMinimumSize(1180, 740)
        self.setStyleSheet(STYLE)
        self._closing = False
        self._vendor_was_active = _service_is_active(VENDOR_SLAM_SERVICE)
        self._vendor_paused = False
        self._navigation_module = None
        self._navigation_node = None
        self._navigation_window = None
        self._navigation_map_snapshot = None
        self._navigation_executor = None
        self._navigation_spin_timer = None
        self._joint_module = None
        self._joint_node = None
        self._joint_panel = None
        self._slam_module = None
        self._slam_page = None
        self._service_panel = None

        self.processes = {
            key: ManagedProcess(key, self)
            for key in ('navigation', 'slam', 'joint')
        }
        for process in self.processes.values():
            process.output.connect(self._append_log)
            process.state_changed.connect(self._process_state_changed)

        self._build_ui()
        self._load_navigation_ui()
        self.statusBar().showMessage(
            f'机器人 {self.robot_id} · 话题后缀 {self.topic_suffix}'
        )
        if self.start_navigation_on_open:
            QtCore.QTimer.singleShot(350, self.start_navigation)

    def _build_ui(self):
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.setSpacing(0)

        browser_bar = QtWidgets.QFrame()
        browser_bar.setObjectName('browserBar')
        browser_bar.setFixedHeight(46)
        browser_layout = QtWidgets.QHBoxLayout(browser_bar)
        browser_layout.setContentsMargins(8, 0, 10, 0)
        browser_layout.setSpacing(8)
        self.tabs = QtWidgets.QTabBar()
        self.tabs.setExpanding(False)
        self.tabs.setDrawBase(False)
        self.tabs.addTab('导航与航点')
        self.tabs.addTab('地图构建')
        self.tabs.addTab('全身关节')
        self.tabs.addTab('系统服务')
        self.tabs.addTab('运行日志')
        self.tabs.currentChanged.connect(self._tab_changed)
        browser_layout.addWidget(self.tabs)
        browser_layout.addStretch(1)
        self.current_state_label = QtWidgets.QLabel('未启动')
        self.current_state_label.setObjectName('stateStopped')
        self.current_start_button = QtWidgets.QPushButton('启动')
        self.current_start_button.setObjectName('startButton')
        self.current_stop_button = QtWidgets.QPushButton('停止')
        self.current_stop_button.setObjectName('stopButton')
        self.current_start_button.clicked.connect(self._start_current_module)
        self.current_stop_button.clicked.connect(self._stop_current_module)
        browser_layout.addWidget(self.current_state_label)
        browser_layout.addWidget(self.current_start_button)
        browser_layout.addWidget(self.current_stop_button)
        layout.addWidget(browser_bar)

        self.stack = QtWidgets.QStackedWidget()
        layout.addWidget(self.stack, 1)

        nav_empty = EmptyModulePage(
            '正在准备导航界面', '导航是综合控制台的默认主功能。'
        )
        self.nav_host = ModuleHost(
            '导航、航点与全局重定位',
            '默认主模块；提供地图、航点、键盘控制和全局重定位。',
            nav_empty,
        )
        self.nav_host.start_requested.connect(self.start_navigation)
        self.nav_host.stop_requested.connect(self.stop_navigation)

        slam_empty = EmptyModulePage(
            '优化双雷达建图',
            'Cartographer 默认不运行。启动时会关闭导航定位，避免 TF 冲突。',
        )
        slam_empty.start_requested.connect(self.start_slam)
        self.slam_host = ModuleHost(
            '地图构建',
            '按需启动；内置地图显示、键盘控制、优化和地图保存。',
            slam_empty,
        )
        self.slam_host.start_requested.connect(self.start_slam)
        self.slam_host.stop_requested.connect(self.stop_slam)

        joint_empty = EmptyModulePage(
            '全身关节监控与动作控制',
            '按需加载三维模型、关节反馈、姿态库和动作库。真机仍需二次解锁。',
        )
        joint_empty.start_requested.connect(self.start_joint)
        self.joint_host = ModuleHost(
            '全身关节控制',
            '按需启动；停止模块后释放反馈、模型和控制节点。',
            joint_empty,
        )
        self.joint_host.start_requested.connect(self.start_joint)
        self.joint_host.stop_requested.connect(self.stop_joint)

        service_empty = EmptyModulePage(
            '系统用户服务',
            '查看和控制视觉、导航、机械臂及其他 systemd 用户服务。',
            button_text='打开服务管理',
        )
        service_empty.start_requested.connect(self._load_service_manager_ui)
        self.service_page = service_empty

        self.log_view = QtWidgets.QPlainTextEdit()
        self.log_view.setReadOnly(True)
        self.log_view.document().setMaximumBlockCount(5000)
        self.stack.addWidget(self.nav_host)
        self.stack.addWidget(self.slam_host)
        self.stack.addWidget(self.joint_host)
        self.stack.addWidget(self.service_page)
        self.stack.addWidget(self.log_view)
        self._refresh_compact_controls()

    def _tab_changed(self, index):
        self.stack.setCurrentIndex(index)
        self._refresh_compact_controls()
        if index == 0 and self._navigation_window is not None:
            self._navigation_window.prepare_for_user_interaction()
        if index == 3:
            self._load_service_manager_ui()
        if index == 4:
            self.log_view.moveCursor(QtGui.QTextCursor.End)

    def _current_module_host(self):
        return {
            0: self.nav_host,
            1: self.slam_host,
            2: self.joint_host,
        }.get(self.tabs.currentIndex())

    def _start_current_module(self):
        host = self._current_module_host()
        if host is not None:
            host.start_requested.emit()

    def _stop_current_module(self):
        host = self._current_module_host()
        if host is not None:
            host.stop_requested.emit()

    def _refresh_compact_controls(self):
        if not hasattr(self, 'current_state_label'):
            return
        host = self._current_module_host()
        visible = host is not None
        self.current_state_label.setVisible(visible)
        self.current_start_button.setVisible(visible)
        self.current_stop_button.setVisible(visible)
        if not visible:
            return
        names = {
            'running': ('运行中', 'stateRunning'),
            'starting': ('启动中', 'stateStarting'),
            'stopped': ('未启动', 'stateStopped'),
            'error': ('异常', 'stateError'),
        }
        state = host.state
        text, object_name = names.get(state, names['stopped'])
        self.current_state_label.setText(text)
        self.current_state_label.setObjectName(object_name)
        self.current_state_label.style().unpolish(self.current_state_label)
        self.current_state_label.style().polish(self.current_state_label)
        self.current_start_button.setVisible(state in ('stopped', 'error'))
        self.current_stop_button.setVisible(state in ('running', 'starting'))
        self.current_start_button.setEnabled(state in ('stopped', 'error'))
        self.current_stop_button.setEnabled(state in ('running', 'starting'))

    def _append_log(self, source, message):
        timestamp = time.strftime('%H:%M:%S')
        for line in str(message).splitlines() or ['']:
            self.log_view.appendPlainText(f'{timestamp} [{source}] {line}')

    def _process_state_changed(self, key, state):
        host = {
            'navigation': self.nav_host,
            'slam': self.slam_host,
            'joint': self.joint_host,
        }[key]
        host.set_state(state)
        self._refresh_compact_controls()
        if key == 'slam' and state in ('stopped', 'error'):
            QtCore.QTimer.singleShot(0, self._finish_slam_map_transition)

    def _finish_slam_map_transition(self):
        if self.processes['slam'].running:
            return
        self._set_navigation_map_updates(False, drain=True)
        self._restore_navigation_map()

    def _service_action_guard(self, action, unit):
        if action == 'joint_reset':
            running = []
            if self.processes['navigation'].running:
                running.append('导航与航点')
            if self.processes['slam'].running:
                running.append('地图构建')
            if self.processes['joint'].running:
                running.append('全身关节控制')
            if running:
                return False, (
                    '全身关节复位前必须先停止可能产生运动命令的模块：'
                    + '、'.join(running)
                )
            return True, ''
        if action not in ('start', 'restart'):
            return True, ''
        vendor_navigation_units = {
            'gv-slam-service.service',
            'gv-control-service.service',
        }
        if unit not in vendor_navigation_units:
            return True, ''
        if self.processes['slam'].running:
            return False, (
                '综合控制台正在构建地图。为避免丢失尚未保存的地图，本次没有启动 '
                f'{unit}；请先停止“地图构建”模块。'
            )
        if self.processes['navigation'].running:
            self._append_log(
                'service',
                f'准备启动 {unit}，先停止“导航与航点”模块及其航点任务。',
            )
            self.stop_navigation()
            if self.processes['navigation'].running:
                return False, '“导航与航点”模块未能完全停止，服务启动已取消。'
        return True, ''

    def _service_state_observed(self, unit, active_state, _enabled_state):
        if unit != 'gv-slam-service.service' or active_state != 'active':
            return
        # A manual start means this is no longer the service instance paused
        # by the integrated navigation module.  Clear the cached flag even
        # when navigation is already stopped, so the next navigation start
        # will stop the newly started service again.
        self._vendor_paused = False
        if not self.processes['navigation'].running:
            return
        # Ignore a stale asynchronous status result produced just before
        # start_navigation() stopped the vendor SLAM service.
        if not _service_is_active(unit):
            return
        self._append_log(
            'service',
            '检测到 SLAM/定位服务已启动，自动停止“导航与航点”模块及其航点任务。',
        )
        self.stop_navigation()

    def _load_service_manager_ui(self):
        if self._service_panel is not None:
            return True
        try:
            from control_center_service_manager.panel import ServiceManagerPanel
            panel = ServiceManagerPanel(
                action_guard=self._service_action_guard,
                auto_refresh=True,
                topic_suffix=self.topic_suffix,
            )
            panel.log_message.connect(
                lambda message: self._append_log('service', message)
            )
            panel.service_state_changed.connect(self._service_state_observed)
            self._service_panel = panel
            old = self.service_page
            self.stack.removeWidget(old)
            old.hide()
            old.deleteLater()
            self.service_page = panel
            self.stack.insertWidget(3, panel)
            self.stack.setCurrentIndex(3)
            self._append_log('service', '系统用户服务管理页面已按需加载。')
            return True
        except Exception as error:
            self._append_log('service', f'加载系统服务页面失败：{error}')
            QtWidgets.QMessageBox.warning(
                self, '系统服务页面加载失败', str(error)
            )
            return False

    def _pause_vendor_stack(self):
        service_active = _service_is_active(VENDOR_SLAM_SERVICE)
        if self._vendor_paused and not service_active:
            return True
        if not service_active:
            return True
        self._append_log('system', f'停止 {VENDOR_SLAM_SERVICE}，由综合控制台接管。')
        result = _set_service_state('stop', VENDOR_SLAM_SERVICE)
        if isinstance(result, Exception) or result.returncode != 0:
            detail = str(result) if isinstance(result, Exception) else result.stderr
            QtWidgets.QMessageBox.critical(
                self, '无法接管机器人导航',
                f'停止 {VENDOR_SLAM_SERVICE} 失败：\n{detail}',
            )
            return False
        if _service_is_active(VENDOR_SLAM_SERVICE):
            QtWidgets.QMessageBox.critical(
                self, '无法接管机器人导航',
                f'{VENDOR_SLAM_SERVICE} 执行停止后仍处于运行状态。请查看该服务日志。',
            )
            return False
        # The vendor launcher exits with status 1 after a normal SIGINT, so
        # systemd records a successful explicit stop as "failed".  Clear only
        # that historical marker; this does not start or otherwise alter the
        # stopped service.
        subprocess.run(
            ['systemctl', '--user', 'reset-failed', VENDOR_SLAM_SERVICE],
            check=False,
            capture_output=True,
            timeout=3.0,
        )
        self._vendor_paused = True
        return True

    def _restore_vendor_stack(self):
        if not (self._vendor_paused and self._vendor_was_active):
            return
        self._append_log('system', f'恢复 {VENDOR_SLAM_SERVICE}。')
        result = _set_service_state('start', VENDOR_SLAM_SERVICE)
        if isinstance(result, Exception) or result.returncode != 0:
            detail = str(result) if isinstance(result, Exception) else result.stderr
            self._append_log('system', f'恢复后台服务失败：{detail}')
        else:
            self._vendor_paused = False

    def _load_navigation_ui(self):
        if self._navigation_module is not None:
            return
        try:
            module = _load_libexec_module(
                'control_center_navigation',
                'navigation_ui_node.py',
                '_control_center_navigation_ui',
            )
            node = module.NavigationRosNode()
            window = module.NavigationWindow(node)
            window.setWindowFlags(QtCore.Qt.Widget)
            window.setMinimumSize(0, 0)
            executor = module.SingleThreadedExecutor()
            executor.add_node(node)
            timer = QtCore.QTimer(self)
            timer.setTimerType(QtCore.Qt.PreciseTimer)
            timer.setInterval(5)
            def drain_navigation_callbacks():
                if (
                    self._closing
                    or self._navigation_spin_timer is not timer
                    or self._navigation_executor is not executor
                    or self._navigation_node is not node
                ):
                    return
                try:
                    module.drain_ros_callbacks(executor)
                except Exception as error:
                    timer.stop()
                    if not self._closing:
                        self._append_log(
                            'navigation', f'导航 ROS 回调已停止：{error}'
                        )

            timer.timeout.connect(drain_navigation_callbacks)
            timer.start()
            self._navigation_module = module
            self._navigation_node = node
            self._navigation_window = window
            self._navigation_executor = executor
            self._navigation_spin_timer = timer
            # The navigation UI is permanent, but /map is shared by Nav2 and
            # Cartographer. Only the active module may feed this canvas.
            node.set_map_updates_enabled(False)
            self.nav_host.set_content(window)
            self._append_log('navigation', '导航页面已嵌入综合控制台。')
        except Exception as error:
            self.nav_host.set_state('error')
            self._append_log('navigation', f'加载导航页面失败：{error}')

    def _capture_navigation_map(self):
        if self._navigation_window is None:
            return
        try:
            snapshot = self._navigation_window.navigation_map_snapshot()
            if snapshot is not None:
                self._navigation_map_snapshot = snapshot
                self._append_log('navigation', '已保存建图前的固定导航地图显示。')
        except Exception as error:
            self._append_log('navigation', f'保存导航地图显示失败：{error}')

    def _restore_navigation_map(self):
        if self._navigation_window is None:
            return
        try:
            self._navigation_window.restore_navigation_map(
                self._navigation_map_snapshot
            )
            self._append_log('navigation', '已清除建图地图并恢复固定导航地图。')
        except Exception as error:
            self._append_log('navigation', f'恢复导航地图显示失败：{error}')

    def _set_navigation_map_updates(self, enabled, drain=False):
        if self._navigation_node is None:
            return
        self._navigation_node.set_map_updates_enabled(enabled)
        if drain and self._navigation_executor is not None:
            try:
                # Discard the transient last Cartographer frame while updates
                # are disabled, before Nav2 is allowed to publish its map.
                for _ in range(12):
                    self._navigation_executor.spin_once(timeout_sec=0.0)
            except Exception as error:
                self._append_log('navigation', f'清理旧地图消息时出现警告：{error}')

    def start_navigation(self):
        if self.processes['navigation'].running:
            return
        if self.processes['slam'].running:
            answer = QtWidgets.QMessageBox.question(
                self, '切换到导航',
                '当前正在建图。切换导航将停止本轮建图，未保存内容可能丢失，是否继续？',
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return
            self.stop_slam()
        external = _external_cartographer_processes()
        if external:
            self.nav_host.set_state('error')
            QtWidgets.QMessageBox.warning(
                self, '检测到其他建图进程',
                '机器人上仍有其他 Cartographer 正在运行。请先退出原建图程序，'
                '再启动综合控制台导航。\n\n' + '\n'.join(external[:4]),
            )
            return
        if not self._pause_vendor_stack():
            self.nav_host.set_state('error')
            return
        remaining_amcl = (_process_matches('/nav2_amcl/amcl') +
                          _process_matches('/control_center_navigation/amcl_startup_compat'))
        if remaining_amcl:
            self.nav_host.set_state('error')
            QtWidgets.QMessageBox.warning(
                self, '检测到其他 AMCL',
                '停止机器人后台服务后仍有其他 AMCL 进程。为了避免 map → odom '
                '冲突，综合导航未启动。请先退出手动启动的旧导航功能包。\n\n'
                + '\n'.join(remaining_amcl[:4]),
            )
            return
        self._load_navigation_ui()
        self._set_navigation_map_updates(False, drain=True)
        self._restore_navigation_map()
        arguments = [
            'launch', 'control_center_navigation', 'bringup_all.launch.py',
            'open_navigation_ui:=false',
            f'text_voice_robot_id:={self.robot_id}',
            'text_voice_ros_domain_id:=0',
        ]
        if self._navigation_node is not None:
            selected_map = str(
                getattr(self._navigation_node, '_loaded_map_path', '') or ''
            ).strip()
            if selected_map:
                arguments.append(f'map_yaml:={selected_map}')
        self._set_navigation_map_updates(True)
        self.processes['navigation'].start(arguments)

    def stop_navigation(self):
        self.processes['navigation'].stop()

    def _load_slam_ui(self, resume_state=None):
        if self._slam_page is not None:
            return True
        try:
            module = _load_libexec_module(
                'control_center_slam', 'slam_ui_node.py',
                '_control_center_slam_ui',
            )
            page = module.SlamPage(topic_suffix=self.topic_suffix, resume_state=resume_state)
            page.stop_requested.connect(self.stop_slam)
            self._slam_module = module
            self._slam_page = page
            self.slam_host.set_content(page)
            return True
        except Exception as error:
            self.slam_host.set_state('error')
            self._append_log('slam', f'加载建图页面失败：{error}')
            return False

    def start_slam(self):
        if self.processes['slam'].running:
            return
        choice, accepted = QtWidgets.QInputDialog.getItem(
            self, '选择建图方式', '本次建图：',
            ['新建地图', '导入旧地图，增量建图'], 0, False)
        if not accepted:
            return
        resume_state = None
        if choice.startswith('导入'):
            filename, _ = QtWidgets.QFileDialog.getOpenFileName(
                self, '选择完整建图状态（原文件保留）',
                str(Path(get_package_prefix('control_center_slam')) / 'share/control_center_slam/maps'),
                'Cartographer 建图状态 (*.pbstream);;地图配置（需同名pbstream） (*.yaml)')
            if not filename:
                return
            try:
                helper = _load_libexec_module('control_center_slam', 'incremental_mapping.py', '_cc_incremental_mapping')
                resume_state = str(helper.resolve_state_file(filename))
            except Exception as error:
                QtWidgets.QMessageBox.warning(self, '无法导入', str(error))
                return
        if self.processes['navigation'].running:
            answer = QtWidgets.QMessageBox.question(
                self, '切换到建图模式',
                '建图与 AMCL 导航不能同时运行。现在将停止导航任务和定位，是否继续？',
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return
            self.stop_navigation()
        external = _external_cartographer_processes()
        if external:
            self.slam_host.set_state('error')
            QtWidgets.QMessageBox.warning(
                self, '检测到其他建图进程',
                '机器人上仍有其他 Cartographer 节点在运行。请先退出旧建图程序，'
                '再启动综合控制台建图，避免地图和 TF 被重复发布。\n\n'
                + '\n'.join(external[:4]),
            )
            return
        if not self._pause_vendor_stack():
            self.slam_host.set_state('error')
            return
        remaining_amcl = (_process_matches('/nav2_amcl/amcl') +
                          _process_matches('/control_center_navigation/amcl_startup_compat'))
        if remaining_amcl:
            self.slam_host.set_state('error')
            QtWidgets.QMessageBox.warning(
                self, '仍有定位进程运行',
                '检测到 AMCL 仍在运行，因此拒绝启动 Cartographer。请先退出其他'
                '导航功能包，防止机器人 TF 大范围跳动。\n\n'
                + '\n'.join(remaining_amcl[:4]),
            )
            return
        if not self._load_slam_ui(resume_state=resume_state):
            return
        self._capture_navigation_map()
        self._set_navigation_map_updates(True)
        launch_args = [
            'launch', 'control_center_slam',
            'multi_lidar_slam_optimized.launch.py',
            'open_rviz:=false',
            'enable_keyboard_control:=false',
            'publish_lidar_static_tf:=true',
            f'front_scan_topic:=/topic_gv_front_lidar_{self.topic_suffix}',
            f'rear_scan_topic:=/topic_gv_rear_lidar_{self.topic_suffix}',
            f'odom_topic:=/topic_gv_wheel_odom_{self.topic_suffix}',
        ]
        if resume_state:
            launch_args.extend([
                f'load_state_filename:={resume_state}',
                'start_new_trajectory:=false',
                'cartographer_configuration_basename:=multi_lidar_incremental.lua',
            ])
        self.processes['slam'].start(launch_args)
        self.tabs.setCurrentIndex(1)

    def stop_slam(self):
        # Block map callbacks before stopping Cartographer. Otherwise its last
        # transient-local OccupancyGrid can be delivered after the snapshot is
        # restored and overwrite the navigation canvas again.
        self._set_navigation_map_updates(False)
        self.processes['slam'].stop()
        self._set_navigation_map_updates(False, drain=True)
        self._restore_navigation_map()
        if self._slam_page is not None:
            try:
                self._slam_page.shutdown()
            except Exception as error:
                self._append_log('slam', f'释放建图页面时出现警告：{error}')
            empty = EmptyModulePage(
                '优化双雷达建图',
                '模块已停止。可再次启动新一轮建图。',
            )
            empty.start_requested.connect(self.start_slam)
            self.slam_host.set_content(empty)
            self._slam_page = None
            self._slam_module = None

    def _load_joint_ui(self):
        if self._joint_panel is not None:
            return True
        try:
            from control_center_joint_control import control_panel
            node = control_panel.JointControlRosNode()
            panel = control_panel.ControlPanel(node)
            panel.setWindowFlags(QtCore.Qt.Widget)
            panel.setMinimumSize(0, 0)
            self._joint_module = control_panel
            self._joint_node = node
            self._joint_panel = panel
            self.joint_host.set_content(panel)
            return True
        except Exception as error:
            self.joint_host.set_state('error')
            self._append_log('joint', f'加载关节界面失败：{error}')
            return False

    def start_joint(self):
        if self.processes['joint'].running:
            return
        old_joint_processes = _legacy_joint_processes()
        if old_joint_processes:
            self._append_log(
                'joint',
                '检测到旧 whole_body_joint_control，正在自动退出旧界面和桥接节点……',
            )
            stopped, remaining = _stop_legacy_joint_processes()
            if remaining:
                self.joint_host.set_state('error')
                QtWidgets.QMessageBox.warning(
                    self, '旧关节控制包无法自动退出',
                    '已尝试自动停止原 whole_body_joint_control，但仍有进程存在。'
                    '为避免两套桥接节点同时控制机器人，本次未启动。\n\n'
                    + '\n'.join(remaining[:4]),
                )
                return
            self._append_log(
                'joint',
                f'旧关节包已自动退出（清理 {len(stopped)} 个进程），继续启动当前模块。',
            )
        if not self._load_joint_ui():
            return
        self.processes['joint'].start([
            'launch', 'control_center_joint_control',
            'whole_body_joint_control.launch.py',
            'panel:=false',
            f'robot_id:={self.robot_id}',
            f'topic_suffix:={self.topic_suffix}',
        ])
        self.tabs.setCurrentIndex(2)

    def stop_joint(self):
        self.processes['joint'].stop()
        if self._joint_panel is not None:
            try:
                if hasattr(self._joint_panel, 'hardware_toggle'):
                    self._joint_panel.hardware_toggle.setChecked(False)
                if hasattr(self._joint_panel, 'spin_timer'):
                    self._joint_panel.spin_timer.stop()
                self._joint_node.destroy_node()
            except Exception as error:
                self._append_log('joint', f'释放关节界面时出现警告：{error}')
            empty = EmptyModulePage(
                '全身关节监控与动作控制',
                '模块已停止，关节控制已锁定。',
            )
            empty.start_requested.connect(self.start_joint)
            self.joint_host.set_content(empty)
            self._joint_panel = None
            self._joint_node = None
            self._joint_module = None

    def _shutdown_navigation_page(self):
        timer = self._navigation_spin_timer
        executor = self._navigation_executor
        node = self._navigation_node
        self._navigation_spin_timer = None
        self._navigation_executor = None
        self._navigation_node = None
        if timer is not None:
            timer.stop()
            try:
                timer.timeout.disconnect()
            except (TypeError, RuntimeError):
                pass
            timer.deleteLater()
        if executor is not None and node is not None:
            try:
                executor.remove_node(node)
                executor.shutdown(timeout_sec=1.0)
            except Exception:
                pass
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass

    def _shutdown_service_page(self):
        if self._service_panel is None:
            return
        try:
            self._service_panel.shutdown()
        except Exception as error:
            self._append_log('service', f'关闭服务管理页面时出现警告：{error}')

    def closeEvent(self, event):
        if self._closing:
            event.accept()
            return
        active = [key for key, process in self.processes.items() if process.running]
        if active:
            answer = QtWidgets.QMessageBox.question(
                self, '退出综合控制台',
                '退出将停止导航、建图和关节控制模块，并恢复机器人原后台服务。是否继续？',
            )
            if answer != QtWidgets.QMessageBox.Yes:
                event.ignore()
                return
        self._closing = True
        if self._joint_panel is not None:
            self.stop_joint()
        if self._slam_page is not None:
            self.stop_slam()
        self._shutdown_service_page()
        self.processes['navigation'].stop()
        self._shutdown_navigation_page()
        self._restore_vendor_stack()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = ControlCenterNode()
    application = QtWidgets.QApplication([sys.argv[0]])
    application.setApplicationName(APP_TITLE)
    lock = QtCore.QLockFile('/tmp/autolife_robot_control_center.lock')
    lock.setStaleLockTime(5000)
    if not lock.tryLock(100):
        QtWidgets.QMessageBox.warning(None, APP_TITLE, '综合控制台已经在运行。')
        node.destroy_node()
        rclpy.shutdown()
        return 2
    window = ControlCenterWindow(node)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal_timer = QtCore.QTimer()
    signal_timer.start(250)
    signal_timer.timeout.connect(lambda: None)
    window.show()
    try:
        return application.exec_()
    finally:
        if not window._closing:
            window._closing = True
            for process in window.processes.values():
                process.stop()
            window._shutdown_service_page()
            window._shutdown_navigation_page()
            window._restore_vendor_stack()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.unlock()


if __name__ == '__main__':
    raise SystemExit(main())
