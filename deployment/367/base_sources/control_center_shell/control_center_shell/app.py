#!/usr/bin/env python3
import importlib.util
import math
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time
import uuid

from ament_index_python.packages import get_package_prefix
import rclpy
from rclpy.node import Node
from PyQt5 import QtCore, QtGui, QtWidgets
from sensor_msgs.msg import BatteryState
from std_msgs.msg import String
from control_center_shell import preflight


APP_TITLE = '机器人综合控制台'
BATTERY_TIMEOUT_SECONDS = 3.5
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
QLabel#batteryState {
  background: #303844; color: #dce6f1; border-radius: 9px;
  padding: 3px 9px; font-size: 12px; font-weight: 700;
}
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


def _active_map_publishers(node):
    return preflight._map_publishers()



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
        self.declare_parameter('robot_id', 367)
        self.declare_parameter('topic_suffix', '0_367')
        self.declare_parameter(
            'battery_topic',
            f"/topic_gv_battery_{self.get_parameter('topic_suffix').value}",
        )
        self.declare_parameter('start_navigation', False)
        self._operator_session = uuid.uuid4().hex
        self._operator_heartbeat_pub = self.create_publisher(
            String, '/control_center/operator_heartbeat', 10
        )

    def publish_operator_heartbeat(self):
        self._operator_heartbeat_pub.publish(
            String(data=self._operator_session)
        )


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
        self.last_arguments = None

    @property
    def running(self):
        return self.process.state() != QtCore.QProcess.NotRunning

    def start(self, arguments):
        if self.running:
            return
        executable = shutil.which('ros2') or '/opt/ros/jazzy/bin/ros2'
        setsid = shutil.which('setsid')
        self.intentional_stop = False
        self.last_arguments = list(arguments)
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
        group_id = self.process_group_id
        self.intentional_stop = True
        self.output.emit(self.key, '正在停止模块……')
        if self.process_group_id is not None and os.name == 'posix':
            try:
                os.killpg(self.process_group_id, signal.SIGINT)
            except ProcessLookupError:
                pass
        else:
            self.process.terminate()
        if not self.process.waitForFinished(timeout_ms):
            self.output.emit(self.key, '模块未及时退出，向整个进程组发送 SIGTERM。')
            if self.process_group_id is not None and os.name == 'posix':
                try:
                    os.killpg(self.process_group_id, signal.SIGTERM)
                except ProcessLookupError:
                    pass
            else:
                self.process.terminate()
            if not self.process.waitForFinished(1800):
                self.output.emit(self.key, '进程组仍未退出，执行最终 SIGKILL。')
                if self.process_group_id is not None and os.name == 'posix':
                    try:
                        os.killpg(self.process_group_id, signal.SIGKILL)
                    except ProcessLookupError:
                        pass
                else:
                    self.process.kill()
                self.process.waitForFinished(1000)
        # ros2 launch may exit before its children. Keep the original PGID
        # locally because _on_finished clears self.process_group_id.
        if group_id is not None and os.name == 'posix':
            for stop_signal, timeout_sec in ((signal.SIGTERM, 1.0),
                                             (signal.SIGKILL, 0.5)):
                members = [p for p in preflight._processes().values()
                           if p['pgid'] == group_id]
                if not members:
                    break
                self.output.emit(self.key, f'清理残留进程组 {group_id}：{len(members)} 个进程')
                try:
                    os.killpg(group_id, stop_signal)
                except ProcessLookupError:
                    break
                deadline = time.monotonic() + timeout_sec
                while time.monotonic() < deadline:
                    if not any(p['pgid'] == group_id
                               for p in preflight._processes().values()):
                        break
                    time.sleep(0.05)
            if any(p['pgid'] == group_id for p in preflight._processes().values()):
                self.output.emit(self.key, f'残留进程组 {group_id} 未完全退出；下次启动将自检。')
            else:
                preflight.forget_process(self.key)

    def _on_started(self):
        # The process is started through `setsid`, so its PID is also the
        # session/process-group id. Signalling the group is essential: killing
        # only `ros2 launch` can orphan Nav2, Cartographer or joint nodes.
        self.process_group_id = int(self.process.processId())
        QtCore.QTimer.singleShot(
            100, lambda key=self.key, pid=self.process_group_id:
            self._record_process_group(key, pid))
        self.state_changed.emit(self.key, 'running')

    def _record_process_group(self, key, pid):
        try:
            preflight.record_process(key, pid)
        except OSError as error:
            self.output.emit(key, f'无法保存进程组记录：{error}')

    def _read_output(self):
        raw = bytes(self.process.readAllStandardOutput())
        text = raw.decode('utf-8', errors='replace').rstrip()
        if text:
            self.output.emit(self.key, text)

    def _on_error(self, error):
        self.output.emit(self.key, f'进程错误：{error}')
        self.state_changed.emit(self.key, 'error')

    def _on_finished(self, exit_code, _status):
        # A failed launch can leave already-started watchdogs alive. QProcess
        # only tracks the launch parent, so clean its isolated group as well.
        group_id = self.process_group_id
        if not self.intentional_stop and group_id is not None:
            self._signal_orphan_group(group_id, signal.SIGTERM)
            QtCore.QTimer.singleShot(
                1500, lambda gid=group_id: self._signal_orphan_group(gid, signal.SIGKILL))
        if self.intentional_stop or exit_code == 0:
            state = 'stopped'
        else:
            state = 'error'
            self.output.emit(self.key, f'模块异常退出，退出码 {exit_code}')
        self.state_changed.emit(self.key, state)
        self.process_group_id = None


    def _signal_orphan_group(self, group_id, stop_signal):
        if os.name != 'posix' or group_id == os.getpgrp():
            return
        # setsid creates an owned session with this same PGID. Only consider
        # descendants left after its leader exited, never a new live leader.
        processes = preflight._processes()
        if group_id in processes:
            return
        if not any(p['pgid'] == group_id for p in processes.values()):
            return
        try:
            os.killpg(group_id, stop_signal)
            self.output.emit(self.key, f'清理退出后残留的模块进程组 {group_id}')
        except (ProcessLookupError, PermissionError) as error:
            self.output.emit(self.key, f'清理模块残留进程：{error}')


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


class PreflightWorker(QtCore.QThread):
    progress = QtCore.pyqtSignal(str)
    completed = QtCore.pyqtSignal(str, str)

    def __init__(self, suffix, parent=None, takeover=False, active_groups=None):
        super().__init__(parent)
        self.suffix = suffix
        self.takeover = bool(takeover)
        self.active_groups = {
            key: int(pid) for key, pid in (active_groups or {}).items() if pid
        }

    def run(self):
        def emit(message):
            print('[preflight] ' + message, flush=True)
            self.progress.emit(message)
        try:
            result, detail = preflight.run(
                self.suffix, emit=emit, takeover=self.takeover,
                active_groups=self.active_groups or None)
        except Exception as error:
            result, detail = 'FAIL', f'启动自检异常：{error}'
            emit('FAIL ' + detail)
        emit(f'result={result} detail={detail}')
        self.completed.emit(result, detail)




class StatusCheckWorker(QtCore.QThread):
    progress = QtCore.pyqtSignal(str)
    completed = QtCore.pyqtSignal(str, str)

    def __init__(self, suffix, active_groups, parent=None):
        super().__init__(parent)
        self.suffix = suffix
        self.active_groups = dict(active_groups)

    def run(self):
        def emit(message):
            print('[status-check] ' + message, flush=True)
            self.progress.emit(message)
        try:
            result, detail = preflight.check_only(
                self.suffix, active_groups=self.active_groups, emit=emit
            )
        except Exception as error:
            result, detail = 'WARN', f'状态自检异常：{error}'
            emit('WARN ' + detail)
        emit(f'result={result} detail={detail}')
        self.completed.emit(result, detail)


class ModuleHealthWorker(QtCore.QThread):
    progress = QtCore.pyqtSignal(str)
    completed = QtCore.pyqtSignal(str, str)

    def __init__(self, module, suffix, group_id, parent=None, active_groups=None):
        super().__init__(parent)
        self.module = module
        self.suffix = suffix
        self.group_id = int(group_id)
        self.active_groups = dict(active_groups or {})

    def run(self):
        def emit(message):
            print('[service-repair] ' + message, flush=True)
            self.progress.emit(message)
        error = preflight.check_module_health(
            self.module, self.group_id, timeout=12.0, emit=emit, active_groups=self.active_groups
        )
        if error:
            emit(f'{self.module}: FAIL {error}')
            cleaned = preflight.stop_managed_group(
                self.module, self.group_id, emit=emit
            )
            if not cleaned:
                error += '；失败进程组未能安全清理'
            self.completed.emit('FAIL', error)
            return
        self.completed.emit('PASS', '模块关键进程与地图输出正常')


class ResourceWorker(QtCore.QThread):
    completed = QtCore.pyqtSignal(str, str)
    progress = QtCore.pyqtSignal(str)

    def __init__(self, mode, suffix, groups, parent):
        super().__init__(parent)
        self.mode, self.suffix, self.groups = mode, suffix, groups

    def run(self):
        try:
            if self.mode == 'plan':
                self.completed.emit('PLAN', preflight.repair_plan(self.suffix, self.groups))
            elif self.mode == 'restore':
                self.completed.emit(*preflight.restore_paused(emit=self.progress.emit))
            else:
                snapshot = (preflight.module_snapshot(self.mode.split(':', 1)[1], self.suffix, self.groups)
                            if self.mode.startswith('start:') else
                            preflight.resource_snapshot(self.suffix, self.groups))
                self.completed.emit('PASS' if snapshot.ok else 'WARN', snapshot.detail)
        except Exception as error:
            self.completed.emit('WARN', f'资源状态无法确认：{error}')


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
        self._preflight_ready = False
        self._preflight_worker = None
        self._service_repair_worker = None
        self._status_check_worker = None
        self._status_check_for_repair = False
        self._module_health_worker = None
        self._resource_worker = self._plan_worker = self._restore_worker = None
        self._launch_guard_worker = None
        self._final_check_worker = None
        self._repair_transaction = False
        self._repair_results = []
        self._resource_timer = QtCore.QTimer(self)
        self._resource_timer.setInterval(3000)
        self._resource_timer.timeout.connect(self._resource_watch)
        self._resource_timer.start()
        self._last_preflight_failure = ''
        self._service_repair_target = None
        self._service_repair_arguments = None
        self._repair_prior_preflight_ready = False
        self._service_repair_restart = None
        self._repair_stop_wait_count = 0
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
        self._battery_percentage = None
        self._battery_received_at = 0.0
        self._battery_display_text = ''
        self._battery_last_warning_at = -math.inf

        self.processes = {
            key: ManagedProcess(key, self)
            for key in ('navigation', 'slam', 'joint')
        }
        for process in self.processes.values():
            process.output.connect(self._append_log)
            process.state_changed.connect(self._process_state_changed)

        self._build_ui()
        self._battery_subscription = self.node.create_subscription(
            BatteryState,
            str(self.node.get_parameter('battery_topic').value),
            self._on_battery_state,
            10,
        )
        self._shell_spin_timer = QtCore.QTimer(self)
        self._shell_spin_timer.setInterval(200)
        self._shell_spin_timer.timeout.connect(self._drain_shell_callbacks)
        self._shell_spin_timer.start()
        self._battery_timeout_timer = QtCore.QTimer(self)
        self._battery_timeout_timer.setInterval(1000)
        self._battery_timeout_timer.timeout.connect(self._refresh_battery_label)
        self._battery_timeout_timer.start()
        self._refresh_battery_label()
        # This timer intentionally runs on the Qt main event loop.  A GUI
        # crash or frozen event loop therefore stops the heartbeat even when
        # a background ROS/thread worker is still alive.
        self._operator_heartbeat_timer = QtCore.QTimer(self)
        self._operator_heartbeat_timer.setInterval(100)
        self._operator_heartbeat_timer.timeout.connect(
            self._publish_operator_heartbeat
        )
        self._operator_heartbeat_timer.start()
        self.centralWidget().setEnabled(False)
        self.current_state_label.setText('启动自检中')
        self.current_state_label.setObjectName('stateStarting')
        self.statusBar().showMessage('正在检查控制环境...')

    def _publish_operator_heartbeat(self):
        if self._preflight_ready and not self._closing:
            self.node.publish_operator_heartbeat()

    def start_preflight(self):
        if self._preflight_worker is not None and self._preflight_worker.isRunning():
            return
        if (self._service_repair_worker is not None
                and self._service_repair_worker.isRunning()):
            return
        if (self._status_check_worker is not None
                and self._status_check_worker.isRunning()):
            return
        if (self._module_health_worker is not None
                and self._module_health_worker.isRunning()):
            return
        active_modules = [
            key for key, process in self.processes.items() if process.running
        ]
        if active_modules:
            detail = '先停止正在运行的模块再重新自检：' + '、'.join(active_modules)
            self.statusBar().showMessage(detail)
            self._append_log('preflight', detail)
            return
        if self._preflight_worker is not None:
            self._preflight_worker.deleteLater()
        self._preflight_ready = False
        self.centralWidget().setEnabled(False)
        self.preflight_failure_label.hide()
        self.preflight_repair_button.hide()
        self.statusBar().showMessage('正在检查控制环境...')
        self._preflight_worker = PreflightWorker(self.topic_suffix, self)
        self._preflight_worker.progress.connect(
            lambda message: self._append_log('preflight', message))
        self._preflight_worker.completed.connect(self._preflight_complete)
        self._preflight_worker.start()

    def _preflight_complete(self, result, detail):
        if self._closing:
            return
        if result in ('FAIL', 'WARN'):
            self._last_preflight_failure = detail
            self._preflight_ready = True
            self.centralWidget().setEnabled(True)
            for host in (self.nav_host, self.slam_host, self.joint_host):
                host.setEnabled(True)
            self._refresh_compact_controls()
            self.current_state_label.setText('自检有警告')
            self.current_state_label.setObjectName('stateError')
            self.current_state_label.style().unpolish(self.current_state_label)
            self.current_state_label.style().polish(self.current_state_label)
            self.statusBar().showMessage(
                '启动自检发现问题，控制中心仍可使用：' + detail
            )
            self.preflight_failure_label.setText(
                '启动自检发现问题：' + detail
                + '。控制中心其他功能仍可使用；若目标模块存在资源占用，'
                '该模块会单独提示并拒绝重复启动。点击“修复”处理地图占用，'
                '其他问题可前往“系统服务 → 服务状态修复”。'
            )
            self.preflight_retry_button.setEnabled(True)
            self.preflight_retry_button.show()
            self.preflight_repair_button.setEnabled(True)
            self.preflight_repair_button.show()
            self.preflight_failure_label.show()
            if self._service_panel is not None:
                self._service_panel.set_service_repair_context(detail)
            if self.start_navigation_on_open:
                QtCore.QTimer.singleShot(350, self.start_navigation)
            return
        self._last_preflight_failure = ''
        self._preflight_ready = True
        self.centralWidget().setEnabled(True)
        for host in (self.nav_host, self.slam_host, self.joint_host):
            host.setEnabled(True)
        self.preflight_retry_button.hide()
        self.preflight_repair_button.hide()
        if self._service_panel is not None:
            self._service_panel.set_service_repair_context('')
        self._refresh_compact_controls()
        self._load_navigation_ui()
        self.statusBar().showMessage(
            f'机器人 {self.robot_id} · 自检 {result} · 话题后缀 {self.topic_suffix}')
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
        self._status_tab = self.tabs.addTab('系统状态')
        self._log_tab = self.tabs.addTab('运行日志')
        self.tabs.currentChanged.connect(self._tab_changed)
        browser_layout.addWidget(self.tabs)
        browser_layout.addStretch(1)
        self.battery_label = QtWidgets.QLabel('电量：--')
        self.battery_label.setObjectName('batteryState')
        browser_layout.addWidget(self.battery_label)
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
        self.preflight_retry_button = QtWidgets.QPushButton('重新自检')
        self.preflight_retry_button.clicked.connect(self.start_preflight)
        self.preflight_retry_button.hide()
        browser_layout.addWidget(self.preflight_retry_button)
        self.preflight_repair_button = QtWidgets.QPushButton('前往服务状态修复')
        self.preflight_repair_button.setObjectName('successButton')
        self.preflight_repair_button.clicked.connect(self._show_service_repair)
        self.preflight_repair_button.hide()
        browser_layout.addWidget(self.preflight_repair_button)
        layout.addWidget(browser_bar)

        self.preflight_failure_label = QtWidgets.QLabel()
        self.preflight_failure_label.setWordWrap(True)
        self.preflight_failure_label.setStyleSheet(
            'background: #48282b; color: #f19a9f; padding: 10px;')
        self.preflight_failure_label.hide()
        layout.addWidget(self.preflight_failure_label)

        self.stack = QtWidgets.QStackedWidget()
        layout.addWidget(self.stack, 1)

        nav_empty = EmptyModulePage(
            '正在准备导航界面', '导航是综合控制台的默认主功能。'
        )
        nav_empty.start_requested.connect(self.start_navigation)
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
        self._status_host = QtWidgets.QWidget()
        QtWidgets.QVBoxLayout(self._status_host).setContentsMargins(0, 0, 0, 0)
        self._status_page = None
        self.stack.addWidget(self._status_host)
        self.stack.addWidget(self.log_view)
        self._refresh_compact_controls()

    def _drain_shell_callbacks(self):
        if self._closing or not rclpy.ok():
            return
        try:
            rclpy.spin_once(self.node, timeout_sec=0.0)
        except Exception as error:
            self._shell_spin_timer.stop()
            self._append_log('battery', f'电量 ROS 回调已停止：{error}')

    def _on_battery_state(self, message):
        raw = getattr(message, 'percentage', None)
        try:
            percentage = float(raw)
        except (TypeError, ValueError, OverflowError):
            percentage = math.nan
        now = time.monotonic()
        if not math.isfinite(percentage) or not 0.0 <= percentage <= 100.0:
            self._battery_percentage = None
            self._battery_received_at = 0.0
            if now - self._battery_last_warning_at >= 10.0:
                self.node.get_logger().warning(
                    f'Ignoring invalid battery percentage: {raw!r}'
                )
                self._battery_last_warning_at = now
        else:
            self._battery_percentage = int(round(percentage))
            self._battery_received_at = now
        self._refresh_battery_label()

    def _refresh_battery_label(self):
        fresh = (
            self._battery_percentage is not None
            and time.monotonic() - self._battery_received_at <= BATTERY_TIMEOUT_SECONDS
        )
        text = f'电量：{self._battery_percentage}%' if fresh else '电量：--'
        if text != self._battery_display_text:
            self._battery_display_text = text
            self.battery_label.setText(text)

    def _tab_changed(self, index):
        self.stack.setCurrentIndex(index)
        self._refresh_compact_controls()
        if index == 0 and self._navigation_window is not None:
            self._navigation_window.prepare_for_user_interaction()
        if index == 3:
            self._load_service_manager_ui()
        if index == self._status_tab:
            self._load_system_status()
        if index == self._log_tab:
            self.log_view.moveCursor(QtGui.QTextCursor.End)

    def _load_system_status(self):
        layout = self._status_host.layout()
        if layout.count():
            return
        try:
            from control_center_shell.system_status import SystemStatusPage
            self._status_page = SystemStatusPage(self._status_host)
            layout.addWidget(self._status_page)
        except Exception as error:
            layout.addWidget(QtWidgets.QLabel('系统状态暂不可用。'))
            self._append_log('system-status', f'加载失败：{type(error).__name__}')

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
        self.current_start_button.setEnabled(
            self._preflight_ready and state in ('stopped', 'error'))
        self.current_stop_button.setEnabled(
            self._preflight_ready and state in ('running', 'starting'))

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
        if self._service_panel is not None and not self._last_preflight_failure:
            detail, button_text = self._service_repair_context()
            self._service_panel.set_service_repair_context(detail, button_text)
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
            panel.repair_requested.connect(self._start_service_repair)
            panel.restore_requested.connect(self._restore_services)
            panel.status_check_requested.connect(self._start_status_check)
            detail, button_text = self._service_repair_context()
            panel.set_service_repair_context(detail, button_text)
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

    def _show_service_repair(self):
        self.tabs.setCurrentIndex(3)
        if self._load_service_manager_ui() and self._service_panel is not None:
            detail, button_text = self._service_repair_context()
            self._service_panel.set_service_repair_context(detail, button_text)




    def _resource_busy(self):
        workers = (self._preflight_worker, self._service_repair_worker,
                   self._status_check_worker, self._module_health_worker,
                   self._resource_worker, self._plan_worker, self._restore_worker,
                   self._final_check_worker, self._launch_guard_worker)
        return any(w is not None and w.isRunning() for w in workers)

    def _active_resource_groups(self):
        groups = {key: p.process_group_id for key, p in self.processes.items()
                  if p.running and p.process_group_id}
        tokens = []
        for obj in (self._navigation_node, self._slam_page):
            lease = getattr(obj, '_chassis_lease', None)
            token = getattr(lease, '_token', None)
            if token:
                tokens.append(token)
        preflight.set_local_lease_tokens(tokens)
        return groups

    def _replace_resource_worker(self, name, worker):
        old = getattr(self, name, None)
        if old is not None:
            old.deleteLater()
        setattr(self, name, worker)

    def start_navigation(self):
        self._request_module_start('navigation')

    def start_slam(self):
        self._request_module_start('slam')

    def start_joint(self):
        self._request_module_start('joint')

    def _request_module_start(self, module):
        if self._closing or not self._preflight_ready or self._resource_busy() or self._repair_transaction:
            self.statusBar().showMessage('正在检查资源，请稍后启动模块')
            return
        worker = ResourceWorker('start:' + module, self.topic_suffix, self._active_resource_groups(), self)
        self._replace_resource_worker('_launch_guard_worker', worker)
        worker.completed.connect(lambda result, detail: self._launch_guard_done(module, result, detail))
        worker.start()

    def _launch_guard_done(self, module, result, detail):
        if self._closing:
            return
        if result != 'PASS':
            QtWidgets.QMessageBox.warning(self, '资源尚不能接管', detail)
            return
        getattr(self, '_start_' + module + '_unchecked')()

    def _start_service_repair(self):
        if self._closing or self._resource_busy():
            if self._service_panel is not None:
                self._service_panel.finish_service_repair('WARN', '其他检查正在运行，请稍后重试')
            return
        self._replace_resource_worker('_plan_worker',
            ResourceWorker('plan', self.topic_suffix, self._active_resource_groups(), self))
        self._plan_worker.completed.connect(self._repair_plan_complete)
        self._plan_worker.start()

    def _repair_plan_complete(self, result, detail):
        if result != 'PLAN':
            self._service_panel.finish_service_repair(result, detail)
            return
        answer = QtWidgets.QMessageBox.question(
            self, '确认资源修复计划', detail,
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer == QtWidgets.QMessageBox.Yes:
            QtCore.QTimer.singleShot(100, self._begin_service_repair)
        elif self._service_panel is not None:
            self._service_panel.finish_service_repair('WARN', '用户取消；未执行服务停止')

    def _execute_service_repair(self):
        try:
            self._execute_service_repair_impl()
        except Exception as error:
            if self._service_panel is not None:
                self._service_panel.finish_service_repair('WARN', f'修复启动失败：{error}')
            self.centralWidget().setEnabled(True)
            self._preflight_ready = True
        finally:
            if self._service_repair_worker is None or not self._service_repair_worker.isRunning():
                self._repair_transaction = False
                if self._service_panel is not None and self._service_panel._repair_busy:
                    self._service_panel.finish_service_repair('WARN', '修复未启动，请等待当前检查结束后重试')

    def _resource_watch(self):
        if self._closing or self._repair_transaction or self._resource_busy():
            return
        try:
            observe = (self.tabs.currentIndex() == 3 or
                       any(p.running for p in self.processes.values()) or bool(preflight.paused_services()))
        except Exception:
            observe = True
        if not observe:
            return
        self._replace_resource_worker('_resource_worker',
            ResourceWorker('watch', self.topic_suffix, self._active_resource_groups(), self))
        self._resource_worker.completed.connect(self._resource_observed)
        self._resource_worker.start()

    def _resource_observed(self, result, detail):
        if self._closing:
            return
        if result == 'PASS':
            if getattr(self, '_resource_recheck_needed', False):
                self._resource_recheck_needed = False
                # Clear stale red banners only after a full read-only check.
                QtCore.QTimer.singleShot(150, self._start_status_check)
            return
        self._resource_recheck_needed = True
        message = '资源状态已变化：' + detail
        if message != self._last_preflight_failure:
            self._append_log('resource-watch', message)
        self._last_preflight_failure = message
        self.preflight_failure_label.setText(message + '。请重新自检；未自动终止外部程序。')
        self.preflight_failure_label.show()
        self.preflight_retry_button.show()
        self.preflight_repair_button.show()
        self.statusBar().showMessage(message)
        if self._service_panel is not None:
            self._service_panel.finish_status_check('WARN', message)
            self._service_panel.set_service_repair_context(message)

    def _restore_services(self):
        if self._resource_busy() or self._repair_transaction or any(p.running for p in self.processes.values()):
            QtWidgets.QMessageBox.warning(self, '暂不能恢复', '请先停止控制模块，并等待当前检查结束。')
            return
        try:
            entries = preflight.paused_services()
        except Exception as error:
            QtWidgets.QMessageBox.warning(self, '记录不可读取', str(error))
            return
        if not entries:
            QtWidgets.QMessageBox.information(self, '恢复被暂停服务', '本次没有暂停服务。')
            return
        names = '\n'.join(f"{e['unit']['manager']}/{e['unit']['name']}" for e in entries)
        answer = QtWidgets.QMessageBox.question(
            self, '恢复被暂停服务',
            names + '\n\n服务自身可能自动启动任务。是否恢复？恢复后将重新自检。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self._replace_resource_worker('_restore_worker', ResourceWorker('restore', self.topic_suffix, {}, self))
        self._restore_worker.progress.connect(lambda message: self._append_log('restore', message))
        self._restore_worker.completed.connect(self._services_restored)
        self._restore_worker.finished.connect(lambda: QtCore.QTimer.singleShot(100, self._start_status_check))
        self._service_panel.set_service_repair_progress('正在恢复本次暂停的服务……')
        self._restore_worker.start()

    def _services_restored(self, result, detail):
        if self._service_panel is not None:
            self._service_panel.finish_service_repair(result, detail)
        self._append_log('restore', detail)

    def _module_repair_complete(self, result, detail):
        self._repair_results.append((result, detail))
        if self._service_repair_restart:
            self._service_repair_target = None
            QtCore.QTimer.singleShot(250, self._restart_next_repaired_module)
            return
        self._replace_resource_worker('_final_check_worker', StatusCheckWorker(
            self.topic_suffix, self._active_resource_groups(), self))
        self._final_check_worker.completed.connect(self._final_repair_checked)
        self._final_check_worker.start()

    def _final_repair_checked(self, result, detail):
        failures = [text for status, text in self._repair_results if status != 'PASS']
        if result != 'PASS':
            failures.append(detail)
        self._repair_results = []
        self._finish_module_repair('WARN' if failures else 'PASS', '；'.join(failures) or detail)

    def _service_repair_context(self):
        if self._last_preflight_failure:
            return self._last_preflight_failure, '修复服务状态'
        navigation = self.processes['navigation']
        slam = self.processes['slam']
        if navigation.running or (
                self.nav_host.state == 'error' and navigation.last_arguments):
            return ('导航模块运行中：点击修复会自动停止并按原参数重启导航，'
                    '同时收回被外部程序占用的地图/底盘资源。'), '修复服务状态'
        if slam.running or (
                self.slam_host.state == 'error' and slam.last_arguments):
            return ('建图模块运行中：点击修复会自动停止并按原参数重启建图，'
                    '同时收回被外部程序占用的地图/底盘资源。'), '修复服务状态'
        return '', '修复服务状态'

    def _select_service_repair_target(self):
        active = [
            key for key in ('navigation', 'slam')
            if self.processes[key].running
        ]
        if len(active) > 1:
            return None, '导航和建图同时运行，拒绝自动重建，请先停止其中一个模块'
        if active:
            return active[0], ''
        candidates = []
        if (self.nav_host.state == 'error'
                and self.processes['navigation'].last_arguments):
            candidates.append('navigation')
        if (self.slam_host.state == 'error'
                and self.processes['slam'].last_arguments):
            candidates.append('slam')
        if len(candidates) == 1:
            return candidates[0], ''
        return None, ''

    def _start_status_check(self, repair_probe=False):
        if self._closing:
            return
        self._active_resource_groups()
        for worker in (
            self._resource_worker, self._plan_worker, self._launch_guard_worker,
            self._restore_worker, self._final_check_worker,
            self._preflight_worker,
            self._service_repair_worker,
            self._status_check_worker,
            self._module_health_worker,
        ):
            if worker is not None and worker.isRunning():
                QtCore.QTimer.singleShot(200, lambda: self._start_status_check(repair_probe))
                return
        if self._status_check_worker is not None:
            self._status_check_worker.deleteLater()
        active_groups = {}
        for key, process in self.processes.items():
            if not process.running:
                continue
            group_id = process.process_group_id
            if group_id is None:
                process_id = process.process.processId()
                group_id = int(process_id) if process_id else None
            if group_id:
                active_groups[key] = int(group_id)
        self._status_check_for_repair = bool(repair_probe)
        if self._service_panel is not None:
            if repair_probe:
                self._service_panel.set_service_repair_progress(
                    '检测到控制模块仍在运行，先执行只读状态检查；正常时不重启。'
                )
            else:
                self._service_panel.set_status_check_progress(
                    '正在只读检查控制进程、服务、双雷达及地图发布者……'
                )
        self._append_log('status-check', '开始执行只读状态自检。')
        self._status_check_worker = StatusCheckWorker(
            self.topic_suffix, active_groups, self
        )
        self._status_check_worker.progress.connect(
            lambda message: self._append_log('status-check', message)
        )
        self._status_check_worker.completed.connect(
            self._status_check_complete
        )
        self._status_check_worker.start()

    def _status_check_complete(self, result, detail):
        if self._closing:
            return
        if self._status_check_for_repair:
            self._status_check_for_repair = False
            if result == 'PASS':
                self._repair_transaction = False
                message = detail + '；当前控制模块保持运行，未执行重启。'
                self._clear_preflight_warning()
                self.statusBar().showMessage('状态正常，无需修复')
                if self._service_panel is not None:
                    self._service_panel.set_service_repair_context('')
                    self._service_panel.finish_service_repair('PASS', message)
                return
            # One-click repair: stop own navigation/mapping modules, run the
            # full takeover repair, then restart them with the same arguments.
            restart = {}
            for key in ('navigation', 'slam'):
                process = self.processes[key]
                if process.running and process.last_arguments:
                    restart[key] = list(process.last_arguments)
            stopped_any = False
            for key in ('navigation', 'slam'):
                if self.processes[key].running:
                    if key == 'navigation':
                        self.stop_navigation()
                    else:
                        self.stop_slam()
                    stopped_any = True
            self._service_repair_restart = restart or None
            if self._service_panel is not None:
                self._service_panel.set_service_repair_progress(
                    '正在停止本控制中心的导航/建图模块，随后执行修复并自动重启……'
                )
            self._append_log(
                'service-repair',
                '检测到问题（' + detail + '）；停止本中心模块后执行一键修复。')
            if not stopped_any:
                self._execute_service_repair()
                return
            self._repair_stop_wait_count = 0
            QtCore.QTimer.singleShot(300, self._wait_repair_module_stop)
            return

        self._last_preflight_failure = detail if result != 'PASS' else ''
        if result == 'PASS':
            self._clear_preflight_warning()
        else:
            self.preflight_failure_label.setText('状态自检发现问题：' + detail)
            self.preflight_failure_label.show()
            self.preflight_repair_button.show()
        if self._service_panel is not None:
            self._service_panel.finish_status_check(result, detail)
            self._service_panel.set_service_repair_context(
                self._last_preflight_failure
            )
        self._append_log('status-check', f'{result}: {detail}')
        self.statusBar().showMessage(
            '状态自检通过' if result == 'PASS' else '状态自检发现问题：' + detail
        )

    def _clear_preflight_warning(self):
        self._last_preflight_failure = ''
        self.preflight_failure_label.hide()
        self.preflight_retry_button.hide()
        self.preflight_repair_button.hide()
        self._refresh_compact_controls()

    def _wait_repair_module_stop(self):
        if self._closing:
            return
        if any(self.processes[key].running for key in ('navigation', 'slam')):
            self._repair_stop_wait_count += 1
            if self._repair_stop_wait_count >= 50:
                self._repair_transaction = False
                detail = '控制模块未能及时停止，已取消本次修复；可手动停止模块后重试。'
                if self._service_panel is not None:
                    self._service_panel.finish_service_repair('FAIL', detail)
                self.statusBar().showMessage(detail)
                return
            QtCore.QTimer.singleShot(300, self._wait_repair_module_stop)
            return
        self._execute_service_repair()

    def _begin_service_repair(self):
        self._repair_transaction = True
        self._repair_results = []
        active_modules = [
            key for key in ('navigation', 'slam', 'joint')
            if self.processes[key].running
        ]
        if active_modules:
            # Healthy modules are not restarted needlessly; the read-only
            # probe decides whether the full repair has to run.
            self._start_status_check(repair_probe=True)
            return
        self._execute_service_repair()

    def _execute_service_repair_impl(self):
        target, target_error = self._select_service_repair_target()
        if target_error:
            if self._service_panel is not None:
                self._service_panel.finish_service_repair(
                    'FAIL', target_error
                )
            return
        blocking = [key for key in ('navigation', 'slam')
                    if self.processes[key].running and key != target]
        if blocking:
            detail = (
                '为避免修复中断正在运行的模块，请先停止：'
                + '、'.join(blocking)
            )
            if self._service_panel is not None:
                self._service_panel.finish_service_repair('FAIL', detail)
            self.statusBar().showMessage(detail)
            return
        if self._preflight_worker is not None and self._preflight_worker.isRunning():
            return
        if (self._status_check_worker is not None
                and self._status_check_worker.isRunning()):
            return
        if (self._service_repair_worker is not None
                and self._service_repair_worker.isRunning()):
            return
        if (self._module_health_worker is not None
                and self._module_health_worker.isRunning()):
            return
        if self._service_repair_worker is not None:
            self._service_repair_worker.deleteLater()
        self._service_repair_target = target
        self._service_repair_arguments = (
            list(self.processes[target].last_arguments)
            if target is not None else None
        )
        self._repair_prior_preflight_ready = self._preflight_ready
        self._preflight_ready = False
        for host in (self.nav_host, self.slam_host, self.joint_host):
            host.setEnabled(False)
        target_name = {'navigation': '导航', 'slam': '建图'}.get(target, '服务')
        if self._service_panel is not None:
            self._service_panel.set_service_repair_progress(
                f'正在检查并修复{target_name}相关服务和进程……'
            )
        self.statusBar().showMessage(f'正在执行{target_name}状态修复...')
        active_groups = {
            key: process.process_group_id
            for key, process in self.processes.items()
            if process.running and process.process_group_id
        }
        self._service_repair_worker = PreflightWorker(
            self.topic_suffix, self, takeover=True, active_groups=active_groups)
        self._service_repair_worker.progress.connect(
            lambda message: self._append_log('service-repair', message)
        )
        self._service_repair_worker.completed.connect(
            self._service_repair_complete
        )
        self._service_repair_worker.start()

    def _service_repair_complete(self, result, detail):
        if self._closing:
            return
        restart = self._service_repair_restart or {}
        target = self._service_repair_target
        if target in ('navigation', 'slam'):
            process = self.processes[target]
            if (process.last_arguments and target not in restart
                    and not process.running):
                restart[target] = list(process.last_arguments)
        restart_allowed = result in ('PASS', 'AUTO_RECOVERED')
        if restart_allowed and restart:
            self._service_repair_restart = restart
            QtCore.QTimer.singleShot(250, self._restart_next_repaired_module)
            return
        self._repair_transaction = False
        self._service_repair_restart = None
        if self._service_panel is not None:
            self._service_panel.finish_service_repair(result, detail)
        self._preflight_ready = True
        self.centralWidget().setEnabled(True)
        for host in (self.nav_host, self.slam_host, self.joint_host):
            host.setEnabled(True)
        self._refresh_compact_controls()
        if result in ('PASS', 'AUTO_RECOVERED'):
            self._last_preflight_failure = ''
            self.preflight_repair_button.hide()
            self.preflight_retry_button.hide()
            self.preflight_failure_label.hide()
            self.statusBar().showMessage('服务状态检查/修复完成，控制中心可用')
        else:
            self._last_preflight_failure = detail
            self.preflight_failure_label.setText(
                '服务状态修复未完成：' + detail
                + '。控制中心仍可使用；可在“系统服务”页面查看后重试。'
            )
            self.preflight_failure_label.show()
            self.preflight_retry_button.show()
            self.preflight_repair_button.show()
            self.statusBar().showMessage('服务状态修复未完成：' + detail)

    def _restart_next_repaired_module(self):
        restart = self._service_repair_restart or {}
        if not restart:
            self._module_repair_complete('PASS', '无需重启模块')
            return
        target = next(iter(restart))
        arguments = restart.pop(target)
        self._service_repair_restart = restart or None
        self._service_repair_target = target
        self._service_repair_arguments = arguments
        process = self.processes.get(target)
        target_name = {'navigation': '导航', 'slam': '建图'}.get(target, '模块')
        if process is None or not arguments:
            self._module_repair_complete('FAIL', f'{target_name}启动参数不可用')
            return
        if process.running:
            self._module_repair_complete('FAIL', f'{target_name}旧进程组未完全退出')
            return
        if not self._pause_vendor_stack():
            self._module_repair_complete(
                'FAIL', f'{target_name}接管厂商服务失败，模块未重启')
            return
        host = self.nav_host if target == 'navigation' else self.slam_host
        if not self._require_unowned_map(target, host):
            self._module_repair_complete(
                'FAIL', f'{target_name}启动前 /map 仍被占用，模块未重启')
            return
        if self._service_panel is not None:
            self._service_panel.set_service_repair_progress(
                f'正在重新启动{target_name}并验证关键进程和 /map……'
            )
        self._append_log('service-repair', f'使用原启动参数重新启动{target_name}模块。')
        process.start(arguments)
        QtCore.QTimer.singleShot(700, self._start_module_health_check)

    def _start_module_health_check(self):
        target = self._service_repair_target
        process = self.processes.get(target)
        target_name = {'navigation': '导航', 'slam': '建图'}.get(target, '模块')
        if process is None or not process.running or not process.process_group_id:
            self._module_repair_complete('FAIL', f'{target_name}进程未能启动')
            return
        self._module_health_worker = ModuleHealthWorker(
            target, self.topic_suffix, process.process_group_id, self,
            active_groups=self._active_resource_groups()
        )
        self._module_health_worker.progress.connect(
            lambda message: self._append_log('service-repair', message)
        )
        self._module_health_worker.completed.connect(
            self._module_repair_complete
        )
        self._module_health_worker.start()

    def _finish_module_repair(self, result, detail):
        self._repair_transaction = False
        if self._service_repair_restart:
            # More modules are waiting to be restarted after this one.
            self._service_repair_target = None
            QtCore.QTimer.singleShot(250, self._restart_next_repaired_module)
            return
        target = self._service_repair_target
        target_name = {'navigation': '导航', 'slam': '建图'}.get(target, '模块')
        success = result == 'PASS'
        message = (
            f'{target_name}状态修复完成，关键进程和地图输出已恢复'
            if success else f'{target_name}状态修复未完成：{detail}'
        )
        if self._service_panel is not None:
            self._service_panel.finish_service_repair(result, message)
        self.statusBar().showMessage(message)
        self._last_preflight_failure = '' if success else message
        self._preflight_ready = True
        self.centralWidget().setEnabled(True)
        for host in (self.nav_host, self.slam_host, self.joint_host):
            host.setEnabled(True)
        self._refresh_compact_controls()
        if success:
            self.preflight_failure_label.hide()
            self.preflight_retry_button.hide()
            self.preflight_repair_button.hide()
        else:
            self.preflight_failure_label.setText(
                message + '。控制中心仍可使用；可再次执行“服务状态修复”或点击“重新自检”。'
            )
            self.preflight_failure_label.show()
            self.preflight_retry_button.show()
            self.preflight_repair_button.show()
        self._service_repair_target = None
        self._service_repair_arguments = None

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
        try:
            if any(e['unit']['name'] == VENDOR_SLAM_SERVICE for e in preflight.paused_services()):
                self._append_log('system', '厂商导航保持本次暂停状态；请使用“恢复被暂停服务”。')
                self._vendor_paused = False
                return
        except Exception as error:
            self._append_log('system', f'暂停记录无法确认，未自动启动厂商导航：{error}')
            return
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
            node = module.NavigationRosNode(
                manual_cmd_vel_topic='/control_center/manual_cmd_vel'
            )
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

    def _require_unowned_map(self, module_key, host):
        """Do not start a GUI map producer while another stack owns /map."""
        try:
            owners = _active_map_publishers(self.node)
        except Exception as error:
            host.set_state('error')
            self._append_log(module_key, f'无法检查 /map 发布者：{error}')
            QtWidgets.QMessageBox.warning(
                self, '无法确认地图归属',
                '未能检查 /map 的发布者，已取消启动。请检查 ROS 连接后重试。',
            )
            return False
        if owners:
            host.set_state('error')
            self._append_log(module_key, '已有 /map 发布者：' + ', '.join(owners))
            QtWidgets.QMessageBox.warning(
                self, '地图已由其他程序发布',
                '当前 /map 发布者：\n' + '\n'.join(owners)
                + '\n\n可到“系统服务”页点击“修复服务状态”自动收回地图资源，'
                '或先停止现有建图或定位程序，再启动控制中心模块。',
            )
            return False
        return True

    def _guard_module_process_conflict(self, module, host, title, description):
        conflicts = preflight.module_process_conflicts(module)
        if not conflicts:
            return False
        detail = '\n'.join(
            f"PID {item['pid']}: {item['command']}" for item in conflicts[:4]
        )
        host.set_state('error')
        self._append_log(module, f'{description}，发现进程占用：{detail}')
        QtWidgets.QMessageBox.warning(
            self, title,
            f'{description}。为避免重复控制，本次不启动该模块；'
            f'控制中心其他功能仍可使用。\n\n{detail}',
        )
        return True

    def _start_navigation_unchecked(self):
        if not self._preflight_ready:
            return
        if self.processes['navigation'].running:
            return
        if self._guard_module_process_conflict(
                'navigation', self.nav_host, '导航资源被占用',
                '检测到已有导航或底盘仲裁进程'):
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
        if not self._require_unowned_map('navigation', self.nav_host):
            return
        remaining_amcl = _process_matches('/nav2_amcl/amcl')
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
        if self._navigation_node is not None:
            try:
                self._navigation_node.stop_immediate()
            except Exception as error:
                self._append_log(
                    'navigation', f'发送导航停止交接命令失败：{error}'
                )
        self.processes['navigation'].stop()

    def _load_slam_ui(self, resume_state=None):
        if self._slam_page is not None:
            return True
        try:
            module = _load_libexec_module(
                'control_center_slam', 'slam_ui_node.py',
                '_control_center_slam_ui',
            )
            page = module.SlamPage(
                topic_suffix=self.topic_suffix,
                resume_state=resume_state,
                chassis_command_topic='/control_center/chassis/slam_input',
            )
            page.stop_requested.connect(self.stop_slam)
            self._slam_module = module
            self._slam_page = page
            self.slam_host.set_content(page)
            return True
        except Exception as error:
            self.slam_host.set_state('error')
            self._append_log('slam', f'加载建图页面失败：{error}')
            return False

    def _start_slam_unchecked(self):
        if not self._preflight_ready:
            return
        if self.processes['slam'].running:
            return
        if self._guard_module_process_conflict(
                'slam', self.slam_host, '建图资源被占用',
                '检测到已有 SLAM、雷达适配或 Cartographer 进程'):
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
        if not self._require_unowned_map('slam', self.slam_host):
            return
        remaining_amcl = _process_matches('/nav2_amcl/amcl')
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
            'enable_motion_safety_watchdog:=true',
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
        if self._slam_page is not None:
            try:
                self._slam_page.node.stop_motion()
            except Exception as error:
                self._append_log('slam', f'发送建图停止交接命令失败：{error}')
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

    def _start_joint_unchecked(self):
        if not self._preflight_ready:
            return
        if self.processes['joint'].running:
            return
        if self._guard_module_process_conflict(
                'joint', self.joint_host, '关节控制资源被占用',
                '检测到已有 joint bridge 或关节控制进程'):
            return
        old_joint_processes = _legacy_joint_processes()
        if old_joint_processes:
            self.joint_host.set_state('error')
            QtWidgets.QMessageBox.warning(
                self, '关节控制资源被占用',
                '检测到旧 whole_body_joint_control，无法确认是否由其他人使用；'
                '本次未自动终止，也未启动第二套桥接节点。\n\n'
                + '\n'.join(old_joint_processes[:4]),
            )
            return
        if not self._load_joint_ui():
            return
        self.processes['joint'].start([
            'launch', 'control_center_joint_control',
            'whole_body_joint_control.launch.py',
            'panel:=false',
            'require_operator_heartbeat:=true',
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
                    self._joint_panel.shutdown_ros()
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
        if any(w is not None and w.isRunning() for w in (
                self._resource_worker, self._plan_worker, self._restore_worker, self._final_check_worker, self._launch_guard_worker)):
            self.statusBar().showMessage('正在完成资源检查，请稍后关闭')
            event.ignore()
            return
        if self._preflight_worker is not None and self._preflight_worker.isRunning():
            event.ignore()
            return
        if (self._service_repair_worker is not None
                and self._service_repair_worker.isRunning()):
            event.ignore()
            return
        if (self._status_check_worker is not None
                and self._status_check_worker.isRunning()):
            event.ignore()
            return
        if (self._module_health_worker is not None
                and self._module_health_worker.isRunning()):
            event.ignore()
            return
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
        self._resource_timer.stop()
        if self._status_page is not None:
            self._status_page.shutdown()
        self._preflight_ready = False
        self._operator_heartbeat_timer.stop()
        self._shell_spin_timer.stop()
        self._battery_timeout_timer.stop()
        if self._joint_panel is not None:
            self.stop_joint()
        if self._slam_page is not None:
            self.stop_slam()
        self._shutdown_service_page()
        self._shutdown_navigation_page()
        self.processes['navigation'].stop()
        self._restore_vendor_stack()
        event.accept()


def main(args=None):
    rclpy.init(args=args)
    node = ControlCenterNode()
    from autolife_gui_runtime import configure_qt_plugins
    configure_qt_plugins()
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
    initial_panel = os.environ.get('AUTOLIFE_START_PANEL', '')
    if initial_panel in ('navigation', 'mapping'):
        window.tabs.setCurrentIndex(0 if initial_panel == 'navigation' else 1)
    signal.signal(signal.SIGINT, lambda *_: window.close())
    signal_timer = QtCore.QTimer()
    signal_timer.start(250)
    signal_timer.timeout.connect(lambda: None)
    window.show()
    QtCore.QTimer.singleShot(0, window.start_preflight)
    try:
        return application.exec_()
    finally:
        if not window._closing:
            window._closing = True
            if window._status_page is not None:
                window._status_page.shutdown()
            window._preflight_ready = False
            window._operator_heartbeat_timer.stop()
            window._shutdown_navigation_page()
            for process in window.processes.values():
                process.stop()
            window._shutdown_service_page()
            window._restore_vendor_stack()
        preflight.close_observer()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.unlock()


if __name__ == '__main__':
    raise SystemExit(main())
