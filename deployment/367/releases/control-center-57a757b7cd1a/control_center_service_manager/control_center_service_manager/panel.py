#!/usr/bin/env python3
"""Small, shell-free Qt frontend for systemd user services."""

from dataclasses import dataclass
import ipaddress
import json
from pathlib import Path
import re
import signal
import subprocess
import sys
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets


SAFE_SERVICE_RE = re.compile(r'^[A-Za-z0-9_.@:-]+\.service$')
ARM_SETTINGS_PATH = Path(
    '/home/ubuntu/autolife-v3/env/lib/python3.12/site-packages/'
    'autolife_robot_arm/settings.toml'
)
VR_GV_SETTING_RE = re.compile(
    r'^\s*vr_gv_control_enabled\s*=\s*(true|false)\s*$', re.MULTILINE
)
RELAY_CONFIG_PATH = Path(
    '/home/ubuntu/Documents/autolife-relay/conf/config.yaml'
)
VISION_SETTINGS_PATH = Path(
    '/home/ubuntu/autolife-v3/env/lib/python3.12/site-packages/'
    'autolife_robot_vision/settings.toml'
)
def navigation_voice_config():
    from ament_index_python.packages import PackageNotFoundError, get_package_share_directory
    try:
        share = Path(get_package_share_directory('control_center_navigation'))
    except PackageNotFoundError:
        share = Path.home() / 'ros2_ws/src/control_center_navigation'
    return share / 'config/voice_feedback.yaml'


NAVIGATION_VOICE_CONFIG_PATH = navigation_voice_config()
VISION_ROBOT_CONFIG_PATH = Path(
    '/home/ubuntu/autolife-v3/env/lib/python3.12/site-packages/'
    'autolife_robot_vision/configs/robot_v3_1.json'
)
ALSA_CARDS_PATH = Path('/proc/asound/cards')
LOGO_SERVICE_UNIT = 'autolife-logo-display.service'
TTS_ENABLED_RE = re.compile(
    r'^(?P<prefix>\s*tts_enabled\s*=\s*)(?P<value>true|false)(?P<suffix>\s*)$',
    re.MULTILINE,
)
NAVIGATION_VOICE_ENABLED_RE = re.compile(
    r'^(?P<prefix>\s*voice_feedback_enabled\s*:\s*)'
    r'(?P<value>true|false)(?P<suffix>\s*(?:#.*)?)$',
    re.MULTILINE,
)
VOICE_CHAIN_DIALOGUE_UNIT = 'void-cog-dialogue.service'
VOICE_CHAIN_DIALOGUE_URL = 'http://127.0.0.1:8767/'
SPEAKER_VOLUME_RE = re.compile(
    r'^(?P<prefix>\s*"speaker_volume"\s*:\s*)'
    r'(?P<value>\d+)(?P<suffix>\s*,?\s*)$',
    re.MULTILINE,
)
RELAY_IP_RE = re.compile(
    r'^(?P<prefix>ip\s*:\s*)(?P<quote>["\']?)'
    r'(?P<value>[^"\'\s#]+)(?P=quote)(?P<suffix>\s*(?:#.*)?)$',
    re.MULTILINE,
)


@dataclass(frozen=True)
class CoreService:
    unit: str
    title: str
    description: str
    accent: str


CORE_SERVICES = (
    CoreService(
        'vision-service.service', '视觉核心', '语音 · 人脸 · 视频', '#4da3ff'
    ),
    CoreService(
        'gv-slam-service.service', 'SLAM / 定位', '导航基础服务', '#9b7bff'
    ),
    CoreService(
        'gv-control-service.service', '导航控制', '启动前需先运行 SLAM', '#45c68b'
    ),
    CoreService(
        'arm-control-service.service', '机械臂', '机械臂控制服务', '#f0a85a'
    ),
)


STYLE = """
QWidget {
  background: #20242b; color: #e7eaf0; font-size: 13px;
}
QLabel { background: transparent; }
QFrame#pageHeader { background: #252b34; border-bottom: 1px solid #38414d; }
QLabel#pageTitle { font-size: 20px; font-weight: 700; color: #f3f6fa; }
QLabel#pageHint, QLabel#muted { color: #9ea8b6; }
QFrame#serviceCard {
  background: #292f38; border: 1px solid #3b4552; border-radius: 8px;
}
QFrame#modeCard {
  background: #292f38; border: 1px solid #3b4552; border-radius: 8px;
}
QLabel#cardTitle { font-size: 16px; font-weight: 700; }
QLabel#unitName { color: #8996a7; font-family: 'DejaVu Sans Mono'; font-size: 11px; }
QLabel#stateGood, QLabel#stateWarn, QLabel#stateBad, QLabel#stateIdle {
  border-radius: 9px; padding: 3px 8px; font-size: 12px; font-weight: 700;
}
QLabel#stateGood { background: #173f31; color: #6cdaa0; }
QLabel#stateWarn { background: #49391e; color: #f1c36c; }
QLabel#stateBad { background: #4b292d; color: #f09aa0; }
QLabel#stateIdle { background: #333b47; color: #aeb9c8; }
QGroupBox {
  background: #262c35; border: 1px solid #39434f; border-radius: 8px;
  margin-top: 10px; padding: 12px 8px 8px 8px; font-weight: 700;
}
QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 5px; }
QPushButton, QToolButton, QComboBox, QLineEdit {
  background: #323b47; color: #e1e8f0; border: 1px solid #485565;
  border-radius: 5px; padding: 6px 11px;
}
QLineEdit:focus { border-color: #4da3d9; background: #27313d; }
QPushButton:hover, QToolButton:hover { background: #3d4958; }
QPushButton:disabled, QToolButton:disabled { color: #697583; background: #292f37; }
QPushButton#primaryButton { background: #247fba; border-color: #2f91d0; color: white; }
QPushButton#dangerButton { background: #47282c; border-color: #724047; color: #f1a0a6; }
QPushButton#successButton { background: #1d6248; border-color: #2d8c68; color: #dff7eb; }
QPushButton#modeButton:checked {
  background: #247fba; border-color: #55aee8; color: white; font-weight: 700;
}
QPushButton#primaryButton:disabled, QPushButton#dangerButton:disabled {
  background: #292f37; border-color: #3c4653; color: #697583;
}
QTableWidget, QPlainTextEdit {
  background: #151a21; alternate-background-color: #1b2129;
  border: 1px solid #37414d; border-radius: 6px;
  gridline-color: #303945; selection-background-color: #285f88;
}
QHeaderView::section {
  background: #2b323c; color: #cfd8e3; border: 0; padding: 7px;
}
QTabWidget::pane { border: 1px solid #39434f; border-radius: 5px; }
QTabBar::tab { background: #2b323c; color: #aeb8c6; padding: 7px 14px; }
QTabBar::tab:selected { background: #247fba; color: white; }
QSplitter::handle { background: #353e49; width: 2px; }
QScrollBar:vertical { background: #1b2027; width: 10px; }
QScrollBar::handle:vertical { background: #46515f; border-radius: 5px; min-height: 25px; }
"""


def validate_service_name(unit):
    name = str(unit or '').strip()
    if not SAFE_SERVICE_RE.fullmatch(name):
        raise ValueError(f'不安全或无效的服务名：{name!r}')
    return name


def validate_ipv4(value):
    text = str(value or '').strip()
    try:
        address = ipaddress.ip_address(text)
    except ValueError as error:
        raise ValueError(f'无效的 IP 地址：{text or "（空）"}') from error
    if address.version != 4:
        raise ValueError('这里只支持 IPv4 地址')
    return str(address)


def parse_relay_ip(content):
    matches = list(RELAY_IP_RE.finditer(str(content)))
    if len(matches) != 1:
        raise ValueError(f'期望找到 1 个根级 ip 配置项，实际找到 {len(matches)} 个')
    return validate_ipv4(matches[0].group('value'))


def update_relay_ip(content, value):
    address = validate_ipv4(value)
    matches = list(RELAY_IP_RE.finditer(str(content)))
    if len(matches) != 1:
        raise ValueError(f'期望修改 1 个根级 ip 配置项，实际找到 {len(matches)} 个')
    updated = RELAY_IP_RE.sub(
        lambda match: (
            f'{match.group("prefix")}\"{address}\"{match.group("suffix")}'
        ),
        str(content),
        count=1,
    )
    return updated, address


def parse_boolean_setting(content, pattern, name):
    matches = list(pattern.finditer(str(content)))
    if len(matches) != 1:
        raise ValueError(f'期望找到 1 个 {name} 配置项，实际找到 {len(matches)} 个')
    return matches[0].group('value') == 'true'


def update_boolean_setting(content, pattern, name, enabled):
    parse_boolean_setting(content, pattern, name)
    value = 'true' if enabled else 'false'
    return pattern.sub(
        lambda match: f'{match.group("prefix")}{value}{match.group("suffix")}',
        str(content), count=1,
    )


def parse_speaker_volume(content):
    matches = list(SPEAKER_VOLUME_RE.finditer(str(content)))
    if len(matches) != 1:
        raise ValueError(
            f'期望找到 1 个 speaker_volume 配置项，实际找到 {len(matches)} 个'
        )
    value = int(matches[0].group('value'))
    if not 0 <= value <= 100:
        raise ValueError(f'speaker_volume 超出 0～100：{value}')
    return value


def update_speaker_volume(content, value):
    volume = int(value)
    if not 0 <= volume <= 100:
        raise ValueError(f'音量必须在 0～100：{volume}')
    parse_speaker_volume(content)
    return SPEAKER_VOLUME_RE.sub(
        lambda match: (
            f'{match.group("prefix")}{volume}{match.group("suffix")}'
        ),
        str(content), count=1,
    )


def parse_pch_card_index(content):
    matches = re.findall(
        r'^\s*(\d+)\s+\[\s*PCH\s*\]\s*:', str(content), re.MULTILINE
    )
    if len(matches) != 1:
        raise ValueError(f'期望找到 1 个 PCH 扬声器声卡，实际找到 {len(matches)} 个')
    return int(matches[0])


def speaker_mixer_command(volume, cards_content=None):
    level = int(volume)
    if not 0 <= level <= 100:
        raise ValueError(f'音量必须在 0～100：{level}')
    if cards_content is None:
        cards_content = ALSA_CARDS_PATH.read_text(encoding='utf-8')
    card = parse_pch_card_index(cards_content)
    return sys.executable, [
        str(Path(__file__).with_name('speaker_volume.py')),
        '--card', str(card), '--volume', str(level),
    ], card


def atomic_replace_text(path, content):
    target = Path(path).resolve(strict=True)
    temporary = target.with_name(f'.{target.name}.control-center.tmp')
    original_mode = target.stat().st_mode & 0o777
    temporary.write_text(content, encoding='utf-8')
    temporary.chmod(original_mode)
    temporary.replace(target)


def command_for(action, unit=None):
    """Return an executable and argv; never return a shell command string."""
    service_actions = {
        'start': ['--user', 'start'],
        'stop': ['--user', 'stop'],
        'restart': ['--user', 'restart'],
        'status': ['--user', 'status'],
        'enable': ['--user', 'enable'],
        'disable': ['--user', 'disable'],
        'is-enabled': ['--user', 'is-enabled'],
        'show': ['--user', 'show'],
    }
    if action in service_actions:
        name = validate_service_name(unit)
        args = [*service_actions[action], name]
        if action == 'status':
            args += ['--no-pager', '--full']
        elif action == 'show':
            args += [
                '--property=LoadState', '--property=ActiveState', '--property=SubState',
                '--property=UnitFileState', '--property=Description',
                '--no-pager',
            ]
        return 'systemctl', args
    if action == 'list-running':
        return 'systemctl', [
            '--user', 'list-units', '--type=service', '--state=running',
            '--no-legend', '--no-pager', '--plain',
        ]
    if action == 'list-unit-files':
        return 'systemctl', [
            '--user', 'list-unit-files', '--type=service',
            '--no-legend', '--no-pager', '--plain',
        ]
    if action in ('journal-recent', 'journal-follow'):
        name = validate_service_name(unit)
        args = ['--user', '-u', name, '-o', 'short-iso']
        if action == 'journal-follow':
            args += ['-f', '-n', '50']
        else:
            args += ['-n', '120', '--no-pager']
        return 'journalctl', args
    raise ValueError(f'未知操作：{action}')


def parse_show_output(text):
    values = {}
    for line in str(text).splitlines():
        if '=' in line:
            key, value = line.split('=', 1)
            values[key.strip()] = value.strip()
    return {
        'load': values.get('LoadState', 'unknown'),
        'active': values.get('ActiveState', 'unknown'),
        'sub': values.get('SubState', 'unknown'),
        'enabled': values.get('UnitFileState', 'unknown'),
        'description': values.get('Description', ''),
    }


def parse_unit_files(text):
    result = {}
    for line in str(text).splitlines():
        fields = line.split()
        if len(fields) >= 2 and fields[0].endswith('.service'):
            result[fields[0]] = fields[1]
    return result


def parse_running_units(text):
    result = {}
    for line in str(text).splitlines():
        fields = line.split(None, 4)
        if len(fields) >= 4 and fields[0].endswith('.service'):
            result[fields[0]] = fields[4] if len(fields) > 4 else ''
    return result


class ServiceCard(QtWidgets.QFrame):
    action_requested = QtCore.pyqtSignal(str, str)

    def __init__(self, service, parent=None):
        super().__init__(parent)
        self.service = service
        self.setObjectName('serviceCard')
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(13, 11, 13, 11)
        layout.setSpacing(7)

        heading = QtWidgets.QHBoxLayout()
        accent = QtWidgets.QLabel('●')
        accent.setStyleSheet(f'color: {service.accent}; font-size: 17px;')
        title = QtWidgets.QLabel(service.title)
        title.setObjectName('cardTitle')
        self.state_label = QtWidgets.QLabel('检查中')
        self.state_label.setObjectName('stateIdle')
        heading.addWidget(accent)
        heading.addWidget(title)
        heading.addStretch()
        heading.addWidget(self.state_label)
        layout.addLayout(heading)

        unit = QtWidgets.QLabel(service.unit)
        unit.setObjectName('unitName')
        layout.addWidget(unit)
        detail_row = QtWidgets.QHBoxLayout()
        description = QtWidgets.QLabel(service.description)
        description.setObjectName('muted')
        self.enabled_label = QtWidgets.QLabel('自启：检查中')
        self.enabled_label.setObjectName('muted')
        detail_row.addWidget(description)
        detail_row.addStretch()
        detail_row.addWidget(self.enabled_label)
        layout.addLayout(detail_row)

        actions = QtWidgets.QHBoxLayout()
        self.buttons = {}
        for action, text, object_name in (
            ('start', '启动', 'primaryButton'),
            ('stop', '停止', 'dangerButton'),
            ('restart', '重启', ''),
            ('status', '详情', ''),
            ('enable', '启用自启', ''),
            ('disable', '禁用自启', ''),
        ):
            button = QtWidgets.QPushButton(text)
            if object_name:
                button.setObjectName(object_name)
            button.clicked.connect(
                lambda _checked=False, a=action: self.action_requested.emit(
                    a, self.service.unit
                )
            )
            self.buttons[action] = button
            actions.addWidget(button)
        layout.addLayout(actions)

    def apply_state(self, state):
        active = state.get('active', 'unknown')
        sub = state.get('sub', 'unknown')
        enabled = state.get('enabled', 'unknown')
        if active == 'active':
            text, object_name = f'运行中 · {sub}', 'stateGood'
        elif active == 'failed':
            text, object_name = '运行失败', 'stateBad'
        elif active in ('inactive', 'deactivating'):
            text, object_name = '已停止', 'stateIdle'
        else:
            text, object_name = active, 'stateWarn'
        self.state_label.setText(text)
        self.state_label.setObjectName(object_name)
        self.state_label.style().unpolish(self.state_label)
        self.state_label.style().polish(self.state_label)
        enabled_text = {
            'enabled': '已启用', 'enabled-runtime': '本次启用',
            'disabled': '已禁用', 'static': '静态', 'masked': '已屏蔽',
        }.get(enabled, enabled)
        self.enabled_label.setText(f'自启：{enabled_text}')
        self.buttons['start'].setEnabled(active != 'active')
        self.buttons['stop'].setEnabled(active == 'active')
        self.buttons['restart'].setEnabled(active == 'active')
        self.buttons['enable'].setEnabled(enabled not in ('enabled', 'static'))
        self.buttons['disable'].setEnabled(enabled in ('enabled', 'enabled-runtime'))


def _voice_chain_run(command, timeout):
    """Run one bounded read-only command; returns (returncode, stdout)."""
    try:
        result = subprocess.run(
            command, capture_output=True, text=True, timeout=timeout,
            check=False)
        return result.returncode, (result.stdout or '').strip()
    except Exception as error:
        return 124, str(error)


def collect_voice_chain_report():
    """Read-only health checks for the voice output/interaction chain.

    Returns a list of (level, title, detail) with level in
    'pass'/'warn'/'fail'/'info'. Nothing is modified and every command is
    bounded; journal evidence is scoped to the current service lifetime,
    so restart-window drops show up as missing TTS/race markers.
    """
    results = []

    def add(level, title, detail=''):
        results.append((level, title, detail))

    unit = 'vision-service.service'
    code, state = _voice_chain_run(
        ['systemctl', '--user', 'is-active', unit], 3.0)
    state = state or 'unknown'
    if state != 'active':
        add('fail', 'vision-service', f'状态 {state}（语音播报与喇叭都不可用）')
        return results

    code, ts_raw = _voice_chain_run(
        ['systemctl', '--user', 'show', unit,
         '--property=ActiveEnterTimestamp', '--value'], 3.0)
    since_epoch = 0
    if ts_raw:
        _code, epoch_text = _voice_chain_run(
            ['date', '-d', ts_raw, '+%s'], 3.0)
        try:
            since_epoch = int(float(epoch_text))
        except ValueError:
            since_epoch = 0
    if since_epoch:
        uptime_min = max(0, int(time.time() - since_epoch)) // 60
        add('pass', 'vision-service', f'运行中（约 {uptime_min} 分钟）')
    else:
        add('pass', 'vision-service', '运行中')

    journal_command = ['journalctl', '--user', '-u', unit, '--no-pager', '-q']
    if since_epoch:
        journal_command += ['--since', f'@{int(since_epoch)}']
    _code, journal_text = _voice_chain_run(journal_command, 12.0)

    def journal_count(pattern):
        return len(re.findall(pattern, journal_text, re.MULTILINE))

    if journal_count('TTS initialized'):
        add('pass', 'TTS 引擎', '本运行周期已初始化，可以发声')
    else:
        add('fail', 'TTS 引擎', '本运行周期未见初始化日志，播报不会出声')

    registered = (journal_count('Microphone registered')
                  + journal_count('Speaker registered on'))
    if registered >= 2:
        add('pass', '麦克风/扬声器', '均已注册')
    elif registered == 1:
        add('warn', '麦克风/扬声器', '本周期仅注册 1 路音频设备')
    else:
        add('warn', '麦克风/扬声器', '本周期未见音频设备注册日志')

    races = journal_count(r'Speaker module.*not found')
    if races:
        add('warn', '扬声器竞态错误',
            f'本周期 {races} 次：重启后短窗口内的播报会被丢弃')
    else:
        add('pass', '扬声器竞态错误', '本周期 0 次')

    drops = journal_count(r"Can't record audio fast enough")
    if drops > 20:
        add('warn', '麦克风采集', f'本周期 {drops} 次丢帧警告（影响识别）')
    elif drops:
        add('info', '麦克风采集', f'本周期 {drops} 次丢帧警告')
    else:
        add('pass', '麦克风采集', '本周期无丢帧警告')

    try:
        vision_text = VISION_SETTINGS_PATH.read_text(encoding='utf-8')
        match = TTS_ENABLED_RE.search(vision_text)
        tts_enabled = bool(match and match.group('value') == 'true')
    except OSError as error:
        tts_enabled = None
        add('warn', '语音总开关（vision tts_enabled）', f'无法读取：{error}')
    if tts_enabled is True:
        add('pass', '语音总开关（vision tts_enabled）', '开')
    elif tts_enabled is False:
        add('fail', '语音总开关（vision tts_enabled）',
            '关：vision-service 不会播放任何语音（含导航播报）')

    try:
        nav_text = NAVIGATION_VOICE_CONFIG_PATH.read_text(encoding='utf-8')
        match = NAVIGATION_VOICE_ENABLED_RE.search(nav_text)
        nav_enabled = bool(match and match.group('value') == 'true')
    except OSError as error:
        nav_enabled = None
        add('warn', '航点播报开关（voice_feedback.yaml）', f'无法读取：{error}')
    if nav_enabled is True:
        add('pass', '航点播报开关（voice_feedback.yaml）', '开')
    elif nav_enabled is False:
        add('warn', '航点播报开关（voice_feedback.yaml）',
            '关：发送导航点不会播报')

    code, installed = _voice_chain_run(
        ['systemctl', '--user', 'list-unit-files', '--no-legend',
         VOICE_CHAIN_DIALOGUE_UNIT], 3.0)
    if not installed:
        add('info', '对话服务 void-cog-dialogue',
            '该机未部署（语音交互由 vision-service 承担）')
        return results
    code, dialogue_state = _voice_chain_run(
        ['systemctl', '--user', 'is-active', VOICE_CHAIN_DIALOGUE_UNIT], 3.0)
    dialogue_state = dialogue_state or 'unknown'
    if dialogue_state != 'active':
        add('fail', '对话服务 void-cog-dialogue',
            f'状态 {dialogue_state}：语音听得见但不会回答')
    else:
        code, restarts_text = _voice_chain_run(
            ['systemctl', '--user', 'show', VOICE_CHAIN_DIALOGUE_UNIT,
             '--property=NRestarts', '--value'], 3.0)
        try:
            restarts = int((restarts_text or '0').strip() or '0')
        except ValueError:
            restarts = 0
        if restarts:
            add('warn', '对话服务 void-cog-dialogue',
                f'运行中，本机已重启 {restarts} 次（曾出现崩溃循环）')
        else:
            add('pass', '对话服务 void-cog-dialogue', '运行中，无重启')
    code, http_code = _voice_chain_run(
        ['curl', '-s', '-o', '/dev/null', '-w', '%{http_code}',
         '-m', '2', VOICE_CHAIN_DIALOGUE_URL], 4.0)
    http_code = (http_code.splitlines()[-1]
                 if http_code else '').strip()
    if http_code and http_code != '000':
        add('pass', '对话端口 8767', f'有应答（HTTP {http_code}）')
    else:
        add('fail', '对话端口 8767', '无应答：语音交互不可用')
    return results


class VoiceChainWorker(QtCore.QThread):
    completed = QtCore.pyqtSignal(list)

    def run(self):
        self.completed.emit(collect_voice_chain_report())


def _speak_voice_chain_confirmation(topic_suffix, text='语音自检通过'):
    """Best-effort one-shot announcement on the vendor speech topic.

    Publishes one String on /topic_tts_<suffix> - the same path navigation
    announcements use - so hearing it proves the whole TTS/speaker chain.
    Vendor TTS protocol: {"status": "play", "text": "..."}.
    rclpy is imported lazily so this module stays usable without ROS.
    """
    try:
        import rclpy
        from std_msgs.msg import String
    except Exception as error:
        print(f'voice chain speak skipped (no rclpy): {error}', flush=True)
        return False
    topic = f'/topic_tts_{str(topic_suffix).strip()}'
    payload = json.dumps(
        {'status': 'play', 'text': text}, ensure_ascii=False)
    owns_context = not rclpy.ok()
    node = None
    try:
        if owns_context:
            rclpy.init()
        node = rclpy.create_node('control_center_voice_chain_speak')
        publisher = node.create_publisher(String, topic, 10)
        time.sleep(0.4)  # let DDS discovery match the vision-service sub
        publisher.publish(String(data=payload))
        time.sleep(0.4)  # keep the participant alive while delivering
        return True
    except Exception as error:
        print(f'voice chain speak failed: {error}', flush=True)
        return False
    finally:
        if node is not None:
            try:
                node.destroy_node()
            except Exception:
                pass
        if owns_context and rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception:
                pass


class ServiceManagerPanel(QtWidgets.QWidget):
    log_message = QtCore.pyqtSignal(str)
    service_state_changed = QtCore.pyqtSignal(str, str, str)
    repair_requested = QtCore.pyqtSignal()
    status_check_requested = QtCore.pyqtSignal()
    restore_requested = QtCore.pyqtSignal()

    def __init__(
        self,
        action_guard=None,
        auto_refresh=True,
        topic_suffix='0_367',
        parent=None,
    ):
        super().__init__(parent)
        self.action_guard = action_guard
        self.topic_suffix = str(topic_suffix).strip()
        self.setStyleSheet(STYLE)
        self._processes = set()
        self._log_process = None
        self._unit_files = {}
        self._running_units = {}
        self._core_cards = {}
        self._navigation_mode_busy = False
        self._joint_reset_busy = False
        self._relay_ip_busy = False
        self._voice_feedback_busy = False
        self._voice_volume_busy = False
        self._logo_busy = False
        self._repair_busy = False
        self._status_check_busy = False
        self._voice_chain_busy = False
        self._voice_chain_worker = None
        self._build_ui()

        self.refresh_timer = QtCore.QTimer(self)
        self.refresh_timer.setInterval(6000)
        self.refresh_timer.timeout.connect(self.refresh_core_services)
        if auto_refresh:
            self.refresh_timer.start()
            QtCore.QTimer.singleShot(100, self.refresh_all)

    def _build_ui(self):
        outer = QtWidgets.QVBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        header = QtWidgets.QFrame()
        header.setObjectName('pageHeader')
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(16, 9, 16, 9)
        text_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('系统用户服务')
        title.setObjectName('pageTitle')
        hint = QtWidgets.QLabel('核心服务一键控制 · 状态、自启与日志集中查看')
        hint.setObjectName('pageHint')
        text_box.addWidget(title)
        text_box.addWidget(hint)
        header_layout.addLayout(text_box)
        header_layout.addStretch()
        start_nav = QtWidgets.QPushButton('启动导航组合')
        start_nav.setObjectName('successButton')
        start_nav.clicked.connect(self.start_navigation_stack)
        stop_nav = QtWidgets.QPushButton('停止导航组合')
        stop_nav.setObjectName('dangerButton')
        stop_nav.clicked.connect(self.stop_navigation_stack)
        refresh = QtWidgets.QPushButton('刷新全部')
        refresh.setObjectName('primaryButton')
        refresh.clicked.connect(self.refresh_all)
        header_layout.addWidget(start_nav)
        header_layout.addWidget(stop_nav)
        header_layout.addWidget(refresh)
        outer.addWidget(header)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        layout.setContentsMargins(12, 10, 12, 10)
        layout.setSpacing(9)

        cards_group = QtWidgets.QGroupBox('核心服务')
        cards_layout = QtWidgets.QGridLayout(cards_group)
        cards_layout.setContentsMargins(9, 13, 9, 9)
        cards_layout.setSpacing(8)
        for index, service in enumerate(CORE_SERVICES):
            card = ServiceCard(service)
            card.action_requested.connect(self.request_action)
            self._core_cards[service.unit] = card
            cards_layout.addWidget(card, index // 2, index % 2)
        layout.addWidget(cards_group)
        layout.addWidget(self._build_service_repair_card())
        layout.addWidget(self._build_navigation_mode_card())
        layout.addWidget(self._build_voice_feedback_card())
        layout.addWidget(self._build_voice_chain_card())
        layout.addWidget(self._build_system_volume_card())
        layout.addWidget(self._build_logo_display_card())
        layout.addWidget(self._build_relay_ip_card())
        layout.addWidget(self._build_joint_reset_card())

        lower = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        lower.setChildrenCollapsible(False)
        lower.addWidget(self._build_service_browser())
        lower.addWidget(self._build_output_panel())
        lower.setSizes([760, 600])
        layout.addWidget(lower, 1)

        self.event_label = QtWidgets.QLabel('就绪')
        self.event_label.setObjectName('muted')
        layout.addWidget(self.event_label)
        scroll.setWidget(content)
        outer.addWidget(scroll, 1)

    def _build_service_repair_card(self):
        card = QtWidgets.QFrame()
        card.setObjectName('modeCard')
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(13, 10, 13, 10)
        layout.setSpacing(12)

        text_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('服务状态修复')
        title.setObjectName('cardTitle')
        hint = QtWidgets.QLabel(
            '“状态自检”只读检查控制进程、厂商服务、双雷达、地图与底盘资源；'
            '“修复服务状态”一键把发现的问题修复到可用，包括收回被外部程序占用的 /map 与底盘控制权'
            '——这会中断外部导航/建图任务，未保存地图可能丢失。'
            '厂商核心底盘/机械臂服务只做启停恢复，不会被终止；与控制中心资源无关的进程不受影响。'
            '所有中断动作均记录日志。'
        )
        hint.setObjectName('muted')
        hint.setWordWrap(True)
        self.service_repair_detail = QtWidgets.QLabel(
            '可随时运行状态自检；发现问题后点击“修复服务状态”一键修复，'
            '受影响的导航/建图模块会自动按原参数重启。'
        )
        self.service_repair_detail.setWordWrap(True)
        self.service_repair_detail.setObjectName('muted')
        text_box.addWidget(title)
        text_box.addWidget(hint)
        text_box.addWidget(self.service_repair_detail)
        layout.addLayout(text_box, 1)

        actions = QtWidgets.QVBoxLayout()
        repair_row = QtWidgets.QHBoxLayout()
        self.service_repair_status = QtWidgets.QLabel('待命')
        self.service_repair_status.setObjectName('stateIdle')
        repair_row.addWidget(self.service_repair_status)
        self.service_repair_button = QtWidgets.QPushButton('修复服务状态')
        self.service_repair_button.setObjectName('successButton')
        self.service_repair_button.setEnabled(True)
        self.service_repair_button.clicked.connect(self._request_service_repair)
        repair_row.addWidget(self.service_repair_button)
        self.restore_services_button = QtWidgets.QPushButton('恢复被暂停服务')
        self.restore_services_button.clicked.connect(self.restore_requested.emit)
        repair_row.addWidget(self.restore_services_button)
        actions.addLayout(repair_row)

        check_row = QtWidgets.QHBoxLayout()
        self.status_check_status = QtWidgets.QLabel('待自检')
        self.status_check_status.setObjectName('stateIdle')
        check_row.addWidget(self.status_check_status)
        self.status_check_button = QtWidgets.QPushButton('状态自检')
        self.status_check_button.clicked.connect(self._request_status_check)
        check_row.addWidget(self.status_check_button)
        actions.addLayout(check_row)
        layout.addLayout(actions)
        return card

    def _set_service_repair_status(self, text, object_name):
        self.service_repair_status.setText(text)
        self.service_repair_status.setObjectName(object_name)
        self.service_repair_status.style().unpolish(self.service_repair_status)
        self.service_repair_status.style().polish(self.service_repair_status)

    def set_service_repair_context(self, detail, button_text='修复服务状态'):
        if self._repair_busy or self._status_check_busy:
            return
        message = str(detail or '').strip()
        self.service_repair_button.setText('修复服务状态')
        self.service_repair_button.setEnabled(True)
        if message:
            self.service_repair_detail.setText('待处理问题：' + message)
            self._set_service_repair_status('可修复', 'stateWarn')
        else:
            self.service_repair_detail.setText(
                '可随时运行状态自检；发现问题后点击“修复服务状态”一键修复，'
                '受影响的导航/建图模块会自动按原参数重启。'
            )
            self._set_service_repair_status('待命', 'stateIdle')

    def _request_service_repair(self):
        if self._repair_busy or self._status_check_busy:
            return
        self._repair_busy = True
        self.service_repair_button.setEnabled(False)
        self.status_check_button.setEnabled(False)
        self.restore_services_button.setEnabled(False)
        self.service_repair_detail.setText(
            '正在执行安全检查和一次性恢复，请勿关闭控制中心。'
        )
        self._set_service_repair_status('修复中', 'stateWarn')
        self.repair_requested.emit()

    def _request_status_check(self):
        if self._repair_busy or self._status_check_busy:
            return
        self._status_check_busy = True
        self.service_repair_button.setEnabled(False)
        self.status_check_button.setEnabled(False)
        self.restore_services_button.setEnabled(False)
        self.status_check_status.setText('检查中')
        self.status_check_status.setObjectName('stateWarn')
        self.status_check_status.style().unpolish(self.status_check_status)
        self.status_check_status.style().polish(self.status_check_status)
        self.service_repair_detail.setText('正在只读检查服务、进程、双雷达及地图发布者……')
        self.status_check_requested.emit()

    def set_service_repair_progress(self, detail):
        self._repair_busy = True
        self.service_repair_button.setEnabled(False)
        self.status_check_button.setEnabled(False)
        self.restore_services_button.setEnabled(False)
        self.service_repair_detail.setText(str(detail))
        self._set_service_repair_status('修复中', 'stateWarn')

    def set_status_check_progress(self, detail):
        self._status_check_busy = True
        self.service_repair_button.setEnabled(False)
        self.status_check_button.setEnabled(False)
        self.restore_services_button.setEnabled(False)
        self.status_check_status.setText('检查中')
        self.status_check_status.setObjectName('stateWarn')
        self.status_check_status.style().unpolish(self.status_check_status)
        self.status_check_status.style().polish(self.status_check_status)
        self.service_repair_detail.setText(str(detail))

    def finish_service_repair(self, result, detail):
        self._repair_busy = False
        self.service_repair_button.setEnabled(True)
        self.status_check_button.setEnabled(True)
        self.restore_services_button.setEnabled(True)
        if result in ('PASS', 'AUTO_RECOVERED'):
            self.service_repair_detail.setText(str(detail))
            self._set_service_repair_status('修复完成', 'stateGood')
            return
        self.service_repair_detail.setText('服务状态修复未完成：' + str(detail))
        self._set_service_repair_status('修复失败', 'stateBad')

    def finish_status_check(self, result, detail):
        self._status_check_busy = False
        self.service_repair_button.setEnabled(True)
        self.status_check_button.setEnabled(True)
        self.restore_services_button.setEnabled(True)
        if result == 'PASS':
            self.status_check_status.setText('检查通过')
            self.status_check_status.setObjectName('stateGood')
            self.service_repair_detail.setText(str(detail))
        else:
            self.status_check_status.setText('发现异常')
            self.status_check_status.setObjectName('stateBad')
            self.service_repair_detail.setText('自检发现问题：' + str(detail))
        self.status_check_status.style().unpolish(self.status_check_status)
        self.status_check_status.style().polish(self.status_check_status)

    def _build_navigation_mode_card(self):
        card = QtWidgets.QFrame()
        card.setObjectName('modeCard')
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(13, 10, 13, 10)
        layout.setSpacing(10)

        text_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('导航控制来源')
        title.setObjectName('cardTitle')
        hint = QtWidgets.QLabel(
            '切换机械臂服务是否接受智动 VR/GV 控制；切换后自动重启机械臂服务'
        )
        hint.setObjectName('muted')
        text_box.addWidget(title)
        text_box.addWidget(hint)
        layout.addLayout(text_box, 1)

        self.navigation_mode_status = QtWidgets.QLabel('检查中')
        self.navigation_mode_status.setObjectName('stateIdle')
        layout.addWidget(self.navigation_mode_status)

        self.ours_navigation_button = QtWidgets.QPushButton('使用我们的导航')
        self.vendor_navigation_button = QtWidgets.QPushButton('使用智动导航')
        self.ours_navigation_button.setObjectName('modeButton')
        self.vendor_navigation_button.setObjectName('modeButton')
        self.ours_navigation_button.setCheckable(True)
        self.vendor_navigation_button.setCheckable(True)
        self.navigation_mode_group = QtWidgets.QButtonGroup(self)
        self.navigation_mode_group.setExclusive(True)
        self.navigation_mode_group.addButton(self.ours_navigation_button)
        self.navigation_mode_group.addButton(self.vendor_navigation_button)
        self.ours_navigation_button.clicked.connect(
            lambda checked: self.set_navigation_mode(False) if checked else None
        )
        self.vendor_navigation_button.clicked.connect(
            lambda checked: self.set_navigation_mode(True) if checked else None
        )
        layout.addWidget(self.ours_navigation_button)
        layout.addWidget(self.vendor_navigation_button)
        return card

    def _build_voice_feedback_card(self):
        card = QtWidgets.QFrame()
        card.setObjectName('modeCard')
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(13, 10, 13, 10)
        layout.setSpacing(10)
        text_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('导航语音播报')
        title.setObjectName('cardTitle')
        hint = QtWidgets.QLabel(
            '控制航点出发、到达和导航避让提示；默认开启，切换后立即生效'
        )
        hint.setObjectName('muted')
        text_box.addWidget(title)
        text_box.addWidget(hint)
        layout.addLayout(text_box, 1)
        self.voice_feedback_status = QtWidgets.QLabel('检查中')
        self.voice_feedback_status.setObjectName('stateIdle')
        layout.addWidget(self.voice_feedback_status)
        self.voice_feedback_on_button = QtWidgets.QPushButton('开启播报')
        self.voice_feedback_off_button = QtWidgets.QPushButton('关闭播报')
        self.voice_feedback_on_button.setObjectName('modeButton')
        self.voice_feedback_off_button.setObjectName('modeButton')
        self.voice_feedback_on_button.setCheckable(True)
        self.voice_feedback_off_button.setCheckable(True)
        group = QtWidgets.QButtonGroup(self)
        group.setExclusive(True)
        group.addButton(self.voice_feedback_on_button)
        group.addButton(self.voice_feedback_off_button)
        self.voice_feedback_group = group
        self.voice_feedback_on_button.clicked.connect(
            lambda checked: self.set_voice_feedback_enabled(True) if checked else None
        )
        self.voice_feedback_off_button.clicked.connect(
            lambda checked: self.set_voice_feedback_enabled(False) if checked else None
        )
        layout.addWidget(self.voice_feedback_on_button)
        layout.addWidget(self.voice_feedback_off_button)
        return card

    def _build_system_volume_card(self):
        card = QtWidgets.QFrame()
        card.setObjectName('modeCard')
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(13, 10, 13, 10)
        layout.setSpacing(10)
        text_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('系统音量调节')
        title.setObjectName('cardTitle')
        hint = QtWidgets.QLabel('调节本机扬声器音量；松开滑块后保存并播放短试听音，关闭控制中心后仍生效')
        hint.setObjectName('muted')
        text_box.addWidget(title)
        text_box.addWidget(hint)
        layout.addLayout(text_box, 1)
        volume_box = QtWidgets.QVBoxLayout()
        self.voice_volume_value = QtWidgets.QLabel('70%')
        self.voice_volume_value.setAlignment(QtCore.Qt.AlignCenter)
        self.voice_volume_value.setMinimumWidth(48)
        self.voice_volume_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.voice_volume_slider.setRange(0, 100)
        self.voice_volume_slider.setValue(70)
        self.voice_volume_slider.setSingleStep(5)
        self.voice_volume_slider.setPageStep(10)
        self.voice_volume_slider.setMinimumWidth(170)
        self.voice_volume_slider.setToolTip('拖动后松开即应用系统音量并试听；也可以点击应用并试听')
        self.voice_volume_slider.valueChanged.connect(
            lambda value: self.voice_volume_value.setText(f'{value}%')
        )
        self.voice_volume_slider.sliderReleased.connect(self.apply_voice_volume)
        volume_box.addWidget(self.voice_volume_value)
        volume_box.addWidget(self.voice_volume_slider)
        layout.addLayout(volume_box)
        self.voice_volume_apply_button = QtWidgets.QPushButton('应用并试听')
        self.voice_volume_apply_button.setObjectName('primaryButton')
        self.voice_volume_apply_button.clicked.connect(self.apply_voice_volume)
        layout.addWidget(self.voice_volume_apply_button)
        return card

    def _set_voice_feedback_state(self, enabled, detail='', service_active=None):
        self.voice_feedback_on_button.setChecked(enabled is True)
        self.voice_feedback_off_button.setChecked(enabled is False)
        if enabled is True:
            if service_active is False:
                text, state = '已开启 · 语音服务未运行', 'stateWarn'
            else:
                text, state = '已开启', 'stateGood'
        elif enabled is False:
            text, state = '已关闭', 'stateIdle'
        else:
            text, state = '配置异常', 'stateBad'
        self.voice_feedback_status.setText(text)
        self.voice_feedback_status.setToolTip(detail)
        self.voice_feedback_status.setObjectName(state)
        self.voice_feedback_status.style().unpolish(self.voice_feedback_status)
        self.voice_feedback_status.style().polish(self.voice_feedback_status)

    def _read_voice_feedback_state(self):
        vision_content = VISION_SETTINGS_PATH.read_text(encoding='utf-8')
        navigation_content = NAVIGATION_VOICE_CONFIG_PATH.read_text(encoding='utf-8')
        tts_enabled = parse_boolean_setting(
            vision_content, TTS_ENABLED_RE, 'tts_enabled'
        )
        navigation_enabled = parse_boolean_setting(
            navigation_content,
            NAVIGATION_VOICE_ENABLED_RE,
            'voice_feedback_enabled',
        )
        if tts_enabled != navigation_enabled:
            raise RuntimeError(
                '导航播报和视觉 TTS 配置不一致，请重新点击开启或关闭'
            )
        return tts_enabled

    def refresh_voice_feedback(self):
        try:
            enabled = self._read_voice_feedback_state()
        except Exception as error:
            self._set_voice_feedback_state(None, str(error))
            return

        def service_checked(success, text, _code):
            state = parse_show_output(text) if success and text else {}
            active = state.get('active') == 'active'
            self._set_voice_feedback_state(
                enabled,
                f'vision-service.service: {state.get("active", "unknown")}',
                active,
            )

        self._run_named('show', 'vision-service.service', service_checked)

    def _set_voice_feedback_busy(self, busy):
        self._voice_feedback_busy = bool(busy)
        self.voice_feedback_on_button.setEnabled(not busy)
        self.voice_feedback_off_button.setEnabled(not busy)

    def refresh_voice_volume(self):
        if self._voice_volume_busy or getattr(self, '_voice_volume_read_pending', False):
            return None
        try:
            content = VISION_ROBOT_CONFIG_PATH.read_text(encoding='utf-8')
            volume = parse_speaker_volume(content)
        except Exception as error:
            self.voice_volume_value.setText('读取失败')
            self.voice_volume_value.setToolTip(str(error))
            return None
        self._voice_volume_read_pending = True
        def received(success, text, _code):
            self._voice_volume_read_pending = False
            if self._voice_volume_busy:
                return
            try:
                if not success:raise RuntimeError(text)
                actual=json.loads(text)
                measured=int(actual['volume'])
                if not self.voice_volume_slider.isSliderDown() and not self.voice_volume_slider.hasFocus():
                    self.voice_volume_slider.blockSignals(True)
                    self.voice_volume_slider.setValue(measured)
                    self.voice_volume_slider.blockSignals(False)
                self.voice_volume_value.setText(f'{measured}%')
                self.voice_volume_value.setToolTip(
                    f"实际Master {actual['master']['percent']}%，PCM {actual['pcm']['percent']}%；保存值{volume}%")
            except Exception as error:
                self.voice_volume_value.setText(f'保存{volume}% / 实际未知')
                self.voice_volume_value.setToolTip(str(error))
        self._run_command(sys.executable,[str(Path(__file__).with_name('speaker_volume.py')),'--read'],
                          received,label='读取扬声器实际音量')
        return volume

    def _set_voice_volume_busy(self, busy):
        self._voice_volume_busy = bool(busy)
        self.voice_volume_slider.setEnabled(not busy)
        self.voice_volume_apply_button.setEnabled(not busy)
        self.voice_volume_apply_button.setText('应用中…' if busy else '应用并试听')

    def apply_voice_volume(self):
        if self._voice_volume_busy:
            return
        requested = int(self.voice_volume_slider.value())
        try:
            original = VISION_ROBOT_CONFIG_PATH.read_text(encoding='utf-8')
            current = parse_speaker_volume(original)
            if requested != current:
                updated = update_speaker_volume(original, requested)
                atomic_replace_text(VISION_ROBOT_CONFIG_PATH, updated)
            verified = parse_speaker_volume(
                VISION_ROBOT_CONFIG_PATH.read_text(encoding='utf-8')
            )
            if verified != requested:
                raise RuntimeError('音量保存后校验失败')
            mixer_program, mixer_arguments, mixer_card = speaker_mixer_command(
                requested
            )
        except Exception as error:
            self._show_error(f'修改系统音量失败：{error}')
            self.refresh_voice_volume()
            return

        self._set_voice_volume_busy(True)
        self.event_label.setText(f'正在应用系统音量 {requested}%……')

        def mixer_applied(success, text, code):
            if not success:
                try:
                    atomic_replace_text(VISION_ROBOT_CONFIG_PATH, original)
                except Exception:
                    pass
                self._set_voice_volume_busy(False)
                self.refresh_voice_volume()
                detail = text or f'amixer 退出码 {code}'
                self._show_error(f'设置实际扬声器音量失败：{detail}')
                return
            self._set_voice_volume_busy(False)
            self.refresh_voice_volume()
            if requested == 0:
                self.event_label.setText('系统音量已保存为静音；试听音不会响')
                return
            self.event_label.setText(f'系统音量已保存为 {requested}%，正在播放试听音……')

            def preview_finished(played, output, preview_code):
                if played:
                    self.event_label.setText(
                        f'系统音量 {requested}% 已生效并保存；试听音已播放（PCH {mixer_card}）')
                else:
                    detail = output or f'播放进程退出码 {preview_code}'
                    self.event_label.setText(
                        f'系统音量 {requested}% 已生效并保存，但试听音播放失败：{detail}')
                    self.log_message.emit(f'系统音量试听失败：{detail}')

            self._run_command(
                sys.executable,
                [str(Path(__file__).with_name('speaker_volume.py')),
                 '--card', str(mixer_card), '--preview'],
                preview_finished,
                label='播放系统音量试听音',
            )

        self._run_command(
            mixer_program,
            mixer_arguments,
            mixer_applied,
            label=f'设置 PCH 声卡 {mixer_card} 音量',
        )

    def set_voice_feedback_enabled(self, enabled):
        if self._voice_feedback_busy or not self._guard_allows('restart', 'vision-service.service'):
            return
        try:
            vision_original = VISION_SETTINGS_PATH.read_text(encoding='utf-8')
            navigation_original = NAVIGATION_VOICE_CONFIG_PATH.read_text(encoding='utf-8')
            vision_updated = update_boolean_setting(
                vision_original, TTS_ENABLED_RE, 'tts_enabled', enabled
            )
            navigation_updated = update_boolean_setting(
                navigation_original,
                NAVIGATION_VOICE_ENABLED_RE,
                'voice_feedback_enabled',
                enabled,
            )
            atomic_replace_text(VISION_SETTINGS_PATH, vision_updated)
            try:
                atomic_replace_text(NAVIGATION_VOICE_CONFIG_PATH, navigation_updated)
            except Exception:
                atomic_replace_text(VISION_SETTINGS_PATH, vision_original)
                raise
        except Exception as error:
            self._show_error(f'修改导航语音播报配置失败：{error}')
            self.refresh_voice_feedback()
            return

        self._set_voice_feedback_busy(True)
        self._set_voice_feedback_state(enabled, '正在重启语音服务')
        self.event_label.setText('正在应用导航语音播报设置……')

        def parameter_done(success, text, _code):
            self._set_voice_feedback_busy(False)
            self.refresh_voice_feedback()
            if success and re.search(r'\bset parameter successful\b', text or '', re.IGNORECASE):
                self.event_label.setText(f'导航语音播报已{"开启" if enabled else "关闭"}并立即生效')
            else:
                self.event_label.setText('语音配置已保存；运行中的导航参数未确认，下一次启动将读取配置。')

        def service_restarted(success):
            if not success:
                self._set_voice_feedback_busy(False)
                self._show_error('配置已保存，但 vision-service.service 重启失败')
                self.refresh_voice_feedback()
                return
            self._run_command(
                'ros2', [
                    'param', 'set', '/waypoint_manager',
                    'voice_feedback_enabled', 'true' if enabled else 'false',
                ], parameter_done, label='更新运行中的导航语音参数',
            )

        self._execute_action(
            'restart', 'vision-service.service', service_restarted
        )

    def _build_logo_display_card(self):
        card = QtWidgets.QFrame()
        card.setObjectName('modeCard')
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(13, 10, 13, 10)
        layout.setSpacing(10)
        text_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('Logo 胸屏网页')
        title.setObjectName('cardTitle')
        hint = QtWidgets.QLabel(
            '开启时切换为 1024×768 Portrait Left；关闭时恢复 1920×1080 Landscape (flipped)'
        )
        hint.setObjectName('muted')
        hint.setWordWrap(True)
        text_box.addWidget(title)
        text_box.addWidget(hint)
        layout.addLayout(text_box, 1)
        self.logo_display_status = QtWidgets.QLabel('检查中')
        self.logo_display_status.setObjectName('stateIdle')
        layout.addWidget(self.logo_display_status)
        self.logo_display_start_button = QtWidgets.QPushButton('开启 Logo 网页')
        self.logo_display_start_button.setObjectName('successButton')
        self.logo_display_start_button.clicked.connect(
            lambda: self.set_logo_display_running(True)
        )
        self.logo_display_stop_button = QtWidgets.QPushButton('关闭 Logo 网页')
        self.logo_display_stop_button.setObjectName('dangerButton')
        self.logo_display_stop_button.clicked.connect(
            lambda: self.set_logo_display_running(False)
        )
        layout.addWidget(self.logo_display_start_button)
        layout.addWidget(self.logo_display_stop_button)
        return card

    def _set_logo_state(self, active, detail=''):
        if active is True:
            text, state = '运行中', 'stateGood'
        elif active is False:
            text, state = '已关闭', 'stateIdle'
        else:
            text, state = '状态异常', 'stateBad'
        self.logo_display_status.setText(text)
        self.logo_display_status.setToolTip(detail)
        self.logo_display_status.setObjectName(state)
        self.logo_display_status.style().unpolish(self.logo_display_status)
        self.logo_display_status.style().polish(self.logo_display_status)
        if not self._logo_busy:
            self.logo_display_start_button.setEnabled(active is False)
            self.logo_display_stop_button.setEnabled(active is True)

    def refresh_logo_display(self):
        def checked(success, text, _code):
            state = parse_show_output(text or '')
            if state['load'] == 'not-found':
                self._set_logo_state(None, f'未安装 {LOGO_SERVICE_UNIT}')
                self.logo_display_status.setText('专用胸屏服务未安装')
                return
            active_state = state.get('active', 'unknown') if success else 'unknown'
            if active_state == 'active':
                active = True
            elif active_state in ('inactive', 'failed'):
                active = False
            else:
                active = None
            self._set_logo_state(active, f'{LOGO_SERVICE_UNIT}: {active_state}')

        self._run_named('show', LOGO_SERVICE_UNIT, checked)

    def _set_logo_busy(self, busy):
        self._logo_busy = bool(busy)
        self.logo_display_start_button.setEnabled(not busy)
        self.logo_display_stop_button.setEnabled(not busy)

    def set_logo_display_running(self, running):
        if self._logo_busy:
            return
        self._set_logo_busy(True)
        action = 'start' if running else 'stop'
        self.event_label.setText(
            '正在开启 Logo 网页……' if running else '正在关闭 Logo 网页……'
        )

        def completed(success):
            self._set_logo_busy(False)
            self.refresh_logo_display()
            if success:
                self.event_label.setText(
                    'Logo 网页已开启' if running else 'Logo 网页已关闭'
                )
            else:
                self._show_error(
                    'Logo 网页开启失败' if running else 'Logo 网页关闭失败'
                )

        self._execute_action(action, LOGO_SERVICE_UNIT, completed)

    def _build_voice_chain_card(self):
        card = QtWidgets.QFrame()
        card.setObjectName('modeCard')
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(13, 10, 13, 10)
        layout.setSpacing(12)

        text_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('语音链路自检')
        title.setObjectName('cardTitle')
        hint = QtWidgets.QLabel(
            '一键只读检查语音播报与语音交互链路：vision-service 活性与 TTS 就绪、'
            '麦克风/扬声器注册、扬声器竞态错误、语音总开关与航点播报开关、'
            '对话服务与端口健康。不修改任何配置；'
            '检查通过（无告警与故障）时机器人会播报“语音自检通过”。'
        )
        hint.setObjectName('muted')
        hint.setWordWrap(True)
        self.voice_chain_detail = QtWidgets.QLabel(
            '可随时点击“语音链路自检”查看当前语音链路状态。'
        )
        self.voice_chain_detail.setWordWrap(True)
        self.voice_chain_detail.setObjectName('muted')
        text_box.addWidget(title)
        text_box.addWidget(hint)
        text_box.addWidget(self.voice_chain_detail)
        layout.addLayout(text_box, 1)

        row = QtWidgets.QHBoxLayout()
        self.voice_chain_status = QtWidgets.QLabel('待自检')
        self.voice_chain_status.setObjectName('stateIdle')
        row.addWidget(self.voice_chain_status)
        self.voice_chain_button = QtWidgets.QPushButton('语音链路自检')
        self.voice_chain_button.setObjectName('modeButton')
        self.voice_chain_button.clicked.connect(self._request_voice_chain_check)
        row.addWidget(self.voice_chain_button)
        layout.addLayout(row)
        return card

    def _request_voice_chain_check(self):
        if self._voice_chain_busy:
            return
        self._voice_chain_busy = True
        self.voice_chain_button.setEnabled(False)
        self.voice_chain_status.setText('检查中')
        self.voice_chain_status.setObjectName('stateWarn')
        self.voice_chain_status.style().unpolish(self.voice_chain_status)
        self.voice_chain_status.style().polish(self.voice_chain_status)
        self.voice_chain_detail.setText('正在只读检查语音链路（约数秒）……')
        self._voice_chain_worker = VoiceChainWorker()
        self._voice_chain_worker.completed.connect(self._voice_chain_complete)
        self._voice_chain_worker.start()

    def _voice_chain_complete(self, results):
        self._voice_chain_busy = False
        self.voice_chain_button.setEnabled(True)
        markers = {'pass': '✓', 'warn': '⚠', 'fail': '✗', 'info': '·'}
        order = {'fail': 0, 'warn': 1, 'info': 2, 'pass': 3}
        worst = min((order.get(level, 2) for level, _t, _d in results),
                    default=2)
        text, object_name = {
            0: ('有故障', 'stateBad'),
            1: ('有告警', 'stateWarn'),
            2: ('有提示', 'stateIdle'),
            3: ('正常', 'stateGood'),
        }[worst]
        self.voice_chain_status.setText(text)
        self.voice_chain_status.setObjectName(object_name)
        self.voice_chain_status.style().unpolish(self.voice_chain_status)
        self.voice_chain_status.style().polish(self.voice_chain_status)
        lines = [
            f"{markers.get(level, '·')} {title}" + (f"：{detail}" if detail else '')
            for level, title, detail in results
        ]
        self.voice_chain_detail.setText('\n'.join(lines))
        if worst >= 2:
            threading.Thread(
                target=_speak_voice_chain_confirmation,
                args=(self.topic_suffix,), daemon=True).start()
            self.log_message.emit(
                f'语音链路自检：{text}（共 {len(results)} 项）；机器人正在播报确认')
        else:
            self.log_message.emit(f'语音链路自检：{text}（共 {len(results)} 项）')

    def _build_relay_ip_card(self):
        card = QtWidgets.QFrame()
        card.setObjectName('modeCard')
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(13, 10, 13, 10)
        layout.setSpacing(10)

        text_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('中继配置 IP')
        title.setObjectName('cardTitle')
        hint = QtWidgets.QLabel(
            f'查看或修改 {RELAY_CONFIG_PATH} 中的根级 ip；保存后自动重启中继服务并立即生效'
        )
        hint.setObjectName('muted')
        hint.setWordWrap(True)
        text_box.addWidget(title)
        text_box.addWidget(hint)
        layout.addLayout(text_box, 1)

        self.relay_ip_status = QtWidgets.QLabel('未读取')
        self.relay_ip_status.setObjectName('stateIdle')
        layout.addWidget(self.relay_ip_status)

        self.relay_ip_edit = QtWidgets.QLineEdit()
        self.relay_ip_edit.setPlaceholderText('例如 192.168.8.186')
        self.relay_ip_edit.setClearButtonEnabled(True)
        self.relay_ip_edit.setMaximumWidth(190)
        self.relay_ip_edit.returnPressed.connect(self.save_relay_ip)
        layout.addWidget(self.relay_ip_edit)

        self.relay_ip_refresh_button = QtWidgets.QPushButton('读取')
        self.relay_ip_refresh_button.clicked.connect(
            lambda: self.refresh_relay_ip(force=True)
        )
        layout.addWidget(self.relay_ip_refresh_button)
        self.relay_ip_save_button = QtWidgets.QPushButton('保存并应用')
        self.relay_ip_save_button.setObjectName('primaryButton')
        self.relay_ip_save_button.clicked.connect(self.save_relay_ip)
        layout.addWidget(self.relay_ip_save_button)
        return card

    def _set_relay_ip_state(self, text, state='stateIdle', detail=''):
        self.relay_ip_status.setText(text)
        self.relay_ip_status.setToolTip(detail)
        self.relay_ip_status.setObjectName(state)
        self.relay_ip_status.style().unpolish(self.relay_ip_status)
        self.relay_ip_status.style().polish(self.relay_ip_status)

    def refresh_relay_ip(self, force=False):
        try:
            content = RELAY_CONFIG_PATH.read_text(encoding='utf-8')
            address = parse_relay_ip(content)
        except Exception as error:
            self._set_relay_ip_state('读取失败', 'stateBad', str(error))
            if force:
                self._show_error(f'读取中继配置 IP 失败：{error}')
            return None
        self._set_relay_ip_state(f'当前：{address}', 'stateGood')
        if force or not self.relay_ip_edit.hasFocus():
            self.relay_ip_edit.setText(address)
        return address

    def _set_relay_ip_busy(self, busy):
        self._relay_ip_busy = bool(busy)
        self.relay_ip_edit.setEnabled(not busy)
        self.relay_ip_refresh_button.setEnabled(not busy)
        self.relay_ip_save_button.setEnabled(not busy)
        self.relay_ip_save_button.setText('应用中…' if busy else '保存并应用')

    def save_relay_ip(self):
        if self._relay_ip_busy:
            return
        try:
            requested = validate_ipv4(self.relay_ip_edit.text())
            content = RELAY_CONFIG_PATH.read_text(encoding='utf-8')
            current = parse_relay_ip(content)
        except Exception as error:
            self._show_error(f'无法保存中继配置 IP：{error}')
            return
        if requested == current:
            self._set_relay_ip_state(f'当前：{current}', 'stateGood')
            self.event_label.setText('IP 未变化，无需保存')
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            '确认修改中继配置 IP',
            f'确认将中继配置 IP 从 {current} 修改为 {requested}？\n\n'
            '保存后将自动重启 autolife-relay.service，使新 IP 立即生效。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return

        self._set_relay_ip_busy(True)
        try:
            updated, address = update_relay_ip(content, requested)
            target = RELAY_CONFIG_PATH.resolve(strict=True)
            atomic_replace_text(target, updated)
            verified = parse_relay_ip(target.read_text(encoding='utf-8'))
            if verified != address:
                raise RuntimeError('保存后的 IP 校验不一致')
        except Exception as error:
            self._set_relay_ip_state('保存失败', 'stateBad', str(error))
            self._set_relay_ip_busy(False)
            self._show_error(f'保存中继配置 IP 失败：{error}')
            return
        self.relay_ip_edit.setText(address)
        self._set_relay_ip_state(f'正在应用：{address}', 'stateWarn')
        self.output.appendPlainText(
            f'\n$ 更新中继配置 IP\n{current} -> {address}\n配置文件：{RELAY_CONFIG_PATH}\n'
        )

        def relay_restarted(success):
            self._set_relay_ip_busy(False)
            if success:
                self._set_relay_ip_state(f'当前：{address}', 'stateGood')
                self.event_label.setText(
                    f'中继配置 IP 已保存为 {address}，服务已重启并立即生效'
                )
            else:
                self._set_relay_ip_state(
                    f'已保存：{address} · 服务重启失败', 'stateBad'
                )
                self._show_error(
                    'IP 已写入配置，但 autolife-relay.service 重启失败'
                )

        self._execute_action(
            'restart', 'autolife-relay.service', relay_restarted
        )

    def _build_joint_reset_card(self):
        card = QtWidgets.QFrame()
        card.setObjectName('modeCard')
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(13, 10, 13, 10)
        layout.setSpacing(10)

        text_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel('全身关节系统复位')
        title.setObjectName('cardTitle')
        hint = QtWidgets.QLabel(
            '调用厂家官方预设默认位置；头、双臂、腰腿等关节可能同时运动'
        )
        hint.setObjectName('muted')
        hint.setWordWrap(True)
        text_box.addWidget(title)
        text_box.addWidget(hint)
        layout.addLayout(text_box, 1)

        self.joint_reset_status = QtWidgets.QLabel('待命')
        self.joint_reset_status.setObjectName('stateIdle')
        layout.addWidget(self.joint_reset_status)
        self.joint_reset_button = QtWidgets.QPushButton('全身关节复位')
        self.joint_reset_button.setObjectName('dangerButton')
        self.joint_reset_button.setToolTip(
            f'向 /control_reset_{self.topic_suffix} 发送一次厂家默认复位命令'
        )
        self.joint_reset_button.clicked.connect(self.request_joint_reset)
        layout.addWidget(self.joint_reset_button)
        return card

    def _set_joint_reset_busy(self, busy, text=None, state='stateIdle'):
        self._joint_reset_busy = bool(busy)
        self.joint_reset_button.setEnabled(not busy)
        self.joint_reset_button.setText(
            '正在发送复位…' if busy else '全身关节复位'
        )
        if text is not None:
            self.joint_reset_status.setText(text)
        self.joint_reset_status.setObjectName(state)
        self.joint_reset_status.style().unpolish(self.joint_reset_status)
        self.joint_reset_status.style().polish(self.joint_reset_status)

    def request_joint_reset(self):
        if self._joint_reset_busy:
            return
        if not self._guard_allows(
            'joint_reset', 'arm-control-service.service'
        ):
            return
        answer = QtWidgets.QMessageBox.warning(
            self,
            '确认全身关节系统复位',
            '将调用厂家官方预设默认位置，头部、双臂、腰部和腿部等关节可能同时运动。\n\n'
            '请确认机器人已停止移动、站立稳定、没有负载，周围无人且不存在 VR、抓取或'
            '其它动作控制程序。\n\n确认发送全身关节复位命令吗？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self._set_joint_reset_busy(True, '检查机械臂服务…', 'stateWarn')
        self._run_named(
            'show',
            'arm-control-service.service',
            self._joint_reset_service_checked,
        )

    def _joint_reset_service_checked(self, success, text, _code):
        state = parse_show_output(text) if success and text else {}
        if state.get('active') != 'active':
            self._set_joint_reset_busy(
                False, '机械臂服务未运行', 'stateBad'
            )
            self._show_error(
                'arm-control-service.service 未运行，拒绝发送全身关节复位命令。'
            )
            return
        if not self._guard_allows('joint_reset', 'arm-control-service.service'):
            self._set_joint_reset_busy(False, '资源已被占用', 'stateWarn')
            return
        self._set_joint_reset_busy(True, '正在发送…', 'stateWarn')
        self._run_command(
            'ros2',
            [
                'run', 'control_center_service_manager', 'joint_reset_once',
                '--ros-args', '-p', f'topic_suffix:={self.topic_suffix}',
            ],
            self._joint_reset_finished,
            label='全身关节默认复位',
        )

    def _joint_reset_finished(self, success, text, code):
        message = text.strip() or (
            '厂家默认复位命令已发送' if success else f'命令退出码 {code}'
        )
        self.output.appendPlainText(f'\n$ 全身关节系统复位\n{message}\n')
        self.log_message.emit(f'joint reset: {message}')
        if success:
            self._set_joint_reset_busy(
                False, '复位命令已发送', 'stateGood'
            )
            self.event_label.setText(
                '厂家默认复位命令已发送，请等待机器人完成动作'
            )
            QtWidgets.QMessageBox.information(
                self,
                '复位命令已发送',
                '已调用厂家系统预设默认位置。请勿触碰机器人，等待全身关节完成运动。',
            )
        else:
            self._set_joint_reset_busy(False, '发送失败', 'stateBad')
            self._show_error(message)

    def _set_navigation_mode_widgets(self, mode, detail=''):
        ours = mode == 'ours'
        vendor = mode == 'vendor'
        self.ours_navigation_button.setChecked(ours)
        self.vendor_navigation_button.setChecked(vendor)
        if mode == 'ours':
            text, object_name = '当前：我们的导航', 'stateGood'
        elif mode == 'vendor':
            text, object_name = '当前：智动导航', 'stateWarn'
        else:
            text, object_name = detail or '配置不可用', 'stateBad'
        self.navigation_mode_status.setText(text)
        self.navigation_mode_status.setToolTip(detail)
        self.navigation_mode_status.setObjectName(object_name)
        self.navigation_mode_status.style().unpolish(self.navigation_mode_status)
        self.navigation_mode_status.style().polish(self.navigation_mode_status)

    def _read_navigation_mode(self):
        try:
            content = ARM_SETTINGS_PATH.read_text(encoding='utf-8')
            match = VR_GV_SETTING_RE.search(content)
            if match is None:
                raise RuntimeError('找不到 vr_gv_control_enabled 配置项')
            return 'vendor' if match.group(1) == 'true' else 'ours'
        except Exception as error:
            self._set_navigation_mode_widgets('unknown', str(error))
            return None

    def refresh_navigation_mode(self):
        mode = self._read_navigation_mode()
        if mode is not None:
            self._set_navigation_mode_widgets(mode)

    def set_navigation_mode(self, vendor_enabled):
        if self._navigation_mode_busy:
            return
        target_mode = 'vendor' if vendor_enabled else 'ours'
        current_mode = self._read_navigation_mode()
        if current_mode == target_mode:
            self._set_navigation_mode_widgets(target_mode)
            return
        target_text = '智动导航' if vendor_enabled else '我们的导航'
        answer = QtWidgets.QMessageBox.question(
            self,
            '切换导航控制来源',
            f'确认切换为“{target_text}”并重启机械臂控制服务？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            self.refresh_navigation_mode()
            return

        if not self._guard_allows('restart', 'arm-control-service.service'):
            return
        try:
            content = ARM_SETTINGS_PATH.read_text(encoding='utf-8')
            value = 'true' if vendor_enabled else 'false'
            updated, count = VR_GV_SETTING_RE.subn(
                f'vr_gv_control_enabled = {value}', content
            )
            if count != 1:
                raise RuntimeError(
                    f'期望修改 1 个配置项，实际找到 {count} 个'
                )
            temporary = ARM_SETTINGS_PATH.with_name(
                f'.{ARM_SETTINGS_PATH.name}.control-center.tmp'
            )
            temporary.write_text(updated, encoding='utf-8')
            temporary.replace(ARM_SETTINGS_PATH)
        except Exception as error:
            self._show_error(f'切换导航控制来源失败：{error}')
            self.refresh_navigation_mode()
            return

        self._navigation_mode_busy = True
        self.ours_navigation_button.setEnabled(False)
        self.vendor_navigation_button.setEnabled(False)
        self.event_label.setText(f'正在切换为{target_text}并重启机械臂服务…')

        def restarted(success):
            self._navigation_mode_busy = False
            self.ours_navigation_button.setEnabled(True)
            self.vendor_navigation_button.setEnabled(True)
            actual_mode = self._read_navigation_mode()
            self._set_navigation_mode_widgets(actual_mode or 'unknown')
            if success and actual_mode == target_mode:
                self.event_label.setText(f'已切换为{target_text}，机械臂服务已重启')
            else:
                self._show_error(
                    f'配置已写入，但机械臂服务重启失败；请查看 {ARM_SETTINGS_PATH}'
                )

        self._execute_action(
            'restart', 'arm-control-service.service', restarted
        )

    def _build_service_browser(self):
        group = QtWidgets.QGroupBox('用户服务列表')
        layout = QtWidgets.QVBoxLayout(group)
        tools = QtWidgets.QHBoxLayout()
        self.service_filter = QtWidgets.QComboBox()
        self.service_filter.addItem('全部服务', 'all')
        self.service_filter.addItem('仅运行中', 'running')
        self.service_filter.addItem('仅已启用', 'enabled')
        self.service_filter.currentIndexChanged.connect(self._render_service_table)
        refresh = QtWidgets.QPushButton('刷新列表')
        refresh.clicked.connect(self.refresh_service_list)
        tools.addWidget(self.service_filter)
        tools.addWidget(refresh)
        tools.addStretch()
        layout.addLayout(tools)

        self.service_table = QtWidgets.QTableWidget(0, 4)
        self.service_table.setHorizontalHeaderLabels(
            ['服务名', '运行状态', '开机自启', '说明']
        )
        self.service_table.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
        self.service_table.setSelectionMode(QtWidgets.QAbstractItemView.SingleSelection)
        self.service_table.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
        self.service_table.setAlternatingRowColors(True)
        self.service_table.verticalHeader().hide()
        header = self.service_table.horizontalHeader()
        header.setSectionResizeMode(0, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(1, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(2, QtWidgets.QHeaderView.ResizeToContents)
        header.setSectionResizeMode(3, QtWidgets.QHeaderView.Stretch)
        layout.addWidget(self.service_table, 1)

        actions = QtWidgets.QHBoxLayout()
        for action, text in (
            ('start', '启动'), ('stop', '停止'), ('restart', '重启'),
            ('status', '状态详情'), ('enable', '启用自启'), ('disable', '禁用自启'),
        ):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(
                lambda _checked=False, a=action: self._selected_action(a)
            )
            actions.addWidget(button)
        layout.addLayout(actions)
        return group

    def _build_output_panel(self):
        group = QtWidgets.QGroupBox('状态与服务日志')
        layout = QtWidgets.QVBoxLayout(group)
        controls = QtWidgets.QHBoxLayout()
        self.log_service = QtWidgets.QComboBox()
        for service in CORE_SERVICES:
            self.log_service.addItem(f'{service.title} · {service.unit}', service.unit)
        recent = QtWidgets.QPushButton('最近 120 行')
        recent.clicked.connect(self.show_recent_log)
        self.follow_button = QtWidgets.QPushButton('实时跟踪')
        self.follow_button.clicked.connect(self.follow_log)
        stop = QtWidgets.QPushButton('停止跟踪')
        stop.clicked.connect(self.stop_follow_log)
        clear = QtWidgets.QPushButton('清空')
        clear.clicked.connect(lambda: self.output.clear())
        controls.addWidget(self.log_service, 1)
        controls.addWidget(recent)
        controls.addWidget(self.follow_button)
        controls.addWidget(stop)
        controls.addWidget(clear)
        layout.addLayout(controls)
        self.output = QtWidgets.QPlainTextEdit()
        self.output.setReadOnly(True)
        self.output.document().setMaximumBlockCount(4000)
        font = QtGui.QFontDatabase.systemFont(QtGui.QFontDatabase.FixedFont)
        font.setPointSize(10)
        self.output.setFont(font)
        layout.addWidget(self.output, 1)
        return group

    def _run_command(self, program, arguments, callback=None, label='command'):
        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.setProgram(program)
        process.setArguments(list(arguments))
        self._processes.add(process)
        chunks = []

        def read_output():
            data = bytes(process.readAllStandardOutput()).decode(
                'utf-8', errors='replace'
            )
            if data:
                chunks.append(data)

        def finished(code, _status):
            read_output()
            text = ''.join(chunks).strip()
            self._processes.discard(process)
            process.deleteLater()
            if callback is not None:
                callback(code == 0, text, code)

        process.readyReadStandardOutput.connect(read_output)
        process.finished.connect(finished)
        process.errorOccurred.connect(
            lambda error: self.log_message.emit(f'{label} 进程错误：{error}')
        )
        process.start()
        return process

    def _run_named(self, action, unit=None, callback=None):
        try:
            program, arguments = command_for(action, unit)
        except ValueError as error:
            self._show_error(str(error))
            return None
        return self._run_command(
            program, arguments, callback,
            label=f'{action} {unit or ""}'.strip(),
        )

    def _show_error(self, message):
        self.event_label.setText(message)
        QtWidgets.QMessageBox.warning(self, '服务操作失败', message)

    def _guard_allows(self, action, unit):
        if self.action_guard is None:
            return True
        try:
            result = self.action_guard(action, unit)
        except Exception as error:
            self._show_error(f'服务安全检查失败：{error}')
            return False
        if isinstance(result, tuple):
            allowed, message = result
        else:
            allowed, message = bool(result), ''
        if not allowed:
            self._show_error(message or '当前控制中心状态不允许此操作。')
        return bool(allowed)

    def request_action(self, action, unit):
        try:
            unit = validate_service_name(unit)
        except ValueError as error:
            self._show_error(str(error))
            return
        if action == 'status':
            self.show_status(unit)
            return
        if action == 'start' and unit == 'gv-control-service.service':
            self.start_navigation_stack()
            return
        if action in ('stop', 'restart', 'disable'):
            verbs = {'stop': '停止', 'restart': '重启', 'disable': '禁用开机自启'}
            answer = QtWidgets.QMessageBox.question(
                self, '确认服务操作', f'确认{verbs[action]} {unit}？',
                QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
                QtWidgets.QMessageBox.No,
            )
            if answer != QtWidgets.QMessageBox.Yes:
                return
        self._execute_action(action, unit)

    def _execute_action(self, action, unit, done=None):
        if not self._guard_allows(action, unit):
            if done is not None:
                done(False)
            return
        self.event_label.setText(f'正在执行：{action} {unit}')

        def completed(success, text, code):
            message = text or ('操作成功' if success else f'命令退出码 {code}')
            self.event_label.setText(f'{unit}：{message.splitlines()[-1]}')
            self.output.appendPlainText(
                f'\n$ systemctl --user {action} {unit}\n{message}\n'
            )
            self.log_message.emit(f'{action} {unit}: {message}')
            QtCore.QTimer.singleShot(250, self.refresh_all)
            if done is not None:
                done(success)

        self._run_named(action, unit, completed)

    def show_status(self, unit):
        self.output.appendPlainText(f'\n$ systemctl --user status {unit}\n')

        def completed(_success, text, _code):
            self.output.appendPlainText(text or '没有状态输出')
            self.output.moveCursor(QtGui.QTextCursor.End)

        self._run_named('status', unit, completed)

    def start_navigation_stack(self):
        self._execute_action(
            'start', 'gv-slam-service.service',
            lambda success: self._execute_action(
                'start', 'gv-control-service.service'
            ) if success else None,
        )

    def stop_navigation_stack(self):
        answer = QtWidgets.QMessageBox.question(
            self, '停止导航组合',
            '将依次停止导航控制和 SLAM 服务，是否继续？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self._execute_action(
            'stop', 'gv-control-service.service',
            lambda success: self._execute_action(
                'stop', 'gv-slam-service.service'
            ) if success else None,
        )

    def refresh_all(self):
        self.refresh_navigation_mode()
        self.refresh_voice_feedback()
        self.refresh_voice_volume()
        self.refresh_logo_display()
        self.refresh_relay_ip()
        self.refresh_core_services()
        self.refresh_service_list()

    def refresh_core_services(self):
        for unit, card in self._core_cards.items():
            self._run_named(
                'show', unit,
                lambda success, text, code, u=unit, c=card:
                    self._core_state_received(u, c, success, text, code),
            )

    def _core_state_received(self, unit, card, success, text, _code):
        state = parse_show_output(text) if text else {
            'active': 'unknown' if success else 'not-found',
            'sub': 'unknown', 'enabled': 'unknown',
        }
        card.apply_state(state)
        self.service_state_changed.emit(
            unit,
            str(state.get('active', 'unknown')),
            str(state.get('enabled', 'unknown')),
        )

    def refresh_service_list(self):
        self._run_named('list-unit-files', callback=self._unit_files_received)
        self._run_named('list-running', callback=self._running_units_received)

    def _unit_files_received(self, _success, text, _code):
        self._unit_files = parse_unit_files(text)
        self._render_service_table()
        self._refresh_log_service_choices()

    def _running_units_received(self, _success, text, _code):
        self._running_units = parse_running_units(text)
        self._render_service_table()

    def _render_service_table(self, *_unused):
        mode = self.service_filter.currentData() or 'all'
        units = set(self._unit_files) | set(self._running_units)
        if mode == 'running':
            units &= set(self._running_units)
        elif mode == 'enabled':
            units = {
                unit for unit in units
                if self._unit_files.get(unit) in ('enabled', 'enabled-runtime')
            }
        selected = self._selected_unit()
        self.service_table.setRowCount(0)
        for unit in sorted(units):
            row = self.service_table.rowCount()
            self.service_table.insertRow(row)
            active = '运行中' if unit in self._running_units else '已停止'
            enabled = self._unit_files.get(unit, 'unknown')
            description = self._running_units.get(unit, '')
            for column, value in enumerate((unit, active, enabled, description)):
                item = QtWidgets.QTableWidgetItem(str(value))
                if column == 0:
                    item.setData(QtCore.Qt.UserRole, unit)
                self.service_table.setItem(row, column, item)
                if unit == selected:
                    self.service_table.selectRow(row)

    def _refresh_log_service_choices(self):
        current = str(self.log_service.currentData() or '')
        core = {service.unit: service.title for service in CORE_SERVICES}
        units = sorted(set(self._unit_files) | set(core))
        self.log_service.blockSignals(True)
        self.log_service.clear()
        selected_index = 0
        for index, unit in enumerate(units):
            label = f'{core[unit]} · {unit}' if unit in core else unit
            self.log_service.addItem(label, unit)
            if unit == current:
                selected_index = index
        if self.log_service.count():
            self.log_service.setCurrentIndex(selected_index)
        self.log_service.blockSignals(False)

    def _selected_unit(self):
        row = self.service_table.currentRow()
        if row < 0:
            return ''
        item = self.service_table.item(row, 0)
        return str(item.data(QtCore.Qt.UserRole) or item.text()) if item else ''

    def _selected_action(self, action):
        unit = self._selected_unit()
        if not unit:
            QtWidgets.QMessageBox.information(
                self, '请选择服务', '请先在服务列表中选择一行。'
            )
            return
        self.request_action(action, unit)

    def show_recent_log(self):
        unit = str(self.log_service.currentData() or '')
        if not unit:
            return
        self.stop_follow_log()
        self.output.clear()
        self.output.appendPlainText(f'$ journalctl --user -u {unit} -n 120\n')
        self._run_named(
            'journal-recent', unit,
            lambda _success, text, _code: self.output.appendPlainText(
                text or '没有日志记录'
            ),
        )

    def follow_log(self):
        unit = str(self.log_service.currentData() or '')
        if not unit:
            return
        self.stop_follow_log()
        program, arguments = command_for('journal-follow', unit)
        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.setProgram(program)
        process.setArguments(arguments)
        process.readyReadStandardOutput.connect(self._read_follow_log)
        process.finished.connect(lambda *_args: self._follow_finished(process))
        self._log_process = process
        self.output.appendPlainText(f'\n$ journalctl --user -u {unit} -f\n')
        self.follow_button.setEnabled(False)
        process.start()

    def _read_follow_log(self):
        if self._log_process is None:
            return
        text = bytes(self._log_process.readAllStandardOutput()).decode(
            'utf-8', errors='replace'
        )
        if text:
            self.output.moveCursor(QtGui.QTextCursor.End)
            self.output.insertPlainText(text)
            self.output.moveCursor(QtGui.QTextCursor.End)

    def _follow_finished(self, process):
        if self._log_process is process:
            self._read_follow_log()
            self._log_process = None
            self.follow_button.setEnabled(True)
        process.deleteLater()

    def stop_follow_log(self):
        process = self._log_process
        if process is None:
            self.follow_button.setEnabled(True)
            return
        self._log_process = None
        process.terminate()
        if not process.waitForFinished(700):
            process.kill()
            process.waitForFinished(300)
        process.deleteLater()
        self.follow_button.setEnabled(True)

    def shutdown(self):
        self.refresh_timer.stop()
        worker = getattr(self, '_voice_chain_worker', None)
        if worker is not None and worker.isRunning():
            worker.wait(15000)
        self.stop_follow_log()
        for process in list(self._processes):
            if process.state() != QtCore.QProcess.NotRunning:
                process.terminate()
                if not process.waitForFinished(400):
                    process.kill()
            process.deleteLater()
        self._processes.clear()

    def closeEvent(self, event):
        self.shutdown()
        event.accept()


def main():
    application = QtWidgets.QApplication([sys.argv[0]])
    application.setApplicationName('系统用户服务')
    panel = ServiceManagerPanel()
    panel.resize(1440, 900)
    signal.signal(signal.SIGINT, lambda *_args: panel.close())
    timer = QtCore.QTimer()
    timer.start(250)
    timer.timeout.connect(lambda: None)
    panel.show()
    return application.exec_()


if __name__ == '__main__':
    raise SystemExit(main())
