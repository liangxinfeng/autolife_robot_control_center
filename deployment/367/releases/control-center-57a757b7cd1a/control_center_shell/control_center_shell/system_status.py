"""A lazy, read-only Qt status page for robot 347."""

from queue import Empty, Queue
import os
from pathlib import Path
import re
from threading import Event, Thread
import time

from PyQt5 import QtCore, QtGui, QtWidgets

from .system_status_metrics import Sampler


def _percent(value):
    return "—" if value is None else f"{value:.1f}%"


def _size(value):
    if value is None:
        return "—"
    if value >= 1024 ** 3:
        return f"{value / 1024 ** 3:.1f} GiB"
    return f"{value / 1024 ** 2:.1f} MiB"


def _rate(value, network=False):
    if value is None:
        return "—"
    if network:
        return f"{value * 8 / 1_000_000:.3f} Mbps"
    if value >= 1024 ** 2:
        return f"{value / 1024 ** 2:.2f} MiB/s"
    return f"{value / 1024:.1f} KiB/s"


def _collect(sampler, cancelled, mailbox):
    # Worker owns no Qt objects; a late result after a page switch is discarded.
    try:
        result = sampler.sample(cancelled)
        message = result, None
    except Exception as error:
        message = None, type(error).__name__
    if not cancelled.is_set():
        mailbox.put_nowait(message)


def _unit_from_cgroup(data):
    groups = [line.split(":", 2) for line in data.splitlines()]
    group = next((p[2] for p in groups if len(p) == 3 and p[0] == "0" and not p[1]), "")
    if not group:
        group = next((p[2] for p in groups if len(p) == 3 and p[1] == "name=systemd"), "")
    parts = group.split("/")
    service = next((p for p in reversed(parts)
                    if p.endswith(".service") and not re.fullmatch(r"user@[0-9]+\.service", p)), "")
    scope = next((p for p in reversed(parts) if p.endswith(".scope")), "")
    kind = "用户服务" if any(re.fullmatch(r"user@[0-9]+\.service", p) for p in parts) else "系统服务"
    return service, kind, scope, group


def _read_pid_entry(fd, name, limit):
    with os.fdopen(os.open(name, os.O_RDONLY, dir_fd=fd), "rb") as stream:
        return stream.read(limit).decode("utf-8", errors="replace")


def _entrypoint(comm, exe, cmdline):
    executable = Path(exe).name if exe else comm
    args = cmdline.split("\0")
    if len(args) < 3:
        return ""
    target = args[1]
    if re.fullmatch(r"python(?:[0-9.]+)?", executable) and target == "-m":
        module = args[2]
        if len(args) >= 4 and re.fullmatch(r"[A-Za-z_]\w*(?:\.[A-Za-z_]\w*)*", module):
            return f"模块：{module}"
    elif (re.fullmatch(r"python(?:[0-9.]+)?|bash|sh|dash", executable)
          and re.fullmatch(r"[A-Za-z0-9_./-]+\.(?:py|sh)", target) and not target.startswith("-")):
        return f"脚本：{target}"
    return ""


def _pid_description(pid, proc_root=Path("/proc")):
    if not 1 <= pid <= 2147483647:
        return "请输入有效的正整数 PID。"
    # Use one proc directory handle so all fields refer to the same process.
    try:
        fd = os.open(proc_root / str(pid), os.O_RDONLY | os.O_DIRECTORY)
        try:
            stat = _read_pid_entry(fd, "stat", 4096)
            end = stat.rindex(")")
            name = stat[stat.index("(") + 1:end]
            parent = int(stat[end + 2:].split()[1])
            try:
                cgroup = _read_pid_entry(fd, "cgroup", 8192)
            except PermissionError:
                cgroup = None
            try:
                exe = os.readlink("exe", dir_fd=fd)
            except OSError:
                exe = ""
            try:
                entrypoint = _entrypoint(name, exe, _read_pid_entry(fd, "cmdline", 4096))
            except PermissionError:
                entrypoint = ""
        finally:
            os.close(fd)
    except (FileNotFoundError, ProcessLookupError):
        return f"没有找到 PID {pid}，进程可能已经退出。"
    except PermissionError:
        return f"当前用户无权读取 PID {pid} 的进程信息。"
    except (OSError, IndexError, ValueError):
        return f"暂时无法读取 PID {pid} 的进程信息，请重试。"

    def display(value):
        return "".join(ch if ch.isprintable() else " " for ch in value)

    service, kind, scope, group = _unit_from_cgroup(cgroup or "")
    unit = f"{service}（{kind}）" if service else "未关联到具体服务"
    if cgroup is None:
        unit = "权限不足，无法判断"
    lines = [f"进程：{display(name)}", f"PID：{pid}    父 PID：{parent}",
             f"所属服务：{display(unit)}"]
    if scope:
        lines.append(f"作用域：{display(scope)}")
    if entrypoint:
        lines.append(display(entrypoint))
    lines.append(f"可执行文件：{display(exe) if exe else '不可读取'}")
    if group:
        lines.append(f"控制组：{display(group)}")
    return "\n".join(lines)


class PidQueryDialog(QtWidgets.QDialog):
    def __init__(self, parent, initial=""):
        super().__init__(parent)
        self.setWindowTitle("查询 PID")
        self.setMinimumWidth(530)
        layout = QtWidgets.QVBoxLayout(self)
        row = QtWidgets.QHBoxLayout()
        prompt = QtWidgets.QLabel("进程 PID")
        self.pid = QtWidgets.QLineEdit(initial)
        self.pid.setMaxLength(32)
        self.pid.setPlaceholderText("输入进程榜中的 PID")
        prompt.setBuddy(self.pid)
        query = QtWidgets.QPushButton("查询")
        query.setDefault(True)
        row.addWidget(prompt)
        row.addWidget(self.pid, 1)
        row.addWidget(query)
        layout.addLayout(row)
        self.result = QtWidgets.QPlainTextEdit()
        self.result.setReadOnly(True)
        self.result.setMinimumHeight(175)
        self.result.setPlaceholderText("查询结果会显示在这里，可选中复制。")
        layout.addWidget(self.result)
        close = QtWidgets.QPushButton("关闭")
        close.setAutoDefault(False)
        close.clicked.connect(self.reject)
        layout.addWidget(close, 0, QtCore.Qt.AlignRight)
        query.clicked.connect(self._query)
        self.pid.setFocus()
        self.pid.selectAll()

    def _query(self):
        value = self.pid.text().strip()
        text = (_pid_description(int(value)) if re.fullmatch(r"[0-9]{1,10}", value)
                else "请输入有效的正整数 PID。")
        self.result.setPlainText(text)


class Rows(QtCore.QAbstractTableModel):
    def __init__(self, headers, parent):
        super().__init__(parent)
        self.headers = headers
        self.rows = []

    def rowCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() else len(self.rows)

    def columnCount(self, parent=QtCore.QModelIndex()):
        return 0 if parent.isValid() else len(self.headers)

    def headerData(self, section, orientation, role=QtCore.Qt.DisplayRole):
        if role == QtCore.Qt.DisplayRole and orientation == QtCore.Qt.Horizontal:
            return self.headers[section]
        return None

    def data(self, index, role=QtCore.Qt.DisplayRole):
        if index.isValid() and role == QtCore.Qt.DisplayRole:
            return str(self.rows[index.row()][index.column()])
        return None

    def replace(self, rows):
        if rows != self.rows:
            self.beginResetModel()
            self.rows = rows
            self.endResetModel()


class UsageRing(QtWidgets.QWidget):
    """Used portion of a circle; numeric label remains readable without color."""

    def __init__(self, title, color, parent):
        super().__init__(parent)
        self._color = QtGui.QColor(color)
        self._value = None
        self.setFixedSize(84, 84)
        self.setStyleSheet("background: transparent;")
        self.setAccessibleName(f"{title}占用率")

    def set_value(self, value):
        self.setAccessibleDescription("采样中" if value is None else _percent(value))
        if value != self._value:
            self._value = value
            self.update()

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        circle = QtCore.QRectF(self.rect()).adjusted(7, 7, -7, -7)
        painter.setPen(QtGui.QPen(QtGui.QColor("#394657"), 8))
        painter.drawEllipse(circle)
        if self._value is not None and self._value > 0:
            painter.setPen(QtGui.QPen(self._color, 8, QtCore.Qt.SolidLine, QtCore.Qt.FlatCap))
            if self._value >= 100:
                painter.drawEllipse(circle)
            else:
                painter.drawArc(circle, 90 * 16, -round(self._value * 360 * 16 / 100))
        font = painter.font()
        font.setPixelSize(18)
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QtGui.QColor("#f2f6fc"))
        painter.drawText(self.rect(), QtCore.Qt.AlignCenter, _percent(self._value))


class SystemStatusPage(QtWidgets.QWidget):
    def __init__(self, parent):
        super().__init__(parent)
        self._window = self.window()
        self._active = False
        self._closed = False
        self._worker = None
        self._cancelled = None
        self._mailbox = None
        self._snapshot = None
        self._last_at = None
        self._sampler = None
        self._process_views = []
        self._timer = QtCore.QTimer(self)
        self._timer.setInterval(2000)
        self._timer.timeout.connect(self._request)
        self._poll = QtCore.QTimer(self)
        self._poll.setInterval(100)
        self._poll.timeout.connect(self._receive)

        layout = QtWidgets.QVBoxLayout(self)
        cards = QtWidgets.QHBoxLayout()
        cards.setSpacing(10)
        self.cpu_ring, self.cpu_detail = self._add_ring_card(cards, "CPU", "#54c8f2")
        self.mem_ring, self.mem_detail = self._add_ring_card(cards, "内存", "#b6a4ff")
        quick = self._make_card()
        quick_layout = QtWidgets.QVBoxLayout(quick)
        quick_layout.setContentsMargins(14, 8, 14, 8)
        quick_layout.setSpacing(2)
        quick_layout.addWidget(self._card_label("磁盘与网络", title=True))
        self.disk_summary = self._card_label("系统盘 —")
        self.net_summary = self._card_label("最活跃接口 —")
        quick_layout.addWidget(self.disk_summary)
        quick_layout.addWidget(self.net_summary)
        cards.addWidget(quick, 3)
        layout.addLayout(cards)
        self.status = QtWidgets.QLabel("等待采样")
        self.status.setTextFormat(QtCore.Qt.PlainText)
        self.status.setWordWrap(True)
        toolbar = QtWidgets.QHBoxLayout()
        toolbar.addWidget(self.status, 1)
        self.pid_query = QtWidgets.QPushButton("查询 PID")
        self.pid_query.setToolTip("按进程 ID 查看所属服务与启动模块")
        self.pid_query.clicked.connect(self._show_pid_query)
        toolbar.addWidget(self.pid_query)
        layout.addLayout(toolbar)
        self.views = QtWidgets.QTabWidget()
        layout.addWidget(self.views, 1)
        self.cpu = self._add_view("CPU", [
            ("进程排行 · 整机算力为 100%", ("PID", "进程", "CPU 占用率"), 1),
        ])[0]
        self.mem = self._add_view("内存", [
            ("进程排行 · 按驻留内存（RSS）排序，可能包含共享内存",
             ("PID", "进程", "驻留内存", "占总内存"), 1),
        ])[0]
        self.fs, self.disk, self.io = self._add_view("磁盘", [
            ("文件系统空间", ("挂载点", "设备", "使用率", "已用", "可用"), 1),
            ("物理磁盘 · 忙碌率表示采样期间处理 I/O 的时间比例",
             ("设备", "I/O 忙碌率", "读取", "写入"), 1),
            ("进程磁盘读写排行 · 仅统计当前用户可读取的数据",
             ("PID", "进程", "读取", "写入"), 2),
        ])
        self.net, self.other = self._add_view("网络", [
            ("有线全双工网口 · 占用率取收发两方向中较高值",
             ("网口", "接收", "发送", "占用率"), 1),
            ("未知链路速率或虚拟接口 · 按收发总量排序",
             ("接口", "接收", "发送"), 1),
        ])
        self.views.currentChanged.connect(self._render)
        self._window.installEventFilter(self)

    def _show_pid_query(self):
        initial = ""
        for view in self._process_views:
            if view.isVisible():
                selected = view.selectionModel().selectedRows(0)
                if selected:
                    initial = str(selected[0].data())
                    break
        dialog = PidQueryDialog(self, initial)
        try:
            dialog.exec_()
        finally:
            dialog.deleteLater()

    def _make_card(self):
        card = QtWidgets.QFrame(self)
        card.setObjectName("systemResourceCard")
        card.setMinimumHeight(102)
        card.setStyleSheet(
            "QFrame#systemResourceCard { background: #29323f; border: 1px solid #3a4656;"
            " border-radius: 9px; }"
            "QLabel#resourceTitle { background: transparent; color: #f2f6fc;"
            " font-size: 17px; font-weight: 700; }"
            "QLabel#resourceDetail { background: transparent; color: #bbcadb; }")
        return card

    def _card_label(self, text, title=False):
        label = QtWidgets.QLabel(text)
        label.setObjectName("resourceTitle" if title else "resourceDetail")
        label.setTextFormat(QtCore.Qt.PlainText)
        label.setWordWrap(True)
        return label

    def _add_ring_card(self, cards, title, color):
        card = self._make_card()
        layout = QtWidgets.QHBoxLayout(card)
        layout.setContentsMargins(13, 7, 12, 7)
        layout.setSpacing(12)
        ring = UsageRing(title, color, card)
        layout.addWidget(ring, 0, QtCore.Qt.AlignVCenter)
        text = QtWidgets.QVBoxLayout()
        text.setSpacing(2)
        text.addStretch(1)
        text.addWidget(self._card_label(title, title=True))
        detail = self._card_label("等待采样")
        text.addWidget(detail)
        text.addStretch(1)
        layout.addLayout(text, 1)
        cards.addWidget(card, 2)
        return ring, detail

    def _add_view(self, title, sections):
        content = QtWidgets.QWidget()
        layout = QtWidgets.QVBoxLayout(content)
        models = []
        for subtitle, headers, stretch in sections:
            label = QtWidgets.QLabel(subtitle)
            label.setWordWrap(True)
            layout.addWidget(label)
            if title == "磁盘" and stretch == 2:
                self.io_coverage = label
            view = QtWidgets.QTableView()
            view.setEditTriggers(QtWidgets.QAbstractItemView.NoEditTriggers)
            view.setSelectionBehavior(QtWidgets.QAbstractItemView.SelectRows)
            view.setAlternatingRowColors(True)
            view.setStyleSheet(
                "QTableView { alternate-background-color: #2b3340; "
                "selection-background-color: #285479; selection-color: #f2f6fc; }")
            view.verticalHeader().hide()
            view.verticalHeader().setDefaultSectionSize(25)
            view.horizontalHeader().setSectionResizeMode(QtWidgets.QHeaderView.Stretch)
            model = Rows(headers, view)
            view.setModel(model)
            if headers[0] == "PID":
                self._process_views.append(view)
            models.append(model)
            layout.addWidget(view, stretch)
        self.views.addTab(content, title)
        return models

    def _sync(self):
        active = not self._closed and self.isVisible() and not self._window.isMinimized()
        if active == self._active:
            return
        self._active = active
        if active:
            self._sampler = Sampler()
            self._snapshot = self._last_at = None
            for model in (self.cpu, self.mem, self.fs, self.disk, self.io, self.net, self.other):
                model.replace([])
            self.cpu_ring.set_value(None)
            self.mem_ring.set_value(None)
            self.cpu_detail.setText("等待采样")
            self.mem_detail.setText("等待采样")
            self.disk_summary.setText("系统盘 —")
            self.net_summary.setText("最活跃接口 —")
            self.status.setText("正在采样，占用率将在下一次采样后显示。")
            self._timer.start()
            self._request()
        else:
            self._timer.stop()
            self._poll.stop()
            if self._cancelled is not None:
                self._cancelled.set()
            self._sampler = self._mailbox = None

    def eventFilter(self, obj, event):
        if obj is self._window and event.type() == QtCore.QEvent.WindowStateChange:
            self._sync()
        return super().eventFilter(obj, event)

    def showEvent(self, event):
        super().showEvent(event)
        self._sync()

    def hideEvent(self, event):
        super().hideEvent(event)
        self._sync()

    def shutdown(self):
        self._closed = True
        self._sync()
        self._window.removeEventFilter(self)

    def _request(self):
        if not self._active:
            return
        if self._last_at is not None and time.monotonic() - self._last_at > 6:
            self.status.setText("采样延迟，页面显示上一次结果。")
        if self._mailbox is not None or (self._worker is not None and self._worker.is_alive()):
            return
        self._mailbox = Queue(maxsize=1)
        self._cancelled = Event()
        self._worker = Thread(target=_collect, args=(self._sampler, self._cancelled, self._mailbox),
                              name="system-status-sample", daemon=True)
        try:
            self._worker.start()
        except RuntimeError:
            self._worker = self._mailbox = None
            self.status.setText("暂时无法启动采样，下轮重试。")
            return
        self._poll.start()

    def _receive(self):
        if self._mailbox is None:
            return
        try:
            snapshot, error = self._mailbox.get_nowait()
        except Empty:
            return
        self._poll.stop()
        self._mailbox = None
        if error:
            self.status.setText(f"本轮读取失败（{error}），下轮重试；旧数据保留。")
        elif snapshot:
            self._snapshot = snapshot
            self._last_at = time.monotonic()
            self._render()

    def _render(self):
        data = self._snapshot
        if data is None:
            return
        used, total = data["memory"]
        root = next((row for row in data["filesystems"] if row[0] == "/"), None)
        interfaces = data["net_known"] + data["net_other"]
        top_link = max((row for row in interfaces if row[1] is not None),
                       key=lambda row: row[1] + row[2], default=None)
        net = ("—" if top_link is None else
               f"{top_link[0]} ↓{_rate(top_link[1], True)} ↑{_rate(top_link[2], True)}")
        self.cpu_ring.set_value(data["cpu"])
        self.mem_ring.set_value(100 * used / total)
        self.cpu_detail.setText(f"{data['cores']} 个逻辑核")
        self.mem_detail.setText(f"已用 {_size(used)} / {_size(total)}")
        self.disk_summary.setText(f"系统盘  {_percent(root[2] if root else None)}")
        self.net_summary.setText(f"最活跃接口  {net}")
        self.status.setText("每 2 秒刷新 · 切离或最小化暂停" if data["cpu"] is not None
                            else "正在建立采样基线，占用率将在下次更新后显示。")
        tab = self.views.currentIndex()
        if tab == 0:
            self.cpu.replace([(pid, name, _percent(pct)) for pid, name, pct in data["cpu_rows"]])
        elif tab == 1:
            self.mem.replace([(pid, name, _size(size), _percent(pct))
                              for pid, name, size, pct in data["memory_rows"]])
        elif tab == 2:
            self.fs.replace([(mount, dev, _percent(pct), _size(used), _size(free))
                             for mount, dev, pct, used, free in data["filesystems"]])
            self.disk.replace([(name, _percent(pct), _rate(rd), _rate(wr))
                               for name, pct, rd, wr in data["disks"]])
            read, count = data["io_coverage"]
            suffix = "" if data["io_ready"] else "；等待第二轮 I/O 数据"
            self.io_coverage.setText(
                f"进程磁盘读写排行 · 可读 {read}/{count}，其余不可读取{suffix}")
            self.io.replace([(pid, name, _rate(rd), _rate(wr))
                             for pid, name, rd, wr in data["io_rows"]])
        elif tab == 3:
            self.net.replace([(name, _rate(rx, True), _rate(tx, True), _percent(pct))
                              for name, rx, tx, pct in data["net_known"]])
            self.other.replace([(name, _rate(rx, True), _rate(tx, True))
                                for name, rx, tx in data["net_other"]])
