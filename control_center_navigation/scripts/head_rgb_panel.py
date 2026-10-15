"""Optional, bounded-cost RGB viewer. Never opens/restarts the camera hardware."""
import importlib.util
import os
from pathlib import Path
import sys
import threading
import time

from PyQt5 import QtCore, QtGui, QtWidgets


def _head_consumer():
    # Load only the ABI-compatible vendor extensions, not its Conda Qt/NumPy.
    vendor = Path(os.environ.get(
        'AUTOLIFE_CAMERA_PYTHON_PATH',
        str(Path.home() / 'miniconda3/envs/robot_env/lib/python3.12/site-packages'),
    ))
    if importlib.util.find_spec('posix_ipc') is None:
        candidates = sorted(vendor.glob('posix_ipc*.so'))
        if not candidates:
            raise RuntimeError('缺少相机共享内存依赖 posix_ipc')
        spec = importlib.util.spec_from_file_location('posix_ipc', candidates[0])
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        sys.modules['posix_ipc'] = module
    candidates = sorted((vendor / 'autolife_robot_sdk/utils').glob('camera_shm.*.so'))
    if not candidates:
        raise RuntimeError('未找到系统相机共享内存适配器')
    spec = importlib.util.spec_from_file_location('camera_shm', candidates[0])
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    suffix = 'rgbd_head_color'
    options = {
        'data_shm_name': f'/camera_image_buffer_{suffix}',
        'meta_shm_name': f'/camera_metadata_struct_{suffix}',
        'read_mutex_name': f'/read_mutex_lock_{suffix}',
        'write_lock_name': f'/write_data_lock_{suffix}',
        'writer_queue_name': f'/writer_queue_sem_{suffix}',
        'intrinsics_shm_name': f'/camera_intrinsics_struct_{suffix}',
    }
    return module.SHMCameraFrameConsumer(
        {'selected_protocol': 'v1', 'protocols': {'v1': options}},
        name='navigation_rgb_readonly',
    )


class LatestHeadRGB:
    """One worker and one latest frame; no queued Qt frame signals or ROS load."""
    def __init__(self, factory=_head_consumer):
        self.factory = factory
        self._lock = threading.Lock()
        self._enabled = threading.Event()
        self._quit = threading.Event()
        self._thread = None
        self._latest = None
        self._error = '正在连接头部RGB…'

    def set_enabled(self, enabled):
        with self._lock:
            self._latest = None
            self._error = '正在连接头部RGB…' if enabled else 'RGB已暂停'
            if enabled and not self._quit.is_set():
                self._enabled.set()
                if self._thread is None:
                    self._thread = threading.Thread(
                        target=self._run, name='navigation-head-rgb', daemon=True)
                    self._thread.start()
            else:
                self._enabled.clear()

    def snapshot(self):
        with self._lock:
            return self._latest, self._error

    def _run(self):
        consumer = None
        last_id = None
        try:
            import cv2
            import numpy as np
            while not self._quit.is_set():
                if not self._enabled.is_set():
                    if consumer is not None:
                        consumer.close()
                        consumer = None
                    self._enabled.wait(.20)
                    continue
                try:
                    if consumer is None:
                        consumer = self.factory()
                        last_id = None
                    pair = consumer.get_latest(nonblock=True)
                    if pair is not None and pair[1] != last_id:
                        bgr, frame_id = pair
                        if bgr.ndim != 3 or bgr.shape[2] != 3:
                            raise ValueError('头部RGB图像格式无效')
                        # Cap preview work, without changing capture resolution.
                        if bgr.shape[1] > 1280:
                            bgr = cv2.resize(bgr, (1280, round(bgr.shape[0] * 1280 / bgr.shape[1])),
                                             interpolation=cv2.INTER_AREA)
                        rgb = np.ascontiguousarray(cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB))
                        stamp = time.monotonic()
                        last_id = frame_id
                        with self._lock:
                            if self._enabled.is_set() and not self._quit.is_set():
                                self._latest = (rgb, stamp, frame_id)
                                self._error = ''
                    self._quit.wait(.08)  # <= 12.5 fps, independent of navigation callbacks
                except Exception as error:
                    with self._lock:
                        self._latest = None
                        self._error = 'RGB暂不可用，请检查视觉服务：' + str(error)[:160]
                    if consumer is not None:
                        try:
                            consumer.close()
                        except Exception:
                            pass
                        consumer = None
                    self._quit.wait(1.0)
        except Exception as error:
            with self._lock:
                self._latest = None
                self._error = 'RGB读取失败：' + str(error)[:160]
        finally:
            if consumer is not None:
                try:
                    consumer.close()
                except Exception:
                    pass

    def close(self):
        self._quit.set()
        self._enabled.set()  # wake a paused worker
        if self._thread is not None:
            self._thread.join(timeout=.35)


class RGBViewport(QtWidgets.QWidget):
    focus_lost = QtCore.pyqtSignal()
    enlarge_requested = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.setMinimumSize(280, 210)
        self.image = None
        self.message = '正在连接头部RGB…'

    def sizeHint(self):
        return QtCore.QSize(640, 480)

    def paintEvent(self, event):
        painter = QtGui.QPainter(self)
        painter.fillRect(self.rect(), QtGui.QColor('#090d12'))
        if self.image is not None:
            size = self.image.size().scaled(self.size(), QtCore.Qt.KeepAspectRatio)
            rect = QtCore.QRect(QtCore.QPoint(0, 0), size)
            rect.moveCenter(self.rect().center())
            painter.drawImage(rect, self.image)
        else:
            painter.setPen(QtGui.QColor('#c3ceda'))
            painter.drawText(self.rect().adjusted(20, 20, -20, -20),
                             QtCore.Qt.AlignCenter | QtCore.Qt.TextWordWrap, self.message)

    def mousePressEvent(self, event):
        self.setFocus(QtCore.Qt.MouseFocusReason)
        event.accept()

    def mouseDoubleClickEvent(self, event):
        self.enlarge_requested.emit()
        event.accept()

    def focusOutEvent(self, event):
        self.focus_lost.emit()
        super().focusOutEvent(event)


class HeadRGBPanel(QtWidgets.QWidget):
    enlarge_requested = QtCore.pyqtSignal()
    focus_lost = QtCore.pyqtSignal()

    def __init__(self, parent=None, source=None):
        super().__init__(parent)
        self.source = source if source is not None else LatestHeadRGB()
        self._last_stamp = None
        self._closed = False
        self.setMinimumWidth(300)
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        bar = QtWidgets.QHBoxLayout()
        self.status = QtWidgets.QLabel('头部 RGB')
        self.status.setMinimumWidth(0)
        bar.addWidget(self.status, 1)
        self.enlarge_button = QtWidgets.QToolButton()
        self.enlarge_button.setText('放大画面')
        self.enlarge_button.setFocusPolicy(QtCore.Qt.NoFocus)
        self.enlarge_button.clicked.connect(self.enlarge_requested)
        bar.addWidget(self.enlarge_button)
        layout.addLayout(bar)
        self.viewport = RGBViewport()
        self.viewport.focus_lost.connect(self.focus_lost)
        self.viewport.enlarge_requested.connect(self.enlarge_requested)
        layout.addWidget(self.viewport, 1)
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(80)
        self.timer.timeout.connect(self.refresh_frame)
        QtWidgets.QApplication.instance().aboutToQuit.connect(self.shutdown)

    def refresh_frame(self):
        snapshot, error = self.source.snapshot()
        if snapshot is None or time.monotonic() - snapshot[1] > 1.0:
            self.viewport.image = None
            self._last_stamp = None
            self.viewport.message = error or '画面已中断，请检查视觉服务'
            self.status.setText('头部 RGB · 等待画面')
            self.viewport.update()
            return
        rgb, stamp, frame_id = snapshot
        if stamp == self._last_stamp:
            return
        self._last_stamp = stamp
        height, width = rgb.shape[:2]
        self.viewport.image = QtGui.QImage(
            rgb.data, width, height, rgb.strides[0], QtGui.QImage.Format_RGB888).copy()
        self.status.setText(f'头部 RGB · {width}×{height}')
        self.viewport.update()

    def showEvent(self, event):
        if not self._closed:
            self._last_stamp = None
            self.source.set_enabled(True)
            self.timer.start()
        super().showEvent(event)

    def hideEvent(self, event):
        self.timer.stop()
        self.source.set_enabled(False)
        self.viewport.image = None
        super().hideEvent(event)

    def shutdown(self):
        if not self._closed:
            self._closed = True
            self.timer.stop()
            self.source.close()
