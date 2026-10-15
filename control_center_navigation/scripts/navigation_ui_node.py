#!/usr/bin/env python3
"""Standalone Qt navigation interface; no RViz runtime is required."""

import json
import importlib.util
import math
from pathlib import Path as FilePath
import re
import shutil
import sys
import threading
import time
from functools import partial

import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import yaml

from ament_index_python.packages import get_package_share_directory
import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped, Twist
from nav2_msgs.srv import LoadMap
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.duration import Duration
from rclpy.executors import SingleThreadedExecutor
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from std_srvs.srv import Trigger, Empty
import tf2_ros
from visualization_msgs.msg import Marker, MarkerArray

from control_center_navigation.srv import (
    DeleteWaypoint,
    ListWaypoints,
    NavigateToPoseRefined,
    NavigateToWaypoint,
    NavigateWaypointSequence,
    RenameWaypoint,
    SaveWaypoint,
    SwitchWaypointMap,
)

try:
    from control_center_chassis_guard import (
        ChassisControlLease,
        KeyEventDiagnostics,
        ManualKeyFilter,
        VelocityRamp,
        VelocityRampDiagnostics,
    )
except ImportError:  # deployment safety net; manual driving stays fail-closed
    ChassisControlLease = None

    class ManualKeyFilter:  # no-debounce fallback, matches pre-fix behaviour
        def __init__(self, release_delay_sec=0.0, clock=None):
            self._keys = set()

        def press(self, key, now=None):
            changed = key not in self._keys
            self._keys.add(key)
            return changed

        def release(self, key, now=None):
            changed = key in self._keys
            self._keys.discard(key)
            return changed

        def clear(self):
            changed = bool(self._keys)
            self._keys.clear()
            return changed

        def expire(self, now=None):
            return False

        def pressed_keys(self):
            return frozenset(self._keys)

        def next_deadline(self):
            return None

    class VelocityRamp:  # unsmoothed fallback, matches pre-fix behaviour
        def __init__(self, *args, **kwargs):
            self._values = [0.0, 0.0, 0.0]

        @property
        def target(self):
            return tuple(self._values)

        @property
        def current(self):
            return tuple(self._values)

        @property
        def idle(self):
            return not any(self._values)

        def set_target(self, linear_x, linear_y, angular_z):
            self._values = [float(linear_x), float(linear_y), float(angular_z)]
            return self.target

        def step(self, now=None):
            return self.current

        def reset(self):
            was_moving = not self.idle
            self._values = [0.0, 0.0, 0.0]
            return was_moving

    class KeyEventDiagnostics:  # inert fallback
        enabled = False
        path = None

        def log(self, kind, **fields):
            return None

        def close(self):
            return None

    class VelocityRampDiagnostics:  # inert fallback
        enabled = False
        path = None

        def record(self, timestamp, target, current):
            return None

        def close(self):
            return None


PACKAGE_TITLE = '全局重定位导航中心'
PACKAGE_NAME = (
    'control_center_navigation'
)


def _source_package_directory():
    share_path = FilePath(get_package_share_directory(PACKAGE_NAME)).resolve()
    install_prefix = share_path.parents[1]
    workspace_root = install_prefix.parent.parent
    for candidate in (
        workspace_root / 'src' / PACKAGE_NAME,
        workspace_root / 'src' / 'autolife_robot_control_center' / PACKAGE_NAME,
    ):
        if (candidate / 'package.xml').is_file():
            return candidate
    return share_path


def _legacy_default_map_yaml():
    # Editing source maps must not silently select a stale colcon install copy.
    source_map = (
        _source_package_directory().parent / 'multi_lidar_slam'
        / 'maps' / 'yundie_2f.yaml'
    )
    if source_map.is_file():
        return str(source_map.resolve())
    try:
        return str(
            FilePath(get_package_share_directory('multi_lidar_slam'))
            / 'maps'
            / 'yundie_2f.yaml'
        )
    except Exception:
        return ''


def _active_map_config_path():
    return _source_package_directory() / 'config' / 'active_map.yaml'


def _resolve_configured_path(value, source_package=None):
    path = FilePath(str(value or '')).expanduser()
    if not path.is_absolute():
        path = (source_package or _source_package_directory()) / path
    return path.resolve()


def _read_active_map_yaml(config_path=None):
    source_package = _source_package_directory()
    config = FilePath(config_path or _active_map_config_path()).resolve()
    try:
        payload = yaml.safe_load(config.read_text(encoding='utf-8')) or {}
        candidate = _resolve_configured_path(
            payload.get('active_map', ''), source_package
        )
        if candidate.is_file() and candidate.suffix.lower() == '.yaml':
            return str(candidate)
    except Exception:
        pass
    return _legacy_default_map_yaml()


def _waypoint_library_directory():
    directory = _source_package_directory() / 'maps' / 'waypoints'
    directory.mkdir(parents=True, exist_ok=True)
    return directory


def _waypoints_file_for_map(map_yaml, directory=None):
    map_path = FilePath(str(map_yaml)).expanduser().resolve()
    if map_path.suffix.lower() != '.yaml':
        raise ValueError('地图文件必须是 YAML')
    safe_stem = re.sub(r'[^\w\u4e00-\u9fff-]+', '_', map_path.stem).strip('_')
    if not safe_stem:
        raise ValueError('无法为该地图生成航点文件名')
    target_directory = FilePath(
        directory or _waypoint_library_directory()
    ).expanduser().resolve()
    map_directory = target_directory / safe_stem
    map_directory.mkdir(parents=True, exist_ok=True)
    return map_directory / f'{safe_stem}.yaml'


def _write_active_map_config(
    config_path, map_yaml, source_package=None, waypoint_directory=None
):
    source = FilePath(source_package or _source_package_directory()).resolve()
    config = FilePath(config_path).expanduser().resolve()
    map_path = FilePath(map_yaml).expanduser().resolve()
    if not map_path.is_file() or map_path.suffix.lower() != '.yaml':
        raise FileNotFoundError(f'默认地图文件无效：{map_path}')
    try:
        stored_map = str(map_path.relative_to(source))
    except ValueError:
        stored_map = str(map_path)
    waypoint_path = _waypoints_file_for_map(map_path, waypoint_directory)
    try:
        stored_waypoints = str(waypoint_path.relative_to(source))
    except ValueError:
        stored_waypoints = str(waypoint_path)
    payload = {
        'version': 1,
        'active_map': stored_map,
        'waypoints_file': stored_waypoints,
    }
    config.parent.mkdir(parents=True, exist_ok=True)
    temporary = config.with_name(f'.{config.name}.{time.time_ns()}.tmp')
    try:
        temporary.write_text(
            yaml.safe_dump(payload, allow_unicode=True, sort_keys=False),
            encoding='utf-8',
        )
        temporary.replace(config)
    finally:
        temporary.unlink(missing_ok=True)
    return config


def _default_initial_map_yaml():
    return _read_active_map_yaml()


def save_map_files(directory, name, grid, metadata, reserved_paths=()):
    """Save a new PGM/YAML map pair without ever overwriting a map."""
    map_name = str(name).strip()
    if (
        not map_name
        or len(map_name) > 64
        or FilePath(map_name).name != map_name
        or not re.fullmatch(r'[\w\u4e00-\u9fff-]+', map_name)
    ):
        return False, '地图名称只能包含中英文、数字、下划线和短横线', None

    output_directory = FilePath(directory).expanduser().resolve()
    output_directory.mkdir(parents=True, exist_ok=True)
    yaml_path = output_directory / f'{map_name}.yaml'
    pgm_path = output_directory / f'{map_name}.pgm'
    reserved = {
        FilePath(value).expanduser().resolve()
        for value in reserved_paths if str(value).strip()
    }
    reserved_names = {path.stem for path in reserved}
    if (
        yaml_path.exists()
        or pgm_path.exists()
        or yaml_path.resolve() in reserved
        or map_name in reserved_names
    ):
        return False, f'地图“{map_name}”已存在，禁止覆盖旧地图', None

    values = np.asarray(grid, dtype=np.int16)
    if values.ndim != 2 or values.size == 0:
        return False, '当前没有可保存的地图数据', None
    pixels = np.full(values.shape, 205, dtype=np.uint8)
    known = values >= 0
    clipped = np.clip(values[known], 0, 100)
    pixels[known] = np.rint(
        (100.0 - clipped.astype(np.float64)) * 254.0 / 100.0
    ).astype(np.uint8)
    pixels = np.ascontiguousarray(np.flipud(pixels))

    pgm_created = False
    yaml_created = False
    try:
        with pgm_path.open('xb') as stream:
            pgm_created = True
            stream.write(
                f'P5\n# edited by {PACKAGE_NAME}\n'
                f'{values.shape[1]} {values.shape[0]}\n255\n'.encode('ascii')
            )
            stream.write(pixels.tobytes())
        map_yaml = {
            'image': pgm_path.name,
            'mode': 'trinary',
            'resolution': float(metadata['resolution']),
            'origin': [
                float(metadata['origin_x']),
                float(metadata['origin_y']),
                float(metadata.get('origin_yaw', 0.0)),
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            # Gray 205 is UNKNOWN (1 - 205/255 = 0.196078), not free.
            'free_thresh': 0.196,
        }
        with yaml_path.open('x', encoding='utf-8') as stream:
            yaml_created = True
            yaml.safe_dump(
                map_yaml,
                stream,
                allow_unicode=True,
                sort_keys=False,
            )
    except FileExistsError:
        if pgm_created and pgm_path.exists():
            pgm_path.unlink()
        return False, f'地图“{map_name}”已存在，禁止覆盖旧地图', None
    except Exception as error:
        if yaml_created and yaml_path.exists():
            yaml_path.unlink()
        if pgm_created and pgm_path.exists():
            pgm_path.unlink()
        return False, f'保存地图失败：{error}', None
    return True, f'新地图已保存：{yaml_path}', str(yaml_path)


def _map_file_pair(path, catalog):
    path = FilePath(path).expanduser()
    if path.is_symlink():
        raise ValueError('不允许修改符号链接地图')
    path = path.resolve()
    if path not in {FilePath(p).resolve() for p in catalog} or not path.is_file():
        raise ValueError('地图不在当前地图库中，请刷新后重试')
    data = yaml.safe_load(path.read_text(encoding='utf-8'))
    image = FilePath(data['image']).expanduser()
    image = image if image.is_absolute() else path.parent / image
    if image.is_symlink() or image.resolve().parent != path.parent:
        raise ValueError('地图图像不在同一目录，不能覆盖或删除，请先另存地图')
    image = image.resolve()
    if not image.is_file():
        raise ValueError('地图图像不存在')
    return path, image, data

def _image_shared(image, path, catalog):
    for other in catalog:
        other = FilePath(other).resolve()
        if other == path:
            continue
        try:
            data = yaml.safe_load(other.read_text(encoding='utf-8'))
            target = FilePath(data['image'])
            if not target.is_absolute():
                target = other.parent / target
            if target.resolve() == image:
                return True
        except Exception:
            # Cannot establish ownership; keep the image rather than risk loss.
            return True
    return False

def overwrite_map_files(path, grid, metadata, catalog):
    """Keep the original YAML/image paths; back up both before replacing."""
    path, image, data = _map_file_pair(path, catalog)
    if _image_shared(image, path, catalog):
        raise ValueError('图像被其他地图共用，不能覆盖，请另存新地图')
    if image.suffix.lower() not in ('.png', '.pgm'):
        raise ValueError('当前图像格式不支持覆盖，请另存为新地图')
    old_image = QtGui.QImage(str(image))
    values = np.asarray(grid)
    expected = [float(metadata[k]) for k in ('origin_x', 'origin_y', 'origin_yaw')]
    if (old_image.isNull() or values.shape != (old_image.height(), old_image.width())
            or not np.allclose(data['origin'], expected, atol=1e-7, rtol=0)
            or abs(float(data['resolution']) - float(metadata['resolution'])) > 1e-9):
        raise ValueError('当前画面与原地图的尺寸/坐标不一致，请重新加载地图或另存')
    if not np.isfinite(values).all() or np.any(values < -1) or np.any(values > 100):
        raise ValueError('地图像素数据无效')
    pixels = np.full(values.shape, 205, dtype=np.uint8)
    known = values >= 0
    pixels[known] = np.rint((100. - values[known]) * 254. / 100.).astype(np.uint8)
    pixels = np.ascontiguousarray(np.flipud(pixels))
    picture = QtGui.QImage(pixels.data, pixels.shape[1], pixels.shape[0], pixels.shape[1],
                          QtGui.QImage.Format_Grayscale8).copy()
    stamp = str(time.time_ns())
    backup = path.parent / '.map_backups' / (path.stem + '_' + stamp)
    backup.mkdir(parents=True)
    shutil.copy2(path, backup / path.name)
    shutil.copy2(image, backup / image.name)
    temp_image = image.with_name('.' + image.name + '.' + stamp + '.tmp')
    temp_yaml = path.with_name('.' + path.name + '.' + stamp + '.tmp')
    changed = False
    try:
        if not picture.save(str(temp_image), image.suffix[1:].upper()):
            raise OSError('图像写入失败')
        data.update(mode='trinary', negate=0, occupied_thresh=.65, free_thresh=.196)
        temp_yaml.write_text(yaml.safe_dump(data, allow_unicode=True, sort_keys=False), encoding='utf-8')
        temp_image.replace(image)
        changed = True
        temp_yaml.replace(path)
    except Exception:
        if changed:
            shutil.copy2(backup / image.name, image)
            shutil.copy2(backup / path.name, path)
        raise
    finally:
        temp_image.unlink(missing_ok=True)
        temp_yaml.unlink(missing_ok=True)
    return backup

def archive_map_files(path, catalog, protected_paths=()):
    """Remove from catalog by moving exact files to a recoverable local trash."""
    path, image, _ = _map_file_pair(path, catalog)
    if path in {FilePath(p).resolve() for p in protected_paths if p}:
        raise ValueError('不能删除当前使用/默认地图，请先加载另一张地图')
    files = [path]
    if not _image_shared(image, path, catalog):
        files.append(image)
    state = path.with_suffix('.pbstream')
    if state.is_file() and not state.is_symlink():
        files.append(state)
    trash = path.parent / '.map_trash' / (path.stem + '_' + str(time.time_ns()))
    trash.mkdir(parents=True)
    moved = []
    try:
        for item in files:
            item.rename(trash / item.name)
            moved.append(item)
    except Exception:
        for item in reversed(moved):
            (trash / item.name).rename(item)
        raise
    return trash


def yaw_from_quaternion(quaternion):
    return math.atan2(
        2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y),
        1.0 - 2.0 * (
            quaternion.y * quaternion.y + quaternion.z * quaternion.z
        ),
    )


def quaternion_from_yaw(yaw):
    half = float(yaw) * 0.5
    return 0.0, 0.0, math.sin(half), math.cos(half)


class RosSignals(QtCore.QObject):
    map_received = QtCore.pyqtSignal(object)
    pose_received = QtCore.pyqtSignal(object)
    global_path_received = QtCore.pyqtSignal(object)
    local_path_received = QtCore.pyqtSignal(object)
    laser_received = QtCore.pyqtSignal(object)
    waypoint_markers_received = QtCore.pyqtSignal(object)
    navigation_status = QtCore.pyqtSignal(object)
    relocalization_status = QtCore.pyqtSignal(object)
    service_result = QtCore.pyqtSignal(str, bool, str, object)
    direct_navigation_status = QtCore.pyqtSignal(object)
    chassis_lease_error = QtCore.pyqtSignal(str)


class _UnavailableChassisLease:
    """Fail-closed stand-in when control_center_chassis_guard is missing."""

    held = False
    last_error = 'control_center_chassis_guard 未安装，手动底盘控制已禁用。'

    def acquire(self, timeout_sec=None):
        return False

    def release(self):
        return True

    def shutdown(self):
        return None


class NavigationRosNode(Node):
    def __init__(self, manual_cmd_vel_topic=None):
        super().__init__('navigation_native_ui')
        self.signals = RosSignals()
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('global_path_topic', '/plan')
        self.declare_parameter('local_path_topic', '/local_plan')
        self.declare_parameter(
            'front_scan_topic', '/topic_gv_front_lidar_0_300'
        )
        self.declare_parameter(
            'rear_scan_topic', '/topic_gv_rear_lidar_0_300'
        )
        # Show exactly the per-sensor observations used for costmap marking.
        # Keep raw topic parameters above for robot identity and diagnostics.
        self.declare_parameter('front_display_scan_topic', '/navigation/front_scan')
        self.declare_parameter('rear_display_scan_topic', '/navigation/rear_scan')
        self.declare_parameter(
            'manual_cmd_vel_topic',
            str(manual_cmd_vel_topic or '/manual_cmd_vel'),
        )
        self.declare_parameter('manual_cmd_vel_rate_hz', 50.0)
        self.declare_parameter('display_map_frame', 'map')
        self.declare_parameter('display_robot_frame', 'base_link')
        self.declare_parameter('display_pose_rate_hz', 30.0)
        self.declare_parameter(
            'edited_maps_directory',
            str(_source_package_directory() / 'maps'),
        )
        self.declare_parameter('initial_map_yaml', _default_initial_map_yaml())
        self.declare_parameter(
            'active_map_config', str(_active_map_config_path())
        )
        self.declare_parameter(
            'waypoint_library_directory',
            str(_waypoint_library_directory()),
        )
        self._edited_maps_directory = FilePath(
            self._param('edited_maps_directory')
        ).expanduser().resolve()
        self._initial_map_yaml = self._param('initial_map_yaml').strip()
        self._loaded_map_path = self._initial_map_yaml
        self._active_map_config = FilePath(
            self._param('active_map_config')
        ).expanduser().resolve()
        self._waypoint_library_directory = FilePath(
            self._param('waypoint_library_directory')
        ).expanduser().resolve()
        self._map_load_in_progress = False
        self._last_pose_time = 0.0
        self._last_map_time = 0.0
        self._last_tf_pose_time = 0.0
        self._manual_localization = None
        self._manual_amcl_stamp = 0.0
        self._last_cov_x = 0.0
        self._last_cov_y = 0.0
        self._accept_map_updates = True

        latched = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        status_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        laser_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.create_subscription(
            OccupancyGrid,
            self._param('map_topic'),
            self._on_map,
            latched,
        )
        self.create_subscription(
            PoseWithCovarianceStamped,
            self._param('amcl_pose_topic'),
            self._on_pose,
            20,
        )
        self.create_subscription(
            Path,
            self._param('global_path_topic'),
            lambda msg: self.signals.global_path_received.emit(msg),
            10,
        )
        self.create_subscription(
            Path,
            self._param('local_path_topic'),
            lambda msg: self.signals.local_path_received.emit(msg),
            10,
        )
        self.create_subscription(
            MarkerArray,
            '/waypoints/markers',
            self._on_waypoint_markers,
            latched,
        )
        self.create_subscription(
            String,
            '/waypoints/navigation_status',
            self._on_navigation_status,
            status_qos,
        )
        self.create_subscription(
            String,
            '/global_relocalization/status',
            self._on_relocalization_status,
            status_qos,
        )

        self.continuity_status = {}
        self.create_subscription(String, '/localization/continuity_status',
                                 self._on_continuity_status, status_qos)
        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False
        )
        self._display_map_frame = self._param('display_map_frame')
        self._display_robot_frame = self._param('display_robot_frame')
        display_pose_rate = max(
            5.0,
            min(
                60.0,
                float(self.get_parameter('display_pose_rate_hz').value),
            ),
        )
        self._display_pose_timer = self.create_timer(
            1.0 / display_pose_rate,
            self._publish_display_pose_from_tf,
        )
        self._laser_by_topic = {}
        self._pending_laser = {}
        self._laser_display_timer = self.create_timer(0.05, self._flush_laser_display)
        for topic in (
            self._param('front_display_scan_topic'),
            self._param('rear_display_scan_topic'),
        ):
            self.create_subscription(
                LaserScan,
                topic,
                partial(self._on_laser, topic),
                laser_qos,
            )

        self.velocity_publisher = self.create_publisher(
            Twist, self._param('manual_cmd_vel_topic'), 10
        )
        self.initial_pose_publisher = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', latched
        )
        self._velocity_lock = threading.Lock()
        self._velocity_shutdown = threading.Event()
        requested_rate = float(
            self.get_parameter('manual_cmd_vel_rate_hz').value
        )
        self._velocity_period = 1.0 / max(
            10.0, min(100.0, requested_rate)
        )
        # Keyboard input only sets the ramp target; this node's heartbeat thread
        # advances the commanded velocity toward it.  Keeping the ramp on that
        # thread means a stalled Qt event loop still decelerates to zero.
        self._velocity_diagnostics = VelocityRampDiagnostics(tag='nav')
        self._velocity_ramp = VelocityRamp(
            update_hz=1.0 / self._velocity_period,
            diagnostics=self._velocity_diagnostics,
        )
        self._velocity_thread = threading.Thread(
            target=self._velocity_heartbeat_loop,
            name='manual-cmd-vel-heartbeat',
            daemon=True,
        )
        self._velocity_thread.start()
        self._chassis_lease = (
            ChassisControlLease(
                topic_hint=self._param('front_scan_topic'),
                logger=self.get_logger(),
                on_lease_lost=self._handle_chassis_lease_lost,
            )
            if ChassisControlLease is not None
            else _UnavailableChassisLease()
        )

        self._service_clients = {
            'list': self.create_client(ListWaypoints, 'list_waypoints'),
            'save': self.create_client(SaveWaypoint, 'save_waypoint'),
            'delete': self.create_client(DeleteWaypoint, 'delete_waypoint'),
            'rename': self.create_client(RenameWaypoint, 'rename_waypoint'),
            'navigate': self.create_client(
                NavigateToWaypoint, 'navigate_to_waypoint'
            ),
            'navigate_pose_refined': self.create_client(
                NavigateToPoseRefined, 'navigate_to_pose_refined'
            ),
            'sequence': self.create_client(
                NavigateWaypointSequence, 'navigate_waypoint_sequence'
            ),
            'pause': self.create_client(Trigger, 'pause_navigation'),
            'resume': self.create_client(Trigger, 'resume_navigation'),
            'cancel': self.create_client(Trigger, 'cancel_navigation'),
            'relocalize': self.create_client(
                Trigger, '/cartographer_global_relocalization/relocalize'
            ),
            'cancel_localization': self.create_client(
                Trigger, '/cartographer_global_relocalization/cancel'
            ),
            'nomotion': self.create_client(Empty, '/request_nomotion_update'),
            'load_map': self.create_client(
                LoadMap, '/map_server/load_map'
            ),
            'switch_waypoint_map': self.create_client(
                SwitchWaypointMap, 'switch_waypoint_map'
            ),
        }

    def _param(self, name):
        return self.get_parameter(name).get_parameter_value().string_value

    def _on_map(self, message):
        if self._accept_map_updates:
            self.signals.map_received.emit(message)

    def set_map_updates_enabled(self, enabled):
        self._accept_map_updates = bool(enabled)
        if not self._accept_map_updates:
            self._last_map_time = 0.0

    @property
    def map_age(self):
        return time.monotonic() - self._last_map_time if self._last_map_time else 1e9

    @property
    def pose_age(self):
        return time.monotonic() - self._last_pose_time if self._last_pose_time else 1e9

    def _on_pose(self, message):
        pose = message.pose.pose
        covariance = message.pose.covariance
        self._last_cov_x = max(0.0, float(covariance[0]))
        self._last_cov_y = max(0.0, float(covariance[7]))
        now = time.monotonic()
        if self._manual_localization is not None:
            stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
            request = self._manual_localization
            if stamp < request.get('stamp', float('inf')):
                return
            self._manual_amcl_stamp = stamp
            self._manual_amcl_pose = (pose.position.x, pose.position.y,
                                      yaw_from_quaternion(pose.orientation))
            return  # TF and AMCL must agree before replacing the requested marker.
        if now - self._last_tf_pose_time <= 0.25:
            return
        self._last_pose_time = now
        self.signals.pose_received.emit({
            'x': float(pose.position.x),
            'y': float(pose.position.y),
            'yaw': yaw_from_quaternion(pose.orientation),
            'cov_x': self._last_cov_x,
            'cov_y': self._last_cov_y,
        })

    def _publish_display_pose_from_tf(self):
        request = self._manual_localization
        if request is not None:
            if time.monotonic() - request['started'] > 12:
                if not request.get('warned'):
                    request['warned'] = True
                    self.signals.service_result.emit('initial', False,
                        '尚未确认定位，请核对地图及激光；橙色为指定位置，不是已确认定位', None)
            if self._manual_amcl_stamp < request.get('stamp', float('inf')):
                return
        try:
            transform = self.tf_buffer.lookup_transform(
                self._display_map_frame,
                self._display_robot_frame,
                Time(),
            )
        except Exception:
            return
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        stamp = transform.header.stamp.sec + transform.header.stamp.nanosec * 1e-9
        age = self.get_clock().now().nanoseconds * 1e-9 - stamp
        if not -2.0 <= age <= 1.0:
            return
        if request is not None:
            p = self._manual_amcl_pose
            yaw = yaw_from_quaternion(rotation)
            if stamp < request['stamp'] or math.hypot(translation.x-p[0], translation.y-p[1]) > .15:
                return
            if abs(math.atan2(math.sin(yaw-p[2]), math.cos(yaw-p[2]))) > .15:
                return
            self._manual_localization = None
            self.signals.service_result.emit('initial', True, '手动定位已由AMCL与实时坐标确认', None)
        now = time.monotonic()
        self._last_tf_pose_time = now
        self._last_pose_time = now
        self.signals.pose_received.emit({
            'x': float(translation.x),
            'y': float(translation.y),
            'yaw': yaw_from_quaternion(rotation),
            'cov_x': self._last_cov_x,
            'cov_y': self._last_cov_y,
        })

    def _on_navigation_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = {'state': 'unknown', 'message': message.data}
        self.signals.navigation_status.emit(payload)

    def _on_continuity_status(self, message):
        try:
            self.continuity_status = json.loads(message.data)
        except (ValueError, TypeError):
            pass

    def _on_relocalization_status(self, message):
        try:
            payload = json.loads(message.data)
        except (TypeError, ValueError):
            payload = {'state': 'unknown', 'detail': message.data}
        self.signals.relocalization_status.emit(payload)

    def _on_waypoint_markers(self, message):
        waypoints = {}
        for marker in message.markers:
            if marker.action == Marker.DELETEALL:
                continue
            name = str(marker.text).strip()
            if not name:
                continue
            entry = waypoints.setdefault(name, {
                'name': name,
                'x': float(marker.pose.position.x),
                'y': float(marker.pose.position.y),
                'yaw': 0.0,
                'color': (
                    float(marker.color.r),
                    float(marker.color.g),
                    float(marker.color.b),
                ),
            })
            if marker.ns == 'waypoint_headings':
                entry['yaw'] = yaw_from_quaternion(marker.pose.orientation)
            if marker.ns == 'waypoint_points':
                entry['x'] = float(marker.pose.position.x)
                entry['y'] = float(marker.pose.position.y)
                entry['color'] = (
                    float(marker.color.r),
                    float(marker.color.g),
                    float(marker.color.b),
                )
        self.signals.waypoint_markers_received.emit(list(waypoints.values()))

    def _on_laser(self, topic, message):
        # Capacity one per scanner; retry stamped TF briefly instead of drawing
        # old rays at the robot's latest pose or accumulating a message queue.
        self._pending_laser[topic] = (message, time.monotonic())

    def _flush_laser_display(self):
        now = time.monotonic()
        changed = False
        for topic, (message, received) in list(self._pending_laser.items()):
            stamp = Time.from_msg(message.header.stamp)
            age = (self.get_clock().now().nanoseconds - stamp.nanoseconds) / 1e9
            if stamp.nanoseconds == 0 or age > 0.5 or age < -0.1 or now - received > 0.3:
                self._pending_laser.pop(topic, None)
                changed = self._laser_by_topic.pop(topic, None) is not None or changed
                continue
            points = self._laser_points_at_stamp(message, stamp)
            if points is None:
                continue
            self._laser_by_topic[topic] = (points, received)
            self._pending_laser.pop(topic, None)
            changed = True
        for topic, (_, received) in list(self._laser_by_topic.items()):
            if now - received > 0.5:
                self._laser_by_topic.pop(topic, None)
                changed = True
        if changed:
            self.signals.laser_received.emit([
                point for points, _ in self._laser_by_topic.values() for point in points
            ])

    def _laser_points_at_stamp(self, message, stamp):
        try:
            transform = self.tf_buffer.lookup_transform(
                self._display_map_frame, message.header.frame_id, stamp
            )
        except Exception:
            return
        rotation = transform.transform.rotation
        translation = transform.transform.translation
        qx, qy, qz, qw = (
            rotation.x, rotation.y, rotation.z, rotation.w
        )
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        step = max(1, len(message.ranges) // 420)
        points = []
        for index in range(0, len(message.ranges), step):
            distance = float(message.ranges[index])
            if not math.isfinite(distance):
                continue
            if distance < message.range_min or distance > message.range_max:
                continue
            angle = message.angle_min + index * message.angle_increment
            local_x = distance * math.cos(angle)
            local_y = distance * math.sin(angle)
            points.append((
                translation.x + r00 * local_x + r01 * local_y,
                translation.y + r10 * local_x + r11 * local_y,
            ))
        return points

    def _publish_velocity_values(self, values):
        linear_x, linear_y, angular_z = values
        message = Twist()
        message.linear.x = linear_x
        message.linear.y = linear_y
        message.angular.z = angular_z
        self.velocity_publisher.publish(message)

    @property
    def _velocity(self):
        """Commanded velocity, i.e. the value the ramp last published."""
        return self._velocity_ramp.current

    @property
    def _velocity_active(self):
        return not self._velocity_ramp.idle

    def _publish_ramp_step_locked(self):
        """Advance the slew-rate limiter one tick and publish its output."""
        self._publish_velocity_values(self._velocity_ramp.step())

    def publish_velocity(self):
        with self._velocity_lock:
            self._publish_velocity_values(self._velocity)

    def set_velocity(self, linear_x, linear_y, angular_z):
        """Request a velocity; the heartbeat ramps the command toward it.

        The target applies on this call, so a key press is never delayed, but
        the chassis sees the first limited increment instead of a step from zero.
        """
        values = (
            float(linear_x), float(linear_y), float(angular_z)
        )
        nonzero = any(abs(value) > 1e-6 for value in values)
        release = False
        with self._velocity_lock:
            if nonzero and not self._chassis_lease.acquire():
                # Fail closed: publish zeros and keep the state inert.
                self._velocity_ramp.reset()
                self._publish_velocity_values((0.0, 0.0, 0.0))
                return
            self._velocity_ramp.set_target(*values)
            self._publish_ramp_step_locked()
            release = self._velocity_ramp.idle
        if release:
            # A zero request with an already-stopped ramp: hand the chassis
            # back.  A ramped stop keeps the lease until the command reaches
            # zero, which the heartbeat does.
            self._chassis_lease.release()

    def stop(self):
        """Request a smooth stop (normal key release)."""
        self.set_velocity(0.0, 0.0, 0.0)

    def stop_immediate(self):
        """Zero the chassis command on this call, bypassing the ramp.

        Reserved for the safety paths -- X key, focus loss, page switch, window
        close and shutdown -- which must keep their original instant-stop
        meaning and must never leave a ramped command outstanding.
        """
        with self._velocity_lock:
            self._velocity_ramp.reset()
            try:
                self._publish_velocity_values((0.0, 0.0, 0.0))
            except Exception:  # pragma: no cover - shutdown race guard
                pass
        self._chassis_lease.release()

    def _handle_chassis_lease_lost(self, reason):
        """Vendor takeover or Redis failure mid-drive: stop immediately."""
        with self._velocity_lock:
            self._velocity_ramp.reset()
            try:
                self._publish_velocity_values((0.0, 0.0, 0.0))
            except Exception:  # pragma: no cover - shutdown race guard
                pass
        message = self._chassis_lease.last_error or str(reason)
        self.signals.chassis_lease_error.emit(str(message))

    def tick_velocity(self):
        """Advance the ramp one control period and publish the commanded value.

        This is the heartbeat's body; it stops publishing once the command and
        the request are both zero and hands the chassis back at that point.
        """
        release = False
        with self._velocity_lock:
            if self._velocity_ramp.idle:
                # Nothing to ramp: stay quiet instead of re-publishing zero.
                return
            self._publish_ramp_step_locked()
            release = self._velocity_ramp.idle
        if release:
            # The ramp has just reached zero, so the stop is complete.
            self._chassis_lease.release()

    def _velocity_heartbeat_loop(self):
        while not self._velocity_shutdown.wait(self._velocity_period):
            self.tick_velocity()

    def publish_initial_pose(self, x, y, yaw):
        if self._map_load_in_progress or not all(math.isfinite(v) for v in (x, y, yaw)):
            self.signals.service_result.emit('initial', False, '地图切换中或坐标无效，未设置定位', None)
            return
        request = dict(x=x, y=y, yaw=yaw, started=time.monotonic(), map=self._loaded_map_path)
        self._manual_localization = request
        self._manual_amcl_stamp = 0.0
        self.signals.pose_received.emit(dict(x=x, y=y, yaw=yaw, pending=True))
        client = self._service_clients['cancel_localization']
        if not client.service_is_ready():
            self._manual_localization = None
            self.signals.service_result.emit('initial', False, '定位服务未就绪，未发布初始位置', None)
            return
        future = client.call_async(Trigger.Request())
        def canceled(future):
            if self._manual_localization is not request:
                return  # An older click must never overwrite the newest request.
            try:
                response = future.result()
                if not response.success:
                    raise ValueError(response.message)
                if self._map_load_in_progress or request['map'] != self._loaded_map_path:
                    raise ValueError('地图已改变，请重新设置位置')
                if time.monotonic()-request['started'] > 5:
                    raise ValueError('定位停止确认超时，请重试')
                self._publish_manual_pose(request)
            except Exception as error:
                self._manual_localization = None
                self.signals.service_result.emit('initial', False, str(error), None)
        future.add_done_callback(canceled)

    def _publish_manual_pose(self, request):
        x, y, yaw = request['x'], request['y'], request['yaw']
        message = PoseWithCovarianceStamped()
        message.header.frame_id = 'map'
        message.header.stamp = self.get_clock().now().to_msg()
        message.pose.pose.position.x = float(x)
        message.pose.pose.position.y = float(y)
        qx, qy, qz, qw = quaternion_from_yaw(yaw)
        message.pose.pose.orientation.x = qx
        message.pose.pose.orientation.y = qy
        message.pose.pose.orientation.z = qz
        message.pose.pose.orientation.w = qw
        # A deliberate map click is a local correction, not a broad global search.
        message.pose.covariance[0] = 0.01
        message.pose.covariance[7] = 0.01
        message.pose.covariance[35] = math.radians(10) ** 2
        request['stamp'] = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        self.initial_pose_publisher.publish(message)
        client = self._service_clients['nomotion']
        if client.service_is_ready():
            client.call_async(Empty.Request())
        self.signals.service_result.emit('initial', True, '已指定橙色位置，等待实时定位确认', None)

    def call_service(self, key, request, payload=None):
        client = self._service_clients[key]
        if key == 'navigate':
            self.get_logger().info(f'用户请求导航：{payload}')
        if not client.service_is_ready():
            self.signals.service_result.emit(
                key, False, 'ROS 服务尚未就绪', payload
            )
            return
        future = client.call_async(request)
        future.add_done_callback(
            partial(self._service_done, key, payload)
        )

    def _service_done(self, key, payload, future):
        try:
            response = future.result()
            success = bool(getattr(response, 'success', True))
            message = str(getattr(response, 'message', '操作完成'))
            data = getattr(response, 'names', payload)
        except Exception as error:
            success, message, data = False, str(error), payload
        if key == 'navigate':
            self.get_logger().info(f'导航请求结果：{payload} success={success} {message}')
        self.signals.service_result.emit(key, success, message, data)

    def refresh_waypoints(self):
        self.call_service('list', ListWaypoints.Request())

    def map_catalog(self):
        entries = []
        seen = set()
        active_path = (
            FilePath(self._loaded_map_path).expanduser().resolve()
            if self._loaded_map_path else None
        )
        legacy_path = _legacy_default_map_yaml()
        directories = []
        if legacy_path:
            directories.append(
                ('原地图', FilePath(legacy_path).expanduser().resolve().parent)
            )
        directories.append(('地图库', self._edited_maps_directory))
        if active_path:
            directories.append(('当前地图目录', active_path.parent))
        for category, directory in directories:
            if not directory.is_dir():
                continue
            for path in sorted(directory.glob('*.yaml')):
                resolved = path.resolve()
                if resolved in seen:
                    continue
                label = (
                    f'当前默认地图 · {path.stem}'
                    if resolved == active_path
                    else f'{category} · {path.stem}'
                )
                entries.append({'label': label, 'path': str(resolved)})
                seen.add(resolved)
        return entries

    def save_map_as(self, name, grid, metadata):
        reserved_paths = [
            entry['path'] for entry in self.map_catalog()
        ]
        return save_map_files(
            self._edited_maps_directory,
            name,
            grid,
            metadata,
            reserved_paths=reserved_paths,
        )

    def save_current_map(self, grid, metadata):
        if self._map_load_in_progress:
            raise ValueError('地图正在切换，请稍候')
        return overwrite_map_files(self._loaded_map_path, grid, metadata,
                                   [entry['path'] for entry in self.map_catalog()])

    def delete_map(self, path):
        if self._map_load_in_progress:
            raise ValueError('地图正在切换，请稍候')
        return archive_map_files(path, [entry['path'] for entry in self.map_catalog()],
                                 [self._loaded_map_path, _read_active_map_yaml(self._active_map_config)])

    def load_map(self, yaml_path):
        if self._map_load_in_progress:
            self.signals.service_result.emit(
                'load_map', False, '另一张地图正在切换，请稍候', str(yaml_path)
            )
            return
        path = FilePath(str(yaml_path)).expanduser().resolve()
        catalog_paths = {
            FilePath(entry['path']).resolve() for entry in self.map_catalog()
        }
        if path not in catalog_paths or not path.is_file():
            self.signals.service_result.emit(
                'load_map', False, '所选地图文件不存在或不在地图库中',
                str(path),
            )
            return
        client = self._service_clients['load_map']
        waypoint_client = self._service_clients['switch_waypoint_map']
        if not client.service_is_ready():
            self.signals.service_result.emit(
                'load_map', False, 'map_server/load_map 服务尚未就绪',
                str(path),
            )
            return
        if not waypoint_client.service_is_ready():
            self.signals.service_result.emit(
                'load_map', False, '航点库切换服务尚未就绪', str(path)
            )
            return
        try:
            _waypoints_file_for_map(path, self._waypoint_library_directory)
        except Exception as error:
            self.signals.service_result.emit(
                'load_map', False, f'航点文件准备失败：{error}', str(path)
            )
            return
        self._map_load_in_progress = True
        request = LoadMap.Request()
        request.map_url = str(path)
        future = client.call_async(request)
        future.add_done_callback(
            partial(self._load_map_done, str(path))
        )

    def _load_map_done(self, yaml_path, future):
        try:
            response = future.result()
            result_code = int(response.result)
            success = result_code == 0
            result_messages = {
                0: '地图加载成功',
                1: '地图文件不存在',
                2: '地图文件数据无效',
                3: '地图元数据无效',
                255: '地图加载发生未定义错误',
            }
            message = result_messages.get(
                result_code, f'地图加载失败，错误码 {result_code}'
            )
        except Exception as error:
            success, message = False, f'地图加载失败：{error}'
        if not success:
            self._map_load_in_progress = False
            self.signals.service_result.emit(
                'load_map', False, message, yaml_path
            )
            return
        request = SwitchWaypointMap.Request()
        request.map_yaml = str(yaml_path)
        future = self._service_clients['switch_waypoint_map'].call_async(request)
        future.add_done_callback(
            partial(self._switch_waypoint_map_done, str(yaml_path))
        )

    def _switch_waypoint_map_done(self, yaml_path, future):
        self._map_load_in_progress = False
        try:
            response = future.result()
            if not response.success:
                raise RuntimeError(response.message)
            _write_active_map_config(
                self._active_map_config,
                yaml_path,
                _source_package_directory(),
                self._waypoint_library_directory,
            )
            self._loaded_map_path = str(yaml_path)
            self._initial_map_yaml = str(yaml_path)
            payload = {
                'path': str(yaml_path),
                'waypoints_file': str(response.waypoints_file),
                'names': list(response.names),
            }
            message = (
                '地图已加载并设为默认；航点库已切换为 '
                f'{response.waypoints_file}'
            )
            self.signals.service_result.emit(
                'load_map', True, message, payload
            )
        except Exception as error:
            self.signals.service_result.emit(
                'load_map', False,
                f'地图已加载，但航点库或默认地图切换失败：{error}',
                str(yaml_path),
            )

    def navigate_to_pose(self, x, y, yaw):
        client = self._service_clients['navigate_pose_refined']
        if not client.service_is_ready():
            self.signals.direct_navigation_status.emit({
                'state': 'failed',
                'message': '精确导航服务尚未就绪',
            })
            return
        request = NavigateToPoseRefined.Request()
        request.x = float(x)
        request.y = float(y)
        request.yaw = float(yaw)
        request.frame_id = 'map'
        self.call_service('navigate_pose_refined', request)
        self.signals.direct_navigation_status.emit({
            'state': 'sending',
            'message': f'正在发送精确地图目标 ({x:.2f}, {y:.2f})',
        })
        return

        if not self.navigate_action.server_is_ready():
            self.signals.direct_navigation_status.emit({
                'state': 'failed',
                'message': 'Nav2 navigate_to_pose Action 尚未就绪',
            })
            return
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        _, _, qz, qw = quaternion_from_yaw(yaw)
        goal.pose.pose.orientation.z = qz
        goal.pose.pose.orientation.w = qw
        future = self.navigate_action.send_goal_async(
            goal, feedback_callback=self._direct_feedback
        )
        future.add_done_callback(self._direct_goal_response)
        self.signals.direct_navigation_status.emit({
            'state': 'sending',
            'message': f'正在发送地图目标 ({x:.2f}, {y:.2f})',
        })

    def _direct_goal_response(self, future):
        try:
            goal_handle = future.result()
        except Exception as error:
            self.signals.direct_navigation_status.emit({
                'state': 'failed', 'message': str(error)
            })
            return
        if not goal_handle.accepted:
            self.signals.direct_navigation_status.emit({
                'state': 'rejected', 'message': 'Nav2 拒绝了地图目标'
            })
            return
        self.direct_goal_handle = goal_handle
        result = goal_handle.get_result_async()
        result.add_done_callback(self._direct_result)

    def _direct_feedback(self, feedback_message):
        feedback = feedback_message.feedback
        self.signals.direct_navigation_status.emit({
            'state': 'running',
            'message': (
                f'地图导航中，剩余 '
                f'{float(feedback.distance_remaining):.2f} m'
            ),
            'distance_remaining': float(feedback.distance_remaining),
        })

    def _direct_result(self, future):
        try:
            status = int(future.result().status)
        except Exception as error:
            status = -1
            message = str(error)
        else:
            message = '地图目标已到达' if status == 4 else (
                f'地图导航结束，状态码 {status}'
            )
        self.direct_goal_handle = None
        self.signals.direct_navigation_status.emit({
            'state': 'succeeded' if status == 4 else 'failed',
            'message': message,
        })

    def cancel_direct_goal(self):
        self.call_service('cancel', Trigger.Request())
        self.stop()
        self.signals.direct_navigation_status.emit({
            'state': 'canceling', 'message': '正在取消地图导航'
        })
        return

        if self.direct_goal_handle is None:
            self.signals.direct_navigation_status.emit({
                'state': 'idle', 'message': '当前没有界面地图目标'
            })
            return
        self.direct_goal_handle.cancel_goal_async()
        self.stop()
        self.signals.direct_navigation_status.emit({
            'state': 'canceling', 'message': '正在取消地图导航'
        })

    def destroy_node(self):
        # Shutdown is a safety path: a ramped stop here would stop the heartbeat
        # before the command reached zero and leave the chassis driving.
        self.stop_immediate()
        self._velocity_shutdown.set()
        self._velocity_thread.join(timeout=1.0)
        self._velocity_diagnostics.close()
        if self._chassis_lease is not None:
            self._chassis_lease.shutdown()
        super().destroy_node()


class MapCanvas(QtWidgets.QWidget):
    goal_requested = QtCore.pyqtSignal(float, float, float)
    initial_pose_requested = QtCore.pyqtSignal(float, float, float)
    coordinate_changed = QtCore.pyqtSignal(str)
    edit_state_changed = QtCore.pyqtSignal(bool, bool, bool)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setObjectName('mapCanvas')
        self.setMinimumSize(620, 520)
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.image = None
        self.grid = None
        self.loaded_grid = None
        self.map_meta = None
        self.robot = None
        self.requested_pose = None
        self.global_path = []
        self.local_path = []
        self.laser_points = []
        self.waypoints = []
        self.goal = None
        self.mode = 'browse'
        self.zoom = 1.0
        self.pan = QtCore.QPointF(0.0, 0.0)
        self.drag_start = None
        self.drag_last = None
        self.drag_world = None
        self.cursor_point = None
        self.brush_size_m = 0.20
        self._stroke_before = None
        self._stroke_last_cell = None
        self._stroke_changed = False
        self._undo_stack = []
        self._redo_stack = []
        self.show_laser = True

    def set_map(self, message):
        width = int(message.info.width)
        height = int(message.info.height)
        if width <= 0 or height <= 0:
            return
        self.grid = np.asarray(
            message.data, dtype=np.int16
        ).reshape(height, width).copy()
        self.loaded_grid = self.grid.copy()
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._stroke_before = None
        self._stroke_last_cell = None
        self._stroke_changed = False
        origin = message.info.origin
        self.map_meta = {
            'width': width,
            'height': height,
            'resolution': float(message.info.resolution),
            'origin_x': float(origin.position.x),
            'origin_y': float(origin.position.y),
            'origin_yaw': yaw_from_quaternion(origin.orientation),
        }
        self._render_grid()
        self._emit_edit_state()

    def display_snapshot(self):
        """Copy the fixed navigation map before the shared /map becomes SLAM."""
        if self.grid is None or self.map_meta is None:
            return None
        return {
            'grid': self.grid.copy(),
            'loaded_grid': (
                self.loaded_grid.copy()
                if self.loaded_grid is not None else None
            ),
            'map_meta': dict(self.map_meta),
            'zoom': float(self.zoom),
            'pan': QtCore.QPointF(self.pan),
        }

    def clear_map(self):
        """Drop a stale /map image and all overlays tied to its coordinates."""
        self.image = None
        self.grid = None
        self.loaded_grid = None
        self.map_meta = None
        self.robot = None
        self.requested_pose = None
        self.global_path = []
        self.local_path = []
        self.laser_points = []
        self.goal = None
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._stroke_before = None
        self._stroke_last_cell = None
        self._stroke_changed = False
        self.cancel_interaction('browse')
        self.update()
        self._emit_edit_state()

    def restore_display_snapshot(self, snapshot):
        if not snapshot:
            self.clear_map()
            return
        self.grid = snapshot['grid'].copy()
        loaded_grid = snapshot.get('loaded_grid')
        self.loaded_grid = (
            loaded_grid.copy() if loaded_grid is not None else self.grid.copy()
        )
        self.map_meta = dict(snapshot['map_meta'])
        self.zoom = float(snapshot.get('zoom', 1.0))
        self.pan = QtCore.QPointF(snapshot.get('pan', QtCore.QPointF()))
        self.robot = None
        self.global_path = []
        self.local_path = []
        self.laser_points = []
        self.goal = None
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._stroke_before = None
        self._stroke_last_cell = None
        self._stroke_changed = False
        self.cancel_interaction('browse')
        self._render_grid()
        self._emit_edit_state()

    def _render_grid(self):
        if self.grid is None:
            return
        grid = self.grid
        height, width = grid.shape
        rgb = np.empty((height, width, 3), dtype=np.uint8)
        rgb[:] = (42, 47, 56)
        rgb[grid == 0] = (224, 228, 232)
        rgb[grid >= 65] = (19, 23, 29)
        graded = (grid > 0) & (grid < 65)
        if np.any(graded):
            shade = (220 - grid[graded] * 2.7).clip(50, 220).astype(np.uint8)
            rgb[graded] = np.column_stack((shade, shade, shade))
        rgb = np.ascontiguousarray(np.flipud(rgb))
        self.image = QtGui.QImage(
            rgb.data, width, height, width * 3, QtGui.QImage.Format_RGB888
        ).copy()
        self.update()

    def set_mode(self, mode):
        self.mode = str(mode)
        cursors = {
            'browse': QtCore.Qt.OpenHandCursor,
            'goal': QtCore.Qt.CrossCursor,
            'initial': QtCore.Qt.CrossCursor,
            'draw_obstacle': QtCore.Qt.CrossCursor,
            'erase_obstacle': QtCore.Qt.CrossCursor,
        }
        self.setCursor(cursors.get(self.mode, QtCore.Qt.ArrowCursor))

    def cancel_interaction(self, mode=None):
        """Cancel pointer state that may survive a tab/module switch."""
        self.drag_start = None
        self.drag_last = None
        self.drag_world = None
        self.cursor_point = None
        self._stroke_before = None
        self._stroke_last_cell = None
        self._stroke_changed = False
        self.set_mode(self.mode if mode is None else mode)
        self.update()

    def set_brush_size_cm(self, centimeters):
        self.brush_size_m = max(0.05, float(centimeters) / 100.0)
        self.update()

    def has_edits(self):
        return (
            self.grid is not None
            and self.loaded_grid is not None
            and not np.array_equal(self.grid, self.loaded_grid)
        )

    def edited_map_snapshot(self):
        if self.grid is None or self.map_meta is None:
            return None, None
        return self.grid.copy(), dict(self.map_meta)

    def undo_edit(self):
        if not self._undo_stack or self.grid is None:
            return
        self._redo_stack.append(self.grid.copy())
        self.grid = self._undo_stack.pop()
        self._render_grid()
        self._emit_edit_state()

    def redo_edit(self):
        if not self._redo_stack or self.grid is None:
            return
        self._undo_stack.append(self.grid.copy())
        self.grid = self._redo_stack.pop()
        self._render_grid()
        self._emit_edit_state()

    def discard_edits(self):
        if self.loaded_grid is None:
            return
        self.grid = self.loaded_grid.copy()
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._render_grid()
        self._emit_edit_state()

    def _emit_edit_state(self):
        self.edit_state_changed.emit(
            self.has_edits(),
            bool(self._undo_stack),
            bool(self._redo_stack),
        )

    def _screen_to_grid(self, point):
        if not self.map_meta or self.image is None or self.grid is None:
            return None
        scale = self._fit_scale() * self.zoom
        u = (
            (point.x() - self.width() * 0.5 - self.pan.x()) / scale
            + self.image.width() * 0.5
        )
        v = (
            (point.y() - self.height() * 0.5 - self.pan.y()) / scale
            + self.image.height() * 0.5
        )
        column = int(math.floor(u))
        display_row = int(math.floor(v))
        row = self.grid.shape[0] - 1 - display_row
        if (
            row < 0 or row >= self.grid.shape[0]
            or column < 0 or column >= self.grid.shape[1]
        ):
            return None
        return row, column

    def _paint_grid_cell(self, row, column):
        if self.grid is None or self.map_meta is None:
            return False
        radius = max(
            1,
            int(round(
                self.brush_size_m / self.map_meta['resolution']
            )),
        )
        row_min = max(0, row - radius)
        row_max = min(self.grid.shape[0], row + radius + 1)
        col_min = max(0, column - radius)
        col_max = min(self.grid.shape[1], column + radius + 1)
        rows, columns = np.ogrid[row_min:row_max, col_min:col_max]
        mask = (
            (rows - row) * (rows - row)
            + (columns - column) * (columns - column)
        ) <= radius * radius
        region = self.grid[row_min:row_max, col_min:col_max]
        value = 100 if self.mode == 'draw_obstacle' else 0
        changed = bool(np.any(region[mask] != value))
        if changed:
            region[mask] = value
        return changed

    def _paint_to_screen_point(self, point):
        cell = self._screen_to_grid(point)
        if cell is None:
            return
        previous = self._stroke_last_cell
        radius_cells = max(
            1.0,
            self.brush_size_m / self.map_meta['resolution'],
        )
        if previous is None:
            cells = [cell]
        else:
            delta_row = cell[0] - previous[0]
            delta_column = cell[1] - previous[1]
            distance = math.hypot(delta_row, delta_column)
            steps = max(
                1, int(math.ceil(distance / max(1.0, radius_cells * 0.5)))
            )
            cells = [
                (
                    int(round(previous[0] + delta_row * index / steps)),
                    int(round(previous[1] + delta_column * index / steps)),
                )
                for index in range(1, steps + 1)
            ]
        changed = False
        for row, column in cells:
            changed = self._paint_grid_cell(row, column) or changed
        self._stroke_last_cell = cell
        self._stroke_changed = self._stroke_changed or changed
        if changed:
            self._render_grid()

    def fit_map(self):
        self.zoom = 1.0
        self.pan = QtCore.QPointF(0.0, 0.0)
        self.update()

    def _fit_scale(self):
        if self.image is None:
            return 1.0
        return min(
            max(0.01, (self.width() - 32) / self.image.width()),
            max(0.01, (self.height() - 32) / self.image.height()),
        )

    def _image_to_screen(self, u, v):
        scale = self._fit_scale() * self.zoom
        return QtCore.QPointF(
            self.width() * 0.5 + self.pan.x()
            + (u - self.image.width() * 0.5) * scale,
            self.height() * 0.5 + self.pan.y()
            + (v - self.image.height() * 0.5) * scale,
        )

    def world_to_screen(self, x, y):
        if not self.map_meta or self.image is None:
            return None
        meta = self.map_meta
        dx = x - meta['origin_x']
        dy = y - meta['origin_y']
        cosine = math.cos(meta['origin_yaw'])
        sine = math.sin(meta['origin_yaw'])
        local_x = cosine * dx + sine * dy
        local_y = -sine * dx + cosine * dy
        u = local_x / meta['resolution']
        v = meta['height'] - local_y / meta['resolution']
        return self._image_to_screen(u, v)

    def screen_to_world(self, point):
        if not self.map_meta or self.image is None:
            return None
        scale = self._fit_scale() * self.zoom
        u = (
            (point.x() - self.width() * 0.5 - self.pan.x()) / scale
            + self.image.width() * 0.5
        )
        v = (
            (point.y() - self.height() * 0.5 - self.pan.y()) / scale
            + self.image.height() * 0.5
        )
        meta = self.map_meta
        local_x = u * meta['resolution']
        local_y = (meta['height'] - v) * meta['resolution']
        cosine = math.cos(meta['origin_yaw'])
        sine = math.sin(meta['origin_yaw'])
        return (
            meta['origin_x'] + cosine * local_x - sine * local_y,
            meta['origin_y'] + sine * local_x + cosine * local_y,
        )

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        painter.fillRect(self.rect(), QtGui.QColor('#11151b'))
        if self.image is None:
            painter.setPen(QtGui.QColor('#8993a3'))
            painter.setFont(QtGui.QFont('', 15))
            painter.drawText(self.rect(), QtCore.Qt.AlignCenter, '等待 /map 地图…')
            return
        top_left = self._image_to_screen(0.0, 0.0)
        bottom_right = self._image_to_screen(
            self.image.width(), self.image.height()
        )
        painter.setRenderHint(QtGui.QPainter.SmoothPixmapTransform, False)
        painter.drawImage(
            QtCore.QRectF(top_left, bottom_right), self.image
        )
        painter.setRenderHint(QtGui.QPainter.Antialiasing, True)
        self._draw_path(painter, self.global_path, '#42a5f5', 3.0)
        self._draw_path(painter, self.local_path, '#ffb74d', 2.0)
        if self.show_laser:
            painter.setPen(QtGui.QPen(QtGui.QColor(255, 82, 99, 170), 2))
            for x, y in self.laser_points:
                point = self.world_to_screen(x, y)
                if point is not None and self.rect().contains(point.toPoint()):
                    painter.drawPoint(point)
        for waypoint in self.waypoints:
            self._draw_waypoint(painter, waypoint)
        if self.goal is not None:
            self._draw_arrow(
                painter, self.goal['x'], self.goal['y'], self.goal['yaw'],
                QtGui.QColor('#ffb45c'), 17
            )
        if self.robot is not None:
            self._draw_robot(painter)
        if self.requested_pose is not None:
            pose = self.requested_pose
            self._draw_arrow(painter, pose['x'], pose['y'], pose['yaw'], QtGui.QColor('#ffb45c'), 19)
            point = self.world_to_screen(pose['x'], pose['y'])
            painter.setPen(QtGui.QColor('#ffb45c'))
            painter.drawText(point + QtCore.QPointF(18, -18), '指定位置 · 待确认')
        if self.drag_world is not None and self.drag_last is not None:
            start = self.world_to_screen(*self.drag_world)
            if start is not None:
                color = QtGui.QColor(
                    '#ffb45c' if self.mode == 'goal' else '#55d68b'
                )
                painter.setPen(QtGui.QPen(color, 2))
                painter.drawLine(start, self.drag_last)
                painter.setBrush(color)
                painter.drawEllipse(start, 5, 5)
        if (
            self.mode in ('draw_obstacle', 'erase_obstacle')
            and self.cursor_point is not None
            and self.map_meta is not None
        ):
            radius = (
                self.brush_size_m / self.map_meta['resolution']
                * self._fit_scale() * self.zoom
            )
            color = QtGui.QColor(
                '#e53935' if self.mode == 'draw_obstacle' else '#1976d2'
            )
            painter.setPen(QtGui.QPen(color, 2, QtCore.Qt.DashLine))
            painter.setBrush(QtGui.QColor(color.red(), color.green(),
                                          color.blue(), 35))
            painter.drawEllipse(self.cursor_point, radius, radius)
        self._draw_legend(painter)

    def _draw_path(self, painter, values, color, width):
        if len(values) < 2:
            return
        path = QtGui.QPainterPath()
        first = self.world_to_screen(*values[0])
        if first is None:
            return
        path.moveTo(first)
        for value in values[1:]:
            point = self.world_to_screen(*value)
            if point is not None:
                path.lineTo(point)
        painter.setPen(QtGui.QPen(
            QtGui.QColor(color), width, QtCore.Qt.SolidLine,
            QtCore.Qt.RoundCap, QtCore.Qt.RoundJoin
        ))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawPath(path)

    def _draw_waypoint(self, painter, waypoint):
        point = self.world_to_screen(waypoint['x'], waypoint['y'])
        if point is None:
            return
        color = QtGui.QColor('#33b9ef')
        self._draw_waypoint_heading(
            painter, waypoint['x'], waypoint['y'],
            float(waypoint.get('yaw', 0.0)), color
        )
        painter.setPen(QtGui.QPen(QtGui.QColor('#eaf7ff'), 1))
        painter.setBrush(color)
        painter.drawEllipse(point, 6, 6)
        painter.setPen(QtGui.QColor('#087fd1'))
        painter.setFont(QtGui.QFont('', 10, QtGui.QFont.DemiBold))
        painter.drawText(point + QtCore.QPointF(9, -8), waypoint['name'])

    def _draw_waypoint_heading(self, painter, x, y, yaw, color):
        center = self.world_to_screen(x, y)
        direction = self.world_to_screen(
            x + math.cos(yaw) * 0.25,
            y + math.sin(yaw) * 0.25,
        )
        if center is None or direction is None:
            return
        vector = direction - center
        length = math.hypot(vector.x(), vector.y())
        if length < 1e-3:
            return
        ux, uy = vector.x() / length, vector.y() / length
        px, py = -uy, ux
        start = center + QtCore.QPointF(ux * 5.0, uy * 5.0)
        tip = center + QtCore.QPointF(ux * 22.0, uy * 22.0)
        head = 6.0
        arrow = QtGui.QPolygonF([
            tip,
            tip - QtCore.QPointF(ux * head, uy * head)
            + QtCore.QPointF(px * head * 0.65, py * head * 0.65),
            tip - QtCore.QPointF(ux * head, uy * head)
            - QtCore.QPointF(px * head * 0.65, py * head * 0.65),
        ])
        painter.setPen(QtGui.QPen(color, 2.4, QtCore.Qt.SolidLine,
                                  QtCore.Qt.RoundCap))
        painter.setBrush(color)
        painter.drawLine(start, tip)
        painter.drawPolygon(arrow)

    def _draw_arrow(self, painter, x, y, yaw, color, size):
        center = self.world_to_screen(x, y)
        tip_world = (
            x + math.cos(yaw) * 0.55,
            y + math.sin(yaw) * 0.55,
        )
        tip = self.world_to_screen(*tip_world)
        if center is None or tip is None:
            return
        vector = tip - center
        length = math.hypot(vector.x(), vector.y())
        if length < 1e-3:
            return
        ux, uy = vector.x() / length, vector.y() / length
        px, py = -uy, ux
        polygon = QtGui.QPolygonF([
            center + QtCore.QPointF(px * size * 0.55, py * size * 0.55),
            tip,
            center - QtCore.QPointF(px * size * 0.55, py * size * 0.55),
        ])
        painter.setPen(QtGui.QPen(color, 2))
        painter.setBrush(color)
        painter.drawPolygon(polygon)

    def _draw_robot(self, painter):
        robot = self.robot
        center = self.world_to_screen(robot['x'], robot['y'])
        if center is None:
            return
        if self.map_meta:
            scale = self._fit_scale() * self.zoom / self.map_meta['resolution']
            rx = min(100.0, math.sqrt(robot['cov_x']) * scale)
            ry = min(100.0, math.sqrt(robot['cov_y']) * scale)
            painter.setPen(QtGui.QPen(QtGui.QColor(85, 214, 139, 105), 1))
            painter.setBrush(QtGui.QColor(85, 214, 139, 25))
            painter.drawEllipse(center, rx, ry)
        self._draw_arrow(
            painter, robot['x'], robot['y'], robot['yaw'],
            QtGui.QColor('#55d68b'), 15
        )
        painter.setPen(QtGui.QColor('#087a3f'))
        painter.drawText(center + QtCore.QPointF(10, 18), '机器人')

    def _draw_legend(self, painter):
        box = QtCore.QRectF(14, self.height() - 48, 290, 32)
        painter.setPen(QtGui.QPen(QtGui.QColor('#3a4350'), 1))
        painter.setBrush(QtGui.QColor(20, 25, 32, 220))
        painter.drawRoundedRect(box, 7, 7)
        painter.setFont(QtGui.QFont('', 9))
        items = (
            ('#55d68b', '机器人'), ('#33b9ef', '航点'),
            ('#42a5f5', '全局路径'), ('#ff5263', '激光'),
        )
        x = box.left() + 10
        for color, label in items:
            painter.setBrush(QtGui.QColor(color))
            painter.setPen(QtCore.Qt.NoPen)
            painter.drawEllipse(QtCore.QPointF(x + 4, box.center().y()), 4, 4)
            painter.setPen(QtGui.QColor('#cbd3df'))
            painter.drawText(
                QtCore.QPointF(x + 12, box.center().y() + 4), label
            )
            x += 67

    def wheelEvent(self, event):
        if self.image is None:
            return
        event_point = QtCore.QPointF(event.pos())
        before = self.screen_to_world(event_point)
        factor = 1.18 if event.angleDelta().y() > 0 else 1.0 / 1.18
        self.zoom = min(12.0, max(0.35, self.zoom * factor))
        if before is not None:
            after = self.world_to_screen(*before)
            self.pan += event_point - after
        self.update()

    def mousePressEvent(self, event):
        self.setFocus()
        event_point = QtCore.QPointF(event.pos())
        self.cursor_point = event_point
        self.drag_last = event_point
        if self.mode == 'browse' or event.button() == QtCore.Qt.RightButton:
            self.drag_start = event_point
            self.setCursor(QtCore.Qt.ClosedHandCursor)
        elif (
            self.mode in ('draw_obstacle', 'erase_obstacle')
            and event.button() == QtCore.Qt.LeftButton
            and self.grid is not None
        ):
            self._stroke_before = self.grid.copy()
            self._stroke_last_cell = None
            self._stroke_changed = False
            self._paint_to_screen_point(event_point)
        elif event.button() == QtCore.Qt.LeftButton:
            self.drag_world = self.screen_to_world(event_point)
        self.update()

    def mouseMoveEvent(self, event):
        event_point = QtCore.QPointF(event.pos())
        self.cursor_point = event_point
        world = self.screen_to_world(event_point)
        if world is not None:
            self.coordinate_changed.emit(f'x {world[0]:.2f}　y {world[1]:.2f}')
        if self.drag_start is not None:
            delta = event_point - self.drag_start
            self.pan += delta
            self.drag_start = event_point
        elif (
            self._stroke_before is not None
            and event.buttons() & QtCore.Qt.LeftButton
        ):
            self._paint_to_screen_point(event_point)
        self.drag_last = event_point
        self.update()

    def mouseReleaseEvent(self, event):
        if self.drag_start is not None:
            self.drag_start = None
            self.set_mode(self.mode)
        elif (
            self._stroke_before is not None
            and event.button() == QtCore.Qt.LeftButton
        ):
            if self._stroke_changed:
                self._undo_stack.append(self._stroke_before)
                self._undo_stack = self._undo_stack[-30:]
                self._redo_stack.clear()
            self._stroke_before = None
            self._stroke_last_cell = None
            self._stroke_changed = False
            self._emit_edit_state()
        elif self.drag_world is not None and event.button() == QtCore.Qt.LeftButton:
            end_world = self.screen_to_world(QtCore.QPointF(event.pos()))
            if end_world is not None:
                dx = end_world[0] - self.drag_world[0]
                dy = end_world[1] - self.drag_world[1]
                yaw = math.atan2(dy, dx) if math.hypot(dx, dy) > 0.08 else 0.0
                x, y = self.drag_world
                if self.mode == 'goal':
                    self.goal_requested.emit(x, y, yaw)
                elif self.mode == 'initial':
                    self.initial_pose_requested.emit(x, y, yaw)
            self.drag_world = None
        self.drag_last = None
        self.update()

    def leaveEvent(self, event):
        if self._stroke_before is None:
            self.cursor_point = None
            self.update()
        super().leaveEvent(event)


STYLE = """
QMainWindow, QWidget {
  background: #20242b;
  color: #e7eaf0;
  font-size: 13px;
}
QFrame#header, QFrame#sideCard, QGroupBox {
  background: #292e37;
  border: 1px solid #343b46;
  border-radius: 8px;
}
QFrame#header { border-radius: 0; border-width: 0 0 1px 0; }
QGroupBox {
  margin-top: 10px;
  padding: 12px 8px 8px 8px;
  font-weight: 600;
}
QGroupBox::title {
  subcontrol-origin: margin;
  left: 10px;
  padding: 0 5px;
  color: #cdd5e1;
}
QLabel#title { font-size: 21px; font-weight: 700; color: #f0f3f8; }
QLabel#subtitle, QLabel#hint, QLabel#coordinates { color: #8993a3; }
QLabel#chipGood { color: #55d68b; background: #203b31; border-radius: 10px; padding: 4px 9px; }
QLabel#chipWarn { color: #ffbf69; background: #403426; border-radius: 10px; padding: 4px 9px; }
QLabel#chipBad { color: #ff7c7c; background: #452b30; border-radius: 10px; padding: 4px 9px; }
QLabel#statusInfo { color: #8fc7f3; }
QLabel#statusGood { color: #55d68b; }
QLabel#statusWarn { color: #ffbf69; }
QLabel#statusBad { color: #ff7c7c; }
QPushButton, QToolButton {
  min-height: 32px;
  padding: 4px 10px;
  color: #e7eaf0;
  background: #343b46;
  border: 1px solid #48515f;
  border-radius: 6px;
}
QPushButton:hover, QToolButton:hover { background: #3e4856; }
QPushButton:pressed, QToolButton:pressed { background: #29303a; }
QPushButton:checked, QToolButton:checked {
  background: #2676bd;
  border-color: #4ca7ef;
  color: white;
}
QPushButton#primaryButton { background: #2676bd; border-color: #3897e8; }
QPushButton#dangerButton { color: #ff9a9a; border-color: #934b50; }
QPushButton#stopButton {
  color: white; background: #a54149; border-color: #d05b64; font-weight: 700;
}
QListWidget, QLineEdit, QComboBox, QSpinBox, QDoubleSpinBox {
  background: #171a20;
  border: 1px solid #3a4350;
  border-radius: 6px;
  selection-background-color: #2f7fbd;
  padding: 5px;
}
QListWidget::item { min-height: 28px; padding: 2px 5px; }
QSlider::groove:horizontal { height: 5px; background: #15191f; border-radius: 2px; }
QSlider::sub-page:horizontal { background: #3f94df; border-radius: 2px; }
QSlider::handle:horizontal {
  width: 15px; margin: -5px 0; background: #e8f2fb;
  border: 2px solid #3f94df; border-radius: 8px;
}
QScrollArea { border: 0; }
QWidget#mapCanvas { border: 1px solid #343b46; border-radius: 8px; }
QToolTip { background: #101318; color: #f0f2f5; border: 1px solid #4c5665; }
"""


class WaypointSequenceDialog(QtWidgets.QDialog):
    """Build an ordered waypoint route with duplicates allowed."""

    def __init__(self, waypoint_names, initial_names=None, parent=None):
        super().__init__(parent)
        self.setWindowTitle('编辑多航点导航顺序')
        self.resize(720, 500)
        self.setMinimumSize(620, 420)

        root = QtWidgets.QVBoxLayout(self)
        hint = QtWidgets.QLabel(
            '从左侧反复添加航点到右侧；右侧顺序就是实际导航顺序，'
            '支持拖动排序。'
        )
        hint.setObjectName('hint')
        hint.setWordWrap(True)
        root.addWidget(hint)

        body = QtWidgets.QHBoxLayout()
        available_box = QtWidgets.QGroupBox('可用航点')
        available_layout = QtWidgets.QVBoxLayout(available_box)
        self.available_list = QtWidgets.QListWidget()
        self.available_list.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection
        )
        self.available_list.addItems([str(name) for name in waypoint_names])
        self.available_list.itemDoubleClicked.connect(
            lambda _item: self._add_selected()
        )
        available_layout.addWidget(self.available_list)
        default_dwell_row = QtWidgets.QHBoxLayout()
        default_dwell_row.addWidget(QtWidgets.QLabel('新加入航点默认停留'))
        self.default_dwell_spin = QtWidgets.QDoubleSpinBox()
        self.default_dwell_spin.setRange(0.0, 3600.0)
        self.default_dwell_spin.setDecimals(1)
        self.default_dwell_spin.setSingleStep(1.0)
        self.default_dwell_spin.setSuffix(' 秒')
        self.default_dwell_spin.setValue(0.0)
        default_dwell_row.addWidget(self.default_dwell_spin)
        available_layout.addLayout(default_dwell_row)
        body.addWidget(available_box, 1)

        controls = QtWidgets.QVBoxLayout()
        controls.addStretch()
        for text, slot in (
            ('添加 →', self._add_selected),
            ('← 移除', self._remove_selected),
            ('上移', lambda: self._move_selected(-1)),
            ('下移', lambda: self._move_selected(1)),
            ('清空', self._clear_route),
        ):
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(slot)
            controls.addWidget(button)
        controls.addStretch()
        body.addLayout(controls)

        route_box = QtWidgets.QGroupBox('执行顺序（可重复）')
        route_layout = QtWidgets.QVBoxLayout(route_box)
        self.route_list = QtWidgets.QListWidget()
        self.route_list.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection
        )
        self.route_list.setDragDropMode(
            QtWidgets.QAbstractItemView.InternalMove
        )
        self.route_list.setDefaultDropAction(QtCore.Qt.MoveAction)
        self.route_list.model().rowsMoved.connect(
            lambda *_args: QtCore.QTimer.singleShot(0, self._renumber)
        )
        self.route_list.currentItemChanged.connect(
            self._load_selected_dwell
        )
        route_layout.addWidget(self.route_list)
        dwell_row = QtWidgets.QHBoxLayout()
        dwell_row.addWidget(QtWidgets.QLabel('选中项停留'))
        self.selected_dwell_spin = QtWidgets.QDoubleSpinBox()
        self.selected_dwell_spin.setRange(0.0, 3600.0)
        self.selected_dwell_spin.setDecimals(1)
        self.selected_dwell_spin.setSingleStep(1.0)
        self.selected_dwell_spin.setSuffix(' 秒')
        self.selected_dwell_spin.editingFinished.connect(
            self._apply_selected_dwell
        )
        apply_dwell = QtWidgets.QPushButton('应用')
        apply_dwell.clicked.connect(self._apply_selected_dwell)
        dwell_row.addWidget(self.selected_dwell_spin, 1)
        dwell_row.addWidget(apply_dwell)
        route_layout.addLayout(dwell_row)
        body.addWidget(route_box, 1)
        root.addLayout(body, 1)

        self.loop_checkbox = QtWidgets.QCheckBox('循环执行整个队列')
        root.addWidget(self.loop_checkbox)

        buttons = QtWidgets.QDialogButtonBox(
            QtWidgets.QDialogButtonBox.Cancel
        )
        self.start_button = buttons.addButton(
            '开始多航点导航', QtWidgets.QDialogButtonBox.AcceptRole
        )
        self.start_button.setObjectName('primaryButton')
        self.start_button.clicked.connect(self._accept_if_valid)
        buttons.rejected.connect(self.reject)
        root.addWidget(buttons)

        for name in list(initial_names or []):
            self._append_name(str(name))
        self._renumber()

    def _append_name(self, name, dwell_time=None):
        item = QtWidgets.QListWidgetItem()
        item.setData(QtCore.Qt.UserRole, str(name))
        if dwell_time is None:
            dwell_time = self.default_dwell_spin.value()
        item.setData(QtCore.Qt.UserRole + 1, float(dwell_time))
        self.route_list.addItem(item)

    def _add_selected(self):
        selected = self.available_list.selectedItems()
        if not selected and self.available_list.currentItem() is not None:
            selected = [self.available_list.currentItem()]
        for item in selected:
            self._append_name(item.text())
        self._renumber()
        if self.route_list.count():
            self.route_list.setCurrentRow(self.route_list.count() - 1)

    def _remove_selected(self):
        rows = sorted(
            (self.route_list.row(item)
             for item in self.route_list.selectedItems()),
            reverse=True,
        )
        for row in rows:
            self.route_list.takeItem(row)
        self._renumber()

    def _move_selected(self, offset):
        current = self.route_list.currentRow()
        target = current + int(offset)
        if current < 0 or target < 0 or target >= self.route_list.count():
            return
        item = self.route_list.takeItem(current)
        self.route_list.insertItem(target, item)
        self.route_list.setCurrentItem(item)
        self._renumber()

    def _clear_route(self):
        self.route_list.clear()

    def _load_selected_dwell(self, current, _previous=None):
        if current is None:
            return
        value = float(current.data(QtCore.Qt.UserRole + 1) or 0.0)
        self.selected_dwell_spin.blockSignals(True)
        self.selected_dwell_spin.setValue(value)
        self.selected_dwell_spin.blockSignals(False)

    def _apply_selected_dwell(self):
        selected = self.route_list.selectedItems()
        if not selected and self.route_list.currentItem() is not None:
            selected = [self.route_list.currentItem()]
        value = float(self.selected_dwell_spin.value())
        for item in selected:
            item.setData(QtCore.Qt.UserRole + 1, value)
        self._renumber()

    def _renumber(self):
        for index in range(self.route_list.count()):
            item = self.route_list.item(index)
            name = str(item.data(QtCore.Qt.UserRole) or item.text())
            dwell_time = float(
                item.data(QtCore.Qt.UserRole + 1) or 0.0
            )
            item.setData(QtCore.Qt.UserRole, name)
            item.setData(QtCore.Qt.UserRole + 1, dwell_time)
            item.setText(
                f'{index + 1:02d}.  {name}    停留 {dwell_time:g} 秒'
            )

    def sequence_names(self):
        return [
            str(self.route_list.item(index).data(QtCore.Qt.UserRole))
            for index in range(self.route_list.count())
        ]

    def sequence_dwell_times(self):
        return [
            float(
                self.route_list.item(index).data(QtCore.Qt.UserRole + 1)
                or 0.0
            )
            for index in range(self.route_list.count())
        ]

    def _accept_if_valid(self):
        if self.route_list.count() < 2:
            QtWidgets.QMessageBox.information(
                self, '路线不完整', '请至少加入两个航点。'
            )
            return
        self.accept()


class NavigationWindow(QtWidgets.QMainWindow):
    MOVES = {
        QtCore.Qt.Key_W: (1.0, 0.0, 0.0),
        QtCore.Qt.Key_S: (-1.0, 0.0, 0.0),
        QtCore.Qt.Key_Q: (0.0, 1.0, 0.0),
        QtCore.Qt.Key_E: (0.0, -1.0, 0.0),
        QtCore.Qt.Key_A: (0.0, 0.0, 1.0),
        QtCore.Qt.Key_D: (0.0, 0.0, -1.0),
    }

    def __init__(self, node):
        super().__init__()
        self.node = node
        self.setWindowTitle(PACKAGE_TITLE)
        self.resize(1480, 900)
        self.setMinimumSize(1120, 700)
        self.setStyleSheet(STYLE)
        self._key_filter = ManualKeyFilter()
        self._key_diag = KeyEventDiagnostics()
        self._release_timer = QtCore.QTimer(self)
        self._release_timer.setSingleShot(True)
        self._release_timer.timeout.connect(self._flush_pending_release)
        self._waypoint_markers = []
        self._navigation_active = False
        self._last_chassis_lease_error = None
        self._rgb_fullscreen = False
        self._rgb_split_sizes = None
        self._build_ui()
        QtWidgets.QApplication.instance().installEventFilter(self)
        self._connect_ros()
        self._refresh_map_catalog()
        self.health_timer = QtCore.QTimer(self)
        self.health_timer.setInterval(500)
        self.health_timer.timeout.connect(self._update_health)
        self.health_timer.start()
        # The embedded UI is created before the launch process has necessarily
        # finished creating waypoint_manager.  Retry during startup so a slow
        # ROS graph does not leave a valid on-disk waypoint library looking
        # empty until the user presses “刷新列表”.
        for delay_ms in (1300, 3000, 6000):
            QtCore.QTimer.singleShot(delay_ms, self.node.refresh_waypoints)

    def _build_ui(self):
        root = QtWidgets.QWidget()
        self.setCentralWidget(root)
        outer = QtWidgets.QVBoxLayout(root)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(0)

        header = QtWidgets.QFrame()
        header.setObjectName('header')
        header_layout = QtWidgets.QHBoxLayout(header)
        header_layout.setContentsMargins(18, 11, 18, 11)
        title_box = QtWidgets.QVBoxLayout()
        title = QtWidgets.QLabel(PACKAGE_TITLE)
        title.setObjectName('title')
        subtitle = QtWidgets.QLabel('无需 RViz2 · 地图、重定位、航点与底盘一体控制')
        subtitle.setObjectName('subtitle')
        title_box.addWidget(title)
        title_box.addWidget(subtitle)
        header_layout.addLayout(title_box)
        header_layout.addStretch()
        self.map_chip = self._chip('地图：等待', 'chipWarn')
        self.pose_chip = self._chip('定位：等待', 'chipWarn')
        self.nav_chip = self._chip('导航：待机', 'chipWarn')
        header_layout.addWidget(self.map_chip)
        header_layout.addWidget(self.pose_chip)
        header_layout.addWidget(self.nav_chip)
        outer.addWidget(header)

        body = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        body.setChildrenCollapsible(False)
        outer.addWidget(body, 1)

        map_panel = QtWidgets.QWidget()
        map_layout = QtWidgets.QVBoxLayout(map_panel)
        map_layout.setContentsMargins(12, 12, 6, 10)
        toolbar = QtWidgets.QHBoxLayout()
        self.rgb_toggle = QtWidgets.QToolButton()
        self.rgb_toggle.setText('RGB画面')
        self.rgb_toggle.setCheckable(True)
        self.rgb_toggle.setFocusPolicy(QtCore.Qt.NoFocus)
        self.rgb_toggle.setToolTip('点击并排显示头部画面，再次点击恢复地图')
        self.rgb_toggle.toggled.connect(self._toggle_rgb)
        toolbar.addWidget(self.rgb_toggle)
        self.mode_group = QtWidgets.QButtonGroup(self)
        self.mode_group.setExclusive(True)
        self.mode_buttons = {}
        for text, mode in (
            ('浏览地图', 'browse'),
            ('设置导航目标', 'goal'),
            ('设置初始位姿', 'initial'),
            ('描绘障碍', 'draw_obstacle'),
            ('擦除障碍', 'erase_obstacle'),
        ):
            button = QtWidgets.QToolButton()
            button.setText(text)
            button.setCheckable(True)
            button.setChecked(mode == 'browse')
            button.clicked.connect(partial(self.map_canvas_mode, mode))
            self.mode_group.addButton(button)
            self.mode_buttons[mode] = button
            toolbar.addWidget(button)
        fit_button = QtWidgets.QToolButton()
        fit_button.setText('适应窗口')
        fit_button.clicked.connect(lambda: self.map_canvas.fit_map())
        toolbar.addWidget(fit_button)
        self.laser_toggle = QtWidgets.QCheckBox('显示双激光')
        self.laser_toggle.setChecked(True)
        self.laser_toggle.toggled.connect(self._toggle_laser)
        toolbar.addWidget(self.laser_toggle)
        toolbar.addWidget(QtWidgets.QLabel('笔刷半径'))
        self.brush_size_spin = QtWidgets.QSpinBox()
        self.brush_size_spin.setRange(5, 100)
        self.brush_size_spin.setSingleStep(5)
        self.brush_size_spin.setSuffix(' cm')
        self.brush_size_spin.setValue(20)
        self.brush_size_spin.valueChanged.connect(
            lambda value: self.map_canvas.set_brush_size_cm(value)
        )
        toolbar.addWidget(self.brush_size_spin)
        toolbar.addStretch()
        self.coordinates = QtWidgets.QLabel('x --　y --')
        self.coordinates.setObjectName('coordinates')
        toolbar.addWidget(self.coordinates)
        map_layout.addLayout(toolbar)
        self.map_canvas = MapCanvas()
        self.map_canvas.set_brush_size_cm(self.brush_size_spin.value())
        self._map_minimum_size = self.map_canvas.minimumSize()
        self.visual_splitter = QtWidgets.QSplitter(QtCore.Qt.Horizontal)
        self.visual_splitter.setChildrenCollapsible(False)
        self.visual_splitter.setHandleWidth(8)
        self.visual_splitter.addWidget(self.map_canvas)
        # The shell loads this script by path, not as a Python package.
        rgb_spec = importlib.util.spec_from_file_location(
            '_navigation_head_rgb', FilePath(__file__).with_name('head_rgb_panel.py'))
        rgb_module = importlib.util.module_from_spec(rgb_spec)
        rgb_spec.loader.exec_module(rgb_module)
        self.rgb_panel = rgb_module.HeadRGBPanel()
        self.rgb_panel.enlarge_requested.connect(self._toggle_rgb_fullscreen)
        self.rgb_panel.focus_lost.connect(self._stop_motion)
        self.visual_splitter.addWidget(self.rgb_panel)
        self.visual_splitter.setStretchFactor(0, 1)
        self.visual_splitter.setStretchFactor(1, 1)
        self.rgb_panel.hide()
        map_layout.addWidget(self.visual_splitter, 1)
        body.addWidget(map_panel)

        side_scroll = QtWidgets.QScrollArea()
        side_scroll.setWidgetResizable(True)
        side_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        side = QtWidgets.QWidget()
        side.setMinimumWidth(360)
        side_layout = QtWidgets.QVBoxLayout(side)
        side_layout.setContentsMargins(8, 12, 12, 12)
        side_layout.setSpacing(10)

        reloc_group = QtWidgets.QGroupBox('全局自主定位')
        reloc_layout = QtWidgets.QVBoxLayout(reloc_group)
        self.reloc_state = QtWidgets.QLabel('等待重定位节点…')
        self.reloc_state.setObjectName('statusWarn')
        self.reloc_state.setWordWrap(True)
        self.reloc_detail = QtWidgets.QLabel('启动后将自动进行全地图粗搜与精搜')
        self.reloc_detail.setObjectName('hint')
        self.reloc_detail.setWordWrap(True)
        retry = QtWidgets.QPushButton('重新执行全局定位')
        retry.setObjectName('primaryButton')
        retry.clicked.connect(self._retry_relocalization)
        reloc_layout.addWidget(self.reloc_state)
        reloc_layout.addWidget(self.reloc_detail)
        reloc_layout.addWidget(retry)
        side_layout.addWidget(reloc_group)

        nav_group = QtWidgets.QGroupBox('导航状态')
        nav_layout = QtWidgets.QVBoxLayout(nav_group)
        self.navigation_state = QtWidgets.QLabel('等待航点管理器…')
        self.navigation_state.setWordWrap(True)
        self.navigation_target = QtWidgets.QLabel('目标：无')
        self.navigation_target.setObjectName('hint')
        nav_buttons = QtWidgets.QHBoxLayout()
        cancel = QtWidgets.QPushButton('取消地图目标')
        cancel.setObjectName('dangerButton')
        cancel.clicked.connect(self.node.cancel_direct_goal)
        pause = QtWidgets.QPushButton('暂停')
        pause.clicked.connect(lambda: self._trigger_service('pause'))
        resume = QtWidgets.QPushButton('继续')
        resume.clicked.connect(lambda: self._trigger_service('resume'))
        nav_buttons.addWidget(cancel)
        nav_buttons.addWidget(pause)
        nav_buttons.addWidget(resume)
        nav_layout.addWidget(self.navigation_state)
        nav_layout.addWidget(self.navigation_target)
        nav_layout.addLayout(nav_buttons)
        side_layout.addWidget(nav_group)

        map_edit_group = QtWidgets.QGroupBox('地图编辑与地图库')
        map_edit_layout = QtWidgets.QVBoxLayout(map_edit_group)
        self.map_edit_status = QtWidgets.QLabel('地图未修改')
        self.map_edit_status.setObjectName('hint')
        map_edit_layout.addWidget(self.map_edit_status)
        history_buttons = QtWidgets.QHBoxLayout()
        self.undo_map_button = QtWidgets.QPushButton('撤销')
        self.redo_map_button = QtWidgets.QPushButton('重做')
        self.discard_map_button = QtWidgets.QPushButton('丢弃修改')
        self.undo_map_button.clicked.connect(self.map_canvas.undo_edit)
        self.redo_map_button.clicked.connect(self.map_canvas.redo_edit)
        self.discard_map_button.clicked.connect(self._discard_map_edits)
        for button in (
            self.undo_map_button,
            self.redo_map_button,
            self.discard_map_button,
        ):
            button.setEnabled(False)
            history_buttons.addWidget(button)
        map_edit_layout.addLayout(history_buttons)

        self.save_current_map_button = QtWidgets.QPushButton('保存当前地图')
        self.save_current_map_button.setObjectName('primaryButton')
        self.save_current_map_button.clicked.connect(self._save_current_map)
        map_edit_layout.addWidget(self.save_current_map_button)

        save_row = QtWidgets.QHBoxLayout()
        self.map_name_edit = QtWidgets.QLineEdit()
        self.map_name_edit.setPlaceholderText('输入新地图名称')
        save_map_button = QtWidgets.QPushButton('另存新地图')
        save_map_button.setObjectName('primaryButton')
        save_map_button.clicked.connect(self._save_edited_map)
        save_row.addWidget(self.map_name_edit, 1)
        save_row.addWidget(save_map_button)
        map_edit_layout.addLayout(save_row)

        self.map_selector = QtWidgets.QComboBox()
        map_edit_layout.addWidget(self.map_selector)
        library_buttons = QtWidgets.QHBoxLayout()
        refresh_maps = QtWidgets.QPushButton('刷新地图库')
        refresh_maps.clicked.connect(self._refresh_map_catalog)
        load_map_button = QtWidgets.QPushButton('加载选中地图')
        load_map_button.clicked.connect(self._load_selected_map)
        library_buttons.addWidget(refresh_maps)
        library_buttons.addWidget(load_map_button)
        map_edit_layout.addLayout(library_buttons)
        delete_map_button = QtWidgets.QPushButton('删除选中地图')
        delete_map_button.clicked.connect(self._delete_selected_map)
        map_edit_layout.addWidget(delete_map_button)
        map_edit_hint = QtWidgets.QLabel(
            '保存当前地图会先备份原图；删除可恢复，航点库保留。'
            '加载地图会切换默认地图、对应航点库并重新定位。'
            '涂改仅更新导航栅格，不修改增量建图的历史轨迹。'
        )
        map_edit_hint.setObjectName('hint')
        map_edit_hint.setWordWrap(True)
        map_edit_layout.addWidget(map_edit_hint)
        side_layout.addWidget(map_edit_group)

        waypoint_group = QtWidgets.QGroupBox('航点库')
        waypoint_layout = QtWidgets.QVBoxLayout(waypoint_group)
        self.waypoint_list = QtWidgets.QListWidget()
        self.waypoint_list.setSelectionMode(
            QtWidgets.QAbstractItemView.ExtendedSelection
        )
        self.waypoint_list.itemDoubleClicked.connect(
            lambda _item: self._navigate_waypoint()
        )
        self.waypoint_list.itemClicked.connect(
            lambda item: self.event_label.setText(
                f'已选中“{item.text()}”，点击“导航到选中点”或双击该航点开始；单击不会移动机器人'
            )
        )
        waypoint_layout.addWidget(self.waypoint_list)
        hint = QtWidgets.QLabel('单击选中 · 双击或点击下方按钮开始导航')
        hint.setWordWrap(True)
        waypoint_layout.addWidget(hint)
        wp_buttons = QtWidgets.QGridLayout()
        actions = (
            ('导航到选中点', self._navigate_waypoint, 0, 0),
            ('多航点导航', self._navigate_sequence, 0, 1),
            ('保存当前位置', self._save_waypoint, 1, 0),
            ('刷新列表', self.node.refresh_waypoints, 1, 1),
            ('重命名', self._rename_waypoint, 2, 0),
            ('删除', self._delete_waypoint, 2, 1),
        )
        for text, slot, row, column in actions:
            button = QtWidgets.QPushButton(text)
            button.clicked.connect(slot)
            wp_buttons.addWidget(button, row, column)
        waypoint_layout.addLayout(wp_buttons)
        side_layout.addWidget(waypoint_group)

        motion_group = QtWidgets.QGroupBox('底盘控制（按住移动，松开即停）')
        motion_layout = QtWidgets.QVBoxLayout(motion_group)
        speed_row = QtWidgets.QHBoxLayout()
        speed_row.addWidget(QtWidgets.QLabel('线速度'))
        self.linear_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.linear_slider.setRange(4, 50)
        self.linear_slider.setValue(20)
        self.linear_value = QtWidgets.QLabel('0.20 m/s')
        self.linear_slider.valueChanged.connect(
            lambda value: self.linear_value.setText(f'{value / 100:.2f} m/s')
        )
        speed_row.addWidget(self.linear_slider, 1)
        speed_row.addWidget(self.linear_value)
        motion_layout.addLayout(speed_row)
        angular_row = QtWidgets.QHBoxLayout()
        angular_row.addWidget(QtWidgets.QLabel('角速度'))
        self.angular_slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.angular_slider.setRange(10, 150)
        self.angular_slider.setValue(80)
        self.angular_value = QtWidgets.QLabel('0.80 rad/s')
        self.angular_slider.valueChanged.connect(
            lambda value: self.angular_value.setText(f'{value / 100:.2f} rad/s')
        )
        angular_row.addWidget(self.angular_slider, 1)
        angular_row.addWidget(self.angular_value)
        motion_layout.addLayout(angular_row)
        grid = QtWidgets.QGridLayout()
        for text, vector, row, column in (
            ('Q 左移', (0, 1, 0), 0, 0),
            ('W 前进', (1, 0, 0), 0, 1),
            ('E 右移', (0, -1, 0), 0, 2),
            ('A 左转', (0, 0, 1), 1, 0),
            ('D 右转', (0, 0, -1), 1, 2),
            ('S 后退', (-1, 0, 0), 2, 1),
        ):
            button = QtWidgets.QPushButton(text)
            button.pressed.connect(partial(self._start_motion, *vector))
            button.released.connect(self._release_motion_button)
            grid.addWidget(button, row, column)
        stop = QtWidgets.QPushButton('X 急停')
        stop.setObjectName('stopButton')
        stop.clicked.connect(self._stop_motion)
        grid.addWidget(stop, 1, 1)
        motion_layout.addLayout(grid)
        hint = QtWidgets.QLabel('快捷键：W/S、A/D、Q/E；X 急停；空格取消地图导航')
        hint.setObjectName('hint')
        hint.setWordWrap(True)
        motion_layout.addWidget(hint)
        side_layout.addWidget(motion_group)

        self.event_label = QtWidgets.QLabel('界面已启动，正在连接 ROS…')
        self.event_label.setObjectName('statusInfo')
        self.event_label.setWordWrap(True)
        side_layout.addWidget(self.event_label)
        side_layout.addStretch()
        side_scroll.setWidget(side)
        body.addWidget(side_scroll)
        body.setStretchFactor(0, 1)
        body.setStretchFactor(1, 0)
        body.setSizes([1050, 390])

        self.map_canvas.goal_requested.connect(self._map_goal)
        self.map_canvas.initial_pose_requested.connect(self._initial_pose)
        self.map_canvas.coordinate_changed.connect(self.coordinates.setText)
        self.map_canvas.edit_state_changed.connect(
            self._map_edit_state_changed
        )

    def _toggle_rgb(self, enabled):
        # A layout switch must not strand a held manual-control key.
        self._stop_motion()
        if not enabled:
            if not self._rgb_fullscreen:
                self._rgb_split_sizes = self.visual_splitter.sizes()
            self._rgb_fullscreen = False
            self.rgb_panel.enlarge_button.setText('放大画面')
            self.map_canvas.show()
            self.rgb_panel.hide()
            self.map_canvas.setMinimumSize(self._map_minimum_size)
            self.map_canvas.setFocus(QtCore.Qt.OtherFocusReason)
        else:
            self.map_canvas.setMinimumSize(300, 300)
            self.rgb_panel.show()
            self.visual_splitter.setSizes(self._rgb_split_sizes or [600, 600])
            self.rgb_panel.viewport.setFocus(QtCore.Qt.OtherFocusReason)

    def _toggle_rgb_fullscreen(self):
        if not self.rgb_toggle.isChecked():
            return
        self._stop_motion()
        self._rgb_fullscreen = not self._rgb_fullscreen
        if self._rgb_fullscreen:
            self._rgb_split_sizes = self.visual_splitter.sizes()
            self.map_canvas.hide()
        else:
            self.map_canvas.show()
            self.visual_splitter.setSizes(self._rgb_split_sizes or [600, 600])
        self.rgb_panel.enlarge_button.setText(
            '恢复并排' if self._rgb_fullscreen else '放大画面')
        self.rgb_panel.viewport.setFocus(QtCore.Qt.OtherFocusReason)

    def eventFilter(self, watched, event):
        # Embedded QMainWindow does not itself receive host-window deactivation.
        if (event.type() == QtCore.QEvent.WindowDeactivate
                and watched is self.window() and self.isVisible()
                and self.rgb_toggle.isChecked()):
            self._stop_motion()
        return super().eventFilter(watched, event)

    def _chip(self, text, object_name):
        label = QtWidgets.QLabel(text)
        label.setObjectName(object_name)
        return label

    def _set_chip(self, label, text, state):
        label.setText(text)
        label.setObjectName({
            'good': 'chipGood', 'warn': 'chipWarn', 'bad': 'chipBad'
        }[state])
        label.style().unpolish(label)
        label.style().polish(label)

    def _connect_ros(self):
        signals = self.node.signals
        signals.map_received.connect(self._map_received)
        signals.pose_received.connect(self._pose_received)
        signals.global_path_received.connect(
            lambda msg: self._set_path(msg, False)
        )
        signals.local_path_received.connect(
            lambda msg: self._set_path(msg, True)
        )
        signals.laser_received.connect(self._laser_received)
        signals.waypoint_markers_received.connect(self._markers_received)
        signals.navigation_status.connect(self._navigation_status)
        signals.direct_navigation_status.connect(self._navigation_status)
        signals.relocalization_status.connect(self._relocalization_status)
        signals.service_result.connect(self._service_result)
        signals.chassis_lease_error.connect(self._chassis_lease_error)

    def _chassis_lease_error(self, message):
        if message == self._last_chassis_lease_error:
            return
        self._last_chassis_lease_error = message
        QtWidgets.QMessageBox.warning(
            self, '底盘手动控制不可用',
            '未获得底盘控制所有权，本次按键不会产生非零速度：\n' + message,
        )

    def navigation_map_snapshot(self):
        return self.map_canvas.display_snapshot()

    def prepare_for_user_interaction(self):
        self.map_canvas.cancel_interaction('browse')
        browse_button = self.mode_buttons.get('browse')
        if browse_button is not None:
            browse_button.setChecked(True)

    def clear_navigation_map(self):
        self.map_canvas.clear_map()
        self.prepare_for_user_interaction()
        self.node._last_map_time = 0.0
        self.event_label.setText('建图地图已清除，等待固定导航地图')

    def restore_navigation_map(self, snapshot):
        self.map_canvas.restore_display_snapshot(snapshot)
        self.prepare_for_user_interaction()
        # A restored image is only a display cache. Keep health in a waiting
        # state until map_server actually republishes the fixed map.
        self.node._last_map_time = 0.0
        if snapshot:
            self.event_label.setText(
                '已恢复建图前的固定导航地图；启动导航后将重新连接 map_server'
            )
        else:
            self.event_label.setText('等待固定导航地图')

    def map_canvas_mode(self, mode):
        self.map_canvas.set_mode(mode)
        messages = {
            'browse': '浏览模式：拖动平移，滚轮缩放',
            'goal': '目标模式：按下位置并向朝向方向拖动',
            'initial': '初始位姿：按下位置并向机器人朝向拖动',
            'draw_obstacle': '地图编辑：按住左键描绘障碍，右键拖动地图',
            'erase_obstacle': '地图编辑：按住左键擦除障碍，右键拖动地图',
        }
        self.event_label.setText(messages[mode])

    def _toggle_laser(self, checked):
        self.map_canvas.show_laser = bool(checked)
        self.map_canvas.update()

    def _map_edit_state_changed(self, dirty, can_undo, can_redo):
        self.undo_map_button.setEnabled(bool(can_undo))
        self.redo_map_button.setEnabled(bool(can_redo))
        self.discard_map_button.setEnabled(bool(dirty))
        self.map_edit_status.setText(
            '地图有未保存修改' if dirty else '地图未修改'
        )
        self.map_edit_status.setObjectName(
            'statusWarn' if dirty else 'hint'
        )
        self.map_edit_status.style().unpolish(self.map_edit_status)
        self.map_edit_status.style().polish(self.map_edit_status)

    def _discard_map_edits(self):
        if not self.map_canvas.has_edits():
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            '丢弃地图修改',
            '确认丢弃当前尚未保存的地图修改？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer == QtWidgets.QMessageBox.Yes:
            self.map_canvas.discard_edits()

    def _save_edited_map(self):
        if not self.map_canvas.has_edits():
            QtWidgets.QMessageBox.information(
                self, '地图没有修改', '请先描绘或擦除地图障碍物。'
            )
            return
        name = self.map_name_edit.text().strip()
        if not name:
            QtWidgets.QMessageBox.information(
                self, '需要地图名称', '请输入一个新的地图名称。'
            )
            return
        grid, metadata = self.map_canvas.edited_map_snapshot()
        success, message, path = self.node.save_map_as(
            name, grid, metadata
        )
        self.event_label.setText(message)
        self.event_label.setObjectName(
            'statusGood' if success else 'statusBad'
        )
        self.event_label.style().unpolish(self.event_label)
        self.event_label.style().polish(self.event_label)
        if success:
            self.map_name_edit.clear()
            self._refresh_map_catalog(path)
            self.map_edit_status.setText('新地图已保存，尚未加载')
            self.map_edit_status.setObjectName('statusGood')
            self.map_edit_status.style().unpolish(self.map_edit_status)
            self.map_edit_status.style().polish(self.map_edit_status)
            QtWidgets.QMessageBox.information(
                self,
                '新地图已保存',
                f'{message}\n\n旧地图没有被覆盖。'
                '如需立即使用，请点击“加载选中地图”。',
            )
        else:
            QtWidgets.QMessageBox.warning(self, '保存失败', message)

    def _save_current_map(self):
        if self._navigation_active or self.node._map_load_in_progress:
            QtWidgets.QMessageBox.warning(self, '暂不能保存', '请先停止导航，并等待地图切换完成。')
            return
        if not self.map_canvas.has_edits():
            QtWidgets.QMessageBox.information(self, '地图没有修改', '当前没有需要保存的修改。')
            return
        path = self.node._loaded_map_path
        answer = QtWidgets.QMessageBox.question(
            self, '保存当前地图', f'将覆盖当前地图（不是下拉框预选地图）：\n{path}\n\n'
            '原文件会备份，航点保持不变。导航已启动时将重新加载并重新定位。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            grid, metadata = self.map_canvas.edited_map_snapshot()
            backup = self.node.save_current_map(grid, metadata)
            self.map_canvas.loaded_grid = grid.copy()
            self.map_canvas._undo_stack.clear()
            self.map_canvas._redo_stack.clear()
            self.map_canvas._emit_edit_state()
            self.event_label.setText(f'当前地图已保存；原图备份：{backup}')
            if all(self.node._service_clients[k].service_is_ready()
                   for k in ('load_map', 'switch_waypoint_map')):
                self._stop_motion()
                self.node.load_map(path)
            else:
                QtWidgets.QMessageBox.information(self, '保存成功',
                    '当前地图已保存。导航尚未就绪，下次启动导航时生效。\n原图备份：' + str(backup))
        except Exception as error:
            QtWidgets.QMessageBox.warning(self, '保存失败', str(error))

    def _delete_selected_map(self):
        if self._navigation_active or self.node._map_load_in_progress:
            QtWidgets.QMessageBox.warning(self, '暂不能删除', '请先停止导航，并等待地图切换完成。')
            return
        path = str(self.map_selector.currentData() or '')
        if not path:
            return
        protected = [self.node._loaded_map_path, _read_active_map_yaml(self.node._active_map_config)]
        if FilePath(path).resolve() in {FilePath(p).resolve() for p in protected if p}:
            QtWidgets.QMessageBox.warning(self, '不能删除当前地图', '请先加载并使用另一张地图，再删除该地图。')
            return
        answer = QtWidgets.QMessageBox.question(self, '确认删除地图',
            f'从地图库删除：\n{path}\n\n地图及其独占图像、建图状态将移入.map_trash，可恢复。航点库保留。',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No, QtWidgets.QMessageBox.No)
        if answer != QtWidgets.QMessageBox.Yes:
            return
        try:
            trash = self.node.delete_map(path)
            self._refresh_map_catalog()
            self.event_label.setText(f'地图已移入回收目录：{trash}；航点库保留')
        except Exception as error:
            QtWidgets.QMessageBox.warning(self, '删除失败', str(error))

    def _refresh_map_catalog(self, select_path=None):
        if isinstance(select_path, bool):
            select_path = None
        selected = (
            str(select_path)
            if select_path
            else str(self.map_selector.currentData() or '')
            if hasattr(self, 'map_selector')
            else ''
        )
        entries = self.node.map_catalog()
        self.map_selector.blockSignals(True)
        self.map_selector.clear()
        selected_index = -1
        for index, entry in enumerate(entries):
            self.map_selector.addItem(entry['label'], entry['path'])
            if str(entry['path']) == selected:
                selected_index = index
        if selected_index < 0 and self.node._loaded_map_path:
            for index in range(self.map_selector.count()):
                if (
                    str(self.map_selector.itemData(index))
                    == str(FilePath(
                        self.node._loaded_map_path
                    ).expanduser().resolve())
                ):
                    selected_index = index
                    break
        if selected_index >= 0:
            self.map_selector.setCurrentIndex(selected_index)
        self.map_selector.blockSignals(False)

    def _load_selected_map(self):
        yaml_path = str(self.map_selector.currentData() or '')
        if not yaml_path:
            QtWidgets.QMessageBox.information(
                self, '没有地图', '地图库中没有可加载的地图。'
            )
            return
        if self._navigation_active:
            QtWidgets.QMessageBox.warning(
                self,
                '导航正在运行',
                '请先取消当前导航，再切换地图。',
            )
            return
        warning = '加载所选地图后，将自动重新执行全局定位。'
        if self.map_canvas.has_edits():
            warning = (
                '当前有未保存修改，加载地图后这些修改会丢失。\n\n'
                + warning
            )
        answer = QtWidgets.QMessageBox.question(
            self,
            '确认加载地图',
            f'{warning}\n\n{yaml_path}',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        self._stop_motion()
        self.node.load_map(yaml_path)

    def _map_received(self, message):
        self.node._last_map_time = time.monotonic()
        self.map_canvas.set_map(message)

    def _pose_received(self, pose):
        if pose.get('pending'):
            self.map_canvas.requested_pose = pose
            self.map_canvas.robot = None
            self.map_canvas.update()
            return
        self.map_canvas.requested_pose = None
        self.map_canvas.robot = pose
        self.map_canvas.update()

    def _set_path(self, message, local):
        values = [
            (float(item.pose.position.x), float(item.pose.position.y))
            for item in message.poses
        ]
        if local:
            self.map_canvas.local_path = values
        else:
            self.map_canvas.global_path = values
        self.map_canvas.update()

    def _laser_received(self, points):
        self.map_canvas.laser_points = points
        self.map_canvas.update()

    def _markers_received(self, markers):
        self._waypoint_markers = markers
        self.map_canvas.waypoints = markers
        self.map_canvas.update()

    def _navigation_status(self, payload):
        state = str(payload.get('state', 'unknown'))
        message = str(payload.get('message', state))
        target = str(payload.get('active_waypoint', '') or '')
        self._navigation_active = state in (
            'running', 'accepted', 'started', 'sending', 'advancing',
            'dwelling', 'docking_control', 'docking_staging_approach',
            'undocking', 'far_navigation_refine_pending',
            'far_navigation_refining', 'short_distance_started',
            'short_distance_lateral', 'short_distance_longitudinal',
            'short_distance_yaw',
        )
        self.navigation_state.setText(message)
        self.navigation_target.setText(f'目标：{target or "地图指定位置/无"}')
        if state == 'dwelling':
            self._set_chip(self.nav_chip, '导航：航点停留', 'good')
        elif state == 'operator_arrived':
            self._set_chip(self.nav_chip, '导航：人工确认到达', 'good')
        elif state in (
            'running', 'accepted', 'started', 'sending', 'advancing',
            'far_navigation_refine_pending', 'far_navigation_refining',
            'short_distance_started', 'short_distance_lateral',
            'short_distance_longitudinal', 'short_distance_yaw',
        ):
            self._set_chip(self.nav_chip, '导航：运行中', 'good')
        elif state in ('failed', 'rejected', 'canceled'):
            self._set_chip(self.nav_chip, '导航：异常', 'bad')
        elif state in (
            'succeeded', 'sequence_succeeded', 'short_distance_succeeded',
            'succeeded_by_tolerance',
        ):
            self._set_chip(self.nav_chip, '导航：已到达', 'good')
        else:
            self._set_chip(self.nav_chip, f'导航：{state}', 'warn')

    def _relocalization_status(self, payload):
        state = str(payload.get('state', 'unknown'))
        detail = str(payload.get('detail', state))
        attempt = int(payload.get('attempt', 0) or 0)
        score = float(payload.get('score', 0.0) or 0.0)
        self.reloc_state.setText({
            'waiting': '等待地图与激光',
            'map_ready': '地图已就绪',
            'waiting_scans': '激光数据未就绪',
            'matching': f'正在全地图搜索（第 {attempt} 次）',
            'succeeded': '全局定位成功',
            'failed': '全局定位失败',
        }.get(state, state))
        self.reloc_detail.setText(
            f'{detail}　匹配分数 {score:.3f}' if score else detail
        )
        self.reloc_state.setObjectName(
            'statusGood' if state == 'succeeded'
            else 'statusBad' if state == 'failed'
            else 'statusWarn'
        )
        self.reloc_state.style().unpolish(self.reloc_state)
        self.reloc_state.style().polish(self.reloc_state)

    def _update_health(self):
        lease = self.node._chassis_lease
        lease_error = not lease.held and lease.last_error
        if lease_error:
            self._set_chip(self.pose_chip, '底盘控制权：失败', 'bad')
            self.pose_chip.setToolTip(lease.last_error)
            return
        map_connected = self.node.map_age < 5.0
        map_cached = self.map_canvas.image is not None
        self._set_chip(
            self.map_chip,
            '地图：已连接' if map_connected
            else '地图：已缓存' if map_cached
            else '地图：等待',
            'good' if map_connected else 'warn',
        )
        self._set_chip(
            self.pose_chip,
            '定位：已连接' if self.node.pose_age < 2.0 else '定位：等待',
            'good' if self.node.pose_age < 2.0 else 'warn',
        )
        if self.node.continuity_status.get('state') == 'blocked':
            self._set_chip(self.pose_chip, '定位跳变：请重新定位', 'warn')
        self.pose_chip.setToolTip(self.node.continuity_status.get('detail', ''))

    def _map_goal(self, x, y, yaw):
        self.map_canvas.goal = {'x': x, 'y': y, 'yaw': yaw}
        self.map_canvas.update()
        answer = QtWidgets.QMessageBox.question(
            self, '确认导航目标',
            f'导航到 x={x:.2f}, y={y:.2f}, 朝向={math.degrees(yaw):.1f}°？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.Yes,
        )
        if answer == QtWidgets.QMessageBox.Yes:
            self.node.navigate_to_pose(x, y, yaw)

    def _initial_pose(self, x, y, yaw):
        if self._navigation_active or self.node._map_load_in_progress:
            self.event_label.setText('请先停止导航并等待地图加载完成，再设置初始位姿')
            return
        answer = QtWidgets.QMessageBox.question(
            self, '确认初始位姿',
            f'发布 x={x:.2f}, y={y:.2f}, 朝向={math.degrees(yaw):.1f}°？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.Yes,
        )
        if answer == QtWidgets.QMessageBox.Yes:
            self.node.publish_initial_pose(x, y, yaw)
            self.event_label.setText('正在停止全局搜索并设置手动位置…')

    def _retry_relocalization(self):
        self.node.call_service('relocalize', Trigger.Request())

    def _trigger_service(self, key):
        self.node.call_service(key, Trigger.Request())

    def _selected_names(self):
        return [item.text() for item in self.waypoint_list.selectedItems()]

    def _navigate_waypoint(self):
        names = self._selected_names()
        if not names:
            self.event_label.setText('请先选择一个航点')
            return
        request = NavigateToWaypoint.Request()
        request.name = names[0]
        self.event_label.setText(f'正在请求导航到“{names[0]}”…')
        self.node.call_service('navigate', request, names[0])

    def _navigate_sequence(self):
        all_names = [
            self.waypoint_list.item(index).text()
            for index in range(self.waypoint_list.count())
        ]
        if not all_names:
            self.event_label.setText('航点列表为空，请先保存或刷新航点')
            return
        dialog = WaypointSequenceDialog(
            all_names, self._selected_names(), self
        )
        if dialog.exec_() != QtWidgets.QDialog.Accepted:
            return
        names = dialog.sequence_names()
        request = NavigateWaypointSequence.Request()
        request.names = names
        dwell_times = dialog.sequence_dwell_times()
        request.dwell_times = dwell_times
        request.loop = dialog.loop_checkbox.isChecked()
        route_text = ' → '.join(
            f'{name}（{dwell:g}秒）'
            for name, dwell in zip(names, dwell_times)
        )
        self.event_label.setText('导航顺序：' + route_text)
        self.node.call_service('sequence', request, names)

    def _save_waypoint(self):
        name, accepted = QtWidgets.QInputDialog.getText(
            self, '保存当前位置', '航点名称：'
        )
        if not accepted or not name.strip():
            return
        request = SaveWaypoint.Request()
        request.name = name.strip()
        self.node.call_service('save', request, request.name)

    def _rename_waypoint(self):
        names = self._selected_names()
        if not names:
            self.event_label.setText('请先选择要重命名的航点')
            return
        new_name, accepted = QtWidgets.QInputDialog.getText(
            self, '重命名航点', '新名称：', text=names[0]
        )
        if not accepted or not new_name.strip():
            return
        request = RenameWaypoint.Request()
        request.old_name = names[0]
        request.new_name = new_name.strip()
        self.node.call_service('rename', request, request.new_name)

    def _delete_waypoint(self):
        names = self._selected_names()
        if not names:
            self.event_label.setText('请先选择要删除的航点')
            return
        answer = QtWidgets.QMessageBox.question(
            self, '删除航点', f'确认删除“{names[0]}”？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        request = DeleteWaypoint.Request()
        request.name = names[0]
        self.node.call_service('delete', request, names[0])

    def _service_result(self, key, success, message, data):
        self.event_label.setText(message)
        self.event_label.setObjectName('statusGood' if success else 'statusBad')
        self.event_label.style().unpolish(self.event_label)
        self.event_label.style().polish(self.event_label)
        if key == 'navigate' and not success:
            QtWidgets.QMessageBox.warning(self, '导航未启动', message)
        if key == 'list' and success:
            selected = {item.text() for item in self.waypoint_list.selectedItems()}
            self.waypoint_list.clear()
            for name in list(data or []):
                item = QtWidgets.QListWidgetItem(str(name))
                self.waypoint_list.addItem(item)
                item.setSelected(str(name) in selected)
        elif key in ('save', 'delete', 'rename') and success:
            QtCore.QTimer.singleShot(300, self.node.refresh_waypoints)
        elif key == 'load_map' and success:
            payload = data if isinstance(data, dict) else {'path': data}
            self._refresh_map_catalog(payload.get('path'))
            self.map_canvas.discard_edits()
            self.waypoint_list.clear()
            for name in payload.get('names', []):
                self.waypoint_list.addItem(str(name))
            self._waypoint_markers = []
            self.map_canvas.waypoints = []
            self.map_canvas.update()
            QtCore.QTimer.singleShot(250, self.node.refresh_waypoints)
            QtCore.QTimer.singleShot(1500, self._retry_relocalization)

    def _start_motion(self, x_scale, y_scale, angular_scale):
        linear = self.linear_slider.value() / 100.0
        angular = self.angular_slider.value() / 100.0
        self.node.set_velocity(
            x_scale * linear, y_scale * linear, angular_scale * angular
        )

    def _stop_motion(self, *_unused):
        # Safety stop: X key / 急停 button, focus loss, page switch and window
        # close must not be rate limited, and must clear the ramp so no request
        # survives.
        self._key_filter.clear()
        self._release_timer.stop()
        self.node.stop_immediate()

    def _release_motion_button(self):
        # The on-screen direction buttons mirror the keys, so letting one go is
        # a normal release and decelerates like a key release does.
        self._key_filter.clear()
        self._release_timer.stop()
        self.node.stop()

    def _apply_pressed_motion(self):
        pressed = self._key_filter.pressed_keys()
        x_scale = 0.0
        y_scale = 0.0
        angular_scale = 0.0
        for key in pressed:
            dx, dy, da = self.MOVES[key]
            x_scale += dx
            y_scale += dy
            angular_scale += da

        translation_length = math.hypot(x_scale, y_scale)
        if translation_length > 1.0:
            x_scale /= translation_length
            y_scale /= translation_length
        angular_scale = max(-1.0, min(1.0, angular_scale))

        if not pressed or (
            abs(x_scale) < 1e-6
            and abs(y_scale) < 1e-6
            and abs(angular_scale) < 1e-6
        ):
            # Keys released: zero only the target, so the heartbeat decelerates.
            self.node.stop()
            return
        self._start_motion(x_scale, y_scale, angular_scale)

    def _schedule_release_flush(self):
        """Arm the single-shot timer for the earliest pending key release."""
        deadline = self._key_filter.next_deadline()
        if deadline is None:
            self._release_timer.stop()
            return
        delay_ms = max(
            1, int(math.ceil((deadline - time.monotonic()) * 1000.0)))
        self._release_timer.start(delay_ms)

    def _flush_pending_release(self):
        if self._key_filter.expire():
            self._key_diag.log(
                'expire',
                pressed=sorted(int(key) for key in self._key_filter.pressed_keys()),
            )
            self._apply_pressed_motion()
        self._schedule_release_flush()

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() == QtCore.Qt.Key_Escape and self._rgb_fullscreen:
            self._toggle_rgb_fullscreen()
            event.accept()
            return
        if event.key() in self.MOVES:
            self._key_diag.log(
                'press', key=int(event.key()), autorepeat=False,
                pending=self._key_filter.pending_count)
            self._key_filter.press(event.key())
            self._apply_pressed_motion()
            event.accept()
            return
        if event.key() == QtCore.Qt.Key_X:
            self._key_diag.log('stop_key', key=int(event.key()))
            self._stop_motion()
            event.accept()
            return
        if event.key() == QtCore.Qt.Key_Space:
            self.node.cancel_direct_goal()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            return
        if event.key() in self.MOVES:
            self._key_diag.log(
                'release', key=int(event.key()), autorepeat=False,
                pending=self._key_filter.pending_count)
            if self._key_filter.release(event.key()):
                self._schedule_release_flush()
            self._apply_pressed_motion()
            event.accept()
            return
        super().keyReleaseEvent(event)

    def focusOutEvent(self, event):
        self._key_diag.log('focus_out')
        self._stop_motion()
        super().focusOutEvent(event)

    def hideEvent(self, event):
        # The page is embedded in the control center stack, so switching to the
        # mapping page hides it instead of closing it.  Leaving the page must
        # never leave a requested velocity behind for the heartbeat to ramp to.
        self._key_diag.log('hide')
        self._stop_motion()
        super().hideEvent(event)

    def closeEvent(self, event):
        self._key_diag.log('close')
        self._key_diag.close()
        self._stop_motion()
        self.rgb_panel.shutdown()
        event.accept()


def drain_ros_callbacks(executor, maximum_callbacks=6):
    """Consume a bounded callback batch so sensor messages do not queue up."""
    for _ in range(maximum_callbacks):
        executor.spin_once(timeout_sec=0.0)


def main(args=None):
    rclpy.init(args=args)
    application = QtWidgets.QApplication([sys.argv[0]])
    application.setApplicationName(PACKAGE_TITLE)
    lock = QtCore.QLockFile('/tmp/navigation_global_relocalization_ui.lock')
    lock.setStaleLockTime(5000)
    if not lock.tryLock(100):
        QtWidgets.QMessageBox.warning(
            None, PACKAGE_TITLE, '导航界面已经在运行。'
        )
        rclpy.shutdown()
        return 2
    node = NavigationRosNode()
    window = NavigationWindow(node)
    executor = SingleThreadedExecutor()
    executor.add_node(node)
    spin_timer = QtCore.QTimer()
    spin_timer.setTimerType(QtCore.Qt.PreciseTimer)
    spin_timer.setInterval(5)
    spin_timer.timeout.connect(
        lambda: drain_ros_callbacks(executor)
    )
    spin_timer.start()
    window.show()
    try:
        result = application.exec_()
    finally:
        spin_timer.stop()
        executor.remove_node(node)
        executor.shutdown(timeout_sec=1.0)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        lock.unlock()
    return result


if __name__ == '__main__':
    raise SystemExit(main())
