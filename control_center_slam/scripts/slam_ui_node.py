#!/usr/bin/env python3
import math
from collections import deque
from pathlib import Path
import re
import shutil
import sys
import time

from ament_index_python.packages import get_package_share_directory
import numpy as np
from PyQt5 import QtCore, QtGui, QtWidgets
import rclpy
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
from cartographer_ros_msgs.srv import FinishTrajectory, WriteState, StartTrajectory, GetTrajectoryStates, TrajectoryQuery
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid, Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parent))
from incremental_mapping import resolve_state_file, relative_initial_pose, scan_map_support, refine_initial_pose, validate_state_submaps

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

        @property
        def update_hz(self):
            return 50.0

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


PACKAGE_NAME = 'control_center_slam'
DISPLAY_LASER_POINTS = False


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


def _occupancy_pixels(cells, occupied_color=25):
    """Match the saved map's free/occupied thresholds without filling unknowns."""
    cells = np.asarray(cells)
    pixels = np.full(cells.shape, 205, dtype=np.uint8)
    pixels[(cells >= 0) & (cells <= 25)] = 254
    pixels[cells >= 65] = occupied_color
    return pixels


def _yaw(quaternion):
    siny = 2.0 * (
        quaternion.w * quaternion.z + quaternion.x * quaternion.y
    )
    cosy = 1.0 - 2.0 * (
        quaternion.y * quaternion.y + quaternion.z * quaternion.z
    )
    return math.atan2(siny, cosy)


def _source_maps_directory():
    share = Path(get_package_share_directory(PACKAGE_NAME)).resolve()
    install_prefix = share.parents[1]
    workspace_root = install_prefix.parent.parent
    for source_package in (
        workspace_root / 'src' / PACKAGE_NAME,
        workspace_root / 'src' / 'autolife_robot_control_center' / PACKAGE_NAME,
    ):
        if (source_package / 'package.xml').is_file():
            source_maps = source_package / 'maps'
            source_maps.mkdir(parents=True, exist_ok=True)
            return source_maps
    installed_maps = share / 'maps'
    installed_maps.mkdir(parents=True, exist_ok=True)
    return installed_maps


def _navigation_maps_directory():
    """Return the navigation package's persistent source map directory."""
    share = Path(get_package_share_directory(PACKAGE_NAME)).resolve()
    install_prefix = share.parents[1]
    workspace_root = install_prefix.parent.parent
    for navigation_package in (
        workspace_root / 'src' / 'control_center_navigation',
        workspace_root / 'src' / 'autolife_robot_control_center'
        / 'control_center_navigation',
    ):
        if (navigation_package / 'package.xml').is_file():
            maps = navigation_package / 'maps'
            maps.mkdir(parents=True, exist_ok=True)
            return maps
    try:
        installed = Path(
            get_package_share_directory('control_center_navigation')
        ).resolve() / 'maps'
        installed.mkdir(parents=True, exist_ok=True)
        return installed
    except Exception as error:
        raise RuntimeError('找不到导航模块地图存储目录') from error


def _rotation_xy(quaternion):
    """Return the XY rows of a full quaternion rotation matrix."""
    qx = float(quaternion.x)
    qy = float(quaternion.y)
    qz = float(quaternion.z)
    qw = float(quaternion.w)
    norm = math.sqrt(qx * qx + qy * qy + qz * qz + qw * qw)
    if not math.isfinite(norm) or norm < 1e-9:
        raise ValueError('invalid transform quaternion')
    qx, qy, qz, qw = qx / norm, qy / norm, qz / norm, qw / norm
    return (
        1.0 - 2.0 * (qy * qy + qz * qz),
        2.0 * (qx * qy - qz * qw),
        2.0 * (qx * qy + qz * qw),
        1.0 - 2.0 * (qx * qx + qz * qz),
    )


def _scan_points_in_map(message, transform, maximum_points=900):
    """Vectorize a LaserScan through the complete 3-D sensor rotation."""
    ranges = np.asarray(message.ranges, dtype=np.float64)
    if ranges.size == 0:
        return []
    step = max(1, int(math.ceil(ranges.size / max(1, maximum_points))))
    indexes = np.arange(0, ranges.size, step, dtype=np.int64)
    distances = ranges[indexes]
    valid = (
        np.isfinite(distances)
        & (distances >= float(message.range_min))
        & (distances <= float(message.range_max))
    )
    if not np.any(valid):
        return []
    indexes = indexes[valid]
    distances = distances[valid]
    angles = float(message.angle_min) + indexes * float(message.angle_increment)
    local_x = distances * np.cos(angles)
    local_y = distances * np.sin(angles)
    r00, r01, r10, r11 = _rotation_xy(transform.transform.rotation)
    translation = transform.transform.translation
    world_x = float(translation.x) + r00 * local_x + r01 * local_y
    world_y = float(translation.y) + r10 * local_x + r11 * local_y
    return list(zip(world_x.tolist(), world_y.tolist()))


def _stabilized_scan_points(history, points, voxel_size=0.04):
    """Keep scan endpoints confirmed by consecutive frames.

    Raw rotating lidar endpoints flicker because neighbouring beams do not hit
    exactly the same millimetre.  A 4 cm world-frame voxel and a two-frame
    confirmation remove isolated returns without adding a long-lived trail.
    """
    frame = {}
    for x, y in points:
        key = (int(round(x / voxel_size)), int(round(y / voxel_size)))
        frame.setdefault(key, []).append((float(x), float(y)))
    compact = {
        key: (
            sum(value[0] for value in values) / len(values),
            sum(value[1] for value in values) / len(values),
        )
        for key, values in frame.items()
    }
    history.append(compact)
    if len(history) < 2:
        return list(compact.values())
    accumulated = {}
    for scan in history:
        for key, value in scan.items():
            bucket = accumulated.setdefault(key, [])
            bucket.append(value)
    return [
        (
            sum(value[0] for value in values) / len(values),
            sum(value[1] for value in values) / len(values),
        )
        for values in accumulated.values()
        if len(values) >= 2
    ]


class SlamSignals(QtCore.QObject):
    map_received = QtCore.pyqtSignal(object)
    optimized_trajectory = QtCore.pyqtSignal(object)
    robot_pose = QtCore.pyqtSignal(object)
    laser_points = QtCore.pyqtSignal(str, object)
    chassis_lease_error = QtCore.pyqtSignal(str)


class SlamRosNode(Node):
    def __init__(self, topic_suffix='0_283'):
        super().__init__('control_center_slam_ui')
        self.signals = SlamSignals()
        self.map_age = 1e9
        self.pose_age = 1e9
        self._last_map = 0.0
        self._last_pose = 0.0
        self._velocity = (0.0, 0.0, 0.0)
        self._pending_scans = {}
        self._last_scan_emit = {'front': 0.0, 'rear': 0.0}
        self._last_scan_stamp = {'front': -1, 'rear': -1}
        self._scan_visible = {'front': False, 'rear': False}
        self._scan_history = {
            'front': deque(maxlen=3),
            'rear': deque(maxlen=3),
        }

        map_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            OccupancyGrid, '/map', self._on_map, map_qos
        )
        self.create_subscription(MarkerArray, '/trajectory_node_list',
                                 self._on_optimized_trajectory, 1)
        if DISPLAY_LASER_POINTS:
            laser_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST,
                depth=1,
                reliability=ReliabilityPolicy.BEST_EFFORT,
                durability=DurabilityPolicy.VOLATILE,
            )
            for key, topic in (
                ('front', f'/topic_gv_front_lidar_{topic_suffix}'),
                ('rear', f'/topic_gv_rear_lidar_{topic_suffix}'),
            ):
                self.create_subscription(
                    LaserScan,
                    topic,
                    lambda message, sensor=key: self._on_scan(sensor, message),
                    laser_qos,
                )

        self.tf_buffer = tf2_ros.Buffer(cache_time=Duration(seconds=10.0))
        self.tf_listener = tf2_ros.TransformListener(
            self.tf_buffer, self, spin_thread=False
        )
        self.create_timer(1.0 / 30.0, self._update_robot_pose)
        if DISPLAY_LASER_POINTS:
            self.create_timer(0.01, self._process_pending_scans)
        self.velocity_pub = self.create_publisher(
            Twist, f'/topic_gv_target_cmd_vel_{topic_suffix}', 10
        )
        # Keyboard input only sets the ramp target; the page's velocity timer
        # advances the commanded velocity toward it, so a key press no longer
        # steps the chassis command from zero to the slider speed.
        self._velocity_diagnostics = VelocityRampDiagnostics(tag='slam')
        self._velocity_ramp = VelocityRamp(
            diagnostics=self._velocity_diagnostics)
        self.finish_client = self.create_client(
            FinishTrajectory, '/finish_trajectory'
        )
        self.write_client = self.create_client(WriteState, '/write_state')
        self.start_client = self.create_client(StartTrajectory, '/start_trajectory')
        self.states_client = self.create_client(GetTrajectoryStates, '/get_trajectory_states')
        self.query_client = self.create_client(TrajectoryQuery, '/trajectory_query')
        self.alignment_scans = {}
        self.alignment_odometry = deque(maxlen=80)
        self.create_subscription(Odometry, f'/topic_gv_wheel_odom_{topic_suffix}',
            lambda m: self.alignment_odometry.append((time.monotonic(), m.pose.pose)),
            QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT))
        for side in ('front', 'rear'):
            self.create_subscription(LaserScan, '/mapping_scan/' + side,
                lambda m, side=side: self.alignment_scans.update({side: (time.monotonic(), m)}),
                QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT))
        self._chassis_lease = (
            ChassisControlLease(
                topic_hint=f'/topic_gv_target_cmd_vel_{topic_suffix}',
                logger=self.get_logger(),
                on_lease_lost=self._handle_chassis_lease_lost,
            )
            if ChassisControlLease is not None
            else _UnavailableChassisLease()
        )

    def alignment_points(self):
        now = time.monotonic()
        recent = [(stamp, pose) for stamp, pose in self.alignment_odometry if now-stamp < .5]
        if len(recent) < 3 or now-recent[-1][0] > .2 or recent[-1][0]-recent[0][0] < .3:
            raise ValueError('等待新鲜里程计和静止观测，请保持底盘静止。')
        origin = recent[0][1]
        for _, pose in recent:
            angle = _yaw(pose.orientation)-_yaw(origin.orientation)
            if (math.hypot(pose.position.x-origin.position.x, pose.position.y-origin.position.y) > .01
                    or abs(math.atan2(math.sin(angle), math.cos(angle))) > .015):
                raise ValueError('底盘仍在移动，请停稳后再核对初始定位。')
        points = []
        for side in ('front', 'rear'):
            stamp, scan = self.alignment_scans.get(side, (0, None))
            if scan is None or time.monotonic() - stamp > .5:
                raise ValueError('等待前后雷达的新鲜数据，请保持底盘静止。')
            transform = self.tf_buffer.lookup_transform('base_link', scan.header.frame_id, Time())
            t, q = transform.transform.translation, transform.transform.rotation
            # Full 3D rotation is essential: both scanners are upside down.
            x, y, z, w = q.x, q.y, q.z, q.w
            rotation = np.array([[1-2*(y*y+z*z), 2*(x*y-z*w)],
                                 [2*(x*y+z*w), 1-2*(x*x+z*z)]])
            ranges = np.asarray(scan.ranges)[::3]
            angles = scan.angle_min + np.arange(0, len(scan.ranges), 3) * scan.angle_increment
            good = np.isfinite(ranges) & (ranges > max(.15, scan.range_min)) & (ranges < min(8., scan.range_max))
            laser = np.column_stack((ranges[good]*np.cos(angles[good]), ranges[good]*np.sin(angles[good])))
            points.extend((laser @ rotation.T + [t.x, t.y]).tolist())
        return points

    def _on_optimized_trajectory(self, message):
        # Cartographer publishes complete optimized LINE_STRIPs, including old
        # trajectories. Rebuild instead of mixing samples from different maps.
        segments = []
        for marker in message.markers:
            if (marker.action == Marker.ADD and marker.type == Marker.LINE_STRIP
                    and marker.header.frame_id == 'map' and marker.points):
                segments.append([(float(p.x), float(p.y)) for p in marker.points])
        if segments:
            self.signals.optimized_trajectory.emit(segments)

    def _on_map(self, message):
        self._last_map = time.monotonic()
        self.signals.map_received.emit(message)

    def _update_robot_pose(self):
        self.map_age = (
            time.monotonic() - self._last_map if self._last_map else 1e9
        )
        try:
            transform = self.tf_buffer.lookup_transform(
                'map', 'base_link', Time()
            )
        except Exception:
            self.pose_age = 1e9
            return
        stamp = transform.header.stamp
        self.pose_age = max(0.0, self.get_clock().now().nanoseconds * 1e-9
                            - stamp.sec - stamp.nanosec * 1e-9)
        if self.pose_age > .5:
            return
        self._last_pose = time.monotonic()
        translation = transform.transform.translation
        self.signals.robot_pose.emit({
            'x': float(translation.x),
            'y': float(translation.y),
            'yaw': _yaw(transform.transform.rotation),
        })

    def _on_scan(self, sensor, message):
        # Keep only the newest frame for each lidar. The matching map TF may
        # arrive a few milliseconds after the scan, so conversion is retried
        # by a timer instead of blocking this single-threaded ROS executor.
        stamp = (
            int(message.header.stamp.sec) * 1_000_000_000
            + int(message.header.stamp.nanosec)
        )
        if stamp and stamp <= self._last_scan_stamp[sensor]:
            return
        self._pending_scans[sensor] = (message, time.monotonic(), stamp)

    def _process_pending_scans(self):
        now = time.monotonic()
        for sensor, pending in list(self._pending_scans.items()):
            message, received_at, stamp = pending
            try:
                scan_time = Time.from_msg(message.header.stamp)
                transform = self.tf_buffer.lookup_transform(
                    'map', message.header.frame_id, scan_time
                )
            except Exception:
                if now - received_at > 0.18:
                    self._pending_scans.pop(sensor, None)
                continue
            # A newer message may have replaced this one while callbacks were
            # dispatched. Never remove that newer pending frame.
            if self._pending_scans.get(sensor) is pending:
                self._pending_scans.pop(sensor, None)
            self._last_scan_stamp[sensor] = max(
                self._last_scan_stamp[sensor], stamp
            )
            self._emit_scan(sensor, message, transform)
        # Never leave an old world-fixed cloud painted while the robot keeps
        # moving. A stale cloud was the main visual source of apparent drift.
        for sensor, emitted_at in self._last_scan_emit.items():
            if (
                self._scan_visible[sensor]
                and emitted_at
                and now - emitted_at > 0.30
            ):
                self._scan_visible[sensor] = False
                self._scan_history[sensor].clear()
                self.signals.laser_points.emit(sensor, [])

    def _emit_scan(self, sensor, message, transform):
        try:
            points = _scan_points_in_map(message, transform)
            points = _stabilized_scan_points(
                self._scan_history[sensor], points
            )
        except Exception:
            return
        self._last_scan_emit[sensor] = time.monotonic()
        self._scan_visible[sensor] = bool(points)
        self.signals.laser_points.emit(sensor, points)

    def set_target_velocity(self, linear_x, linear_y, angular_z):
        """Request a velocity; the page timer ramps the command toward it.

        The target applies on this call, so a key press is never delayed, but
        the chassis sees the first limited increment instead of a step from zero.
        """
        nonzero = (
            abs(float(linear_x)) > 1e-6
            or abs(float(linear_y)) > 1e-6
            or abs(float(angular_z)) > 1e-6
        )
        if nonzero and not self._chassis_lease.acquire():
            # Fail closed: publish zeros and keep the state inert.
            self._velocity_ramp.reset()
            self._publish_velocity_values((0.0, 0.0, 0.0))
            self.signals.chassis_lease_error.emit(
                self._chassis_lease.last_error
                or '未获得底盘控制所有权。')
            return
        self._velocity_ramp.set_target(linear_x, linear_y, angular_z)
        self._publish_velocity_values(self._velocity_ramp.step())
        if self._velocity_ramp.idle:
            self._chassis_lease.release()

    def _publish_velocity_values(self, values):
        linear_x, linear_y, angular_z = values
        self._velocity = (float(linear_x), float(linear_y), float(angular_z))
        message = Twist()
        message.linear.x = float(linear_x)
        message.linear.y = float(linear_y)
        message.angular.z = float(angular_z)
        self.velocity_pub.publish(message)

    @property
    def velocity_idle(self):
        """True when no teleop command is outstanding."""
        return self._velocity_ramp.idle

    @property
    def velocity_period(self):
        """Seconds between two ramp ticks, shared with the page timer."""
        return 1.0 / self._velocity_ramp.update_hz

    def _handle_chassis_lease_lost(self, reason):
        """Vendor takeover or Redis failure mid-drive: stop immediately."""
        self.stop_motion()
        self.signals.chassis_lease_error.emit(
            self._chassis_lease.last_error or str(reason))

    def stop_motion(self):
        """Zero the chassis command on this call, bypassing the ramp.

        Reserved for the safety paths -- focus loss, page switch, mapping
        shutdown -- which must keep their original instant-stop meaning and must
        never leave a ramped command outstanding.
        """
        self._velocity_ramp.reset()
        self._publish_velocity_values((0.0, 0.0, 0.0))
        self._chassis_lease.release()

    def destroy_node(self):
        if self._velocity_diagnostics is not None:
            self._velocity_diagnostics.close()
        if self._chassis_lease is not None:
            self._chassis_lease.shutdown()
        super().destroy_node()


class InitialAlignmentWorker(QtCore.QThread):
    result_ready = QtCore.pyqtSignal(object)

    def __init__(self, cells, info, points, seed, parent):
        super().__init__(parent)
        self.inputs = (cells, info, points, seed)

    def run(self):
        try:
            result = refine_initial_pose(*self.inputs, cancelled=self.isInterruptionRequested)
        except Exception as error:
            result = {'error': str(error)}
        if not self.isInterruptionRequested():
            self.result_ready.emit(result)


class SlamCanvas(QtWidgets.QWidget):
    edit_state_changed = QtCore.pyqtSignal(bool, bool, bool)
    initial_pose_selected = QtCore.pyqtSignal(object)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(620, 440)
        self.setMouseTracking(True)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.map_image = None
        self.map_info = None
        self.map_cells = None
        self._saved_cells = None
        self.robot_pose = None
        self.lasers = {'front': [], 'rear': []}
        self.live_trajectory = deque(maxlen=300)
        self._trajectory_path = QtGui.QPainterPath()
        self.zoom = 1.0
        self.pan = QtCore.QPointF(0.0, 0.0)
        self._drag_start = None
        self.mode = 'browse'
        self.editing_enabled = False
        self.brush_size_cm = 20
        self._undo_stack = []
        self._redo_stack = []
        self._stroke_before = None
        self._stroke_last_cell = None
        self._stroke_changed = False
        self.select_initial_pose = False
        self.initial_pose = None
        self._initial_start = None
        self.initial_hint = ''

    def set_map(self, message):
        width = int(message.info.width)
        height = int(message.info.height)
        if width <= 0 or height <= 0:
            return
        cells = np.asarray(message.data, dtype=np.int8).reshape(height, width)
        # Once the final map is being edited, do not let a late latched /map
        # message silently erase the operator's brush strokes.
        if self.has_edits():
            return
        info = {
            'width': width,
            'height': height,
            'resolution': float(message.info.resolution),
            'origin_x': float(message.info.origin.position.x),
            'origin_y': float(message.info.origin.position.y),
            'origin_yaw': _yaw(message.info.origin.orientation),
        }
        if info == self.map_info and np.array_equal(cells, self.map_cells):
            return
        self.map_cells = cells.copy()
        # Streaming maps need one owned byte per cell, not two int16 copies
        # plus an undo baseline. Allocate the baseline only for actual editing.
        self._saved_cells = self.map_cells.copy() if self.editing_enabled else None
        self.map_info = info
        self._rebuild_map_image()
        self.update()

    def _rebuild_map_image(self):
        if self.map_cells is None or self.map_info is None:
            self.map_image = None
            return
        cells = self.map_cells
        height, width = cells.shape
        pixels = _occupancy_pixels(cells)
        image = QtGui.QImage(
            pixels.data,
            width,
            height,
            width,
            QtGui.QImage.Format_Grayscale8,
        ).copy().mirrored(False, True)
        self.map_image = image

    def set_robot_pose(self, pose):
        self.robot_pose = pose
        point = (float(pose['x']), float(pose['y']))
        if not self.live_trajectory:
            self.live_trajectory.append(point)
        else:
            previous = self.live_trajectory[-1]
            distance = math.hypot(point[0] - previous[0], point[1] - previous[1])
            if distance >= 0.025:
                if distance > 1.0:
                    self.live_trajectory.clear()
                self.live_trajectory.append(point)
        self.update()

    def set_optimized_trajectory(self, segments):
        # History is rebuilt only on graph updates; the live tail does not
        # wait for a new graph node (e.g. while insertion is motion-gated).
        path = QtGui.QPainterPath()
        for segment in segments:
            if not segment:
                continue
            path.moveTo(*segment[0])
            for point in segment[1:]:
                path.lineTo(*point)
        self._trajectory_path = path
        if segments and segments[-1] and self.live_trajectory:
            end = segments[-1][-1]
            distances = [math.hypot(p[0]-end[0], p[1]-end[1]) for p in self.live_trajectory]
            nearest = int(np.argmin(distances))
            if distances[nearest] < .08:
                for _ in range(nearest):
                    self.live_trajectory.popleft()
        self.update()

    def set_laser(self, sensor, points):
        self.lasers[sensor] = points
        self.update()

    def fit_map(self):
        self.zoom = 1.0
        self.pan = QtCore.QPointF(0.0, 0.0)
        self.update()

    def set_mode(self, mode):
        if mode not in ('browse', 'draw_obstacle', 'erase_obstacle'):
            raise ValueError(f'Unknown map mode: {mode}')
        if mode != 'browse' and not self.editing_enabled:
            return False
        self.mode = mode
        self.setCursor(
            QtCore.Qt.CrossCursor if mode != 'browse'
            else QtCore.Qt.ArrowCursor
        )
        self.update()
        return True

    def set_editing_enabled(self, enabled):
        self.editing_enabled = bool(enabled)
        if enabled and self._saved_cells is None and self.map_cells is not None:
            self._saved_cells = self.map_cells.copy()
        if not self.editing_enabled:
            self.mode = 'browse'
        self._emit_edit_state()
        self.update()

    def set_brush_size_cm(self, value):
        self.brush_size_cm = max(5, min(100, int(value)))

    def has_edits(self):
        return bool(
            self.map_cells is not None
            and self._saved_cells is not None
            and not np.array_equal(self.map_cells, self._saved_cells)
        )

    def undo_edit(self):
        if not self._undo_stack or self.map_cells is None:
            return
        self._redo_stack.append(self.map_cells.copy())
        self.map_cells = self._undo_stack.pop()
        self._rebuild_map_image()
        self._emit_edit_state()
        self.update()

    def redo_edit(self):
        if not self._redo_stack or self.map_cells is None:
            return
        self._undo_stack.append(self.map_cells.copy())
        self.map_cells = self._redo_stack.pop()
        self._rebuild_map_image()
        self._emit_edit_state()
        self.update()

    def discard_edits(self):
        if self._saved_cells is None:
            return
        self.map_cells = self._saved_cells.copy()
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._rebuild_map_image()
        self._emit_edit_state()
        self.update()

    def commit_edits(self):
        if self.map_cells is None:
            return
        self._saved_cells = self.map_cells.copy()
        self._undo_stack.clear()
        self._redo_stack.clear()
        self._emit_edit_state()

    def edited_snapshot(self):
        if self.map_cells is None or self.map_info is None:
            return None, None
        return self.map_cells.copy(), dict(self.map_info)

    def _emit_edit_state(self):
        self.edit_state_changed.emit(
            self.has_edits(), bool(self._undo_stack), bool(self._redo_stack)
        )

    def _screen_to_grid(self, point):
        if self.map_info is None:
            return None
        rect = self._base_rect()
        if rect.isEmpty() or not rect.contains(point):
            return None
        gx = int((point.x() - rect.left()) / rect.width() * self.map_info['width'])
        gy = int((1.0 - (point.y() - rect.top()) / rect.height()) * self.map_info['height'])
        gx = max(0, min(self.map_info['width'] - 1, gx))
        gy = max(0, min(self.map_info['height'] - 1, gy))
        return gx, gy

    def _paint_cell(self, gx, gy):
        if self.map_cells is None or self.map_info is None:
            return
        radius_m = self.brush_size_cm / 100.0
        radius = max(1, int(round(radius_m / self.map_info['resolution'])))
        y0, y1 = max(0, gy - radius), min(self.map_cells.shape[0], gy + radius + 1)
        x0, x1 = max(0, gx - radius), min(self.map_cells.shape[1], gx + radius + 1)
        yy, xx = np.ogrid[y0:y1, x0:x1]
        mask = (xx - gx) ** 2 + (yy - gy) ** 2 <= radius ** 2
        value = 100 if self.mode == 'draw_obstacle' else 0
        region = self.map_cells[y0:y1, x0:x1]
        before = region[mask].copy()
        region[mask] = value
        if not np.array_equal(before, region[mask]):
            self._stroke_changed = True

    def _paint_to_screen_point(self, point):
        cell = self._screen_to_grid(point)
        if cell is None:
            return
        if self._stroke_last_cell is None:
            cells = [cell]
        else:
            x0, y0 = self._stroke_last_cell
            x1, y1 = cell
            count = max(abs(x1 - x0), abs(y1 - y0), 1)
            cells = [
                (
                    int(round(x0 + (x1 - x0) * index / count)),
                    int(round(y0 + (y1 - y0) * index / count)),
                )
                for index in range(count + 1)
            ]
        for gx, gy in cells:
            self._paint_cell(gx, gy)
        self._stroke_last_cell = cell
        self._rebuild_map_image()
        self.update()

    def _base_rect(self):
        if self.map_image is None:
            return QtCore.QRectF()
        margin = 18.0
        available_w = max(1.0, self.width() - 2.0 * margin)
        available_h = max(1.0, self.height() - 2.0 * margin)
        scale = min(
            available_w / self.map_image.width(),
            available_h / self.map_image.height(),
        ) * self.zoom
        width = self.map_image.width() * scale
        height = self.map_image.height() * scale
        left = (self.width() - width) * 0.5 + self.pan.x()
        top = (self.height() - height) * 0.5 + self.pan.y()
        return QtCore.QRectF(left, top, width, height)

    def _world_to_screen(self, x, y):
        if self.map_info is None:
            return None
        rect = self._base_rect()
        gx = (x - self.map_info['origin_x']) / self.map_info['resolution']
        gy = (y - self.map_info['origin_y']) / self.map_info['resolution']
        px = rect.left() + gx / self.map_info['width'] * rect.width()
        py = rect.top() + (
            1.0 - gy / self.map_info['height']
        ) * rect.height()
        return QtCore.QPointF(px, py)

    def paintEvent(self, _event):
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        painter.fillRect(self.rect(), QtGui.QColor('#e8edf3'))
        if self.map_image is None:
            painter.setPen(QtGui.QColor('#65758a'))
            painter.drawText(
                self.rect(), QtCore.Qt.AlignCenter,
                '正在导入旧地图，显示后请设置初始定位光标……'
                if self.select_initial_pose else '等待 Cartographer 发布 /map……',
            )
            return
        rect = self._base_rect()
        painter.drawImage(rect, self.map_image)
        painter.setClipRect(rect)
        # Transform cached world-space history, rather than rebuilding every
        # historical point at the 30 Hz robot-pose redraw rate.
        painter.save()
        sx = rect.width() / (self.map_info['width'] * self.map_info['resolution'])
        sy = -rect.height() / (self.map_info['height'] * self.map_info['resolution'])
        painter.setTransform(QtGui.QTransform(sx, 0, 0, sy,
            rect.left()-sx*self.map_info['origin_x'],
            rect.bottom()-sy*self.map_info['origin_y']), True)
        pen = QtGui.QPen(QtGui.QColor('#287fd1'), 2.2)
        pen.setCosmetic(True)
        painter.setPen(pen)
        painter.drawPath(self._trajectory_path)
        tail = QtGui.QPainterPath()
        for i, point in enumerate(self.live_trajectory):
            if i == 0:
                tail.moveTo(*point)
            else:
                tail.lineTo(*point)
        if self.live_trajectory and self.robot_pose:
            tail.lineTo(self.robot_pose['x'], self.robot_pose['y'])
        painter.drawPath(tail)
        painter.restore()
        if DISPLAY_LASER_POINTS:
            for sensor, color in (
                ('front', QtGui.QColor(225, 45, 60, 205)),
                ('rear', QtGui.QColor(18, 170, 89, 205)),
            ):
                painter.setPen(QtGui.QPen(color, 2.2))
                for x, y in self.lasers[sensor]:
                    point = self._world_to_screen(x, y)
                    if point is not None:
                        painter.drawPoint(point)
        if self.robot_pose is not None:
            center = self._world_to_screen(
                self.robot_pose['x'], self.robot_pose['y']
            )
            if center is not None:
                painter.setPen(QtGui.QPen(QtGui.QColor('#075f38'), 2))
                painter.setBrush(QtGui.QColor('#12a35c'))
                painter.drawEllipse(center, 9, 9)
                yaw = self.robot_pose['yaw']
                tip = center + QtCore.QPointF(
                    23 * math.cos(yaw), -23 * math.sin(yaw)
                )
                painter.drawLine(center, tip)
        painter.setClipping(False)
        if self.initial_hint:
            hint_rect = QtCore.QRectF(18, 18, self.width()-36, 76)
            painter.setPen(QtCore.Qt.NoPen)
            painter.setBrush(QtGui.QColor(32, 42, 55, 235))
            painter.drawRoundedRect(hint_rect, 10, 10)
            painter.setPen(QtGui.QColor('#ffcf7a'))
            font = painter.font(); font.setPointSize(12); font.setBold(True)
            painter.setFont(font)
            painter.drawText(hint_rect.adjusted(12, 6, -12, -6), QtCore.Qt.AlignVCenter | QtCore.Qt.TextWordWrap, self.initial_hint)
        if self.initial_pose is not None:
            x, y, yaw = self.initial_pose
            center = self._world_to_screen(x, y)
            painter.setPen(QtGui.QPen(QtGui.QColor('#e99d25'), 4))
            painter.drawEllipse(center, 8, 8)
            painter.drawLine(center, center + QtCore.QPointF(35*math.cos(yaw), -35*math.sin(yaw)))
        painter.setPen(QtGui.QColor('#53657b'))
        painter.drawText(
            16, self.height() - 12,
            '蓝线：行驶轨迹 · 右键拖动地图',
        )

    def wheelEvent(self, event):
        factor = 1.15 if event.angleDelta().y() > 0 else 1.0 / 1.15
        self.zoom = max(0.25, min(8.0, self.zoom * factor))
        self.update()

    def mousePressEvent(self, event):
        self.setFocus(QtCore.Qt.MouseFocusReason)
        if self.select_initial_pose and event.button() == QtCore.Qt.LeftButton and self.map_info:
            self._initial_start = QtCore.QPointF(event.pos())
            return
        if event.button() == QtCore.Qt.RightButton:
            self._drag_start = event.pos()
        elif (
            event.button() == QtCore.Qt.LeftButton
            and self.editing_enabled
            and self.mode in ('draw_obstacle', 'erase_obstacle')
            and self.map_cells is not None
        ):
            self._stroke_before = self.map_cells.copy()
            self._stroke_last_cell = None
            self._stroke_changed = False
            self._paint_to_screen_point(QtCore.QPointF(event.pos()))

    def mouseMoveEvent(self, event):
        if self.select_initial_pose and self._initial_start is not None:
            self._preview_initial_pose(self._initial_start, QtCore.QPointF(event.pos()))
            return
        if self._drag_start is not None:
            delta = event.pos() - self._drag_start
            self.pan += QtCore.QPointF(delta)
            self._drag_start = event.pos()
            self.update()
        elif (
            event.buttons() & QtCore.Qt.LeftButton
            and self.editing_enabled
            and self.mode in ('draw_obstacle', 'erase_obstacle')
        ):
            self._paint_to_screen_point(QtCore.QPointF(event.pos()))

    def _preview_initial_pose(self, start, end):
        rect, info = self._base_rect(), self.map_info
        if not rect.contains(start) or (end-start).manhattanLength() < 12:
            return False
        x = info['origin_x']+(start.x()-rect.left())/rect.width()*info['width']*info['resolution']
        y = info['origin_y']+(rect.bottom()-start.y())/rect.height()*info['height']*info['resolution']
        self.initial_pose = (x, y, math.atan2(start.y()-end.y(), end.x()-start.x()))
        self.update()
        return True

    def mouseReleaseEvent(self, event):
        if self.select_initial_pose and event.button() == QtCore.Qt.LeftButton and self._initial_start is not None:
            start, end = self._initial_start, QtCore.QPointF(event.pos())
            self._initial_start = None
            if self._preview_initial_pose(start, end):
                self.initial_pose_selected.emit(self.initial_pose)
            return
        if event.button() == QtCore.Qt.RightButton:
            self._drag_start = None
        elif event.button() == QtCore.Qt.LeftButton and self._stroke_before is not None:
            if self._stroke_changed:
                self._undo_stack.append(self._stroke_before)
                self._undo_stack = self._undo_stack[-20:]
                self._redo_stack.clear()
            self._stroke_before = None
            self._stroke_last_cell = None
            self._stroke_changed = False
            self._emit_edit_state()


class SlamPage(QtWidgets.QWidget):
    stop_requested = QtCore.pyqtSignal()
    MOVES = {
        QtCore.Qt.Key_W: (1.0, 0.0, 0.0),
        QtCore.Qt.Key_S: (-1.0, 0.0, 0.0),
        QtCore.Qt.Key_Q: (0.0, 1.0, 0.0),
        QtCore.Qt.Key_E: (0.0, -1.0, 0.0),
        QtCore.Qt.Key_A: (0.0, 0.0, 1.0),
        QtCore.Qt.Key_D: (0.0, 0.0, -1.0),
    }

    def __init__(self, topic_suffix='0_283', parent=None, resume_state=None):
        super().__init__(parent)
        self.setFocusPolicy(QtCore.Qt.StrongFocus)
        self.node = SlamRosNode(topic_suffix=topic_suffix)
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.node)
        self._key_filter = ManualKeyFilter()
        self._key_diag = KeyEventDiagnostics()
        self._release_timer = QtCore.QTimer(self)
        self._release_timer.setSingleShot(True)
        self._release_timer.timeout.connect(self._flush_pending_release)
        self._save_final = False
        self._trajectory_finished = False
        self._resume_state = Path(resume_state) if resume_state else None
        self._trajectory_id = None if resume_state else 0
        self._resume_busy = False
        self._alignment_worker = None
        self._shutting_down = False
        self._map_prefix = None
        self._map_saver = None
        self._editable_map_prefix = None
        self.maps_directory = _source_maps_directory()
        self.navigation_maps_directory = _navigation_maps_directory()
        self._build_ui()
        self.canvas.setFocusProxy(self)
        self.node.signals.map_received.connect(self.canvas.set_map)
        self.node.signals.robot_pose.connect(self.canvas.set_robot_pose)
        self.node.signals.optimized_trajectory.connect(self.canvas.set_optimized_trajectory)
        if DISPLAY_LASER_POINTS:
            self.node.signals.laser_points.connect(self.canvas.set_laser)
        self.canvas.edit_state_changed.connect(self._map_edit_state_changed)
        self.node.signals.chassis_lease_error.connect(self._chassis_lease_error)
        self._last_chassis_lease_error = None
        self.canvas.select_initial_pose = bool(resume_state)
        self.canvas.initial_pose_selected.connect(self._initial_pose_marked)
        if resume_state:
            self.canvas.initial_hint = '① 请设置定位光标：在机器人大致位置按住左键，朝正前方拖动后松开。'
            self.map_name.setText(self._resume_state.stem + time.strftime('_增量_%Y%m%d_%H%M%S'))
            self._set_save_busy(False)
        self.spin_timer = QtCore.QTimer(self)
        self.spin_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.spin_timer.timeout.connect(self._spin_once)
        self.spin_timer.start(5)
        self.velocity_timer = QtCore.QTimer(self)
        self.velocity_timer.setTimerType(QtCore.Qt.PreciseTimer)
        self.velocity_timer.timeout.connect(self._publish_pressed_motion)
        self.velocity_timer.start(max(
            10, int(round(self.node.velocity_period * 1000.0))))
        self.health_timer = QtCore.QTimer(self)
        self.health_timer.timeout.connect(self._update_health)
        self.health_timer.start(500)

    def _build_ui(self):
        outer = QtWidgets.QHBoxLayout(self)
        outer.setContentsMargins(0, 0, 0, 0)
        outer.setSpacing(10)
        left = QtWidgets.QWidget()
        left_layout = QtWidgets.QVBoxLayout(left)
        left_layout.setContentsMargins(0, 0, 0, 0)
        self.canvas = SlamCanvas()
        tools = QtWidgets.QHBoxLayout()
        fit_button = QtWidgets.QPushButton('适应地图')
        fit_button.clicked.connect(self.canvas_fit)
        self.map_mode_group = QtWidgets.QButtonGroup(self)
        self.map_mode_group.setExclusive(True)
        self.map_mode_buttons = {}
        for text, mode in (
            ('浏览', 'browse'),
            ('描绘障碍', 'draw_obstacle'),
            ('擦除障碍', 'erase_obstacle'),
        ):
            button = QtWidgets.QToolButton()
            button.setText(text)
            button.setCheckable(True)
            button.setChecked(mode == 'browse')
            button.setEnabled(mode == 'browse')
            button.clicked.connect(
                lambda _checked=False, selected=mode: self._set_map_mode(selected)
            )
            self.map_mode_group.addButton(button)
            self.map_mode_buttons[mode] = button
            tools.addWidget(button)
        tools.addWidget(QtWidgets.QLabel('笔刷'))
        self.brush_size = QtWidgets.QSpinBox()
        self.brush_size.setRange(5, 100)
        self.brush_size.setSingleStep(5)
        self.brush_size.setValue(20)
        self.brush_size.setSuffix(' cm')
        self.brush_size.setEnabled(False)
        self.brush_size.valueChanged.connect(self.canvas.set_brush_size_cm)
        tools.addWidget(self.brush_size)
        self.map_chip = QtWidgets.QLabel('地图：等待')
        self.pose_chip = QtWidgets.QLabel('TF：等待')
        tools.addWidget(fit_button)
        tools.addStretch()
        tools.addWidget(self.map_chip)
        tools.addWidget(self.pose_chip)
        left_layout.addLayout(tools)
        left_layout.addWidget(self.canvas, 1)
        outer.addWidget(left, 1)

        side = QtWidgets.QFrame()
        side.setObjectName('slamSide')
        side.setMinimumWidth(340)
        side.setStyleSheet(
            'QFrame#slamSide { background: #252b34; border: 1px solid #3b4552; '
            'border-radius: 9px; }'
            'QFrame#slamSide QLabel { background: transparent; border: 0; color: #dce4ee; }'
            'QFrame#slamSide QLineEdit, QFrame#slamSide QDoubleSpinBox, '
            'QFrame#slamSide QSpinBox { background: #171c23; color: #edf3fa; '
            'border: 1px solid #465363; border-radius: 5px; padding: 5px; }'
            'QFrame#slamSide QPushButton { background: #323b47; color: #e4ebf3; '
            'border: 1px solid #485565; border-radius: 5px; padding: 7px; }'
            'QFrame#slamSide QPushButton:hover { background: #3d4958; }'
            'QFrame#slamSide QPushButton:disabled { color: #687483; background: #292f37; }'
            'QFrame#slamSide QGroupBox { color: #e8eef6; border: 1px solid #414c5a; '
            'border-radius: 7px; margin-top: 10px; padding-top: 10px; font-weight: 700; }'
            'QFrame#slamSide QGroupBox::title { subcontrol-origin: margin; left: 9px; '
            'padding: 0 5px; }'
        )
        side_layout = QtWidgets.QVBoxLayout(side)
        title = QtWidgets.QLabel('建图控制')
        title.setStyleSheet('font-size: 18px; font-weight: 700; border: 0;')
        help_label = QtWidgets.QLabel(
            '点击地图后使用 W/S 前后、A/D 转向、Q/E 横移。\n'
            '普通保存不会中断建图；最终优化保存会结束当前轨迹。'
        )
        help_label.setWordWrap(True)
        help_label.setStyleSheet('color: #9eabbc; border: 0;')
        side_layout.addWidget(title)
        side_layout.addWidget(help_label)
        side_layout.addSpacing(10)
        side_layout.addWidget(QtWidgets.QLabel('线速度'))
        self.linear_spin = QtWidgets.QDoubleSpinBox()
        self.linear_spin.setRange(0.03, 0.60)
        self.linear_spin.setSingleStep(0.03)
        self.linear_spin.setValue(0.18)
        self.linear_spin.setSuffix(' m/s')
        side_layout.addWidget(self.linear_spin)
        side_layout.addWidget(QtWidgets.QLabel('角速度'))
        self.angular_spin = QtWidgets.QDoubleSpinBox()
        self.angular_spin.setRange(0.10, 1.50)
        self.angular_spin.setSingleStep(0.05)
        self.angular_spin.setValue(0.45)
        self.angular_spin.setSuffix(' rad/s')
        side_layout.addWidget(self.angular_spin)
        side_layout.addSpacing(14)
        side_layout.addWidget(QtWidgets.QLabel('地图名称'))
        self.map_name = QtWidgets.QLineEdit(
            time.strftime('map_%Y%m%d_%H%M%S')
        )
        side_layout.addWidget(self.map_name)
        self.checkpoint_button = QtWidgets.QPushButton('优化快照并保存（继续建图）')
        self.final_button = QtWidgets.QPushButton('最终优化并保存（结束建图）')
        self.checkpoint_button.clicked.connect(lambda: self._begin_save(False))
        self.final_button.clicked.connect(lambda: self._begin_save(True))
        side_layout.addWidget(self.checkpoint_button)
        side_layout.addWidget(self.final_button)
        self.resume_status = QtWidgets.QLabel('')
        self.resume_status.setWordWrap(True)
        self.resume_button = QtWidgets.QPushButton('自动精细对齐并继续建图')
        self.resume_button.clicked.connect(self._begin_resume)
        self.resume_status.setVisible(bool(self._resume_state))
        self.resume_button.setVisible(bool(self._resume_state))
        if self._resume_state:
            self.resume_status.setText('① 在地图上设置大致位置和朝向。\n② 自动搜索前后/左右各1.5m、朝向±45°，精细对齐后确认。\n请保持底盘静止。')
        side_layout.addWidget(self.resume_status)
        side_layout.addWidget(self.resume_button)
        self.save_status = QtWidgets.QLabel(
            f'建图保存：\n{self.maps_directory}\n导航同步：\n'
            f'{self.navigation_maps_directory}'
        )
        self.save_status.setWordWrap(True)
        self.save_status.setStyleSheet('color: #9eabbc; border: 0;')
        side_layout.addWidget(self.save_status)

        edit_group = QtWidgets.QGroupBox('最终地图编辑')
        edit_layout = QtWidgets.QVBoxLayout(edit_group)
        self.edit_status = QtWidgets.QLabel(
            '完成“最终优化并保存”后，可描绘或擦除障碍物。'
        )
        self.edit_status.setWordWrap(True)
        edit_layout.addWidget(self.edit_status)
        history = QtWidgets.QHBoxLayout()
        self.undo_button = QtWidgets.QPushButton('撤销')
        self.redo_button = QtWidgets.QPushButton('重做')
        self.discard_button = QtWidgets.QPushButton('还原')
        self.undo_button.clicked.connect(self.canvas.undo_edit)
        self.redo_button.clicked.connect(self.canvas.redo_edit)
        self.discard_button.clicked.connect(self._discard_map_edits)
        for button in (self.undo_button, self.redo_button, self.discard_button):
            button.setEnabled(False)
            history.addWidget(button)
        edit_layout.addLayout(history)
        self.overwrite_button = QtWidgets.QPushButton('覆盖当前已保存地图')
        self.overwrite_button.setEnabled(False)
        self.overwrite_button.setStyleSheet(
            'QPushButton { color: #ffb3b8; border-color: #78464b; }'
        )
        self.overwrite_button.clicked.connect(self._overwrite_saved_map)
        edit_layout.addWidget(self.overwrite_button)
        edit_hint = QtWidgets.QLabel(
            '左键使用笔刷，右键拖动地图。覆盖时保留同目录备份；PBStream 不修改。'
        )
        edit_hint.setWordWrap(True)
        edit_hint.setStyleSheet('color: #8997a8;')
        edit_layout.addWidget(edit_hint)
        side_layout.addWidget(edit_group)
        side_layout.addStretch()
        stop_button = QtWidgets.QPushButton('结束并停止建图模块')
        stop_button.setStyleSheet('color: #a93434;')
        stop_button.clicked.connect(self.stop_requested)
        side_layout.addWidget(stop_button)
        side_scroll = QtWidgets.QScrollArea()
        side_scroll.setObjectName('slamSideScroll')
        side_scroll.setWidgetResizable(True)
        side_scroll.setFrameShape(QtWidgets.QFrame.NoFrame)
        side_scroll.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        side_scroll.setFixedWidth(370)
        side_scroll.setWidget(side)
        side_scroll.setStyleSheet(
            'QScrollArea#slamSideScroll { background: transparent; border: 0; }'
            'QScrollArea#slamSideScroll > QWidget > QWidget { background: transparent; }'
            'QScrollBar:vertical { background: #20262e; width: 10px; margin: 2px; }'
            'QScrollBar::handle:vertical { background: #566273; min-height: 28px; '
            'border-radius: 4px; }'
            'QScrollBar::add-line:vertical, QScrollBar::sub-line:vertical { height: 0; }'
        )
        outer.addWidget(side_scroll)

    def _initial_pose_marked(self, _pose):
        self.resume_status.setText('光标已设置。请保持静止，点击“自动精细对齐并继续建图”。')
        self.canvas.initial_hint = '② 光标已设置：点击右侧“自动精细对齐并继续建图”，系统会在附近自动搜索。'
        self.canvas.update()

    def _check_alignment_unchanged(self):
        # Reject a match computed from a scan captured before the base moved,
        # including movement during the operator's confirmation dialog.
        points = self.node.alignment_points()
        pose = self.node.alignment_odometry[-1][1]
        x, y, yaw = self._alignment_origin
        angle = _yaw(pose.orientation)-yaw
        if (math.hypot(pose.position.x-x, pose.position.y-y) > .02 or
                abs(math.atan2(math.sin(angle), math.cos(angle))) > .02):
            raise ValueError('搜索期间机器人位置发生变化，请停稳后重新自动对齐。')
        return points

    def _begin_resume(self):
        if self._resume_busy or self._trajectory_id is not None:
            return
        if self.canvas.initial_pose is None or self.canvas.map_cells is None:
            self.resume_status.setText('请先在旧地图上用左键拖动箭头，标记实际位置和方向。')
            return
        try:
            if abs(self.canvas.map_info.get('origin_yaw', 0)) > 1e-5:
                raise ValueError('当前地图原点含旋转，不能按普通栅格验证。')
            points = self.node.alignment_points()
            if not all(c.service_is_ready() for c in (self.node.states_client, self.node.query_client, self.node.start_client)):
                raise ValueError('建图状态服务尚未就绪，请稍后重试。')
            self._resume_busy = True
            pose = self.node.alignment_odometry[-1][1]
            self._alignment_origin = (pose.position.x, pose.position.y, _yaw(pose.orientation))
            self._alignment_seed = tuple(self.canvas.initial_pose)
            self.canvas.select_initial_pose = False
            self.resume_button.setEnabled(False)
            self.resume_status.setText('正在附近粗搜并精细对齐，请保持机器人静止……')
            self.canvas.initial_hint = '正在自动对齐：无需精确点击位置，请保持底盘静止。'
            self.canvas.update()
            self._alignment_worker = InitialAlignmentWorker(self.canvas.map_cells.copy(),
                dict(self.canvas.map_info), points, self._alignment_seed, self)
            self._alignment_worker.result_ready.connect(self._alignment_ready)
            worker = self._alignment_worker
            worker.finished.connect(lambda: self._alignment_finished(worker))
            self._alignment_worker.start()
        except Exception as error:
            self._resume_failed(str(error))

    def _alignment_finished(self, worker):
        if self._alignment_worker is worker:
            self._alignment_worker = None
        worker.deleteLater()

    def _alignment_ready(self, result):
        if self._shutting_down:
            return
        try:
            if 'error' in result:
                raise ValueError(result['error'])
            points = self._check_alignment_unchanged()
            corrected = result['pose']
            support = scan_map_support(self.canvas.map_cells, self.canvas.map_info, points, corrected)
            if support < .60:
                raise ValueError('当前激光与搜索结果不再一致，请重新自动对齐。')
            self.canvas.initial_pose = corrected
            shift = math.hypot(corrected[0]-self._alignment_seed[0], corrected[1]-self._alignment_seed[1])
            angle = math.degrees(corrected[2]-self._alignment_seed[2])
            self.canvas.initial_hint = '③ 已自动对齐：橙色箭头为修正后的位置，请确认没有匹配到其他相似区域。'
            self.canvas.update()
            answer = QtWidgets.QMessageBox.question(self, '自动对齐结果',
                f'已修正位置约{shift:.2f}m、朝向{angle:+.1f}°；端点吻合{support:.0%}。\n'
                '请查看修正后的橙色箭头，确认所在区域和大致朝向正确。\n'
                '确认后开始续建；不会自动驱动机器人。')
            if answer != QtWidgets.QMessageBox.Yes:
                self._resume_failed('已保留修正光标；可重新标记位置后再次自动对齐，尚未开始写图。')
                return
            self._check_alignment_unchanged()
            self._confirmed_initial_pose = corrected
            QtCore.QTimer.singleShot(15000, self._resume_timeout)
            self.node.states_client.call_async(GetTrajectoryStates.Request()).add_done_callback(self._resume_states)
        except Exception as error:
            self._resume_failed(str(error))

    def _resume_failed(self, message):
        self._resume_busy = False
        self.canvas.select_initial_pose = True
        self.resume_button.setEnabled(True)
        self.resume_status.setText(message)
        self.canvas.initial_hint = message + '\n左键拖动重新设置定位光标，再点击自动对齐。'
        self.canvas.update()

    def _resume_timeout(self):
        if self._resume_busy and self._trajectory_id is None:
            self.resume_status.setText('状态服务响应较慢，仍在等待。不要重复启动；若长时间无响应，请停止模块后重新导入。')

    def _resume_states(self, future):
        try:
            response = future.result()
            if response.status.code != 0:
                raise ValueError(response.status.message)
            states = response.trajectory_states
            if 0 in states.trajectory_state:
                raise ValueError('已有活动轨迹，拒绝重复开始；请停止模块后重新导入。')
            references = [i for i, s in zip(states.trajectory_id, states.trajectory_state) if s in (1, 2)]
            if not references:
                raise ValueError('导入文件没有可用历史轨迹。')
            self._reference_id = min(references)
            request = TrajectoryQuery.Request(); request.trajectory_id = self._reference_id
            self.node.query_client.call_async(request).add_done_callback(self._resume_reference)
        except Exception as error:
            self._resume_failed(str(error))

    def _resume_reference(self, future):
        try:
            self._check_alignment_unchanged()
            response = future.result()
            if response.status.code != 0 or not response.trajectory:
                raise ValueError('无法读取历史轨迹起点，未开始续建。')
            reference = response.trajectory[0].pose
            relative = relative_initial_pose(self._confirmed_initial_pose,
                (reference.position.x, reference.position.y, _yaw(reference.orientation)))
            request = StartTrajectory.Request()
            request.configuration_directory = str(Path(get_package_share_directory(PACKAGE_NAME)) / 'config')
            request.configuration_basename = 'multi_lidar_incremental.lua'
            request.use_initial_pose = True
            request.relative_to_trajectory_id = self._reference_id
            request.initial_pose.position.x, request.initial_pose.position.y = relative[:2]
            request.initial_pose.orientation.z = math.sin(relative[2]/2)
            request.initial_pose.orientation.w = math.cos(relative[2]/2)
            self.node.start_client.call_async(request).add_done_callback(self._resume_started)
        except Exception as error:
            self._resume_failed(str(error))

    def _resume_started(self, future):
        try:
            response = future.result()
            if response.status.code != 0:
                raise ValueError(response.status.message)
            self._trajectory_id = response.trajectory_id
            self._resume_busy = False
            self.canvas.select_initial_pose = False
            self.canvas.initial_pose = None
            self.canvas.initial_hint = ''
            self.resume_button.hide()
            self.resume_status.setText('已进入增量建图。请先缓慢复扫附近旧区域再扩展；移走的障碍物需充分复扫，随旧子图替换更新。')
            self._set_save_busy(False)
        except Exception as error:
            self._resume_failed(str(error))

    def _set_map_mode(self, mode):
        if self.canvas.set_mode(mode):
            return
        self.map_mode_buttons['browse'].setChecked(True)
        self.canvas.set_mode('browse')
        QtWidgets.QMessageBox.information(
            self,
            '地图尚不可编辑',
            '请先执行“最终优化并保存”，再描绘或擦除已保存地图。',
        )

    def _set_map_edit_available(self, available):
        available = bool(available and self.canvas.map_cells is not None)
        self.canvas.set_editing_enabled(available)
        for mode in ('draw_obstacle', 'erase_obstacle'):
            self.map_mode_buttons[mode].setEnabled(available)
        self.brush_size.setEnabled(available)
        self.overwrite_button.setEnabled(available and self.canvas.has_edits())
        if available:
            self.edit_status.setText(
                f'可编辑：{self._editable_map_prefix.name}'
            )

    def _map_edit_state_changed(self, dirty, can_undo, can_redo):
        self.undo_button.setEnabled(bool(can_undo))
        self.redo_button.setEnabled(bool(can_redo))
        self.discard_button.setEnabled(bool(dirty))
        self.overwrite_button.setEnabled(
            bool(dirty and self._editable_map_prefix is not None)
        )
        if dirty:
            self.edit_status.setText('地图有尚未覆盖保存的修改')
        elif self._editable_map_prefix is not None:
            self.edit_status.setText(
                f'已保存：{self._editable_map_prefix.name}'
            )

    def _discard_map_edits(self):
        if not self.canvas.has_edits():
            return
        answer = QtWidgets.QMessageBox.question(
            self,
            '还原地图修改',
            '确认丢弃尚未覆盖保存的所有地图修改？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer == QtWidgets.QMessageBox.Yes:
            self.canvas.discard_edits()

    @staticmethod
    def _map_files_payload(cells, metadata, prefix):
        pixels = _occupancy_pixels(cells, occupied_color=0)
        pgm_pixels = np.flipud(pixels)
        height, width = pgm_pixels.shape
        pgm = (
            f'P5\n# edited by control_center_slam\n{width} {height}\n255\n'.encode('ascii')
            + pgm_pixels.tobytes(order='C')
        )
        yaml_record = {
            'image': prefix.with_suffix('.pgm').name,
            'mode': 'trinary',
            'resolution': float(metadata['resolution']),
            'origin': [
                float(metadata['origin_x']),
                float(metadata['origin_y']),
                float(metadata.get('origin_yaw', 0.0)),
            ],
            'negate': 0,
            'occupied_thresh': 0.65,
            # PGM unknown=205 must remain unknown after loading (50/255).
            # The 25% occupancy decision has already been encoded above.
            'free_thresh': 0.196,
        }
        yaml_text = yaml.safe_dump(
            yaml_record, allow_unicode=True, sort_keys=False
        )
        return pgm, yaml_text

    def _sync_map_bundle_to_navigation(self, prefix, replace_existing=False):
        source_prefix = Path(prefix).resolve()
        destination_prefix = self.navigation_maps_directory / source_prefix.name
        if source_prefix.parent == destination_prefix.parent.resolve():
            return destination_prefix
        suffixes = [
            suffix for suffix in ('.yaml', '.pgm', '.pbstream')
            if source_prefix.with_suffix(suffix).is_file()
        ]
        if '.yaml' not in suffixes or '.pgm' not in suffixes:
            raise FileNotFoundError('建图结果缺少 YAML 或 PGM，不能同步到导航地图库')
        destinations = {
            suffix: destination_prefix.with_suffix(suffix) for suffix in suffixes
        }
        existing = [path for path in destinations.values() if path.exists()]
        if existing and not replace_existing:
            raise FileExistsError(
                '导航地图库已有同名地图，拒绝覆盖：\n'
                + '\n'.join(str(path) for path in existing)
            )

        stamp = time.strftime('%Y%m%d_%H%M%S')
        backups = {}
        temporaries = {}
        created = []
        try:
            for suffix, destination in destinations.items():
                if destination.exists():
                    backup = destination.with_name(
                        f'{destination.name}.before_slam_sync_{stamp}.bak'
                    )
                    shutil.copy2(destination, backup)
                    backups[destination] = backup
                temporary = destination.with_name(
                    f'.{destination.name}.slam-sync.tmp'
                )
                temporary.unlink(missing_ok=True)
                shutil.copy2(source_prefix.with_suffix(suffix), temporary)
                temporaries[destination] = temporary
            for destination, temporary in temporaries.items():
                was_present = destination.exists()
                temporary.replace(destination)
                if not was_present:
                    created.append(destination)
        except Exception:
            for temporary in temporaries.values():
                temporary.unlink(missing_ok=True)
            for destination in created:
                destination.unlink(missing_ok=True)
            for destination, backup in backups.items():
                if backup.is_file():
                    shutil.copy2(backup, destination)
            raise
        return destination_prefix

    def _overwrite_saved_map(self):
        if self._editable_map_prefix is None or not self.canvas.has_edits():
            return
        prefix = self._editable_map_prefix
        pgm_path = prefix.with_suffix('.pgm')
        yaml_path = prefix.with_suffix('.yaml')
        answer = QtWidgets.QMessageBox.warning(
            self,
            '确认覆盖地图',
            '即将用当前绘制结果覆盖：\n'
            f'{pgm_path}\n{yaml_path}\n\n'
            '系统会先在同目录创建带时间戳的备份，PBStream 保持不变。继续吗？',
            QtWidgets.QMessageBox.Yes | QtWidgets.QMessageBox.No,
            QtWidgets.QMessageBox.No,
        )
        if answer != QtWidgets.QMessageBox.Yes:
            return
        cells, metadata = self.canvas.edited_snapshot()
        if cells is None:
            return
        stamp = time.strftime('%Y%m%d_%H%M%S')
        pgm_backup = prefix.with_name(
            f'{prefix.name}.before_edit_{stamp}.pgm.bak'
        )
        yaml_backup = prefix.with_name(
            f'{prefix.name}.before_edit_{stamp}.yaml.bak'
        )
        pgm_tmp = prefix.with_name(f'.{prefix.name}.pgm.tmp')
        yaml_tmp = prefix.with_name(f'.{prefix.name}.yaml.tmp')
        try:
            if not pgm_path.is_file() or not yaml_path.is_file():
                raise FileNotFoundError('原始 PGM/YAML 文件不完整，拒绝覆盖')
            shutil.copy2(pgm_path, pgm_backup)
            shutil.copy2(yaml_path, yaml_backup)
            pgm_payload, yaml_payload = self._map_files_payload(
                cells, metadata, prefix
            )
            pgm_tmp.write_bytes(pgm_payload)
            yaml_tmp.write_text(yaml_payload, encoding='utf-8')
            pgm_tmp.replace(pgm_path)
            yaml_tmp.replace(yaml_path)
            navigation_prefix = self._sync_map_bundle_to_navigation(
                prefix, replace_existing=True
            )
        except Exception as error:
            pgm_tmp.unlink(missing_ok=True)
            yaml_tmp.unlink(missing_ok=True)
            if pgm_backup.is_file():
                shutil.copy2(pgm_backup, pgm_path)
            if yaml_backup.is_file():
                shutil.copy2(yaml_backup, yaml_path)
            QtWidgets.QMessageBox.critical(
                self, '覆盖地图失败', str(error)
            )
            return
        self.canvas.commit_edits()
        text = (
            f'已覆盖地图：{prefix.name}\n'
            f'备份：{pgm_backup.name}、{yaml_backup.name}\n'
            f'导航地图已同步：{navigation_prefix}'
        )
        self.save_status.setText(text)
        QtWidgets.QMessageBox.information(self, '地图覆盖完成', text)

    def canvas_fit(self):
        self.canvas.fit_map()
        self.setFocus(QtCore.Qt.OtherFocusReason)

    def _spin_once(self):
        for _ in range(5):
            self.executor.spin_once(timeout_sec=0.0)

    def _update_health(self):
        lease = self.node._chassis_lease
        if not lease.held and lease.last_error:
            self.pose_chip.setText('底盘控制权：失败')
            self.pose_chip.setStyleSheet('color: #c0392b;')
            self.pose_chip.setToolTip(lease.last_error)
            return
        map_good = self.node.map_age < 2.0
        pose_good = self.node.pose_age < 1.0
        self.map_chip.setText('地图：正常' if map_good else '地图：等待')
        self.pose_chip.setText('TF：正常' if pose_good else 'TF：等待')
        self.map_chip.setStyleSheet(
            'color: #167342;' if map_good else 'color: #a36b00;'
        )
        self.pose_chip.setStyleSheet(
            'color: #167342;' if pose_good else 'color: #a36b00;'
        )

    def keyPressEvent(self, event):
        if event.isAutoRepeat():
            event.accept()
            return
        if event.key() in self.MOVES:
            self._key_diag.log(
                'press', key=int(event.key()), autorepeat=False,
                pending=self._key_filter.pending_count)
            self._key_filter.press(event.key())
            self._publish_pressed_motion()
            event.accept()
            return
        super().keyPressEvent(event)

    def keyReleaseEvent(self, event):
        if event.isAutoRepeat():
            event.accept()
            return
        if event.key() in self.MOVES:
            self._key_diag.log(
                'release', key=int(event.key()), autorepeat=False,
                pending=self._key_filter.pending_count)
            if self._key_filter.release(event.key()):
                self._schedule_release_flush()
            self._publish_pressed_motion()
            event.accept()
            return
        super().keyReleaseEvent(event)

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
            self._publish_pressed_motion()
        self._schedule_release_flush()

    def focusOutEvent(self, event):
        if self._shutting_down:
            return super().focusOutEvent(event)
        self._key_diag.log('focus_out')
        self._key_filter.clear()
        self._release_timer.stop()
        self.node.stop_motion()
        super().focusOutEvent(event)

    def _chassis_lease_error(self, message):
        if message == self._last_chassis_lease_error:
            return
        self._last_chassis_lease_error = message
        QtWidgets.QMessageBox.warning(
            self, '底盘手动控制不可用',
            '未获得底盘控制所有权，本次按键不会产生非零速度：\n' + message,
        )

    def _publish_pressed_motion(self):
        if self._trajectory_id is None:
            # No mapping session: a command left over from a previous one must
            # not be ramped anywhere, and nothing may stay outstanding.
            if not self.node.velocity_idle:
                self.node.stop_motion()
            return
        linear_x = linear_y = angular = 0.0
        for key in self._key_filter.pressed_keys():
            dx, dy, da = self.MOVES[key]
            linear_x += dx
            linear_y += dy
            angular += da
        linear_x = max(-1.0, min(1.0, linear_x)) * self.linear_spin.value()
        linear_y = max(-1.0, min(1.0, linear_y)) * self.linear_spin.value()
        angular = max(-1.0, min(1.0, angular)) * self.angular_spin.value()
        # Keys released only zero the target; this timer decelerates the command.
        self.node.set_target_velocity(linear_x, linear_y, angular)

    def _set_save_busy(self, busy):
        self.checkpoint_button.setEnabled(not busy and not self._trajectory_finished and self._trajectory_id is not None)
        self.final_button.setEnabled(not busy and not self._trajectory_finished and self._trajectory_id is not None)
        self.map_name.setEnabled(not busy and not self._trajectory_finished)

    def _begin_save(self, final):
        name = self.map_name.text().strip()
        for suffix in ('.yaml', '.pgm', '.pbstream'):
            if name.lower().endswith(suffix):
                name = name[:-len(suffix)]
        if not re.fullmatch(r'[\w\-\u4e00-\u9fff]+', name):
            QtWidgets.QMessageBox.warning(
                self, '地图名称无效',
                '地图名称只能包含中文、字母、数字、下划线和连字符。',
            )
            return
        prefix = self.maps_directory / name
        existing = [
            str(prefix.with_suffix(suffix))
            for suffix in ('.yaml', '.pgm', '.pbstream')
            if prefix.with_suffix(suffix).exists()
        ]
        navigation_prefix = self.navigation_maps_directory / name
        existing.extend(
            str(navigation_prefix.with_suffix(suffix))
            for suffix in ('.yaml', '.pgm', '.pbstream')
            if navigation_prefix.with_suffix(suffix).exists()
        )
        if existing:
            QtWidgets.QMessageBox.warning(
                self, '不会覆盖旧地图',
                '以下文件已经存在，请更换地图名称：\n' + '\n'.join(existing),
            )
            return
        self._save_final = bool(final)
        self._map_prefix = prefix
        self._set_save_busy(True)
        if final:
            if not self.node.finish_client.service_is_ready():
                self._save_failed('找不到 /finish_trajectory 服务。')
                return
            self.save_status.setText('1/3 正在执行最终优化……')
            request = FinishTrajectory.Request()
            request.trajectory_id = self._trajectory_id
            future = self.node.finish_client.call_async(request)
            future.add_done_callback(self._finish_complete)
        else:
            self._write_state()

    def _finish_complete(self, future):
        try:
            response = future.result()
            if response.status.code != 0:
                self._save_failed('最终优化失败：' + response.status.message)
                return
            self._trajectory_finished = True
            self._write_state()
        except Exception as error:
            self._save_failed(f'最终优化调用异常：{error}')

    def _write_state(self):
        if not self.node.write_client.service_is_ready():
            self._save_failed('找不到 /write_state 服务。')
            return
        self.save_status.setText(
            '2/3 正在保存 Cartographer 状态……'
            if self._save_final else
            '1/2 正在保存可继续建图的 Cartographer 快照……'
        )
        request = WriteState.Request()
        request.filename = str(self._map_prefix.with_suffix('.pbstream'))
        # Finishing a trajectory does NOT seal its last active Submap2D grids.
        # Excluding those grids loses the entire map for a short mapping session.
        request.include_unfinished_submaps = True
        future = self.node.write_client.call_async(request)
        future.add_done_callback(self._write_complete)

    def _write_complete(self, future):
        try:
            response = future.result()
            if response.status.code != 0:
                self._save_failed('保存 pbstream 失败：' + response.status.message)
                return
            validate_state_submaps(self._map_prefix.with_suffix('.pbstream'))
            self._start_map_saver()
        except Exception as error:
            self._save_failed(f'保存状态调用异常：{error}')

    def _start_map_saver(self):
        self.save_status.setText(
            '3/3 正在生成 YAML/PGM……'
            if self._save_final else '2/2 正在生成 YAML/PGM……'
        )
        process = QtCore.QProcess(self)
        process.setProcessChannelMode(QtCore.QProcess.MergedChannels)
        process.finished.connect(self._map_saver_finished)
        self._map_saver = process
        process.start(
            'ros2',
            [
                'run', 'nav2_map_server', 'map_saver_cli',
                '-f', str(self._map_prefix),
            ],
        )

    def _map_saver_finished(self, exit_code, _status):
        output = bytes(self._map_saver.readAllStandardOutput()).decode(
            'utf-8', errors='replace'
        )
        if exit_code != 0:
            self._save_failed(
                f'生成导航地图失败（退出码 {exit_code}）：\n{output}'
            )
            return
        try:
            navigation_prefix = self._sync_map_bundle_to_navigation(
                self._map_prefix, replace_existing=False
            )
        except Exception as error:
            self._save_failed(
                '建图文件已生成，但同步到导航地图库失败：\n'
                f'{error}\n\n建图文件保留在：{self._map_prefix}'
            )
            return
        if self._save_final:
            self._editable_map_prefix = Path(self._map_prefix)
            self._set_map_edit_available(True)
            text = (
                f'最终地图已保存：\n{self._map_prefix}\n'
                f'导航地图库副本：\n{navigation_prefix}\n'
                '当前轨迹已结束，可在左侧描绘/擦除障碍并覆盖本地图。'
            )
        else:
            text = (
                f'地图快照已保存：\n{self._map_prefix}\n'
                f'导航地图库副本：\n{navigation_prefix}\n可继续建图。'
            )
        self.save_status.setText(text)
        self._set_save_busy(False)
        QtWidgets.QMessageBox.information(self, '地图保存完成', text)

    def _save_failed(self, message):
        self.save_status.setText(message)
        self._set_save_busy(False)
        QtWidgets.QMessageBox.critical(self, '地图保存失败', message)

    def shutdown(self):
        self._shutting_down = True
        if self._alignment_worker is not None and self._alignment_worker.isRunning():
            self._alignment_worker.requestInterruption()
            self._alignment_worker.wait()
        self._key_diag.log('close')
        self._key_diag.close()
        self._key_filter.clear()
        self._release_timer.stop()
        self.node.stop_motion()
        self.spin_timer.stop()
        self.velocity_timer.stop()
        self.health_timer.stop()
        if self._map_saver is not None and self._map_saver.state() != QtCore.QProcess.NotRunning:
            self._map_saver.terminate()
            self._map_saver.waitForFinished(1000)
        self.executor.remove_node(self.node)
        self.executor.shutdown(timeout_sec=1.0)
        self.node.destroy_node()


def main(args=None):
    rclpy.init(args=args)
    application = QtWidgets.QApplication([sys.argv[0]])
    page = SlamPage()
    window = QtWidgets.QMainWindow()
    window.setWindowTitle('优化双雷达建图')
    window.resize(1450, 880)
    window.setCentralWidget(page)
    window.show()
    try:
        return application.exec_()
    finally:
        page.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
