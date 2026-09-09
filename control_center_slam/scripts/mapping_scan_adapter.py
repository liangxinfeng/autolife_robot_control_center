#!/usr/bin/env python3
"""Mapping-only adapter for the vendor's angle-resampled LaserScan snapshots.

328 emits 10 Hz frames but scan_time jumps between microseconds and ~50 ms.
Those fields cannot describe chronological ray acquisition. Do not invent
per-ray timing, or stamp old samples with now: keep header stamp/angles/ranges,
and treat each snapshot as simultaneous until hardware ray times are available.
"""
import copy
import math
from collections import deque
from functools import partial
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class StationaryScanDenoiser:
    """Three-frame median ONLY at verified rest; preserve edges and invalids."""
    def __init__(self):
        self.history = deque(maxlen=3)
        self.geometry = None
        self.last_stamp = None

    def filter(self, message, stationary):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        geometry = (message.header.frame_id, len(message.ranges),
                    message.angle_min, message.angle_increment)
        if (not stationary or geometry != self.geometry or self.last_stamp is None
                or not 0 < stamp - self.last_stamp < .18):
            self.history.clear()
        self.geometry, self.last_stamp = geometry, stamp
        if not stationary:
            return message
        self.history.append(np.asarray(message.ranges, dtype=np.float32).copy())
        if len(self.history) < 3:
            return message
        values = np.stack(self.history)
        valid = np.all(np.isfinite(values) & (values >= message.range_min)
                       & (values <= message.range_max), axis=0)
        indices = np.flatnonzero(valid)
        # Never average across a new object/depth discontinuity. A new close
        # obstacle and all NaN/Inf samples remain exactly as currently observed.
        eligible = indices[np.ptp(values[:, indices], axis=0) <= .08]
        result = values[-1].copy()
        result[eligible] = np.median(values[:, eligible], axis=0)
        message.ranges = result.tolist()
        return message


class SnapshotGate:
    def __init__(self, max_age_sec=0.5):
        self.max_age_sec = max_age_sec
        self.last_stamp_ns = None

    def accept(self, message, now_ns):
        stamp_ns = message.header.stamp.sec * 1000000000 + message.header.stamp.nanosec
        age = (now_ns - stamp_ns) * 1e-9
        if stamp_ns <= 0 or age > self.max_age_sec or age < -0.05:
            return None, 'stale or future timestamp'
        if self.last_stamp_ns is not None and stamp_ns <= self.last_stamp_ns:
            return None, 'duplicate or out-of-order timestamp'
        if (len(message.ranges) < 2 or not message.header.frame_id
                or not math.isfinite(message.angle_increment)
                or message.angle_increment <= 0
                or not math.isfinite(message.angle_min)
                or not math.isfinite(message.angle_max)):
            return None, 'invalid scan geometry'
        result = copy.deepcopy(message)
        # scan_time remains a diagnostic frame period, never ray timing.
        result.scan_time = 0.1 if self.last_stamp_ns is None else min(
            0.5, (stamp_ns - self.last_stamp_ns) * 1e-9)
        result.time_increment = 0.0
        self.last_stamp_ns = stamp_ns
        return result, ''


class MappingScanAdapter(Node):
    def __init__(self):
        super().__init__('mapping_scan_adapter')
        self.declare_parameter('front_scan_topic', '/topic_gv_front_lidar_0_307')
        self.declare_parameter('rear_scan_topic', '/topic_gv_rear_lidar_0_307')
        self.declare_parameter('max_age_sec', 0.5)
        self.declare_parameter('odom_topic', '/topic_gv_wheel_odom_0_307')
        self.declare_parameter('stationary_denoise_enabled', True)
        self.denoisers = {side: StationaryScanDenoiser() for side in ('front', 'rear')}
        self.last_odom = None
        self.stationary_since = None
        self.gates = {}
        self.publishers_by_side = {}
        self.counts = {side: [0, 0] for side in ('front', 'rear')}
        self.last_log_ns = 0
        sensor_qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Odometry, str(self.get_parameter('odom_topic').value),
                                 self.receive_odometry, sensor_qos)
        for side in self.counts:
            self.gates[side] = SnapshotGate(float(self.get_parameter('max_age_sec').value))
            self.publishers_by_side[side] = self.create_publisher(
                LaserScan, '/mapping_scan/' + side, 1)
            self.create_subscription(LaserScan,
                str(self.get_parameter(side + '_scan_topic').value),
                partial(self.receive, side), sensor_qos)
        self.get_logger().info(
            'Mapping scans use original snapshot stamps, time_increment=0; '
            'raw navigation scans are unchanged. True per-ray deskew needs driver timestamps.')

    def receive_odometry(self, message):
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        p, q = message.pose.pose.position, message.pose.pose.orientation
        yaw = math.atan2(2*(q.w*q.z+q.x*q.y), 1-2*(q.y*q.y+q.z*q.z))
        current = (stamp, p.x, p.y, yaw)
        stable = False
        if self.last_odom and all(math.isfinite(x) for x in current):
            old = self.last_odom
            dt = stamp - old[0]
            angle = math.atan2(math.sin(yaw-old[3]), math.cos(yaw-old[3]))
            stable = (0 < dt <= .2 and math.hypot(p.x-old[1], p.y-old[2]) <= .005*dt
                      and abs(angle) <= .005*dt)
        if not stable:
            self.stationary_since = None
        elif self.stationary_since is None:
            self.stationary_since = stamp
        self.last_odom = current

    def receive(self, side, message):
        now_ns = self.get_clock().now().nanoseconds
        result, reason = self.gates[side].accept(message, now_ns)
        if result is None:
            self.counts[side][1] += 1
            if now_ns - self.last_log_ns > 5_000_000_000:
                self.get_logger().warning(f'{side} scan rejected: {reason}; counts={self.counts}')
                self.last_log_ns = now_ns
            return
        self.counts[side][0] += 1
        now = now_ns * 1e-9
        stationary = (bool(self.get_parameter('stationary_denoise_enabled').value)
                      and self.stationary_since is not None and self.last_odom is not None
                      and 0 <= now-self.last_odom[0] <= .2
                      and self.last_odom[0]-self.stationary_since >= .3)
        result = self.denoisers[side].filter(result, stationary)
        self.publishers_by_side[side].publish(result)


def main():
    rclpy.init()
    node = MappingScanAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
