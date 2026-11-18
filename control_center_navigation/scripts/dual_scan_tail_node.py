#!/usr/bin/env python3
"""Navigation-only dual-scan consistency gate; original scan origins stay intact."""
import copy
import math
from collections import deque
import numpy as np
import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener, TransformException
from dual_scan_tail_core import cross_checked_tail_indices


def rotation_matrix(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    norm = math.sqrt(x*x + y*y + z*z + w*w)
    if not math.isfinite(norm) or abs(norm-1) > .01:
        raise ValueError('invalid rotation')
    return np.array([[1-2*(y*y+z*z), 2*(x*y-z*w), 2*(x*z+y*w)],
                     [2*(x*y+z*w), 1-2*(x*x+z*z), 2*(y*z-x*w)],
                     [2*(x*z-y*w), 2*(y*z+x*w), 1-2*(x*x+y*y)]])


class DualScanTailGate(Node):
    def __init__(self):
        super().__init__('navigation_dual_scan_tail_gate')
        self.declare_parameter('enabled', True)
        self.declare_parameter('fixed_frame', 'odom')
        self.declare_parameter('input_suffix', '_scan_before_crosscheck')
        self.declare_parameter('output_suffix', '_scan')
        self.history = {side: deque(maxlen=8) for side in ('front', 'rear')}
        self.done = {side: -1 for side in self.history}
        self.pubs, self.subs = {}, []
        self.tf = Buffer(cache_time=Duration(seconds=3))
        self.listener = TransformListener(self.tf, self)
        for side in self.history:
            inp = '/navigation/' + side + self.get_parameter('input_suffix').value
            out = '/navigation/' + side + self.get_parameter('output_suffix').value
            self.pubs[side] = self.create_publisher(LaserScan, out, qos_profile_sensor_data)
            self.subs.append(self.create_subscription(
                LaserScan, inp, lambda msg, side=side: self.receive(side, msg),
                qos_profile_sensor_data))
        self.timer = self.create_timer(.02, self.tick)
        self.get_logger().info('Dual-scan near-tail gate ready; missing evidence passes original returns')

    @staticmethod
    def stamp(msg):
        return msg.header.stamp.sec * 1000000000 + msg.header.stamp.nanosec

    def receive(self, side, msg):
        stamp = self.stamp(msg)
        if self.history[side] and stamp < self.stamp(self.history[side][-1]):
            self.history[side].clear()
            self.done[side] = -1
        if not self.history[side] or stamp != self.stamp(self.history[side][-1]):
            self.history[side].append(msg)

    def tick(self):
        now = self.get_clock().now().nanoseconds
        for side in self.history:
            pending = [m for m in self.history[side]
                       if self.stamp(m) > self.done[side] and now-self.stamp(m) >= 60000000]
            if not pending:
                continue
            scan = pending[-1]
            stamp = self.stamp(scan)
            self.done[side] = stamp
            # Never refresh old data with a new timestamp. Stale input ceases
            # publishing and Nav2's existing source watchdog remains effective.
            if not 0 <= now-stamp <= 250000000:
                continue
            result = scan
            other_side = 'rear' if side == 'front' else 'front'
            others = self.history[other_side]
            if self.get_parameter('enabled').value and others:
                other = min(others, key=lambda m: abs(self.stamp(m)-stamp))
                if (abs(self.stamp(other)-stamp) <= 30000000 and
                        0 <= now-self.stamp(other) <= 250000000):
                    try:
                        # Different acquisition times are compensated through
                        # odom; static frame transforms alone are insufficient
                        # while the chassis is moving.
                        tr = self.tf.lookup_transform_full(
                            other.header.frame_id, Time.from_msg(other.header.stamp),
                            scan.header.frame_id, Time.from_msg(scan.header.stamp),
                            self.get_parameter('fixed_frame').value)
                        q, t = tr.transform.rotation, tr.transform.translation
                        ids = cross_checked_tail_indices(
                            scan.ranges, scan.intensities, scan.angle_min, scan.angle_increment,
                            other.ranges, other.intensities, other.angle_min, other.angle_increment,
                            rotation_matrix(q), np.array([t.x, t.y, t.z]))
                        if ids:
                            result = copy.deepcopy(scan)
                            for i in ids:
                                result.ranges[i] = float('nan')
                            self.get_logger().info(
                                f'{side}: removed {len(ids)} cross-checked near-tail returns',
                                throttle_duration_sec=3.)
                    except (TransformException, ValueError, IndexError):
                        # Keep original obstacle evidence on any missing input
                        # or transform; do not manufacture infinity clearing.
                        pass
            self.pubs[side].publish(result)


def main():
    rclpy.init()
    node = DualScanTailGate()
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
