#!/usr/bin/env python3
"""Speak a polite warning when an object stays close in front of the robot."""

import math
import time

import rclpy
from action_msgs.msg import GoalStatus, GoalStatusArray
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String


class ProximityVoiceAlertNode(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            'proximity_voice_alert',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('enabled', True)
        self.declare_parameter(
            'front_scan_topic', '/topic_gv_front_lidar_0_300'
        )
        self.declare_parameter(
            'navigation_status_topic', '/navigate_to_pose/_action/status'
        )
        self.declare_parameter('voice_command_topic', '/text_voice_command')
        self.declare_parameter(
            'alert_text', '前方有人或障碍物，请注意避让'
        )
        self.declare_parameter('trigger_distance_m', 0.80)
        self.declare_parameter('clear_distance_m', 0.95)
        self.declare_parameter('forward_angle_deg', 180.0)
        self.declare_parameter('half_angle_deg', 40.0)
        self.declare_parameter('minimum_points', 3)
        self.declare_parameter('trigger_consecutive_scans', 3)
        self.declare_parameter('clear_consecutive_scans', 3)
        self.declare_parameter('repeat_while_near_sec', 15.0)
        self.declare_parameter('startup_grace_sec', 4.0)

        self._enabled = bool(self.get_parameter('enabled').value)
        self._trigger_distance = max(
            0.10, float(self.get_parameter('trigger_distance_m').value)
        )
        self._clear_distance = max(
            self._trigger_distance + 0.05,
            float(self.get_parameter('clear_distance_m').value),
        )
        self._forward_angle = math.radians(float(
            self.get_parameter('forward_angle_deg').value
        ))
        self._half_angle = math.radians(max(
            5.0,
            min(90.0, float(self.get_parameter('half_angle_deg').value)),
        ))
        self._minimum_points = max(
            1, int(self.get_parameter('minimum_points').value)
        )
        self._trigger_scans = max(
            1, int(self.get_parameter('trigger_consecutive_scans').value)
        )
        self._clear_scans = max(
            1, int(self.get_parameter('clear_consecutive_scans').value)
        )
        self._repeat_period = max(
            0.0, float(self.get_parameter('repeat_while_near_sec').value)
        )
        self._started_at = time.monotonic()
        self._startup_grace = max(
            0.0, float(self.get_parameter('startup_grace_sec').value)
        )
        self._alert_text = str(self.get_parameter('alert_text').value).strip()
        self._near_samples = 0
        self._clear_samples = 0
        self._near_active = False
        self._last_alert_at = -1e9
        self._navigation_active = False

        laser_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self._voice_publisher = self.create_publisher(
            String,
            str(self.get_parameter('voice_command_topic').value),
            10,
        )
        self._scan_subscription = self.create_subscription(
            LaserScan,
            str(self.get_parameter('front_scan_topic').value),
            self._handle_scan,
            laser_qos,
        )
        self._navigation_status_subscription = self.create_subscription(
            GoalStatusArray,
            str(self.get_parameter('navigation_status_topic').value),
            self._handle_navigation_status,
            10,
        )
        self.get_logger().info(
            '前方避让语音提醒%s：距离 %.2fm，扫描中心 %.1f°，扇区 ±%.1f°，连续 %d 帧，重播 %.1fs'
            % (
                '已启用' if self._enabled else '已禁用',
                self._trigger_distance,
                math.degrees(self._forward_angle),
                math.degrees(self._half_angle),
                self._trigger_scans,
                self._repeat_period,
            )
        )

    @staticmethod
    def _normalized_angle(angle):
        return math.atan2(math.sin(angle), math.cos(angle))

    def _points_within(self, message, distance_limit):
        count = 0
        angle = float(message.angle_min)
        increment = float(message.angle_increment)
        for distance_value in message.ranges:
            distance = float(distance_value)
            if (
                math.isfinite(distance)
                and message.range_min <= distance <= message.range_max
                and distance <= distance_limit
                and abs(self._normalized_angle(
                    angle - self._forward_angle
                )) <= self._half_angle
            ):
                count += 1
                if count >= self._minimum_points:
                    return count
            angle += increment
        return count

    def _handle_scan(self, message):
        if (
            not self._enabled
            or not self._alert_text
            or not self._navigation_active
        ):
            self._reset_detection()
            return
        now = time.monotonic()
        if now - self._started_at < self._startup_grace:
            return

        if self._near_active:
            still_near = self._points_within(
                message, self._clear_distance
            ) >= self._minimum_points
            if still_near:
                self._clear_samples = 0
                if (
                    self._repeat_period > 0.0
                    and now - self._last_alert_at >= self._repeat_period
                ):
                    self._publish_alert(now)
            else:
                self._clear_samples += 1
                if self._clear_samples >= self._clear_scans:
                    self._near_active = False
                    self._near_samples = 0
                    self._clear_samples = 0
                    self.get_logger().debug('前方区域已恢复安全，提醒已重新待命')
            return

        is_near = self._points_within(
            message, self._trigger_distance
        ) >= self._minimum_points
        if not is_near:
            self._near_samples = 0
            return
        self._near_samples += 1
        if self._near_samples >= self._trigger_scans:
            self._near_active = True
            self._clear_samples = 0
            self._publish_alert(now)

    def _handle_navigation_status(self, message):
        active_states = {
            GoalStatus.STATUS_ACCEPTED,
            GoalStatus.STATUS_EXECUTING,
            GoalStatus.STATUS_CANCELING,
        }
        active = any(
            status.status in active_states for status in message.status_list
        )
        if self._navigation_active and not active:
            self._reset_detection()
            self.get_logger().debug('导航已结束，前方避让提醒暂停')
        elif active and not self._navigation_active:
            self.get_logger().debug('检测到活动导航目标，前方避让提醒开始监测')
        self._navigation_active = active

    def _reset_detection(self):
        self._near_samples = 0
        self._clear_samples = 0
        self._near_active = False

    def _publish_alert(self, now):
        message = String()
        message.data = self._alert_text
        self._voice_publisher.publish(message)
        self._last_alert_at = now
        self.get_logger().info(f'已发布前方避让语音提醒：{message.data}')


def main(args=None):
    rclpy.init(args=args)
    node = ProximityVoiceAlertNode()
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
