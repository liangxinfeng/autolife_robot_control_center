#!/usr/bin/env python3
import json
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import String
from visualization_msgs.msg import Marker


class NavigationStatusOverlayNode(Node):
    def __init__(self) -> None:
        super().__init__('navigation_status_overlay')

        self.declare_parameter('status_topic', '/waypoints/navigation_status')
        self.declare_parameter('marker_topic', '/waypoints/navigation_status_marker')
        self.declare_parameter('frame_id', 'map')
        self.declare_parameter('x', -4.0)
        self.declare_parameter('y', 4.0)
        self.declare_parameter('z', 1.2)
        self.declare_parameter('text_scale', 0.35)

        self._status_topic = self.get_parameter('status_topic').value
        self._marker_topic = self.get_parameter('marker_topic').value
        self._frame_id = self.get_parameter('frame_id').value
        self._x = float(self.get_parameter('x').value)
        self._y = float(self.get_parameter('y').value)
        self._z = float(self.get_parameter('z').value)
        self._text_scale = float(self.get_parameter('text_scale').value)

        qos = QoSProfile(depth=10)
        qos.reliability = ReliabilityPolicy.RELIABLE
        qos.durability = DurabilityPolicy.TRANSIENT_LOCAL

        self._pub = self.create_publisher(Marker, self._marker_topic, qos)
        self.create_subscription(String, self._status_topic, self._handle_status, qos)

        self._publish_text('导航状态: 等待任务', 'idle')
        self.get_logger().info(
            f'Navigation status overlay: {self._status_topic} -> {self._marker_topic}'
        )

    def _handle_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            self._publish_text(f'导航状态: {msg.data}', 'unknown')
            return

        state = str(payload.get('state', 'unknown'))
        active_waypoint = payload.get('active_waypoint') or '-'
        sequence_index = int(payload.get('sequence_index', -1))
        sequence_total = int(payload.get('sequence_total', 0))
        distance = self._format_number(payload.get('distance_remaining'), 'm')
        eta = self._format_number(payload.get('estimated_time_remaining'), 's')
        nav_time = self._format_number(payload.get('navigation_time'), 's')
        message = payload.get('message') or ''

        sequence_text = '-'
        if sequence_total > 0 and sequence_index >= 0:
            sequence_text = f'{sequence_index + 1}/{sequence_total}'

        lines = [
            f'导航状态: {self._state_label(state)}',
            f'当前航点: {active_waypoint}',
            f'序列进度: {sequence_text}',
            f'剩余距离: {distance}',
            f'预计剩余: {eta}',
            f'已用时间: {nav_time}',
        ]
        if message:
            lines.append(f'消息: {message}')

        self._publish_text('\n'.join(lines), state)

    def _publish_text(self, text: str, state: str) -> None:
        marker = Marker()
        marker.header.frame_id = self._frame_id
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'navigation_status_overlay'
        marker.id = 0
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD
        marker.pose.position.x = self._x
        marker.pose.position.y = self._y
        marker.pose.position.z = self._z
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._text_scale
        marker.color.r, marker.color.g, marker.color.b = self._state_color(state)
        marker.color.a = 1.0
        marker.text = text
        self._pub.publish(marker)

    @staticmethod
    def _format_number(value, unit: str) -> str:
        if value is None:
            return '-'
        try:
            return f'{float(value):.1f} {unit}'
        except (TypeError, ValueError):
            return '-'

    @staticmethod
    def _state_label(state: str) -> str:
        labels = {
            'idle': '空闲',
            'started': '已发送',
            'accepted': '已接收',
            'running': '导航中',
            'succeeded': '已到达',
            'succeeded_by_tolerance': '已到达',
            'advancing': '切换航点',
            'failed': '失败',
            'rejected': '被拒绝',
            'sequence_failed': '序列失败',
            'sequence_succeeded': '序列完成',
            'paused': '已暂停',
        }
        return labels.get(state, state)

    @staticmethod
    def _state_color(state: str):
        if state in ('succeeded', 'succeeded_by_tolerance', 'sequence_succeeded'):
            return 0.1, 1.0, 0.35
        if state in ('failed', 'rejected', 'sequence_failed'):
            return 1.0, 0.2, 0.15
        if state == 'running':
            return 0.2, 0.75, 1.0
        if state in ('started', 'accepted', 'advancing'):
            return 1.0, 0.85, 0.2
        if state == 'paused':
            return 1.0, 0.55, 0.15
        return 1.0, 1.0, 1.0


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationStatusOverlayNode()
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
