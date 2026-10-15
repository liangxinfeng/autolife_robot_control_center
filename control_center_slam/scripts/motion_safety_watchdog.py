#!/usr/bin/env python3
"""Gate mapping teleop output on the control-center UI heartbeat."""

import math
import time

from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


class MotionSafetyWatchdog(Node):
    def __init__(self):
        super().__init__('control_center_slam_motion_watchdog')
        self.declare_parameter(
            'input_topic', '/control_center/chassis/slam_input'
        )
        self.declare_parameter(
            'output_topic', '/topic_gv_target_cmd_vel_0_300'
        )
        self.declare_parameter(
            'heartbeat_topic', '/control_center/operator_heartbeat'
        )
        self.declare_parameter('heartbeat_timeout_sec', 0.30)
        self.declare_parameter('stop_publish_hz', 20.0)
        self.declare_parameter('external_manual_topic', '/manual_cmd_vel')
        self.declare_parameter('external_manual_timeout_sec', 0.30)
        self._heartbeat_timeout = max(
            0.10, float(self.get_parameter('heartbeat_timeout_sec').value)
        )
        self._external_timeout = max(
            0.10, float(self.get_parameter('external_manual_timeout_sec').value)
        )
        self._publisher = self.create_publisher(
            Twist, str(self.get_parameter('output_topic').value), 10
        )
        self.create_subscription(
            Twist, str(self.get_parameter('input_topic').value),
            self._on_command, 10,
        )
        self.create_subscription(
            String, str(self.get_parameter('heartbeat_topic').value),
            self._on_heartbeat, 10,
        )
        self.create_subscription(
            Twist, str(self.get_parameter('external_manual_topic').value),
            self._on_external_manual, 10,
        )
        stop_hz = max(2.0, float(self.get_parameter('stop_publish_hz').value))
        self.create_timer(1.0 / stop_hz, self._tick)
        self._session = ''
        self._last_heartbeat = None
        self._last_external_manual = None
        self._last_control_center_command = None
        self._last_command_nonzero = False
        self._lost_latched = False
        self.get_logger().info(
            'Mapping teleop watchdog armed; heartbeat timeout=%.2fs'
            % self._heartbeat_timeout
        )

    @staticmethod
    def _zero(message):
        values = (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )
        return all(math.isfinite(float(value)) for value in values) and all(
            abs(float(value)) <= 1e-7 for value in values
        )

    def _on_heartbeat(self, message):
        session = str(message.data).strip()
        if not session:
            return
        if not self._session:
            self._session = session
            self.get_logger().info('Mapping teleop session is active')
        elif session != self._session:
            self.get_logger().error(
                'Different control-center session detected; this mapping '
                'watchdog remains locked to its original owner'
            )
            self._trip()
            return
        self._last_heartbeat = time.monotonic()

    def _on_external_manual(self, _message):
        self._last_external_manual = time.monotonic()

    def _on_command(self, message):
        now = time.monotonic()
        self._last_control_center_command = now
        if self._last_heartbeat is None:
            return
        if now - self._last_heartbeat > self._heartbeat_timeout:
            self._trip()
            return
        if self._lost_latched:
            # Require a zero command before allowing the recovered session to
            # send motion again.
            if self._zero(message):
                self._lost_latched = False
            else:
                return
        if not all(math.isfinite(float(value)) for value in (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        )):
            self._publisher.publish(Twist())
            self._last_command_nonzero = False
            self.get_logger().error('Rejected non-finite mapping velocity')
            return
        self._publisher.publish(message)
        self._last_command_nonzero = not self._zero(message)

    def _tick(self):
        if self._last_heartbeat is None or self._lost_latched:
            return
        if time.monotonic() - self._last_heartbeat > self._heartbeat_timeout:
            self._trip()

    def _trip(self):
        if self._lost_latched:
            return
        self._lost_latched = True
        external_active = (
            self._last_external_manual is not None
            and time.monotonic() - self._last_external_manual
            <= self._external_timeout
        )
        if self._last_command_nonzero and not external_active:
            # This is a single ownership handoff stop.  No periodic zeros are
            # sent after it, so another control source can take over.
            self._publisher.publish(Twist())
            self.get_logger().error(
                'Control-center heartbeat lost; sent one stop and became silent'
            )
        else:
            self.get_logger().error(
                'Control-center heartbeat lost; blocked mapping commands'
            )
        self._last_command_nonzero = False


def main(args=None):
    rclpy.init(args=args)
    node = MotionSafetyWatchdog()
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
