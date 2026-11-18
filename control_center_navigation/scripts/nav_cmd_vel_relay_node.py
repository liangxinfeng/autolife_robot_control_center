#!/usr/bin/env python3
from geometry_msgs.msg import Twist
import rclpy
from rclpy.node import Node


class NavCmdVelRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('nav_cmd_vel_relay')

        self.declare_parameter('input_topic', '/nav_cmd_vel')
        self.declare_parameter('output_topic', '/arm_cmd_vel_0_283')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._subscription = self.create_subscription(
            Twist,
            input_topic,
            self._handle_cmd_vel,
            10,
        )

        self.get_logger().info(f'Relaying navigation velocity from {input_topic} to {output_topic}')

    def _handle_cmd_vel(self, msg: Twist) -> None:
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavCmdVelRelayNode()
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
