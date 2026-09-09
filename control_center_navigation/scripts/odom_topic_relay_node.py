#!/usr/bin/env python3
from nav_msgs.msg import Odometry
import rclpy
from rclpy.node import Node


class OdomTopicRelayNode(Node):
    def __init__(self) -> None:
        super().__init__('odom_topic_relay')

        self.declare_parameter('input_topic', '/topic_gv_wheel_odom_0_307')
        self.declare_parameter('output_topic', '/odom')

        input_topic = self.get_parameter('input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value

        self._publisher = self.create_publisher(Odometry, output_topic, 10)
        self._subscription = self.create_subscription(
            Odometry,
            input_topic,
            self._handle_odom,
            10,
        )

        self.get_logger().info(f'Relaying odometry from {input_topic} to {output_topic}')

    def _handle_odom(self, msg: Odometry) -> None:
        self._publisher.publish(msg)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = OdomTopicRelayNode()
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
