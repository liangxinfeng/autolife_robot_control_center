#!/usr/bin/env python3
"""Publish one vendor full-body reset command after subscriber discovery."""

import re
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String


SAFE_SUFFIX_RE = re.compile(r'^[A-Za-z0-9_]+$')


class JointResetOnce(Node):
    def __init__(self, parameter_overrides=None):
        super().__init__(
            'control_center_joint_reset_once',
            parameter_overrides=parameter_overrides,
        )
        self.declare_parameter('topic_suffix', '0_300')
        self.declare_parameter('discovery_timeout_sec', 5.0)
        suffix = str(self.get_parameter('topic_suffix').value).strip()
        if not SAFE_SUFFIX_RE.fullmatch(suffix):
            raise ValueError(f'无效话题后缀：{suffix!r}')
        self.topic = f'/control_reset_{suffix}'
        self.discovery_timeout = max(
            0.5,
            min(
                15.0,
                float(self.get_parameter('discovery_timeout_sec').value),
            ),
        )
        self.publisher = self.create_publisher(String, self.topic, 10)

    def publish_once(self):
        deadline = time.monotonic() + self.discovery_timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self.publisher.get_subscription_count() > 0:
                break
            rclpy.spin_once(self, timeout_sec=0.05)
        if self.publisher.get_subscription_count() == 0:
            self.get_logger().error(
                f'{self.topic} 没有厂家订阅者；请确认机械臂服务、编号和 DDS 配置'
            )
            return False
        self.publisher.publish(String(data=''))
        # Give DDS enough time to deliver the one-shot volatile message.
        delivery_deadline = time.monotonic() + 0.6
        while rclpy.ok() and time.monotonic() < delivery_deadline:
            rclpy.spin_once(self, timeout_sec=0.05)
        self.get_logger().info(
            f'厂家全身默认复位命令已发送：{self.topic}'
        )
        return True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = JointResetOnce()
        return 0 if node.publish_once() else 2
    except Exception as error:
        print(f'全身关节复位命令失败：{error}')
        return 2
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
