import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .command_parser import parse_voice_command
from .topics import build_topic_name


class TextVoiceControlNode(Node):
    def __init__(self) -> None:
        super().__init__("text_voice_control")

        self.declare_parameter("ros_domain_id", "")
        self.declare_parameter("robot_id", "307")
        self.declare_parameter("input_topic", "/text_voice_command")
        self.declare_parameter("tts_topic", "")
        self.declare_parameter("publish_json", True)

        robot_id = self.get_parameter("robot_id").value
        ros_domain_id = self.get_parameter("ros_domain_id").value
        tts_topic = self.get_parameter("tts_topic").value
        self._publish_json = bool(self.get_parameter("publish_json").value)

        if not tts_topic:
            tts_topic = build_topic_name("topic_tts", ros_domain_id or None, robot_id)

        input_topic = self.get_parameter("input_topic").value
        self._tts_pub = self.create_publisher(String, tts_topic, 10)
        self.create_subscription(String, input_topic, self._on_command, 10)

        self.get_logger().info(f"Listening text commands on: {input_topic}")
        self.get_logger().info(f"Publishing robot TTS commands to: {tts_topic}")

    def _on_command(self, msg: String) -> None:
        command = parse_voice_command(msg.data)
        if command is None:
            self.get_logger().warn("Ignored empty or invalid voice command.")
            return

        out = String()
        out.data = command.to_tts_payload(use_json=self._publish_json)
        self._tts_pub.publish(out)
        if command.status == "stop":
            self.get_logger().info("Forwarded TTS stop command.")
        else:
            self.get_logger().info(f"Forwarded TTS play command: {command.text}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = TextVoiceControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
