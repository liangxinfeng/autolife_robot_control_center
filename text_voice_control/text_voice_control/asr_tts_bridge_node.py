import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .command_parser import parse_voice_command, strip_asr_prefix
from .topics import build_topic_name


class AsrTtsBridgeNode(Node):
    def __init__(self) -> None:
        super().__init__("asr_tts_bridge")

        self.declare_parameter("ros_domain_id", "")
        self.declare_parameter("robot_id", "300")
        self.declare_parameter("asr_topic", "")
        self.declare_parameter("tts_topic", "")
        self.declare_parameter("trigger_prefixes", ["播报", "说", "朗读", "请说"])
        self.declare_parameter("publish_json", True)

        robot_id = self.get_parameter("robot_id").value
        ros_domain_id = self.get_parameter("ros_domain_id").value
        asr_topic = self.get_parameter("asr_topic").value
        tts_topic = self.get_parameter("tts_topic").value
        self._prefixes = list(self.get_parameter("trigger_prefixes").value)
        self._publish_json = bool(self.get_parameter("publish_json").value)

        if not asr_topic:
            asr_topic = build_topic_name("topic_chat_text", ros_domain_id or None, robot_id)
        if not tts_topic:
            tts_topic = build_topic_name("topic_tts", ros_domain_id or None, robot_id)

        self._tts_pub = self.create_publisher(String, tts_topic, 10)
        self.create_subscription(String, asr_topic, self._on_asr_text, 10)

        self.get_logger().info(f"Listening ASR text on: {asr_topic}")
        self.get_logger().info(f"Publishing robot TTS commands to: {tts_topic}")
        self.get_logger().info(f"ASR trigger prefixes: {self._prefixes}")

    def _on_asr_text(self, msg: String) -> None:
        requested_text = strip_asr_prefix(msg.data, self._prefixes)
        if not requested_text:
            return

        command = parse_voice_command(requested_text)
        if command is None:
            return

        out = String()
        out.data = command.to_tts_payload(use_json=self._publish_json)
        self._tts_pub.publish(out)
        self.get_logger().info(f"ASR requested speech: {out.data}")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = AsrTtsBridgeNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
