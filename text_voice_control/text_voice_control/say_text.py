import argparse
import time

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

from .command_parser import VoiceCommand
from .topics import build_topic_name


def main() -> int:
    parser = argparse.ArgumentParser(description="Send a text or stop command to robot TTS.")
    parser.add_argument("--text", "-t", default="", help="Text to speak.")
    parser.add_argument("--stop", action="store_true", help="Stop current TTS playback.")
    parser.add_argument("--domain", default=None, help="ROS_DOMAIN_ID, default uses environment or 0.")
    parser.add_argument("--robot", default=None, help="ROBOT_ID, default uses environment or 283.")
    parser.add_argument("--topic", default="", help="Override target TTS topic.")
    parser.add_argument("--plain", action="store_true", help="Publish plain text instead of JSON play payload.")
    args = parser.parse_args()

    if not args.stop and not args.text.strip():
        parser.error("--text is required unless --stop is used.")

    topic = args.topic or build_topic_name("topic_tts", args.domain, args.robot)
    command = VoiceCommand(status="stop") if args.stop else VoiceCommand(status="play", text=args.text)

    rclpy.init()
    node = Node("say_text")
    publisher = node.create_publisher(String, topic, 10)

    try:
        msg = String()
        msg.data = command.to_tts_payload(use_json=not args.plain)
        publisher.publish(msg)
        node.get_logger().info(f"Published to {topic}: {msg.data}")
        time.sleep(0.3)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
