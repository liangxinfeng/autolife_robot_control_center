import os


def build_topic_name(prefix: str, ros_domain_id: str | None = None, robot_id: str | None = None) -> str:
    domain = str(ros_domain_id if ros_domain_id is not None else os.getenv("ROS_DOMAIN_ID", "0"))
    robot = str(robot_id if robot_id is not None else os.getenv("ROBOT_ID", "307"))
    return f"/{prefix}_{domain}_{robot}"
