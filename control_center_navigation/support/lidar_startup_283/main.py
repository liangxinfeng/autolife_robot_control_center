import json
from autolife_robot_gv.gv_sensor_service import gv_sensor_service
from autolife_robot_gv import GV_CONFIG_PATH
import os


def main():
    """Main function to run GV sensor service"""
    from autolife_robot_gv.lidar_startup import install
    install()
    print("Starting GV sensor service...")

    ros_domain_id = os.getenv("ROS_DOMAIN_ID", "0")
    robot_id = os.getenv("ROBOT_ID", "0")
    TOPIC_NODE_ID = f"{ros_domain_id}_{robot_id}"
    print(f"TOPIC_NODE_ID: {TOPIC_NODE_ID}")

    # load config.json
    config = None
    with open(GV_CONFIG_PATH) as f:
        config = json.load(f)
    
    gv_sensor_service(TOPIC_NODE_ID, config)


if __name__ == "__main__":
    main()
