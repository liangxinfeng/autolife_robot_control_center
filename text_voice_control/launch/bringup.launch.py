from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_id = LaunchConfiguration("robot_id")
    ros_domain_id = LaunchConfiguration("ros_domain_id")
    enable_asr_bridge = LaunchConfiguration("enable_asr_bridge")

    text_node = Node(
        package="text_voice_control",
        executable="text_voice_control_node",
        name="text_voice_control",
        output="screen",
        parameters=[{
            "robot_id": ParameterValue(robot_id, value_type=str),
            "ros_domain_id": ParameterValue(ros_domain_id, value_type=str),
        }],
    )

    asr_bridge = Node(
        package="text_voice_control",
        executable="asr_tts_bridge_node",
        name="asr_tts_bridge",
        output="screen",
        parameters=[{
            "robot_id": ParameterValue(robot_id, value_type=str),
            "ros_domain_id": ParameterValue(ros_domain_id, value_type=str),
        }],
        condition=IfCondition(enable_asr_bridge),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_id", default_value="283"),
        DeclareLaunchArgument("ros_domain_id", default_value="0"),
        DeclareLaunchArgument("enable_asr_bridge", default_value="false"),
        text_node,
        asr_bridge,
    ])
