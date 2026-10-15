from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    robot_id = LaunchConfiguration('robot_id')
    topic_suffix = LaunchConfiguration('topic_suffix')
    start_navigation = LaunchConfiguration('start_navigation')
    vendor_uri = (
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="lo"/>'
        '</Interfaces></General><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex>'
        '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>'
    )
    return LaunchDescription([
        DeclareLaunchArgument('robot_id', default_value='300'),
        DeclareLaunchArgument('topic_suffix', default_value='0_300'),
        DeclareLaunchArgument('start_navigation', default_value='false'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('CYCLONEDDS_URI', vendor_uri),
        Node(
            package='control_center_shell',
            executable='control_center',
            output='screen',
            parameters=[{
                'robot_id': ParameterValue(robot_id, value_type=int),
                'topic_suffix': ParameterValue(topic_suffix, value_type=str),
                'start_navigation': ParameterValue(
                    start_navigation, value_type=bool
                ),
            }],
        ),
    ])
