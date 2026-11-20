from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from control_center_module_lease import controller_lease
from launch.actions import (
    DeclareLaunchArgument,
    EmitEvent,
    RegisterEventHandler,
    SetEnvironmentVariable,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    share = Path(get_package_share_directory('control_center_joint_control'))
    urdf_path = share / 'urdf' / 'robot_v3_1.urdf'
    robot_description = urdf_path.read_text().replace(
        '../meshes/robot_v3_1/',
        'package://control_center_joint_control/meshes/robot_v3_1/',
    )

    robot_id = LaunchConfiguration('robot_id')
    topic_suffix = LaunchConfiguration('topic_suffix')
    start_panel = LaunchConfiguration('panel')
    require_operator_heartbeat = LaunchConfiguration(
        'require_operator_heartbeat'
    )

    bridge_node = Node(
        package='control_center_joint_control',
        executable='joint_bridge',
        name='whole_body_joint_bridge',
        output='screen',
        parameters=[{
            'robot_id': ParameterValue(robot_id, value_type=int),
            # Values such as 0_367 otherwise look numeric to the launch
            # YAML parameter parser (and can become octal integer 198).
            'topic_suffix': ParameterValue(topic_suffix, value_type=str),
            'require_operator_heartbeat': ParameterValue(
                require_operator_heartbeat, value_type=bool
            ),
        }],
    )

    return LaunchDescription([
        controller_lease('joint'),
        # The vendor arm-control service uses CycloneDDS on loopback.  Set this
        # explicitly for every process started here so GUI launches and shells
        # with a stale Fast DDS daemon cannot silently join a different graph.
        SetEnvironmentVariable(
            'RMW_IMPLEMENTATION',
            'rmw_cyclonedds_cpp',
        ),
        SetEnvironmentVariable(
            'CYCLONEDDS_URI',
            '<CycloneDDS><Domain><General><Interfaces>'
            '<NetworkInterface name="lo"/>'
            '</Interfaces></General><Discovery>'
            '<ParticipantIndex>auto</ParticipantIndex>'
            '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
            '</Discovery></Domain></CycloneDDS>',
        ),
        DeclareLaunchArgument(
            'robot_id', default_value='367',
            description='Robot number used by vendor topic suffixes.',
        ),
        DeclareLaunchArgument(
            'topic_suffix', default_value='',
            description='Explicit vendor suffix, e.g. 0_367. Empty uses ROS_DOMAIN_ID_robot_id.',
        ),
        DeclareLaunchArgument(
            'panel', default_value='true',
            description='Open the joint monitor/control panel with embedded 3D view.',
        ),
        DeclareLaunchArgument(
            'require_operator_heartbeat', default_value='false',
            description=(
                'Lock joint output when the embedding control-center UI heartbeat expires.'
            ),
        ),
        bridge_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=bridge_node,
                on_exit=[
                    EmitEvent(event=Shutdown(
                        reason='Joint bridge exited; stopping this launch to avoid duplicate TF/state publishers.'
                    ))
                ],
            )
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='whole_body_actual_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'publish_frequency': 30.0,
            }],
            remappings=[
                ('joint_states', '/control_center_joint_control/joint_states'),
            ],
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='whole_body_target_root_tf',
            output='screen',
            arguments=[
                '--x', '0', '--y', '0', '--z', '0',
                '--roll', '0', '--pitch', '0', '--yaw', '0',
                '--frame-id', 'Link_Ground_Vehicle_X',
                '--child-frame-id', 'joint_target/Link_Ground_Vehicle_X',
            ],
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='whole_body_target_state_publisher',
            output='screen',
            parameters=[{
                'robot_description': robot_description,
                'frame_prefix': 'joint_target/',
                'publish_frequency': 30.0,
            }],
            remappings=[
                ('joint_states', '/control_center_joint_control/target_joint_states'),
                ('robot_description', '/control_center_joint_control/target_robot_description'),
            ],
        ),
        Node(
            package='control_center_joint_control',
            executable='joint_control_panel',
            name='control_center_joint_control_panel',
            output='screen',
            condition=IfCondition(start_panel),
        ),
    ])
