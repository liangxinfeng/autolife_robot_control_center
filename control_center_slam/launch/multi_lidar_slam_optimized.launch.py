import os
import importlib.util

from ament_index_python.packages import get_package_share_directory, get_package_prefix
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable, TimerAction, OpaqueFunction, SetLaunchConfiguration
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


PACKAGE_NAME = 'control_center_slam'


def _validate_resume(context):
    path = LaunchConfiguration('load_state_filename').perform(context)
    if path and (not os.path.isfile(path) or not path.lower().endswith('.pbstream')):
        raise RuntimeError('增量建图需要存在的 .pbstream 状态文件；不能仅导入地图图片。')
    if path:
        helper_path = os.path.join(get_package_prefix(PACKAGE_NAME), 'lib', PACKAGE_NAME, 'incremental_mapping.py')
        spec = importlib.util.spec_from_file_location('_slam_state_validation', helper_path)
        helper = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(helper)
        helper.resolve_state_file(path)
        return [SetLaunchConfiguration('start_new_trajectory', 'false'),
                SetLaunchConfiguration('cartographer_configuration_basename', 'multi_lidar_incremental.lua')]
    return []


def generate_launch_description():
    package_share = get_package_share_directory(PACKAGE_NAME)
    config_dir = os.path.join(package_share, 'config')

    use_sim_time = LaunchConfiguration('use_sim_time')
    open_rviz = LaunchConfiguration('open_rviz')
    publish_lidar_static_tf = LaunchConfiguration('publish_lidar_static_tf')
    enable_keyboard_control = LaunchConfiguration('enable_keyboard_control')
    keyboard_cmd_vel_topic = LaunchConfiguration('keyboard_cmd_vel_topic')
    keyboard_linear_speed = LaunchConfiguration('keyboard_linear_speed')
    keyboard_angular_speed = LaunchConfiguration('keyboard_angular_speed')
    front_scan_topic = LaunchConfiguration('front_scan_topic')
    rear_scan_topic = LaunchConfiguration('rear_scan_topic')
    odom_topic = LaunchConfiguration('odom_topic')

    cyclone_uri = (
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="lo"/>'
        '</Interfaces></General><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex>'
        '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>'
    )

    front_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='optimized_slam_static_tf_front',
        condition=IfCondition(publish_lidar_static_tf),
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'front_laser_frame',
            '--x', '0.215', '--y', '0.0', '--z', '0.1515',
            '--roll', '3.145926', '--pitch', '0.0', '--yaw', '3.145926',
        ],
    )
    rear_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='optimized_slam_static_tf_rear',
        condition=IfCondition(publish_lidar_static_tf),
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'rear_laser_frame',
            '--x', '-0.215', '--y', '0.0', '--z', '0.1515',
            '--roll', '3.145926', '--pitch', '0.0', '--yaw', '0.0',
        ],
    )

    scan_adapter = Node(
        package=PACKAGE_NAME,
        executable='mapping_scan_adapter.py',
        output='screen',
        parameters=[{
            'use_sim_time': ParameterValue(use_sim_time, value_type=bool),
            'front_scan_topic': front_scan_topic,
            'rear_scan_topic': rear_scan_topic,
            'odom_topic': odom_topic,
        }],
    )

    cartographer_node = Node(
        package=LaunchConfiguration('cartographer_package'),
        executable=LaunchConfiguration('cartographer_executable'),
        name='cartographer_node_optimized',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-configuration_directory', config_dir,
            '-configuration_basename', LaunchConfiguration('cartographer_configuration_basename'),
            ['-start_trajectory_with_default_topics=', LaunchConfiguration('start_new_trajectory')],
            ['-load_state_filename=', LaunchConfiguration('load_state_filename')],
            '-load_frozen_state=false',
        ],
        remappings=[
            ('scan_1', '/mapping_scan/front'),
            ('scan_2', '/mapping_scan/rear'),
            ('odom', odom_topic),
        ],
    )

    occupancy_grid_node = Node(
        package=PACKAGE_NAME,
        executable='cartographer_occupancy_grid_node',
        name='cartographer_occupancy_grid_node_optimized',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
        arguments=[
            '-resolution', LaunchConfiguration('map_resolution'),
            '-publish_period_sec', '0.2',
        ],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_slam_optimized',
        condition=IfCondition(open_rviz),
        output='screen',
        arguments=['-d', os.path.join(package_share, 'rviz', 'slam.rviz')],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    keyboard_teleop_node = Node(
        package=PACKAGE_NAME,
        executable='mapping_keyboard_teleop.py',
        name='mapping_keyboard_teleop',
        condition=IfCondition(enable_keyboard_control),
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'output_topic': keyboard_cmd_vel_topic,
                'linear_speed': ParameterValue(
                    keyboard_linear_speed,
                    value_type=float,
                ),
                'angular_speed': ParameterValue(
                    keyboard_angular_speed,
                    value_type=float,
                ),
            }
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument('load_state_filename', default_value=''),
        DeclareLaunchArgument('start_new_trajectory', default_value='true'),
        DeclareLaunchArgument('cartographer_package', default_value=PACKAGE_NAME),
        DeclareLaunchArgument('cartographer_executable',
                              default_value='cartographer_multi_origin_node'),
        DeclareLaunchArgument('cartographer_configuration_basename',
                              default_value='multi_lidar_quality.lua'),
        DeclareLaunchArgument('map_resolution', default_value='0.02'),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('open_rviz', default_value='true'),
        DeclareLaunchArgument(
            'enable_keyboard_control',
            default_value='true',
            description='Start low-speed mapping keyboard teleoperation.',
        ),
        DeclareLaunchArgument(
            'keyboard_cmd_vel_topic',
            default_value='/topic_gv_target_cmd_vel_0_283',
        ),
        DeclareLaunchArgument('keyboard_linear_speed', default_value='0.18'),
        DeclareLaunchArgument('keyboard_angular_speed', default_value='0.45'),
        DeclareLaunchArgument(
            'publish_lidar_static_tf',
            default_value='true',
            description=(
                'Publish the calibrated lidar TFs required for standalone '
                'SLAM. Set false only when another active process publishes '
                'the same child frames.'
            ),
        ),
        DeclareLaunchArgument(
            'front_scan_topic',
            default_value='/topic_gv_front_lidar_0_283',
        ),
        DeclareLaunchArgument(
            'rear_scan_topic',
            default_value='/topic_gv_rear_lidar_0_283',
        ),
        DeclareLaunchArgument(
            'odom_topic',
            default_value='/topic_gv_wheel_odom_0_283',
        ),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('ROS_DOMAIN_ID', '0'),
        SetEnvironmentVariable('CYCLONEDDS_URI', cyclone_uri),
        OpaqueFunction(function=_validate_resume),
        front_static_tf,
        rear_static_tf,
        scan_adapter,
        cartographer_node,
        keyboard_teleop_node,
        TimerAction(period=3.0, actions=[occupancy_grid_node]),
        TimerAction(period=4.0, actions=[rviz_node]),
    ])
