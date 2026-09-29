import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    params_file = LaunchConfiguration('params_file')
    docking_params_file = LaunchConfiguration('docking_params_file')
    manual_cmd_vel_topic = LaunchConfiguration('manual_cmd_vel_topic')
    manual_override_topic = LaunchConfiguration('manual_override_topic')

    nav_pkg_share = get_package_share_directory('control_center_navigation')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='false')
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml'),
    )
    declare_docking_params_file = DeclareLaunchArgument(
        'docking_params_file',
        default_value=os.path.join(nav_pkg_share, 'config', 'docking_params.yaml'),
    )
    declare_manual_cmd_vel_topic = DeclareLaunchArgument(
        'manual_cmd_vel_topic',
        default_value='/manual_cmd_vel',
    )
    declare_manual_override_topic = DeclareLaunchArgument(
        'manual_override_topic',
        default_value='/manual_cmd_vel_active',
    )

    planner_server = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    controller_server = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/nav_cmd_vel')],
    )

    behavior_server = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', '/nav_cmd_vel')],
    )

    cmd_vel_arbiter = Node(
        package='control_center_navigation',
        executable='cmd_vel_arbiter_node.py',
        name='cmd_vel_arbiter',
        output='screen',
        parameters=[
            docking_params_file,
            {
                'use_sim_time': use_sim_time,
                'nav_input_topic': '/nav_cmd_vel',
                'arm_input_topic': manual_cmd_vel_topic,
                'manual_override_topic': manual_override_topic,
                'output_topic': '/topic_gv_target_cmd_vel_0_307',
            }
        ],
    )

    bt_navigator = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
    )

    docking_server = Node(
        package='control_center_navigation',
        executable='docking_server_compat',
        name='docking_server',
        output='screen',
        parameters=[docking_params_file, {'use_sim_time': use_sim_time}],
        remappings=[
            ('cmd_vel', '/nav_cmd_vel'),
            ('battery_state', '/topic_gv_battery_0_307'),
        ],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': [
                    'planner_server',
                    'controller_server',
                    'behavior_server',
                    'bt_navigator',
                    'docking_server',
                ],
            }
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_params_file,
        declare_docking_params_file,
        declare_manual_cmd_vel_topic,
        declare_manual_override_topic,
        planner_server,
        controller_server,
        behavior_server,
        cmd_vel_arbiter,
        bt_navigator,
        docking_server,
        lifecycle_manager_navigation,
    ])

