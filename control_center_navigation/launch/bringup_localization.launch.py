import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


def _source_package_directory(package_share):
    share = Path(package_share).resolve()
    workspace_root = share.parents[1].parent.parent
    for candidate in (
        workspace_root / 'src' / 'control_center_navigation',
        workspace_root / 'src' / 'autolife_robot_control_center'
        / 'control_center_navigation',
    ):
        if (candidate / 'package.xml').is_file():
            return candidate
    return share


def _default_active_map(nav_pkg_share, slam_pkg_share):
    source = _source_package_directory(nav_pkg_share)
    config = source / 'config' / 'active_map.yaml'
    try:
        payload = yaml.safe_load(config.read_text(encoding='utf-8')) or {}
        path = Path(str(payload.get('active_map', '') or '')).expanduser()
        if not path.is_absolute():
            path = source / path
        path = path.resolve()
        if path.is_file() and path.suffix.lower() == '.yaml':
            return str(path)
    except Exception:
        pass
    return os.path.join(slam_pkg_share, 'maps', 'yundie_2f.yaml')


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')
    map_yaml = LaunchConfiguration('map_yaml')
    params_file = LaunchConfiguration('params_file')
    laser_filter_params_file = LaunchConfiguration('laser_filter_params_file')

    nav_pkg_share = get_package_share_directory('control_center_navigation')
    slam_pkg_share = get_package_share_directory('multi_lidar_slam')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_autostart = DeclareLaunchArgument('autostart', default_value='true')
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value=_default_active_map(nav_pkg_share, slam_pkg_share),
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml'),
    )
    declare_laser_filter_params_file = DeclareLaunchArgument(
        'laser_filter_params_file',
        default_value=os.path.join(nav_pkg_share, 'config', 'laser_filter.yaml'),
    )

    static_tf_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_front',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'front_laser_frame',
            '--x', '0.215', '--y', '0.0', '--z', '0.1515',
            '--roll', '3.145926', '--pitch', '0.0', '--yaw', '3.145926',
        ],
        output='screen',
    )

    static_tf_rear = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_tf_rear',
        arguments=[
            '--frame-id', 'base_link',
            '--child-frame-id', 'rear_laser_frame',
            '--x', '-0.215', '--y', '0.0', '--z', '0.1515',
            '--roll', '3.145926', '--pitch', '0.0', '--yaw', '0.0',
        ],
        output='screen',
    )

    dual_laser_merger = Node(
        package='dual_laser_merger',
        executable='dual_laser_merger_node',
        name='dual_laser_merger_0_307',
        output='screen',
        parameters=[{
            'laser_1_topic': '/topic_gv_front_lidar_0_307',
            'laser_2_topic': '/topic_gv_rear_lidar_0_307',
            'merged_scan_topic': '/origin_merged',
            'target_frame': 'base_link',
            'laser_1_x_offset': 0.0,
            'laser_1_y_offset': 0.0,
            'laser_1_yaw_offset': 0.0,
            'laser_2_x_offset': 0.0,
            'laser_2_y_offset': 0.0,
            'laser_2_yaw_offset': 0.0,
            'tolerance': 0.01,
            'queue_size': 5,
            'angle_increment': 0.0174,
            'scan_time': 0.067,
            'range_min': 0.01,
            'range_max': 25.0,
            'min_height': -1.0,
            'max_height': 1.0,
            'angle_min': -3.141592654,
            'angle_max': 3.141592654,
            'inf_epsilon': 1.0,
            'use_inf': True,
            'allowed_radius': 0.45,
            'enable_shadow_filter': True,
        }],
    )

    merged_scan_filter = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain_node',
        output='screen',
        parameters=[laser_filter_params_file],
        remappings=[
            ('scan', '/origin_merged'),
            ('scan_filtered', '/merged'),
        ],
    )

    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[
            params_file,
            {
                'use_sim_time': use_sim_time,
                'yaml_filename': map_yaml,
            }
        ],
    )

    amcl = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time}],
        remappings=[('scan', '/merged')],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'autostart': autostart,
                'node_names': ['map_server', 'amcl'],
            }
        ],
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_autostart,
        declare_map_yaml,
        declare_params_file,
        declare_laser_filter_params_file,
        static_tf_front,
        static_tf_rear,
        dual_laser_merger,
        merged_scan_filter,
        map_server,
        amcl,
        lifecycle_manager_localization,
    ])
