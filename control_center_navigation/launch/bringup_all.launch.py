import os
from pathlib import Path
import re

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import yaml


PACKAGE_NAME = 'control_center_navigation'


def _source_package_directory(package_share):
    """Resolve both a direct src package and the grouped control-center suite."""
    share_path = Path(package_share).resolve()
    install_prefix = share_path.parents[1]
    workspace_root = install_prefix.parent.parent
    candidates = (
        workspace_root / 'src' / PACKAGE_NAME,
        workspace_root / 'src' / 'autolife_robot_control_center' / PACKAGE_NAME,
    )
    for candidate in candidates:
        if (candidate / 'package.xml').is_file():
            return candidate
    raise RuntimeError(
        f'Cannot find source package {PACKAGE_NAME} under {workspace_root / "src"}'
    )


def _source_maps_directory(package_share):
    """Return the package source maps directory for persistent edited maps."""
    maps_dir = _source_package_directory(package_share) / 'maps'
    maps_dir.mkdir(parents=True, exist_ok=True)
    return str(maps_dir)


def _active_map_defaults(package_share):
    source = _source_package_directory(package_share)
    config_path = source / 'config' / 'active_map.yaml'
    maps_dir = source / 'maps'
    waypoint_dir = maps_dir / 'waypoints'
    waypoint_dir.mkdir(parents=True, exist_ok=True)
    fallback_map = (
        Path(get_package_share_directory('multi_lidar_slam'))
        / 'maps' / 'yundie_2f.yaml'
    ).resolve()
    payload = {}
    if config_path.is_file():
        payload = yaml.safe_load(config_path.read_text(encoding='utf-8')) or {}

    map_path = Path(str(payload.get('active_map', '') or '')).expanduser()
    if not map_path.is_absolute():
        map_path = source / map_path
    map_path = map_path.resolve()
    if not map_path.is_file() or map_path.suffix.lower() != '.yaml':
        map_path = fallback_map

    safe_stem = re.sub(
        r'[^\w\u4e00-\u9fff-]+', '_', map_path.stem
    ).strip('_')
    waypoint_path = (
        waypoint_dir / safe_stem / f'{safe_stem}.yaml'
    ).resolve()
    waypoint_path.parent.mkdir(parents=True, exist_ok=True)
    waypoint_value = str(payload.get('waypoints_file', '') or '').strip()
    if waypoint_value and not waypoint_path.exists():
        legacy_path = Path(waypoint_value).expanduser()
        if not legacy_path.is_absolute():
            legacy_path = source / legacy_path
        legacy_path = legacy_path.resolve()
        if legacy_path.is_file() and legacy_path != waypoint_path:
            waypoint_path.write_bytes(legacy_path.read_bytes())
    if not waypoint_path.is_file():
        waypoint_path.write_text(
            yaml.safe_dump(
                {'map_yaml': str(map_path), 'waypoints': {}},
                allow_unicode=True,
                sort_keys=True,
            ),
            encoding='utf-8',
        )
    return {
        'map_yaml': str(map_path),
        'waypoints_file': str(waypoint_path),
        'waypoint_library_directory': str(waypoint_dir.resolve()),
        'active_map_config': str(config_path.resolve()),
    }


def generate_launch_description():
    nav_pkg_share = get_package_share_directory(PACKAGE_NAME)
    active_defaults = _active_map_defaults(nav_pkg_share)
    try:
        voice_pkg_share = get_package_share_directory('text_voice_control')
    except Exception:
        # Some migrated workspaces contain the package share tree but are
        # missing text_voice_control/local_setup.*. Resolve that known layout
        # relative to this package instead of requiring a manual AMENT_PREFIX.
        install_root = Path(nav_pkg_share).resolve().parents[2]
        voice_share_fallback = (
            install_root / 'text_voice_control' / 'share' / 'text_voice_control'
        )
        if not voice_share_fallback.is_dir():
            raise
        voice_pkg_share = str(voice_share_fallback)

    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart_localization = LaunchConfiguration('autostart_localization')
    autostart_navigation = LaunchConfiguration('autostart_navigation')
    enable_global_relocalization = LaunchConfiguration('enable_global_relocalization')
    global_relocalization_publish_initial_pose = LaunchConfiguration(
        'global_relocalization_publish_initial_pose'
    )
    enable_waypoint_manager = LaunchConfiguration('enable_waypoint_manager')
    short_distance_direct_start_enabled = LaunchConfiguration(
        'short_distance_direct_start_enabled'
    )
    enable_keyboard_control = LaunchConfiguration('enable_keyboard_control')
    enable_navigation_status_overlay = LaunchConfiguration('enable_navigation_status_overlay')
    enable_text_voice_control = LaunchConfiguration('enable_text_voice_control')
    enable_proximity_voice_alert = LaunchConfiguration('enable_proximity_voice_alert')
    text_voice_robot_id = LaunchConfiguration('text_voice_robot_id')
    text_voice_ros_domain_id = LaunchConfiguration('text_voice_ros_domain_id')
    proximity_front_scan_topic = LaunchConfiguration('proximity_front_scan_topic')
    startup_navigation_on_initialpose = LaunchConfiguration('startup_navigation_on_initialpose')
    enable_odom_topic_relay = LaunchConfiguration('enable_odom_topic_relay')
    manual_cmd_vel_topic = LaunchConfiguration('manual_cmd_vel_topic')
    map_yaml = LaunchConfiguration('map_yaml')
    params_file = LaunchConfiguration('params_file')
    global_relocalization_params_file = LaunchConfiguration(
        'global_relocalization_params_file'
    )
    voice_feedback_params_file = LaunchConfiguration('voice_feedback_params_file')
    proximity_voice_alert_params_file = LaunchConfiguration(
        'proximity_voice_alert_params_file'
    )
    waypoints_file = LaunchConfiguration('waypoints_file')
    waypoint_library_directory = LaunchConfiguration(
        'waypoint_library_directory'
    )
    active_map_config = LaunchConfiguration('active_map_config')
    edited_maps_directory = LaunchConfiguration('edited_maps_directory')
    open_navigation_ui = LaunchConfiguration('open_navigation_ui')

    vendor_cyclonedds_uri = (
        '<CycloneDDS><Domain><General><Interfaces>'
        '<NetworkInterface name="lo"/>'
        '</Interfaces></General><Discovery>'
        '<ParticipantIndex>auto</ParticipantIndex>'
        '<MaxAutoParticipantIndex>200</MaxAutoParticipantIndex>'
        '</Discovery></Domain></CycloneDDS>'
    )

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_autostart_localization = DeclareLaunchArgument(
        'autostart_localization',
        default_value='true',
    )
    declare_autostart_navigation = DeclareLaunchArgument(
        'autostart_navigation',
        default_value='false',
    )
    declare_enable_global_relocalization = DeclareLaunchArgument(
        'enable_global_relocalization',
        default_value='true',
        description='Automatically run full-map scan matching after startup',
    )
    declare_global_relocalization_publish_initial_pose = DeclareLaunchArgument(
        'global_relocalization_publish_initial_pose',
        default_value='true',
        description='Publish a successful global match to AMCL /initialpose',
    )
    declare_enable_waypoint_manager = DeclareLaunchArgument(
        'enable_waypoint_manager',
        default_value='true',
    )
    declare_short_distance_direct_start_enabled = DeclareLaunchArgument(
        'short_distance_direct_start_enabled',
        default_value='false',
        description='Allow goals inside the threshold to bypass Nav2 and start the pose servo directly',
    )
    declare_enable_keyboard_control = DeclareLaunchArgument(
        'enable_keyboard_control',
        default_value='false',
        description='Optional legacy pynput control; the native UI already handles keys',
    )
    declare_enable_navigation_status_overlay = DeclareLaunchArgument(
        'enable_navigation_status_overlay',
        default_value='false',
        description='Legacy marker overlay; not needed by the native UI',
    )
    declare_enable_text_voice_control = DeclareLaunchArgument(
        'enable_text_voice_control',
        default_value='true',
    )
    declare_enable_proximity_voice_alert = DeclareLaunchArgument(
        'enable_proximity_voice_alert',
        default_value='true',
        description='Speak a warning after persistent close returns in the front lidar sector',
    )
    declare_text_voice_robot_id = DeclareLaunchArgument(
        'text_voice_robot_id',
        default_value='283',
    )
    declare_text_voice_ros_domain_id = DeclareLaunchArgument(
        'text_voice_ros_domain_id',
        default_value='0',
    )
    declare_proximity_front_scan_topic = DeclareLaunchArgument(
        'proximity_front_scan_topic',
        default_value='/topic_gv_front_lidar_0_283',
    )
    declare_startup_navigation_on_initialpose = DeclareLaunchArgument(
        'startup_navigation_on_initialpose',
        default_value='true',
    )
    declare_enable_odom_topic_relay = DeclareLaunchArgument(
        'enable_odom_topic_relay',
        default_value='true',
    )
    declare_manual_cmd_vel_topic = DeclareLaunchArgument(
        'manual_cmd_vel_topic',
        default_value='/manual_cmd_vel',
    )
    declare_map_yaml = DeclareLaunchArgument(
        'map_yaml',
        default_value=active_defaults['map_yaml'],
    )
    declare_params_file = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(nav_pkg_share, 'config', 'nav2_params.yaml'),
    )
    declare_global_relocalization_params_file = DeclareLaunchArgument(
        'global_relocalization_params_file',
        default_value=os.path.join(
            nav_pkg_share,
            'config',
            'global_relocalization.yaml',
        ),
    )
    declare_voice_feedback_params_file = DeclareLaunchArgument(
        'voice_feedback_params_file',
        default_value=os.path.join(nav_pkg_share, 'config', 'voice_feedback.yaml'),
    )
    declare_proximity_voice_alert_params_file = DeclareLaunchArgument(
        'proximity_voice_alert_params_file',
        default_value=os.path.join(
            nav_pkg_share, 'config', 'proximity_voice_alert.yaml'
        ),
    )
    declare_waypoints_file = DeclareLaunchArgument(
        'waypoints_file',
        default_value=active_defaults['waypoints_file'],
    )
    declare_waypoint_library_directory = DeclareLaunchArgument(
        'waypoint_library_directory',
        default_value=active_defaults['waypoint_library_directory'],
    )
    declare_active_map_config = DeclareLaunchArgument(
        'active_map_config',
        default_value=active_defaults['active_map_config'],
    )
    declare_edited_maps_directory = DeclareLaunchArgument(
        'edited_maps_directory',
        default_value=_source_maps_directory(nav_pkg_share),
        description='Source directory used for new, non-overwriting edited maps',
    )
    declare_open_navigation_ui = DeclareLaunchArgument(
        'open_navigation_ui',
        default_value='true',
        description='Open the standalone Qt navigation interface',
    )

    localization_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_share, 'launch', 'bringup_localization.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart_localization,
            'map_yaml': map_yaml,
            'params_file': params_file,
        }.items(),
    )

    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_pkg_share, 'launch', 'bringup_navigation.launch.py')
        ),
        launch_arguments={
            'use_sim_time': use_sim_time,
            'autostart': autostart_navigation,
            'params_file': params_file,
            'manual_cmd_vel_topic': manual_cmd_vel_topic,
        }.items(),
    )

    text_voice_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(voice_pkg_share, 'launch', 'bringup.launch.py')
        ),
        condition=IfCondition(enable_text_voice_control),
        launch_arguments={
            'robot_id': text_voice_robot_id,
            'ros_domain_id': text_voice_ros_domain_id,
            'enable_asr_bridge': 'false',
        }.items(),
    )

    waypoint_manager = Node(
        package='control_center_navigation',
        executable='waypoint_manager_node.py',
        name='waypoint_manager',
        condition=IfCondition(enable_waypoint_manager),
        output='screen',
        parameters=[
            voice_feedback_params_file,
            {
                'use_sim_time': use_sim_time,
                'waypoints_file': waypoints_file,
                'waypoint_library_directory': waypoint_library_directory,
                'active_map_yaml': map_yaml,
                'map_frame': 'map',
                'robot_frame': 'base_link',
                'navigate_action_name': 'navigate_to_pose',
                'short_distance_direct_start_enabled': short_distance_direct_start_enabled,
            }
        ],
    )

    proximity_voice_alert = Node(
        package=PACKAGE_NAME,
        executable='proximity_voice_alert_node.py',
        name='proximity_voice_alert',
        condition=IfCondition(enable_proximity_voice_alert),
        output='screen',
        parameters=[
            proximity_voice_alert_params_file,
            {
                'use_sim_time': use_sim_time,
                'front_scan_topic': proximity_front_scan_topic,
            },
        ],
    )

    keyboard_waypoint_control = Node(
        package='control_center_navigation',
        executable='keyboard_waypoint_control_node.py',
        name='keyboard_waypoint_control',
        condition=IfCondition(enable_keyboard_control),
        output='screen',
        emulate_tty=True,
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'manual_cmd_vel_topic': manual_cmd_vel_topic,
                'graphical_prompts': True,
                'show_control_gui': False,
            }
        ],
    )

    navigation_status_overlay = Node(
        package='control_center_navigation',
        executable='navigation_status_overlay_node.py',
        name='navigation_status_overlay',
        condition=IfCondition(enable_navigation_status_overlay),
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'status_topic': '/waypoints/navigation_status',
                'marker_topic': '/waypoints/navigation_status_marker',
                'frame_id': 'map',
                'x': -4.0,
                'y': 4.0,
                'z': 1.2,
                'text_scale': 0.35,
            }
        ],
    )

    navigation_startup_on_initialpose = Node(
        package='control_center_navigation',
        executable='navigation_startup_on_initialpose_node.py',
        name='navigation_startup_on_initialpose',
        condition=IfCondition(startup_navigation_on_initialpose),
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'initialpose_topic': '/initialpose',
                'amcl_pose_topic': '/amcl_pose',
                'manage_nodes_service': '/lifecycle_manager_navigation/manage_nodes',
                'startup_delay_sec': 1.0,
                'require_amcl_convergence': True,
                'convergence_stable_samples': 3,
                'max_covariance_x': 0.50,
                'max_covariance_y': 0.50,
                'max_covariance_yaw': 0.40,
            }
        ],
    )

    global_relocalization = Node(
        package=PACKAGE_NAME,
        executable='cartographer_global_relocalization',
        name='cartographer_global_relocalization',
        condition=IfCondition(enable_global_relocalization),
        output='screen',
        parameters=[
            global_relocalization_params_file,
            {
                'use_sim_time': use_sim_time,
                'publish_initial_pose': global_relocalization_publish_initial_pose,
            },
        ],
    )

    delayed_global_relocalization = TimerAction(
        period=3.0,
        actions=[global_relocalization],
    )

    odom_topic_relay = Node(
        package='control_center_navigation',
        executable='odom_topic_relay_node.py',
        name='odom_topic_relay',
        condition=IfCondition(enable_odom_topic_relay),
        output='screen',
        parameters=[
            {
                'use_sim_time': use_sim_time,
                'input_topic': '/topic_gv_wheel_odom_0_283',
                'output_topic': '/odom',
            }
        ],
    )

    navigation_ui = Node(
        package=PACKAGE_NAME,
        executable='navigation_ui_node.py',
        name='navigation_native_ui',
        condition=IfCondition(open_navigation_ui),
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'map_topic': '/map',
            'amcl_pose_topic': '/amcl_pose',
            'global_path_topic': '/plan',
            'local_path_topic': '/local_plan',
            'front_scan_topic': '/topic_gv_front_lidar_0_283',
            'rear_scan_topic': '/topic_gv_rear_lidar_0_283',
            'manual_cmd_vel_topic': manual_cmd_vel_topic,
            'initial_map_yaml': map_yaml,
            'edited_maps_directory': edited_maps_directory,
            'waypoint_library_directory': waypoint_library_directory,
            'active_map_config': active_map_config,
        }],
    )

    delayed_navigation_ui = TimerAction(
        period=2.0,
        actions=[navigation_ui],
    )

    return LaunchDescription([
        SetEnvironmentVariable('ROBOT_ID', '283'),
        SetEnvironmentVariable('RMW_IMPLEMENTATION', 'rmw_cyclonedds_cpp'),
        SetEnvironmentVariable('CYCLONEDDS_URI', vendor_cyclonedds_uri),
        declare_use_sim_time,
        declare_autostart_localization,
        declare_autostart_navigation,
        declare_enable_global_relocalization,
        declare_global_relocalization_publish_initial_pose,
        declare_enable_waypoint_manager,
        declare_short_distance_direct_start_enabled,
        declare_enable_keyboard_control,
        declare_enable_navigation_status_overlay,
        declare_enable_text_voice_control,
        declare_enable_proximity_voice_alert,
        declare_text_voice_robot_id,
        declare_text_voice_ros_domain_id,
        declare_proximity_front_scan_topic,
        SetEnvironmentVariable('ROS_DOMAIN_ID', text_voice_ros_domain_id),
        declare_startup_navigation_on_initialpose,
        declare_enable_odom_topic_relay,
        declare_manual_cmd_vel_topic,
        declare_map_yaml,
        declare_params_file,
        declare_global_relocalization_params_file,
        declare_voice_feedback_params_file,
        declare_proximity_voice_alert_params_file,
        declare_waypoints_file,
        declare_waypoint_library_directory,
        declare_active_map_config,
        declare_edited_maps_directory,
        declare_open_navigation_ui,
        localization_launch,
        navigation_launch,
        text_voice_control_launch,
        waypoint_manager,
        proximity_voice_alert,
        keyboard_waypoint_control,
        navigation_status_overlay,
        navigation_startup_on_initialpose,
        delayed_global_relocalization,
        odom_topic_relay,
        delayed_navigation_ui,
    ])
