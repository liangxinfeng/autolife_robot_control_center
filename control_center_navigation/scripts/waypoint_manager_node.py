#!/usr/bin/env python3
import math
import json
import os
from pathlib import Path
import re
import tempfile
from typing import Dict, List, Optional, Set

from action_msgs.msg import GoalStatus
import rclpy
from geometry_msgs.msg import PoseStamped, Quaternion, Twist
from nav2_msgs.msg import Costmap
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import ColorRGBA, String
from std_srvs.srv import Trigger
from visualization_msgs.msg import Marker, MarkerArray
from nav2_msgs.action import DockRobot, DriveOnHeading, NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
import yaml

from short_distance_servo import (
    axis_command,
    body_frame_error,
    bounded_pose_update,
    corridor_is_clear,
    rate_limit,
    reproject_target_pose,
)

from control_center_navigation.srv import (
    DeleteWaypoint,
    ListWaypoints,
    NavigateToPoseRefined,
    NavigateToWaypoint,
    NavigateWaypointSequence,
    RenameWaypoint,
    SaveWaypoint,
    SwitchWaypointMap,
)


TRANSIENT_MANUAL_GOAL_NAME = '__manual_map_goal__'
TRANSIENT_MANUAL_GOAL_LABEL = '地图指定位置'


class WaypointManagerNode(Node):
    def __init__(self) -> None:
        super().__init__('waypoint_manager')

        self.declare_parameter(
            'waypoints_file',
            str(Path.home() / '.ros' / 'control_center_navigation' / 'waypoints.yaml'),
        )
        self.declare_parameter(
            'waypoint_library_directory',
            str(Path.home() / '.ros' / 'control_center_navigation' / 'waypoints'),
        )
        self.declare_parameter('active_map_yaml', '')
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('robot_frame', 'base_link')
        self.declare_parameter('navigate_action_name', 'navigate_to_pose')
        self.declare_parameter('waypoint_marker_topic', '/waypoints/markers')
        self.declare_parameter('waypoint_marker_scale', 0.35)
        self.declare_parameter('navigation_status_topic', '/waypoints/navigation_status')
        # Final arrival must stay inside the same strict precision envelope
        # even when the local pose-servo corridor is temporarily unavailable.
        self.declare_parameter('arrival_xy_tolerance', 0.06)
        self.declare_parameter('arrival_yaw_tolerance', 0.08)
        self.declare_parameter('arrival_settle_time', 0.60)
        self.declare_parameter('final_refine_handoff_distance', 0.25)
        self.declare_parameter('final_refine_handoff_yaw', 0.35)
        self.declare_parameter('final_refine_obstacle_wait', 20.0)
        self.declare_parameter('final_refine_clear_settle', 0.60)
        self.declare_parameter('final_precision_nav2_retries', 1)
        self.declare_parameter('short_distance_servo_enabled', True)
        # All front ends use Nav2 for the main leg.  The short-distance servo
        # remains available only for the post-Nav2 final pose refinement.
        self.declare_parameter('short_distance_direct_start_enabled', False)
        self.declare_parameter('far_navigation_auto_refine_enabled', True)
        self.declare_parameter('short_distance_threshold', 2.0)
        self.declare_parameter('short_distance_cmd_vel_topic', '/nav_cmd_vel')
        self.declare_parameter('short_distance_costmap_topic', '/local_costmap/costmap_raw')
        self.declare_parameter('short_distance_costmap_timeout', 0.75)
        self.declare_parameter('short_distance_path_check_period', 0.10)
        self.declare_parameter('short_distance_robot_half_length', 0.25)
        self.declare_parameter('short_distance_robot_half_width', 0.25)
        self.declare_parameter('short_distance_front_extension', 0.0)
        # Match local/global Nav2 footprint and its retained 4 cm padding.
        # 307 uses the original base length without an extra front extension.
        self.declare_parameter('short_distance_safety_margin', 0.04)
        self.declare_parameter('short_distance_lethal_cost', 254)
        self.declare_parameter('short_distance_xy_tolerance', 0.04)
        self.declare_parameter('short_distance_axis_tolerance', 0.020)
        self.declare_parameter('short_distance_yaw_tolerance', 0.05)
        self.declare_parameter('short_distance_yaw_exit_tolerance', 0.075)
        self.declare_parameter('short_distance_settle_time', 0.50)
        self.declare_parameter('short_distance_stage_settle_time', 0.18)
        self.declare_parameter('short_distance_xy_gain', 1.20)
        self.declare_parameter('short_distance_yaw_gain', 1.50)
        self.declare_parameter('short_distance_max_linear_speed', 0.16)
        self.declare_parameter('short_distance_max_angular_speed', 0.45)
        self.declare_parameter('short_distance_min_linear_speed', 0.018)
        self.declare_parameter('short_distance_min_angular_speed', 0.045)
        self.declare_parameter('short_distance_max_linear_acceleration', 0.50)
        self.declare_parameter('short_distance_max_angular_acceleration', 1.00)
        self.declare_parameter('short_distance_progress_timeout', 3.0)
        self.declare_parameter('short_distance_progress_epsilon', 0.015)
        self.declare_parameter('short_distance_max_duration', 35.0)
        self.declare_parameter('short_distance_map_correction_gain', 0.08)
        self.declare_parameter('short_distance_max_map_xy_correction_speed', 0.04)
        self.declare_parameter('short_distance_max_map_yaw_correction_speed', 0.08)
        self.declare_parameter('short_distance_final_map_settle_time', 0.50)
        self.declare_parameter('short_distance_max_final_correction_passes', 4)
        self.declare_parameter('auto_undock_enabled', True)
        self.declare_parameter('charger_waypoint_name', '充电桩')
        self.declare_parameter('undock_action_name', 'drive_on_heading')
        self.declare_parameter('undock_trigger_distance', 0.35)
        self.declare_parameter('undock_yaw_tolerance', 0.35)
        self.declare_parameter('undock_distance', 0.45)
        self.declare_parameter('undock_speed', 0.15)
        self.declare_parameter('undock_timeout', 15.0)
        self.declare_parameter('precise_docking_enabled', True)
        self.declare_parameter('dock_action_name', 'dock_robot')
        self.declare_parameter('dock_type', 'fine_charging_dock')
        self.declare_parameter('dock_pose_yaw_offset', math.pi)
        self.declare_parameter('dock_max_staging_time', 120.0)
        self.declare_parameter('dock_navigate_to_staging_pose', True)
        self.declare_parameter('dock_prealign_enabled', True)
        self.declare_parameter('dock_staging_distance', 0.55)
        self.declare_parameter('dock_max_lateral_drift', 0.06)
        self.declare_parameter('dock_max_heading_drift', 0.14)
        self.declare_parameter('dock_verify_xy_tolerance', 0.05)
        self.declare_parameter('dock_verify_yaw_tolerance', 0.06)
        self.declare_parameter('dock_cmd_vel_topic', '/nav_cmd_vel')
        self.declare_parameter('dock_telemetry_period', 0.5)
        self.declare_parameter('battery_topic', '/topic_gv_battery_0_307')
        self.declare_parameter('charging_status_confirm_samples', 2)
        self.declare_parameter('voice_feedback_enabled', False)
        self.declare_parameter('voice_command_topic', '/text_voice_command')
        self.declare_parameter('voice_departure_template', '我现在前往{waypoint}')
        self.declare_parameter('voice_arrival_template', '我已到达{waypoint}')

        self._waypoints_file = Path(
            self.get_parameter('waypoints_file').get_parameter_value().string_value
        ).expanduser().resolve()
        self._waypoint_library_directory = Path(
            self.get_parameter('waypoint_library_directory').value
        ).expanduser().resolve()
        active_map = str(self.get_parameter('active_map_yaml').value or '').strip()
        self._active_map_yaml = (
            Path(active_map).expanduser().resolve() if active_map else None
        )
        self._map_frame = self.get_parameter('map_frame').get_parameter_value().string_value
        self._odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self._robot_frame = self.get_parameter('robot_frame').get_parameter_value().string_value
        action_name = self.get_parameter('navigate_action_name').get_parameter_value().string_value
        self._marker_topic = self.get_parameter('waypoint_marker_topic').get_parameter_value().string_value
        self._marker_scale = self.get_parameter('waypoint_marker_scale').get_parameter_value().double_value
        self._status_topic = self.get_parameter('navigation_status_topic').get_parameter_value().string_value
        self._arrival_xy_tolerance = self.get_parameter('arrival_xy_tolerance').get_parameter_value().double_value
        self._arrival_yaw_tolerance = self.get_parameter('arrival_yaw_tolerance').get_parameter_value().double_value
        self._arrival_settle_time = self.get_parameter('arrival_settle_time').get_parameter_value().double_value
        self._refine_handoff_distance = min(0.4, max(0.06, float(self.get_parameter('final_refine_handoff_distance').value)))
        self._refine_handoff_yaw = min(0.5, max(0.08, float(self.get_parameter('final_refine_handoff_yaw').value)))
        self._refine_obstacle_wait = max(1.0, float(self.get_parameter('final_refine_obstacle_wait').value))
        self._refine_clear_settle = max(0.3, float(self.get_parameter('final_refine_clear_settle').value))
        self._final_precision_nav2_retries = max(
            0, int(self.get_parameter('final_precision_nav2_retries').value)
        )
        self._far_navigation_auto_refine_enabled = bool(
            self.get_parameter('far_navigation_auto_refine_enabled').value
        )
        self._short_servo_enabled = bool(self.get_parameter('short_distance_servo_enabled').value)
        self._short_direct_start_enabled = bool(
            self.get_parameter('short_distance_direct_start_enabled').value
        )
        self._short_distance_threshold = max(
            0.0, float(self.get_parameter('short_distance_threshold').value)
        )
        short_cmd_vel_topic = str(self.get_parameter('short_distance_cmd_vel_topic').value)
        short_costmap_topic = str(self.get_parameter('short_distance_costmap_topic').value)
        self._short_costmap_timeout = max(
            0.1, float(self.get_parameter('short_distance_costmap_timeout').value)
        )
        self._short_path_check_period = max(
            0.05, float(self.get_parameter('short_distance_path_check_period').value)
        )
        self._short_half_length = max(
            0.0, float(self.get_parameter('short_distance_robot_half_length').value)
        ) + max(0.0, float(self.get_parameter('short_distance_safety_margin').value))
        self._short_half_width = max(
            0.0, float(self.get_parameter('short_distance_robot_half_width').value)
        ) + max(0.0, float(self.get_parameter('short_distance_safety_margin').value))
        self._short_lethal_cost = int(self.get_parameter('short_distance_lethal_cost').value)
        self._short_front_extension = max(
            0.0, float(self.get_parameter('short_distance_front_extension').value)
        )
        self._short_xy_tolerance = max(
            0.01, float(self.get_parameter('short_distance_xy_tolerance').value)
        )
        self._short_axis_tolerance = max(
            0.01, float(self.get_parameter('short_distance_axis_tolerance').value)
        )
        self._short_yaw_tolerance = max(
            0.01, float(self.get_parameter('short_distance_yaw_tolerance').value)
        )
        self._short_yaw_exit_tolerance = max(
            self._short_yaw_tolerance,
            float(self.get_parameter('short_distance_yaw_exit_tolerance').value),
        )
        self._short_settle_time = max(
            0.1, float(self.get_parameter('short_distance_settle_time').value)
        )
        self._short_stage_settle_time = max(
            0.05, float(self.get_parameter('short_distance_stage_settle_time').value)
        )
        self._short_xy_gain = max(0.0, float(self.get_parameter('short_distance_xy_gain').value))
        self._short_yaw_gain = max(0.0, float(self.get_parameter('short_distance_yaw_gain').value))
        self._short_max_linear_speed = max(
            0.01, float(self.get_parameter('short_distance_max_linear_speed').value)
        )
        self._short_max_angular_speed = max(
            0.01, float(self.get_parameter('short_distance_max_angular_speed').value)
        )
        self._short_min_linear_speed = max(
            0.0, float(self.get_parameter('short_distance_min_linear_speed').value)
        )
        self._short_min_angular_speed = max(
            0.0, float(self.get_parameter('short_distance_min_angular_speed').value)
        )
        self._short_max_linear_acceleration = max(
            0.01, float(self.get_parameter('short_distance_max_linear_acceleration').value)
        )
        self._short_max_angular_acceleration = max(
            0.01, float(self.get_parameter('short_distance_max_angular_acceleration').value)
        )
        self._short_progress_timeout = max(
            1.0, float(self.get_parameter('short_distance_progress_timeout').value)
        )
        self._short_progress_epsilon = max(
            0.001, float(self.get_parameter('short_distance_progress_epsilon').value)
        )
        self._short_max_duration = max(
            3.0, float(self.get_parameter('short_distance_max_duration').value)
        )
        self._short_map_correction_gain = max(
            0.0, min(1.0, float(self.get_parameter('short_distance_map_correction_gain').value))
        )
        self._short_max_map_xy_correction_step = max(
            0.0,
            float(self.get_parameter('short_distance_max_map_xy_correction_speed').value)
            * 0.05,
        )
        self._short_max_map_yaw_correction_step = max(
            0.0,
            float(self.get_parameter('short_distance_max_map_yaw_correction_speed').value)
            * 0.05,
        )
        self._short_final_map_settle_time = max(
            0.1, float(self.get_parameter('short_distance_final_map_settle_time').value)
        )
        self._short_max_final_correction_passes = max(
            1, int(self.get_parameter('short_distance_max_final_correction_passes').value)
        )
        self._auto_undock_enabled = self.get_parameter('auto_undock_enabled').value
        self._charger_waypoint_name = self.get_parameter('charger_waypoint_name').value
        undock_action_name = self.get_parameter('undock_action_name').value
        self._undock_trigger_distance = self.get_parameter('undock_trigger_distance').value
        self._undock_yaw_tolerance = self.get_parameter('undock_yaw_tolerance').value
        self._undock_distance = self.get_parameter('undock_distance').value
        self._undock_speed = self.get_parameter('undock_speed').value
        self._undock_timeout = self.get_parameter('undock_timeout').value
        self._precise_docking_enabled = self.get_parameter('precise_docking_enabled').value
        dock_action_name = self.get_parameter('dock_action_name').value
        self._dock_type = self.get_parameter('dock_type').value
        self._dock_pose_yaw_offset = self.get_parameter('dock_pose_yaw_offset').value
        self._dock_max_staging_time = self.get_parameter('dock_max_staging_time').value
        self._dock_navigate_to_staging_pose = self.get_parameter(
            'dock_navigate_to_staging_pose'
        ).value
        self._dock_verify_xy_tolerance = self.get_parameter('dock_verify_xy_tolerance').value
        self._dock_prealign_enabled = bool(self.get_parameter('dock_prealign_enabled').value)
        self._dock_staging_distance = float(self.get_parameter('dock_staging_distance').value)
        if not math.isfinite(self._dock_staging_distance) or not 0.35 <= self._dock_staging_distance <= 1.5:
            raise ValueError('dock_staging_distance must be within 0.35..1.5 m')
        self._dock_staging_active = False
        self._dock_target_odom_pose = None
        self._dock_cancel_pending = False
        self._dock_max_lateral_drift = float(self.get_parameter('dock_max_lateral_drift').value)
        self._dock_max_heading_drift = float(self.get_parameter('dock_max_heading_drift').value)
        if not (0.02 <= self._dock_max_lateral_drift <= 0.10 and
                0.03 <= self._dock_max_heading_drift <= 0.20):
            raise ValueError('invalid docking drift limits')
        self._last_battery_status_time = None
        self._battery_is_charging = False
        self._dock_verify_yaw_tolerance = self.get_parameter('dock_verify_yaw_tolerance').value
        dock_cmd_vel_topic = self.get_parameter('dock_cmd_vel_topic').value
        self._dock_telemetry_period = self.get_parameter('dock_telemetry_period').value
        battery_topic = self.get_parameter('battery_topic').value
        self._charging_status_confirm_samples = self.get_parameter(
            'charging_status_confirm_samples'
        ).value
        self._voice_feedback_enabled = (
            self.get_parameter('voice_feedback_enabled').get_parameter_value().bool_value
        )
        self._voice_command_topic = (
            self.get_parameter('voice_command_topic').get_parameter_value().string_value
        )
        self._voice_departure_template = (
            self.get_parameter('voice_departure_template').get_parameter_value().string_value
        )
        self._voice_arrival_template = (
            self.get_parameter('voice_arrival_template').get_parameter_value().string_value
        )

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)
        self._navigate_client = ActionClient(self, NavigateToPose, action_name)
        self._undock_client = ActionClient(self, DriveOnHeading, undock_action_name)
        self._dock_client = ActionClient(self, DockRobot, dock_action_name)
        self._dock_cmd_vel_sub = self.create_subscription(
            Twist,
            dock_cmd_vel_topic,
            self._handle_docking_cmd_vel,
            20,
        )
        self._battery_sub = self.create_subscription(
            BatteryState,
            battery_topic,
            self._handle_battery_state,
            qos_profile_sensor_data,
        )
        self._short_cmd_vel_pub = self.create_publisher(Twist, short_cmd_vel_topic, 10)
        self._short_costmap_sub = self.create_subscription(
            Costmap,
            short_costmap_topic,
            self._handle_short_costmap,
            qos_profile_sensor_data,
        )
        marker_qos = QoSProfile(depth=1)
        marker_qos.reliability = ReliabilityPolicy.RELIABLE
        marker_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._marker_pub = self.create_publisher(MarkerArray, self._marker_topic, marker_qos)
        status_qos = QoSProfile(depth=10)
        status_qos.reliability = ReliabilityPolicy.RELIABLE
        status_qos.durability = DurabilityPolicy.TRANSIENT_LOCAL
        self._status_pub = self.create_publisher(String, self._status_topic, status_qos)
        self._voice_pub = self.create_publisher(String, self._voice_command_topic, 10)
        self._goal_futures = []
        self._result_futures = []
        index_path = self._waypoint_index_path()
        if index_path.exists():
            self._waypoints = self._load_waypoints_from_index()
        else:
            self.get_logger().warning(
                f'Canonical waypoint index does not exist; creating an empty library: {index_path}'
            )
            self._waypoints = {}
        # maps_index.json is canonical. YAML is only a write-only compatibility mirror.
        self._write_waypoints()

        self._active_sequence: List[str] = []
        self._active_sequence_dwell_times: List[float] = []
        self._active_sequence_index = -1
        self._active_sequence_loop = False
        self._sequence_dwell_timer = None
        self._sequence_dwell_token = 0
        self._active_goal_name: Optional[str] = None
        self._active_goal_id = 0
        self._active_goal_handle = None
        self._active_undock_goal_handle = None
        self._active_dock_goal_handle = None
        self._precise_docking_active = False
        self._last_docking_feedback_state = None
        self._last_docking_telemetry_time = None
        self._charging_status_count = 0
        self._charging_detection_armed = True
        self._undock_pending_goal_name: Optional[str] = None
        self._last_undock_feedback_log_time = None
        self._active_goal_start_time = None
        self._goal_reached_since = None
        self._arrival_refine_pending = False
        self._arrival_refine_requested_at = None
        self._navigation_paused = False
        self._paused_goal_name: Optional[str] = None
        self._last_feedback = None
        self._last_navigation_status = "idle"
        self._last_arrival_log_time = None
        self._completed_waypoints: Set[str] = set()
        self._failed_goal_name: Optional[str] = None
        self._final_precision_retry_count = 0
        self._short_servo_active = False
        self._short_final_refine = False
        self._short_blocked_since = None
        self._short_clear_since = None
        self._short_target = None
        self._short_target_map_pose = None
        self._short_odom_target = None
        self._short_costmap = None
        self._short_costmap_received_time = None
        self._short_translation_locked = False
        self._short_stable_since = None
        self._short_best_distance = None
        self._short_last_progress_time = None
        self._short_last_path_check_time = None
        self._short_last_status_time = None
        self._short_last_command = (0.0, 0.0, 0.0)
        self._short_stage = None
        self._short_stage_ready_since = None
        self._short_correction_pass = 0
        self._short_final_verify_since = None
        self._short_arrival_since = None
        self._short_servo_period = 0.05

        self.create_service(SaveWaypoint, 'save_waypoint', self._handle_save_waypoint)
        self.create_service(DeleteWaypoint, 'delete_waypoint', self._handle_delete_waypoint)
        self.create_service(RenameWaypoint, 'rename_waypoint', self._handle_rename_waypoint)
        self.create_service(ListWaypoints, 'list_waypoints', self._handle_list_waypoints)
        self.create_service(
            NavigateToPoseRefined,
            'navigate_to_pose_refined',
            self._handle_navigate_to_pose_refined,
        )
        self.create_service(
            SwitchWaypointMap,
            'switch_waypoint_map',
            self._handle_switch_waypoint_map,
        )
        self.create_service(
            NavigateToWaypoint,
            'navigate_to_waypoint',
            self._handle_navigate_to_waypoint,
        )
        self.create_service(
            NavigateWaypointSequence,
            'navigate_waypoint_sequence',
            self._handle_navigate_waypoint_sequence,
        )
        self.create_service(Trigger, 'pause_navigation', self._handle_pause_navigation)
        self.create_service(Trigger, 'resume_navigation', self._handle_resume_navigation)
        self.create_service(Trigger, 'cancel_navigation', self._handle_cancel_navigation)

        self.create_timer(2.0, self._publish_waypoint_markers)
        self.create_timer(0.25, self._check_active_goal_arrival)
        self.create_timer(self._short_servo_period, self._run_short_distance_servo)
        self.create_timer(0.10, self._check_docking_drift)
        self._publish_waypoint_markers()

        self.get_logger().info(f'Canonical waypoint index: {self._waypoint_index_path()}')
        self.get_logger().info(f'Waypoint YAML mirror (write-only): {self._waypoints_file}')
        self.get_logger().info(f'Waypoint markers: {self._marker_topic}')
        self.get_logger().info(f'Navigation status: {self._status_topic}')
        self.get_logger().info(
            f'Voice feedback: {self._voice_feedback_enabled}, topic: {self._voice_command_topic}'
        )
        self.get_logger().info(
            f'Precise docking: {self._precise_docking_enabled}, '
            f'charger waypoint: {self._charger_waypoint_name}, dock type: {self._dock_type}'
        )
        self.get_logger().info(
            f'Short-distance pose servo: enabled={self._short_servo_enabled}, '
            f'direct_start={self._short_direct_start_enabled}, '
            f'far_nav_refine={self._far_navigation_auto_refine_enabled}, '
            f'threshold={self._short_distance_threshold:.2f} m, '
            f'cmd_vel={short_cmd_vel_topic}, costmap={short_costmap_topic}'
        )
        self._publish_navigation_status('idle', 'waypoint manager ready')

    def _active_map_name(self) -> str:
        if self._active_map_yaml is not None:
            return self._active_map_yaml.stem
        return self._waypoints_file.stem

    @staticmethod
    def _is_transient_waypoint(name: str) -> bool:
        return name == TRANSIENT_MANUAL_GOAL_NAME

    def _public_waypoint_names(self):
        return sorted(
            name for name in self._waypoints
            if not self._is_transient_waypoint(name)
        )

    def _persistent_waypoints(self):
        return {
            name: waypoint
            for name, waypoint in self._waypoints.items()
            if not self._is_transient_waypoint(name)
        }

    @staticmethod
    def _display_goal_name(name: Optional[str]) -> str:
        if name == TRANSIENT_MANUAL_GOAL_NAME:
            return TRANSIENT_MANUAL_GOAL_LABEL
        return str(name or '')

    def _waypoint_index_path(self, waypoint_path=None) -> Path:
        path = Path(waypoint_path or self._waypoints_file)
        return path.parent / 'maps_index.json'

    def _load_waypoints_from_index(self, waypoint_path=None) -> Dict[str, Dict[str, float]]:
        """Load only maps_index.json; the waypoint YAML mirror is never read."""
        index_path = self._waypoint_index_path(waypoint_path)
        expected_map = self._active_map_name()
        try:
            content = json.loads(index_path.read_text(encoding='utf-8'))
            if not isinstance(content, dict):
                raise ValueError('top level must be an object')
            active_map = str(content.get('active_map', '') or '').strip()
            if active_map != expected_map:
                raise ValueError(
                    f'active_map is {active_map!r}, expected {expected_map!r}'
                )
            maps = content.get('maps')
            if not isinstance(maps, dict):
                raise ValueError('maps must be an object')
            map_entry = maps.get(expected_map)
            if not isinstance(map_entry, dict):
                raise ValueError(f'maps.{expected_map} must be an object')
            prepared_positions = map_entry.get('prepared_positions', [])
            if not isinstance(prepared_positions, list):
                raise ValueError('prepared_positions must be an array')

            waypoints = {}
            for index, item in enumerate(prepared_positions):
                if not isinstance(item, dict):
                    raise ValueError(f'prepared_positions[{index}] must be an object')
                name = str(item.get('name', '') or '').strip()
                if not name:
                    raise ValueError(f'prepared_positions[{index}].name is empty')
                if name in waypoints:
                    raise ValueError(f'duplicate waypoint name: {name}')
                positions = item.get('position')
                if (
                    not isinstance(positions, list)
                    or not positions
                    or not isinstance(positions[0], list)
                    or len(positions[0]) < 3
                ):
                    raise ValueError(
                        f'prepared_positions[{index}].position must contain [x, y, yaw_deg]'
                    )
                x, y, yaw_degrees = (float(value) for value in positions[0][:3])
                if not all(math.isfinite(value) for value in (x, y, yaw_degrees)):
                    raise ValueError(f'waypoint {name} contains a non-finite value')
                waypoints[name] = {
                    'frame_id': self._map_frame,
                    'x': x,
                    'y': y,
                    'yaw': math.radians(yaw_degrees),
                    'description': str(item.get('description', '') or ''),
                }
        except Exception as exc:
            self.get_logger().error(f'Failed to read canonical waypoint index {index_path}: {exc}')
            raise
        return waypoints

    def _load_waypoints_legacy_do_not_use(self, path=None) -> Dict[str, Dict[str, float]]:
        raise RuntimeError('Waypoint YAML is write-only; read maps_index.json instead')
        waypoint_path = Path(path or self._waypoints_file)
        if not waypoint_path.exists():
            return {}

        try:
            content = yaml.safe_load(
                waypoint_path.read_text(encoding='utf-8')
            ) or {}
            waypoints = content.get('waypoints', {})
            if not isinstance(waypoints, dict):
                raise ValueError('waypoints 必须是字典')
        except Exception as exc:
            self.get_logger().error(f'Failed to read waypoints file: {exc}')
            raise

        return waypoints

    def _write_waypoints(self) -> None:
        self._waypoints_file.parent.mkdir(parents=True, exist_ok=True)
        payload = {'waypoints': self._persistent_waypoints()}
        if self._active_map_yaml is not None:
            payload['map_yaml'] = str(self._active_map_yaml)
        fd, temporary_name = tempfile.mkstemp(
            prefix=f'.{self._waypoints_file.name}.',
            suffix='.tmp',
            dir=str(self._waypoints_file.parent),
        )
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                yaml.safe_dump(
                    payload, stream, sort_keys=True, allow_unicode=True
                )
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, self._waypoints_file)
        finally:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)

        self._write_vendor_map_index()

    def _write_vendor_map_index(self) -> None:
        """Write a vendor-compatible waypoint index beside the YAML library."""
        if self._active_map_yaml is None:
            return
        map_name = self._active_map_yaml.stem
        prepared_positions = []
        for name in self._public_waypoint_names():
            waypoint = self._waypoints[name]
            prepared_positions.append({
                'name': name,
                'description': str(waypoint.get('description', '') or ''),
                'position': [[
                    round(float(waypoint['x']), 6),
                    round(float(waypoint['y']), 6),
                    round(math.degrees(float(waypoint['yaw'])), 6),
                ]],
            })
        charger = self._waypoints.get(self._charger_waypoint_name)
        dock_pose = []
        if charger is not None:
            dock_pose = [
                round(float(charger['x']), 6),
                round(float(charger['y']), 6),
                round(math.degrees(float(charger['yaw'])), 6),
            ]
        image_path = ''
        try:
            map_payload = yaml.safe_load(
                self._active_map_yaml.read_text(encoding='utf-8')
            ) or {}
            image_value = str(map_payload.get('image', '') or '').strip()
            if image_value:
                candidate = Path(image_value).expanduser()
                if not candidate.is_absolute():
                    candidate = self._active_map_yaml.parent / candidate
                image_path = str(candidate.resolve())
        except Exception as error:
            self.get_logger().warning(f'读取地图图像路径失败: {error}')
        index_payload = {
            'active_map': map_name,
            'maps': {
                map_name: {
                    'yaml_path': str(self._active_map_yaml),
                    'png_path': image_path,
                    'description': '',
                    'dock_pose': dock_pose,
                    'prepared_positions': prepared_positions,
                }
            },
        }
        index_path = self._waypoint_index_path()
        fd, temporary_name = tempfile.mkstemp(
            prefix='.maps_index.', suffix='.tmp',
            dir=str(index_path.parent),
        )
        try:
            with os.fdopen(fd, 'w', encoding='utf-8') as stream:
                json.dump(index_payload, stream, ensure_ascii=False, indent=4)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary_name, index_path)
        finally:
            if os.path.exists(temporary_name):
                os.unlink(temporary_name)

    def _waypoint_file_for_map(self, map_yaml):
        map_path = Path(str(map_yaml)).expanduser().resolve()
        if not map_path.is_file() or map_path.suffix.lower() != '.yaml':
            raise FileNotFoundError(f'地图 YAML 不存在：{map_path}')
        safe_stem = re.sub(
            r'[^\w\u4e00-\u9fff-]+', '_', map_path.stem
        ).strip('_')
        if not safe_stem:
            raise ValueError('无法为地图生成航点文件名')
        map_directory = self._waypoint_library_directory / safe_stem
        map_directory.mkdir(parents=True, exist_ok=True)
        return map_path, map_directory / f'{safe_stem}.yaml'

    def _handle_switch_waypoint_map(self, request, response):
        if (
            self._active_goal_name is not None
            or self._active_sequence
            or self._sequence_dwell_timer is not None
            or self._navigation_paused
            or self._precise_docking_active
        ):
            response.success = False
            response.message = '导航仍在运行，拒绝切换地图航点库'
            response.waypoints_file = str(self._waypoints_file)
            response.names = self._public_waypoint_names()
            return response
        try:
            map_path, waypoint_path = self._waypoint_file_for_map(
                request.map_yaml
            )
            self._active_map_yaml = map_path
            self._waypoints_file = waypoint_path
            index_path = self._waypoint_index_path()
            if index_path.exists():
                self._waypoints = self._load_waypoints_from_index()
            else:
                self.get_logger().info(
                    f'No waypoint index for {map_path.name}; creating an empty library'
                )
                self._waypoints = {}
            self._write_waypoints()
            self._completed_waypoints.clear()
            self._failed_goal_name = None
            self._publish_waypoint_markers()
            self._publish_navigation_status(
                'idle', f'waypoint library switched: {map_path.name}'
            )
            response.success = True
            response.message = (
                f'已切换到地图 {map_path.name} 的航点库'
            )
            response.waypoints_file = str(waypoint_path)
            response.names = self._public_waypoint_names()
            self.get_logger().info(
                f'Waypoint map switched: {map_path} -> {waypoint_path}'
            )
        except Exception as error:
            response.success = False
            response.message = f'切换航点库失败：{error}'
            response.waypoints_file = str(self._waypoints_file)
            response.names = self._public_waypoint_names()
        return response

    def _lookup_current_pose(self):
        transform = self._tf_buffer.lookup_transform(
            self._map_frame,
            self._robot_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=1.0),
        )
        translation = transform.transform.translation
        rotation = transform.transform.rotation
        yaw = self._yaw_from_quaternion(rotation)
        return translation.x, translation.y, yaw

    def _navigation_target(self, name: str):
        waypoint = dict(self._waypoints[name])
        if getattr(self, '_dock_staging_active', False) and name == self._charger_waypoint_name:
            # Saved pose is the robot pose WHILE CHARGING. Stand in front,
            # facing away from the dock, then reverse. Never edit the library.
            yaw = float(waypoint['yaw'])
            waypoint['x'] = float(waypoint['x']) + self._dock_staging_distance * math.cos(yaw)
            waypoint['y'] = float(waypoint['y']) + self._dock_staging_distance * math.sin(yaw)
        return waypoint

    def _build_goal(self, name: str) -> NavigateToPose.Goal:
        waypoint = self._navigation_target(name)
        goal = NavigateToPose.Goal()
        goal.pose = PoseStamped()
        goal.pose.header.frame_id = waypoint.get('frame_id', self._map_frame)
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(waypoint['x'])
        goal.pose.pose.position.y = float(waypoint['y'])
        goal.pose.pose.orientation = self._quaternion_from_yaw(float(waypoint['yaw']))
        return goal

    def _handle_short_costmap(self, msg: Costmap) -> None:
        self._short_costmap = msg
        self._short_costmap_received_time = self.get_clock().now()

    def _pose_in_frame(self, pose, target_frame: str):
        source_frame = str(pose.get('frame_id', self._map_frame))
        x = float(pose['x'])
        y = float(pose['y'])
        yaw = float(pose.get('yaw', 0.0))
        if source_frame == target_frame:
            return x, y, yaw

        transform = self._tf_buffer.lookup_transform(
            target_frame,
            source_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.2),
        )
        transform_yaw = self._yaw_from_quaternion(transform.transform.rotation)
        cos_yaw = math.cos(transform_yaw)
        sin_yaw = math.sin(transform_yaw)
        return (
            float(transform.transform.translation.x) + cos_yaw * x - sin_yaw * y,
            float(transform.transform.translation.y) + sin_yaw * x + cos_yaw * y,
            self._normalize_angle(transform_yaw + yaw),
        )

    def _robot_pose_in_frame(self, target_frame: str):
        transform = self._tf_buffer.lookup_transform(
            target_frame,
            self._robot_frame,
            rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.2),
        )
        stamp = rclpy.time.Time.from_msg(transform.header.stamp)
        age = (self.get_clock().now() - stamp).nanoseconds / 1e9
        if age > 0.75 or age < -0.6:
            raise TransformException(f'{target_frame} robot pose timestamp invalid: age={age:.3f}s')
        return (
            float(transform.transform.translation.x),
            float(transform.transform.translation.y),
            self._yaw_from_quaternion(transform.transform.rotation),
        )

    def _short_path_is_clear(self, target):
        costmap = self._short_costmap
        received_time = self._short_costmap_received_time
        if costmap is None or received_time is None:
            return False, 'local costmap is not available'
        age = (self.get_clock().now() - received_time).nanoseconds / 1e9
        if age > self._short_costmap_timeout:
            return False, f'local costmap is stale ({age:.2f}s)'

        frame_id = str(costmap.header.frame_id or '').strip()
        if not frame_id:
            return False, 'local costmap frame is empty'
        try:
            start_pose = self._robot_pose_in_frame(frame_id)
            target_pose = self._pose_in_frame(target, frame_id)
        except TransformException as exc:
            return False, f'costmap transform unavailable: {exc}'

        metadata = costmap.metadata
        origin = metadata.origin
        origin_pose = (
            float(origin.position.x),
            float(origin.position.y),
            self._yaw_from_quaternion(origin.orientation),
        )
        clear = corridor_is_clear(
            costmap.data,
            int(metadata.size_x),
            int(metadata.size_y),
            float(metadata.resolution),
            origin_pose,
            start_pose,
            target_pose,
            half_length=self._short_half_length,
            half_width=self._short_half_width,
            front_extension=self._short_front_extension,
            lethal_cost=self._short_lethal_cost,
        )
        if clear:
            return True, 'straight swept corridor is clear'
        endpoint_clear = corridor_is_clear(
            costmap.data, int(metadata.size_x), int(metadata.size_y),
            float(metadata.resolution), origin_pose, target_pose, target_pose,
            half_length=self._short_half_length, half_width=self._short_half_width,
            front_extension=self._short_front_extension, lethal_cost=self._short_lethal_cost,
        )
        if not endpoint_clear:
            return False, '本段终点碰撞轮廓被占用或超出局部地图；请让路，固定障碍需调整航点'
        return False, '本段移动扫掠路径受阻，终点轮廓可用'

    def _choose_short_stage(self, error_x: float, error_y: float) -> str:
        if abs(error_y) > self._short_axis_tolerance:
            return 'lateral'
        if abs(error_x) > self._short_axis_tolerance:
            return 'longitudinal'
        return 'yaw'

    def _short_stage_target(
        self,
        current_pose,
        target_pose,
        error_x: float,
        error_y: float,
        stage: str,
    ):
        current_x, current_y, current_yaw = current_pose
        if stage == 'lateral':
            return (
                current_x - math.sin(current_yaw) * error_y,
                current_y + math.cos(current_yaw) * error_y,
                current_yaw,
            )
        if stage == 'longitudinal':
            return (
                current_x + math.cos(current_yaw) * error_x,
                current_y + math.sin(current_yaw) * error_x,
                current_yaw,
            )
        return current_x, current_y, target_pose[2]

    def _short_stage_path_is_clear(
        self,
        current_pose,
        target_pose,
        error_x: float,
        error_y: float,
        stage: str,
    ):
        stage_target = self._short_stage_target(
            current_pose,
            target_pose,
            error_x,
            error_y,
            stage,
        )
        return self._short_path_is_clear(
            {
                'frame_id': self._map_frame,
                'x': stage_target[0],
                'y': stage_target[1],
                'yaw': stage_target[2],
            }
        )

    def _change_short_stage(self, stage: str, now, *, settle: bool = True) -> None:
        previous_stage = self._short_stage
        self._short_stage = stage
        self._short_translation_locked = stage == 'yaw'
        self._short_stage_ready_since = now if settle else None
        self._short_stable_since = None
        self._short_best_distance = None
        self._short_last_progress_time = now
        self._short_last_path_check_time = None
        self._short_last_command = (0.0, 0.0, 0.0)
        self._short_final_verify_since = None
        self._short_cmd_vel_pub.publish(Twist())
        if previous_stage != stage:
            stage_names = {
                'lateral': '左右平移',
                'longitudinal': '前后平移',
                'yaw': '最终朝向',
            }
            self.get_logger().info(
                f'短距离伺服阶段切换：{stage_names.get(stage, stage)}'
            )

    def _try_start_short_distance(
        self,
        name: str,
        announce_departure: bool = True,
        final_refine: bool = False,
    ):
        if not self._short_servo_enabled or self._short_distance_threshold <= 0.0:
            return False, None

        # A direct velocity servo must never share the output topic with an
        # active or not-yet-accepted Nav2/Dock goal. In that case keep using
        # the normal action path, whose server owns preemption.
        action_request_pending = any(
            not future.done() for future in self._goal_futures
        )
        action_output_active = any(
            handle is not None
            for handle in (
                self._active_goal_handle,
                self._active_undock_goal_handle,
                self._active_dock_goal_handle,
            )
        )
        if action_request_pending or action_output_active:
            self.get_logger().info(
                '已有 Nav2 或入离桩动作占用速度输出，本次目标继续使用普通 Nav2'
            )
            return False, None

        target = self._navigation_target(name)
        try:
            current_pose = self._robot_pose_in_frame(self._map_frame)
            current_odom_pose = self._robot_pose_in_frame(self._odom_frame)
            target_map_pose = self._pose_in_frame(target, self._map_frame)
        except TransformException as exc:
            self.get_logger().warning(
                f'短距离模式无法读取目标位姿，将使用普通 Nav2: {exc}'
            )
            return False, None

        distance = math.hypot(
            target_map_pose[0] - current_pose[0],
            target_map_pose[1] - current_pose[1],
        )
        if distance > self._short_distance_threshold:
            return False, distance

        error_x, error_y, _ = body_frame_error(current_pose, target_map_pose)
        target_odom_pose = reproject_target_pose(
            current_odom_pose,
            current_pose,
            target_map_pose,
        )
        initial_stage = self._choose_short_stage(error_x, error_y)
        path_clear, reason = self._short_stage_path_is_clear(
            current_pose,
            target_map_pose,
            error_x,
            error_y,
            initial_stage,
        )
        if not path_clear and not final_refine:
            self.get_logger().warning(
                f'点位 {name} 距离 {distance:.3f} m，但短距当前阶段路径不可用：{reason}；'
                '回退普通 Nav2'
            )
            self._publish_navigation_status(
                'short_distance_fallback',
                f'short-distance path unavailable, falling back to Nav2: {reason}',
                waypoint_name=name,
                distance_remaining=distance,
            )
            return False, distance

        now = self.get_clock().now()
        self._active_goal_id += 1
        self._active_goal_name = name
        self._active_goal_handle = None
        self._active_goal_start_time = now
        self._goal_reached_since = None
        self._last_arrival_log_time = None
        self._failed_goal_name = None
        self._last_feedback = None
        self._navigation_paused = False
        self._paused_goal_name = None
        self._short_servo_active = True
        self._short_final_refine = final_refine
        self._short_blocked_since = None
        self._short_clear_since = None
        self._short_target = target
        self._short_target_map_pose = target_map_pose
        self._short_odom_target = target_odom_pose
        self._short_translation_locked = initial_stage == 'yaw'
        self._short_stable_since = None
        self._short_best_distance = distance
        self._short_last_progress_time = now
        self._short_last_path_check_time = now
        self._short_last_status_time = None
        self._short_last_command = (0.0, 0.0, 0.0)
        self._short_arrival_since = None
        self._short_stage = initial_stage
        self._short_stage_ready_since = None
        self._short_correction_pass = 0
        self._short_final_verify_since = None
        self._short_cmd_vel_pub.publish(Twist())
        self._publish_waypoint_markers()
        self._publish_navigation_status(
            'short_distance_started',
            f'started staged short-distance pose servo to {name}',
            waypoint_name=name,
            distance_remaining=distance,
        )
        if announce_departure:
            self._publish_voice_prompt(self._voice_departure_template, name)
        if not path_clear:
            self._wait_for_refine_corridor(False, reason, now)
        self.get_logger().info(
            f'点位 {name} 自动进入短距离位姿伺服：distance={distance:.3f} m，'
            f'首阶段={initial_stage}，分轴路径可用'
        )
        return True, distance

    def _deactivate_short_distance(self) -> None:
        self._short_arrival_since = None
        if self._short_servo_active:
            self._short_cmd_vel_pub.publish(Twist())
        self._short_servo_active = False
        self._short_final_refine = False
        self._short_blocked_since = None
        self._short_clear_since = None
        self._short_target = None
        self._short_target_map_pose = None
        self._short_odom_target = None
        self._short_translation_locked = False
        self._short_stable_since = None
        self._short_best_distance = None
        self._short_last_progress_time = None
        self._short_last_path_check_time = None
        self._short_last_status_time = None
        self._short_last_command = (0.0, 0.0, 0.0)
        self._short_stage = None
        self._short_stage_ready_since = None
        self._short_correction_pass = 0
        self._short_final_verify_since = None

    def _fallback_short_distance_to_nav2(self, reason: str) -> None:
        goal_name = self._active_goal_name
        if not self._short_servo_active or not goal_name:
            return
        # A final correction must not repeatedly restart Nav2 at the same pose.
        # Preserve the failed waypoint for the existing VR retry/skip controls.
        if self._short_final_refine:
            self._deactivate_short_distance()
            self._active_goal_id += 1
            self._active_goal_name = None
            self._active_goal_start_time = None
            self._failed_goal_name = goal_name
            self._publish_waypoint_markers()
            self._publish_navigation_status(
                'failed', f'末端精调已停车，目标保持不变：{reason}',
                waypoint_name=goal_name,
            )
            self.get_logger().warning(f'Final refinement stopped at {goal_name}: {reason}')
            self._stop_sequence()
            return
        self.get_logger().warning(
            f'短距离伺服停止并回退普通 Nav2：目标={goal_name}，原因={reason}'
        )
        self._deactivate_short_distance()
        self._active_goal_name = None
        self._active_goal_start_time = None
        self._goal_reached_since = None
        self._publish_navigation_status(
            'short_distance_fallback',
            f'short-distance servo stopped, falling back to Nav2: {reason}',
            waypoint_name=goal_name,
        )
        started, message = self._start_navigation(
            goal_name,
            skip_undock=True,
            force_normal=True,
        )
        if started:
            return
        self._failed_goal_name = goal_name
        self._publish_waypoint_markers()
        self._publish_navigation_status(
            'failed',
            f'short-distance fallback failed for {goal_name}: {message}',
            waypoint_name=goal_name,
        )
        self._stop_sequence()

    def _complete_short_distance(self, distance: float, yaw_error: float) -> None:
        goal_name = self._active_goal_name
        if not self._short_servo_active or not goal_name:
            return
        if getattr(self, '_dock_staging_active', False) and goal_name == self._charger_waypoint_name:
            # Nav2 has terminated before the servo starts; stop the servo before
            # giving DockRobot exclusive output ownership. Not an arrival yet.
            error = self._waypoint_pose_error(goal_name)
            if error is None or error[0] > self._short_xy_tolerance or error[1] > self._short_yaw_tolerance:
                self._fallback_short_distance_to_nav2('入桩预备位复核未通过')
                return
            self._deactivate_short_distance()
            started, message = self._start_precise_docking(goal_name, staging_aligned=True)
            if not started:
                self._fail_precise_docking(goal_name, message)
            return
        self._deactivate_short_distance()
        self._active_goal_id += 1
        self._active_goal_name = None
        self._active_goal_start_time = None
        self._goal_reached_since = None
        self._failed_goal_name = None
        self._completed_waypoints.add(goal_name)
        self._publish_waypoint_markers()
        self._publish_navigation_status(
            'short_distance_succeeded',
            f'arrived at waypoint {goal_name} by staged pose servo',
            waypoint_name=goal_name,
            distance_remaining=distance,
        )
        self._publish_voice_prompt(self._voice_arrival_template, goal_name)
        self.get_logger().info(
            f'短距离位姿伺服到达 {goal_name}：xy_error={distance:.3f} m，'
            f'yaw_error={abs(yaw_error):.3f} rad'
        )
        self._advance_sequence_after_arrival(goal_name)

    def _wait_for_refine_corridor(self, clear, reason, now) -> bool:
        """True means HOLD. Never extend the total servo time or advance tasks."""
        if not clear:
            self._short_block_reason = reason
            if self._short_blocked_since is None:
                self._short_blocked_since = now
                self.get_logger().warning(f'Final refinement waiting: {reason}')
            self._short_clear_since = None
        elif self._short_blocked_since is None:
            return False
        elif self._short_clear_since is None:
            self._short_clear_since = now
        elif (now - self._short_clear_since).nanoseconds / 1e9 >= self._refine_clear_settle:
            self._short_blocked_since = None
            self._short_clear_since = None
            self._short_best_distance = None
            self._short_last_progress_time = now
            self.get_logger().info('Final refinement corridor stable; continuing the SAME waypoint')
            return False
        self._short_cmd_vel_pub.publish(Twist())
        self._short_last_command = (0.0, 0.0, 0.0)
        self._short_last_progress_time = now
        self._short_stable_since = None
        self._short_final_verify_since = None
        if (now - self._short_blocked_since).nanoseconds / 1e9 >= self._refine_obstacle_wait:
            self._fallback_short_distance_to_nav2(
                '等待通路超时：' + getattr(self, '_short_block_reason', '通路受阻或感知未恢复')
            )
        elif self._short_last_status_time is None or (now - self._short_last_status_time).nanoseconds / 1e9 >= 0.5:
            self._short_last_status_time = now
            self._publish_navigation_status(
                'short_distance_waiting', '目标附近通路受阻，已停车等待，通路稳定后继续当前点位',
                waypoint_name=self._active_goal_name,
            )
        return True

    def _settle_final_refine(self, now, distance, yaw_error, map_distance, map_yaw_error):
        # Stop on the actual radial arrival tolerances, not tighter per-axis
        # staging thresholds. Require odom AND map to remain in tolerance;
        # one lucky AMCL sample or a blocked interval cannot complete a goal.
        inside = (
            distance <= self._short_xy_tolerance
            and map_distance <= self._short_xy_tolerance
            and abs(yaw_error) <= self._short_yaw_tolerance
            and abs(map_yaw_error) <= self._short_yaw_tolerance
        )
        if not inside:
            self._short_arrival_since = None
            return False
        self._short_cmd_vel_pub.publish(Twist())
        self._short_last_command = (0.0, 0.0, 0.0)
        if getattr(self, '_short_arrival_since', None) is None:
            self._short_arrival_since = now
        stable = (now - self._short_arrival_since).nanoseconds / 1e9
        if stable >= max(self._short_settle_time, self._short_final_map_settle_time):
            self._complete_short_distance(map_distance, map_yaw_error)
        return True

    def _run_short_distance_servo(self) -> None:
        if not self._short_servo_active:
            return
        goal_name = self._active_goal_name
        target = self._short_target
        if not goal_name or target is None:
            self._deactivate_short_distance()
            return

        now = self.get_clock().now()
        elapsed = (now - self._active_goal_start_time).nanoseconds / 1e9
        if elapsed > self._short_max_duration:
            self._fallback_short_distance_to_nav2(
                f'servo exceeded {self._short_max_duration:.1f}s'
            )
            return

        target_map_pose = self._short_target_map_pose
        odom_target_pose = self._short_odom_target
        if target_map_pose is None or odom_target_pose is None:
            self._fallback_short_distance_to_nav2('short-distance reference pose is missing')
            return

        try:
            current_odom_pose = self._robot_pose_in_frame(self._odom_frame)
            current_map_pose = self._robot_pose_in_frame(self._map_frame)
        except TransformException as exc:
            self._short_cmd_vel_pub.publish(Twist())
            self._short_last_command = (0.0, 0.0, 0.0)
            self._fallback_short_distance_to_nav2(f'pose transform unavailable: {exc}')
            return

        # Odom is the high-rate, continuous inner-loop reference.  AMCL only
        # moves the odom target through a tightly bounded correction, so a
        # map->odom scan-match jump cannot become an abrupt chassis command.
        desired_odom_target = reproject_target_pose(
            current_odom_pose,
            current_map_pose,
            target_map_pose,
        )
        self._short_odom_target = bounded_pose_update(
            odom_target_pose,
            desired_odom_target,
            gain=self._short_map_correction_gain,
            max_xy_step=self._short_max_map_xy_correction_step,
            max_yaw_step=self._short_max_map_yaw_correction_step,
        )
        odom_target_pose = self._short_odom_target

        error_x, error_y, yaw_error = body_frame_error(
            current_odom_pose,
            odom_target_pose,
        )
        distance = math.hypot(error_x, error_y)
        map_error_x, map_error_y, map_yaw_error = body_frame_error(
            current_map_pose,
            target_map_pose,
        )
        map_distance = math.hypot(map_error_x, map_error_y)

        stage = self._short_stage or self._choose_short_stage(error_x, error_y)
        if self._short_stage is None:
            self._change_short_stage(stage, now, settle=False)

        # Check before progress/arrival decisions. Keep checking while stopped;
        # require a continuous clear interval rather than restarting on one scan.
        if self._short_final_refine:
            if self._short_last_path_check_time is None or (now - self._short_last_path_check_time).nanoseconds / 1e9 >= self._short_path_check_period:
                self._short_last_path_check_time = now
                clear, reason = self._short_stage_path_is_clear(
                    current_map_pose, target_map_pose, map_error_x, map_error_y, stage,
                )
                if self._wait_for_refine_corridor(clear, reason, now):
                    self._short_arrival_since = None
                    return
            elif self._short_blocked_since is not None:
                self._short_arrival_since = None
                self._short_cmd_vel_pub.publish(Twist())
                return

            if self._settle_final_refine(now, distance, yaw_error, map_distance, map_yaw_error):
                return
            # Position already meets the requested precision: hold translation
            # and finish yaw instead of alternating axes to chase 2 cm noise.
            if (stage != 'yaw' and distance <= self._short_xy_tolerance
                    and map_distance <= self._short_xy_tolerance):
                self._change_short_stage('yaw', now)
                return

        # Stop completely between axes so chassis inertia from one degree of
        # freedom cannot leak into the next one.
        if self._short_stage_ready_since is not None:
            self._short_cmd_vel_pub.publish(Twist())
            settled = (
                now - self._short_stage_ready_since
            ).nanoseconds / 1e9
            if settled < self._short_stage_settle_time:
                return
            self._short_stage_ready_since = None

        stage = self._short_stage
        if stage == 'lateral' and abs(error_y) <= self._short_axis_tolerance:
            next_stage = (
                'longitudinal'
                if abs(error_x) > self._short_axis_tolerance
                else 'yaw'
            )
            self._change_short_stage(next_stage, now)
            return
        if stage == 'longitudinal' and abs(error_x) <= self._short_axis_tolerance:
            next_stage = (
                'lateral'
                if abs(error_y) > self._short_axis_tolerance
                else 'yaw'
            )
            self._change_short_stage(next_stage, now)
            return

        if stage in ('lateral', 'longitudinal'):
            progress_error = abs(error_y) if stage == 'lateral' else abs(error_x)
            if (
                self._short_best_distance is None
                or progress_error
                <= self._short_best_distance - self._short_progress_epsilon
            ):
                self._short_best_distance = progress_error
                self._short_last_progress_time = now
            elif (
                self._short_last_progress_time is not None
                and (now - self._short_last_progress_time).nanoseconds / 1e9
                > self._short_progress_timeout
            ):
                self._fallback_short_distance_to_nav2(
                    f'no {stage} progress for {self._short_progress_timeout:.1f}s'
                )
                return

        if not self._short_final_refine and (
            self._short_last_path_check_time is None
            or (now - self._short_last_path_check_time).nanoseconds / 1e9
            >= self._short_path_check_period
        ):
            self._short_last_path_check_time = now
            path_clear, reason = self._short_stage_path_is_clear(
                current_map_pose,
                target_map_pose,
                map_error_x,
                map_error_y,
                stage,
            )
            if not path_clear:
                self._fallback_short_distance_to_nav2(reason)
                return

        if stage == 'yaw':
            if self._short_stable_since is None:
                if abs(yaw_error) <= self._short_yaw_tolerance:
                    self._short_stable_since = now
            elif abs(yaw_error) > self._short_yaw_exit_tolerance:
                self._short_stable_since = None
                self._short_final_verify_since = None
            if self._short_stable_since is not None:
                stable_time = (now - self._short_stable_since).nanoseconds / 1e9
                if stable_time >= self._short_settle_time:
                    self._short_cmd_vel_pub.publish(Twist())
                    self._short_last_command = (0.0, 0.0, 0.0)
                    if self._short_final_verify_since is None:
                        self._short_final_verify_since = now
                        return
                    final_settle = (
                        now - self._short_final_verify_since
                    ).nanoseconds / 1e9
                    if final_settle < self._short_final_map_settle_time:
                        return
                    if (
                        map_distance <= self._short_xy_tolerance
                        and abs(map_yaw_error) <= self._short_yaw_tolerance
                        and not self._short_final_refine
                    ):
                        self._complete_short_distance(map_distance, map_yaw_error)
                        return
                    self._short_correction_pass += 1
                    if (
                        self._short_correction_pass
                        > self._short_max_final_correction_passes
                    ):
                        self._fallback_short_distance_to_nav2(
                            'final AMCL verification did not converge after '
                            f'{self._short_max_final_correction_passes} correction passes; '
                            f'xy={map_distance:.3f}m yaw={abs(map_yaw_error):.3f}rad'
                        )
                        return
                    self._short_odom_target = desired_odom_target
                    correction_error_x, correction_error_y, _ = body_frame_error(
                        current_odom_pose,
                        self._short_odom_target,
                    )
                    correction_stage = self._choose_short_stage(
                        correction_error_x,
                        correction_error_y,
                    )
                    self.get_logger().warning(
                        f'最终转向造成位置漂移 {distance:.3f} m，'
                        f'开始第 {self._short_correction_pass} 次分轴位置修正'
                    )
                    self._change_short_stage(correction_stage, now)
                    return

        desired_x = 0.0
        desired_y = 0.0
        desired_yaw = 0.0
        if stage == 'lateral':
            desired_y = axis_command(
                error_y,
                gain=self._short_xy_gain,
                maximum_speed=self._short_max_linear_speed,
                minimum_speed=self._short_min_linear_speed,
                tolerance=self._short_axis_tolerance,
            )
        elif stage == 'longitudinal':
            desired_x = axis_command(
                error_x,
                gain=self._short_xy_gain,
                maximum_speed=self._short_max_linear_speed,
                minimum_speed=self._short_min_linear_speed,
                tolerance=self._short_axis_tolerance,
            )
        else:
            desired_yaw = axis_command(
                yaw_error,
                gain=self._short_yaw_gain,
                maximum_speed=self._short_max_angular_speed,
                minimum_speed=self._short_min_angular_speed,
                tolerance=self._short_yaw_tolerance,
            )

        previous_x, previous_y, previous_yaw = self._short_last_command
        linear_delta = self._short_max_linear_acceleration * self._short_servo_period
        if stage == 'longitudinal':
            command_x = rate_limit(previous_x, desired_x, linear_delta)
            command_y = 0.0
            command_yaw = 0.0
        elif stage == 'lateral':
            command_x = 0.0
            command_y = rate_limit(previous_y, desired_y, linear_delta)
            command_yaw = 0.0
        else:
            command_x = 0.0
            command_y = 0.0
            command_yaw = rate_limit(
                previous_yaw,
                desired_yaw,
                self._short_max_angular_acceleration * self._short_servo_period,
            )
        self._short_last_command = (command_x, command_y, command_yaw)

        command = Twist()
        command.linear.x = command_x
        command.linear.y = command_y
        command.angular.z = command_yaw
        self._short_cmd_vel_pub.publish(command)

        if (
            self._short_last_status_time is None
            or (now - self._short_last_status_time).nanoseconds / 1e9 >= 0.25
        ):
            self._short_last_status_time = now
            state = f'short_distance_{stage}'
            self._publish_navigation_status(
                state,
                f'staged {stage} pose servo to {goal_name}',
                waypoint_name=goal_name,
                distance_remaining=map_distance,
                navigation_time=elapsed,
            )

    def _format_voice_prompt(self, template: str, waypoint_name: str) -> str:
        try:
            return template.format(waypoint=waypoint_name, name=waypoint_name).strip()
        except Exception as exc:
            self.get_logger().warning(f'语音提示词格式错误: {exc}')
            return ''

    def _publish_voice_prompt(self, template: str, waypoint_name: str) -> None:
        if not self._voice_feedback_enabled:
            return

        text = self._format_voice_prompt(
            template, self._display_goal_name(waypoint_name)
        )
        if not text:
            return

        msg = String()
        msg.data = text
        self._voice_pub.publish(msg)
        self.get_logger().info(f'已发布语音提示: {text}')

    def _should_auto_undock(self, target_name: str) -> bool:
        if not self._auto_undock_enabled or target_name == self._charger_waypoint_name:
            return False

        charger = self._waypoints.get(self._charger_waypoint_name)
        if charger is None:
            return False

        try:
            x, y, yaw = self._lookup_current_pose()
        except TransformException as exc:
            self.get_logger().warning(f'自动离桩位姿查询失败，将直接尝试导航: {exc}')
            return False

        distance = math.hypot(x - float(charger['x']), y - float(charger['y']))
        yaw_error = abs(self._normalize_angle(yaw - float(charger.get('yaw', 0.0))))
        if distance > self._undock_trigger_distance:
            return False
        if yaw_error > self._undock_yaw_tolerance:
            self.get_logger().warning(
                f'机器人位于充电桩附近，但朝向偏差 {yaw_error:.3f} rad 超过 '
                f'{self._undock_yaw_tolerance:.3f} rad，不执行自动前移离桩'
            )
            return False

        self.get_logger().info(
            f'检测到机器人位于充电桩附近: distance={distance:.3f} m, '
            f'yaw_error={yaw_error:.3f} rad，准备自动离桩'
        )
        return True

    def _start_automatic_undock(self, target_name: str) -> (bool, str):
        if not self._undock_client.wait_for_server(timeout_sec=2.0):
            return False, 'drive_on_heading action server 未就绪，无法自动离桩'

        goal = DriveOnHeading.Goal()
        goal.target.x = float(self._undock_distance)
        goal.speed = float(self._undock_speed)
        timeout_seconds = max(1.0, float(self._undock_timeout))
        goal.time_allowance.sec = int(timeout_seconds)
        goal.time_allowance.nanosec = int((timeout_seconds % 1.0) * 1e9)

        self._active_goal_id += 1
        goal_id = self._active_goal_id
        self._active_goal_name = target_name
        self._active_goal_handle = None
        self._active_undock_goal_handle = None
        self._undock_pending_goal_name = target_name
        self._last_undock_feedback_log_time = None
        self._navigation_paused = False
        self._paused_goal_name = None
        self._goal_reached_since = None
        self._failed_goal_name = None

        future = self._undock_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback_msg, goal_name=target_name, sent_goal_id=goal_id:
            self._handle_undock_feedback(feedback_msg, goal_name, sent_goal_id),
        )
        future.add_done_callback(
            lambda response_future, goal_name=target_name, sent_goal_id=goal_id:
            self._handle_undock_goal_response(response_future, goal_name, sent_goal_id)
        )
        self._goal_futures.append(future)
        self._publish_waypoint_markers()
        self._publish_navigation_status(
            'undocking',
            f'leaving charger before navigating to {target_name}',
            waypoint_name=target_name,
        )
        return True, f'机器人正在自动离开充电桩，随后前往 {target_name}'

    def _handle_undock_feedback(self, feedback_msg, target_name: str, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            return
        now = self.get_clock().now()
        if self._last_undock_feedback_log_time is not None:
            elapsed = (now - self._last_undock_feedback_log_time).nanoseconds / 1e9
            if elapsed < 1.0:
                return
        self._last_undock_feedback_log_time = now
        distance = float(feedback_msg.feedback.distance_traveled)
        self.get_logger().info(
            f'自动离桩进度: {distance:.3f}/{self._undock_distance:.3f} m, '
            f'随后前往 {target_name}'
        )

    def _start_precise_docking(self, name: str, staging_aligned: bool = False) -> (bool, str):
        if not self._battery_state_fresh():
            return False, '电池状态过期，已停止自动入桩，请检查电池反馈'
        if self._battery_is_charging:
            return False, '机器人当前已在充电，无需重复入桩'
        if not self._dock_client.wait_for_server(timeout_sec=3.0):
            return False, 'dock_robot action server 未就绪，无法执行精确入桩'

        waypoint = self._waypoints[name]
        goal = DockRobot.Goal()
        goal.use_dock_id = False
        goal.dock_pose.header.frame_id = waypoint.get('frame_id', self._map_frame)
        goal.dock_pose.header.stamp = self.get_clock().now().to_msg()
        goal.dock_pose.pose.position.x = float(waypoint['x'])
        goal.dock_pose.pose.position.y = float(waypoint['y'])
        self._dock_target_odom_pose = None
        target_yaw = float(waypoint['yaw'])
        if staging_aligned:
            try:
                current_map = self._robot_pose_in_frame(self._map_frame)
                current_odom = self._robot_pose_in_frame(self._odom_frame)
                contact_map = self._pose_in_frame(waypoint, self._map_frame)
                contact_odom = reproject_target_pose(current_odom, current_map, contact_map)
            except TransformException as exc:
                return False, f'入桩里程计基准不可用，已停车: {exc}'
            # Freeze only the final 55 cm in odom. AMCL jumps must not move
            # the contact target sideways while the robot is reversing.
            self._dock_target_odom_pose = contact_odom
            goal.dock_pose.header.frame_id = self._odom_frame
            goal.dock_pose.pose.position.x, goal.dock_pose.pose.position.y = contact_odom[:2]
            target_yaw = contact_odom[2]
        # The saved waypoint yaw is the robot's final heading. For rear docking,
        # OpenNav expects the dock frame to point in the opposite direction.
        dock_yaw = self._normalize_angle(
            target_yaw + float(self._dock_pose_yaw_offset)
        )
        goal.dock_pose.pose.orientation = self._quaternion_from_yaw(dock_yaw)
        goal.dock_type = str(self._dock_type)
        goal.max_staging_time = float(self._dock_max_staging_time)
        goal.navigate_to_staging_pose = False if staging_aligned else bool(self._dock_navigate_to_staging_pose)
        self._dock_staging_active = False

        self._active_goal_id += 1
        goal_id = self._active_goal_id
        self._active_goal_name = name
        self._active_goal_handle = None
        self._active_undock_goal_handle = None
        self._active_dock_goal_handle = None
        self._precise_docking_active = True
        self._last_docking_feedback_state = None
        self._charging_status_count = 0
        self._navigation_paused = False
        self._paused_goal_name = None
        self._active_goal_start_time = self.get_clock().now()
        self._goal_reached_since = None
        self._last_arrival_log_time = None
        self._failed_goal_name = None

        future = self._dock_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback_msg, goal_name=name, sent_goal_id=goal_id:
            self._handle_docking_feedback(feedback_msg, goal_name, sent_goal_id),
        )
        future.add_done_callback(
            lambda response_future, goal_name=name, sent_goal_id=goal_id:
            self._handle_docking_goal_response(response_future, goal_name, sent_goal_id)
        )
        self._goal_futures.append(future)
        self._publish_waypoint_markers()
        self._publish_navigation_status(
            'docking_started',
            f'started precise docking at {name}',
            waypoint_name=name,
        )
        return True, f'已启动充电桩 {name} 的精确入桩流程'

    def _handle_docking_feedback(self, feedback_msg, goal_name: str, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            return

        feedback = feedback_msg.feedback
        state = int(feedback.state)
        if state == self._last_docking_feedback_state:
            return
        self._last_docking_feedback_state = state
        state_names = {
            DockRobot.Feedback.NAV_TO_STAGING_POSE: '前往入桩预备位',
            DockRobot.Feedback.INITIAL_PERCEPTION: '确认充电桩位姿',
            DockRobot.Feedback.CONTROLLING: '低速精确入桩',
            DockRobot.Feedback.WAIT_FOR_CHARGE: '等待入桩确认',
            DockRobot.Feedback.RETRY: '入桩重试',
        }
        state_text = state_names.get(state, f'未知阶段 {state}')
        retries = int(feedback.num_retries)
        self.get_logger().info(f'充电桩 {goal_name}: {state_text}, retries={retries}')
        self._publish_navigation_status(
            'docking',
            f'{state_text}, retries={retries}',
            waypoint_name=goal_name,
        )

    def _handle_docking_cmd_vel(self, msg: Twist) -> None:
        if not self._precise_docking_active or not self._active_goal_name:
            return

        now = self.get_clock().now()
        if self._last_docking_telemetry_time is not None:
            elapsed = (now - self._last_docking_telemetry_time).nanoseconds / 1e9
            if elapsed < self._dock_telemetry_period:
                return
        self._last_docking_telemetry_time = now

        waypoint = self._waypoints.get(self._active_goal_name)
        if waypoint is None:
            return
        try:
            if self._dock_target_odom_pose is not None:
                x, y, yaw = self._robot_pose_in_frame(self._odom_frame)
                tx, ty, tyaw = self._dock_target_odom_pose
            else:
                x, y, yaw = self._robot_pose_in_frame(self._map_frame)
                tx, ty, tyaw = self._pose_in_frame(waypoint, self._map_frame)
        except TransformException:
            return

        dx = tx - x
        dy = ty - y
        forward_error = math.cos(yaw) * dx + math.sin(yaw) * dy
        lateral_error = -math.sin(yaw) * dx + math.cos(yaw) * dy
        xy_error = math.hypot(dx, dy)
        yaw_error = self._normalize_angle(tyaw - yaw)
        self.get_logger().info(
            f'入桩控制数据: vx={msg.linear.x:.3f} m/s, wz={msg.angular.z:.3f} rad/s, '
            f'xy={xy_error:.3f} m, longitudinal={forward_error:.3f} m, '
            f'lateral={lateral_error:.3f} m, yaw_error={yaw_error:.3f} rad'
        )
        if self._last_docking_feedback_state == DockRobot.Feedback.NAV_TO_STAGING_POSE:
            self._publish_navigation_status(
                'docking_staging_approach',
                f'approaching docking staging pose for {self._active_goal_name}',
                waypoint_name=self._active_goal_name,
                distance_remaining=xy_error,
            )
        elif self._last_docking_feedback_state == DockRobot.Feedback.CONTROLLING:
            self._publish_navigation_status(
                'docking_control',
                f'closed-loop docking to {self._active_goal_name}',
                waypoint_name=self._active_goal_name,
                distance_remaining=xy_error,
            )

    def _check_docking_drift(self) -> None:
        if (not self._precise_docking_active or self._dock_target_odom_pose is None
                or self._active_dock_goal_handle is None
                or self._last_docking_feedback_state != DockRobot.Feedback.CONTROLLING):
            return
        try:
            current = self._robot_pose_in_frame(self._odom_frame)
            # Express the robot in the fixed dock frame, NOT its turning frame.
            longitudinal, lateral, heading = body_frame_error(self._dock_target_odom_pose, current)
            reason = None
            if not self._battery_state_fresh():
                reason = '电池反馈已过期'
            elif abs(lateral) > self._dock_max_lateral_drift or abs(heading) > self._dock_max_heading_drift:
                reason = f'倒车偏离对接走廊: 横向{lateral:.3f}m, 角度{math.degrees(heading):.1f}°'
            elif longitudinal < -0.03:
                reason = '已越过保存的充电接触位置但未确认充电'
        except TransformException as exc:
            reason = f'入桩里程计反馈不可用: {exc}'
        if reason is None:
            return
        name, handle = self._active_goal_name, self._active_dock_goal_handle
        self._active_goal_id += 1
        self._dock_cancel_pending = True
        self._short_cmd_vel_pub.publish(Twist())
        try:
            handle.cancel_goal_async()
            handle.get_result_async().add_done_callback(self._finish_dock_abort)
        except Exception as exc:
            self.get_logger().error(f'入桩取消未确认，禁止启动新目标: {exc}')
        self._fail_precise_docking(name, reason + '；已停车，请重新对准后重试回充')

    def _finish_dock_abort(self, future) -> None:
        try:
            terminal = future.result().status in (
                GoalStatus.STATUS_SUCCEEDED, GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_ABORTED)
            if terminal:
                self._dock_cancel_pending = False
        except Exception as exc:
            self.get_logger().error(f'入桩停止状态未确认: {exc}')

    def _handle_battery_state(self, msg: BatteryState) -> None:
        self._last_battery_status_time = self.get_clock().now()
        status_charging = msg.power_supply_status in (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            BatteryState.POWER_SUPPLY_STATUS_FULL,
        )
        self._battery_is_charging = status_charging
        if not status_charging:
            self._charging_status_count = 0
            if not self._charging_detection_armed:
                self._charging_detection_armed = True
                self.get_logger().info('充电状态已解除，航点管理器已重新武装下一次入桩')
            return

        if (
            not self._charging_detection_armed
            or not self._precise_docking_active
            or self._last_docking_feedback_state not in (
                DockRobot.Feedback.CONTROLLING,
                DockRobot.Feedback.WAIT_FOR_CHARGE,
            )
        ):
            return

        self._charging_status_count += 1
        if self._charging_status_count < self._charging_status_confirm_samples:
            return

        self._charging_detection_armed = False
        self._complete_docking_by_charge_status()

    def _battery_state_fresh(self) -> bool:
        stamp = getattr(self, '_last_battery_status_time', None)
        return stamp is not None and 0 <= (self.get_clock().now() - stamp).nanoseconds / 1e9 <= 3.0

    def _complete_docking_by_charge_status(self) -> None:
        goal_name = self._active_goal_name
        if not goal_name:
            return

        self._active_goal_id += 1
        dock_goal_handle = self._active_dock_goal_handle
        self._active_dock_goal_handle = None
        self._precise_docking_active = False
        self._active_goal_name = None
        self._active_goal_start_time = None
        self._goal_reached_since = None
        self._failed_goal_name = None
        self._completed_waypoints.add(goal_name)

        if dock_goal_handle is not None:
            try:
                dock_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'充电后取消 DockRobot 目标失败: {exc}')

        self._publish_waypoint_markers()
        self.get_logger().info(f'检测到稳定充电状态，点位 {goal_name} 入桩完成')
        self._publish_navigation_status(
            'docked_by_charge_status',
            f'docking completed by battery charging status at {goal_name}',
            waypoint_name=goal_name,
        )
        self._publish_voice_prompt(self._voice_arrival_template, goal_name)
        self._advance_sequence_after_arrival(goal_name)

    def _fail_precise_docking(self, goal_name: str, message: str) -> None:
        self._dock_staging_active = False
        self._active_goal_name = None
        self._active_dock_goal_handle = None
        self._precise_docking_active = False
        self._goal_reached_since = None
        self._failed_goal_name = goal_name
        self._publish_waypoint_markers()
        self._publish_navigation_status('docking_failed', message, waypoint_name=goal_name)
        self.get_logger().warning(message)
        self._stop_sequence()

    def _handle_docking_goal_response(self, future, goal_name: str, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            try:
                stale_handle = future.result()
                if stale_handle.accepted:
                    stale_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'取消过期入桩请求失败: {exc}')
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_precise_docking(goal_name, f'发送精确入桩目标失败: {exc}')
            return

        if not goal_handle.accepted:
            self._fail_precise_docking(goal_name, 'Docking Server 拒绝精确入桩目标')
            return

        self._active_dock_goal_handle = goal_handle
        self._publish_voice_prompt('已对准充电桩，开始低速对接', goal_name)
        self.get_logger().info(f'充电桩 {goal_name} 的精确入桩目标已被接收')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed_future, current_name=goal_name, sent_goal_id=goal_id:
            self._handle_docking_result(completed_future, current_name, sent_goal_id)
        )
        self._result_futures.append(result_future)

    def _handle_docking_result(self, future, goal_name: str, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            return
        try:
            completed = future.result()
        except Exception as exc:
            self._fail_precise_docking(goal_name, f'获取精确入桩结果失败: {exc}')
            return

        self._active_dock_goal_handle = None
        self._precise_docking_active = False
        result = completed.result
        if completed.status != GoalStatus.STATUS_SUCCEEDED or not result.success:
            self._fail_precise_docking(
                goal_name,
                f'精确入桩失败: status={completed.status}, error_code={result.error_code}, '
                f'retries={result.num_retries}, {result.error_msg}',
            )
            return

        if not self._battery_state_fresh() or not self._battery_is_charging:
            self._fail_precise_docking(goal_name, '入桩动作已停止，但未确认充电，请检查对接触点')
            return

        waypoint = self._waypoints[goal_name]
        try:
            x, y, yaw = self._lookup_current_pose()
        except TransformException as exc:
            self._fail_precise_docking(goal_name, f'入桩完成但无法复核最终位姿: {exc}')
            return

        xy_error = math.hypot(x - float(waypoint['x']), y - float(waypoint['y']))
        yaw_error = abs(self._normalize_angle(yaw - float(waypoint.get('yaw', 0.0))))
        if (
            xy_error > self._dock_verify_xy_tolerance
            or yaw_error > self._dock_verify_yaw_tolerance
        ):
            self._fail_precise_docking(
                goal_name,
                f'入桩动作完成但最终位姿未通过复核: '
                f'xy_error={xy_error:.3f}/{self._dock_verify_xy_tolerance:.3f} m, '
                f'yaw_error={yaw_error:.3f}/{self._dock_verify_yaw_tolerance:.3f} rad',
            )
            return

        self._active_goal_name = None
        self._active_goal_start_time = None
        self._failed_goal_name = None
        self._completed_waypoints.add(goal_name)
        self._publish_waypoint_markers()
        self.get_logger().info(
            f'充电桩 {goal_name} 精确入桩完成并通过复核 '
            f'(xy_error={xy_error:.3f} m, yaw_error={yaw_error:.3f} rad)'
        )
        self._publish_navigation_status(
            'docked',
            f'precise docking completed at {goal_name}',
            waypoint_name=goal_name,
            distance_remaining=xy_error,
            result_status=completed.status,
        )
        self._publish_voice_prompt(self._voice_arrival_template, goal_name)
        self._advance_sequence_after_arrival(goal_name)

    def _start_navigation(
        self,
        name: str,
        skip_undock: bool = False,
        force_normal: bool = False,
        precision_retry: bool = False,
    ) -> (bool, str):
        if getattr(self, '_dock_cancel_pending', False):
            return False, '正在等待入桩控制器停止确认，暂不能启动新目标'
        if getattr(self, '_precise_docking_active', False):
            return False, '正在执行入桩，请先停止当前回充再选择其他目标'
        if name not in self._waypoints:
            return False, f'未找到点位 {name}'

        self._arrival_refine_pending = False
        if not precision_retry:
            self._final_precision_retry_count = 0

        if self._short_servo_active:
            self._deactivate_short_distance()

        if self._precise_docking_enabled and name == self._charger_waypoint_name:
            if not self._dock_prealign_enabled:
                return self._start_precise_docking(name)
            if not self._short_servo_enabled or not self._far_navigation_auto_refine_enabled:
                return False, '回充预对准需要启用导航末端精调'
            if not self._battery_state_fresh():
                return False, '未收到新鲜电池状态，请检查电池反馈后回充'
            if self._battery_is_charging:
                return False, '机器人当前已在充电，无需重复回充'
            self._dock_staging_active = True
            force_normal = True
            skip_undock = True
        else:
            self._dock_staging_active = False

        if not skip_undock and self._should_auto_undock(name):
            return self._start_automatic_undock(name)

        # Direct short-distance dispatch and post-Nav2 final refinement are
        # deliberately separate.  Some front ends always use Nav2 for the
        # main leg, while retaining this servo only as the final pose trim.
        if not force_normal and self._short_direct_start_enabled:
            short_started, short_distance = self._try_start_short_distance(name)
            if short_started:
                return True, (
                    f'已启动短距离分轴位姿伺服：{name}（{short_distance:.2f} 米）'
                )

        if not self._navigate_client.wait_for_server(timeout_sec=2.0):
            return False, 'navigate_to_pose action server 未就绪'

        goal = self._build_goal(name)
        self._active_goal_id += 1
        goal_id = self._active_goal_id
        future = self._navigate_client.send_goal_async(
            goal,
            feedback_callback=lambda feedback_msg, goal_name=name, sent_goal_id=goal_id: self._handle_navigation_feedback(
                feedback_msg,
                goal_name,
                sent_goal_id,
            ),
        )
        future.add_done_callback(lambda result_future, goal_name=name, sent_goal_id=goal_id: self._handle_goal_response(
            result_future,
            goal_name,
            sent_goal_id,
        ))
        self._goal_futures.append(future)
        self._navigation_paused = False
        self._paused_goal_name = None
        self._active_goal_name = name
        self._active_goal_handle = None
        self._active_goal_start_time = self.get_clock().now()
        self._goal_reached_since = None
        self._last_arrival_log_time = None
        self._failed_goal_name = None
        self._last_feedback = None
        self._publish_waypoint_markers()
        self._publish_navigation_status('started', f'started navigation to {name}', waypoint_name=name)
        return True, f'已向 Nav2 发送点位 {name} 的导航请求'

    def _fail_automatic_undock(self, target_name: str, message: str) -> None:
        self._active_goal_name = None
        self._active_undock_goal_handle = None
        self._undock_pending_goal_name = None
        self._failed_goal_name = target_name
        self._publish_waypoint_markers()
        self._publish_navigation_status('undock_failed', message, waypoint_name=target_name)
        self.get_logger().warning(message)
        self._stop_sequence()

    def _handle_undock_goal_response(self, future, target_name: str, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self._fail_automatic_undock(target_name, f'发送自动离桩目标失败: {exc}')
            return

        if not goal_handle.accepted:
            self._fail_automatic_undock(target_name, 'Nav2 拒绝自动离桩目标')
            return

        self._active_undock_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed_future, goal_name=target_name, sent_goal_id=goal_id:
            self._handle_undock_result(completed_future, goal_name, sent_goal_id)
        )
        self._result_futures.append(result_future)
        self.get_logger().info(
            f'开始自动离桩: 前移 {self._undock_distance:.2f} m, '
            f'速度 {self._undock_speed:.2f} m/s'
        )

    def _handle_undock_result(self, future, target_name: str, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            return
        try:
            completed = future.result()
        except Exception as exc:
            self._fail_automatic_undock(target_name, f'获取自动离桩结果失败: {exc}')
            return

        self._active_undock_goal_handle = None
        self._undock_pending_goal_name = None
        if completed.status != GoalStatus.STATUS_SUCCEEDED:
            error_code = getattr(completed.result, 'error_code', 0)
            error_msg = getattr(completed.result, 'error_msg', '')
            self._fail_automatic_undock(
                target_name,
                f'自动离桩失败: status={completed.status}, error_code={error_code}, {error_msg}',
            )
            return

        self.get_logger().info(f'自动离桩完成，开始导航到 {target_name}')
        started, message = self._start_navigation(target_name, skip_undock=True)
        if not started:
            self._fail_automatic_undock(target_name, f'离桩后启动导航失败: {message}')

    def _stop_sequence(self) -> None:
        self._cancel_sequence_dwell()
        self._active_sequence = []
        self._active_sequence_dwell_times = []
        self._active_sequence_index = -1
        self._active_sequence_loop = False
        self._clear_paused_navigation()

    def _cancel_sequence_dwell(self) -> None:
        self._sequence_dwell_token += 1
        timer = self._sequence_dwell_timer
        self._sequence_dwell_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)

    def _handle_save_waypoint(self, request, response):
        name = request.name.strip()
        if not name:
            response.success = False
            response.message = '点位名称不能为空'
            return response

        try:
            x, y, yaw = self._lookup_current_pose()
        except TransformException as exc:
            response.success = False
            response.message = f'获取当前位姿失败: {exc}'
            return response

        self._waypoints[name] = {
            'frame_id': self._map_frame,
            'x': float(x),
            'y': float(y),
            'yaw': float(yaw),
            'description': '',
        }
        self._write_waypoints()
        self._publish_waypoint_markers()

        response.success = True
        response.message = f'点位 {name} 已保存'
        return response

    def _handle_delete_waypoint(self, request, response):
        name = request.name.strip()
        if name not in self._waypoints:
            response.success = False
            response.message = f'未找到点位 {name}'
            return response

        del self._waypoints[name]
        self._write_waypoints()
        self._publish_waypoint_markers()
        response.success = True
        response.message = f'点位 {name} 已删除'
        return response

    def _handle_rename_waypoint(self, request, response):
        old_name = request.old_name.strip()
        new_name = request.new_name.strip()

        if not old_name:
            response.success = False
            response.message = '原航点名称不能为空'
            return response

        if not new_name:
            response.success = False
            response.message = '新航点名称不能为空'
            return response

        if old_name not in self._waypoints:
            response.success = False
            response.message = f'未找到点位 {old_name}'
            return response

        if new_name in self._waypoints and new_name != old_name:
            response.success = False
            response.message = f'点位 {new_name} 已存在'
            return response

        if new_name == old_name:
            response.success = True
            response.message = f'点位名称未变化: {old_name}'
            return response

        self._waypoints[new_name] = self._waypoints.pop(old_name)
        self._active_sequence = [
            new_name if name == old_name else name for name in self._active_sequence
        ]

        if self._active_goal_name == old_name:
            self._active_goal_name = new_name
        if self._paused_goal_name == old_name:
            self._paused_goal_name = new_name
        if self._failed_goal_name == old_name:
            self._failed_goal_name = new_name
        if old_name in self._completed_waypoints:
            self._completed_waypoints.discard(old_name)
            self._completed_waypoints.add(new_name)

        self._write_waypoints()
        self._publish_waypoint_markers()
        self._publish_navigation_status(
            'waypoint_renamed',
            f'waypoint renamed from {old_name} to {new_name}',
            waypoint_name=new_name,
        )

        response.success = True
        response.message = f'点位 {old_name} 已重命名为 {new_name}'
        return response

    def _handle_list_waypoints(self, _request, response):
        response.success = True
        response.message = '查询成功'
        response.names = self._public_waypoint_names()
        return response

    def _handle_navigate_to_pose_refined(self, request, response):
        values = (float(request.x), float(request.y), float(request.yaw))
        if not all(math.isfinite(value) for value in values):
            response.success = False
            response.message = '手动导航目标包含无效坐标'
            return response

        self._stop_sequence()
        self._completed_waypoints.discard(TRANSIENT_MANUAL_GOAL_NAME)
        self._failed_goal_name = None
        self._waypoints[TRANSIENT_MANUAL_GOAL_NAME] = {
            'frame_id': str(request.frame_id or self._map_frame),
            'x': values[0],
            'y': values[1],
            'yaw': self._normalize_angle(values[2]),
            'description': 'transient manual navigation goal',
        }
        response.success, response.message = self._start_navigation(
            TRANSIENT_MANUAL_GOAL_NAME,
            force_normal=True,
        )
        if response.success:
            response.message = '已启动地图指定位置导航，末端精修已启用'
        return response

    def _handle_navigate_to_waypoint(self, request, response):
        self._stop_sequence()
        self._completed_waypoints.clear()
        self._failed_goal_name = None
        name = request.name.strip()
        response.success, response.message = self._start_navigation(name)
        return response

    def _handle_navigate_waypoint_sequence(self, request, response):
        names = [name.strip() for name in request.names if name.strip()]
        if not names:
            response.success = False
            response.message = '航点序列不能为空'
            return response

        missing_names = [name for name in names if name not in self._waypoints]
        if missing_names:
            response.success = False
            response.message = f'以下航点不存在: {", ".join(missing_names)}'
            return response

        requested_dwell_times = [float(value) for value in request.dwell_times]
        if requested_dwell_times and len(requested_dwell_times) != len(names):
            response.success = False
            response.message = '停留时间数量必须与航点数量一致'
            return response
        dwell_times = requested_dwell_times or [0.0] * len(names)
        if any(
            not math.isfinite(value) or value < 0.0 or value > 3600.0
            for value in dwell_times
        ):
            response.success = False
            response.message = '航点停留时间必须在 0～3600 秒之间'
            return response

        self._cancel_sequence_dwell()
        self._clear_paused_navigation()
        self._completed_waypoints.clear()
        self._failed_goal_name = None
        self._active_sequence = names
        self._active_sequence_dwell_times = dwell_times
        self._active_sequence_index = 0
        self._active_sequence_loop = bool(request.loop)

        response.success, response.message = self._start_navigation(names[0])
        if not response.success:
            self._stop_sequence()
            return response

        mode_text = '循环' if self._active_sequence_loop else '单次'
        route_text = ' -> '.join(
            f'{name}(停留 {dwell:g}s)'
            for name, dwell in zip(
                self._active_sequence, self._active_sequence_dwell_times
            )
        )
        response.message = (
            f'已启动{mode_text}航点序列导航: {route_text}'
        )
        return response

    def _clear_paused_navigation(self) -> None:
        self._navigation_paused = False
        self._paused_goal_name = None

    def _handle_pause_navigation(self, _request, response):
        if self._navigation_paused:
            response.success = True
            response.message = f'导航已暂停，当前目标: {self._paused_goal_name}'
            return response

        goal_name = self._active_goal_name
        if not goal_name:
            response.success = False
            response.message = '当前没有正在执行的导航目标'
            return response

        if self._short_servo_active:
            self._deactivate_short_distance()

        self._arrival_refine_pending = False

        self._navigation_paused = True
        self._paused_goal_name = goal_name
        self._active_goal_id += 1
        self._active_goal_name = None
        self._active_goal_start_time = None
        self._goal_reached_since = None
        goal_handle = self._active_goal_handle
        self._active_goal_handle = None
        undock_goal_handle = self._active_undock_goal_handle
        self._active_undock_goal_handle = None
        self._undock_pending_goal_name = None
        dock_goal_handle = self._active_dock_goal_handle
        self._active_dock_goal_handle = None
        self._precise_docking_active = False
        self._last_docking_feedback_state = None

        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'failed to cancel paused goal {goal_name}: {exc}')
        if undock_goal_handle is not None:
            try:
                undock_goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'failed to cancel undock goal for {goal_name}: {exc}')
        if dock_goal_handle is not None:
            try:
                self._dock_cancel_pending = True
                dock_goal_handle.cancel_goal_async()
                dock_goal_handle.get_result_async().add_done_callback(self._finish_dock_abort)
            except Exception as exc:
                self.get_logger().warning(f'failed to cancel docking goal for {goal_name}: {exc}')

        self._publish_waypoint_markers()
        self._publish_navigation_status('paused', f'paused navigation at waypoint {goal_name}', waypoint_name=goal_name)
        response.success = True
        response.message = f'已暂停导航，当前目标: {goal_name}'
        return response

    def _handle_resume_navigation(self, _request, response):
        if not self._navigation_paused or not self._paused_goal_name:
            response.success = False
            response.message = '当前没有已暂停的导航'
            return response

        goal_name = self._paused_goal_name
        if goal_name not in self._waypoints:
            self._clear_paused_navigation()
            response.success = False
            response.message = f'暂停目标不存在: {goal_name}'
            return response

        started, message = self._start_navigation(goal_name)
        if not started:
            self._navigation_paused = True
            self._paused_goal_name = goal_name
            response.success = False
            response.message = f'恢复导航失败: {message}'
            return response

        response.success = True
        response.message = f'已继续导航，当前目标: {goal_name}'
        return response

    def _handle_cancel_navigation(self, request, response):
        goal_name = self._active_goal_name or self._paused_goal_name
        if not goal_name:
            response.success = False
            response.message = '当前没有正在执行或暂停的导航目标'
            return response

        if self._active_goal_name:
            pause_response = Trigger.Response()
            self._handle_pause_navigation(request, pause_response)
            if not pause_response.success:
                response.success = False
                response.message = pause_response.message
                return response

        self._arrival_refine_pending = False
        self._stop_sequence()
        self._publish_navigation_status(
            'canceled',
            f'navigation to {goal_name} canceled by operator',
            waypoint_name=goal_name,
        )
        response.success = True
        response.message = f'已取消导航：{self._display_goal_name(goal_name)}'
        return response

    def _handle_goal_response(self, future, goal_name: str, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            try:
                stale_handle = future.result()
                if stale_handle.accepted:
                    stale_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().warning(f'取消过期导航请求失败: {exc}')
            self.get_logger().info(f'Ignoring stale goal response: {goal_name}, goal_id={goal_id}')
            return
        try:
            goal_handle = future.result()
        except Exception as exc:
            self.get_logger().error(f'发送点位 {goal_name} 导航目标失败: {exc}')
            if self._active_goal_name == goal_name:
                self._active_goal_name = None
                self._active_goal_handle = None
                self._failed_goal_name = goal_name
                self._publish_waypoint_markers()
            self._publish_navigation_status('failed', f'failed to send goal {goal_name}', waypoint_name=goal_name)
            self._stop_sequence()
            return

        if not goal_handle.accepted:
            self.get_logger().warning(f'点位 {goal_name} 的导航目标被 Nav2 拒绝')
            if self._active_goal_name == goal_name:
                self._active_goal_name = None
                self._active_goal_handle = None
                self._failed_goal_name = goal_name
                self._publish_waypoint_markers()
            self._publish_navigation_status('rejected', f'Nav2 rejected goal {goal_name}', waypoint_name=goal_name)
            self._stop_sequence()
            return

        self.get_logger().info(f'点位 {goal_name} 的导航目标已被 Nav2 接收')
        self._publish_navigation_status('accepted', f'Nav2 accepted goal {goal_name}', waypoint_name=goal_name)
        self._publish_voice_prompt(self._voice_departure_template, goal_name)
        self._active_goal_handle = goal_handle
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(
            lambda completed_future, current_name=goal_name, sent_goal_id=goal_id: self._handle_navigation_result(
                completed_future,
                current_name,
                sent_goal_id,
            )
        )
        self._result_futures.append(result_future)

    def _waypoint_pose_error(self, goal_name):
        try:
            current = self._robot_pose_in_frame(self._map_frame)
            target = self._pose_in_frame(self._navigation_target(goal_name), self._map_frame)
            x, y, yaw = body_frame_error(current, target)
            return math.hypot(x, y), abs(yaw)
        except (TransformException, KeyError, ValueError):
            return None

    def _handle_navigation_result(self, future, goal_name: str, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            self.get_logger().info(f'Ignoring stale navigation result: {goal_name}, goal_id={goal_id}')
            return
        try:
            result = future.result()
        except Exception as exc:
            self.get_logger().error(f'获取点位 {goal_name} 导航结果失败: {exc}')
            self._arrival_refine_pending = False
            if self._active_goal_name == goal_name:
                self._active_goal_name = None
                self._active_goal_handle = None
                self._goal_reached_since = None
                self._failed_goal_name = goal_name
                self._publish_waypoint_markers()
            self._publish_navigation_status('failed', f'failed to get result for {goal_name}', waypoint_name=goal_name)
            self._stop_sequence()
            return

        status = result.status
        if self._arrival_refine_pending:
            self._arrival_refine_pending = False
            self._active_goal_handle = None
            if status in (GoalStatus.STATUS_CANCELED, GoalStatus.STATUS_SUCCEEDED):
                self._finish_arrival_refine_handover(goal_name)
                return
        if status == GoalStatus.STATUS_SUCCEEDED:
            if self._active_goal_name == goal_name:
                self._active_goal_name = None
                self._active_goal_handle = None
                self._goal_reached_since = None
            if self._far_navigation_auto_refine_enabled:
                refine_started, refine_distance = self._try_start_short_distance(
                    goal_name,
                    announce_departure=False,
                    final_refine=True,
                )
                if refine_started:
                    self.get_logger().info(
                        f'Nav2 completed the far leg; automatically refining {goal_name}: '
                        f'distance={refine_distance:.3f} m'
                    )
                    self._publish_navigation_status(
                        'far_navigation_refining',
                        f'Nav2 reached {goal_name}; refining final pose with odom servo',
                        waypoint_name=goal_name,
                        distance_remaining=refine_distance,
                        result_status=status,
                    )
                    return
                self._failed_goal_name = goal_name
                self._publish_navigation_status(
                    'failed', '末端精调无法启动，请检查定位后重试当前点位',
                    waypoint_name=goal_name,
                )
                self._stop_sequence()
                return
            final_error = self._waypoint_pose_error(goal_name)
            if final_error is None:
                self._failed_goal_name = goal_name
                self._publish_navigation_status('failed', '无法验证最终位姿，请检查定位后重试当前点位', waypoint_name=goal_name)
                self._stop_sequence()
                return
            if final_error is not None:
                xy_error, yaw_error = final_error
                within_precision = (
                    xy_error <= self._arrival_xy_tolerance
                    and yaw_error <= self._arrival_yaw_tolerance
                )
                if not within_precision:
                    if self._final_precision_retry_count < self._final_precision_nav2_retries:
                        self._final_precision_retry_count += 1
                        started, message = self._start_navigation(
                            goal_name,
                            skip_undock=True,
                            force_normal=True,
                            precision_retry=True,
                        )
                        if started:
                            self.get_logger().warning(
                                f'Nav2 reported success outside final precision at {goal_name}; '
                                f'retrying once (xy={xy_error:.3f}m, yaw={yaw_error:.3f}rad)'
                            )
                            self._publish_navigation_status(
                                'final_precision_retry',
                                f'retrying {goal_name} because final pose is outside tolerance',
                                waypoint_name=goal_name,
                                distance_remaining=xy_error,
                            )
                            return
                        self.get_logger().warning(
                            f'Failed to start final precision retry for {goal_name}: {message}'
                        )
                    self._failed_goal_name = goal_name
                    self._publish_waypoint_markers()
                    self._publish_navigation_status(
                        'failed',
                        f'Nav2 stopped outside final precision for {goal_name}',
                        waypoint_name=goal_name,
                        distance_remaining=xy_error,
                        result_status=status,
                    )
                    self._stop_sequence()
                    return
            self._final_precision_retry_count = 0
            self._failed_goal_name = None
            self._completed_waypoints.add(goal_name)
            self._publish_waypoint_markers()
            self.get_logger().info(f'已到达点位 {goal_name}')
            self._publish_navigation_status('succeeded', f'arrived at waypoint {goal_name}', waypoint_name=goal_name, result_status=status)
            self._publish_voice_prompt(self._voice_arrival_template, goal_name)
        else:
            if self._active_goal_name == goal_name:
                self._active_goal_name = None
                self._active_goal_handle = None
                self._goal_reached_since = None
            self._failed_goal_name = goal_name
            self._publish_waypoint_markers()
            self.get_logger().warning(f'点位 {goal_name} 导航未成功结束，status={status}')
            self._publish_navigation_status('failed', f'navigation to {goal_name} finished with status {status}', waypoint_name=goal_name, result_status=status)
            self._stop_sequence()
            return

        self._advance_sequence_after_arrival(goal_name)

    def _advance_sequence_after_arrival(self, goal_name: str) -> None:
        if not self._active_sequence:
            return

        dwell_time = 0.0
        if 0 <= self._active_sequence_index < len(
            self._active_sequence_dwell_times
        ):
            dwell_time = self._active_sequence_dwell_times[
                self._active_sequence_index
            ]
        if dwell_time > 0.0:
            self._begin_sequence_dwell(goal_name, dwell_time)
            return

        self._advance_sequence_now()

    def _begin_sequence_dwell(
        self, goal_name: str, dwell_time: float
    ) -> None:
        if self._sequence_dwell_timer is not None:
            return
        self._sequence_dwell_token += 1
        token = self._sequence_dwell_token
        self.get_logger().info(
            f'已到达航点 {goal_name}，停留 {dwell_time:g} 秒'
        )
        self._publish_navigation_status(
            'dwelling',
            f'已到达 {goal_name}，停留 {dwell_time:g} 秒后继续',
            waypoint_name=goal_name,
        )
        self._sequence_dwell_timer = self.create_timer(
            dwell_time,
            lambda current_token=token:
            self._finish_sequence_dwell(current_token),
        )

    def _finish_sequence_dwell(self, token: int) -> None:
        if token != self._sequence_dwell_token:
            return
        timer = self._sequence_dwell_timer
        self._sequence_dwell_timer = None
        if timer is not None:
            timer.cancel()
            self.destroy_timer(timer)
        if not self._active_sequence:
            return
        self._advance_sequence_now()

    def _advance_sequence_now(self) -> None:
        if not self._active_sequence:
            return
        if self._active_sequence_index >= len(self._active_sequence) - 1:
            if not self._active_sequence_loop:
                self.get_logger().info('航点序列导航已完成')
                self._publish_navigation_status('sequence_succeeded', 'waypoint sequence completed')
                self._stop_sequence()
                return
            self._active_sequence_index = 0
        else:
            self._active_sequence_index += 1

        next_name = self._active_sequence[self._active_sequence_index]
        self.get_logger().info(
            f'Continuing waypoint sequence: {self._active_sequence_index + 1}/{len(self._active_sequence)} -> {next_name}'
        )
        self._publish_navigation_status(
            'advancing',
            f'advancing sequence to waypoint {next_name}',
            waypoint_name=next_name,
        )
        started, message = self._start_navigation(next_name)
        if not started:
            self.get_logger().warning(f'继续航点序列失败: {message}')
            self._failed_goal_name = next_name
            self._publish_waypoint_markers()
            self._publish_navigation_status(
                'sequence_failed',
                f'failed to continue sequence to {next_name}: {message}',
                waypoint_name=next_name,
            )
            self._stop_sequence()

    def _check_active_goal_arrival(self) -> None:
        if self._navigation_paused:
            self._goal_reached_since = None
            return

        # Charging dock arrival is owned by DockRobot and its stricter final pose check.
        if self._precise_docking_active:
            self._goal_reached_since = None
            return

        # Direct short-distance servo owns its stricter latched completion.
        # The legacy 12 cm shortcut must not stop a 50 cm move early.
        if self._short_servo_active:
            self._goal_reached_since = None
            return

        if self._arrival_refine_pending:
            # If cancellation does not finish, Nav2 retains exclusive ownership.
            # Never publish direct velocity merely because cancel was requested.
            if (self.get_clock().now() - self._arrival_refine_requested_at).nanoseconds / 1e9 > 5.0:
                self._arrival_refine_pending = False
                self.get_logger().warning('Final refinement handoff timed out; Nav2 retains control')
            return

        goal_name = self._active_goal_name
        if not goal_name or goal_name not in self._waypoints:
            self._goal_reached_since = None
            return

        try:
            x, y, yaw = self._robot_pose_in_frame(self._map_frame)
        except TransformException:
            return

        waypoint = self._navigation_target(goal_name)
        xy_error = math.hypot(x - float(waypoint['x']), y - float(waypoint['y']))
        yaw_error = abs(self._normalize_angle(yaw - float(waypoint.get('yaw', 0.0))))
        within_goal = (
            xy_error <= self._arrival_xy_tolerance
            and yaw_error <= self._arrival_yaw_tolerance
        )
        can_refine = (
            self._far_navigation_auto_refine_enabled and self._short_servo_enabled
            and self._active_goal_handle is not None
            and self._short_distance_threshold >= xy_error
            and xy_error <= self._refine_handoff_distance
            and yaw_error <= self._refine_handoff_yaw
        )

        now = self.get_clock().now()
        if not (within_goal or can_refine):
            self._goal_reached_since = None
            if xy_error <= 0.5:
                should_log = self._last_arrival_log_time is None
                if not should_log:
                    elapsed = (now - self._last_arrival_log_time).nanoseconds / 1e9
                    should_log = elapsed >= 2.0
                if should_log:
                    self.get_logger().info(
                        f'Waiting for waypoint {goal_name} tolerance: '
                        f'xy_error={xy_error:.3f}/{self._arrival_xy_tolerance:.3f}, '
                        f'yaw_error={yaw_error:.3f}/{self._arrival_yaw_tolerance:.3f}'
                    )
                    self._last_arrival_log_time = now
            return

        if self._goal_reached_since is None:
            self._goal_reached_since = now
            return

        settled = (now - self._goal_reached_since).nanoseconds / 1e9
        if settled < self._arrival_settle_time:
            return

        self._handover_arrival_to_refine(goal_name, xy_error, yaw_error)

    def _handover_arrival_to_refine(
        self, goal_name: str, xy_error: float, yaw_error: float
    ) -> None:
        goal_handle = self._active_goal_handle
        if (
            not self._far_navigation_auto_refine_enabled
            or not self._short_servo_enabled
            or goal_handle is None
        ):
            self._complete_goal_by_tolerance(
                goal_name, xy_error, yaw_error, goal_handle
            )
            return

        # Keep the goal handle/result callback until the action reaches a
        # terminal state. Cancel acknowledgement alone is NOT output ownership.
        self._arrival_refine_pending = True
        self._arrival_refine_requested_at = self.get_clock().now()
        self._goal_reached_since = None
        self._publish_navigation_status(
            'far_navigation_refine_pending',
            f'接近 {goal_name}，准备末端精调（尚未到达）',
            waypoint_name=goal_name,
            distance_remaining=xy_error,
        )
        try:
            cancel_future = goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(lambda future, token=self._active_goal_id:
                self._check_refine_cancel(future, token))
        except Exception as exc:
            self.get_logger().warning(
                f'Nav2 cancellation before final refinement failed: {exc}'
            )
            self._arrival_refine_pending = False

    def _check_refine_cancel(self, future, goal_id):
        if goal_id != self._active_goal_id or not self._arrival_refine_pending:
            return
        try:
            if future.result().return_code != 0:
                self._arrival_refine_pending = False
                self.get_logger().warning('Nav2 refused cancellation; final servo not started')
        except Exception as exc:
            self._arrival_refine_pending = False
            self.get_logger().warning(f'Nav2 cancellation failed; retaining Nav2 ownership: {exc}')

    def _finish_arrival_refine_handover(self, goal_name: str) -> None:
        refine_started, refine_distance = self._try_start_short_distance(
            goal_name,
            announce_departure=False,
            final_refine=True,
        )
        if refine_started:
            self._publish_navigation_status(
                'far_navigation_refining',
                f'Nav2 reached {goal_name}; refining final pose with odom servo',
                waypoint_name=goal_name,
                distance_remaining=refine_distance,
            )
            return
        # Never substitute the larger handoff radius for final arrival accuracy.
        self._active_goal_name = None
        self._failed_goal_name = goal_name
        self._publish_navigation_status('failed', '末端精调无法启动，请检查定位后重试当前点位', waypoint_name=goal_name)
        self._stop_sequence()

    def _complete_goal_by_tolerance(
        self,
        goal_name: str,
        xy_error: float,
        yaw_error: float,
        goal_handle=None,
    ) -> None:
        if getattr(self, '_dock_staging_active', False):
            # Only completed odom/map refinement may hand over to DockRobot.
            # A pending Nav2 acceptance or broad arrival tolerance is not enough.
            return
        self._arrival_refine_pending = False

        self._active_goal_id += 1
        self._active_goal_name = None
        self._active_goal_start_time = None
        self._goal_reached_since = None
        goal_handle = goal_handle or self._active_goal_handle
        self._active_goal_handle = None
        self._failed_goal_name = None
        self._completed_waypoints.add(goal_name)

        if goal_handle is not None:
            try:
                goal_handle.cancel_goal_async()
            except Exception as exc:
                self.get_logger().debug(f'Failed to cancel reached goal {goal_name}: {exc}')

        self.get_logger().info(
            f'Waypoint {goal_name} considered reached by pose tolerance '
            f'(xy_error={xy_error:.3f}, yaw_error={yaw_error:.3f})'
        )
        self._publish_waypoint_markers()
        self._publish_navigation_status(
            'succeeded_by_tolerance',
            f'arrived at waypoint {goal_name} by pose tolerance',
            waypoint_name=goal_name,
            distance_remaining=xy_error,
        )
        self._publish_voice_prompt(self._voice_arrival_template, goal_name)
        self._advance_sequence_after_arrival(goal_name)

    def _handle_navigation_feedback(self, feedback_msg, goal_name: str, goal_id: int) -> None:
        if goal_id != self._active_goal_id:
            return
        feedback = feedback_msg.feedback
        self._last_feedback = feedback
        distance_remaining = float(getattr(feedback, 'distance_remaining', 0.0))
        estimated_time_remaining = self._duration_to_seconds(getattr(feedback, 'estimated_time_remaining', None))
        navigation_time = self._duration_to_seconds(getattr(feedback, 'navigation_time', None))
        self._publish_navigation_status(
            'running',
            f'navigating to {goal_name}, distance remaining {distance_remaining:.2f} m',
            waypoint_name=goal_name,
            distance_remaining=distance_remaining,
            estimated_time_remaining=estimated_time_remaining,
            navigation_time=navigation_time,
        )

    def _publish_navigation_status(
        self,
        state: str,
        message: str,
        waypoint_name: Optional[str] = None,
        distance_remaining: Optional[float] = None,
        estimated_time_remaining: Optional[float] = None,
        navigation_time: Optional[float] = None,
        result_status: Optional[int] = None,
    ) -> None:
        import json

        active_name = waypoint_name if waypoint_name is not None else self._active_goal_name
        display_name = self._display_goal_name(active_name)
        display_message = str(message)
        if active_name and display_name != active_name:
            display_message = display_message.replace(str(active_name), display_name)
        sequence_total = len(self._active_sequence)
        sequence_index = self._active_sequence_index if sequence_total else -1
        payload = {
            'stamp': self.get_clock().now().nanoseconds / 1e9,
            'state': state,
            'message': display_message,
            'active_waypoint': display_name,
            'sequence_index': sequence_index,
            'sequence_total': sequence_total,
            'sequence_loop': self._active_sequence_loop,
            'distance_remaining': distance_remaining,
            'estimated_time_remaining': estimated_time_remaining,
            'navigation_time': navigation_time,
            'result_status': result_status,
        }
        msg = String()
        msg.data = json.dumps(payload, ensure_ascii=False)
        self._last_navigation_status = state
        self._status_pub.publish(msg)

    @staticmethod
    def _duration_to_seconds(duration) -> Optional[float]:
        if duration is None:
            return None
        sec = float(getattr(duration, 'sec', 0.0))
        nanosec = float(getattr(duration, 'nanosec', 0.0))
        return sec + nanosec / 1e9

    def _publish_waypoint_markers(self) -> None:
        marker_array = MarkerArray()
        now = self.get_clock().now().to_msg()

        delete_marker = Marker()
        delete_marker.action = Marker.DELETEALL
        marker_array.markers.append(delete_marker)

        for index, name in enumerate(self._public_waypoint_names()):
            waypoint = self._waypoints[name]
            frame_id = waypoint.get('frame_id', self._map_frame)
            x = float(waypoint['x'])
            y = float(waypoint['y'])
            yaw = float(waypoint.get('yaw', 0.0))
            is_active = name == self._active_goal_name
            is_completed = name in self._completed_waypoints
            is_failed = name == self._failed_goal_name

            marker_array.markers.append(self._make_sphere_marker(index, name, frame_id, now, x, y, is_active, is_completed, is_failed))
            marker_array.markers.append(self._make_arrow_marker(index, name, frame_id, now, x, y, yaw, is_active, is_completed, is_failed))
            marker_array.markers.append(self._make_text_marker(index, name, frame_id, now, x, y, is_active, is_completed, is_failed))

        self._marker_pub.publish(marker_array)

    def _make_sphere_marker(self, index, name, frame_id, stamp, x, y, is_active, is_completed, is_failed):
        marker = self._base_marker(index * 3, 'waypoint_points', name, frame_id, stamp)
        marker.type = Marker.SPHERE
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.12
        marker.pose.orientation.w = 1.0
        scale = self._marker_scale * (1.25 if is_active else 1.0)
        marker.scale.x = scale
        marker.scale.y = scale
        marker.scale.z = 0.12
        if is_failed:
            marker.color = self._color(1.0, 0.15, 0.12, 0.95)
        elif is_active:
            marker.color = self._color(1.0, 0.62, 0.12, 0.95)
        elif is_completed:
            marker.color = self._color(0.05, 0.95, 0.55, 0.9)
        else:
            marker.color = self._color(0.1, 0.72, 1.0, 0.9)
        return marker

    def _make_arrow_marker(self, index, name, frame_id, stamp, x, y, yaw, is_active, is_completed, is_failed):
        marker = self._base_marker(index * 3 + 1, 'waypoint_headings', name, frame_id, stamp)
        marker.type = Marker.ARROW
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.18
        marker.pose.orientation = self._quaternion_from_yaw(yaw)
        marker.scale.x = self._marker_scale * 1.7
        marker.scale.y = self._marker_scale * 0.35
        marker.scale.z = self._marker_scale * 0.35
        if is_failed:
            marker.color = self._color(1.0, 0.15, 0.12, 0.95)
        elif is_active:
            marker.color = self._color(1.0, 0.92, 0.2, 0.95)
        elif is_completed:
            marker.color = self._color(0.05, 0.95, 0.55, 0.9)
        else:
            marker.color = self._color(0.1, 0.72, 1.0, 0.9)
        return marker

    def _make_text_marker(self, index, name, frame_id, stamp, x, y, is_active, is_completed, is_failed):
        marker = self._base_marker(index * 3 + 2, 'waypoint_labels', name, frame_id, stamp)
        marker.type = Marker.TEXT_VIEW_FACING
        marker.pose.position.x = x
        marker.pose.position.y = y
        marker.pose.position.z = 0.65
        marker.pose.orientation.w = 1.0
        marker.scale.z = self._marker_scale * 0.9
        marker.text = name
        if is_failed:
            marker.color = self._color(1.0, 0.25, 0.2, 1.0)
        elif is_active:
            marker.color = self._color(1.0, 0.95, 0.35, 1.0)
        elif is_completed:
            marker.color = self._color(0.2, 1.0, 0.6, 1.0)
        else:
            marker.color = self._color(1.0, 1.0, 1.0, 0.95)
        return marker

    def _base_marker(self, marker_id, namespace, name, frame_id, stamp):
        marker = Marker()
        marker.header.frame_id = frame_id
        marker.header.stamp = stamp
        marker.ns = namespace
        marker.id = marker_id
        marker.action = Marker.ADD
        marker.frame_locked = False
        marker.lifetime.sec = 0
        marker.lifetime.nanosec = 0
        marker.text = name
        return marker

    @staticmethod
    def _color(r, g, b, a):
        color = ColorRGBA()
        color.r = float(r)
        color.g = float(g)
        color.b = float(b)
        color.a = float(a)
        return color

    @staticmethod
    def _yaw_from_quaternion(quaternion: Quaternion) -> float:
        siny_cosp = 2.0 * (quaternion.w * quaternion.z + quaternion.x * quaternion.y)
        cosy_cosp = 1.0 - 2.0 * (quaternion.y * quaternion.y + quaternion.z * quaternion.z)
        return math.atan2(siny_cosp, cosy_cosp)

    @staticmethod
    def _normalize_angle(angle: float) -> float:
        return math.atan2(math.sin(angle), math.cos(angle))

    @staticmethod
    def _quaternion_from_yaw(yaw: float) -> Quaternion:
        quaternion = Quaternion()
        quaternion.x = 0.0
        quaternion.y = 0.0
        quaternion.z = math.sin(yaw / 2.0)
        quaternion.w = math.cos(yaw / 2.0)
        return quaternion


def main(args=None) -> None:
    rclpy.init(args=args)
    node = WaypointManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
