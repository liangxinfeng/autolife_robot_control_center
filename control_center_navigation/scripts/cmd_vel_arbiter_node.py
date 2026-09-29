#!/usr/bin/env python3
import json
import math
import time
from charger_waypoints import is_charger_waypoint
from pathlib import Path
import sys
sys.path.insert(0, str(Path(__file__).resolve().parent))
from pose_continuity import NavigationContinuityMonitor

from geometry_msgs.msg import Twist
import rclpy
from rclpy.clock import Clock
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Bool, String
from action_msgs.srv import CancelGoal
from std_srvs.srv import Trigger


class CmdVelArbiterNode(Node):
    def __init__(self) -> None:
        super().__init__('cmd_vel_arbiter')

        self.declare_parameter('nav_input_topic', '/nav_cmd_vel')
        self.declare_parameter('arm_input_topic', '/arm_cmd_vel_vr_0_307')
        self.declare_parameter('output_topic', '/arm_cmd_vel_raw_0_307')
        self.declare_parameter('nav_priority_timeout_sec', 0.5)
        self.declare_parameter('manual_override_topic', '/manual_cmd_vel_active')
        self.declare_parameter(
            'control_center_input_topic', '/control_center/manual_cmd_vel'
        )
        self.declare_parameter(
            'operator_heartbeat_topic', '/control_center/operator_heartbeat'
        )
        self.declare_parameter('operator_heartbeat_timeout_sec', 0.30)
        self.declare_parameter('manual_override_timeout_sec', 0.45)
        self.declare_parameter('straight_docking_enabled', True)
        self.declare_parameter('docking_status_topic', '/waypoints/navigation_status')
        self.declare_parameter('straight_docking_min_speed', 0.04)
        self.declare_parameter('straight_docking_stop_distance', 0.06)
        self.declare_parameter('straight_docking_status_timeout_sec', 0.8)
        self.declare_parameter('straight_docking_progress_timeout_sec', 2.0)
        self.declare_parameter('straight_docking_min_progress', 0.01)
        self.declare_parameter('straight_docking_progress_stop_enabled', False)
        self.declare_parameter('charger_waypoint_name', '充电桩')
        self.declare_parameter('staging_approach_slowdown_enabled', True)
        self.declare_parameter('staging_approach_slow_distance', 1.2)
        self.declare_parameter('staging_approach_max_linear_speed', 0.12)
        self.declare_parameter('staging_approach_max_angular_speed', 0.35)
        self.declare_parameter('staging_final_heading_distance', 0.35)
        self.declare_parameter('staging_final_max_linear_speed', 0.05)
        self.declare_parameter('staging_final_max_angular_speed', 0.10)
        self.declare_parameter('battery_topic', '/topic_gv_battery_0_307')
        self.declare_parameter('charging_current_threshold', 0.10)
        self.declare_parameter('charging_use_current', False)
        self.declare_parameter('charging_confirm_samples', 2)
        self.declare_parameter('charging_stop_hold_sec', 3.0)
        self.declare_parameter('charging_stop_publish_hz', 20.0)

        nav_input_topic = self.get_parameter('nav_input_topic').get_parameter_value().string_value
        arm_input_topic = self.get_parameter('arm_input_topic').get_parameter_value().string_value
        output_topic = self.get_parameter('output_topic').get_parameter_value().string_value
        manual_override_topic = self.get_parameter(
            'manual_override_topic'
        ).get_parameter_value().string_value
        control_center_input_topic = str(
            self.get_parameter('control_center_input_topic').value
        )
        operator_heartbeat_topic = str(
            self.get_parameter('operator_heartbeat_topic').value
        )
        operator_heartbeat_timeout_sec = max(
            0.10,
            float(self.get_parameter('operator_heartbeat_timeout_sec').value),
        )
        nav_priority_timeout_sec = (
            self.get_parameter('nav_priority_timeout_sec').get_parameter_value().double_value
        )
        manual_override_timeout_sec = float(
            self.get_parameter('manual_override_timeout_sec').value
        )
        self._straight_docking_enabled = self.get_parameter('straight_docking_enabled').value
        docking_status_topic = self.get_parameter('docking_status_topic').value
        self._straight_docking_min_speed = self.get_parameter(
            'straight_docking_min_speed'
        ).value
        self._straight_docking_stop_distance = self.get_parameter(
            'straight_docking_stop_distance'
        ).value
        straight_docking_status_timeout_sec = self.get_parameter(
            'straight_docking_status_timeout_sec'
        ).value
        straight_docking_progress_timeout_sec = self.get_parameter(
            'straight_docking_progress_timeout_sec'
        ).value
        self._straight_docking_min_progress = self.get_parameter(
            'straight_docking_min_progress'
        ).value
        self._straight_docking_progress_stop_enabled = self.get_parameter(
            'straight_docking_progress_stop_enabled'
        ).value
        self._charger_waypoint_name = self.get_parameter('charger_waypoint_name').value
        self._staging_approach_slowdown_enabled = self.get_parameter(
            'staging_approach_slowdown_enabled'
        ).value
        self._staging_approach_slow_distance = self.get_parameter(
            'staging_approach_slow_distance'
        ).value
        self._staging_approach_max_linear_speed = self.get_parameter(
            'staging_approach_max_linear_speed'
        ).value
        self._staging_approach_max_angular_speed = self.get_parameter(
            'staging_approach_max_angular_speed'
        ).value
        self._staging_final_heading_distance = self.get_parameter(
            'staging_final_heading_distance'
        ).value
        self._staging_final_max_linear_speed = self.get_parameter(
            'staging_final_max_linear_speed'
        ).value
        self._staging_final_max_angular_speed = self.get_parameter(
            'staging_final_max_angular_speed'
        ).value
        battery_topic = self.get_parameter('battery_topic').value
        self._charging_current_threshold = self.get_parameter(
            'charging_current_threshold'
        ).value
        self._charging_use_current = self.get_parameter('charging_use_current').value
        self._charging_confirm_samples = self.get_parameter(
            'charging_confirm_samples'
        ).value
        charging_stop_hold_sec = self.get_parameter('charging_stop_hold_sec').value
        self._charging_stop_hold = Duration(seconds=charging_stop_hold_sec)
        charging_stop_publish_hz = max(
            1.0,
            float(self.get_parameter('charging_stop_publish_hz').value),
        )

        self._clock = Clock()
        self._nav_priority_timeout = Duration(seconds=nav_priority_timeout_sec)
        self._manual_override_timeout = Duration(seconds=manual_override_timeout_sec)
        self._manual_override_active = False
        self._last_manual_override_stamp = None
        self._operator_session = ''
        self._last_operator_heartbeat = None
        self._operator_heartbeat_timeout = operator_heartbeat_timeout_sec
        self._operator_lost = False
        self._last_external_manual_at = None
        self._last_control_center_nonzero = False
        self._cancel_goal_clients = [
            self.create_client(CancelGoal, f'/{name}/_action/cancel_goal')
            for name in (
                'navigate_to_pose', 'navigate_through_poses', 'follow_waypoints'
            )
        ]
        self._cancel_waypoint_client = self.create_client(
            Trigger, '/cancel_navigation'
        )
        self._straight_docking_status_timeout = Duration(
            seconds=straight_docking_status_timeout_sec
        )
        self._straight_docking_progress_timeout = Duration(
            seconds=straight_docking_progress_timeout_sec
        )
        self._last_nav_stamp = None
        self._last_docking_status_stamp = None
        self._docking_distance = None
        self._best_docking_distance = None
        self._last_docking_progress_stamp = None
        self._straight_docking_blocked = False
        self._dock_output_inhibit = False
        self._staging_approach_stamp = None
        self._staging_approach_distance = None
        self._charging_sample_count = 0
        self._charging_detected = False
        self._charging_armed = True
        self._charging_hold_until = None
        self._last_battery_log_stamp = None

        self._publisher = self.create_publisher(Twist, output_topic, 10)
        self._continuity = NavigationContinuityMonitor(self)
        self.create_timer(.1, self._publish_localization_stop)
        self._nav_subscription = self.create_subscription(
            Twist,
            nav_input_topic,
            self._handle_nav_cmd_vel,
            10,
        )
        self._arm_subscription = self.create_subscription(
            Twist,
            arm_input_topic,
            self._handle_arm_cmd_vel,
            10,
        )
        self._control_center_subscription = self.create_subscription(
            Twist,
            control_center_input_topic,
            self._handle_control_center_cmd_vel,
            10,
        )
        self._operator_heartbeat_subscription = self.create_subscription(
            String,
            operator_heartbeat_topic,
            self._handle_operator_heartbeat,
            10,
        )
        self._operator_watchdog_timer = self.create_timer(
            0.05, self._check_operator_heartbeat
        )
        self._manual_override_subscription = self.create_subscription(
            Bool,
            manual_override_topic,
            self._handle_manual_override,
            10,
        )
        self._docking_status_subscription = self.create_subscription(
            String,
            docking_status_topic,
            self._handle_docking_status,
            20,
        )
        self._battery_subscription = self.create_subscription(
            BatteryState,
            battery_topic,
            self._handle_battery_state,
            qos_profile_sensor_data,
        )
        self._stop_timer = self.create_timer(
            1.0 / charging_stop_publish_hz,
            self._publish_charging_stop_if_needed,
        )

        self.get_logger().info(
            f'Arbitrating cmd_vel: nav={nav_input_topic}, arm={arm_input_topic}, output={output_topic}'
        )
        self.get_logger().info(
            f'Control-center heartbeat watchdog: topic={operator_heartbeat_topic}, '
            f'timeout={operator_heartbeat_timeout_sec:.2f}s'
        )
        self.get_logger().info(
            f'Explicit manual override: topic={manual_override_topic}, '
            f'timeout={manual_override_timeout_sec:.2f}s'
        )
        self.get_logger().info(
            f'Charging stop lock: hold={charging_stop_hold_sec:.1f}s, '
            f'publish_rate={charging_stop_publish_hz:.1f}Hz'
        )
        self.get_logger().info(
            f'Staging approach slowdown: enabled={self._staging_approach_slowdown_enabled}, '
            f'distance={self._staging_approach_slow_distance:.2f}m, '
            f'linear={self._staging_approach_max_linear_speed:.2f}m/s, '
            f'angular={self._staging_approach_max_angular_speed:.2f}rad/s, '
            f'final_distance={self._staging_final_heading_distance:.2f}m, '
            f'final_linear={self._staging_final_max_linear_speed:.2f}m/s, '
            f'final_angular={self._staging_final_max_angular_speed:.2f}rad/s'
        )

    def _handle_docking_status(self, msg: String) -> None:
        try:
            payload = json.loads(msg.data)
        except (TypeError, ValueError):
            return

        state = payload.get('state')
        if state in ('docking_failed', 'docking_charge_wait', 'paused'):
            self._dock_output_inhibit = True
        elif state in ('started', 'docking_started', 'undocking', 'short_distance_started'):
            self._dock_output_inhibit = False
        waypoint_name = payload.get('active_waypoint') or payload.get('waypoint_name') or ''
        if (
            state == 'docking_staging_approach'
            and is_charger_waypoint(waypoint_name, self._charger_waypoint_name)
        ):
            distance = payload.get('distance_remaining')
            if distance is not None:
                self._staging_approach_distance = float(distance)
                self._staging_approach_stamp = self._clock.now()
            self._last_docking_status_stamp = None
            self._docking_distance = None
            self._best_docking_distance = None
            self._last_docking_progress_stamp = None
            self._straight_docking_blocked = False
            return

        self._staging_approach_stamp = None
        self._staging_approach_distance = None

        if state != 'docking_control':
            self._last_docking_status_stamp = None
            self._docking_distance = None
            self._best_docking_distance = None
            self._last_docking_progress_stamp = None
            self._straight_docking_blocked = False
            self._last_battery_log_stamp = None
            return

        distance = payload.get('distance_remaining')
        if distance is None:
            return
        distance = float(distance)
        now = self._clock.now()
        self._docking_distance = distance
        self._last_docking_status_stamp = now
        if (
            self._best_docking_distance is None
            or distance <= self._best_docking_distance - self._straight_docking_min_progress
        ):
            self._best_docking_distance = distance
            self._last_docking_progress_stamp = now
            self._straight_docking_blocked = False
        elif (
            self._last_docking_progress_stamp is not None
            and now - self._last_docking_progress_stamp > self._straight_docking_progress_timeout
            and self._straight_docking_progress_stop_enabled
            and not self._straight_docking_blocked
        ):
            self._straight_docking_blocked = True
            self.get_logger().warning(
                f'精确入桩连续未前进，停止底盘输出: distance={distance:.3f} m'
            )

    def _handle_battery_state(self, msg: BatteryState) -> None:
        now = self._clock.now()
        if (
            self._last_battery_log_stamp is None
            or now - self._last_battery_log_stamp >= Duration(seconds=1.0)
        ):
            self._last_battery_log_stamp = now
            self.get_logger().info(
                f'入桩电池数据: status={msg.power_supply_status}, '
                f'current={msg.current:.3f} A, voltage={msg.voltage:.2f} V'
            )

        status_charging = msg.power_supply_status in (
            BatteryState.POWER_SUPPLY_STATUS_CHARGING,
            BatteryState.POWER_SUPPLY_STATUS_FULL,
        )
        current_charging = (
            self._charging_use_current
            and math.isfinite(float(msg.current))
            and float(msg.current) >= self._charging_current_threshold
        )
        if not status_charging and not current_charging:
            self._charging_sample_count = 0
            self._charging_detected = False
            if not self._charging_armed:
                self._charging_armed = True
                self.get_logger().info('充电状态已解除，下一次入桩检测已重新武装')
            return

        if not self._charging_armed:
            return

        self._charging_sample_count += 1
        if (
            self._charging_sample_count >= self._charging_confirm_samples
            and not self._charging_detected
        ):
            self._charging_detected = True
            self._charging_armed = False
            self._charging_hold_until = self._clock.now() + self._charging_stop_hold
            self.get_logger().info(
                f'检测到充电状态，立即停止入桩: status={msg.power_supply_status}'
            )

    def _straight_docking_is_active(self) -> bool:
        if (
            not self._straight_docking_enabled
            or self._last_docking_status_stamp is None
            or self._docking_distance is None
        ):
            return False
        return (
            self._clock.now() - self._last_docking_status_stamp
        ) <= self._straight_docking_status_timeout

    def _staging_approach_slowdown_is_active(self) -> bool:
        if (
            not self._staging_approach_slowdown_enabled
            or self._staging_approach_stamp is None
            or self._staging_approach_distance is None
            or self._staging_approach_distance > self._staging_approach_slow_distance
        ):
            return False
        return (
            self._clock.now() - self._staging_approach_stamp
        ) <= self._straight_docking_status_timeout

    def _filter_nav_cmd_vel(self, msg: Twist) -> Twist:
        if getattr(self, '_dock_output_inhibit', False):
            return Twist()
        charging_hold_active = self._charging_hold_active()
        if self._charging_detected and not charging_hold_active:
            self._charging_detected = False
        if charging_hold_active:
            return Twist()
        if self._staging_approach_slowdown_is_active():
            return self._limit_staging_approach_cmd_vel(msg)
        if not self._straight_docking_is_active():
            return msg
        if self._docking_distance <= self._straight_docking_stop_distance:
            return Twist()

        filtered = Twist()
        if not self._straight_docking_blocked:
            # Never turn a controller/collision-check STOP into reverse motion,
            # or amplify a slow request. Keep the docking controller's steering.
            values = (float(msg.linear.x), float(msg.linear.y), float(msg.angular.z))
            if not all(math.isfinite(v) for v in values):
                return filtered
            max_linear = abs(float(self._staging_approach_max_linear_speed))
            max_angular = abs(float(self._staging_approach_max_angular_speed))
            if values[0] > 0.0:  # no unrequested forward/retry movement at contact
                return filtered
            # Uniform reduction preserves the controller's collision-checked
            # curvature. Do not create an unchecked lateral steering command.
            scale = min(1.0, max_linear / max(math.hypot(*values[:2]), 1e-9),
                        max_angular / max(abs(values[2]), 1e-9))
            filtered.linear.x, filtered.linear.y, filtered.angular.z = (v * scale for v in values)
        return filtered

    def _limit_staging_approach_cmd_vel(self, msg: Twist) -> Twist:
        filtered = Twist()
        max_linear = abs(float(self._staging_approach_max_linear_speed))
        max_angular = abs(float(self._staging_approach_max_angular_speed))
        if (
            self._staging_approach_distance is not None
            and self._staging_approach_distance <= self._staging_final_heading_distance
        ):
            max_linear = min(max_linear, abs(float(self._staging_final_max_linear_speed)))
            max_angular = min(max_angular, abs(float(self._staging_final_max_angular_speed)))
        filtered.linear.x = max(-max_linear, min(max_linear, float(msg.linear.x)))
        # Holonomic chassis must retain lateral alignment at the staging pose.
        filtered.linear.y = max(-max_linear, min(max_linear, float(msg.linear.y)))
        filtered.linear.z = 0.0
        filtered.angular.x = 0.0
        filtered.angular.y = 0.0
        filtered.angular.z = max(-max_angular, min(max_angular, float(msg.angular.z)))
        return filtered

    def _charging_hold_active(self) -> bool:
        return (
            self._charging_hold_until is not None
            and self._clock.now() <= self._charging_hold_until
        )

    def _publish_charging_stop_if_needed(self) -> None:
        if self._operator_lost:
            return
        if getattr(self, '_dock_output_inhibit', False) and not self._manual_override_is_active():
            self._publisher.publish(Twist())
            return
        if not self._charging_hold_active():
            return
        self._publisher.publish(Twist())

    def _nav_has_priority(self) -> bool:
        if self._last_nav_stamp is None:
            return False
        return (self._clock.now() - self._last_nav_stamp) <= self._nav_priority_timeout

    def _manual_override_is_active(self) -> bool:
        if not self._manual_override_active or self._last_manual_override_stamp is None:
            return False
        return (
            self._clock.now() - self._last_manual_override_stamp
        ) <= self._manual_override_timeout

    def _handle_manual_override(self, msg: Bool) -> None:
        was_active = self._manual_override_is_active()
        self._manual_override_active = bool(msg.data)
        self._last_manual_override_stamp = self._clock.now()
        is_active = self._manual_override_is_active()
        if is_active != was_active:
            # A zero-speed handoff prevents a command from the previous owner
            # being carried into the new arbitration mode.
            self._publisher.publish(Twist())
            self.get_logger().info(
                'Manual chassis override engaged'
                if is_active else 'Manual chassis override released'
            )

    def _handle_nav_cmd_vel(self, msg: Twist) -> None:
        if self._operator_lost:
            return
        self._last_nav_stamp = self._clock.now()
        if self._manual_override_is_active():
            return
        if not self._continuity.permits_navigation():
            self._publisher.publish(Twist())
            return
        output = self._filter_nav_cmd_vel(msg)
        self._publisher.publish(output)
        self._last_control_center_nonzero = not self._is_zero_twist(output)

    def _publish_localization_stop(self):
        if (self._nav_has_priority() and not self._manual_override_is_active()
                and not self._continuity.permits_navigation()):
            self._publisher.publish(Twist())

    def _handle_arm_cmd_vel(self, msg: Twist) -> None:
        self._last_external_manual_at = time.monotonic()
        if self._charging_hold_active():
            self._publisher.publish(Twist())
            return
        if self._manual_override_is_active():
            self._publisher.publish(msg)
            return
        if self._nav_has_priority():
            return
        self._publisher.publish(msg)

    def _handle_control_center_cmd_vel(self, msg: Twist) -> None:
        if self._operator_lost or not self._operator_heartbeat_fresh():
            return
        if self._charging_hold_active():
            self._publisher.publish(Twist())
            self._last_control_center_nonzero = False
            return
        if self._manual_override_is_active() or self._nav_has_priority():
            return
        output = self._filter_nav_cmd_vel(msg)
        self._publisher.publish(output)
        self._last_control_center_nonzero = not self._is_zero_twist(output)

    @staticmethod
    def _is_zero_twist(message):
        return all(abs(float(value)) <= 1e-7 for value in (
            message.linear.x, message.linear.y, message.linear.z,
            message.angular.x, message.angular.y, message.angular.z,
        ))

    def _handle_operator_heartbeat(self, message):
        session = str(message.data).strip()
        if not session:
            return
        if not self._operator_session:
            self._operator_session = session
            self.get_logger().info('Control-center operator session is active')
        elif session != self._operator_session:
            self.get_logger().error(
                'A different control-center session appeared; keeping this '
                'navigation process locked to its original owner'
            )
            self._lose_operator()
            return
        self._last_operator_heartbeat = time.monotonic()

    def _operator_heartbeat_fresh(self, now=None):
        if self._last_operator_heartbeat is None:
            return False
        current = time.monotonic() if now is None else now
        return current - self._last_operator_heartbeat <= self._operator_heartbeat_timeout

    def _check_operator_heartbeat(self):
        if self._last_operator_heartbeat is None or self._operator_lost:
            return
        now = time.monotonic()
        if self._operator_heartbeat_fresh(now):
            return
        self._lose_operator()

    def _lose_operator(self):
        if self._operator_lost:
            return
        self._operator_lost = True
        self._last_nav_stamp = None
        self.get_logger().error(
            'Control-center heartbeat lost; canceling navigation and blocking its commands'
        )
        for client in self._cancel_goal_clients:
            if client.service_is_ready():
                try:
                    client.call_async(CancelGoal.Request())
                except Exception as error:
                    self.get_logger().warning(
                        f'Navigation goal cancellation request failed: {error}'
                    )
        if self._cancel_waypoint_client.service_is_ready():
            try:
                self._cancel_waypoint_client.call_async(Trigger.Request())
            except Exception as error:
                self.get_logger().warning(
                    f'Waypoint cancellation request failed: {error}'
                )

        external_manual_fresh = (
            self._last_external_manual_at is not None
            and now - self._last_external_manual_at <= 0.30
        )
        if self._last_control_center_nonzero and not external_manual_fresh:
            # One transition stop while this arbiter owns the active command;
            # then remain silent so VR/other publishers can take over.
            self._publisher.publish(Twist())
            self._last_control_center_nonzero = False
            self.get_logger().warning(
                'Published one bounded stop command; watchdog is now silent'
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelArbiterNode()
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
