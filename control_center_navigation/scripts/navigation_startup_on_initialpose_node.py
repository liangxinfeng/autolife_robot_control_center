#!/usr/bin/env python3

from geometry_msgs.msg import PoseWithCovarianceStamped
from nav2_msgs.srv import ManageLifecycleNodes
from rclpy.duration import Duration
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy


class NavigationStartupOnInitialPoseNode(Node):
    def __init__(self) -> None:
        super().__init__('navigation_startup_on_initialpose')

        self.declare_parameter('initialpose_topic', '/initialpose')
        self.declare_parameter(
            'manage_nodes_service',
            '/lifecycle_manager_navigation/manage_nodes',
        )
        self.declare_parameter('amcl_pose_topic', '/amcl_pose')
        self.declare_parameter('startup_delay_sec', 1.0)
        self.declare_parameter('require_amcl_convergence', True)
        self.declare_parameter('convergence_stable_samples', 3)
        self.declare_parameter('min_convergence_wait_sec', 1.0)
        self.declare_parameter('max_covariance_x', 0.50)
        self.declare_parameter('max_covariance_y', 0.50)
        self.declare_parameter('max_covariance_yaw', 0.40)

        initialpose_topic = self.get_parameter('initialpose_topic').value
        manage_nodes_service = self.get_parameter('manage_nodes_service').value
        amcl_pose_topic = self.get_parameter('amcl_pose_topic').value
        self._startup_delay_sec = float(self.get_parameter('startup_delay_sec').value)
        self._require_amcl_convergence = bool(
            self.get_parameter('require_amcl_convergence').value
        )
        self._convergence_stable_samples = max(
            1,
            int(self.get_parameter('convergence_stable_samples').value),
        )
        self._min_convergence_wait = Duration(
            seconds=float(self.get_parameter('min_convergence_wait_sec').value)
        )
        self._max_covariance_x = float(self.get_parameter('max_covariance_x').value)
        self._max_covariance_y = float(self.get_parameter('max_covariance_y').value)
        self._max_covariance_yaw = float(
            self.get_parameter('max_covariance_yaw').value
        )

        self._startup_requested = False
        self._startup_complete = False
        self._startup_timer = None
        self._waiting_for_convergence = False
        self._stable_sample_count = 0
        self._initialpose_received_at = None
        self._latest_amcl_covariance = None

        self._manage_nodes_client = self.create_client(
            ManageLifecycleNodes,
            manage_nodes_service,
        )
        self._initialpose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            initialpose_topic,
            self._handle_initial_pose,
            10,
        )
        # A second durable subscription recovers the automatic pose if DDS
        # discovery completes after its short publication burst. Keep the
        # volatile subscription above for RViz/manual initial-pose publishers.
        self._initialpose_latched_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            initialpose_topic,
            self._handle_initial_pose,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._amcl_pose_sub = self.create_subscription(
            PoseWithCovarianceStamped,
            amcl_pose_topic,
            self._handle_amcl_pose,
            QoSProfile(
                depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL,
            ),
        )
        self._convergence_timer = self.create_timer(
            0.5,
            self._evaluate_amcl_convergence,
        )

        self.get_logger().info(
            'Waiting for initial pose on '
            f'{initialpose_topic}; AMCL convergence topic={amcl_pose_topic}, '
            f'required={self._require_amcl_convergence}; '
            f'navigation service={manage_nodes_service}'
        )

    def _handle_initial_pose(self, _msg: PoseWithCovarianceStamped) -> None:
        if self._startup_complete:
            return

        self._initialpose_received_at = self.get_clock().now()
        self._stable_sample_count = 0
        if self._require_amcl_convergence:
            self._waiting_for_convergence = True
            self.get_logger().info(
                'Initial pose received; waiting for stable AMCL covariance before navigation startup'
            )
            return

        self.get_logger().info('Initial pose received; scheduling navigation startup')
        self._schedule_navigation_startup()

    def _handle_amcl_pose(self, msg: PoseWithCovarianceStamped) -> None:
        self._latest_amcl_covariance = list(msg.pose.covariance)

    def _evaluate_amcl_convergence(self) -> None:
        if (
            not self._waiting_for_convergence
            or self._startup_requested
            or self._startup_complete
            or self._initialpose_received_at is None
            or self._latest_amcl_covariance is None
        ):
            return

        if self.get_clock().now() - self._initialpose_received_at < self._min_convergence_wait:
            return

        covariance = self._latest_amcl_covariance
        converged = (
            0.0 <= covariance[0] <= self._max_covariance_x
            and 0.0 <= covariance[7] <= self._max_covariance_y
            and 0.0 <= covariance[35] <= self._max_covariance_yaw
        )
        if not converged:
            if self._stable_sample_count:
                self.get_logger().info(
                    'AMCL covariance rose above threshold; resetting convergence counter'
                )
            self._stable_sample_count = 0
            return

        self._stable_sample_count += 1
        self.get_logger().info(
            'AMCL convergence sample '
            f'{self._stable_sample_count}/{self._convergence_stable_samples}: '
            f'cov=({covariance[0]:.4f}, {covariance[7]:.4f}, {covariance[35]:.4f})'
        )
        if self._stable_sample_count < self._convergence_stable_samples:
            return

        self._waiting_for_convergence = False
        self.get_logger().info('AMCL localization is stable; scheduling navigation startup')
        self._schedule_navigation_startup()

    def _schedule_navigation_startup(self) -> None:
        if self._startup_requested or self._startup_complete:
            return
        self._startup_requested = True

        if self._startup_delay_sec <= 0.0:
            self._request_navigation_startup()
            return

        self._startup_timer = self.create_timer(
            self._startup_delay_sec,
            self._request_navigation_startup,
        )

    def _request_navigation_startup(self) -> None:
        if self._startup_complete:
            return

        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None

        if not self._manage_nodes_client.service_is_ready():
            if not self._manage_nodes_client.wait_for_service(timeout_sec=1.0):
                self.get_logger().warning(
                    'Navigation lifecycle service is not ready; retrying in one second'
                )
                self._startup_requested = False
                self._startup_timer = self.create_timer(
                    1.0,
                    self._retry_navigation_startup,
                )
                return

        request = ManageLifecycleNodes.Request()
        request.command = ManageLifecycleNodes.Request.STARTUP
        future = self._manage_nodes_client.call_async(request)
        future.add_done_callback(self._handle_startup_response)

    def _retry_navigation_startup(self) -> None:
        if self._startup_timer is not None:
            self._startup_timer.cancel()
            self._startup_timer = None
        self._schedule_navigation_startup()

    def _handle_startup_response(self, future: rclpy.task.Future) -> None:
        try:
            response = future.result()
        except Exception as exc:
            self._startup_requested = False
            self.get_logger().error(f'Navigation startup request failed: {exc}')
            return

        if response.success:
            self._startup_complete = True
            self.get_logger().info('Navigation lifecycle started successfully')
            self._convergence_timer.cancel()
            self.destroy_subscription(self._initialpose_sub)
            self.destroy_subscription(self._initialpose_latched_sub)
            self.destroy_subscription(self._amcl_pose_sub)
            return

        self._startup_requested = False
        self.get_logger().warning(
            'Navigation lifecycle manager reported startup failure; retrying in one second'
        )
        self._startup_timer = self.create_timer(
            1.0,
            self._retry_navigation_startup,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = NavigationStartupOnInitialPoseNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
