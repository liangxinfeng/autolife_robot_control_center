"""Identity resolution tests for the chassis ownership lease keys."""

from control_center_chassis_guard.chassis_control_lease import (
    resolve_chassis_identity,
)


class TestResolveChassisIdentity:
    def test_env_only(self):
        assert resolve_chassis_identity(
            environ={'ROS_DOMAIN_ID': '0', 'ROBOT_ID': '306'}) == ('0', '306')

    def test_topic_suffix_only(self):
        assert resolve_chassis_identity(
            topic_suffix='0_307', environ={}) == ('0', '307')

    def test_topic_hint_extraction(self):
        assert resolve_chassis_identity(
            topic_hint='/topic_gv_front_lidar_0_328', environ={}) == ('0', '328')

    def test_env_wins_over_suffix(self):
        assert resolve_chassis_identity(
            topic_suffix='0_307',
            environ={'ROS_DOMAIN_ID': '5', 'ROBOT_ID': '306'}) == ('5', '306')

    def test_partial_env_filled_by_suffix(self):
        assert resolve_chassis_identity(
            topic_suffix='0_307', environ={'ROBOT_ID': '306'}) == ('0', '306')

    def test_unresolved_returns_empty_strings(self):
        assert resolve_chassis_identity(environ={}) == ('', '')

    def test_garbage_suffix_is_ignored(self):
        assert resolve_chassis_identity(topic_suffix='abc', environ={}) == ('', '')

    def test_domain_defaults_to_zero_for_ros_semantics(self):
        from control_center_chassis_guard.chassis_control_lease import (
            ChassisControlLease,
        )
        from fake_redis import FakeRedis
        lease = ChassisControlLease(
            redis_client=FakeRedis(), robot_id='306', domain_id=None,
            environ={}, lease_ttl_sec=0.3, renew_period_sec=0.05)
        assert lease.domain_id == '0'
        assert lease._vendor_key == 'shared_params:0:306:is_navigating'
        assert lease._owner_key == 'shared_params:0:306:chassis_owner'
