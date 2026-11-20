import time
from concurrent.futures import ThreadPoolExecutor
import pytest
import rclpy
from sensor_msgs.msg import LaserScan
from control_center_shell import preflight


@pytest.mark.parametrize('suffix,front,expected', [
    ('0_345', 's3_front_scan_frame', None),
    ('0_300', 'front_laser_frame', None),
    ('0_345', 'front_laser_frame', '坐标系不匹配'),
    ('0_300', 's3_front_scan_frame', '坐标系不匹配'),
])
def test_scan_profile_is_strict_and_supports_s3(suffix, front, expected):
    # Isolated ROS domain only: synthetic sensor publishers, no actuators.
    rclpy.init()
    node = rclpy.create_node('test_lidar_profile_publisher')
    publishers = {side: node.create_publisher(LaserScan, f'/topic_gv_{side}_lidar_{suffix}', 1)
                  for side in ('front', 'rear')}
    def send():
        stamp = node.get_clock().now().to_msg()
        for side, publisher in publishers.items():
            scan = LaserScan()
            scan.header.stamp = stamp
            scan.header.frame_id = front if side == 'front' else 'rear_laser_frame'
            scan.range_min, scan.range_max = .04, 16.
            scan.ranges = [1.] * 100
            publisher.publish(scan)
    node.create_timer(.05, send)
    try:
        with ThreadPoolExecutor(max_workers=1) as pool:
            job = pool.submit(preflight._check_lidars, suffix, 1.8, lambda _: None)
            while not job.done(): rclpy.spin_once(node, timeout_sec=.02)
            result = job.result()
        assert result is None if expected is None else expected in result
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_s3_projection_failure_does_not_restart_chassis(monkeypatch, tmp_path):
    monkeypatch.setattr(preflight, 'STATE_DIR', tmp_path)
    monkeypatch.setattr(preflight, '_processes', lambda: {})
    monkeypatch.setattr(preflight, '_conflicts', lambda *a: {})
    monkeypatch.setattr(preflight, '_check_joint_feedback', lambda *a, **k: None)
    monkeypatch.setattr(preflight, '_external_map_publishers', lambda *a: [])
    monkeypatch.setattr(preflight, '_gv_control_state', lambda: 'active')
    monkeypatch.setattr(preflight, '_check_lidars', lambda *a: 'front scan missing')
    def unexpected_restart(*args, **kwargs):
        pytest.fail('S3 projection failure must not restart shared chassis service')
    monkeypatch.setattr(preflight, '_recover_gv_control', unexpected_restart)
    status, detail = preflight.run('0_345', emit=lambda _: None)
    assert status == 'WARN' and 'front scan missing' in detail


@pytest.mark.parametrize('name,gid,ready', [('/map_server', 'ab' * 16, True),
    ('/_NODE_NAME_UNKNOWN_', 'ab' * 16, False), ('/map_server', '', False)])
def test_map_endpoint_discovery(monkeypatch, name, gid, ready):
    from control_center_shell.resource_observer import Endpoint
    def read(topics, timeout, anchor):
        assert anchor.endswith('_0_345')
        return [Endpoint('/map', name, gid)]
    monkeypatch.setattr(preflight._observer.GRAPH, 'read', read)
    if ready:
        assert preflight._map_publishers(suffix='0_345') == [name]
    else:
        with pytest.raises(RuntimeError, match='发现中'):
            preflight._map_publishers(suffix='0_345')
