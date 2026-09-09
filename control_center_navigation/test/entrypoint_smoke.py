"""Exercise the real executable lifecycle, isolated from robot topics."""
import os
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID'] = '88'
import rclpy
from std_srvs.srv import Trigger
from std_msgs.msg import String

rclpy.init()
node = rclpy.create_node('relocalization_entrypoint_probe')
statuses = []
sub = node.create_subscription(String, '/entrypoint_test/status',
                               lambda msg: statuses.append(msg.data), 10)
process = subprocess.Popen([
    sys.argv[1], '--ros-args', '-r', '__node:=entrypoint_test',
    '-p', 'auto_start:=false', '-p', 'publish_initial_pose:=false',
    '-p', 'map_topic:=/entrypoint_test/no_map',
    '-p', 'scan_topic:=/entrypoint_test/no_front',
    '-p', 'rear_scan_topic:=/entrypoint_test/no_rear',
    '-p', 'status_topic:=/entrypoint_test/status',
])
try:
    client = node.create_client(Trigger, '/entrypoint_test/relocalize')
    assert client.wait_for_service(timeout_sec=10), 'live process has no retry service'
    for _ in range(2):
        future = client.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(node, future, timeout_sec=5)
        assert future.done() and future.result().success, 'retry service did not respond'
        deadline = time.monotonic() + 1
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
    assert any('manual retry requested' in s for s in statuses), 'no callback status'
    assert process.poll() is None, 'executable exited unexpectedly'
    print('PASS: real entrypoint remains alive and handles repeated retry requests')
finally:
    process.terminate()
    try:
        process.wait(timeout=5)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=5)
    node.destroy_node()
    rclpy.shutdown()
