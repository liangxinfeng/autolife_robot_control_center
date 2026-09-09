"""Isolated Domain 88 lifecycle/plugin loading; no motion goal is ever sent."""
import os
import signal
import subprocess
import sys
import time
from pathlib import Path

import rclpy
from lifecycle_msgs.srv import ChangeState

os.environ['ROS_DOMAIN_ID'] = '88'
os.environ['ROS_LOCALHOST_ONLY'] = '1'
os.environ['RMW_IMPLEMENTATION'] = 'rmw_cyclonedds_cpp'
os.environ.pop('CYCLONEDDS_URI', None)
binary, config = sys.argv[1:3]
log_path = Path(binary).parent / 'docking_lifecycle_smoke.log'
with log_path.open('w') as log:
    server = subprocess.Popen([binary, '--ros-args', '--params-file', config], stdout=log, stderr=log)
    rclpy.init()
    node = rclpy.create_node('docking_isolated_validation')
    try:
        client = node.create_client(ChangeState, '/docking_server/change_state')
        assert client.wait_for_service(timeout_sec=10), 'lifecycle service unavailable'
        for transition in (1, 3, 4, 2):
            request = ChangeState.Request()
            request.transition.id = transition
            future = client.call_async(request)
            rclpy.spin_until_future_complete(node, future, timeout_sec=12)
            assert future.done() and future.result().success, f'transition {transition} failed'
            print('lifecycle transition passed:', transition, flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()
        server.send_signal(signal.SIGINT)
        try:
            server.wait(timeout=8)
        except subprocess.TimeoutExpired:
            server.kill()
            server.wait()
print('No docking goals or robot commands sent. Log:', log_path)
