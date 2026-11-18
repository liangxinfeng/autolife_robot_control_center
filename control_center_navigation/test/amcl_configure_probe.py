"""Isolated AMCL configure-only probe. Never activate or publish robot topics."""
import os
import subprocess
import sys
import time

os.environ['ROS_DOMAIN_ID']='83'
os.environ['ROS_AUTOMATIC_DISCOVERY_RANGE']='LOCALHOST'
import rclpy
from lifecycle_msgs.srv import ChangeState
rclpy.init()
node=rclpy.create_node('amcl_isolated_probe')
for config in sys.argv[1:]:
    p=subprocess.Popen([os.environ.get('AMCL_PROBE_EXECUTABLE','/opt/ros/jazzy/lib/nav2_amcl/amcl'),'--ros-args','--params-file',config],
                       stdout=subprocess.PIPE,stderr=subprocess.STDOUT,text=True)
    client=node.create_client(ChangeState,'/amcl/change_state')
    success=False
    try:
        if client.wait_for_service(timeout_sec=8):
            request=ChangeState.Request();request.transition.id=1
            result=client.call_async(request)
            rclpy.spin_until_future_complete(node,result,timeout_sec=8)
            print(config, 'CONFIGURE=',result.result() if result.done() else 'NO RESPONSE')
            success=result.done() and result.result() is not None and result.result().success
        else: print(config,'NO SERVICE')
    finally:
        if p.poll() is None:p.terminate()
        try:output=p.communicate(timeout=5)[0]
        except subprocess.TimeoutExpired:p.kill();output=p.communicate()[0]
        print('EXIT=',p.returncode,output[-6000:])
        node.destroy_client(client)
    if not success: raise RuntimeError('AMCL failed configure')
node.destroy_node();rclpy.shutdown()
