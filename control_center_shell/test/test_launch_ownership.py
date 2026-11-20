"""Use the actual ROS launch scheduler, with a sleeping process as its only payload."""
import os
from pathlib import Path
import subprocess
import sys
import time
from unittest.mock import Mock

from control_center_service_manager import joint_reset_once as reset


def test_reset_rejects_existing_owner_before_publisher(monkeypatch):
    monkeypatch.setattr(reset.rclpy, 'init', lambda **kw: None)
    monkeypatch.setattr(reset.rclpy, 'ok', lambda: False)
    monkeypatch.setattr(reset, 'claim', Mock(side_effect=RuntimeError('busy')))
    publisher=Mock()
    monkeypatch.setattr(reset,'JointResetOnce',publisher)
    assert reset.main()==2
    publisher.assert_not_called()


def test_real_ros_launch_blocks_competing_launch(tmp_path):
    env=dict(os.environ,HOME=str(tmp_path))
    ready=tmp_path/'ready'
    payload=("from pathlib import Path;import os,time;"
             "assert os.environ.get('AUTOLIFE_CONTROLLER_GENERATION_chassis');"
             f"Path({str(ready)!r}).write_text(str(os.getpid()));time.sleep(30)")
    for module in ('navigation','slam'):
        path=tmp_path/(module+'.launch.py')
        path.write_text("from launch import LaunchDescription\nfrom launch.actions import ExecuteProcess\n"
                        "from control_center_module_lease import controller_lease\n"
                        f"def generate_launch_description():\n    return LaunchDescription([controller_lease({module!r}),"
                        f"ExecuteProcess(cmd={[sys.executable,'-c',payload]!r})])\n")
    owner=subprocess.Popen(['ros2','launch',str(tmp_path/'navigation.launch.py')],env=env,
                           stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL)
    try:
        end=time.monotonic()+8
        while not ready.exists() and time.monotonic()<end and owner.poll() is None:
            time.sleep(.05)
        assert ready.exists()
        other=subprocess.run(['ros2','launch',str(tmp_path/'slam.launch.py')],env=env,
                             capture_output=True,text=True,timeout=8)
        assert other.returncode!=0 and '正在清理' in (other.stdout+other.stderr)
        child=int(ready.read_text())
        owner.kill()
        owner.wait(timeout=5)
        end=time.monotonic()+5
        while time.monotonic()<end:
            stat=Path('/proc')/str(child)/'stat'
            if not stat.exists() or stat.read_text().rsplit(')',1)[1].strip().startswith('Z'):
                return
            time.sleep(.05)
        raise AssertionError('ROS payload survived its owner')
    finally:
        if owner.poll() is None:
            owner.kill()
            owner.wait(timeout=5)
