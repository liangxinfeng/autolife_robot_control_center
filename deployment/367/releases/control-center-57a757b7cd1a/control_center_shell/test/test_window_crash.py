import json
import os
from pathlib import Path
import select
import subprocess
import sys
import time

from control_center_module_lease import open_pid


def test_qt_owner_crash_drains_launch_and_worker(tmp_path):
    path=tmp_path/'ros2'
    path.write_text(f'#!{sys.executable}\n'+"""
from control_center_module_lease import claim
from types import SimpleNamespace
from pathlib import Path
import os,subprocess,sys,time
env=dict(os.environ)
claim(SimpleNamespace(environment=env),'navigation')
worker=subprocess.Popen([sys.executable,'-c','import time; time.sleep(30)'],env=env,start_new_session=True)
Path(os.environ['CC_TEST_READY']).write_text(f'{os.getpid()} {worker.pid}')
time.sleep(30)
""")
    path.chmod(0o700)
    ready=tmp_path/'ready'
    env=dict(os.environ,HOME=str(tmp_path),PATH=str(tmp_path)+':'+os.environ['PATH'],CC_TEST_READY=str(ready))
    parent=subprocess.Popen([sys.executable,'-c',"""
from PyQt5 import QtCore
from control_center_shell.app import ManagedProcess
qt=QtCore.QCoreApplication([])
module=ManagedProcess('navigation')
module.start(['launch','control_center_navigation','bringup_all.launch.py'])
qt.exec_()
"""],env=env,stdout=subprocess.DEVNULL)
    fds=[]
    try:
        record=tmp_path/'.local/state/autolife-control-center/processes/navigation.json'
        deadline=time.monotonic()+8
        while time.monotonic()<deadline and not (ready.exists() and record.exists()):
            time.sleep(.05)
        assert ready.exists() and record.exists()
        pid,worker=map(int,ready.read_text().split())
        assert json.loads(record.read_text())['pid']==pid
        for ident in (pid,worker):
            fds.append(open_pid(ident))
        parent.kill()
        parent.wait(timeout=3)
        for fd in fds:
            poll=select.poll();poll.register(fd,select.POLLIN)
            assert poll.poll(5000),'surviving process after Qt owner crash'
        deadline=time.monotonic()+3
        while record.exists() and time.monotonic()<deadline:
            time.sleep(.05)
        assert not record.exists()
    finally:
        if parent.poll() is None:
            parent.kill()
            parent.wait()
        for fd in fds:
            os.close(fd)
