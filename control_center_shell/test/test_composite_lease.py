from pathlib import Path
import os
import subprocess
import sys
import time


def test_vr_reserves_both_before_handoff_and_delegates_once(tmp_path):
    child = """from control_center_module_lease import claim
from types import SimpleNamespace
import os,sys
env=SimpleNamespace(environment=dict(os.environ))
claim(env,'vr_stack')
claim(env,'navigation')
claim(env,'vr_arm')
for module in ('slam','grasp','vr_arm'):
    try: claim(env,module)
    except RuntimeError: pass
    else: raise AssertionError(module+' admitted twice')
print('composite-ok')
"""
    proc=subprocess.run([sys.executable,'-c',child],env=dict(os.environ,HOME=str(tmp_path)),
                        capture_output=True,text=True,timeout=10)
    assert proc.returncode==0,proc.stderr
    assert 'composite-ok' in proc.stdout


def test_failed_composite_releases_its_partial_acquisitions(tmp_path):
    child = """from control_center_module_lease import claim
from types import SimpleNamespace
import os
ctx=lambda:SimpleNamespace(environment=dict(os.environ))
claim(ctx(),'grasp')
try: claim(ctx(),'vr_stack')
except RuntimeError: pass
else: raise AssertionError('grasp lost joints')
claim(ctx(),'navigation')
"""
    subprocess.run([sys.executable,'-c',child],env=dict(os.environ,HOME=str(tmp_path)),check=True,timeout=10)


def test_failed_exec_releases_grasp(tmp_path):
    env = dict(os.environ,HOME=str(tmp_path))
    cmd = [sys.executable,'-m','control_center_module_lease','run','grasp']
    assert subprocess.run(cmd+['/nonexistent-cc-test'],env=env,capture_output=True).returncode
    deadline=time.monotonic()+5
    while time.monotonic()<deadline:
        if subprocess.run(cmd+['/usr/bin/true'],env=env,capture_output=True).returncode==0:
            return
        time.sleep(.05)
    raise AssertionError('failed exec kept resource')
