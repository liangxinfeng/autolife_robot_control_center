"""Exercise real file locks, process death and orphan cleanup using sleeping test workers."""
from concurrent.futures import ThreadPoolExecutor
import os
from pathlib import Path
import signal
import subprocess
import sys
import time
import pytest

from control_center_shell import preflight as p

DRIVER = """
from types import SimpleNamespace
from control_center_module_lease import claim
import os, subprocess, sys, time
context = SimpleNamespace(environment=dict(os.environ))
try:
    claim(context, sys.argv[1])
except RuntimeError as error:
    print('busy',error,flush=True)
    raise SystemExit(17)
cmd = [sys.executable, '-c', 'import time; time.sleep(20)']
child = subprocess.Popen(cmd, env=context.environment, start_new_session=True,
                         stdout=subprocess.DEVNULL) if sys.argv[2] == 'child' else None
print('ready',child.pid if child else 0,flush=True)
sys.stdin.readline()
"""


def launch(tmp_path, module, child=False):
    proc = subprocess.Popen([sys.executable, '-c', DRIVER, module, 'child' if child else ''],
        stdin=subprocess.PIPE,stdout=subprocess.PIPE,text=True,env=dict(os.environ,HOME=str(tmp_path)))
    return proc, proc.stdout.readline().strip()


def finish(proc):
    proc.communicate(timeout=6)


def reacquire(directory):
    end=time.monotonic()+6
    while time.monotonic()<end:
        proc,status=launch(directory,'slam')
        finish(proc)
        if status.startswith('ready'):
            return
        assert proc.returncode==17
        time.sleep(.05)
    pytest.fail('resource was not released')


@pytest.mark.parametrize('abrupt',[False,True])
def test_mutual_exclusion_and_cleanup(tmp_path,abrupt):
    owner,status=launch(tmp_path,'navigation',True)
    assert status.startswith('ready')
    from control_center_module_lease import open_pid
    child_fd=open_pid(int(status.split()[1]))
    try:
        other,status=launch(tmp_path,'slam')
        finish(other)
        assert status.startswith('busy') and other.returncode==17
        independent,status=launch(tmp_path,'joint')
        finish(independent)
        assert status.startswith('ready')
        if abrupt:
            owner.kill()
        finish(owner)
        reacquire(tmp_path)
        import select
        poll=select.poll()
        poll.register(child_fd,select.POLLIN)
        assert poll.poll(0), 'orphan still alive when next owner admitted'
        locks = list((tmp_path/'.local/state/autolife-control-center/leases').glob('*.lock'))
        deadline = time.monotonic() + 3
        while any(f.read_text() for f in locks) and time.monotonic() < deadline:
            time.sleep(.05)
        assert all(not f.read_text() for f in locks)
    finally:
        if owner.poll() is None:
            owner.kill()
            finish(owner)
        os.close(child_fd)


def test_concurrent_starts(tmp_path):
    with ThreadPoolExecutor(max_workers=6) as pool:
        attempts=list(pool.map(lambda m:launch(tmp_path,m), ['navigation','slam']*3))
    try:
        assert sum(s.startswith('ready') for _,s in attempts)==1
    finally:
        for proc,_ in attempts:
            finish(proc)
    reacquire(tmp_path)


def test_record_of_replacement_survives_old_exit(tmp_path,monkeypatch):
    monkeypatch.setattr(p,'STATE_DIR',tmp_path)
    record=tmp_path/'navigation.json'
    record.write_text('{"pid":22,"start":55}')
    monkeypatch.setattr(p,'_processes',lambda:{22:{'pgid':22}})
    p.forget_process('navigation',21)
    p.forget_process('navigation')
    assert record.exists()
    monkeypatch.setattr(p,'_processes',lambda:{})
    p.forget_process('navigation',22)
    assert not record.exists()
