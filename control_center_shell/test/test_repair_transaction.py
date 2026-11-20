from types import SimpleNamespace
from unittest.mock import Mock
import pytest
from control_center_shell import preflight as p, resource_repair as r
from control_center_shell.resource_observer import Snapshot


@pytest.mark.parametrize('service,diagnostic,expected,starts',[
    ('active',('PASS','healthy'),'PASS',0),
    ('active',('WARN','front lidar unavailable'),'WARN',0),
    ('inactive',('PASS','healthy'),'AUTO_RECOVERED',1),
    ('inactive',('WARN','joint feedback missing'),'WARN',1),
])
def test_repair_to_final_check(monkeypatch,tmp_path,service,diagnostic,expected,starts):
    monkeypatch.setattr(p,'STATE_DIR',tmp_path/'records')
    monkeypatch.setattr(r,'JOURNAL',tmp_path/'paused.json')
    monkeypatch.setattr(p,'resource_snapshot',lambda *args:Snapshot())
    monkeypatch.setattr(p,'_processes',lambda:{})
    monkeypatch.setattr(p,'_gv_control_state',lambda:service)
    recover=Mock(return_value=(True,''))
    monkeypatch.setattr(p,'_recover_gv_control',recover)
    monkeypatch.setattr(p,'_clear_proven_stale_flag',lambda *args:None)
    monkeypatch.setattr(p,'check_only',lambda *args:diagnostic)
    monkeypatch.setattr(p,'_diagnose',lambda *args:[] if diagnostic[0]=='PASS' else [diagnostic[1]])
    initial = r.time.monotonic()
    now=[initial]
    def sleep(seconds):
        now[0]+=seconds
    monkeypatch.setattr(p,'time',SimpleNamespace(monotonic=lambda:now[0],sleep=sleep))
    p.repair_plan('0_367')
    result,detail=p.run('0_367',takeover=True,emit=lambda _:None)
    assert result==expected and recover.call_count==starts
    assert (now[0]-initial >= 15) == bool(starts)
    assert r.PREVIEWED is None
    with r.acquire_transaction():
        pass
