"""Control-center ownership and recovery regressions. All mutation boundaries are fakes."""
import json
import os
import subprocess
import sys
from dataclasses import asdict
from types import SimpleNamespace as NS
from unittest.mock import Mock

import pytest
from control_center_shell import app, preflight as p, resource_repair as repair
from control_center_shell.resource_observer import Snapshot, Issue, Unit
from control_center_service_manager.panel import ServiceManagerPanel


def test_cancel_restart_has_no_handoff(monkeypatch):
    fake = Mock()
    monkeypatch.setattr(app.QtWidgets.QMessageBox, 'question', lambda *args: app.QtWidgets.QMessageBox.No)
    ServiceManagerPanel.request_action(fake, 'restart', 'gv-slam-service.service')
    fake._guard_allows.assert_not_called()
    fake._execute_action.assert_not_called()


def test_failed_stop_does_not_stop_next_dependency(monkeypatch):
    fake = Mock()
    fake._execute_action.side_effect = lambda action, name, callback: callback(False)
    monkeypatch.setattr(app.QtWidgets.QMessageBox, 'question', lambda *args: app.QtWidgets.QMessageBox.Yes)
    ServiceManagerPanel.stop_navigation_stack(fake)
    assert fake._execute_action.call_count == 1


def test_transaction_spans_processes(tmp_path, monkeypatch):
    monkeypatch.setattr(repair, 'JOURNAL', tmp_path / 'journal.json')
    script = ('from control_center_shell import resource_repair as r;from pathlib import Path;'
              'import sys;r.JOURNAL=Path(sys.argv[1]);print(r.acquire_transaction() is not None)')
    with repair.acquire_transaction():
        assert subprocess.check_output([sys.executable, '-c', script, str(repair.JOURNAL)], text=True).strip() == 'False'
    with repair.acquire_transaction():
        pass


def test_live_record_is_owned_by_parent(monkeypatch, tmp_path):
    monkeypatch.setattr(p, 'STATE_DIR', tmp_path)
    ps = {pid: dict(pid=pid, ppid=parent, pgid=group, uid=os.getuid(), start=pid,
          command=cmd, cgroup='') for pid, parent, group, cmd in [
              (31, 1, 31, 'external UI'), (32, 31, 32, p.LAUNCH['navigation']),
              (33, 32, 33, 'map_server')]}
    record = tmp_path / 'navigation.json'
    record.write_text(json.dumps(dict(pid=32,start=32)))
    monkeypatch.setattr(p,'_processes',lambda:ps)
    monkeypatch.setattr(p, '_terminate_group', lambda *a: pytest.fail('unexpected stop'))
    p._clean_records({},lambda _:None)
    assert record.exists()
    units, groups, blocked = repair.plan(Snapshot([Issue('/map','CONFLICT','other',[ps[33]])]))
    assert not units and not groups and '活动应用' in blocked[0]


def test_unconfirmed_plan_cannot_stop(monkeypatch):
    monkeypatch.setattr(repair, 'PREVIEWED', None)
    monkeypatch.setattr(repair, 'plan', lambda _: ([],[],[]))
    with pytest.raises(RuntimeError, match='确认'):
        repair.execute(Snapshot(), print)


@pytest.mark.parametrize('states,expected,remaining', [
    (['activating','activating','failed'], 'WARN', 1),
    (['inactive','active'], 'PASS', 0),
    (['activating','active'], 'PASS', 0),
])
def test_restore_verifies_terminal_state(monkeypatch,tmp_path,states,expected,remaining):
    unit = Unit('demo.service','user',os.getuid(),'/demo')
    monkeypatch.setattr(repair,'JOURNAL',tmp_path/'journal.json')
    repair.save([{'unit':asdict(unit),'state':'paused'}])
    states = iter(states)
    monkeypatch.setattr(repair,'properties',lambda _: {'ActiveState':next(states)})
    monkeypatch.setattr(repair,'command',lambda *args: NS(returncode=0))
    monkeypatch.setattr(repair,'handoff_guard',lambda *args:None)
    monkeypatch.setattr(p,'_processes',lambda:{})
    suffixes=[]
    monkeypatch.setattr(p,'_conflicts',lambda ps,s: suffixes.append(s) or {})
    monkeypatch.setattr(p,'resource_snapshot',lambda s:suffixes.append(s) or Snapshot())
    assert repair.restore(lambda _:None, '0_345')[0] == expected
    assert suffixes == ['0_345']*2 and len(repair.load()) == remaining
    if not remaining:
        assert repair.restore(print, '0_345')[0] == 'PASS'


def test_sensor_warning_preserves_running_modules():
    fake = Mock(_closing=False, _status_check_for_repair=True)
    app.ControlCenterWindow._status_check_complete(fake, 'WARN', 'front scan missing')
    fake.stop_navigation.assert_not_called()
    fake.stop_slam.assert_not_called()
    fake._continue_service_repair.assert_called_once()


def test_joint_watchdog_continues_during_repair():
    fake = Mock(_closing=False, _preflight_ready=False, processes={'joint':NS(running=True)})
    app.ControlCenterWindow._publish_operator_heartbeat(fake)
    fake.node.publish_operator_heartbeat.assert_called_once()


@pytest.mark.parametrize('module',['joint','navigation','slam'])
def test_active_workflow_blocks_only_at_takeover(monkeypatch,module):
    fake=dict(pid=11,uid=os.getuid(),command='worker',cgroup='0::/user.slice/flow-service.service')
    assert p._conflicts({11:fake})['workflow_control']==[fake]
    monkeypatch.setattr(p,'resource_snapshot',lambda *args:Snapshot([Issue('workflow_control','CONFLICT','workflow active',[fake])]))
    assert not p.module_snapshot(module,'0_367').ok


def test_reset_rechecked_after_service_response():
    panel=Mock()
    panel._guard_allows.return_value=False
    ServiceManagerPanel._joint_reset_service_checked(panel,True,'ActiveState=active',0)
    panel._run_command.assert_not_called()
