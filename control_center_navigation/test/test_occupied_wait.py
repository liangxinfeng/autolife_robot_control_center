from types import SimpleNamespace as NS
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from test_mppi_only_navigation import node
import waypoint_manager_node as module

def make_wait_node(monkeypatch):
    now=[100.0];monkeypatch.setattr(module.time,'monotonic',lambda:now[0])
    n=node();n._occupied_wait_enabled=True;n._occupied_deadline=None
    n._occupied_wait_timeout=60.;n._occupied_probe_interval=3.
    n._occupied_wait=None;n._manual_control_active=False;n._navigation_paused=False
    n._active_goal_pose=PoseStamped();n._active_goal_pose.header.frame_id='map'
    n._arrival_odom=(100.,0.,0.);n.zeros=[]
    n._short_cmd_vel_pub=NS(publish=lambda m:n.zeros.append(m))
    n._build_goal=lambda name:NS(pose=n._active_goal_pose)
    return n,now

def blocked():
    return NS(result=lambda:NS(status=6,result=NS(error_code=ComputePathToPose.Result.GOAL_OCCUPIED)))

def clear_path():
    return NS(result=lambda:NS(status=4,result=NS(error_code=0,path=NS(poses=[1]))))

def test_occupied_abort_retains_goal_without_failure_or_advancing(monkeypatch):
    n,_=make_wait_node(monkeypatch);n._handle_navigation_result(blocked(),'test',1)
    assert n._active_goal_name=='test' and n._active_goal_handle is None
    assert n.statuses==['running'] and not n.advanced and not n.stopped and len(n.zeros)==1


def test_first_blockage_probe_is_prompt_without_extending_wait_deadline(monkeypatch):
    n,now=make_wait_node(monkeypatch)
    n._handle_navigation_result(blocked(),'test',1)
    assert n._occupied_wait['next_probe']==now[0]+.5
    assert n._occupied_wait['deadline']==now[0]+60.
    # Repeated probes retain the configured rate; this is not a busy retry loop.
    assert n._occupied_probe_interval==3.

def test_only_valid_plan_and_stopped_feedback_retries_same_goal(monkeypatch):
    n,now=make_wait_node(monkeypatch);n._handle_navigation_result(blocked(),'test',1)
    wait=n._occupied_wait;sent=[]
    n._start_navigation=lambda name,**kw:(sent.append((name,kw)) or (True,'ok'))
    n._occupied_probe_done(blocked(),wait);assert not sent
    now[0]=101.;n._occupied_probe_done(clear_path(),wait);assert not sent # stale odom
    n._arrival_odom=(101.,.1,0.);n._occupied_probe_done(clear_path(),wait);assert not sent
    n._arrival_odom=(101.,0.,0.);n._occupied_probe_done(clear_path(),wait)
    assert sent==[('test',dict(skip_undock=True,force_normal=True,occupied_retry=True))]
    assert not n.advanced and n._occupied_deadline==160.

def test_cleared_or_superseded_wait_cannot_restart(monkeypatch):
    n,_=make_wait_node(monkeypatch);n._handle_navigation_result(blocked(),'test',1);wait=n._occupied_wait
    n._clear_occupied_wait();n._occupied_probe_done(clear_path(),wait)
    assert n._occupied_wait is None and not n.advanced
    n,_=make_wait_node(monkeypatch);n._handle_navigation_result(blocked(),'test',1);wait=n._occupied_wait
    n._active_goal_id=2;n._occupied_probe_done(clear_path(),wait)
    assert not n.advanced

def test_timeout_does_not_skip_waypoint(monkeypatch):
    n,now=make_wait_node(monkeypatch);n._handle_navigation_result(blocked(),'test',1)
    now[0]=161.;n._tick_occupied_wait()
    assert n.statuses==['running','failed'] and n._failed_goal_name=='test'
    assert not n.advanced and n._occupied_wait is None

def test_manual_takeover_cancels_wait(monkeypatch):
    n,_=make_wait_node(monkeypatch);n._handle_navigation_result(blocked(),'test',1);wait=n._occupied_wait
    calls=[]
    def pause(*a): calls.append(1);n._clear_occupied_wait()
    n._handle_pause_navigation=pause;n._on_manual_control(NS(data=True))
    n._occupied_probe_done(clear_path(),wait)
    assert calls==[1] and n._occupied_wait is None

def test_changed_waypoint_requires_confirmation(monkeypatch):
    n,_=make_wait_node(monkeypatch);n._handle_navigation_result(blocked(),'test',1);wait=n._occupied_wait
    p=PoseStamped();p.header.frame_id='map';p.pose.position.x=1.
    n._build_goal=lambda name:NS(pose=p)
    n._occupied_probe_done(clear_path(),wait)
    assert n.statuses[-1]=='failed' and not n.advanced

def test_repeated_blockage_does_not_reset_deadline(monkeypatch):
    n,now=make_wait_node(monkeypatch);n._handle_navigation_result(blocked(),'test',1)
    now[0]=130.;n._occupied_wait=None
    assert n._begin_occupied_wait('test',1) and n._occupied_wait['deadline']==160.

def test_docking_not_automatically_retried(monkeypatch):
    n,_=make_wait_node(monkeypatch);n._dock_staging_active=True
    assert not n._begin_occupied_wait('test',1)
