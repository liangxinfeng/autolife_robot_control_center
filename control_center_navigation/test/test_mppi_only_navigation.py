"""No ROS initialization or motor publishers; arrival ownership regressions."""
from types import SimpleNamespace as NS
import pytest
from waypoint_manager_node import WaypointManagerNode
from test_staged_short_servo_logic import Logger

def node():
    n=WaypointManagerNode.__new__(WaypointManagerNode)
    n._normal_navigation_mppi_only=True;n._dock_staging_active=False
    n._active_goal_id=1;n._active_goal_name='test';n._active_goal_handle=object()
    n._arrival_refine_pending=False;n._goal_reached_since=None
    n._far_navigation_auto_refine_enabled=True;n._completed_waypoints=set()
    n._arrival_xy_tolerance=.05;n._arrival_yaw_tolerance=.06
    n._final_precision_retry_count=0;n._final_precision_nav2_retries=1
    n._voice_arrival_template='arrived';n.get_logger=lambda:Logger()
    n.statuses=[];n.advanced=[];n.stopped=[]
    n._publish_navigation_status=lambda state,*a,**k:n.statuses.append(state)
    n._publish_waypoint_markers=lambda:None;n._publish_voice_prompt=lambda *a:None
    n._advance_sequence_after_arrival=lambda name:n.advanced.append(name)
    n._stop_sequence=lambda:n.stopped.append(True)
    n._try_start_short_distance=lambda *a,**k:pytest.fail('normal goal must not start servo')
    n._start_navigation=lambda *a,**k:pytest.fail('no second goal for precision retry')
    return n

def test_normal_timer_never_cancels_nav2_at_broad_or_fine_radius():
    n=node();n._goal_reached_since=object()
    n._check_active_goal_arrival();n._handover_arrival_to_refine('test',.04,.01)
    assert n._active_goal_handle is not None and not n.statuses
    assert not n._arrival_refine_pending

@pytest.mark.parametrize('error',[(.04,.05),(.051,.02),(.01,.061),None])
def test_mppi_result_is_verified_without_servo_or_repeated_goal(error):
    n=node();n._waypoint_pose_error=lambda name:error
    n._handle_navigation_result(NS(result=lambda:NS(status=4)),'test',1)
    good=error is not None and error[0]<=.05 and error[1]<=.06
    assert n.statuses[-1]==('succeeded' if good else 'failed')
    assert bool(n.advanced)==good

def test_docking_keeps_its_existing_alignment_policy():
    n=node();assert n._uses_mppi_only()
    n._dock_staging_active=True;assert not n._uses_mppi_only()
    n._dock_staging_active=False;n._normal_navigation_mppi_only=False
    assert not n._uses_mppi_only()

def test_aborted_or_stale_result_cannot_report_arrival():
    n=node();n._handle_navigation_result(NS(result=lambda:NS(status=6)),'test',1)
    assert n.statuses==['failed'] and not n.advanced
    n=node();n._handle_navigation_result(NS(result=lambda:NS(status=4)),'test',0)
    assert not n.statuses and not n.advanced

def test_occupied_goal_has_actionable_message_without_retry():
    from nav2_msgs.action import ComputePathToPose
    n=node(); messages=[]
    n._publish_navigation_status=lambda state,message,**kw:messages.append(message)
    result=NS(status=6,result=NS(error_code=ComputePathToPose.Result.GOAL_OCCUPIED,error_msg='occupied'))
    n._handle_navigation_result(NS(result=lambda:result),'test',1)
    assert '目标位置被占用' in messages[-1] and not n.advanced

def test_settled_arrival_uses_one_certificate_and_announces_once():
    n=node();n._nav2_settled_arrival_authoritative=True;voices=[]
    n._publish_voice_prompt=lambda *a:voices.append(a)
    n._waypoint_pose_error=lambda name:pytest.fail('do not re-judge a later TF frame')
    f=NS(result=lambda:NS(status=4))
    n._handle_navigation_result(f,'test',1)
    n._handle_navigation_result(f,'test',1)
    assert n.statuses==['succeeded'] and len(voices)==1 and n.advanced==['test']

def test_stale_or_failed_settled_goal_does_not_announce():
    n=node();n._nav2_settled_arrival_authoritative=True
    n._publish_voice_prompt=lambda *a:pytest.fail('no success speech for stale/failed goal')
    n._handle_navigation_result(NS(result=lambda:NS(status=4)),'test',0)
    n._handle_navigation_result(NS(result=lambda:NS(status=6)),'test',1)
    assert n.statuses==['failed'] and not n.advanced
