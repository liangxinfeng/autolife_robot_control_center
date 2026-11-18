"""Guard the planner/controller/task arrival contract."""
from pathlib import Path
import yaml

def test_exact_goal_and_arrival_contract():
    root=Path(__file__).resolve().parents[1]
    config=yaml.safe_load((root/'config/nav2_params.yaml').read_text())
    voice=yaml.safe_load((root/'config/voice_feedback.yaml').read_text())
    planner=config['planner_server']['ros__parameters']['GridBased']
    control=config['controller_server']['ros__parameters']
    goal=control['goal_checker']
    assert planner['plugin']=='control_center_navigation::ExactGoalSmacPlanner'
    assert planner['tolerance']==0 and not planner['use_final_approach_orientation']
    assert goal['plugin']=='control_center_navigation::SettledGoalChecker'
    assert control['FollowPath']['plugin']=='control_center_navigation::SettledMPPIController'
    assert voice['waypoint_manager']['ros__parameters']['nav2_settled_arrival_authoritative'] is True
    assert goal['stateful'] is False and control['publish_zero_velocity'] is True
    assert control['progress_checker_plugins']==['progress_checker']
    # These are configured values, not claims about measured physical accuracy.
    assert goal['xy_goal_tolerance']==.05 and goal['yaw_goal_tolerance']==.05
    assert voice['waypoint_manager']['ros__parameters']['arrival_xy_tolerance']==goal['xy_goal_tolerance']
    assert control['FollowPath']['CostCritic']['consider_footprint'] is True
    assert any(v.get('ros__parameters',{}).get('normal_navigation_mppi_only') is True
               for v in voice.values() if isinstance(v,dict))
