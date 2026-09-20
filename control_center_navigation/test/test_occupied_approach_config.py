from pathlib import Path
import xml.etree.ElementTree as ET
import yaml
from test_mppi_only_navigation import node

ROOT=Path(__file__).resolve().parents[1]

def test_approach_keeps_strict_planner_for_probes_and_docking():
    config=yaml.safe_load((ROOT/'config/nav2_params.yaml').read_text())
    params=config['planner_server']['ros__parameters']
    assert not params['GridBased'].get('allow_occupied_approach',False)
    assert params['ApproachGridBased']['allow_occupied_approach']
    assert config['bt_navigator']['ros__parameters']['goal_reached_tol']==.05
    assert config['controller_server']['ros__parameters']['goal_checker']['xy_goal_tolerance']==.05

def test_bt_never_replaces_the_requested_goal_or_skips_follow_path():
    tree=ET.parse(ROOT/'config/occupied_approach.xml')
    assert tree.find('.//ComputePathToPose').attrib['goal']=='{goal}'
    assert tree.find('.//ComputePathToPose').attrib['planner_id']=='ApproachGridBased'
    pipeline=tree.find('.//PipelineSequence')
    assert pipeline[-1].tag=='ReactiveFallback'
    assert pipeline[-1][0].tag=='GoalReached' and pipeline[-1][0].attrib['goal']=='{goal}'
    assert pipeline[-2].find('FollowPath') is not None

def test_only_normal_waypoints_get_approach_tree():
    n=node();n._map_frame='map'
    n._navigation_target=lambda name:dict(x=1.,y=2.,yaw=0.,frame_id='map')
    from types import SimpleNamespace as NS
    from builtin_interfaces.msg import Time
    n.get_clock=lambda:NS(now=lambda:NS(to_msg=lambda:Time()))
    assert n._build_goal('test').behavior_tree.endswith('/config/occupied_approach.xml')
    n._dock_staging_active=True
    assert n._build_goal('test').behavior_tree==''
