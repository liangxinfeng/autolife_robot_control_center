from pathlib import Path
import yaml

def test_controller_keeps_omni_fallback_and_exact_final_gate():
    p=Path(__file__).resolve().parents[1]
    c=yaml.safe_load((p/'config/nav2_params.yaml').read_text())['controller_server']['ros__parameters']
    m=c['FollowPath']
    assert abs(m['model_dt']*c['controller_frequency']-1)<1e-8
    assert m['motion_model']=='Omni' and m['vx_min']<0 and m['vy_max']>0
    assert c['goal_checker']['xy_goal_tolerance']==.05
    assert c['goal_checker']['yaw_goal_tolerance']==.05
    assert m['PathFollowCritic']['threshold_to_consider']==.15
    assert m['PreferForwardCritic']['threshold_to_consider']==.15
    assert m['CostCritic']['consider_footprint'] is True
