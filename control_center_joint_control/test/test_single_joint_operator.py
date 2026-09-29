import json
from types import SimpleNamespace
import pytest
from control_center_joint_control.joint_definitions import ALL_JOINTS, JOINT_TO_GROUP, JOINT_LIMITS, GROUPS, group_joint_names
from control_center_joint_control.operator_targets import resolve_operator_targets
from control_center_joint_control.joint_bridge import JointBridge

def current_pose():
    return {j:sum(JOINT_LIMITS[j])/2 for j in ALL_JOINTS}

@pytest.mark.parametrize('joint',ALL_JOINTS)
def test_each_joint_changes_only_its_own_target(joint):
    current=current_pose();group=JOINT_TO_GROUP[joint]
    result=resolve_operator_targets(group,{joint:current[joint]+1},current,joint)
    assert set(result)==set(group_joint_names(group))
    assert [j for j in result if result[j]!=current[j]]==[joint]

def test_unedited_joint_is_not_clamped_into_a_new_pose():
    c=current_pose();c['Joint_Left_Elbow']=151
    result=resolve_operator_targets('left_arm',{'Joint_Left_Shoulder_Inner':21},c,'Joint_Left_Shoulder_Inner')
    assert result['Joint_Left_Elbow']==151

def test_cross_group_and_hidden_extra_targets_rejected():
    c=current_pose()
    with pytest.raises(ValueError):resolve_operator_targets('left_arm',{'Joint_Right_Elbow':-20},c,'Joint_Right_Elbow')
    with pytest.raises(ValueError):resolve_operator_targets('left_arm',{'Joint_Left_Elbow':50,'Joint_Left_Shoulder_Inner':10},c,'Joint_Left_Elbow')

def test_nonfinite_hold_rejected():
    c=current_pose();c['Joint_Left_Elbow']=float('nan')
    with pytest.raises(ValueError):resolve_operator_targets('left_arm',{'Joint_Left_Shoulder_Inner':20},c,'Joint_Left_Shoulder_Inner')

@pytest.mark.parametrize('group',list(dict.fromkeys(JOINT_TO_GROUP.values())))
def test_explicit_group_pose_still_supported(group):
    c=current_pose();requested={j:c[j]+1 for j in group_joint_names(group)}
    assert resolve_operator_targets(group,requested,c)==requested

@pytest.mark.parametrize('joint',ALL_JOINTS)
def test_bridge_cancels_stale_unedited_progressive_goals(joint):
    c=current_pose();group=JOINT_TO_GROUP[joint];names=group_joint_names(group)
    bridge=SimpleNamespace(_current=c,_action_state=None,_hardware_enabled=True,
        _hardware_ready=True,_leg_waist_enabled=True,_preview={},_goal_sent_groups=set(),
        _active_goals={group:{'commands':{j:c[j]+5 for j in names}}},
        _is_feedback_fresh=lambda:True,_communication_lost_joints=lambda targets:[],
        _event=lambda *a:None)
    bridge._parse_operator_command=lambda p:JointBridge._parse_operator_command(bridge,p)
    message=SimpleNamespace(data=json.dumps({'group':group,'single_joint':joint,'targets_deg':{joint:c[joint]+10}}))
    JointBridge._on_apply(bridge,message);goal=bridge._active_goals[group]
    assert goal['targets'][joint]==c[joint]+10
    for name in names:
        if name!=joint:
            assert goal['targets'][name]==goal['commands'][name]==goal['start'][name]==c[name]
    assert goal['motion_profile']==''
    # Exercise the real serializer/progressive loop without creating a ROS node.
    sent=[];bridge._controller_target=c.copy();bridge._whole_body_pub=object();bridge._gripper_pub=object()
    bridge._stable_progressive_target=JointBridge._stable_progressive_target
    bridge.get_parameter=lambda key:SimpleNamespace(value={'goal_tolerance_deg':1.,'gripper_goal_tolerance_deg':2.,'max_single_step_deg':5.,'max_gripper_command_lead_deg':10.}[key])
    bridge._publish_vendor=lambda pub,payload:sent.append(payload)
    JointBridge._advance_active_goals(bridge)
    assert len(sent)==1
    payload=sent[0];changed=[]
    for g in ['neck','left_arm','right_arm','leg_waist']:
        key=GROUPS[g]['command_key']
        if key in payload:
            changed += [name for name,value in zip(group_joint_names(g),payload[key]) if value!=c[name]]
    for side in ['Left','Right']:
        key=side.lower()+'_gripper_target_joints_position';name='Joint_'+side+'_Gripper'
        if key in payload and payload[key][0]!=c[name]:changed.append(name)
    assert changed==[joint]

def test_single_joint_cannot_enable_body_height_coupling():
    b=SimpleNamespace(_current=current_pose())
    with pytest.raises(ValueError):JointBridge._parse_operator_command(b,json.dumps({'group':'leg_waist','single_joint':'Joint_Knee','targets_deg':{'Joint_Knee':20},'motion_profile':'body_height'}))

def test_live_gui_model_and_slider_send_only_edited_joint():
    from PyQt5 import QtWidgets
    from control_center_joint_control.control_panel import GroupPage,ControlPanel,JointControlRosNode
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    page=GroupPage('left_arm');joint='Joint_Left_Shoulder_Inner';c=current_pose()
    for name,row in page.rows.items():row.current=c[name];row.set_target(c[name]+10)
    page._row_target_changed(joint)
    messages=[];pub=SimpleNamespace(publish=lambda m:messages.append(json.loads(m.data)))
    node=SimpleNamespace(preview_pub=pub,apply_pub=pub)
    node.publish_targets=lambda *args,**kwargs:JointControlRosNode.publish_targets(node,*args,**kwargs)
    panel=SimpleNamespace(pages={'left_arm':page},_live_apply_allowed=lambda g:True,
        _pending_preview_groups=set(),_pending_live_groups=set(),_pending_live_joint_names={},
        action_page=SimpleNamespace(suggest_groups=lambda g:None),
        robot_view=SimpleNamespace(view=SimpleNamespace(set_target_joint=lambda *a:None)),
        preview_timer=SimpleNamespace(isActive=lambda:False,start=lambda:None),node=node)
    ControlPanel.schedule_preview(panel,'left_arm');ControlPanel.flush_previews(panel)
    assert messages[-1]['single_joint']==joint
    assert messages[-1]['targets_deg']=={joint:c[joint]+10}
    assert all(row.target()==c[name] for name,row in page.rows.items() if name!=joint)
    page.close();page.deleteLater();app.processEvents()
