import json,math
from types import SimpleNamespace as NS
import pytest
from PyQt5 import QtWidgets
from control_center_joint_control.body_height import body_height_targets,estimate_body_lowering
from control_center_joint_control.joint_definitions import JOINT_LIMITS
from control_center_joint_control.control_panel import GroupPage,ControlPanel,JointControlRosNode

@pytest.fixture
def page():
    app=QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    p=GroupPage('leg_waist')
    for row in p.rows.values():row.current=0.;row.set_target(0.)
    p.rows['Joint_Waist_Yaw'].current=12.
    p.update_body_height_current(True)
    yield p
    p.close();p.deleteLater();app.processEvents()

def test_fresh_feedback_enables_slider_and_buttons(page):
    h=page.body_height_control
    assert all(w.isEnabled() for w in (h.slider,h.target_box,h.straight_button,h.direction_box))
    h.slider.setValue(100)
    assert page.motion_profile()=='body_height'
    assert page.targets()['Joint_Knee']==pytest.approx(155*.1/.56,abs=.06)
    assert page.targets()['Joint_Waist_Yaw']==12.
    assert not h.direction_box.isEnabled()
    h.straight_button.click()
    assert all(page.targets()[n]==0. for n in body_height_targets(0.))

@pytest.mark.parametrize('fault',['stale','missing','nan'])
def test_feedback_fault_disables_height_control(page,fault):
    if fault=='missing':page.rows['Joint_Knee'].current=None
    if fault=='nan':page.rows['Joint_Knee'].current=float('nan')
    page.update_body_height_current(fault!='stale')
    h=page.body_height_control
    assert not any(w.isEnabled() for w in (h.slider,h.target_box,h.straight_button,h.direction_box))

def test_direction_selection_does_not_create_motion_and_lowered_cannot_flip(page):
    h=page.body_height_control;seen=[];h.height_changed.connect(seen.append)
    h.direction_box.setCurrentIndex(1)
    assert not seen and h._reverse_squat
    h.slider.setValue(100)
    assert seen[-1]<0 and page.targets()['Joint_Knee']<0
    before=page.targets().copy();h._direction_changed(0)
    assert page.targets()==before and h._reverse_squat
    for n,v in body_height_targets(-.1,allow_reverse=True).items():page.rows[n].current=v
    page.update_body_height_current(True)
    h.straight_button.click()
    assert not h.direction_box.isEnabled()  # zero target is not zero measured pose

@pytest.mark.parametrize('value',[-1.,-.53,-.3,-.1,0.,.1,.3,.56,1.])
def test_model_geometry_and_shared_limit_preserve_upright_torso(value):
    q=body_height_targets(value,allow_reverse=True)
    for n,v in q.items():assert JOINT_LIMITS[n][0]<=v<=JOINT_LIMITS[n][1]+1e-9
    a,k,w=[q[n] for n in ('Joint_Ankle','Joint_Knee','Joint_Waist_Pitch')]
    assert a/76.==pytest.approx(k/155.)
    assert w/81.==pytest.approx(k/155.)
    assert abs(a-k+w)<=2.01
    height=.28+.3395*math.cos(math.radians(a))+.3675*math.cos(math.radians(a-k))
    assert .28<height<=.9870001
    assert estimate_body_lowering(q,signed=True)[1]<1e-9

@pytest.mark.parametrize('value',[float('nan'),float('inf'),-float('inf')])
def test_nonfinite_targets_rejected(value):
    with pytest.raises(ValueError):body_height_targets(value,allow_reverse=True)

@pytest.mark.parametrize('enabled,ready,leg,live,expected',[
 (False,False,False,True,False),(True,False,True,True,False),
 (True,True,False,True,False),(True,True,True,False,False),
 (True,True,True,True,True)])
def test_slider_requires_all_original_unlocks_before_real_output(page,enabled,ready,leg,live,expected):
    messages=[];preview=object();apply=object()
    node=NS(preview_pub=preview,apply_pub=apply,
        publish_targets=lambda pub,g,q,**kw:messages.append((pub,g,q.copy(),kw)))
    panel=NS(pages={'leg_waist':page},node=node,
        live_toggle=NS(isChecked=lambda:live),
        _status=dict(connected=True,hardware_enabled=enabled,hardware_ready=ready,leg_waist_enabled=leg),
        _pending_preview_groups=set(),_pending_live_groups=set(),_pending_live_joint_names={},
        action_page=NS(suggest_groups=lambda g:None),
        robot_view=NS(view=NS(set_target_joint=lambda *a:None)),
        preview_timer=NS(isActive=lambda:False,start=lambda:None))
    panel._live_apply_allowed=lambda g:ControlPanel._live_apply_allowed(panel,g)
    page.preview_requested.connect(lambda g:ControlPanel.schedule_preview(panel,g))
    page.body_height_control.slider.setValue(50)
    ControlPanel.flush_previews(panel)
    real=[m for m in messages if m[0] is apply]
    assert bool(real)==expected
    assert messages and messages[0][0] is preview
    if real:
        assert real[0][3]==dict(motion_profile='body_height',single_joint=None)
        assert real[0][2]['Joint_Waist_Yaw']==12.
