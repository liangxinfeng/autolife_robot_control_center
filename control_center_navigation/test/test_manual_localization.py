import math,time
from concurrent.futures import Future
from types import SimpleNamespace as NS
import pytest
from PyQt5 import QtWidgets,QtCore,QtTest
from nav_msgs.msg import OccupancyGrid
from test_nav_rgb import nav


@pytest.fixture(scope='module')
def qt():return QtWidgets.QApplication.instance() or QtWidgets.QApplication([])


@pytest.mark.parametrize('yaw',[0.,.7,-1.2])
@pytest.mark.parametrize('zoom',[.6,2.5])
def test_map_drag_position_is_press_point_with_origin_rotation_and_pan(qt,yaw,zoom):
    w=nav.MapCanvas();w.resize(900,600)
    m=OccupancyGrid();m.info.width=100;m.info.height=80;m.info.resolution=.05
    m.info.origin.position.x=-5.;m.info.origin.position.y=-3.
    m.info.origin.orientation.z=math.sin(yaw/2);m.info.origin.orientation.w=math.cos(yaw/2)
    m.data=[0]*8000;w.set_map(m);w.zoom=zoom;w.pan=QtCore.QPointF(45,-25)
    start=w._image_to_screen(50,40);end=start+QtCore.QPointF(40,-20)
    expected=w.screen_to_world(start)
    assert (w.world_to_screen(*expected)-start).manhattanLength()<1e-7
    results=[];w.initial_pose_requested.connect(lambda *v:results.append(v));w.set_mode('initial')
    QtTest.QTest.mousePress(w,QtCore.Qt.LeftButton,pos=start.toPoint())
    QtTest.QTest.mouseRelease(w,QtCore.Qt.LeftButton,pos=end.toPoint())
    assert len(results)==1
    assert math.dist(results[0][:2],expected)<.01
    w.deleteLater()


def test_manual_request_waits_for_cancel_and_publishes_once():
    f=Future();published=[];events=[]
    signal=NS(emit=lambda *a:events.append(a))
    n=NS(_map_load_in_progress=False,_loaded_map_path='map5',
         signals=NS(pose_received=signal,service_result=signal),
         _service_clients={'cancel_localization':NS(service_is_ready=lambda:True,call_async=lambda _:f)},
         _publish_manual_pose=lambda r:published.append(r))
    nav.NavigationRosNode.publish_initial_pose(n,1,2,.3)
    assert not published and n._manual_localization['x']==1
    f.set_result(NS(success=True,message='ok'));assert len(published)==1


def test_cancel_reply_cannot_publish_after_map_change():
    f=Future();published=[]
    sig=NS(emit=lambda *a:None)
    n=NS(_map_load_in_progress=False,_loaded_map_path='map3',signals=NS(pose_received=sig,service_result=sig),
         _service_clients={'cancel_localization':NS(service_is_ready=lambda:True,call_async=lambda _:f)},
         _publish_manual_pose=published.append)
    nav.NavigationRosNode.publish_initial_pose(n,1,2,.3);n._loaded_map_path='map5'
    f.set_result(NS(success=True,message='ok'));assert not published


def test_manual_pose_render_separates_requested_and_confirmed(qt):
    canvas=nav.MapCanvas();n=NS(map_canvas=canvas)
    nav.NavigationWindow._pose_received(n,dict(x=1,y=2,yaw=0,pending=True))
    assert canvas.robot is None and canvas.requested_pose['x']==1
    nav.NavigationWindow._pose_received(n,dict(x=1.01,y=2,yaw=0))
    assert canvas.requested_pose is None and canvas.robot['x']==1.01
    canvas.deleteLater()
