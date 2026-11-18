"""Production dispatch tests with fake action clients; never sends ROS goals."""
import math,sys,unittest
from pathlib import Path
from types import SimpleNamespace as NS
from unittest.mock import Mock
import yaml
sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'scripts'))
from charger_waypoints import is_charger_waypoint
from test_dock_prealignment import extract,manager

Departure=extract('waypoint_manager_node.py',{'_should_auto_undock'})
class NumberedDocks(unittest.TestCase):
 def test_naming(self):
  for s in ['充电桩','充电桩1','充电桩2','充电装2','充电桩 12','充电桩３']:self.assertTrue(is_charger_waypoint(s),s)
  for s in ['前台','充电桩旁边','非充电桩1','充电桩1前','']:self.assertFalse(is_charger_waypoint(s),s)
  self.assertTrue(is_charger_waypoint('Charging2','Charging'))
 def test_each_numbered_dock_uses_own_pose(self):
  for name in ['充电桩','充电桩1','充电装2']:
   for yaw in [0,math.pi/2,math.pi,-math.pi/2]:
    n=manager();n._dock_staging_distance=.35;n._waypoints[name]=dict(x=20.,y=30.,yaw=yaw,frame_id='map')
    self.assertTrue(n._start_navigation(name)[0]);self.assertTrue(n._dock_staging_active)
    p=n._navigation_target(name);self.assertAlmostEqual(p['x'],20+.35*math.cos(yaw));self.assertAlmostEqual(p['y'],30+.35*math.sin(yaw))
    self.assertEqual(n._waypoints[name]['x'],20.)
 def test_departure_from_numbered_dock(self):
  n=Departure();n._auto_undock_enabled=True;n._charger_waypoint_name='充电桩'
  n._waypoints={'充电桩':dict(x=0,y=0,yaw=0),'充电桩2':dict(x=10,y=10,yaw=math.pi/2)}
  n._lookup_current_pose=lambda:(10.,10.,math.pi/2);n._normalize_angle=lambda x:math.remainder(x,math.tau)
  n._undock_trigger_distance=.35;n._undock_yaw_tolerance=.3;n.get_logger=lambda:Mock()
  self.assertTrue(n._should_auto_undock('前台'));self.assertFalse(n._should_auto_undock('充电桩2'))
  n._lookup_current_pose=lambda:(5.,5.,math.pi/2);self.assertFalse(n._should_auto_undock('前台'))
 def test_staging_parameters_agree(self):
  config=Path(__file__).resolve().parents[1]/'config'
  w=yaml.safe_load((config/'voice_feedback.yaml').read_text())['waypoint_manager']['ros__parameters']
  d=yaml.safe_load((config/'docking_params.yaml').read_text())['docking_server']['ros__parameters']
  self.assertEqual(w['dock_staging_distance'],.35);self.assertEqual(d['fine_charging_dock']['staging_x_offset'],-.35)
  self.assertTrue(d['controller']['use_collision_detection'])
if __name__=='__main__':unittest.main()
