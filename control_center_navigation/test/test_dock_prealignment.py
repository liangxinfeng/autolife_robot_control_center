"""Execute production methods with fake action clients: NO live ROS nodes/motion."""
import ast
import math
from pathlib import Path
from types import SimpleNamespace as NS
import unittest
import sys
from unittest.mock import Mock
from geometry_msgs.msg import Twist, PoseStamped, Quaternion
from nav2_msgs.action import DockRobot, NavigateToPose
from action_msgs.msg import GoalStatus
from sensor_msgs.msg import BatteryState
from rclpy.time import Time
from tf2_ros import TransformException

ROOT=Path(__file__).resolve().parent
SCRIPTS=ROOT if (ROOT/'waypoint_manager_node.py').exists() else ROOT.parent/'scripts'
sys.path.insert(0,str(SCRIPTS))
from short_distance_servo import body_frame_error, reproject_target_pose
from charger_waypoints import is_charger_waypoint
def extract(filename, methods):
    tree=ast.parse((SCRIPTS/filename).read_text())
    original=next(n for n in tree.body if isinstance(n,ast.ClassDef))
    cls=ast.ClassDef(name='Subject',bases=[],keywords=[],decorator_list=[],
        body=[n for n in original.body if isinstance(n,ast.FunctionDef) and n.name in methods])
    ns=dict(globals());exec(compile(ast.fix_missing_locations(ast.Module(body=[cls],type_ignores=[])),filename,'exec'),ns)
    return ns['Subject']

Manager=extract('waypoint_manager_node.py',{
    '_navigation_target','_build_goal','_start_navigation','_battery_state_fresh',
    '_start_precise_docking','_complete_short_distance','_handle_docking_goal_response',
    '_complete_goal_by_tolerance','_handle_docking_result','_handle_battery_state',
    '_check_docking_drift','_finish_dock_abort','_clear_occupied_wait','_uses_mppi_only',
})
Arbiter=extract('cmd_vel_arbiter_node.py',{'_filter_nav_cmd_vel','_limit_staging_approach_cmd_vel'})

def manager():
    n=Manager(); n.__dict__.update(
        _waypoints={'充电桩':dict(x=2.,y=3.,yaw=0.,frame_id='map')},
        _charger_waypoint_name='充电桩',_map_frame='map',_odom_frame='odom',_dock_staging_active=False,
        _dock_staging_distance=.55,_dock_prealign_enabled=True,_precise_docking_enabled=True,
        _short_servo_enabled=True,_far_navigation_auto_refine_enabled=True,
        _short_servo_active=False,_short_direct_start_enabled=True,_active_goal_id=0,
        _goal_futures=[],_result_futures=[],_short_xy_tolerance=.04,_short_yaw_tolerance=.05,
        _battery_is_charging=False,_last_battery_status_time=Time(seconds=9),
        _dock_pose_yaw_offset=math.pi,_dock_type='fine_charging_dock',
        _dock_max_staging_time=120.,_dock_navigate_to_staging_pose=True,
        _voice_departure_template='go',_charging_status_confirm_samples=2,
        _charging_status_count=0,_charging_detection_armed=True,
    )
    n.get_clock=lambda:NS(now=lambda:Time(seconds=10))
    n.get_logger=lambda:Mock()
    n._normalize_angle=lambda yaw:math.remainder(yaw,2*math.pi)
    n._quaternion_from_yaw=lambda yaw:Quaternion(z=math.sin(yaw/2),w=math.cos(yaw/2))
    n._navigate_client=Mock();n._dock_client=Mock()
    n._robot_pose_in_frame=lambda frame:(2.55,3.,0.)
    n._lookup_current_pose=lambda:(2.55,3.,0.)
    n._dock_verify_xy_tolerance=.08;n._dock_verify_yaw_tolerance=.10
    n._pose_in_frame=lambda waypoint,frame:(waypoint['x'],waypoint['y'],waypoint['yaw'])
    for name in ('_publish_waypoint_markers','_publish_navigation_status','_publish_voice_prompt',
        '_fail_precise_docking','_fallback_short_distance_to_nav2','_stop_sequence','_complete_docking_by_charge_status'):
        setattr(n,name,Mock())
    n._try_start_short_distance=Mock(side_effect=AssertionError('direct short dispatch must not run'))
    return n

class DockTests(unittest.TestCase):
    def test_staging_geometry_all_headings_and_no_library_edit(self):
        n=manager(); n._dock_staging_active=True
        for yaw in (0,math.pi/2,math.pi,-math.pi/2):
            n._waypoints['充电桩']['yaw']=yaw
            pose=n._navigation_target('充电桩')
            self.assertAlmostEqual(pose['x'],2+.55*math.cos(yaw))
            self.assertAlmostEqual(pose['y'],3+.55*math.sin(yaw))
            self.assertEqual(n._waypoints['充电桩']['x'],2.)

    def test_charger_starts_nav2_not_dock_and_not_direct_servo(self):
        n=manager(); self.assertTrue(n._start_navigation('充电桩')[0])
        goal=n._navigate_client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(goal.pose.pose.position.x,2.55)
        n._dock_client.send_goal_async.assert_not_called()

    def test_stale_battery_and_already_charging_reject_motion(self):
        for stamp,charging in ((Time(seconds=5),False),(Time(seconds=9),True)):
            n=manager();n._last_battery_status_time=stamp;n._battery_is_charging=charging
            self.assertFalse(n._start_navigation('充电桩')[0])
            n._navigate_client.send_goal_async.assert_not_called()
            n._dock_client.send_goal_async.assert_not_called()

    def test_strict_refinement_required(self):
        n=manager();n._far_navigation_auto_refine_enabled=False
        self.assertFalse(n._start_navigation('充电桩')[0])

    def test_servo_stops_before_dock_goal_without_arrival(self):
        n=manager();n._short_servo_active=True;n._dock_staging_active=True;n._active_goal_name='充电桩'
        n._waypoint_pose_error=lambda name:(.02,.02)
        n._deactivate_short_distance=lambda:setattr(n,'_short_servo_active',False)
        original=n._dock_client.send_goal_async
        def send(*args,**kwargs):
            self.assertFalse(n._short_servo_active)
            return original(*args,**kwargs)
        n._dock_client.send_goal_async=send
        n._complete_short_distance(.02,.02)
        goal=original.call_args.args[0]
        self.assertFalse(goal.navigate_to_staging_pose)
        self.assertEqual(goal.dock_pose.header.frame_id,'odom')
        self.assertAlmostEqual(goal.dock_pose.pose.position.x,2.)
        self.assertAlmostEqual(abs(goal.dock_pose.pose.orientation.z),1.)
        self.assertFalse(n._dock_staging_active)
        self.assertFalse(any('succeeded' in str(c) for c in n._publish_navigation_status.call_args_list))

    def test_changed_pose_does_not_start_docking(self):
        n=manager();n._short_servo_active=True;n._dock_staging_active=True;n._active_goal_name='充电桩'
        n._waypoint_pose_error=lambda name:(.2,.02)
        n._complete_short_distance(.02,.02)
        n._dock_client.send_goal_async.assert_not_called()
        n._fallback_short_distance_to_nav2.assert_called_once()

    def test_broad_arrival_never_completes_charger_staging(self):
        n=manager();n._dock_staging_active=True
        n._complete_goal_by_tolerance('充电桩',.05,.05)
        n._publish_navigation_status.assert_not_called()

    def test_stale_accepted_dock_is_cancelled(self):
        n=manager();n._active_goal_id=5;handle=Mock(accepted=True)
        n._handle_docking_goal_response(NS(result=lambda:handle),'充电桩',4)
        handle.cancel_goal_async.assert_called_once()

    def test_pose_success_without_charge_is_failure(self):
        n=manager();result=NS(status=GoalStatus.STATUS_SUCCEEDED,result=NS(success=True))
        n._handle_docking_result(NS(result=lambda:result),'充电桩',0)
        n._fail_precise_docking.assert_called_once()

    def test_charge_needs_two_fresh_messages_in_contact_stage(self):
        n=manager();n._precise_docking_active=True;n._last_docking_feedback_state=DockRobot.Feedback.CONTROLLING
        b=BatteryState(power_supply_status=BatteryState.POWER_SUPPLY_STATUS_CHARGING)
        n._handle_battery_state(b);n._complete_docking_by_charge_status.assert_not_called()
        n._handle_battery_state(b);n._complete_docking_by_charge_status.assert_called_once()

    def test_filter_stop_small_reverse_and_steering(self):
        n=Arbiter();n._charging_detected=False;n._charging_hold_active=lambda:False
        n._staging_approach_slowdown_is_active=lambda:False;n._straight_docking_is_active=lambda:True
        n._docking_distance=.3;n._straight_docking_stop_distance=0.;n._straight_docking_blocked=False
        n._staging_approach_max_linear_speed=.12;n._staging_approach_max_angular_speed=.18
        msg=Twist();self.assertEqual(n._filter_nav_cmd_vel(msg).linear.x,0.)
        msg.linear.x=-.01;msg.angular.z=.08
        out=n._filter_nav_cmd_vel(msg);self.assertEqual(out.linear.x,-.01);self.assertEqual(out.angular.z,.08)
        msg.linear.x=.01;self.assertEqual(n._filter_nav_cmd_vel(msg).linear.x,0.)
        msg.linear.x=float('nan');self.assertEqual(n._filter_nav_cmd_vel(msg).linear.x,0.)
        msg.linear.x=-.04;n._straight_docking_blocked=True
        self.assertEqual(n._filter_nav_cmd_vel(msg).linear.x,0.)

    def test_staging_retains_lateral_velocity(self):
        n=Arbiter();n._staging_approach_max_linear_speed=.12;n._staging_approach_max_angular_speed=.18
        n._staging_approach_distance=.5;n._staging_final_heading_distance=.25
        msg=Twist();msg.linear.y=.06
        self.assertEqual(n._limit_staging_approach_cmd_vel(msg).linear.y,.06)

    def test_dock_target_is_frozen_in_odom_at_handover(self):
        n=manager();n._robot_pose_in_frame=lambda frame:(2.55,3.,0.) if frame=='map' else (12.55,3.,0.)
        self.assertTrue(n._start_precise_docking('充电桩',staging_aligned=True)[0])
        goal=n._dock_client.send_goal_async.call_args.args[0]
        self.assertAlmostEqual(goal.dock_pose.pose.position.x,12.)
        n._robot_pose_in_frame=lambda frame:(9.,9.,0.)
        self.assertEqual(n._dock_target_odom_pose,(12.,3.,0.))

    def test_large_drift_stops_and_waits_for_terminal_cancel(self):
        n=manager();n._precise_docking_active=True;n._active_goal_name='充电桩'
        n._active_dock_goal_handle=Mock();handle=n._active_dock_goal_handle
        n._dock_target_odom_pose=(2.,3.,0.);n._robot_pose_in_frame=lambda frame:(2.3,3.08,0.)
        n._dock_max_lateral_drift=.06;n._dock_max_heading_drift=.14
        n._last_docking_feedback_state=DockRobot.Feedback.CONTROLLING
        n._short_cmd_vel_pub=Mock()
        n._check_docking_drift()
        handle.cancel_goal_async.assert_called_once();n._short_cmd_vel_pub.publish.assert_called_once()
        self.assertTrue(n._dock_cancel_pending)
        self.assertFalse(n._start_navigation('充电桩')[0])
        n._finish_dock_abort(NS(result=lambda:NS(status=GoalStatus.STATUS_CANCELED)))
        self.assertFalse(n._dock_cancel_pending)

    def test_small_drift_keeps_existing_controller_in_charge(self):
        n=manager();n._precise_docking_active=True;n._active_goal_name='充电桩'
        n._active_dock_goal_handle=Mock();n._dock_target_odom_pose=(2.,3.,0.)
        n._robot_pose_in_frame=lambda frame:(2.3,3.02,.03)
        n._dock_max_lateral_drift=.06;n._dock_max_heading_drift=.14
        n._last_docking_feedback_state=DockRobot.Feedback.CONTROLLING
        n._check_docking_drift();n._active_dock_goal_handle.cancel_goal_async.assert_not_called()

    def test_inhibit_suppresses_late_controller_velocity(self):
        n=Arbiter();n._dock_output_inhibit=True
        msg=Twist();msg.linear.x=-.04
        self.assertEqual(n._filter_nav_cmd_vel(msg).linear.x,0.)

if __name__=='__main__':unittest.main(verbosity=2)
