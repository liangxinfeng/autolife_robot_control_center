// Test the actual plugin gate without loading costmaps or publishing motion.
#include "settled_mppi.cpp"
#include <thread>
void check(bool value){if(!value)throw std::runtime_error("arrival certificate regression");}
int main(int argc,char **argv){
  rclcpp::init(argc,argv);
  auto node=std::make_shared<rclcpp_lifecycle::LifecycleNode>("arrival_gate_unit_test");
  node->declare_parameter("test.xy_goal_tolerance",.05);
  node->declare_parameter("test.yaw_goal_tolerance",.05);
  node->declare_parameter("test.stateful",false);
  node->declare_parameter("test.trans_stopped_velocity",.015);
  node->declare_parameter("test.rot_stopped_velocity",.025);
  control_center_navigation::SettledGoalChecker gate;
  gate.initialize(node,"test",nullptr);
  geometry_msgs::msg::Pose pose,goal;pose.orientation.w=goal.orientation.w=1.;
  geometry_msgs::msg::Twist speed;
  gate.observe(true,.02,.02);check(!gate.isGoalReached(pose,goal,speed));
  std::this_thread::sleep_for(std::chrono::milliseconds(370));
  gate.observe(true,.02,.02);check(gate.isGoalReached(pose,goal,speed));
  speed.linear.x=.02;check(!gate.isGoalReached(pose,goal,speed));speed.linear.x=0;
  gate.observe(true,.051,.02);check(!gate.isGoalReached(pose,goal,speed));
  gate.observe(true,.02,.051);check(!gate.isGoalReached(pose,goal,speed));
  gate.observe(true,.02,.02);pose.position.x=.051;check(!gate.isGoalReached(pose,goal,speed));
  pose.position.x=0;gate.reset();check(!gate.isGoalReached(pose,goal,speed));
  gate.observe(true,.02,.02);std::this_thread::sleep_for(std::chrono::milliseconds(210));
  check(!gate.isGoalReached(pose,goal,speed));
  rclcpp::shutdown();
}
