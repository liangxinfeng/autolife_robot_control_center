#include <atomic>
#include <chrono>
#include "nav2_mppi_controller/controller.hpp"
#include "nav2_controller/plugins/stopped_goal_checker.hpp"
#include "nav2_core/controller_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "arrival_settle_policy.hpp"

namespace control_center_navigation {
using Steady=std::chrono::steady_clock;
class SettledGoalChecker : public nav2_controller::StoppedGoalChecker {
public:
  void reset() override {StoppedGoalChecker::reset(); stable_=false; fresh_=false;}
  void observe(bool holding,double distance,double angle){
    eligible_=holding && distance<=.05 && angle<=.05;
    observed_=Steady::now();fresh_=true;
  }
  bool isGoalReached(const geometry_msgs::msg::Pose & pose,
    const geometry_msgs::msg::Pose & goal,const geometry_msgs::msg::Twist & velocity) override {
    const auto now=Steady::now();
    const bool good=fresh_ && std::chrono::duration<double>(now-observed_).count()<.2 &&
      eligible_ && StoppedGoalChecker::isGoalReached(pose,goal,velocity);
    if(!good){stable_=false;return false;}
    if(!stable_){since_=now;stable_=true;}
    return std::chrono::duration<double>(now-since_).count()>=.35;
  }
private:
  bool eligible_=false,fresh_=false,stable_=false;
  Steady::time_point observed_,since_;
};

class SettledMPPIController : public nav2_mppi_controller::MPPIController {
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,std::string name,
    const std::shared_ptr<tf2_ros::Buffer> tf,
    const std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap) override {
    MPPIController::configure(parent,name,tf,costmap);
    max_speed_=parent.lock()->get_parameter(name+".vx_max").as_double();
    if(max_speed_<=0)throw nav2_core::ControllerException("invalid forward speed limit");
  }
  void setPlan(const nav_msgs::msg::Path & path) override {
    if(path.poses.empty())throw nav2_core::InvalidPath("empty path");
    const auto & g=path.poses.back();
    // Periodic replanning of the SAME goal must not reset the settling state.
    if(!has_goal_ || g.header.frame_id!=goal_.header.frame_id ||
      std::hypot(g.pose.position.x-goal_.pose.position.x,g.pose.position.y-goal_.pose.position.y)>1e-5 ||
      std::abs(angles::shortest_angular_distance(tf2::getYaw(goal_.pose.orientation),tf2::getYaw(g.pose.orientation)))>1e-5)
    {policy_.reset();}
    goal_=g;has_goal_=true;
    MPPIController::setPlan(path);
  }
  void reset() override {policy_.reset();has_goal_=false;MPPIController::reset();}
  void setSpeedLimit(const double & limit,const bool & percent) override {
    external_percent_=limit<=0?100.:std::clamp(percent?limit:100.*limit/max_speed_,0.,100.);
    MPPIController::setSpeedLimit(limit,percent);
  }
  geometry_msgs::msg::TwistStamped computeVelocityCommands(
    const geometry_msgs::msg::PoseStamped & pose,const geometry_msgs::msg::Twist & speed,
    nav2_core::GoalChecker * checker) override {
    auto gate=dynamic_cast<SettledGoalChecker *>(checker);
    if(!gate || !has_goal_)throw nav2_core::ControllerException("settled MPPI requires matching goal checker and goal");
    geometry_msgs::msg::TransformStamped tf;
    try {tf=tf_buffer_->lookupTransform(goal_.header.frame_id,costmap_ros_->getBaseFrameID(),tf2::TimePointZero);}
    catch(const tf2::TransformException & e){gate->observe(false,1.,1.);throw nav2_core::ControllerTFError(e.what());}
    const double age=(parent_.lock()->now()-rclcpp::Time(tf.header.stamp)).seconds();
    if(age>.3 || age<-.1){gate->observe(false,1.,1.);throw nav2_core::ControllerTFError("stale arrival transform");}
    const double distance=std::hypot(tf.transform.translation.x-goal_.pose.position.x,
      tf.transform.translation.y-goal_.pose.position.y);
    const double angle=std::abs(angles::shortest_angular_distance(tf2::getYaw(tf.transform.rotation),tf2::getYaw(goal_.pose.orientation)));
    const auto local_goal=path_handler_.getTransformedGoal(pose.header.stamp);
    const double local_distance=std::hypot(pose.pose.position.x-local_goal.pose.position.x,
      pose.pose.position.y-local_goal.pose.position.y);
    const double local_angle=std::abs(angles::shortest_angular_distance(tf2::getYaw(pose.pose.orientation),
      tf2::getYaw(local_goal.pose.orientation)));
    const bool hold=policy_.hold(std::max(distance,local_distance),std::max(angle,local_angle));
    gate->observe(hold,distance,angle);
    if(hold){
      geometry_msgs::msg::TwistStamped zero;zero.header=pose.header;
      return zero;
    }
    MPPIController::setSpeedLimit(ArrivalSettlePolicy::percentage(distance,max_speed_,external_percent_.load()),true);
    return MPPIController::computeVelocityCommands(pose,speed,checker);
  }
private:
  ArrivalSettlePolicy policy_;
  geometry_msgs::msg::PoseStamped goal_;
  bool has_goal_=false;
  double max_speed_=.45;
  std::atomic<double> external_percent_{100.};
};
}
PLUGINLIB_EXPORT_CLASS(control_center_navigation::SettledMPPIController,nav2_core::Controller)
PLUGINLIB_EXPORT_CLASS(control_center_navigation::SettledGoalChecker,nav2_core::GoalChecker)
