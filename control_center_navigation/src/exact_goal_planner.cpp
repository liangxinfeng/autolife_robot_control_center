#include "nav2_smac_planner/smac_planner_2d.hpp"
#include "nav2_costmap_2d/footprint_collision_checker.hpp"
#include "nav2_core/planner_exceptions.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "exact_goal_endpoint.hpp"
#include "terminal_approach.hpp"
#include "exact_near_goal.hpp"
#include <algorithm>
#include <cmath>
#include <optional>

namespace control_center_navigation
{
// Keep upstream Smac search/smoothing and MPPI collision checking. Only remove
// the grid/fallback goal mismatch at the interface between planner and controller.
class ExactGoalSmacPlanner : public nav2_smac_planner::SmacPlanner2D
{
public:
  void configure(const rclcpp_lifecycle::LifecycleNode::WeakPtr & parent,
    std::string name, std::shared_ptr<tf2_ros::Buffer> tf,
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros) override
  {
    costmap_ros_ = costmap_ros;
    SmacPlanner2D::configure(parent, name, tf, costmap_ros);
    auto node = parent.lock();
    if (!node->has_parameter(name + ".allow_occupied_approach")) {
      node->declare_parameter(name + ".allow_occupied_approach", false);
    }
    approach_enabled_ = node->get_parameter(name + ".allow_occupied_approach").as_bool();
    if (!node->has_parameter(name + ".align_terminal_approach")) {
      node->declare_parameter(name + ".align_terminal_approach", true);
    }
    align_terminal_ = node->get_parameter(name + ".align_terminal_approach").as_bool();
  }

  nav_msgs::msg::Path createPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal,
    std::function<bool()> cancelled) override
  {
    bool occupied = false;
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
      unsigned int mx, my;
      if (!_costmap->worldToMap(goal.pose.position.x, goal.pose.position.y, mx, my)) {
        throw nav2_core::GoalOutsideMapBounds("requested goal is outside the map");
      }
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(_costmap);
      const double cost = checker.footprintCostAtPose(goal.pose.position.x,
        goal.pose.position.y, tf2::getYaw(goal.pose.orientation), costmap_ros_->getRobotFootprint());
      if (_costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE ||
        cost < 0 || cost >= nav2_costmap_2d::LETHAL_OBSTACLE)
      {
        occupied = true;
      }
    }
    if (occupied) {
      if (!approach_enabled_) {
        throw nav2_core::GoalOccupied("requested goal is occupied or within collision clearance");
      }
      return approachPlan(start, goal, cancelled);
    }
    approach_.reset();
    return exactPlan(start, goal, cancelled);
  }

private:
  nav_msgs::msg::Path exactPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancelled)
  {
    {
      std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
      nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(_costmap);
      const auto footprint=costmap_ros_->getRobotFootprint();
      auto near=exact_near_goal(start,goal,_costmap->getResolution(),
        [&](double x,double y,double yaw){
          unsigned int mx,my;
          if(!_costmap->worldToMap(x,y,mx,my) ||
             _costmap->getCost(mx,my)>=nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return false;
          const double cost=checker.footprintCostAtPose(x,y,yaw,footprint);
          return cost>=0 && cost<nav2_costmap_2d::LETHAL_OBSTACLE;
        },cancelled);
      if(cancelled())throw nav2_core::PlannerCancelled("planning cancelled");
      if(near)return *near;
    }
    auto plan = SmacPlanner2D::createPlan(start, goal, cancelled);
    if (cancelled()) {throw nav2_core::PlannerCancelled("planning cancelled");}
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(_costmap);
    const auto footprint = costmap_ros_->getRobotFootprint();
    const double yaw = tf2::getYaw(goal.pose.orientation);
    try {
      preserve_exact_goal(plan, goal, _costmap->getResolution(), [&](double x, double y) {
        unsigned int mx, my;
        if (!_costmap->worldToMap(x, y, mx, my) ||
          _costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE)
        {return false;}
        const double cost = checker.footprintCostAtPose(x, y, yaw, footprint);
        return cost >= 0 && cost < nav2_costmap_2d::LETHAL_OBSTACLE;
      });
    } catch (const std::runtime_error & e) {
      throw nav2_core::NoValidPathCouldBeFound(e.what());
    }
    if (align_terminal_) {
      align_terminal_approach(plan, [&](double x, double y, double heading) {
        unsigned int mx, my;
        if (!_costmap->worldToMap(x, y, mx, my) ||
          _costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return false;
        const double cost = checker.footprintCostAtPose(x, y, heading, footprint);
        return cost >= 0 && cost < nav2_costmap_2d::LETHAL_OBSTACLE;
      });
    }
    return plan;
  }

  bool safeWaitingPose(const geometry_msgs::msg::PoseStamped & pose)
  {
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*_costmap->getMutex());
    unsigned int mx, my;
    if (!_costmap->worldToMap(pose.pose.position.x, pose.pose.position.y, mx, my) ||
      _costmap->getCost(mx, my) >= nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE) return false;
    nav2_costmap_2d::FootprintCollisionChecker<nav2_costmap_2d::Costmap2D *> checker(_costmap);
    const double cost = checker.footprintCostAtPose(pose.pose.position.x, pose.pose.position.y,
      tf2::getYaw(pose.pose.orientation), costmap_ros_->getRobotFootprint());
    return cost >= 0 && cost < nav2_costmap_2d::LETHAL_OBSTACLE;
  }

  nav_msgs::msg::Path approachPlan(const geometry_msgs::msg::PoseStamped & start,
    const geometry_msgs::msg::PoseStamped & goal, std::function<bool()> cancelled)
  {
    // This planner is ONLY selected by occupied_approach.xml, whose final gate
    // still checks the ORIGINAL goal. Never use it with an ordinary success BT.
    const bool same = approach_ && goal.header.frame_id == requested_.header.frame_id &&
      std::hypot(goal.pose.position.x-requested_.pose.position.x,
        goal.pose.position.y-requested_.pose.position.y) < 1e-5 &&
      std::abs(angles::shortest_angular_distance(tf2::getYaw(goal.pose.orientation),
        tf2::getYaw(requested_.pose.orientation))) < 1e-5;
    if (!same) approach_.reset();
    requested_ = goal;
    const double remaining = std::hypot(start.pose.position.x-goal.pose.position.x,
      start.pose.position.y-goal.pose.position.y);
    if (!approach_ && remaining <= .85) {
      throw nav2_core::GoalOccupied("already near occupied goal; stop and wait without backing away");
    }
    std::vector<geometry_msgs::msg::PoseStamped> candidates;
    // Retain one waiting pose across periodic replans; avoid chasing a moving ring.
    if (approach_ && safeWaitingPose(*approach_)) candidates.push_back(*approach_);
    const double bearing = std::atan2(start.pose.position.y-goal.pose.position.y,
      start.pose.position.x-goal.pose.position.x);
    for (double radius : {0.8, 1.1, 1.4}) {
      if (radius >= remaining-.15) continue;
      for (double angle : {0., .5, -.5, 1., -1.}) {
        auto candidate = goal;
        candidate.pose.position.x += radius*std::cos(bearing+angle);
        candidate.pose.position.y += radius*std::sin(bearing+angle);
        if (safeWaitingPose(candidate)) candidates.push_back(candidate);
      }
    }
    // Bounded search, original costmap untouched. No obstacle clearing, unknown
    // endpoint, straight-line shortcut, or unverified path reuse is permitted.
    size_t tried = 0;
    for (auto candidate : candidates) {
      if (cancelled()) throw nav2_core::PlannerCancelled("approach planning cancelled");
      if (++tried > 3) break;
      candidate.header.stamp = goal.header.stamp;
      const auto saved_time = _max_planning_time;
      _max_planning_time = std::min(saved_time, .3);
      try {
        auto path = exactPlan(start, candidate, cancelled);
        _max_planning_time = saved_time;
        approach_ = candidate;
        return path;
      } catch (const nav2_core::PlannerCancelled &) {
        _max_planning_time = saved_time;
        throw;
      } catch (const nav2_core::PlannerException &) {
        _max_planning_time = saved_time;
      } catch (...) {
        _max_planning_time = saved_time;
        throw;
      }
    }
    throw nav2_core::GoalOccupied("goal occupied; no safe reachable waiting pose near goal");
  }
  bool approach_enabled_ = false;
  bool align_terminal_ = true;
  std::optional<geometry_msgs::msg::PoseStamped> approach_;
  geometry_msgs::msg::PoseStamped requested_;
  std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};
}
PLUGINLIB_EXPORT_CLASS(control_center_navigation::ExactGoalSmacPlanner, nav2_core::GlobalPlanner)
