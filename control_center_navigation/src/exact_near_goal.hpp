#pragma once
#include <algorithm>
#include <cmath>
#include <optional>
#include "nav_msgs/msg/path.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2/utils.h"

namespace control_center_navigation {
// Avoid a grid-centre detour once the real poses are within two cells. This is
// only a checked geometric connection, never a controller or arrival shortcut.
template<class Safe, class Cancelled>
std::optional<nav_msgs::msg::Path> exact_near_goal(
  const geometry_msgs::msg::PoseStamped& start,
  const geometry_msgs::msg::PoseStamped& goal,
  double resolution, Safe safe, Cancelled cancelled)
{
  const double dx=goal.pose.position.x-start.pose.position.x;
  const double dy=goal.pose.position.y-start.pose.position.y;
  const double distance=std::hypot(dx,dy);
  const double yaw=tf2::getYaw(start.pose.orientation);
  const double turn=std::remainder(tf2::getYaw(goal.pose.orientation)-yaw,2*M_PI);
  if(start.header.frame_id!=goal.header.frame_id || !std::isfinite(resolution) ||
     resolution<=0 || !std::isfinite(distance) || !std::isfinite(yaw) ||
     !std::isfinite(turn) || distance>std::min(.05,2*resolution) ||
     std::abs(turn)>.25) return std::nullopt;
  // At 55cm body size, each corner moves <=~4.5mm per check including rotation;
  // use denser checks if the grid is finer. Caller uses its original footprint.
  const double ds=std::min(.0025,resolution*.1);
  const int steps=std::max(1,int(std::ceil(std::max(distance/ds,std::abs(turn)/.005))));
  nav_msgs::msg::Path path;path.header=goal.header;
  for(int i=0;i<=steps;++i){
    if(cancelled())return std::nullopt;
    const double t=double(i)/steps,x=start.pose.position.x+t*dx,
      y=start.pose.position.y+t*dy,a=yaw+t*turn;
    if(!safe(x,y,a))return std::nullopt;
    auto pose=goal;pose.pose.position.x=x;pose.pose.position.y=y;
    pose.pose.orientation.x=pose.pose.orientation.y=0;
    pose.pose.orientation.z=std::sin(a*.5);pose.pose.orientation.w=std::cos(a*.5);
    path.poses.push_back(pose);
  }
  path.poses.front()=start;path.poses.front().header=path.header;
  path.poses.back()=goal;
  return path;
}
}
