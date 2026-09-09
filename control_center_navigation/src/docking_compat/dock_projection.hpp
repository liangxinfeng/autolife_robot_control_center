#pragma once

#include <cmath>
#include "geometry_msgs/msg/pose.hpp"
#include "nav2_util/geometry_utils.hpp"
#include "tf2/utils.h"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"

// The dock's +X points beyond the contact point. Project in this frame BEFORE
// rotating the controller's orientation for reverse driving (Nav2 Jazzy fix).
inline geometry_msgs::msg::Pose projectDockControlTarget(
  geometry_msgs::msg::Pose pose, bool backwards)
{
  const double dock_yaw = tf2::getYaw(pose.orientation);
  pose.position.x += 0.25 * std::cos(dock_yaw);
  pose.position.y += 0.25 * std::sin(dock_yaw);
  if (backwards) {
    pose.orientation = nav2_util::geometry_utils::orientationAroundZAxis(dock_yaw + M_PI);
  }
  return pose;
}
