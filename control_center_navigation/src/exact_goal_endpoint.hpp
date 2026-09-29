#pragma once
#include <algorithm>
#include <cmath>
#include <functional>
#include <stdexcept>
#include "nav_msgs/msg/path.hpp"

namespace control_center_navigation
{
// Correct only grid quantization, never bridge to a remote fallback endpoint.
inline void preserve_exact_goal(
  nav_msgs::msg::Path & path, const geometry_msgs::msg::PoseStamped & goal,
  double resolution, const std::function<bool(double, double)> & safe)
{
  if (path.poses.empty() || resolution <= 0.0) {
    throw std::runtime_error("empty path or invalid costmap resolution");
  }
  const auto from = path.poses.back().pose.position;
  const auto to = goal.pose.position;
  const double distance = std::hypot(to.x - from.x, to.y - from.y);
  if (!std::isfinite(distance) || distance > std::sqrt(2.0) * resolution + 1e-6) {
    throw std::runtime_error("planner endpoint is not in the requested goal cell");
  }
  const int count = std::max(1, static_cast<int>(std::ceil(distance / (resolution * .2))));
  for (int i = 0; i <= count; ++i) {
    const double t = static_cast<double>(i) / count;
    if (!safe(from.x + t * (to.x - from.x), from.y + t * (to.y - from.y))) {
      throw std::runtime_error("requested goal or final connection is obstructed");
    }
  }
  auto exact = goal;
  exact.header = path.header;
  if (distance < 1e-6) {path.poses.back() = exact;}
  else {path.poses.push_back(exact);}
}
}
