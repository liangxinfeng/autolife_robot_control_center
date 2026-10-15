#pragma once
#include <algorithm>
#include <cmath>
#include <vector>

namespace control_center_navigation
{
// A waiting endpoint is not arrival. Every generated candidate still goes
// through the unchanged footprint, exact endpoint and full path checks.
inline std::vector<double> occupied_approach_radii(double remaining, double resolution)
{
  std::vector<double> radii;
  if (!std::isfinite(remaining) || !std::isfinite(resolution) || resolution <= 0.) {
    return radii;
  }
  const double progress = std::max(.10, 2. * resolution);
  const double upper = remaining - progress;
  const double lower = std::max(.35, 3. * resolution);
  if (upper < lower - 1e-6) {return radii;}
  if (remaining <= 1.05) {
    // In the old 0.85--0.95m gap, all fixed 0.8m+ candidates were
    // discarded. Sample the reachable interval instead of declaring blocked.
    for (double radius = lower; radius < upper - .04; radius += .15) {
      radii.push_back(radius);
    }
    radii.push_back(upper);
  } else {
    for (double radius : {.8, 1.1, 1.4}) {
      if (radius <= upper + 1e-6) {radii.push_back(radius);}
    }
  }
  return radii;
}
}
