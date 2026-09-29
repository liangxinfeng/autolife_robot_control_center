#pragma once
#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace control_center_navigation {
// An end-of-return weak tail, not a general intensity or near-map-wall filter.
// No temporal cache, map lookup, ray extension or replacement ranges.
struct WallTailConfig {
  double weak_intensity = 8.0;
  double strong_intensity = 14.0;
  double protected_range = 0.40;
  double max_range = 1.5;
  double max_deviation = 0.08;
};
struct WallTailPoint { double x, y; };
inline std::vector<int> weakWallTailIndices(
  const std::vector<float>& ranges, const std::vector<float>& intensities,
  double angle_min, double increment, double range_min, double range_max,
  const WallTailConfig& cfg = {})
{
  const int n = static_cast<int>(ranges.size());
  std::vector<int> removed;
  if (n < 48 || intensities.size() != ranges.size() || !std::isfinite(increment) ||
      std::abs(increment) < 1e-5 || std::abs(increment) > 0.02) return removed;
  const auto valid = [&](int i) {
    return i >= 0 && i < n && std::isfinite(ranges[i]) &&
      ranges[i] >= range_min && ranges[i] <= range_max;
  };
  const auto point = [&](int i) {
    const double a = angle_min + i * increment;
    return WallTailPoint{ranges[i] * std::cos(a), ranges[i] * std::sin(a)};
  };
  // Only inspect boundaries next to >=3 unobserved rays. Scan seam is not a gap.
  for (int e = 3; e < n-3; ++e) {
    if (!valid(e)) continue;
    for (int step : {-1, 1}) {
      if (valid(e+step) || valid(e+2*step) || valid(e+3*step)) continue;
      std::vector<int> tail;
      int j = e, gaps = 0;
      const auto weak = [&](int i) {
        return valid(i) && std::isfinite(intensities[i]) && intensities[i]>=0 &&
          intensities[i]<=cfg.weak_intensity;
      };
      while (j>=0 && j<n && std::abs(e-j)<=32) {
        if (weak(j)) { tail.push_back(j); j-=step; continue; }
        // A few missing bins inside an otherwise geometrically continuous tail
        // are not fabricated or interpolated. The same point-spacing check
        // below still applies across them. Never bridge to a separate wall.
        if (!valid(j) && gaps<3 && (weak(j-step) || weak(j-2*step))) {
          ++gaps; j-=step; continue;
        }
        break;
      }
      if (tail.size() < 4 || tail.size() > 32) continue;
      std::reverse(tail.begin(), tail.end());  // wall -> end, increasing departure
      std::vector<WallTailPoint> anchor;
      int strong = 0;
      bool good = true;
      for (int k = 0; k < 32; ++k, j -= step) {
        if (!valid(j) || !std::isfinite(intensities[j]) || intensities[j] < 0 ||
            ranges[j] < cfg.protected_range || ranges[j] > cfg.max_range) {
          good = false; break;
        }
        strong += intensities[j] >= cfg.strong_intensity;
        anchor.push_back(point(j));
      }
      if (!good || strong < 24) continue;
      WallTailPoint mean{0,0};
      for (auto p : anchor) { mean.x += p.x/32.; mean.y += p.y/32.; }
      double xx=0, xy=0, yy=0;
      for (auto p : anchor) {
        const double x=p.x-mean.x, y=p.y-mean.y;
        xx+=x*x; xy+=x*y; yy+=y*y;
      }
      const double theta=.5*std::atan2(2*xy,xx-yy);
      double nx=-std::sin(theta), ny=std::cos(theta);
      if (nx*mean.x+ny*mean.y>0) { nx=-nx; ny=-ny; } // toward sensor
      const auto departure = [&](WallTailPoint p) {
        return (p.x-mean.x)*nx+(p.y-mean.y)*ny;
      };
      double sum2=0, max_residual=0;
      for (auto p : anchor) {
        const double d=departure(p); sum2+=d*d; max_residual=std::max(max_residual,std::abs(d));
      }
      if (std::hypot(anchor.front().x-anchor.back().x,anchor.front().y-anchor.back().y)<.10 ||
          std::sqrt(sum2/32.)>.005 || max_residual>.012) continue;
      std::vector<int> candidate;
      double prev=0, max_d=0;
      const auto start=point(tail.front());
      WallTailPoint last=anchor.front();
      for (int i : tail) {
        const auto p=point(i); const double d=departure(p);
        // Preserve close returns, abrupt corners, large intrusions and tails
        // that turn back toward the wall (e.g. a separate thin object).
        if (ranges[i]<cfg.protected_range || ranges[i]>cfg.max_range ||
            std::hypot(p.x-last.x,p.y-last.y)>.025 ||
            std::hypot(p.x-start.x,p.y-start.y)>.20 ||
            d<-.012 || d>cfg.max_deviation || d<prev-.012) { good=false; break; }
        if (d>.018) candidate.push_back(i);
        max_d=std::max(max_d,d);prev=d;last=p;
      }
      if (good && max_d>=.03 && candidate.size()>=3)
        removed.insert(removed.end(),candidate.begin(),candidate.end());
    }
  }
  std::sort(removed.begin(),removed.end());
  removed.erase(std::unique(removed.begin(),removed.end()),removed.end());
  return removed;
}
} // namespace control_center_navigation
