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
  // Extra range only for short all-zero-strength tails backed by a stricter
  // stable anchor and retained strong transition. Not a global cutoff.
  double zero_tail_max_deviation = 0.12;
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
        const bool zero_support = !tail.empty() && std::all_of(tail.begin(), tail.end(),
          [&](int i) { return intensities[i]==0; }) &&
          ((weak(j-step) && intensities[j-step]==0) ||
           (weak(j-2*step) && intensities[j-2*step]==0));
        if (!valid(j) && gaps<(zero_support ? 6 : 3) && (weak(j-step) || weak(j-2*step))) {
          ++gaps; j-=step; continue;
        }
        break;
      }
      if (tail.size() < 4 || tail.size() > 32) continue;
      std::reverse(tail.begin(), tail.end());  // wall -> end, increasing departure
      // A few strong transition returns can still bend away from the wall.
      // Search only a bounded contiguous prefix, retaining those strong points.
      // Do not relax the straight-wall fit or cross an unobserved region.
      int adjacent = j;
      // At most two missing boundary bins are left unobserved. Require actual
      // weak/strong neighbours and retain the same 2.5 cm geometric limit.
      if (!valid(adjacent) && weak(adjacent+step)) {
        for (int gap=1; gap<=2; ++gap) {
          const int i=adjacent-gap*step;
          if (!valid(i)) continue;
          if (std::isfinite(intensities[i]) && intensities[i]>=cfg.strong_intensity)
            adjacent=i;
          break;
        }
      }
      int selected_trim = -1;
      double best_rms = std::numeric_limits<double>::infinity();
      double best_max_residual = std::numeric_limits<double>::infinity();
      WallTailPoint mean{0,0}, last{0,0};
      double nx = 0, ny = 0, prev = 0;
      // Preserve the v2 search first. Only if it has no usable wall anchor,
      // inspect a longer retained mixed-strength transition with stricter
      // independent wall evidence. No additional return is deleted here.
      for (int search = 0; search < 2 && selected_trim < 0; ++search) {
      for (int trim = 0; trim <= (search ? 24 : 8); ++trim) {
        bool valid_prefix = true;
        int prefix_missing = 0;
        // protected_range governs deletions, not read-only wall evidence.
        // A nearby strong wall is still useful as an anchor; all of these
        // connector/anchor returns remain untouched, including close ones.
        for (int k = 0; k < trim; ++k) {
          const int i = adjacent - k * step;
          if (!valid(i)) {
            if (k==0 || ++prefix_missing>2 ||
                (!valid(i-step) && !valid(i-2*step)) ||
                (!valid(i+step) && !valid(i+2*step))) {
              valid_prefix=false; break;
            }
            continue;
          }
          if (!std::isfinite(intensities[i]) ||
              intensities[i] < (search ? 0 : cfg.strong_intensity) ||
              ranges[i] > cfg.max_range) {
            valid_prefix = false; break;
          }
        }
        if (!valid_prefix) break;
        std::vector<WallTailPoint> anchor;
        int strong = 0, index = adjacent - trim * step;
        int missing = 0;
        bool good = true;
        for (int k = 0; k < 42 && anchor.size()<32; ++k, index -= step) {
          if (!valid(index)) {
            if (k==0 || ++missing>8 ||
                (!valid(index-step) && !valid(index-2*step)) ||
                (!valid(index+step) && !valid(index+2*step))) {
              good=false; break;
            }
            continue;
          }
          if (!std::isfinite(intensities[index]) || intensities[index] < 0 ||
              ranges[index] > cfg.max_range) {
            good = false; break;
          }
          const auto p=point(index);
          if (!anchor.empty() && std::hypot(p.x-anchor.back().x,p.y-anchor.back().y)>.025) {
            good=false; break;
          }
          strong += intensities[index] >= cfg.strong_intensity;
          anchor.push_back(p);
        }
        if (!good || anchor.size()!=32 || strong < (search ? 28 : 24) ||
            (search && missing+prefix_missing>8)) continue;
        WallTailPoint centre{0,0};
        for (auto p : anchor) { centre.x += p.x/32.; centre.y += p.y/32.; }
        double xx=0, xy=0, yy=0;
        for (auto p : anchor) {
          const double x=p.x-centre.x, y=p.y-centre.y;
          xx+=x*x; xy+=x*y; yy+=y*y;
        }
        const double theta=.5*std::atan2(2*xy,xx-yy);
        double normal_x=-std::sin(theta), normal_y=std::cos(theta);
        if (normal_x*centre.x+normal_y*centre.y>0) {
          normal_x=-normal_x; normal_y=-normal_y;
        }
        const auto residual = [&](WallTailPoint p) {
          return (p.x-centre.x)*normal_x+(p.y-centre.y)*normal_y;
        };
        double sum2=0, max_residual=0;
        for (auto p : anchor) {
          const double d=residual(p); sum2+=d*d;
          max_residual=std::max(max_residual,std::abs(d));
        }
        const double rms=std::sqrt(sum2/32.);
        if (std::hypot(anchor.front().x-anchor.back().x,anchor.front().y-anchor.back().y)<.10 ||
            std::hypot(anchor.front().x-anchor.back().x,anchor.front().y-anchor.back().y)>.35 ||
            rms>(search ? .003 : .005) || max_residual>(search ? .008 : .012)) continue;
        // Keep the skipped strong connector intact and reject gaps, corners,
        // long bridges or a strong ledge protruding more than 5 cm.
        WallTailPoint connector=anchor.front();
        double bridge_length=0, previous=0;
        for (int k=trim-1; k>=0; --k) {
          if (!valid(adjacent-k*step)) continue; // keep the missing bin missing
          const auto p=point(adjacent-k*step); const double d=residual(p);
          const double distance=std::hypot(p.x-connector.x,p.y-connector.y);
          bridge_length+=distance;
          if (distance>.025 || bridge_length>(search ? .10 : .06) ||
              d<-.012 || d>.05 || d<previous-.012) {
            good=false; break;
          }
          previous=d; connector=p;
        }
        if (!good) continue;
        // Select by independent wall quality, never by which fit erases more
        // tail points. An all-zero tail can have a curved strong transition
        // that passes the coarse fit. Compare the same bounded anchor choices
        // instead of accepting the first marginal fit on those tails.
        if (rms<best_rms) {
          best_rms=rms; selected_trim=trim; mean=centre;
          best_max_residual=max_residual;
          nx=normal_x; ny=normal_y; last=connector; prev=previous;
        }
        if (trim==0 && !std::all_of(tail.begin(), tail.end(),
            [&](int i) { return intensities[i]==0; })) break;
      }
      }
      if (selected_trim<0) continue;
      const auto departure = [&](WallTailPoint p) {
        return (p.x-mean.x)*nx+(p.y-mean.y)*ny;
      };
      bool good=true;
      std::vector<int> candidate;
      double max_d=0;
      const auto start=point(tail.front());
      const bool zero_tail = tail.size() <= 16 && std::all_of(tail.begin(), tail.end(),
        [&](int i) { return intensities[i] == 0; });
      const bool zero_certificate = zero_tail && best_rms <= .0035 && best_max_residual <= .012;
      const double deviation_limit = zero_certificate
        ? std::max(cfg.max_deviation, std::min(.12, cfg.zero_tail_max_deviation))
        : cfg.max_deviation;
      int previous_index = adjacent;
      for (int i : tail) {
        const auto p=point(i); const double d=departure(p);
        const int index_gap=std::abs(i-previous_index);
        const double spacing_limit=zero_certificate && index_gap>1
          ? std::min(.04, .025+.005*(index_gap-1)) : .025;
        // Preserve close returns, abrupt corners, large intrusions and tails
        // that turn back toward the wall (e.g. a separate thin object).
        if (ranges[i]<cfg.protected_range || ranges[i]>cfg.max_range ||
            std::hypot(p.x-last.x,p.y-last.y)>spacing_limit ||
            std::hypot(p.x-start.x,p.y-start.y)>.20 ||
            d<-.012 || d>deviation_limit || d<prev-.012) { good=false; break; }
        if (d>.018) candidate.push_back(i);
        max_d=std::max(max_d,d);prev=d;last=p;previous_index=i;
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
