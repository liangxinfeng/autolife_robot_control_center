#pragma once
#include <algorithm>
#include <cmath>
namespace control_center_navigation {
struct ArrivalSettlePolicy {
  bool holding=false;
  void reset(){holding=false;}
  bool hold(double distance,double angle){
    if(!std::isfinite(distance)||!std::isfinite(angle)){holding=false;return false;}
    if(holding && (distance>.05 || angle>.05))holding=false;
    if(!holding && distance<=.025 && angle<=.035)holding=true;
    return holding;
  }
  // Continuous speed envelope, applied inside the MPPI optimizer (not to a
  // collision-checked trajectory after optimization). 0 means unlimited to Nav2.
  static double percentage(double distance,double max_speed,double external){
    const double approach=100.*std::min(max_speed,.035+1.2*std::max(0.,distance))/max_speed;
    return std::min(approach,external);
  }
};
}
