#include "wall_tail_filter_core.hpp"
#include <filters/filter_base.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <pluginlib/class_list_macros.hpp>

namespace control_center_navigation {
class WeakWallTailFilter : public filters::FilterBase<sensor_msgs::msg::LaserScan> {
public:
  bool configure() override {
    getParam("enabled", enabled_, false, true);
    return true;
  }
  bool update(const sensor_msgs::msg::LaserScan& in, sensor_msgs::msg::LaserScan& out) override {
    out = in;
    if (enabled_) {
      for (int i : weakWallTailIndices(in.ranges,in.intensities,in.angle_min,
          in.angle_increment,in.range_min,in.range_max))
        out.ranges[i]=std::numeric_limits<float>::quiet_NaN();
    }
    return true;
  }
private:
  bool enabled_=true;
};
}
PLUGINLIB_EXPORT_CLASS(control_center_navigation::WeakWallTailFilter,
                      filters::FilterBase<sensor_msgs::msg::LaserScan>)
