#include <algorithm>
#include <cmath>
#include "nav2_amcl/amcl_node.hpp"

// Nav2 1.3.10 calls initParticleFilter() before initOdometry() in on_configure().
// Seed the protected arrays with the SAME upstream unknown-pose defaults before
// their first use. This does not assert a known pose, activate AMCL, or publish TF.
class InitializedAmclNode : public nav2_amcl::AmclNode
{
public:
  explicit InitializedAmclNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : nav2_amcl::AmclNode(options)
  {
    std::fill_n(init_pose_, 3, 0.0);
    init_cov_[0] = 0.5 * 0.5;
    init_cov_[1] = 0.5 * 0.5;
    init_cov_[2] = (M_PI / 12.0) * (M_PI / 12.0);
    last_published_pose_.pose.pose.orientation.w = 1.0;
  }
};

#ifndef AMCL_STARTUP_TEST
int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<InitializedAmclNode>();
  rclcpp::spin(node->get_node_base_interface());
  rclcpp::shutdown();
  return 0;
}
#endif
