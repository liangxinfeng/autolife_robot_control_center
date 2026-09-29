#undef NDEBUG
#define AMCL_STARTUP_TEST
#include <cassert>
#include "../src/amcl_startup_compat.cpp"
class Probe : public InitializedAmclNode
{
public:
  void verify() {
    for(int i=0;i<3;i++)assert(init_pose_[i]==0.0 && std::isfinite(init_cov_[i]) && init_cov_[i]>0.0);
    assert(!initial_pose_is_known_);
    assert(last_published_pose_.pose.pose.orientation.w==1.0);
  }
};
int main(int argc,char **argv) {
  rclcpp::init(argc,argv);
  for(int i=0;i<5;i++) {
    auto node=std::make_shared<Probe>();node->verify();
  }
  rclcpp::shutdown();
}
