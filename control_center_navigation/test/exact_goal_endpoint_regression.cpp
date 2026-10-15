#include "exact_goal_endpoint.hpp"
#include <iostream>
using control_center_navigation::preserve_exact_goal;
int main()
{
  int checked = 0;
  auto check = [&](bool value) {if (!value) {throw std::runtime_error("regression failed");} ++checked;};
  auto goal = geometry_msgs::msg::PoseStamped();
  goal.pose.position.x=.025; goal.pose.position.y=.020; goal.pose.orientation.w=1.;
  nav_msgs::msg::Path p; p.header.frame_id="map";p.poses.emplace_back();
  preserve_exact_goal(p,goal,.05,[](double,double){return true;});
  check(p.poses.back().pose == goal.pose && p.poses.back().header.frame_id=="map");
  const auto count=p.poses.size();
  preserve_exact_goal(p,goal,.05,[](double,double){return true;});check(p.poses.size()==count);
  for (int mode=0;mode<4;++mode) {
    nav_msgs::msg::Path q;if(mode!=0)q.poses.emplace_back();
    auto g=goal;if(mode==1)g.pose.position.x=.15;
    bool rejected=false;
    try {preserve_exact_goal(q,g,.05,[&](double x,double){return mode==2?false:(mode!=3 || x<.01);});}
    catch(const std::runtime_error &){rejected=true;}
    check(rejected);
  }
  // Same-cell goals must not collapse to the robot's start position.
  nav_msgs::msg::Path q;q.poses.emplace_back();
  preserve_exact_goal(q,goal,.05,[](double,double){return true;});
  check(q.poses.back().pose.position.x==.025);
  std::cout<<checked<<" endpoint assertions passed\n";
}
